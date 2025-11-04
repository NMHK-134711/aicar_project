import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import pickle
import os
from ament_index_python.packages import get_package_share_directory

# -----------------------------------------------------------------------------
# 'Advance_LaneFinding' 레포지토리의 로직을 클래스로 통합
# -----------------------------------------------------------------------------

# 차선 추적을 위한 헬퍼 클래스
class Line:
    def __init__(self):
        # 최근 n개의 프레임을 저장
        self.n_frames = 10
        # 최근 n개의 다항식 계수 (a, b, c)
        self.recent_fits = []
        # 최근 n 프레임 동안의 평균 계수
        self.avg_fit = None
        # 현재 프레임의 다항식 계수
        self.current_fit = None
        # 마지막으로 성공한 프레임과의 차이
        self.diffs = np.array([0, 0, 0], dtype='float')
        # 차선이 감지되었는지 여부
        self.detected = False

    def update(self, fit):
        if fit is None:
            # 감지 실패 시, 최근 n개 중 1개 제거
            if len(self.recent_fits) > 0:
                self.recent_fits.pop(0)
            self.detected = False
        else:
            # 감지 성공
            if self.avg_fit is not None:
                # 마지막 평균과의 차이 계산
                self.diffs = self.avg_fit - fit
            
            # 현재 프레임 계수 저장
            self.current_fit = fit
            self.recent_fits.append(fit)
            
            # n개가 넘으면 가장 오래된 것 제거
            if len(self.recent_fits) > self.n_frames:
                self.recent_fits.pop(0)
            
            # 최근 n 프레임의 평균 계산
            self.avg_fit = np.mean(self.recent_fits, axis=0)
            self.detected = True

# --- 메인 ROS2 노드 ---

class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_node')
        self.get_logger().info('Advanced Lane Detector Node started.')

        self.bridge = CvBridge()
        
        # --- 1. 파라미터 로드 ---
        
        # 카메라 보정 파일 경로 (이 파일이 반드시 있어야 함)
        # 예: aicar_vision 패키지 폴더 내에 저장
        package_share_directory = get_package_share_directory('aicar_vision')
        
        # 'calibration.p' 파일의 기본 경로를 설정합니다.
        default_calib_path = os.path.join(
            package_share_directory,
            'calibration_data',
            'calibration.p'
        )

        # 카메라 보정 파일 경로 (기본값을 방금 찾은 경로로 설정)
        self.declare_parameter('calibration_file', default_calib_path)
        calib_file_path = self.get_parameter('calibration_file').get_parameter_value().string_value

        try:
            with open(calib_file_path, 'rb') as f:
                calib_data = pickle.load(f)
                self.mtx = calib_data['mtx']
                self.dist = calib_data['dist']
                self.get_logger().info(f'Camera calibration file loaded from: {calib_file_path}')
        except Exception as e:
            self.get_logger().fatal(f'Failed to load calibration file: {e}')
            self.get_logger().fatal(f'File not found at: {calib_file_path}')
            self.get_logger().fatal('Please ensure calibration.p is in aicar_vision/calibration_data/ and build again.')
            rclpy.shutdown()
            return
            
        # 이미지 크기
        self.img_width = 640
        self.img_height = 480

        # BEV 변환 파라미터 (이전과 동일, 튜닝 필요)
        self.src_points = np.float32([
            (int(self.img_width * 0.1), int(self.img_height * 0.9)),
            (int(self.img_width * 0.4), int(self.img_height * 0.6)),
            (int(self.img_width * 0.6), int(self.img_height * 0.6)),
            (int(self.img_width * 0.9), int(self.img_height * 0.9))
        ])
        self.dst_points = np.float32([
            (int(self.img_width * 0.2), int(self.img_height)),
            (int(self.img_width * 0.2), 0),
            (int(self.img_width * 0.8), 0),
            (int(self.img_width * 0.8), int(self.img_height))
        ])
        
        # BEV 변환 행렬 (M) 및 역변환 행렬 (Minv)
        self.M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
        self.Minv = cv2.getPerspectiveTransform(self.dst_points, self.src_points)

        # 차선 추적 객체 초기화
        self.left_line = Line()
        self.right_line = Line()
        
        # --- 2. ROS 구독 및 발행 설정 ---
        
        # 구독: C++ 카메라 노드 
        self.subscription = self.create_subscription(
            Image,
            '/camera_node/image_raw',
            self.image_callback,
            10)

        # 발행: 최종 결과 이미지 (차선이 그려진)
        self.publisher_processed = self.create_publisher(
            Image,
            '/image_processed',
            10)
            
        # 발행: BEV 2진 이미지 (디버깅용)
        self.publisher_bev = self.create_publisher(
            Image,
            '/image_bev_binary',
            10)

    # -----------------------------------------------------------------
    # 파이프라인 헬퍼 함수 (Advance_LaneFinding 로직)
    # -----------------------------------------------------------------

    def undistort_image(self, img):
        """ 1. 렌즈 왜곡 보정 """
        return cv2.undistort(img, self.mtx, self.dist, None, self.mtx)

    def threshold_pipeline(self, img):
        """ 2. 복합 임계값 (Sobel + HLS) """
        
        # HLS 색 공간으로 변환
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        s_channel = hls[:, :, 2] # S(채도) 채널
        
        # Sobel X (수직선 감지)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
        abs_sobelx = np.absolute(sobelx)
        scaled_sobel = np.uint8(255 * abs_sobelx / np.max(abs_sobelx))
        
        # Sobel 임계값 적용
        sobel_binary = np.zeros_like(scaled_sobel)
        sobel_binary[(scaled_sobel >= 20) & (scaled_sobel <= 100)] = 1
        
        # S 채널 임계값 적용 (노란색/흰색 차선)
        s_binary = np.zeros_like(s_channel)
        s_binary[(s_channel >= 170) & (s_channel <= 255)] = 1
        
        # Sobel과 S 채널 조합
        combined_binary = np.zeros_like(sobel_binary)
        combined_binary[(s_binary == 1) | (sobel_binary == 1)] = 255
        
        return combined_binary

    def warp_image(self, img):
        """ 3. BEV 변환 """
        return cv2.warpPerspective(img, self.M, 
                                   (self.img_width, self.img_height), 
                                   flags=cv2.INTER_LINEAR)

    def find_lanes_sliding_window(self, binary_warped):
        """ 4. 슬라이딩 윈도우로 차선 픽셀 탐지 (추적 실패 시) """
        
        # 이미지 하단 절반의 히스토그램
        histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0)
        midpoint = int(histogram.shape[0] / 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 9
        window_height = int(binary_warped.shape[0] / nwindows)
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = leftx_base
        rightx_current = rightx_base
        margin = 100
        minpix = 50
        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        left_fit = None
        right_fit = None
        
        if len(leftx) > 0 and len(lefty) > 0:
            left_fit = np.polyfit(lefty, leftx, 2)
        if len(rightx) > 0 and len(righty) > 0:
            right_fit = np.polyfit(righty, rightx, 2)
            
        return left_fit, right_fit

    def find_lanes_from_prior(self, binary_warped, left_fit, right_fit):
        """ 5. 이전 프레임 기반 차선 탐색 (추적 성공 시) """
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 100

        left_lane_inds = ((nonzerox > (left_fit[0] * (nonzeroy**2) + left_fit[1] * nonzeroy + left_fit[2] - margin)) & 
                          (nonzerox < (left_fit[0] * (nonzeroy**2) + left_fit[1] * nonzeroy + left_fit[2] + margin)))
        right_lane_inds = ((nonzerox > (right_fit[0] * (nonzeroy**2) + right_fit[1] * nonzeroy + right_fit[2] - margin)) & 
                           (nonzerox < (right_fit[0] * (nonzeroy**2) + right_fit[1] * nonzeroy + right_fit[2] + margin)))

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        new_left_fit = None
        new_right_fit = None
        
        if len(leftx) > 0 and len(lefty) > 0:
            new_left_fit = np.polyfit(lefty, leftx, 2)
        if len(rightx) > 0 and len(righty) > 0:
            new_right_fit = np.polyfit(righty, rightx, 2)
            
        return new_left_fit, new_right_fit

    def draw_lane_on_road(self, original_img, binary_warped, left_fit, right_fit):
        """ 6. 최종 이미지에 차선 그리기 """
        
        ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
        
        try:
            left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
            right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
        except TypeError:
            # 다항식 피팅 실패 시
            return original_img

        # BEV 이미지에 차선 영역 그리기
        warp_zero = np.zeros_like(binary_warped).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

        # BEV를 원본 이미지 시점으로 역변환
        new_warp = cv2.warpPerspective(color_warp, self.Minv, 
                                       (original_img.shape[1], original_img.shape[0]))
        
        # 원본 이미지와 차선 영역 합성
        result = cv2.addWeighted(original_img, 1, new_warp, 0.3, 0)
        return result

    # -----------------------------------------------------------------
    # 메인 콜백 함수
    # -----------------------------------------------------------------
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # ★ 파이프라인 실행 ★
        
        # 0. 180도 회전
        rotated_image = cv2.rotate(cv_image, cv2.ROTATE_180)
        
        # 1. 렌즈 왜곡 보정
        undistorted_img = self.undistort_image(rotated_image)
        
        # 2. 복합 임계값 적용
        thresholded_img = self.threshold_pipeline(undistorted_img)
        
        # 3. BEV 변환
        bev_binary = self.warp_image(thresholded_img)
        
        # 4. 차선 탐지 (추적 또는 슬라이딩 윈도우)
        if self.left_line.detected and self.right_line.detected:
            # 추적 성공 시 -> 이전 프레임 기반으로 탐색 (빠름)
            left_fit, right_fit = self.find_lanes_from_prior(bev_binary, 
                                                             self.left_line.avg_fit, 
                                                             self.right_line.avg_fit)
        else:
            # 추적 실패 시 -> 슬라이딩 윈도우 (느림)
            left_fit, right_fit = self.find_lanes_sliding_window(bev_binary)

        # 5. 차선 정보 업데이트 (스무딩)
        self.left_line.update(left_fit)
        self.right_line.update(right_fit)

        # 6. 최종 이미지에 그리기
        # (스무딩된 avg_fit을 사용해야 차선이 떨리지 않습니다)
        if self.left_line.avg_fit is not None and self.right_line.avg_fit is not None:
            result_img = self.draw_lane_on_road(undistorted_img, bev_binary,
                                                self.left_line.avg_fit, 
                                                self.right_line.avg_fit)
        else:
            result_img = undistorted_img # 피팅 실패 시 원본(왜곡보정)만 출력

        # --- 3. ROS 토픽 발행 ---
        try:
            # 최종 결과 이미지 발행
            processed_msg = self.bridge.cv2_to_imgmsg(result_img, "bgr8")
            processed_msg.header = msg.header
            self.publisher_processed.publish(processed_msg)
            
            # BEV (디버깅용) 이미지 발행
            bev_msg = self.bridge.cv2_to_imgmsg(bev_binary, "mono8")
            bev_msg.header = msg.header
            self.publisher_bev.publish(bev_msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish images: {e}')

def main(args=None):
    rclpy.init(args=args)
    lane_detector_node = LaneDetectorNode()
    try:
        rclpy.spin(lane_detector_node)
    except KeyboardInterrupt:
        pass
    finally:
        lane_detector_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()