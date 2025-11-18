import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import pickle
import os
from ament_index_python.packages import get_package_share_directory

class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_node')
        self.get_logger().info('Lane Detector Node (Dual Output: Yellow Lane + Red Finish) started.')

        self.bridge = CvBridge()
        
        # --- 1. 파라미터 로드 (카메라 보정) ---
        package_share_directory = get_package_share_directory('aicar_vision')
        default_calib_path = os.path.join(
            package_share_directory, 'calibration_data', 'calibration.p'
        )
        self.declare_parameter('calibration_file', default_calib_path)
        calib_file_path = self.get_parameter('calibration_file').get_parameter_value().string_value

        try:
            with open(calib_file_path, 'rb') as f:
                calib_data = pickle.load(f)
                self.mtx = calib_data['mtx']
                self.dist = calib_data['dist']
        except Exception as e:
            self.get_logger().fatal(f'Failed to load calibration file: {e}')
            rclpy.shutdown()
            return
            
        # --- 2. BEV 변환 파라미터 ---
        self.img_width = 640
        self.img_height = 480
        self.src_points = np.float32([
            (0, 480), 
            (170, 350), 
            (470, 350), 
            (640, 480)])
        self.dst_points = np.float32([
            (int(self.img_width * 0.2), int(self.img_height)),
            (int(self.img_width * 0.2), 0),
            (int(self.img_width * 0.8), 0),
            (int(self.img_width * 0.8), int(self.img_height))
        ])
        self.M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)

        # --- 3. 색상 임계값 정의 ---

        # (A) 주행 차선용 (노란색)
        self.lower_yellow = np.array([20, 40, 100])
        self.upper_yellow = np.array([35, 255, 255])

        # (B) 종료선용 (빨간색) - HSV에서 Red는 0과 180 양쪽에 걸쳐 있음
        self.lower_red1 = np.array([0, 40, 100])
        self.upper_red1 = np.array([15, 255, 255])
        self.lower_red2 = np.array([165, 40, 100])
        self.upper_red2 = np.array([180, 255, 255])

        # 모폴로지 커널
        self.dilate_kernel = np.ones((5, 5), np.uint8)

        # --- 4. ROS 설정 ---
        self.subscription = self.create_subscription(Image, '/camera_node/image_raw', self.image_callback, 10)
        
        # 토픽 1: 주행용 차선 (노란색) -> PID 제어에 사용
        self.publisher_bev_lane = self.create_publisher(Image, '/image_bev_binary', 10)
        # 토픽 2: 종료 확인용 라인 (빨간색) -> 종료 로직에만 사용
        self.publisher_bev_red = self.create_publisher(Image, '/image_red_bev', 10)
        
        self.publisher_processed = self.create_publisher(Image, '/image_processed', 10)

    def undistort_image(self, img):
        return cv2.undistort(img, self.mtx, self.dist, None, self.mtx)

    def warp_image(self, img):
        return cv2.warpPerspective(img, self.M, (self.img_width, self.img_height), flags=cv2.INTER_LINEAR)

    def process_color(self, hsv_img, lower, upper, is_red=False):
        """ 색상 마스크 생성 및 전처리 함수 """
        if is_red:
            # 빨간색은 두 범위를 합쳐야 함
            mask1 = cv2.inRange(hsv_img, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv_img, self.lower_red2, self.upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            # 일반 색상 (노란색)
            mask = cv2.inRange(hsv_img, lower, upper)
            
        # 모폴로지 (선 두껍게)
        dilated_mask = cv2.dilate(mask, self.dilate_kernel, iterations=2)
        return dilated_mask

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception: return

        # 1. 전처리 (왜곡 보정 + 블러)
        undistorted = self.undistort_image(cv_image)
        blurred = cv2.GaussianBlur(undistorted, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 2. [주행용] 노란색 차선 처리
        mask_lane = self.process_color(hsv, self.lower_yellow, self.upper_yellow, is_red=False)
        bev_lane = self.warp_image(mask_lane)
        
        # 3. [종료용] 빨간색 라인 처리
        mask_red = self.process_color(hsv, None, None, is_red=True)
        bev_red = self.warp_image(mask_red)

        # 4. 발행
        # (A) 주행용 BEV
        msg_lane = self.bridge.cv2_to_imgmsg(bev_lane, "mono8")
        msg_lane.header = msg.header
        self.publisher_bev_lane.publish(msg_lane)

        # (B) 종료용 Red BEV
        msg_red = self.bridge.cv2_to_imgmsg(bev_red, "mono8")
        msg_red.header = msg.header
        self.publisher_bev_red.publish(msg_red)

        # (C) 디버깅용 (원본)
        proc_msg = self.bridge.cv2_to_imgmsg(undistorted, "bgr8")
        proc_msg.header = msg.header
        self.publisher_processed.publish(proc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
