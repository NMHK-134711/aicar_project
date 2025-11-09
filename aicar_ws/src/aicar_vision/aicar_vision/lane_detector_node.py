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
        self.get_logger().info('Lane Detector Node (Full BEV) started.')

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
                self.get_logger().info(f'Loaded calibration from: {calib_file_path}')
        except Exception as e:
            self.get_logger().fatal(f'Failed to load calibration file: {e}')
            rclpy.shutdown()
            return
            
        # --- 2. 파라미터 로드 (BEV 변환) ---
        self.img_width = 640
        self.img_height = 480

        # 튜닝된 BEV 좌표 
        self.src_points = np.float32([
            (0, 480),   # 좌하단
            (170, 250), # 좌상단
            (470, 250), # 우상단
            (640, 480)  # 우하단
        ])
        # dst_points는 640x480 전체를 꽉 채우도록 설정됨
        self.dst_points = np.float32([
            (int(self.img_width * 0.2), int(self.img_height)),
            (int(self.img_width * 0.2), 0),
            (int(self.img_width * 0.8), 0),
            (int(self.img_width * 0.8), int(self.img_height))
        ])
        self.M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)

        # --- 3. 파라미터 로드 (노란색 HSV 임계값) ---
        self.declare_parameter('hsv_lower_yellow_h', 20)
        self.declare_parameter('hsv_lower_yellow_s', 100)
        self.declare_parameter('hsv_lower_yellow_v', 100)
        self.declare_parameter('hsv_upper_yellow_h', 35)
        self.declare_parameter('hsv_upper_yellow_s', 255)
        self.declare_parameter('hsv_upper_yellow_v', 255)

        self.lower_yellow = np.array([
            self.get_parameter('hsv_lower_yellow_h').get_parameter_value().integer_value,
            self.get_parameter('hsv_lower_yellow_s').get_parameter_value().integer_value,
            self.get_parameter('hsv_lower_yellow_v').get_parameter_value().integer_value
        ])
        self.upper_yellow = np.array([
            self.get_parameter('hsv_upper_yellow_h').get_parameter_value().integer_value,
            self.get_parameter('hsv_upper_yellow_s').get_parameter_value().integer_value,
            self.get_parameter('hsv_upper_yellow_v').get_parameter_value().integer_value
        ])

        # --- 4. ROS 구독 및 발행 설정 ---
        self.subscription = self.create_subscription(
            Image, '/camera_node/image_raw', self.image_callback, 10)
            
        self.publisher_processed = self.create_publisher(
            Image, '/image_processed', 10)
            
        self.publisher_bev = self.create_publisher(
            Image, '/image_bev_binary', 10)

    # -----------------------------------------------------------------
    # 헬퍼 함수
    # -----------------------------------------------------------------
    def undistort_image(self, img):
        return cv2.undistort(img, self.mtx, self.dist, None, self.mtx)

    def threshold_pipeline(self, img):
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv_image, self.lower_yellow, self.upper_yellow)
        return yellow_mask

    def warp_image(self, img):
        warped_img = cv2.warpPerspective(img, self.M, (self.img_width, self.img_height), flags=cv2.INTER_LINEAR)
        return warped_img

    # -----------------------------------------------------------------
    # 메인 콜백 함수 (수정됨)
    # -----------------------------------------------------------------
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # 1. 공통 전처리: 왜곡 보정
        undistorted_img = self.undistort_image(cv_image)

        # 2. 표지판용 이미지 발행
        try:
            processed_msg = self.bridge.cv2_to_imgmsg(undistorted_img, "bgr8")
            processed_msg.header = msg.header
            self.publisher_processed.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish processed image: {e}')

        # 3. HSV 필터 -> BEV 변환 (원본 undistorted_img 사용)
        thresholded_img = self.threshold_pipeline(undistorted_img)
        bev_binary_img = self.warp_image(thresholded_img)

        # 4. BEV 이미지 발행
        try:
            bev_msg = self.bridge.cv2_to_imgmsg(bev_binary_img, "mono8")
            bev_msg.header = msg.header
            self.publisher_bev.publish(bev_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish BEV image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()