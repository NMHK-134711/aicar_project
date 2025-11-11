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
        self.get_logger().info('Lane Detector Node (Enhanced for Thin Red Track) started.')

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

        # --- 3. 색상 임계값 (현장 상황에 맞게 주석 교체) ---

        # === 빨간색 트랙용 ===
        # 채도(S)를 40까지 낮추어 희미한 색도 잡도록 설정
        #self.lower_red1 = np.array([0, 40, 100])
        #self.upper_red1 = np.array([15, 255, 255])
        #self.lower_red2 = np.array([165, 40, 100])
        #self.upper_red2 = np.array([180, 255, 255])
        # =================================

        # === 노란색 트랙용 (===
        self.lower_yellow = np.array([20, 40, 100])
        self.upper_yellow = np.array([35, 255, 255])
        # ===============================================


        # --- 4. 모폴로지 연산용 커널 ---
        # 5x5 크기로 팽창시켜 선을 두껍게 만듦
        self.dilate_kernel = np.ones((5, 5), np.uint8)

        # --- 5. ROS 설정 ---
        self.subscription = self.create_subscription(Image, '/camera_node/image_raw', self.image_callback, 10)
        self.publisher_processed = self.create_publisher(Image, '/image_processed', 10)
        self.publisher_bev = self.create_publisher(Image, '/image_bev_binary', 10)

    def undistort_image(self, img):
        return cv2.undistort(img, self.mtx, self.dist, None, self.mtx)

    def threshold_pipeline(self, img):
        # 1. 가우시안 블러로 노이즈 제거 (선이 부드러워짐)
        blurred = cv2.GaussianBlur(img, (5, 5), 0)
        
        # 2. HSV 변환 및 마스크 생성
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # === 빨간색 마스크 ===
        #mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        #mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        #mask = cv2.bitwise_or(mask1, mask2)
        # ==========================
        
        # === 노란색 마스크 ===
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        # ===============================================

        
        # 3. [중요] 모폴로지 팽창 (Dilation) - 얇은 선을 두껍게
        # iterations=2로 설정하여 꽤 두껍게 만듭니다. 노이즈가 심하면 1로 줄이세요.
        dilated_mask = cv2.dilate(mask, self.dilate_kernel, iterations=2)
        
        return dilated_mask

    def warp_image(self, img):
        return cv2.warpPerspective(img, self.M, (self.img_width, self.img_height), flags=cv2.INTER_LINEAR)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception: return

        undistorted = self.undistort_image(cv_image)
        
        # 디버깅용 원본 발행
        proc_msg = self.bridge.cv2_to_imgmsg(undistorted, "bgr8")
        proc_msg.header = msg.header
        self.publisher_processed.publish(proc_msg)

        # 개선된 파이프라인 적용
        mask = self.threshold_pipeline(undistorted)
        bev = self.warp_image(mask)

        bev_msg = self.bridge.cv2_to_imgmsg(bev, "mono8")
        bev_msg.header = msg.header
        self.publisher_bev.publish(bev_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()