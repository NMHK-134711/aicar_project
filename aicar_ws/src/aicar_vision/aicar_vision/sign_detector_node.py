import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class SignDetectorNode(Node):
    def __init__(self):
        super().__init__('sign_detector_node')
        self.get_logger().info('Sign Detector Node (YOLOv8 TFLite) started.')

        self.bridge = CvBridge()
        self.is_model_loaded = False # 모델 로딩 상태 플래그

        # --- 1. 모델 파일 경로 설정 ---
        package_name = 'aicar_vision'
        try:
            pkg_share = get_package_share_directory(package_name)
            model_path = os.path.join(pkg_share, 'models', 'best_float32.tflite')
        except KeyError:
            model_path = '/root/aicar_ws/src/aicar_vision/models/best_float32.tflite'

        self.declare_parameter('model_path', model_path)
        final_model_path = self.get_parameter('model_path').get_parameter_value().string_value

        self.get_logger().info(f'Loading model from: {final_model_path}')

        # --- 2. YOLO 모델 로드 ---
        try:
            self.model = YOLO(final_model_path, task='detect')
            self.is_model_loaded = True # 로딩 완료
            self.get_logger().info('>>> Model Loaded Successfully! System Ready.')
        except Exception as e:
            self.get_logger().fatal(f'Failed to load YOLO model: {e}')
            return

        # --- 3. 상태 변수 및 라벨 매핑 ---
        self.last_published_sign = None
        self.confidence_threshold = 0.60
        self.class_mapping = {
            'stop_sign': 'stop',
            'left_turn_sign': 'left_turn',
            'right_turn_sign': 'right_turn',
            'horn_sign': 'horn',
            '20_sign': 'slow',
            'traffic_light': 'traffic_light_green' 
        }

        # --- 4. ROS 구독/발행 ---
        self.subscription = self.create_subscription(
            Image, '/camera_node/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(String, '/sign_detection', 10)

        # [신규] 시스템 상태 발행용 퍼블리셔
        self.status_publisher_ = self.create_publisher(String, '/system_status', 10)
        # 1초마다 준비 상태를 알림
        self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        """ 모델이 로드되었다면 시스템 준비 신호를 보냄 """
        if self.is_model_loaded:
            msg = String()
            msg.data = "system_ready"
            self.status_publisher_.publish(msg)

    def image_callback(self, msg):
        if not self.is_model_loaded:
            return # 모델 로딩 전이면 무시

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # --- 5. 추론 실행 ---
        results = self.model(cv_image, conf=self.confidence_threshold, imgsz=640, verbose=False)

        detected_raw_label = None
        highest_conf = 0.0

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                if hasattr(self.model, 'names'):
                    raw_label = self.model.names[cls_id]
                else:
                    raw_label = str(cls_id)
                
                conf = float(box.conf[0])
                if conf > highest_conf:
                    highest_conf = conf
                    detected_raw_label = raw_label

        if detected_raw_label:
            if detected_raw_label in self.class_mapping:
                command_sign = self.class_mapping[detected_raw_label]
                if command_sign != self.last_published_sign:
                    msg = String()
                    msg.data = command_sign
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'>>> SIGN DETECTED: {detected_raw_label} -> {command_sign} ({highest_conf:.2f})')
                    self.last_published_sign = command_sign

def main(args=None):
    rclpy.init(args=args)
    node = SignDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()