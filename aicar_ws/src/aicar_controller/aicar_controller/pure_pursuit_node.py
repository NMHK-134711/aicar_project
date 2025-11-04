import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.get_logger().info('Pure Pursuit Node (Upgraded) started.')

        self.bridge = CvBridge()

        # --- 1. 파라미터 선언 ---
        self.declare_parameter('lookahead_distance', 1.0) # 전방 주시 거리 (Ld) [m]
        self.declare_parameter('vehicle_speed', 0.5)      # 기본 주행 속도 [m/s]
        self.declare_parameter('wheelbase', 0.33)         # 차량 축거 (L) [m]
        self.declare_parameter('ym_per_pix', 0.01) # y축 (종방향) 픽셀 당 미터
        self.declare_parameter('xm_per_pix', 0.005) # x축 (횡방향) 픽셀 당 미터
        self.declare_parameter('stop_duration', 3.0) # 'stop' 표지판 정지 시간 (초)

        # 파라미터 값 읽어오기
        self.Ld_meters = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.speed = self.get_parameter('vehicle_speed').get_parameter_value().double_value
        self.L = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.ym_per_pix = self.get_parameter('ym_per_pix').get_parameter_value().double_value
        self.xm_per_pix = self.get_parameter('xm_per_pix').get_parameter_value().double_value
        self.stop_duration_sec = self.get_parameter('stop_duration').get_parameter_value().double_value

        self.get_logger().info(f'Ld: {self.Ld_meters}m, Speed: {self.speed}m/s, L: {self.L}m')

        # --- 상태 변수 ---
        self.current_sign = None
        self.stop_sign_time = None
        self.is_stopped = False

        # --- 2. ROS 구독 및 발행 설정 ---
        self.subscription = self.create_subscription(
            Image,
            '/image_bev_binary',
            self.bev_callback,
            10)
        self.sign_subscription = self.create_subscription(
            String,
            '/sign_detection',
            self.sign_callback,
            10)
        self.publisher_drive = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)

    def sign_callback(self, msg):
        self.current_sign = msg.data
        self.get_logger().info(f'New sign detected: {self.current_sign}')
        
        if self.current_sign == 'stop' and not self.is_stopped:
            self.stop_sign_time = self.get_clock().now()
            self.is_stopped = True
            self.get_logger().info('STOP sign detected. Stopping vehicle.')

    # -----------------------------------------------------------------
    # 메인 콜백 함수 (BEV 이미지 처리)
    # -----------------------------------------------------------------
    def bev_callback(self, msg):
        try:
            bev_binary = self.bridge.imgmsg_to_cv2(msg, "mono8")
            h, w = bev_binary.shape
        except Exception as e:
            self.get_logger().error(f'Failed to convert BEV image: {e}')
            return

        # --- 1. 주행 속도 결정 (상태 머신) ---
        current_speed = self.speed
        if self.is_stopped:
            if self.current_sign == 'stop':
                time_since_stop = (self.get_clock().now() - self.stop_sign_time).nanoseconds / 1e9
                if time_since_stop < self.stop_duration_sec:
                    current_speed = 0.0
                else:
                    self.get_logger().info('Stop duration complete. Resuming drive.')
                    self.is_stopped = False
                    self.current_sign = None
                    current_speed = self.speed
            else:
                 current_speed = 0.0

        # --- 2. Pure Pursuit 조향각 계산 ---
        
        # 2.1. 목표 y좌표(y_row) 찾기
        # 전방 주시 거리(Ld)를 픽셀 단위로 변환
        lookahead_pixels_y = self.Ld_meters / self.ym_per_pix
        
        # 목표 y좌표 (이미지 하단(h)에서부터 위로)
        y_row = int(h - lookahead_pixels_y)
        if y_row < 0: y_row = 0
            
        # 2.2. 목표 x좌표(x_center_pixels) 찾기
        lane_slice = bev_binary[y_row, :]
        indices = np.nonzero(lane_slice)[0]
        x_center_pixels = w / 2.0
        
        if len(indices) >= 2:
            x_left_pixels = indices[0]
            x_right_pixels = indices[-1]
            x_center_pixels = (x_left_pixels + x_right_pixels) / 2.0
        elif len(indices) == 1:
            x_center_pixels = indices[0]
        else:
            self.get_logger().warn('No lanes detected at lookahead distance.')
            pass

        # 2.3. 'alpha' 각도 계산
        # 차량 좌표계(이미지 하단 중앙) 기준 목표점의 (x, y) 위치 [미터]
        
        # 차량 기준 x방향 오차 (미터)
        # (목표x - 현재x) * (미터/픽셀)
        target_x_meters = (x_center_pixels - (w / 2.0)) * self.xm_per_pix
        
        # 차량 기준 y방향 거리 (미터)
        # (현재y - 목표y) * (미터/픽셀)
        target_y_meters = (h - y_row) * self.ym_per_pix

        # alpha = 차량의 정면(y축)과 목표점 사이의 각도
        alpha = math.atan2(target_x_meters, target_y_meters)

        # 2.4. Pure Pursuit 조향각 공식
        # delta = atan(2 * L * sin(alpha) / Ld)
        # (Ld = 전방 주시 거리 (미터))
        steering_angle = math.atan(2.0 * self.L * math.sin(alpha) / self.Ld_meters)
        
        # --- 3. 제어 명령(Ackermann) 메시지 생성 및 발행 ---
        drive_msg = AckermannDriveStamped()
        drive_msg.header = msg.header
        drive_msg.drive.speed = current_speed
        drive_msg.drive.steering_angle = steering_angle
        
        self.publisher_drive.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuitNode()
    try:
        rclpy.spin(pure_pursuit_node)
    except KeyboardInterrupt:
        pass
    finally:
        pure_pursuit_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()