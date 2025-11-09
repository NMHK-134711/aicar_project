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
        self.get_logger().info('Pure Pursuit Node (Robust Split-Lane Logic) started.')

        self.bridge = CvBridge()

        # --- 1. 파라미터 선언 ---
        self.declare_parameter('lookahead_distance', 0.8)
        self.declare_parameter('vehicle_speed', 0.5)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('ym_per_pix', 0.01)
        self.declare_parameter('xm_per_pix', 0.0005208) # 0.2m / 384px
        self.declare_parameter('camera_offset', 0.1)
        self.declare_parameter('stop_duration', 3.0)

        # 직선에서 한쪽으로 쏠린다면 이 값을 조절해야 합니다.
        # 왼쪽으로 붙으면 -> 값을 키우세요 (예: 220, 250)
        # 오른쪽으로 붙으면 -> 값을 줄이세요 (예: 170, 150)
        self.declare_parameter('half_track_width_pixels', 270) 
        
        # 파라미터 읽기
        self.Ld_meters = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.speed = self.get_parameter('vehicle_speed').get_parameter_value().double_value
        self.L = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.ym_per_pix = self.get_parameter('ym_per_pix').get_parameter_value().double_value
        self.xm_per_pix = self.get_parameter('xm_per_pix').get_parameter_value().double_value
        self.camera_offset = self.get_parameter('camera_offset').get_parameter_value().double_value
        self.stop_duration_sec = self.get_parameter('stop_duration').get_parameter_value().double_value
        self.half_track_width_px = self.get_parameter('half_track_width_pixels').get_parameter_value().integer_value

        self.get_logger().info(f'Ld: {self.Ld_meters}m, Half Track Width: {self.half_track_width_px}px')

        self.current_sign = None
        self.stop_sign_time = None
        self.is_stopped = False

        # --- 2. 구독/발행 ---
        self.subscription = self.create_subscription(
            Image, '/image_bev_binary', self.bev_callback, 10)
        self.sign_subscription = self.create_subscription(
            String, '/sign_detection', self.sign_callback, 10)
        self.publisher_drive = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

    def sign_callback(self, msg):
        self.current_sign = msg.data
        if self.current_sign == 'stop' and not self.is_stopped:
            self.stop_sign_time = self.get_clock().now()
            self.is_stopped = True
            self.get_logger().info('STOP sign detected. Stopping vehicle.')

    def bev_callback(self, msg):
        try:
            bev_binary = self.bridge.imgmsg_to_cv2(msg, "mono8")
            h, w = bev_binary.shape
        except Exception as e:
            self.get_logger().error(f'Failed to convert BEV image: {e}')
            return

        # --- 1. 주행 속도 결정 ---
        current_speed = self.speed
        if self.is_stopped:
            if self.current_sign == 'stop':
                time_since_stop = (self.get_clock().now() - self.stop_sign_time).nanoseconds / 1e9
                if time_since_stop < self.stop_duration_sec:
                    current_speed = 0.0
                else:
                    self.is_stopped = False
                    self.current_sign = None
                    current_speed = self.speed
            else:
                 current_speed = 0.0

        # --- 2. Pure Pursuit 계산 (개선된 로직) ---
        
        lookahead_from_camera = max(0.1, self.Ld_meters - self.camera_offset)
        y_row = int(h - (lookahead_from_camera / self.ym_per_pix))
        if y_row < 0: y_row = 0
            
        midpoint = w // 2
        left_half = bev_binary[y_row, :midpoint]
        right_half = bev_binary[y_row, midpoint:]

        # 각 절반에서 흰색 픽셀의 위치를 찾습니다.
        left_indices = np.nonzero(left_half)[0]
        right_indices = np.nonzero(right_half)[0]

        # 노이즈 필터링: 최소 10픽셀 이상 뭉쳐있어야 차선으로 인정
        MIN_PIXELS = 10
        has_left = len(left_indices) > MIN_PIXELS
        has_right = len(right_indices) > MIN_PIXELS

        robot_center_px = w / 2.0
        x_center_pixels = robot_center_px # 기본값

        if has_left and has_right:
            # [상황 1: 정상] 양쪽 모두 보임
            l_center = np.mean(left_indices)
            r_center = np.mean(right_indices) + midpoint # 오른쪽은 midpoint를 더해줘야 함
            x_center_pixels = (l_center + r_center) / 2.0

        elif has_left and not has_right:
            # [상황 2: 우회전 중] 왼쪽만 보임 -> 오른쪽으로 추측
            l_center = np.mean(left_indices)
            x_center_pixels = l_center + self.half_track_width_px
            self.get_logger().info(f'Tracking LEFT lane only. Target: {x_center_pixels:.0f}')

        elif not has_left and has_right:
            # [상황 3: 좌회전 중] 오른쪽만 보임 -> 왼쪽으로 추측
            r_center = np.mean(right_indices) + midpoint
            x_center_pixels = r_center - self.half_track_width_px
            self.get_logger().info(f'Tracking RIGHT lane only. Target: {x_center_pixels:.0f}')

        else:
            # [상황 0: 차선 없음]
            pass
        # ===============================================================

        # 2.3. 좌표 변환 및 조향각 계산
        target_x_meters = (x_center_pixels - robot_center_px) * self.xm_per_pix
        target_y_from_robot = (h - y_row) * self.ym_per_pix + self.camera_offset

        alpha = math.atan2(target_x_meters, target_y_from_robot)
        actual_lookahead = math.sqrt(target_x_meters**2 + target_y_from_robot**2)
        steering_angle = math.atan(2.0 * self.L * math.sin(alpha) / actual_lookahead)

        # --- 3. 명령 발행 ---
        drive_msg = AckermannDriveStamped()
        drive_msg.header = msg.header
        drive_msg.drive.speed = current_speed
        drive_msg.drive.steering_angle = steering_angle
        self.publisher_drive.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()