import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        self.get_logger().info('PID Controller Node started.')

        self.bridge = CvBridge()

        # --- 1. 파라미터 선언 ---
        self.declare_parameter('vehicle_speed', 0.5)
        self.declare_parameter('ym_per_pix', 0.01)
        self.declare_parameter('xm_per_pix', 0.0005208) # 0.2m / 384px
        self.declare_parameter('half_track_width_pixels', 250) # 튜닝 필요
        self.declare_parameter('stop_duration', 3.0)

        # ★★★ PID 게인 파라미터 (핵심 튜닝 대상) ★★★
        self.declare_parameter('kp', 0.5)   # P(비례): 클수록 목표에 빠르게 반응 (너무 크면 진동)
        self.declare_parameter('ki', 0.0)   # I(적분): 누적 오차 보정 (보통 라인트레이싱에선 0)
        self.declare_parameter('kd', 0.1)   # D(미분): 급격한 변화 억제 (진동 감소)
        # ★★★★★★★★★★★★★★★★★★★★★★★★★★★

        # 목표점을 찾을 y좌표 (이미지 하단에서의 거리, 0~479)
        # 너무 멀리 보면 코너 안쪽을 파고들고, 너무 가까이 보면 진동합니다.
        self.declare_parameter('lookahead_row_offset', 100) 

        # 파라미터 읽기
        self.speed = self.get_parameter('vehicle_speed').get_parameter_value().double_value
        self.xm_per_pix = self.get_parameter('xm_per_pix').get_parameter_value().double_value
        self.half_track_width_px = self.get_parameter('half_track_width_pixels').get_parameter_value().integer_value
        self.stop_duration_sec = self.get_parameter('stop_duration').get_parameter_value().double_value
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.row_offset = self.get_parameter('lookahead_row_offset').get_parameter_value().integer_value

        # PID 상태 변수
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.prev_time = self.get_clock().now().nanoseconds / 1e9

        # 상태 변수
        self.current_sign = None
        self.stop_sign_time = None
        self.is_stopped = False

        # 구독/발행
        self.subscription = self.create_subscription(
            Image, '/image_bev_binary', self.bev_callback, 10)
        self.sign_subscription = self.create_subscription(
            String, '/sign_detection', self.sign_callback, 10)
        self.publisher_drive = self.create_publisher(
            Twist, '/cmd_vel', 10)

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

        # --- 1. 속도 결정 (정지 신호 로직) ---
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

        # --- 2. 목표점(오차) 계산 ---
        # 이미지 하단에서 row_offset만큼 올라간 지점을 봅니다.
        y_row = h - self.row_offset
        if y_row < 0: y_row = 0
        if y_row >= h: y_row = h - 1
            
        lane_slice = bev_binary[y_row, :]
        indices = np.nonzero(lane_slice)[0]
        
        robot_center_px = w / 2.0
        target_px = robot_center_px # 기본값 (에러 0)

        if len(indices) > 0:
            min_x = indices[0]
            max_x = indices[-1]
            if min_x < robot_center_px and max_x > robot_center_px:
                target_px = (min_x + max_x) / 2.0
            elif min_x < robot_center_px and max_x < robot_center_px:
                target_px = max_x + self.half_track_width_px # 왼쪽만 보임
            elif min_x > robot_center_px and max_x > robot_center_px:
                target_px = min_x - self.half_track_width_px # 오른쪽만 보임

        # 에러 계산 (미터 단위로 변환)
        # 화면 중앙(robot_center_px)에 있어야 할 차선이 target_px에 있음
        # error가 양수(+)면 차선이 왼쪽에 있음 -> 왼쪽으로 핸들 꺾어야 함 (PID 출력 부호에 따라 다름)
        error_meters = (target_px - robot_center_px) * self.xm_per_pix

        # --- 3. PID 제어 ---
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.prev_time
        if dt <= 0: dt = 0.03 # 첫 프레임 등 예외 처리 (약 30fps 기준)

        # P term
        p_term = self.kp * error_meters
        
        # I term (적분 누적 제한 필요할 수 있음)
        self.integral_error += error_meters * dt
        i_term = self.ki * self.integral_error
        
        # D term
        d_term = self.kd * (error_meters - self.prev_error) / dt

        # PID 출력 (조향각)
        steering_angle = p_term + i_term + d_term

        # 상태 업데이트
        self.prev_error = error_meters
        self.prev_time = current_time

        # --- 4. 명령 발행 ---
        twist_msg = Twist()
        twist_msg.linear.x = current_speed
        twist_msg.angular.z = steering_angle
        self.publisher_drive.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()