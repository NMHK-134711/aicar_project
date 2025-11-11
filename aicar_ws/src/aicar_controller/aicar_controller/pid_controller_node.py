import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from collections import deque
import lgpio # 부저 제어를 위해 추가

# --- 상태 상수 정의 ---
STATE_NORMAL = 'NORMAL'
STATE_STOP_WAIT = 'STOP_WAIT'
STATE_PRE_TURN_STRAIGHT = 'PRE_TURN_STRAIGHT'
STATE_TURNING = 'TURNING'
STATE_POST_TURN_STRAIGHT = 'POST_TURN_STRAIGHT'

BUZZER_PIN = 12
BUZZER_FREQ = 2000  # [수정] 부저 주파수 (Hz), 2kHz

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        self.get_logger().info('PID Controller Node (Full State Machine) started.')

        self.bridge = CvBridge()

        # --- 1. 파라미터 선언 ---
        self.declare_parameter('vehicle_speed', 0.2)
        self.declare_parameter('ym_per_pix', 0.01)
        self.declare_parameter('xm_per_pix', 0.005)
        self.declare_parameter('half_track_width_pixels', 250)
        self.declare_parameter('kp', 0.4)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('lookahead_row_offset', 20)
        self.declare_parameter('cmd_delay', 0.7)

        self.current_sign = None
        self.last_buzzer_time = 0.0
        self.is_slow_mode = False # ★ [추가] slow 모드 토글 플래그

        # 파라미터 읽기
        self.base_speed = self.get_parameter('vehicle_speed').get_parameter_value().double_value
        self.xm_per_pix = self.get_parameter('xm_per_pix').get_parameter_value().double_value
        self.half_track_width_px = self.get_parameter('half_track_width_pixels').get_parameter_value().integer_value
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.row_offset = self.get_parameter('lookahead_row_offset').get_parameter_value().integer_value
        self.cmd_delay = self.get_parameter('cmd_delay').get_parameter_value().double_value

        # PID 상태 변수
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.prev_time = self.get_clock().now().nanoseconds / 1e9

        # --- [핵심] 주행 상태 머신 변수 ---
        self.drive_state = STATE_NORMAL
        self.state_start_time = 0.0
        self.turn_direction = 0.0 # -1.0(우회전), 1.0(좌회전)
        self.next_state = STATE_NORMAL # 복합 시퀀스용
        self.red_light_sequence = False # 적색 신호등 시퀀스 플래그

        # 표지판 상태
        self.current_sign = None
        self.last_buzzer_time = 0.0

        # 하드웨어(부저) 초기화
        self.h = lgpio.gpiochip_open(4)
        lgpio.gpio_claim_output(self.h, BUZZER_PIN) # PWM을 위해서도 output으로 claim합니다.

        # 명령 버퍼
        self.cmd_buffer = deque()

        # 구독/발행
        self.subscription = self.create_subscription(
            Image, '/image_bev_binary', self.bev_callback, 10)
        self.sign_subscription = self.create_subscription(
            String, '/sign_detection', self.sign_callback, 10)
        self.publisher_drive = self.create_publisher(
            Twist, '/cmd_vel', 10)

        # 타이머
        self.create_timer(0.01, self.timer_callback)

    def sign_callback(self, msg):
        new_sign = msg.data
        now = self.get_clock().now().nanoseconds / 1e9

        # --- 1. 독립적인 이벤트 처리 (부저) ---
        if new_sign == 'horn':
            # 2초 쿨타임으로 중복 울림 방지
            if now - self.last_buzzer_time > 2.0:
                self.beep_buzzer()
                self.last_buzzer_time = now
            return # 부저 울림 후 즉시 종료 (다른 로직에 영향 X)

        # --- 2. 독립적인 이벤트 처리 (Slow 토글) ---
        if new_sign == 'slow':
            # 'slow' 사인을 받을 때마다 토글
            self.is_slow_mode = not self.is_slow_mode
            self.get_logger().info(f'SLOW mode toggled. Now: {self.is_slow_mode}')
            self.current_sign = new_sign # 마지막 사인 기록 (선택 사항)
            return # Slow 토글 후 즉시 종료 (다른 로직에 영향 X)

        # --- 3. 상태 머신 변경 로직 (NORMAL 상태일 때만 작동) ---
        
        # 중복된 사인으로 인한 재시작 방지
        if new_sign == self.current_sign:
            return 
        
        # 현재 주행 상태가 'NORMAL'이 아닐 경우, 새로운 상태 변경 명령 무시
        if self.drive_state != STATE_NORMAL:
            self.get_logger().warn(f'Ignoring sign "{new_sign}" while in state "{self.drive_state}"')
            return

        # NORMAL 상태에서 새로운 상태 변경 사인이 들어옴
        self.current_sign = new_sign
        self.get_logger().info(f'New state-changing sign detected: {self.current_sign}')
        
        if self.current_sign == 'stop':
            self.set_state(STATE_STOP_WAIT)
        
        elif self.current_sign == 'left_turn':
            self.turn_direction = 1.0 # 좌회전 (Twist.angular.z 양수)
            self.set_state(STATE_PRE_TURN_STRAIGHT)
        
        elif self.current_sign == 'right_turn':
            self.turn_direction = -1.0 # 우회전 (Twist.angular.z 음수)
            self.set_state(STATE_PRE_TURN_STRAIGHT)

        elif self.current_sign == 'traffic_light_green':
            self.turn_direction = -1.0
            self.set_state(STATE_PRE_TURN_STRAIGHT)

        elif self.current_sign == 'traffic_light_red':
            self.red_light_sequence = True
            self.set_state(STATE_STOP_WAIT)

    def set_state(self, new_state):
        self.drive_state = new_state
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f'State changed to: {self.drive_state}')

    def beep_buzzer(self):
        # [수정] gpio_write 대신 tx_pwm 사용
        # 50% 듀티 사이클로 PWM 신호를 0.1초간 발생
        self.get_logger().info('BEEP!')
        lgpio.tx_pwm(self.h, BUZZER_PIN, BUZZER_FREQ, 50) 
        self.create_timer(0.5, self.buzzer_off_callback) # 0.1초 후에 끄는 타이머

    def buzzer_off_callback(self):
        # [수정] PWM 종료 (듀티 사이클 0으로 설정)
        lgpio.tx_pwm(self.h, BUZZER_PIN, BUZZER_FREQ, 0)
        pass 

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        while self.cmd_buffer:
            cmd_time, twist_msg = self.cmd_buffer[0]
            if now - cmd_time >= self.cmd_delay:
                self.publisher_drive.publish(twist_msg)
                self.cmd_buffer.popleft()
            else:
                break

    def bev_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        state_duration = now - self.state_start_time

        # 기본 명령 (정지)
        linear_vel = 0.0
        angular_vel = 0.0
        
        # --- 상태 머신 로직 ---
        if self.drive_state == STATE_NORMAL:
            # 자율 주행 (PID 계산)
            linear_vel = self.base_speed
            
            if self.is_slow_mode:
                linear_vel *= 0.6 # 속도 줄임

            # PID 계산 로직
            steering_angle = self.calculate_pid(msg)
            angular_vel = steering_angle

        elif self.drive_state == STATE_STOP_WAIT:
            linear_vel = 0.0
            angular_vel = 0.0
            wait_time = 4.0 if self.red_light_sequence else 5.0
            
            if state_duration >= wait_time:
                if self.red_light_sequence:
                    self.red_light_sequence = False
                    self.turn_direction = -1.0 # 우회전 예약
                    self.set_state(STATE_PRE_TURN_STRAIGHT)
                else:
                    self.current_sign = None # 표지판 효력 상실
                    self.set_state(STATE_NORMAL)

        elif self.drive_state == STATE_PRE_TURN_STRAIGHT:
            linear_vel = self.base_speed # 직진
            angular_vel = 0.0
            if state_duration >= 1.0: # 1초 직진 완료
                self.set_state(STATE_TURNING)

        elif self.drive_state == STATE_TURNING:
            # 90도 테스트 기반: 속도 40%로 1.9초 회전
            # PID 노드에서는 제자리 회전(linear=0)을 시도
            # 각속도 값은 모터 노드 설정에 따라 튜닝 필요. 우선 1.5 rad/s 정도로 시도
            linear_vel = 0.0  
            angular_vel = self.turn_direction * 2.0 # [튜닝 필요] 회전 속도
            
            if state_duration >= 1.4: # 1.4초 회전 완료
                self.set_state(STATE_POST_TURN_STRAIGHT)

        elif self.drive_state == STATE_POST_TURN_STRAIGHT:
            linear_vel = self.base_speed # 직진
            angular_vel = 0.0
            if state_duration >= 1.0: # 1초 직진 완료
                self.current_sign = None
                self.set_state(STATE_NORMAL)

        # 명령 버퍼에 추가
        twist = Twist()
        twist.linear.x = float(linear_vel)
        twist.angular.z = float(angular_vel)
        self.cmd_buffer.append((now, twist))

    def calculate_pid(self, msg):
        try:
            bev_binary = self.bridge.imgmsg_to_cv2(msg, "mono8")
            h, w = bev_binary.shape
        except: 
            return self.prev_error / self.xm_per_pix # 이미지 처리 실패 시 이전 값 사용

        y_row = h - self.row_offset
        if y_row < 0: y_row = 0
        lane_slice = bev_binary[int(y_row), :]
        indices = np.nonzero(lane_slice)[0]
        
        center_offset = 30.0
        robot_center_px = (w / 2.0) + center_offset
        target_px = robot_center_px
        
        if len(indices) > 0:
            min_x, max_x = indices[0], indices[-1]
            if min_x < robot_center_px and max_x > robot_center_px: 
                target_px = (min_x + max_x) / 2.0
            elif min_x < robot_center_px: 
                target_px = max_x + self.half_track_width_px
            elif min_x > robot_center_px: 
                target_px = min_x - self.half_track_width_px
        else:
            # --- 차선 미감지 시, 이전 에러를 유지하도록 target_px를 역산 ---
            target_px = robot_center_px - (self.prev_error / self.xm_per_pix)

        
        error_meters = (robot_center_px - target_px) * self.xm_per_pix
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.prev_time
        if dt <= 0: dt = 0.03
        p = self.kp * error_meters
        self.integral_error += error_meters * dt
        i = self.ki * self.integral_error
        
        # D-term은 차선을 놓쳤을 때 0이 되어야 함 (스파이크 방지)
        d_error = 0.0
        if len(indices) > 0: # 차선이 감지될 때만 D항 계산
             d_error = (error_meters - self.prev_error)
             
        d = self.kd * (d_error) / dt
        
        self.prev_error = error_meters # 에러는 매번 업데이트
        self.prev_time = now
        return p + i + d

    def destroy_node(self):
        # 노드 종료 시 PWM 정지
        lgpio.tx_pwm(self.h, BUZZER_PIN, BUZZER_FREQ, 0)
        if self.h: lgpio.gpiochip_close(self.h)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()