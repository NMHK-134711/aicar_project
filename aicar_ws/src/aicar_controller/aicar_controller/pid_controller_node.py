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
import lgpio 

# --- 상태 상수 정의 ---
STATE_WAITING_FOR_SYSTEM = 'WAITING_FOR_SYSTEM'
STATE_NORMAL = 'NORMAL'
STATE_STOP_WAIT = 'STOP_WAIT'
STATE_PRE_TURN_STRAIGHT = 'PRE_TURN_STRAIGHT'
STATE_TURNING = 'TURNING'
STATE_POST_TURN_STRAIGHT = 'POST_TURN_STRAIGHT'
STATE_FINISHED = 'FINISHED'

BUZZER_PIN = 12
BUZZER_FREQ = 2000

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        self.get_logger().info('PID Controller Node (Waiting for Sign Detector...) started.')

        self.bridge = CvBridge()

        # --- 파라미터 ---
        self.declare_parameter('vehicle_speed', 0.2)
        self.declare_parameter('ym_per_pix', 0.01)
        self.declare_parameter('xm_per_pix', 0.005)
        self.declare_parameter('half_track_width_pixels', 250)
        self.declare_parameter('kp', 0.38)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('lookahead_row_offset', 20)
        self.declare_parameter('cmd_delay', 0.7)

        self.base_speed = self.get_parameter('vehicle_speed').get_parameter_value().double_value
        self.xm_per_pix = self.get_parameter('xm_per_pix').get_parameter_value().double_value
        self.half_track_width_px = self.get_parameter('half_track_width_pixels').get_parameter_value().integer_value
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.row_offset = self.get_parameter('lookahead_row_offset').get_parameter_value().integer_value
        self.cmd_delay = self.get_parameter('cmd_delay').get_parameter_value().double_value

        self.prev_error = 0.0
        self.integral_error = 0.0
        self.prev_time = self.get_clock().now().nanoseconds / 1e9

        # --- [수정] 초기 상태를 대기 상태로 설정 ---
        self.drive_state = STATE_WAITING_FOR_SYSTEM 
        
        self.state_start_time = 0.0
        self.turn_direction = 0.0 
        self.stop_wait_time = 0.0        
        self.next_state_after_stop = STATE_NORMAL

        self.detected_signs = set()
        self.current_sign = None
        self.slow_mode_end_time = 0.0

        self.h = lgpio.gpiochip_open(4)
        lgpio.gpio_claim_output(self.h, BUZZER_PIN)

        self.cmd_buffer = deque()

        # --- 구독 설정 ---
        self.subscription = self.create_subscription(
            Image, '/image_bev_binary', self.bev_callback, 10)
        
        self.red_subscription = self.create_subscription(
            Image, '/image_red_bev', self.red_bev_callback, 10)

        self.sign_subscription = self.create_subscription(
            String, '/sign_detection', self.sign_callback, 10)

        # [신규] 시스템 상태 구독
        self.status_subscription = self.create_subscription(
            String, '/system_status', self.status_callback, 10)
        
        self.publisher_drive = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self.create_timer(0.01, self.timer_callback)

    # --- [신규] 시스템 상태 콜백 ---
    def status_callback(self, msg):
        # 대기 상태일 때 "system_ready" 신호를 받으면 주행 시작
        if self.drive_state == STATE_WAITING_FOR_SYSTEM:
            if msg.data == "system_ready":
                self.get_logger().info(">>> System Ready Signal Received! STARTING DRIVE.")
                self.set_state(STATE_NORMAL)

    def red_bev_callback(self, msg):
        if self.drive_state == STATE_FINISHED or self.drive_state == STATE_WAITING_FOR_SYSTEM:
            return 
        if self.drive_state != STATE_NORMAL:
            return

        try:
            red_bev = self.bridge.imgmsg_to_cv2(msg, "mono8")
            h, w = red_bev.shape
        except: return

        check_row = int(h * 0.85)
        if check_row >= h: check_row = h - 1
        
        row_pixels = red_bev[check_row, :]
        white_count = np.count_nonzero(row_pixels)
        
        if white_count > w * 0.7:
            self.get_logger().warn("FINISH LINE DETECTED! Stopping Robot.")
            self.set_state(STATE_FINISHED)
            self.cmd_buffer.clear() 
            twist = Twist()
            self.publisher_drive.publish(twist)

    def sign_callback(self, msg):
        new_sign = msg.data
        now = self.get_clock().now().nanoseconds / 1e9

        # 대기 중이거나 끝났으면 무시
        if self.drive_state == STATE_FINISHED or self.drive_state == STATE_WAITING_FOR_SYSTEM: return
        if new_sign in self.detected_signs: return
        if self.drive_state != STATE_NORMAL: return

        self.get_logger().info(f'>>> NEW SIGN DETECTED: {new_sign}')
        self.detected_signs.add(new_sign)

        if new_sign == 'stop':
            self.stop_wait_time = 5.0
            self.next_state_after_stop = STATE_NORMAL
            self.set_state(STATE_STOP_WAIT)
        elif new_sign == 'traffic_light_green' or new_sign == 'traffic_light':
            self.stop_wait_time = 3.0
            self.next_state_after_stop = STATE_PRE_TURN_STRAIGHT
            self.turn_direction = -1.0 
            self.set_state(STATE_STOP_WAIT)
        elif new_sign == 'horn':
            self.beep_buzzer()
        elif new_sign == 'slow':
            self.slow_mode_end_time = now + 5.0
            self.get_logger().info("Slow mode activated for 5 seconds.")
        elif new_sign == 'left_turn':
            self.turn_direction = 1.0
            self.set_state(STATE_PRE_TURN_STRAIGHT)
        elif new_sign == 'right_turn':
            self.turn_direction = -1.0
            self.set_state(STATE_PRE_TURN_STRAIGHT)

    def set_state(self, new_state):
        self.drive_state = new_state
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f'State changed to: {self.drive_state}')

    def beep_buzzer(self):
        self.get_logger().info('BEEP! (1 sec)')
        lgpio.tx_pwm(self.h, BUZZER_PIN, BUZZER_FREQ, 50) 
        self.create_timer(1.0, self.buzzer_off_callback) 

    def buzzer_off_callback(self):
        lgpio.tx_pwm(self.h, BUZZER_PIN, BUZZER_FREQ, 0)

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        
        if self.drive_state == STATE_FINISHED or self.drive_state == STATE_WAITING_FOR_SYSTEM:
            self.cmd_buffer.clear()
            stop_msg = Twist()
            self.publisher_drive.publish(stop_msg)
            return

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

        # 대기 중이거나 끝났으면 정지
        if self.drive_state == STATE_FINISHED or self.drive_state == STATE_WAITING_FOR_SYSTEM:
            return

        try:
            bev_binary = self.bridge.imgmsg_to_cv2(msg, "mono8")
            h, w = bev_binary.shape
        except: return

        linear_vel = 0.0
        angular_vel = 0.0
        
        if self.drive_state == STATE_NORMAL:
            linear_vel = self.base_speed
            if now < self.slow_mode_end_time:
                linear_vel *= 0.5 
            steering_angle = self.calculate_pid(bev_binary, h, w, now)
            angular_vel = steering_angle

        elif self.drive_state == STATE_STOP_WAIT:
            linear_vel = 0.0
            angular_vel = 0.0
            if state_duration >= self.stop_wait_time:
                self.set_state(self.next_state_after_stop)

        elif self.drive_state == STATE_PRE_TURN_STRAIGHT:
            linear_vel = self.base_speed
            angular_vel = 0.0
            if state_duration >= 1.0:
                self.set_state(STATE_TURNING)

        elif self.drive_state == STATE_TURNING:
            linear_vel = 0.0  
            angular_vel = self.turn_direction * 2.0 
            if state_duration >= 1.4: 
                self.set_state(STATE_POST_TURN_STRAIGHT)

        elif self.drive_state == STATE_POST_TURN_STRAIGHT:
            linear_vel = self.base_speed
            angular_vel = 0.0
            if state_duration >= 1.0:
                self.current_sign = None
                self.set_state(STATE_NORMAL)

        twist = Twist()
        twist.linear.x = float(linear_vel)
        twist.angular.z = float(angular_vel)
        self.cmd_buffer.append((now, twist))

    def calculate_pid(self, bev_binary, h, w, now):
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
            target_px = robot_center_px - (self.prev_error / self.xm_per_pix)
        
        error_meters = (robot_center_px - target_px) * self.xm_per_pix
        dt = now - self.prev_time
        if dt <= 0: dt = 0.03
        p = self.kp * error_meters
        self.integral_error += error_meters * dt
        i = self.ki * self.integral_error
        
        d_error = 0.0
        if len(indices) > 0:
             d_error = (error_meters - self.prev_error)
             
        d = self.kd * (d_error) / dt
        
        self.prev_error = error_meters
        self.prev_time = now
        return p + i + d

    def destroy_node(self):
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
