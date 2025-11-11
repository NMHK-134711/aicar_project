import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import lgpio
import time
from collections import deque # 큐 추가

# --- 상태 상수 ---
STATE_NORMAL = 'NORMAL'
STATE_STOP_WAIT = 'STOP_WAIT'
STATE_PRE_TURN_STRAIGHT = 'PRE_TURN_STRAIGHT'
STATE_TURNING = 'TURNING'
STATE_POST_TURN_STRAIGHT = 'POST_TURN_STRAIGHT'

BUZZER_PIN = 12

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.get_logger().info('Pure Pursuit Node (Full State Machine + Delay) started.')

        self.bridge = CvBridge()

        # --- 파라미터 ---
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('vehicle_speed', 0.5)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('ym_per_pix', 0.01)
        self.declare_parameter('xm_per_pix', 0.005)
        # [추가] 명령 지연 시간 파라미터
        self.declare_parameter('cmd_delay', 0.0)

        self.Ld = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.base_speed = self.get_parameter('vehicle_speed').get_parameter_value().double_value
        self.L = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.ym_per_pix = self.get_parameter('ym_per_pix').get_parameter_value().double_value
        self.xm_per_pix = self.get_parameter('xm_per_pix').get_parameter_value().double_value
        self.cmd_delay = self.get_parameter('cmd_delay').get_parameter_value().double_value

        # --- 상태 머신 변수 ---
        self.drive_state = STATE_NORMAL
        self.state_start_time = 0.0
        self.turn_direction = 0.0
        self.red_light_sequence = False
        self.current_sign = None
        self.last_buzzer_time = 0.0

        # [추가] 명령 버퍼
        self.cmd_buffer = deque()

        self.h = lgpio.gpiochip_open(4)
        lgpio.gpio_claim_output(self.h, BUZZER_PIN)

        self.subscription = self.create_subscription(Image, '/image_bev_binary', self.bev_callback, 10)
        self.sign_subscription = self.create_subscription(String, '/sign_detection', self.sign_callback, 10)
        self.publisher_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # [추가] 100Hz 타이머로 버퍼 확인 및 명령 발행
        self.create_timer(0.01, self.timer_callback)

    def sign_callback(self, msg):
        new_sign = msg.data
        if new_sign == self.current_sign: return
        self.current_sign = new_sign
        now = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f'Sign: {self.current_sign}')

        if self.drive_state == STATE_NORMAL:
            if self.current_sign == 'stop': self.set_state(STATE_STOP_WAIT)
            elif self.current_sign == 'left_turn':
                self.turn_direction = 1.0
                self.set_state(STATE_PRE_TURN_STRAIGHT)
            elif self.current_sign == 'right_turn':
                self.turn_direction = -1.0
                self.set_state(STATE_PRE_TURN_STRAIGHT)
            elif self.current_sign == 'traffic_light_green':
                self.turn_direction = -1.0
                self.set_state(STATE_PRE_TURN_STRAIGHT)
            elif self.current_sign == 'traffic_light_red':
                self.red_light_sequence = True
                self.set_state(STATE_STOP_WAIT)

        if self.current_sign == 'horn' and (now - self.last_buzzer_time > 2.0):
            self.beep_buzzer()
            self.last_buzzer_time = now

    def set_state(self, new_state):
        self.drive_state = new_state
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9

    def beep_buzzer(self):
        lgpio.gpio_write(self.h, BUZZER_PIN, 1)
        time.sleep(0.1) 
        lgpio.gpio_write(self.h, BUZZER_PIN, 0)

    # [추가] 타이머 콜백: 버퍼에서 지연 시간이 지난 명령 발행
    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        while self.cmd_buffer:
            cmd_time, drive_msg = self.cmd_buffer[0]
            if now - cmd_time >= self.cmd_delay:
                self.publisher_drive.publish(drive_msg)
                self.cmd_buffer.popleft()
            else:
                break

    def bev_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        duration = now - self.state_start_time
        speed = 0.0
        steering = 0.0

        if self.drive_state == STATE_NORMAL:
            speed = self.base_speed
            if self.current_sign == 'slow': speed *= 0.5
            steering = self.calculate_pure_pursuit(msg)

        elif self.drive_state == STATE_STOP_WAIT:
            speed = 0.0
            wait_time = 4.0 if self.red_light_sequence else 5.0
            if duration >= wait_time:
                if self.red_light_sequence:
                    self.red_light_sequence = False
                    self.turn_direction = -1.0
                    self.set_state(STATE_PRE_TURN_STRAIGHT)
                else:
                    self.current_sign = None
                    self.set_state(STATE_NORMAL)

        elif self.drive_state == STATE_PRE_TURN_STRAIGHT:
            speed = self.base_speed; steering = 0.0
            if duration >= 1.0: self.set_state(STATE_TURNING)

        elif self.drive_state == STATE_TURNING:
            speed = 0.4 
            steering = self.turn_direction * 1.0 
            if duration >= 1.9: self.set_state(STATE_POST_TURN_STRAIGHT)

        elif self.drive_state == STATE_POST_TURN_STRAIGHT:
            speed = self.base_speed; steering = 0.0
            if duration >= 1.0:
                self.current_sign = None
                self.set_state(STATE_NORMAL)

        drive_msg = AckermannDriveStamped()
        drive_msg.header = msg.header
        drive_msg.drive.speed = float(speed)
        drive_msg.drive.steering_angle = float(steering)
        
        # [수정] 바로 발행하지 않고 버퍼에 저장
        self.cmd_buffer.append((now, drive_msg))

    def calculate_pure_pursuit(self, msg):
        try:
            bev_binary = self.bridge.imgmsg_to_cv2(msg, "mono8")
            h, w = bev_binary.shape
        except: return 0.0
        Ld_pixels = self.Ld / self.ym_per_pix
        y_row = int(max(0, h - Ld_pixels))
        indices = np.nonzero(bev_binary[y_row, :])[0]
        if len(indices) == 0: return 0.0
        
        cx = (indices[0] + indices[-1]) / 2.0 if len(indices) >= 2 else indices[0]
        target_x = (cx - w/2.0) * self.xm_per_pix
        target_y = (h - y_row) * self.ym_per_pix
        alpha = math.atan2(target_x, target_y)
        return math.atan(2.0 * self.L * math.sin(alpha) / self.Ld)

    def destroy_node(self):
        if self.h: lgpio.gpiochip_close(self.h)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()