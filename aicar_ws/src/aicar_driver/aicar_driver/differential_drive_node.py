import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import lgpio 

# --- 하드웨어 핀 정의 ---
PWMA = 18; AIN1 = 22; AIN2 = 27
PWMB = 23; BIN1 = 25; BIN2 = 24
GPIOCHIP = 4; PWM_FREQ = 1000

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.get_logger().info('Motor Node (Standard Twist) started.')
        
        # --- 튜닝 파라미터 ---
        # wheel_separation: 왼쪽 바퀴 중심과 오른쪽 바퀴 중심 사이의 거리 [미터]
        self.declare_parameter('wheel_separation', 0.106)
        self.declare_parameter('speed_gain', 150.0) # m/s -> PWM 변환 비율

        self.wheel_sep = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.speed_gain = self.get_parameter('speed_gain').get_parameter_value().double_value
        
        # GPIO 초기화
        self.h = lgpio.gpiochip_open(GPIOCHIP)
        for pin in [PWMA, AIN1, AIN2, PWMB, BIN1, BIN2]:
            lgpio.gpio_claim_output(self.h, pin)
        self.motor_stop()

        # 구독 변경: /drive (Ackermann) -> /cmd_vel (Twist)
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        # 1. Twist 메시지에서 선속도(v), 각속도(w) 추출
        linear_x = msg.linear.x   # m/s
        angular_z = msg.angular.z # rad/s

        # 2. 차동 구동 역기구학(Inverse Kinematics) 공식 적용
        # v_left  = v - (w * L / 2)
        # v_right = v + (w * L / 2)
        vl_ms = linear_x - (angular_z * self.wheel_sep / 2.0)
        vr_ms = linear_x + (angular_z * self.wheel_sep / 2.0)

        # 3. m/s를 PWM 듀티비(0~100)로 변환
        duty_l = np.clip(vl_ms * self.speed_gain, -100, 100)
        duty_r = np.clip(vr_ms * self.speed_gain, -100, 100)

        # 4. 모터 하드웨어 제어
        self.set_motor(AIN1, AIN2, PWMA, duty_l)
        self.set_motor(BIN1, BIN2, PWMB, duty_r)

    def set_motor(self, in1, in2, pwm_pin, duty):
        if duty >= 0:   # 전진
            lgpio.gpio_write(self.h, in1, 0)
            lgpio.gpio_write(self.h, in2, 1)
        else:           # 후진
            lgpio.gpio_write(self.h, in1, 1)
            lgpio.gpio_write(self.h, in2, 0)
        lgpio.tx_pwm(self.h, pwm_pin, PWM_FREQ, abs(duty))

    def motor_stop(self):
        self.set_motor(AIN1, AIN2, PWMA, 0)
        self.set_motor(BIN1, BIN2, PWMB, 0)

    def destroy_node(self):
        if self.h:
            self.motor_stop()
            lgpio.gpiochip_close(self.h)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()