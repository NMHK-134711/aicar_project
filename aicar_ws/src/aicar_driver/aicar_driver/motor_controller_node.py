import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import lgpio  # <-- gpiozero 대신 lgpio를 직접 임포트

# --- 1. 하드웨어 핀 및 설정 정의 (BCM 번호 기준) ---
PWMA_PIN = 18
AIN1_PIN = 22
AIN2_PIN = 27

PWMB_PIN = 23
BIN1_PIN = 25
BIN2_PIN = 24

GPIOCHIP = 4      # gpiodetect로 확인한 RPi 5의 메인 칩
PWM_FREQ = 1000   # 모터 PWM 주파수 (1000 Hz)

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.get_logger().info('Low-Level Motor Controller (lgpio) Node started.')
        
        # --- 2. 파라미터 선언 ---
        # "직진"으로 간주할 조향각의 임계값 (radian)
        self.declare_parameter('turn_threshold', 0.1) 
        self.turn_threshold = self.get_parameter('turn_threshold').get_parameter_value().double_value

        self.h = None # lgpio 핸들

        try:
            # --- 3. lgpio 칩 열기 및 핀 설정 ---
            self.h = lgpio.gpiochip_open(GPIOCHIP)
            
            # 모든 핀을 출력(Output)으로 설정
            lgpio.gpio_claim_output(self.h, PWMA_PIN)
            lgpio.gpio_claim_output(self.h, AIN1_PIN)
            lgpio.gpio_claim_output(self.h, AIN2_PIN)
            lgpio.gpio_claim_output(self.h, PWMB_PIN)
            lgpio.gpio_claim_output(self.h, BIN1_PIN)
            lgpio.gpio_claim_output(self.h, BIN2_PIN)
            
            self.get_logger().info(f"Successfully opened gpiochip{GPIOCHIP} and claimed 6 pins.")
            
            # 모든 모터를 정지 상태로 초기화
            self.motor_stop()

        except Exception as e:
            self.get_logger().fatal(f'Failed to initialize lgpio: {e}')
            self.get_logger().fatal('Is python3-lgpio installed? Is the Docker running with --privileged or --device=/dev/gpiochip4?')
            rclpy.shutdown() # 초기화 실패 시 노드 종료
            return

        # --- 4. ROS 구독 설정 ---
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            '/drive',  # PurePursuitNode가 발행하는 토픽
            self.drive_callback,
            10)

    def drive_callback(self, msg):
        """ /drive 토픽을 받아 스키드 스티어 로직으로 모터를 구동합니다. """
        
        base_speed = msg.drive.speed
        steering_angle = msg.drive.steering_angle
        
        # 속도(Speed)를 0.0 ~ 1.0 범위의 PWM 듀티 사이클(0~100)로 변환
        # (PurePursuit 노드가 0.5m/s 같은 값을 보내므로, 1.0을 최대 속도로 간주)
        # TODO: self.speed 파라미터(예: 0.5)를 100% 듀티 사이클로 매핑하는 것이 더 좋을 수 있음
        pwm_val = np.clip(abs(base_speed), 0.0, 1.0) * 80 # 듀티 사이클 (0-100)

        # 1. 조향각(steering_angle)에 따라 모터 방향 결정
        if abs(steering_angle) < self.turn_threshold:
            # "직진" 또는 "후진"
            if base_speed > 0:
                self.motor_go(pwm_val)
            elif base_speed < 0:
                self.motor_back(pwm_val)
            else:
                self.motor_stop()
        
        elif steering_angle < 0:
            # "좌회전" (스키드 스티어)
            self.motor_left(pwm_val)
            
        elif steering_angle > 0:
            # "우회전" (스키드 스티어)
            self.motor_right(pwm_val)

    def motor_go(self, pwm_duty_cycle):
        lgpio.gpio_write(self.h, AIN1_PIN, 0) # A: 전진
        lgpio.gpio_write(self.h, AIN2_PIN, 1)
        lgpio.gpio_write(self.h, BIN1_PIN, 0) # B: 전진
        lgpio.gpio_write(self.h, BIN2_PIN, 1)
        
        lgpio.tx_pwm(self.h, PWMA_PIN, PWM_FREQ, pwm_duty_cycle) # A: PWM 설정
        lgpio.tx_pwm(self.h, PWMB_PIN, PWM_FREQ, pwm_duty_cycle) # B: PWM 설정

    def motor_back(self, pwm_duty_cycle):
        lgpio.gpio_write(self.h, AIN1_PIN, 1) # A: 후진
        lgpio.gpio_write(self.h, AIN2_PIN, 0)
        lgpio.gpio_write(self.h, BIN1_PIN, 1) # B: 후진
        lgpio.gpio_write(self.h, BIN2_PIN, 0)
        
        lgpio.tx_pwm(self.h, PWMA_PIN, PWM_FREQ, pwm_duty_cycle)
        lgpio.tx_pwm(self.h, PWMB_PIN, PWM_FREQ, pwm_duty_cycle)

    def motor_right(self, pwm_duty_cycle):
        lgpio.gpio_write(self.h, AIN1_PIN, 1) # A: 후진
        lgpio.gpio_write(self.h, AIN2_PIN, 0)
        lgpio.gpio_write(self.h, BIN1_PIN, 0) # B: 전진
        lgpio.gpio_write(self.h, BIN2_PIN, 1)
        
        lgpio.tx_pwm(self.h, PWMA_PIN, PWM_FREQ, pwm_duty_cycle)
        lgpio.tx_pwm(self.h, PWMB_PIN, PWM_FREQ, pwm_duty_cycle)

    def motor_left(self, pwm_duty_cycle):
        lgpio.gpio_write(self.h, AIN1_PIN, 0) # A: 전진
        lgpio.gpio_write(self.h, AIN2_PIN, 1)
        lgpio.gpio_write(self.h, BIN1_PIN, 1) # B: 후진
        lgpio.gpio_write(self.h, BIN2_PIN, 0)
        
        lgpio.tx_pwm(self.h, PWMA_PIN, PWM_FREQ, pwm_duty_cycle)
        lgpio.tx_pwm(self.h, PWMB_PIN, PWM_FREQ, pwm_duty_cycle)

    def motor_stop(self):
        # PWM 듀티 사이클을 0으로 설정
        lgpio.tx_pwm(self.h, PWMA_PIN, PWM_FREQ, 0)
        lgpio.tx_pwm(self.h, PWMB_PIN, PWM_FREQ, 0)
        # 안전을 위해 방향 핀도 0으로 (Brake 모드)
        lgpio.gpio_write(self.h, AIN1_PIN, 0)
        lgpio.gpio_write(self.h, AIN2_PIN, 0)
        lgpio.gpio_write(self.h, BIN1_PIN, 0)
        lgpio.gpio_write(self.h, BIN2_PIN, 0)

    def destroy_node(self):
        """ 노드 종료 시 모터 정지 및 GPIO 해제 """
        self.get_logger().info('Stopping motors and closing GPIO chip.')
        if self.h:
            self.motor_stop()
            # PWM 핀 해제 (출력을 0으로 설정)
            lgpio.gpio_write(self.h, PWMA_PIN, 0)
            lgpio.gpio_write(self.h, PWMB_PIN, 0)
            # 칩 핸들 닫기
            lgpio.gpiochip_close(self.h)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = MotorControllerNode()
    try:
        rclpy.spin(motor_controller_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        motor_controller_node.get_logger().error(f'Unhandled exception: {e}')
    finally:
        # 노드가 어떻게 종료되든 (spin()이 끝나거나, 예외 발생)
        # destroy_node()가 호출되도록 보장합니다.
        # (rclpy.spin()이 예외로 종료되면 destroy_node()가 자동으로 호출되지 않을 수 있음)
        if rclpy.ok():
            motor_controller_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()