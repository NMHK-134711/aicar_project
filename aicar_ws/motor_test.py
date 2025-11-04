import sys
import tty
import termios
import time
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
TEST_SPEED = 50   # 'w' 키 등을 눌렀을 때의 듀티 사이클 (0~100)

h = None # lgpio 핸들

# --- 2. 저수준 모터 제어 함수 (lgpio) ---
def motor_go(pwm_duty_cycle):
    lgpio.gpio_write(h, AIN1_PIN, 0) # A: 전진
    lgpio.gpio_write(h, AIN2_PIN, 1)
    lgpio.gpio_write(h, BIN1_PIN, 0) # B: 전진
    lgpio.gpio_write(h, BIN2_PIN, 1)
    lgpio.tx_pwm(h, PWMA_PIN, PWM_FREQ, pwm_duty_cycle)
    lgpio.tx_pwm(h, PWMB_PIN, PWM_FREQ, pwm_duty_cycle)

def motor_back(pwm_duty_cycle):
    lgpio.gpio_write(h, AIN1_PIN, 1) # A: 후진
    lgpio.gpio_write(h, AIN2_PIN, 0)
    lgpio.gpio_write(h, BIN1_PIN, 1) # B: 후진
    lgpio.gpio_write(h, BIN2_PIN, 0)
    lgpio.tx_pwm(h, PWMA_PIN, PWM_FREQ, pwm_duty_cycle)
    lgpio.tx_pwm(h, PWMB_PIN, PWM_FREQ, pwm_duty_cycle)

def motor_left(pwm_duty_cycle):
    lgpio.gpio_write(h, AIN1_PIN, 1) # A: 후진
    lgpio.gpio_write(h, AIN2_PIN, 0)
    lgpio.gpio_write(h, BIN1_PIN, 0) # B: 전진
    lgpio.gpio_write(h, BIN2_PIN, 1)
    lgpio.tx_pwm(h, PWMA_PIN, PWM_FREQ, pwm_duty_cycle)
    lgpio.tx_pwm(h, PWMB_PIN, PWM_FREQ, pwm_duty_cycle)

def motor_right(pwm_duty_cycle):
    lgpio.gpio_write(h, AIN1_PIN, 0) # A: 전진
    lgpio.gpio_write(h, AIN2_PIN, 1)
    lgpio.gpio_write(h, BIN1_PIN, 1) # B: 후진
    lgpio.gpio_write(h, BIN2_PIN, 0)
    lgpio.tx_pwm(h, PWMA_PIN, PWM_FREQ, pwm_duty_cycle)
    lgpio.tx_pwm(h, PWMB_PIN, PWM_FREQ, pwm_duty_cycle)

def motor_stop():
    print("Stopping motors...")
    # PWM 듀티 사이클을 0으로 설정
    lgpio.tx_pwm(h, PWMA_PIN, PWM_FREQ, 0)
    lgpio.tx_pwm(h, PWMB_PIN, PWM_FREQ, 0)
    # 안전을 위해 방향 핀도 0으로 (Brake 모드)
    lgpio.gpio_write(h, AIN1_PIN, 0)
    lgpio.gpio_write(h, AIN2_PIN, 0)
    lgpio.gpio_write(h, BIN1_PIN, 0)
    lgpio.gpio_write(h, BIN2_PIN, 0)

# --- 3. 리눅스 터미널에서 키 입력 감지 ---
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# --- 4. 메인 테스트 루프 ---
def main_loop():
    global h # 전역 핸들 변수 사용
    
    try:
        # --- GPIO 칩 열기 및 핀 설정 ---
        h = lgpio.gpiochip_open(GPIOCHIP)
        
        lgpio.gpio_claim_output(h, PWMA_PIN)
        lgpio.gpio_claim_output(h, AIN1_PIN)
        lgpio.gpio_claim_output(h, AIN2_PIN)
        lgpio.gpio_claim_output(h, PWMB_PIN)
        lgpio.gpio_claim_output(h, BIN1_PIN)
        lgpio.gpio_claim_output(h, BIN2_PIN)
        
        print(f"Successfully opened gpiochip{GPIOCHIP}. Starting test...")
        motor_stop() # 안전하게 정지 상태로 시작

    except Exception as e:
        print("="*50)
        print(f"ERROR: GPIO 핀 초기화 실패: {e}")
        print("1. sudo로 실행했는지 확인하세요.")
        print("2. python3-lgpio가 설치되었는지 확인하세요.")
        print("3. Docker가 --privileged 또는 --device=/dev/gpiochip4로 실행되었는지 확인하세요.")
        print("="*50)
        sys.exit(1)

    print("\n" + "="*50)
    print("모터 제어 테스트 (저수준 LGPIO 로직)")
    print("w: 전진, s: 후진, a: 좌회전, d: 우회전")
    print(f"(속도: {TEST_SPEED}%)")
    print("space: 정지, q: 종료")
    print("="*50)

    while True:
        key = getch()

        if key == 'w':
            print("Action: Forward")
            motor_go(TEST_SPEED)
        elif key == 's':
            print("Action: Backward")
            motor_back(TEST_SPEED)
        elif key == 'a':
            print("Action: Left (Skid)")
            motor_left(TEST_SPEED)
        elif key == 'd':
            print("Action: Right (Skid)")
            motor_right(TEST_SPEED)
        elif key == ' ':
            print("Action: Stop")
            motor_stop()
        elif key == 'q':
            print("Action: Quit")
            break
        else:
            continue # 다른 키는 무시
        
        time.sleep(0.05)

if __name__ == '__main__':
    try:
        main_loop()
    except KeyboardInterrupt:
        print("\nCtrl+C로 종료합니다.")
    except Exception as e:
        print(f"\n오류 발생: {e}")
    finally:
        # 스크립트가 어떻게 종료되든 항상 모터를 정지시킵니다.
        if h:
            motor_stop()
            lgpio.gpiochip_close(h) # GPIO 핸들 닫기
            print("GPIO closed. 테스트 종료.")