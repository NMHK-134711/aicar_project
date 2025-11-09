#!/usr/bin/env python3
import sys
import tty
import termios
import time
import lgpio
import numpy as np

# --- 1. 하드웨어 핀 및 설정 ---
PWMA_PIN = 18
AIN1_PIN = 22
AIN2_PIN = 27
PWMB_PIN = 23
BIN1_PIN = 25
BIN2_PIN = 24
GPIOCHIP = 4
PWM_FREQ = 1000

h = None

# --- 2. 저수준 모터 제어 함수 (개별 모터 제어) ---
def set_motor(in1, in2, pwm_pin, pwm_value):
    # pwm_value는 -100 ~ 100 사이의 값
    pwm_value = np.clip(pwm_value, -100, 100)
    
    if pwm_value > 0:   # 전진
        lgpio.gpio_write(h, in1, 0)
        lgpio.gpio_write(h, in2, 1)
        lgpio.tx_pwm(h, pwm_pin, PWM_FREQ, abs(pwm_value))
    elif pwm_value < 0: # 후진
        lgpio.gpio_write(h, in1, 1)
        lgpio.gpio_write(h, in2, 0)
        lgpio.tx_pwm(h, pwm_pin, PWM_FREQ, abs(pwm_value))
    else:               # 정지
        lgpio.gpio_write(h, in1, 0)
        lgpio.gpio_write(h, in2, 0)
        lgpio.tx_pwm(h, pwm_pin, PWM_FREQ, 0)

def motor_stop():
    print("Stopping motors...")
    set_motor(AIN1_PIN, AIN2_PIN, PWMA_PIN, 0)
    set_motor(BIN1_PIN, BIN2_PIN, PWMB_PIN, 0)

# --- 3. 키 입력 감지 ---
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# --- 4. 메인 테스트 루프 (차동 구동 로직 적용) ---
def main_loop():
    global h
    try:
        h = lgpio.gpiochip_open(GPIOCHIP)
        for pin in [PWMA_PIN, AIN1_PIN, AIN2_PIN, PWMB_PIN, BIN1_PIN, BIN2_PIN]:
            lgpio.gpio_claim_output(h, pin)
        print(f"Successfully opened gpiochip{GPIOCHIP}. Starting Differential Drive test...")
        motor_stop()
    except Exception as e:
        print(f"ERROR: GPIO Init Failed: {e}")
        sys.exit(1)

    print("\n" + "="*50)
    print("모터 제어 테스트 (차동 구동 - Differential Drive)")
    print("w/s: 기본 속도 증가/감소")
    print("a/d: 조향각(회전율) 왼쪽/오른쪽 증가")
    print("space: 즉시 정지 (속도=0, 조향=0)")
    print("q: 종료")
    print("="*50)

    speed = 0.0      # 현재 기본 속도 (-100 ~ 100)
    steering = 0.0   # 현재 조향각 (음수=좌회전, 양수=우회전)
    
    SPEED_STEP = 10.0    # w/s 누를 때 속도 변화량
    STEER_STEP = 5.0     # a/d 누를 때 조향 변화량
    MAX_SPEED = 80.0     # 최대 속도 제한

    while True:
        key = getch()

        if key == 'w':
            speed = min(speed + SPEED_STEP, MAX_SPEED)
        elif key == 's':
            speed = max(speed - SPEED_STEP, -MAX_SPEED)
        elif key == 'a':
            steering -= STEER_STEP  # 왼쪽으로 더 꺾음
        elif key == 'd':
            steering += STEER_STEP  # 오른쪽으로 더 꺾음
        elif key == ' ':
            speed = 0.0
            steering = 0.0
        elif key == 'q':
            break
        else:
            continue

        # --- 차동 구동 계산 핵심 ---
        # 왼쪽 바퀴 = 기본속도 + 조향값
        # 오른쪽 바퀴 = 기본속도 - 조향값
        # 예: 속도 50, 조향 +10(우회전) -> 왼쪽 60, 오른쪽 40 -> 우회전
        left_pwm = speed + steering
        right_pwm = speed - steering

        print(f"Speed: {speed:.0f}, Steer: {steering:.0f} | Left PWM: {left_pwm:.0f}, Right PWM: {right_pwm:.0f}")
        
        # 모터에 적용 (set_motor 함수 내부에서 -100~100 클리핑 됨)
        set_motor(AIN1_PIN, AIN2_PIN, PWMA_PIN, left_pwm)
        set_motor(BIN1_PIN, BIN2_PIN, PWMB_PIN, right_pwm)
        
        time.sleep(0.05)

if __name__ == '__main__':
    try:
        main_loop()
    except KeyboardInterrupt:
        print("\nCtrl+C detected.")
    finally:
        if h:
            motor_stop()
            lgpio.gpiochip_close(h)
            print("Test finished.")