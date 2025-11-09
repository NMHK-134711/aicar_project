#!/usr/bin/env python3
import sys, tty, termios, time
import lgpio
import numpy as np

# --- 1. 하드웨어 설정 ---
PWMA = 18; AIN1 = 22; AIN2 = 27
PWMB = 23; BIN1 = 25; BIN2 = 24
GPIOCHIP = 4; PWM_FREQ = 1000
h = None

# --- 2. 로봇 파라미터 (실제 환경에 맞게 수정 필수!) ---
WHEEL_SEP = 0.15   # 두 바퀴 사이 거리 (미터)
SPEED_GAIN = 150.0 # m/s -> PWM 변환 비율

# --- 3. 저수준 모터 제어 ---
def set_motor(in1, in2, pwm_pin, duty):
    if duty >= 0: lgpio.gpio_write(h, in1, 0); lgpio.gpio_write(h, in2, 1)
    else:         lgpio.gpio_write(h, in1, 1); lgpio.gpio_write(h, in2, 0)
    lgpio.tx_pwm(h, pwm_pin, PWM_FREQ, int(abs(duty)))

def stop():
    set_motor(AIN1, AIN2, PWMA, 0)
    set_motor(BIN1, BIN2, PWMB, 0)

# --- 4. 핵심: Twist -> Differential 변환 로직 ---
def apply_twist(v, w):
    # v = 선속도(m/s), w = 각속도(rad/s)
    vl = v - (w * WHEEL_SEP / 2.0)
    vr = v + (w * WHEEL_SEP / 2.0)
    
    # PWM 변환 및 클리핑
    duty_l = np.clip(vl * SPEED_GAIN, -100, 100)
    duty_r = np.clip(vr * SPEED_GAIN, -100, 100)
    
    print(f"Twist[v={v:.2f}, w={w:.2f}] -> PWM[L={duty_l:.0f}, R={duty_r:.0f}]")
    set_motor(AIN1, AIN2, PWMA, duty_l)
    set_motor(BIN1, BIN2, PWMB, duty_r)

# --- 5. 키 입력 유틸 ---
def getch():
    fd = sys.stdin.fileno(); old = termios.tcgetattr(fd)
    try: tty.setcbreak(fd); ch = sys.stdin.read(1)
    finally: termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

# --- 6. 메인 루프 ---
def main():
    global h
    try:
        h = lgpio.gpiochip_open(GPIOCHIP)
        for p in [PWMA, AIN1, AIN2, PWMB, BIN1, BIN2]: lgpio.gpio_claim_output(h, p)
        stop()
    except Exception as e: print(f"GPIO Error: {e}"); return

    v, w = 0.0, 0.0
    print("=== Twist Drive Test ===")
    print("w/x: 선속도(v) 증가/감소 | a/d: 각속도(w) 좌/우 증가 | s: 정지 | q: 종료")

    try:
        while True:
            key = getch()
            if key == 'w': v += 0.1
            elif key == 'x': v -= 0.1
            elif key == 'a': w += 0.5  # 좌회전 (각속도 증가)
            elif key == 'd': w -= 0.5  # 우회전 (각속도 감소)
            elif key == 's': v, w = 0.0, 0.0
            elif key == 'q': break
            apply_twist(v, w)
    finally:
        stop()
        if h: lgpio.gpiochip_close(h)

if __name__ == '__main__': main()