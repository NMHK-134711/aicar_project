ROS2 자율주행 차선 추종 프로젝트 (On-Device AI)

광운대학교 2025년도 2학기 온디바이스AI 수업 프로젝트

이 레포지토리는 Raspberry Pi 5와 단일 카메라를 사용하여 차선을 인식하고, Pure Pursuit 알고리즘을 통해 트랙을 자율주행하는 ROS2 (Humble) 기반의 자율주행 스택입니다.

모든 연산은 RPi 5의 Docker 컨테이너(Ubuntu 22.04) 내부에서 온디바이스로 처리됩니다.

🚀 시스템 아키텍처 (System Architecture)

본 프로젝트는 여러 개의 ROS2 노드가 토픽(Topic)을 통해 유기적으로 통신하는 모듈형 구조를 가집니다.

데이터 흐름 (Data Flow):

(C++) camera_ros (camera_node)

RPi 5의 Pi Camera에서 이미지를 캡처합니다. (640x480 @ 30fps)

하드웨어 ID를 직접 지정하고, 180도 회전된 이미지를 /raw_image 토픽으로 발행합니다.

(이 노드는 camera.launch.py를 통해 실행됩니다.)

(Python) aicar_vision (lane_detector_node)

/raw_image를 구독합니다.

[핵심] 고급 차선 감지 파이프라인을 실행합니다.

calibration.p 파일을 로드하여 렌즈 왜곡을 보정합니다.

복합 임계값(Sobel + HLS)을 적용하여 2진(binary) 이미지를 생성합니다.

Bird's Eye View (BEV)로 이미지를 변환합니다.

슬라이딩 윈도우 및 다항식 피팅으로 곡선 차선을 추적합니다.

차선이 그려진 /image_processed와 BEV 2진 이미지인 /image_bev_binary를 발행합니다.

(Python) aicar_controller (pure_pursuit_node)

/image_bev_binary (차선 경로)와 /sign_detection (표지판 정보, 추후 구현)을 구독합니다.

[핵심] BEV 이미지의 차선 중앙을 기반으로 Pure Pursuit 알고리즘을 계산합니다.

전방 주시 거리(Ld)와 차량 축거(L)를 기반으로 alpha 각도를 계산하여 조향각(steering_angle)을 도출합니다.

'stop' 표지판을 받으면 3초간 정지하는 상태 머신(State Machine)을 포함합니다.

최종 제어 명령인 AckermannDriveStamped 메시지를 /drive 토픽으로 발행합니다.

(Python) aicar_driver (motor_controller_node)

/drive 토픽을 구독합니다.

[핵심] Ackermann 메시지(조향각)를 실제 하드웨어인 2-Wheel 스키드 스티어(Skid Steer) 로직(좌/우 모터 속도 차이)으로 변환합니다.

gpiozero 대신 저수준 lgpio 라이브러리를 사용하여 RPi 5의 gpiochip4에 직접 PWM 및 디지털 신호를 전송하여 모터를 구동합니다.

🛠️ 환경 설정 (Environment Setup)

하드웨어: Raspberry Pi 5, Pi Camera, 2-Wheel (Skid Steer) 모터 드라이버 (L298N 등)

OS: Ubuntu 24.04 (Host)

플랫폼: Docker (Ubuntu 22.04 jammy 컨테이너, --privileged 또는 --device=/dev/gpiochip4 필요)

ROS: ROS2 Humble (Humble Hawksbill)

필수 의존성 (Docker 내부)

# ROS2 기본 및 Ackermann 메시지
sudo apt install ros-humble-desktop ros-humble-ackermann-msgs ros-humble-rqt-image-view

# RPi 5 저수준 GPIO 제어 (gpiozero 대체)
sudo apt install python3-lgpio gpiod

# OpenCV 및 비전 관련
sudo apt install python3-opencv python3-numpy python3-scipy

# (C++ 카메라 노드 설치 - camera_ros 레포지토리 빌드 필요)
# (이미 src/camera_ros에 포함되어 있음)


🕹️ 사용 방법 (Workflow)

프로젝트를 처음부터 실행하는 전체 워크플로우입니다.

1. (최초 1회) 하드웨어 테스트

모터가 올바른 핀에 연결되었는지, 방향이 맞는지 lgpio로 직접 테스트합니다.

# (환경 변수 설정이 필요할 수 있음)
# export GPIOZERO_PIN_FACTORY=lgpio
# export LGPIO_GPIOCHIP=4

# sudo를 사용하여 실행
sudo python3 motor_test.py 


w, a, s, d 키로 모터 작동을 확인합니다.

만약 a(좌)와 d(우)가 반대라면, motor_test.py와 motor_controller_node.py의 motor_left / motor_right 함수 내용을 서로 교체합니다.

2. (최초 1회) 카메라 보정

lane_detector_node가 렌즈 왜곡을 보정할 수 있도록 calibration.p 파일을 생성해야 합니다.

1. 원본 카메라 노드 실행

(별도의 터미널에서 camera_ros의 기본 노드를 실행합니다)

source install/setup.bash
ros2 run camera_ros camera_node --ros-args -r __ns:=/camera_node


2. 보정용 이미지 캡처

(별도의 터미널에서 체스판을 비추며 1장씩 저장)

# calibration_input 폴더 생성
mkdir -p calibration_input

# 체스판을 첫 번째 각도로 맞춘 뒤, 아래 명령어 실행
python3 save_frame.py calibration_input/frame_01.jpg

# 다른 각도에서 다시 실행
python3 save_frame.py calibration_input/frame_02.jpg
# (20~30장 수집)


3. 보정 파일 생성

create_calibration.py 스크립트를 실행하여 calibration.p 파일을 생성합니다.

# (체스판 내부 코너 개수에 맞게 --nx, --ny 수정)
python3 create_calibration.py \
    --input_dir 'calibration_input' \
    --output_file 'src/aicar_vision/calibration_data/calibration.p' \
    --nx 9 --ny 6


3. ROS2 워크스페이스 빌드

모든 패키지를 빌드합니다.

cd /root/aicar_ws
colcon build
source install/setup.bash


4. 🚀 메인 시스템 실행

모든 설정이 완료되면, aicar_bringup 런치 파일 하나로 모든 노드를 실행합니다.

ros2 launch aicar_bringup aicar_drive.launch.py


이 명령어는 다음 4개의 노드를 모두 실행시킵니다:

camera_node (C++ 카메라)

lane_detector_node (비전/차선 감지)

pure_pursuit_node (제어/알고리즘)

motor_controller_node (하드웨어/모터 구동)
