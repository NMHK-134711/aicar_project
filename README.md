# ROS2 자율주행 차선 추종 프로젝트 (On-Device AI)

**광운대학교 2025년도 2학기 온디바이스AI 수업 프로젝트**

본 프로젝트는 Raspberry Pi 5와 단일 카메라 환경에서, ROS2 (Humble) 프레임워크를 기반으로 차선을 인식하고 표지판에 반응하여 트랙을 자율주행하는 온디바이스 AI 시스템입니다.

---

### 🖥️ OS 셋팅 (Operating Environment)

본 프로젝트는 Raspberry Pi 5의 하드웨어 성능을 활용하면서, 안정적인 ROS2 Humble 배포판을 사용하기 위해 다층화된 OS 환경에서 개발되었습니다.

* **Host OS:** Ubuntu 24.04 (라즈베리파이 5 호스트)
* **플랫폼:** Docker
* **Container OS:** Ubuntu 22.04 (Jammy Jellyfish)
* **ROS2:** ROS2 Humble (Ubuntu 22.04의 공식 지원 버전)

Docker 컨테이너는 RPi 5의 GPIO 하드웨어에 접근하기 위해 `--privileged` 또는 `--device=/dev/gpiochip4` 옵션으로 실행되어야 합니다.

---

## 🚀 프로젝트 아키텍처 및 워크플로우

본 시스템은 각각의 독립적인 기능을 수행하는 5개의 ROS2 노드가 유기적으로 연결되어 작동합니다. 전체 데이터 흐름은 "인식 → 판단 → 제어"의 3단계로 구성됩니다.

### 🌊 데이터 흐름도 (Node Workflow)

![ROS2 Node Workflow](workflow.png)


###  1. 👁️ 1단계: 인식 (Perception)

로봇이 "보고" "이해"하는 단계입니다. 2개의 인식 노드가 카메라의 원본 이미지를 병렬로 처리합니다.

#### (1) 카메라 노드 (`camera_ros`)
* **역할:** 시스템의 "눈"
* **작업:** RPi 5의 Pi Camera 하드웨어를 `libcamera` (C++)를 통해 직접 제어합니다. 180도 회전된 640x480 이미지를 30fps로 캡처합니다.
* **발행 (Output):** `/camera_node/image_raw` (Type: `sensor_msgs/Image`)
    * 시스템의 모든 비전 노드가 사용할 원본 이미지입니다.

#### (2) 차선 검출 노드 (`aicar_vision`)
* **역할:** "바닥(경로) 인식"
* **구독 (Input):** `/camera_node/image_raw`
* **작업:**
    1.  **왜곡 보정:** `calibration.p` 파일을 이용해 카메라 렌즈의 어안 왜곡을 폅니다.
    2.  **임계값 처리:** Sobel, HLS 등 복합 필터를 적용해 조명 변화에 강인한 2진(binary) 이미지를 생성합니다.
    3.  **BEV 변환:** 원근감이 있는 이미지를 하늘에서 내려다본 **Bird's Eye View (BEV)** 이미지로 변환합니다.
    4.  **차선 피팅:** BEV 이미지에서 슬라이딩 윈도우로 곡선 차선을 추적하고 다항식을 피팅합니다.
* **발행 (Output):** `/image_bev_binary` (Type: `sensor_msgs/Image`)
    * 자율주행 노드가 "경로"로 사용할, 차선만 하얗게 표시된 평면도입니다.

#### (3) 표지판 인식 노드 (향후 구현)
* **역할:** "신호등(규칙) 인식"
* **구독 (Input):** `/camera_node/image_raw`
* **작업 (계획):** YOLO 같은 경량화된 객체 감지 모델을 사용해 'stop', 'right_turn' 등의 표지판을 인식합니다.
* **발행 (Output):** `/sign_detection` (Type: `std_msgs/String`)
    * 표지판을 감지했을 때만 해당 문자열(예: "stop")을 발행합니다.

---

###  2. 🧠 2단계: 판단 (Decision Making)

인식된 정보를 바탕으로 차량이 "어떻게 움직여야 할지" 결정하는 단계입니다.

#### (4) 자율주행 노드 (`aicar_controller`)
* **역할:** 시스템의 "뇌" (알고리즘)
* **구독 (Input):**
    1.  `/image_bev_binary` (차선 경로)
    2.  `/sign_detection` (표지판 규칙)
* **작업:**
    1.  **경로 추종:** `/image_bev_binary` (BEV 이미지)의 차선 중앙을 경로로 간주하고, **Pure Pursuit** 알고리즘을 사용해 이 경로를 따라갈 **조향각(steering_angle)**을 계산합니다.
    2.  **규칙 준수:** `/sign_detection` 토픽을 감시하여, 만약 "stop" 메시지를 받으면 `vehicle_speed`를 0으로 설정하고 3초간 정지하는 **상태 머신(State Machine)**을 실행합니다.
* **발행 (Output):** `/drive` (Type: `ackermann_msgs/AckermannDriveStamped`)
    * 차량의 목표 속도(예: 0.5m/s)와 계산된 조향각(예: 0.15 rad)이 포함된 표준 제어 명령입니다.

---

###  3. 💪 3단계: 제어 (Control)

판단된 제어 명령을 실제 하드웨어의 "근육"으로 전달하는 단계입니다.

#### (5) 모터 제어 노드 (`aicar_driver`)
* **역할:** 시스템의 "근육" (하드웨어 드라이버)
* **구독 (Input):** `/drive`
* **작업:**
    1.  **로직 변환:** `Ackermann` 메시지(앞바퀴 조향각 개념)를 2륜 모터 하드웨어에 맞는 **스키드 스티어(Skid Steer)** 로직(탱크처럼 좌우 바퀴의 속도를 다르게 하는)으로 변환합니다.
    2.  **저수준 제어:** RPi 5의 `gpiochip4`에 직접 접근하기 위해 `gpiozero` 대신 저수준 **`lgpio`** 라이브러리를 사용합니다.
    3.  AIN/BIN 핀으로 방향을 설정하고, PWM 핀으로 `lgpio.tx_pwm` 신호를 보내 모터 속도를 정밀하게 제어합니다.
* **출력 (Output):** 실제 모터 구동
