# ROS2 자율주행 차선 추종 프로젝트 (On-Device AI)

**광운대학교 2025년도 2학기 온디바이스AI 수업 프로젝트**

본 프로젝트는 Raspberry Pi 5와 단일 카메라 환경에서, ROS2 (Humble) 프레임워크를 기반으로 차선을 인식하고 표지판에 반응하여 트랙을 자율주행하는 온디바이스 AI 시스템입니다.

-----

### 🖥️ OS 셋팅 (Operating Environment)

본 프로젝트는 Raspberry Pi 5의 하드웨어 성능을 활용하면서, 안정적인 ROS2 Humble 배포판을 사용하기 위해 다층화된 OS 환경에서 개발되었습니다.

  * **Host OS:** Ubuntu 24.04 (라즈베리파이 5 호스트)
  * **플랫폼:** Docker
  * **Container OS:** Ubuntu 22.04 
  * **ROS2:** ROS2 Humble

Docker 컨테이너는 RPi 5의 GPIO 및 하드웨어에 접근하기 위해 `--privileged` 옵션으로 실행되어야 합니다.

-----

## 🚀 프로젝트 아키텍처 및 워크플로우

본 시스템은 각각의 독립적인 기능을 수행하는 ROS2 노드가 유기적으로 연결되어 작동합니다. 전체 데이터 흐름은 **"인식 → 판단 → 제어"**의 3단계로 구성되며, **모델 로딩 대기 후 시스템 시작(Handshake)** 로직이 포함되어 있습니다.

### 🌊 데이터 흐름도 (Node Workflow)

### 1\. 👁️ 1단계: 인식 (Perception)

로봇이 "보고" "이해"하는 단계입니다. 3개의 핵심 인식 노드가 카메라의 원본 이미지를 처리합니다.

#### (1) 카메라 노드 (`camera_ros`)

  * **역할:** 시스템의 "눈"
  * **작업:** RPi 5의 Pi Camera 하드웨어를 `libcamera` (C++)를 통해 직접 제어합니다. 640x480 이미지를 캡처합니다.
  * **발행 (Output):** `/camera_node/image_raw` (Type: `sensor_msgs/Image`)
      * 시스템의 모든 비전 노드가 사용할 원본 이미지입니다.

#### (2) 차선 검출 노드 (`lane_detector_node`)

  * **역할:** "바닥(경로) 및 종료선 인식"
  * **구독 (Input):** `/camera_node/image_raw`
  * **작업:**
    1.  **왜곡 보정:** `calibration.p` 파일을 이용해 카메라 렌즈의 어안 왜곡을 보정합니다.
    2.  **BEV 변환:** 원근감이 있는 이미지를 하늘에서 내려다본 **Bird's Eye View (BEV)** 이미지로 변환합니다.
    3.  **이중 색상 처리:**
        * **Yellow:** 주행용 차선을 추출합니다.
        * **Red:** 주행 종료 지점(Finish Line)을 추출합니다.
    4.  **모폴로지 연산:** 얇은 차선을 더 잘 인식하도록 `dilate` (팽창) 연산을 적용합니다.
  * **발행 (Output):**
      * `/image_bev_binary` (주행용 노란색 차선 BEV)
      * `/image_red_bev` (종료 확인용 빨간색 라인 BEV)
      * `/image_processed` (표지판 인식용 보정 이미지)

#### (3) 표지판 인식 노드 (`sign_detector_node`)

  * **역할:** "신호등(규칙) 인식 및 시스템 준비 상태 알림"
  * **구독 (Input):** `/camera_node/image_raw`
  * **작업:**
    1.  **모델 로딩:** YOLOv8 (TFLite float32) 모델을 로드합니다. 모델 로딩이 완료되기 전까지는 추론을 수행하지 않습니다.
    2.  **객체 감지:** `stop`, `left_turn`, `right_turn`, `horn`, `slow(20)`, `traffic_light` 등을 인식합니다.
    3.  **시스템 준비 알림:** 모델 로딩이 완료되면 주기적으로 시스템 준비 신호를 보냅니다.
  * **발행 (Output):** * `/sign_detection` (Type: `std_msgs/String`): 인식된 표지판의 명령어를 발행합니다.
      * `/system_status` (Type: `std_msgs/String`): 모델 로드 완료 시 `"system_ready"` 메시지를 발행하여 주행 시작을 알립니다.

-----

### 2\. 🧠 2단계: 판단 (Decision Making)

인식된 정보를 바탕으로 차량이 "어떻게 움직여야 할지" 결정하는 단계입니다.

#### (4) PID 제어 노드 (`pid_controller_node`)

  * **역할:** 시스템의 "뇌" (주행 알고리즘 및 상태 머신)
  * **구독 (Input):**
    1.  `/image_bev_binary` (차선 경로)
    2.  `/image_red_bev` (종료선 감지)
    3.  `/sign_detection` (표지판 규칙)
    4.  `/system_status` (시스템 준비 신호)
  * **작업:**
    1.  **시스템 동기화 (Start-up Logic):** 초기 상태는 `WAITING_FOR_SYSTEM`입니다. `/system_status` 토픽에서 `"system_ready"` 신호를 받아야만 `NORMAL` 상태로 전환되어 주행을 시작합니다.
    2.  **상태 머신 (State Machine):**
        * `NORMAL`: 기본 차선 추종 주행
        * `STOP_WAIT`: 정지 표지판 또는 신호등 감지 시 정지
        * `PRE_TURN_STRAIGHT` / `TURNING` / `POST_TURN_STRAIGHT`: 교차로 회전 시퀀스 수행
        * `FINISHED`: 빨간색 종료선을 감지하면 주행을 영구 정지
    3.  **PID 제어:** BEV 이미지에서 차선 중심과 차량 중심의 오차(error)를 계산하고, PID 알고리즘을 통해 목표 조향각(angular.z)을 산출합니다.
  * **발행 (Output):** `/cmd_vel` (Type: `geometry_msgs/Twist`)
      * 선속도(linear.x)와 각속도(angular.z) 명령을 발행합니다.

-----

### 3\. 💪 3단계: 제어 (Control)

판단된 상위 레벨의 속도 명령을 실제 하드웨어 특성에 맞게 변환하여 모터를 구동하는 단계입니다.

#### (5) 차동 구동 노드 (`differential_drive_node`)

  * **역할:** 시스템의 "근육" (모터 드라이버)
  * **구독 (Input):** `/cmd_vel` (`geometry_msgs/Twist`)
  * **작업:**
    1.  **역기구학(Inverse Kinematics):** 로봇의 선속도($v$)와 각속도($\omega$)를 입력받아, **차동 구동 모델** 공식을 통해 좌/우 바퀴의 필요 속도를 계산합니다. 
          * $v_{left} = v - \frac{(\omega \cdot gain) L}{2}, \quad v_{right} = v + \frac{(\omega \cdot gain) L}{2}$
    2.  **하드웨어 제어:** `lgpio` 라이브러리를 사용하여 RPi 5의 GPIO 핀에 PWM 신호를 인가, DC 모터를 정밀 제어합니다.
  * **출력 (Output):** 좌우 DC 모터 구동
