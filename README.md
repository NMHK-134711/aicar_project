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

Docker 컨테이너는 RPi 5의 GPIO 하드웨어에 접근하기 위해 `--privileged` 옵션으로 실행되어야 합니다.

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

인식된 정보를 바탕으로 차량이 "어떻게 움직여야 할지" 결정하는 단계입니다. 기존 Pure Pursuit 방식에서 응답성이 더 뛰어난 PID 제어 방식으로 변경되었습니다.

#### (4) PID 제어 노드 (`pid_controller_node`)
* **역할:** 시스템의 "뇌" (주행 알고리즘)
* **구독 (Input):**
    1.  `/image_bev_binary` (차선 경로)
    2.  `/sign_detection` (표지판 규칙)
* **작업:**
    1.  **조향각 계산 (PID Control):** BEV 이미지의 특정 전방 지점(`lookahead_row_offset`)에서 차선 중심과 차량 중심 사이의 오차(error)를 계산합니다. 이 오차에 대해 **PID(비례-적분-미분)** 제리기법을 적용하여 목표 각속도(`angular.z`)를 계산합니다.
        * *P(비례):* 트랙 중심에서 벗어난 만큼 빠르게 복귀하려 합니다.
        * *D(미분):* 급격한 조향으로 인한 진동(wobble)을 억제하여 부드러운 주행을 구현합니다.
    2.  **규칙 준수:** `/sign_detection` 토픽을 감시하여, "stop" 표지판 인식 시 설정된 시간(`stop_duration`) 동안 정지하는 상태 머신을 실행합니다.
* **발행 (Output):** `/cmd_vel` (Type: `geometry_msgs/Twist`)
    * 기존의 조향각 전용 메시지 대신, ROS2 표준 속도 명령인 **Twist** 메시지를 사용하여 선속도(linear.x)와 각속도(angular.z)를 함께 발행합니다.

---

###  3. 💪 3단계: 제어 (Control)

판단된 상위 레벨의 속도 명령을 실제 하드웨어 특성에 맞게 변환하여 모터를 구동하는 단계입니다.

#### (5) 차동 구동 노드 (`differential_drive_node`)
* **역할:** 시스템의 "근육" (모터 드라이버)
* **구독 (Input):** `/cmd_vel` (`geometry_msgs/Twist`)
* **작업:**
    1.  **역기구학(Inverse Kinematics):** 로봇의 선속도($v$)와 각속도($\omega$)를 입력받아, 두 바퀴 사이의 거리($L$, `wheel_separation`)를 고려한 **차동 구동 모델** 공식을 통해 좌/우 바퀴의 필요 속도를 계산합니다.
        * $v_{left} = v - \frac{\omega L}{2}, \quad v_{right} = v + \frac{\omega L}{2}$
    2.  **모터 제어:** 계산된 m/s 단위의 속도를 모터 드라이버의 PWM 듀티비로 변환하고, `lgpio` 라이브러리를 이용해 RPi 5의 하드웨어 핀에 신호를 인가합니다.
* **출력 (Output):** 좌우 DC 모터 개별 속도 제어
