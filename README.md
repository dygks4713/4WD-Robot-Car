# 🏎️ AI-Car: ROS2 Humble Based 4WD Control System

라즈베리 파이 4와 **ROS2 Humble**을 활용하여 구축한 **지능형 4륜 구동 로봇 자동차** 제어 시스템입니다.

---

## 1. 프로젝트 개요 (Introduction)
* **프로젝트명**: ROS2 Humble 4WD RC Car with Real-time Camera Stream
* **주요 목적**: ROS2 미들웨어를 기반으로 노드 간 통신을 활용한 로봇 제어 시스템 구축
* **핵심 기능**: 
    - **Teleop Control**: ROS2 노드 기반의 비차단(Non-blocking) WASD 조종
    - **Vision Streaming**: OpenCV를 활용한 실시간 영상 데이터 처리 및 송출

---

## 2. 기술 스택 (Tech Stack)

### 🤖 Robotics Middleware
> - **ROS2 Version**: **Humble Hawksbill**
> - **Communication**: `rclpy` (Python Client Library) 기반 Topic 통신

### 🛠 Hardware
* **Main Board**: Raspberry Pi 4B
* **Motor Driver**: L9110S (Dual Channel) x 2
* **Camera**: Raspberry Pi Camera Module
* **Power**: XL4015 DC-DC Buck Converter (안정적인 전압 공급)

### 💻 Development Environment
* **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
* **Tools**: WSL 2 (Windows Subsystem for Linux), VS Code Remote-SSH, NoMachine

---

## 3. 주요 기능 및 특징 (Key Features)

### 📡 ROS2 기반 분산 제어 시스템
- **Control Node**: 사용자의 키보드 입력을 `/cmd_vel` 또는 사용자 정의 토픽으로 발행(Publish)합니다.
- **Motor Node**: 발행된 제어 신호를 구독(Subscribe)하여 PWM 방식으로 L9110S 드라이버를 제어합니다.
- **Camera Node**: OpenCV 프레임을 캡처하여 실시간으로 영상 메시지를 전송합니다.



### 🕹️ 비차단(Non-blocking) I/O 처리
`select` 모듈을 활용하여 입력 대기 시간 동안 프로세스가 멈추지 않도록 설계함으로써, 조종과 영상 스트리밍이 지연 없이 동시에 이루어집니다.

---

## 4. 하드웨어 배선 정보 (Pin Mapping)

| Motor Location | Input A (GPIO) | Input B (GPIO) |
| :--- | :---: | :---: |
| **Front Left (FL)** | 17 | 18 (PWM) |
| **Rear Left (RL)** | 27 | 22 |
| **Front Right (FR)** | 23 | 24 |
| **Rear Right (RR)** | 25 | 8 |

---

## 5. 실행 방법 (Usage)

1. **Workspace 설정 및 빌드**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/dygks4713/4WD-Robot-Car.git
   cd ..
   colcon build --symlink-install
   source install/setup.bash

## 6. 📂 프로젝트 구조 (Project Tree)

워크스페이스 내의 패키지 구성은 다음과 같습니다.

```text
.
├── camera_pkg              # 라즈베리 파이: 카메라 영상 발행 패키지
│   ├── camera_pkg
│   │   ├── camera_node.py
│   │   └── __init__.py
│   └── ...
├── camera_view_pkg         # 원격 PC: 영상 수신 및 화면 출력 패키지
│   ├── camera_view_pkg
│   │   ├── view_node.py
│   │   └── __init__.py
│   └── ...
├── my_car_controller       # 라즈베리 파이: 모터 드라이버 제어 패키지
│   ├── my_car_controller
│   │   ├── motor_sub.py
│   │   └── __init__.py
│   └── ...
└── my_teleop               # 원격 PC: 키보드 입력 제어기 패키지
    ├── my_teleop
    │   ├── teleop_node.py
    │   └── __init__.py
    └── ...
```

## 📂 패키지 정보 (Package Summary)

| 패키지명 | 실행 환경 | 주요 역할 | 주요 노드 | 핵심 기술 |
| :--- | :---: | :--- | :--- | :--- |
| **camera_pkg** | Raspberry Pi | 실시간 영상 발행 | camera_node.py | OpenCV, CvBridge, V4L2 |
| **camera_view_pkg** | PC (Ubuntu) | 영상 수신 및 출력 | view_node.py | OpenCV GUI, CvBridge |
| **my_teleop** | PC (Ubuntu) | 키보드 제어 명령 발행 | teleop_node.py | geometry_msgs/Twist |
| **my_car_controller** | Raspberry Pi | 모터 드라이버 제어 | motor_sub.py | RPi.GPIO, Subscriber |

---

## 🛠️ 상세 설명

### 1. camera_pkg
- **목적**: 로봇의 시각 데이터를 네트워크로 전송합니다.
- **설정**: 2.2A 배터리 환경을 고려하여 **320x240 해상도**와 **20 FPS**로 최적화되었습니다.
- **노드**: `camera_node.py`

### 2. camera_view_pkg
- **목적**: 수신된 데이터를 사용자 화면에 팝업합니다.
- **기능**: 'q' 키를 눌러 안전하게 창을 닫고 노드를 종료할 수 있습니다.
- **노드**: `view_node.py`

### 3. my_teleop & my_car_controller
- **원격 조작**: PC에서 보낸 속도 명령(`cmd_vel`)을 라즈베리 파이가 구독하여 실제 바퀴를 구동합니다.
- **통신 방식**: ROS2의 Standard 메시지 형식을 준수합니다.

---

## 📡 네트워크 및 환경 설정

> [!IMPORTANT]
> **VirtualBox 네트워크 설정**
> - 반드시 **어댑터에 브리지(Bridged Adapter)** 모드를 사용해야 합니다.
> - 가상 머신과 라즈베리 파이는 동일한 핫스팟/공유기 망에 연결되어야 합니다.

- **ROS_DOMAIN_ID**: `30` (통신 채널 일치 필요)
---

## 🚀 실행 방법 (Quick Start)

**라즈베리 파이 (SBC)**
```bash
ros2 run camera_pkg camera_node
ros2 run my_car_controller motor_sub
