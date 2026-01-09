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
   git clone [https://github.com/dygks4713/AI-Car_ROS2.git](https://github.com/dygks4713/AI-Car_ROS2.git)
   cd ..
   colcon build --symlink-install
   source install/setup.bash
