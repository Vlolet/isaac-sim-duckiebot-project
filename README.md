# Isaac Sim 기반 Duckiebot 자율주행 시뮬레이션 프로젝트

**2025-2 Robotics and AI, final project**

---

## 1. 프로젝트 개요

본 프로젝트는 NVIDIA Isaac Sim 환경에서 Duckiebot 로봇을 구현하고, ROS2(Humble)와 연동하여 기본적인 원격 제어 및 센서 데이터를 수신하는 시스템을 구축하는 것을 목표로 합니다. 최종적으로는 영상처리를 통한 자율주행 기능 구현을 지향합니다.

## 2. 기반 환경: Isaac Sim ROS & ROS2 Workspaces

이 저장소는 NVIDIA에서 제공하는 [Isaac Sim ROS & ROS2 Workspaces](https://github.com/NVIDIA-Omniverse/IsaacSim-ROS-Workspaces)를 기반으로 `humble_ws` 내에서 Duckiebot 관련 패키지를 개발한 것입니다. 이로 인해 원본 저장소의 커밋 히스토리가 일부 포함되어 있습니다.

- **[원본 Isaac Sim ROS/ROS2 설치 및 사용법 링크](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_ros.html)**

---

## 3. 시스템 아키텍처

- **시뮬레이터:** NVIDIA Isaac Sim
- **미들웨어:** ROS2 Humble
- **핵심 연결:** Isaac Sim - ROS2 Bridge
- **주요 흐름:**
    1.  **제어:** ROS2 토픽 (`/duckie/cmd_vel`) → Isaac Sim Bridge → 시뮬레이션 내 로봇 구동
    2.  **센서:** 시뮬레이션 내 카메라 → Isaac Sim Bridge → ROS2 토픽 (`/duckie/camera/image_raw`)

## 4. 프로젝트 구조

- **`humble_ws/src/duckiebot_description`**: Duckiebot의 URDF/XACRO 및 3D 메시 파일 포함
- **`humble_ws/src/my_duckie_interfaces`**: 사용자 정의 ROS2 인터페이스 (서비스)
- **`humble_ws/src/my_duckie_nodes`**: 프로젝트에 사용된 ROS2 노드 소스 코드

## 5. 빌드 및 실행 방법

### 사전 요구사항
- NVIDIA Isaac Sim 2022.2.1 이상 설치
- ROS2 Humble 설치 (WSL2 또는 Native Ubuntu)
- Isaac Sim 내 ROS2 Bridge 확장 기능 활성화

### 실행 순서
1.  **ROS2 워크스페이스 빌드:**
    ```bash
    # 저장소의 루트 디렉토리에서 실행
    cd humble_ws
    colcon build
    source install/setup.bash
    ```

2.  **Isaac Sim 실행:**
    - Isaac Sim을 실행하고 Duckiebot 관련 `usd` 파일을 로드합니다.
    - Action Graph가 포함된 경우, `Play` 버튼을 눌러 시뮬레이션을 시작합니다.

3.  **ROS2 노드 실행:**
    - 새 터미널을 열고 워크스페이스를 source 한 뒤, 각 기능에 맞는 ROS2 노드를 실행합니다.
      ```bash
      # 예시: LED 제어 서비스 노드 실행
      ros2 run my_duckie_nodes led_service_bridge
      ```

## 6. 구현된 기능

- **Duckiebot 모델링:** URDF 파일을 Isaac Sim으로 임포트하여 로봇 모델을 성공적으로 시뮬레이션 환경에 구현했습니다.
- **고수준 모터 제어:** `/duckie/cmd_vel` (Twist) 토픽을 구독하여 로봇의 선속도와 각속도를 제어하는 기능을 구현했습니다.
- **카메라 영상 퍼블리시:** 시뮬레이션 내 카메라 영상을 `/duckie/camera/image_raw` 토픽으로 실시간 발행하는 기능을 구현했습니다.

## 7. 향후 계획 (미구현 사항)

- **ROS 기반 LED 서비스:** `/duckie_led_control` 서비스를 구현하여 LED 색상을 변경하는 기능
- **압축 이미지 토픽:** `/duckie/camera/image_raw/compressed` 토픽 발행 기능 추가
- **영상처리 기반 자율주행:** 빨간 큐브를 인식하고 추적하는 자율주행 알고리즘 노드 구현