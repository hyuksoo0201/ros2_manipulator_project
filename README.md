# ROS2 Manipulator Project

MyCobot 280 기반의 비전-매니퓰레이션-태스크 통합 ROS 2 워크스페이스입니다.  
`vision`, `manipulator`, `task_manager` 3개 노드가 연동되어 마커 인식 후 Pick-and-Place stack 작업을 수행합니다.

## 1. 구성

- `src/mycobot_system`: 실행 노드(vision/manipulator/task_manager/trigger), launch 파일
- `src/smartfactory_interfaces`: action/srv/msg 인터페이스 정의

주요 실행 노드:
- `manipulator_node`
- `vision_node`
- `task_manager_node`
- `pick_and_place_trigger_node` (옵션)

## 2. 요구 사항

- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy
- Python 패키지:
  - `pymycobot`
  - `numpy`
  - `scipy`
  - `opencv` (aruco 포함 빌드 필요)
- 하드웨어:
  - MyCobot 280 (기본 포트: `/dev/ttyJETCOBOT`)
  - 카메라 (기본 index: `0`)

## 3. 빌드

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 4. 실행

### A. 런치 파일로 3노드 동시 실행

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mycobot_system assembly_task.launch.py
```

### B. 수동 실행(런치 대체)

터미널 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run mycobot_system manipulator_node
```

터미널 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run mycobot_system vision_node
```

터미널 3:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run mycobot_system task_manager_node
```

## 5. Stack 작업 트리거

### 방법 1. 토픽 트리거 (GUI 연동 방식)

```bash
ros2 topic pub --once /jetcobot/assembly/stack/request std_msgs/msg/Int32 "{data: 1}"
```

### 방법 2. 액션 직접 호출

```bash
ros2 action send_goal /task/execute smartfactory_interfaces/action/ExecuteTask "{marker_ids: [1, 2, 3], reset_stack: true}"
```

## 6. 주요 인터페이스

- Action
  - `/task/execute` (`smartfactory_interfaces/action/ExecuteTask`)
  - `/vision/observe_marker` (`smartfactory_interfaces/action/ObserveMarker`)
  - `/manipulator/execute_pnp_action` (`smartfactory_interfaces/action/ExecutePnP`)
- Service
  - `/manipulator/get_current_coords` (`smartfactory_interfaces/srv/GetCurrentCoords`)
  - `/manipulator/execute_pnp` (`smartfactory_interfaces/srv/ExecutePnP`, legacy)
- Topic
  - `/jetcobot/assembly/stack/request` (`std_msgs/msg/Int32`)
  - `/assembly_start` (`std_msgs/msg/Bool`)
  - `/jetcobot/pick_and_place/done` (`std_msgs/msg/Bool`)
  - `/jetcobot/assembly/db_update` (`smartfactory_interfaces/msg/SectionResult`)

## 7. 위치 보정(vision_node)

비전 좌표 보정은 `vision_node`에서 수행됩니다.

- 기본 offset: `offset_x`, `offset_y`, `offset_z`
- 처짐 보정: `sag_coeff`
- 거리 기반 추가 보정: `dist_threshold`, `extra_offset_x`, `extra_offset_z`

필요 시 `vision_node.py`의 파라미터 값을 조정해 튜닝할 수 있습니다.

## 8. 트러블슈팅

- `The passed action type is invalid`
  - 원인: `jetcobot_interfaces` 사용
  - 해결: `smartfactory_interfaces/action/ExecuteTask` 사용

- `ImportError: cannot import name 'PathSubstitution'`
  - 원인: `launch` 패키지 버전 불일치
  - 해결:

```bash
sudo apt update
sudo apt install --only-upgrade -y \
  ros-jazzy-launch ros-jazzy-launch-ros ros-jazzy-ros2launch \
  ros-jazzy-launch-xml ros-jazzy-launch-yaml \
  ros-jazzy-launch-testing ros-jazzy-launch-testing-ros
```

- 로봇 연결 실패
  - `/dev/ttyJETCOBOT` 권한/장치명 확인
  - 케이블/전원 연결 상태 확인

