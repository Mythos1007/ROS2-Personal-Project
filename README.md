# TurtleBot3 미로 자율탐색 시스템

## 프로젝트 개요

1m 격자 기반 DFS(깊이 우선 탐색) 알고리즘을 사용하여 TurtleBot3가 Gazebo 시뮬레이터 내 미로를 자율적으로 탐색하고, 벽 기반 지도를 생성·시각화하는 ROS2 시스템입니다.

### 핵심 특징

- **1m 격자 기반 탐색**: 정수 좌표 교차점에서만 판단 및 이동
- **DFS 알고리즘**: 미해결 분기점 추적 및 백트래킹을 통한 완전 탐색
- **벽 기반 지도 생성**: LiDAR 스캔 데이터로 벽 세그먼트 추적 (SLAM 아님)
- **히스테리시스 필터링**: 노이즈에 강건한 개방/폐쇄 판정
- **PySide6 GUI**: 실시간 상태 모니터링 및 제어 인터페이스
- **목표 도달 검증**: 지정 목표까지의 경로 탐색 및 도달 확인

---

## 시스템 구조

### 1. 상태 기계 (FSM)

자율주행은 다음 상태들로 구성된 유한 상태 기계로 동작합니다:

| 상태 | 설명 |
|------|------|
| **IDLE** | 초기 상태, 가까운 교차점으로 스냅 준비 |
| **SNAP** | 가장 가까운 정수 좌표 교차점으로 이동 |
| **SCAN** | LiDAR로 4방향(전/후/좌/우) 스캔 및 hiDAR로 4방향(전/후/좌/우) 스캔 및 히스테리시스 적용 |
| **TURN** | 선택된 방향으로 90도 또는 180도 회전 |
| **DRIVE** | 다음 교차점까지 1m 전진 |
| **SUCCESS** | 목표 도달, 미션 완료 |

### 2. DFS 탐색 로직

#### 핵심 자료구조

```python
_discovered[node]  # 각 노드의 열린 출구 집합 (set)
_tried[node]       # 이미 시도한 출구 집합 (set)
_parent[node]      # 백트래킹용 부모 노드
_parent_dir[node]  # 부모로 돌아가는 방향
```

#### 탐색 전략

1. **전진 탐색**: `(discovered - tried)` 중 목표에 가까워지는 방향 우선
2. **백트래킹**: 미탐색 출구가 없으면 가장 가까운 미해결 분기점으로 복귀
3. **탐색 완료**: 모든 노드의 모든 출구를 시도하면 종료

#### 히스테리시스 필터

```python
OPEN_THRESH  = 0.70m  # 열림 판정 거리
CLOSE_THRESH = 0.55m  # 닫힘 판정 거리
```

이전 상태가 "열림"이었으면 0.55m 이하로 떨어져야 "닫힘"으로 전환되어 노이즈를 억제합니다.

### 3. 벽 지도 생성 (WallMapBuilder)

#### 동작 원리

- **교차점 스캔**: 각 노드에서 4방향 스캔 수행
- **벽 세그먼트 추가**: 닫힌 방향에 대해 벽 세그먼트 기록
- **경로 추적**: 이동 경로를 회색 선으로 오버레이 표시
- **PNG 렌더링**: matplotlib로 최종 지도 이미지 생성

#### 좌표 시스템

- 각 벽은 노드 중심에서 ±0.5m 위치에 배치
- 방향: 0=북(+y), 1=동(+x), 2=남(-y), 3=서(-x)

### 4. GUI 시스템

#### 모드

- **MANUAL**: 키보드/버튼으로 직접 제어
- **AUTO**: 자율 탐색 모드

#### 주요 기능

- 실시간 로봇 위치 표시 (X, Y)
- 전방 장애물 거리 표시
- 네비게이션 로그 출력
- 목표 좌표 설정 및 전송
- 탐색 완료 시 지도 팝업 표시

  
![IuWsR5XF](https://github.com/user-attachments/assets/1d2922fb-3f18-4141-a178-948451630262)

---

## 파일 구조

```
turtlebot_gui_control/
├── turtlebot_gui_control/
│   ├── auto_control.py         # 자율주행 메인 로직 (FSM + DFS)
│   ├── wall_map.py              # 벽 지도 생성 및 렌더링
│   ├── detect_obstacle.py       # LiDAR 기반 장애물 감지
│   ├── manual_control.py        # 수동 제어 노드
│   ├── turtle_pose.py           # 오도메트리 모니터링
│   ├── turtlebot_gui_qt.py      # PySide6 GUI 메인 윈도우
│   └── turtlebot_gui_ui.py      # Qt UI 정의 파일
│
├── launch/
│   └── maze_with_turtlebot.launch.py  # Gazebo + 로봇 통합 런처
│
├── make_maze.py                 # 랜덤 미로 생성 스크립트 (DFS 기반)
├── maze.sdf                     # Gazebo 미로 월드 파일
└── README.md                    # 본 문서
```

---

## 패키지 생성 (처음부터 시작)

### 새 ROS2 패키지 생성

```bash
# 워크스페이스 생성 및 이동
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Python 패키지 생성
ros2 pkg create turtlebot_gui_control \
  --build-type ament_python \
  --dependencies rclpy geometry_msgs nav_msgs sensor_msgs std_msgs

# 커스텀 인터페이스 패키지 생성
ros2 pkg create turtlebot_gui_interfaces \
  --build-type ament_cmake \
  --dependencies builtin_interfaces rosidl_default_generators
```

### 소스코드 가져오기

이 리포지토리를 직접 클론하여 전체 패키지를 가져옵니다:

```bash
cd ~/robot_ws/src
git clone https://github.com/Mythos1007/ROS2-Personal-Project.git
cd ROS2-Personal-Project

# 패키지들을 src 디렉토리로 이동
mv turtlebot_gui_control turtlebot_gui_interfaces ../
cd ..
rm -rf ROS2-Personal-Project
```

또는 처음부터 패키지를 생성하고 싶다면:

```bash
# Python 패키지 생성
ros2 pkg create turtlebot_gui_control \
  --build-type ament_python \
  --dependencies rclpy geometry_msgs nav_msgs sensor_msgs std_msgs

# 커스텀 인터페이스 패키지 생성
ros2 pkg create turtlebot_gui_interfaces \
  --build-type ament_cmake \
  --dependencies builtin_interfaces rosidl_default_generators

# GitHub에서 소스 파일만 다운로드하여 각 패키지에 넣기
```

### setup.py 설정

`setup.py`를 다음과 같이 수정:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot_gui_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name),
            glob('*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='TurtleBot3 maze exploration with DFS',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_control = turtlebot_gui_control.auto_control:main',
            'manual_control = turtlebot_gui_control.manual_control:main',
            'detect_obstacle = turtlebot_gui_control.detect_obstacle:main',
            'turtle_pose = turtlebot_gui_control.turtle_pose:main',
            'turtlebot_gui_qt = turtlebot_gui_control.turtlebot_gui_qt:main',
        ],
    },
)
```

### package.xml 의존성 추가

`package.xml`에 다음 의존성 추가:

```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>sensor_msgs</depend>
<depend>std_msgs</depend>
<depend>turtlebot_gui_interfaces</depend>

<exec_depend>gazebo_ros_pkgs</exec_depend>
<exec_depend>turtlebot3_gazebo</exec_depend>
```

### 커스텀 메시지/서비스 정의

`~/robot_ws/src/turtlebot_gui_interfaces/msg/` 생성:

```bash
mkdir -p ~/robot_ws/src/turtlebot_gui_interfaces/msg
mkdir -p ~/robot_ws/src/turtlebot_gui_interfaces/srv
```

#### ObstacleDistance.msg
```
float64 distance
```

#### GoalPose.msg
```
float64 x
float64 y
```

#### StartStop.srv
```
bool start
---
bool accepted
string message
```

### CMakeLists.txt 설정 (turtlebot_gui_interfaces)

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleDistance.msg"
  "msg/GoalPose.msg"
  "srv/StartStop.srv"
  DEPENDENCIES builtin_interfaces
)
```

### PySide6 설치

```bash
pip3 install PySide6
```

### 빌드

```bash
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 실행 방법

### 환경 요구사항

- **ROS2**: Humble Hawksbill
- **Gazebo**: Classic 11
- **Python**: 3.10+
- **TurtleBot3 패키지**: `ros-humble-turtlebot3*`
- **matplotlib**: 지도 렌더링용

### 필수 패키지 설치

```bash
# TurtleBot3 패키지
sudo apt install ros-humble-turtlebot3-*

# matplotlib
pip3 install matplotlib

# 워크스페이스 빌드
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash

# TurtleBot3 모델 설정 (bash 영구 적용)
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

### 1. 시뮬레이션 시작

```bash
# 터미널 1: Gazebo 미로 환경 실행
cd ~/robot_ws/src/turtlebot_gui_control
ros2 launch launch/maze_with_turtlebot.launch.py
```

### 2. GUI 실행

```bash
# 터미널 2: GUI 실행
source ~/robot_ws/install/setup.bash
ros2 run turtlebot_gui_control turtlebot_gui_qt
```

### 3. 자율 탐색 시작

1. GUI 로그인 (기본값: ID=`admin`, PW=`1234` / 변경은 `turtlebot_gui_qt.py`의 `EXPECTED_LOGIN_ID`, `EXPECTED_LOGIN_PW` 참조)
2. `AUTO` 버튼 클릭하여 자율 모드 전환
3. 목표 좌표 입력 (예: `X=3.0, Y=4.0`)
4. `NAV START` 버튼 클릭
5. 탐색 완료 후 지도 이미지 자동 표시

> **참고**: 로그인 인증 정보는 `turtlebot_gui_qt.py` 파일의 `EXPECTED_LOGIN_ID`와 `EXPECTED_LOGIN_PW` 변수에서 변경 가능합니다.

---

## 주요 파라미터

### auto_control.py 상수

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| `GRID_STEP` | 1.0m | 격자 간격 |
| `INTERSECTION_OPEN_THRESH` | 0.70m | 출구 열림 판정 거리 |
| `INTERSECTION_CLOSE_THRESH` | 0.55m | 히스테리시스 닫힘 거리 |
| `POSITION_TOLERANCE` | 0.10m | 교차점 도달 허용 오차 |
| `YAW_TOLERANCE` | 0.05 rad | 회전 완료 허용 오차 (~2.9°) |
| `V_DRIVE` | 0.22 m/s | 직진 속도 |
| `W_TURN` | 0.50 rad/s | 회전 각속도 |
| `SECTOR_PERCENTILE` | 50% | LiDAR 섹터 대표값 (중앙값) |

### LiDAR 섹터 각도

| 방향 | 각도 범위 | 용도 |
|------|-----------|------|
| 전방 | -20° ~ +20° | 주행 장애물 감지 |
| 좌측 | 45° ~ 135° | 교차점 출구 판정 |
| 우측 | -135° ~ -45° | 교차점 출구 판정 |
| 후방 | 150° ~ 180°, -180° ~ -150° | Recovery 모드 전용 |

---

## 지도 생성 과정

### 1. 스캔 단계

각 교차점에서 4방향을 스캔하여 개방/폐쇄 상태를 판별:

```
노드 (2, 3) 스캔 결과:
  - 북: 0.85m → OPEN
  - 동: 0.42m → CLOSED (벽)
  - 남: 1.20m → OPEN
  - 서: 0.38m → CLOSED (벽)
```

### 2. 벽 세그먼트 저장

`WallMapBuilder.update_from_scan()`이 닫힌 방향에 대해 벽 세그먼트를 set에 추가:

```python
# 동쪽이 닫혔으면
wall_segment = ((2.5, 2.5), (2.5, 3.5))  # 노드 중심 +0.5m
```

### 3. PNG 렌더링

탐색 완료 시 `~/.ros/maze_walls.png`에 지도 저장:

- **검은 굵은 선**: 벽 세그먼트
- **회색 얇은 선**: 이동 경로
- **파란 점**: 시작 노드
- **빨간 점**: 목표 노드

### 4. GUI 표시

`/mission_event` 토픽으로 이벤트를 수신하면 GUI가 세 번째 페이지에 지도 이미지를 표시합니다.

---

## 미로 생성

https://github.com/Mythos1007/Gazebo_Maze

### 랜덤 미로 생성

```bash
cd ~/robot_ws/src/turtlebot_gui_control

# 기본 5x5 미로
python3 make_maze.py

# 사용자 정의 미로
python3 make_maze.py -s 7 -m 15 -o custom_maze.sdf --seed 42
```

### 옵션

- `-s SIZE`: 그리드 크기 (기본값: 5)
- `-m MIN_WALLS`: 최소 내부 벽 개수 (기본값: 5)
- `-o OUTPUT`: 출력 파일명 (기본값: maze.sdf)
- `--seed SEED`: 랜덤 시드 (재현 가능한 미로)

생성된 미로는 DFS로 연결성이 보장되며, 모든 영역에 도달 가능합니다.

---

## 개선 가능성

### 단기 개선

1. **동적 장애물 회피**: 정적 미로 외 이동 장애물 대응
2. **A* 경로 계획**: DFS 대신 최단 경로 알고리즘 적용
3. **멀티플 목표**: 여러 목표를 순차 방문하는 TSP 기반 탐색
4. **맵 저장/로드**: 이전 탐색 결과 재사용

### 장기 확장

1. **실제 로봇 적용**: Gazebo → 실제 TurtleBot3 포팅
2. **SLAM 통합**: wall map → occupancy grid SLAM로 전환
3. **다중 로봇 협력**: 여러 로봇이 동시 탐색 및 정보 공유
4. **3D 환경**: 다층 구조 탐색 및 3D 지도 생성
5. **강화학습**: DFS → DRL 기반 적응형 탐색

---

## 문제 해결

### 로봇이 정지하거나 회전만 하는 경우

- **원인**: LiDAR 섹터 설정 불일치 또는 노이즈
- **해결**: `SECTOR_PERCENTILE` 값 조정 또는 `SCAN_STABILIZE_TIME` 증가

### 지도가 표시되지 않는 경우

1. matplotlib 설치 확인: `pip3 list | grep matplotlib`
2. 파일 경로 확인: `ls ~/.ros/maze_walls.png`
3. GUI 콘솔에서 에러 로그 확인

### 백트래킹이 무한 반복되는 경우

- **원인**: 부모 체인에 순환 발생
- **해결**: `_find_unresolved_ancestor()`에서 `visited_chain` 확인 로직 추가됨

---

## 라이선스

이 프로젝트는 교육 및 연구 목적으로 개발되었습니다.

---

## 참고 자료

- [ROS2 Humble 문서](https://docs.ros.org/en/humble/)
- [TurtleBot3 매뉴얼](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Gazebo Classic 문서](https://classic.gazebosim.org/)
- [PySide6 문서](https://doc.qt.io/qtforpython-6/)

---

**개발 환경**: Ubuntu 22.04, ROS2 Humble, Gazebo 11
**테스트 완료**: 5x5 ~ 15x15 미로 환경
