# Obserbot Move 패키지

이 패키지는 Obserbot 로봇의 다리 움직임을 제어하기 위한 ROS 패키지입니다.

## 필요 사항

- ROS (Noetic 이상)
- Python 3
- obserbot_ik 패키지 (의존성)

## 설치 방법

1. ROS 워크스페이스의 `src` 디렉토리로 이동:
```bash
cd ~/ros_ws/src
```

2. 패키지 클론 (필요한 경우):
```bash
git clone <repository_url>
```

3. 워크스페이스 빌드:
```bash
cd ~/ros_ws
catkin_make
```

4. 환경 설정 소스:
```bash
source ~/ros_ws/devel/setup.bash
```

## 사용법

### 1. 기본 실행

로봇의 기본 동작을 실행합니다:

```bash
rosrun obserbot_move_pkg obserbot_move_node.py
```

### 2. 특정 움직임 패턴 실행

패키지는 다양한 미리 정의된 움직임 패턴을 지원합니다:

```bash
# 움직임 패턴 1 실행
rosrun obserbot_move_pkg obserbot_move_node.py move [반복횟수]

# 움직임 패턴 2 실행 
rosrun obserbot_move_pkg obserbot_move_node.py move_2 [반복횟수]

# 움직임 패턴 3 실행
rosrun obserbot_move_pkg obserbot_move_node.py move_3 [반복횟수]

# 움직임 패턴 4 실행
rosrun obserbot_move_pkg obserbot_move_node.py move_4 [반복횟수]
```

반복횟수는 선택적 매개변수로, 패턴을 몇 번 반복할지 지정합니다(기본값: 1).

### 3. 초기 자세로 이동

로봇을 초기 자세로 이동시킵니다:

```bash
rosrun obserbot_move_pkg obserbot_move_node.py origen
```

### 4. 런치 파일로 실행

전체 시스템(IK 테스트 노드와 Obserbot 이동 노드)을 동시에 시작하려면:

```bash
roslaunch obserbot_move_pkg obserbot_move.launch
```

## 토픽

이 노드는 다음 토픽을 구독합니다:
- `/ik_rl_target` (Float32MultiArray): 오른쪽 다리 목표 위치
- `/ik_ll_target` (Float32MultiArray): 왼쪽 다리 목표 위치
- `/imu/data` (Imu): IMU 센서 데이터

이 노드는 다음 토픽을 발행합니다:
- `/canopen/multiple_joints` (JointState): 로봇 조인트 명령