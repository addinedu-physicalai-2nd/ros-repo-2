# fleet_graph — 노드 그래프 C++ 라이브러리

메인 서비스(C++)가 `fleet_msgs/docs/node_graph.yaml`을 읽고 경로를 Move 메시지로 바꿀 때 쓰는 라이브러리입니다.
lock 관리, 작업 배정 같은 서버 로직은 들어 있지 않습니다. 메인 서비스에서 가져다 씁니다.

| 기능 | 함수 |
|---|---|
| yaml 읽기 | `fleet_graph::Graph::load(path)` |
| 검사 (없는 노드, 후진 구간 끝 lock 등) | `g.validate()` → 문제 목록 |
| 최단 경로 (일방통행 반영) | `g.shortest_path("N11", "D3")` |
| 경로 → Move 구간 (방향 하나씩, yaw 포함) | `g.to_segments(path)` |

## 빌드

```bash
sudo apt install libyaml-cpp-dev          # 없으면
cd ~/ros2_ws/src && ln -s ~/ros-repo-2/fleet_msgs . && ln -s ~/ros-repo-2/fleet_graph .
cd ~/ros2_ws && colcon build --packages-up-to fleet_graph
source install/setup.bash
```

## 확인 (로봇 없이)

```bash
ros2 run fleet_graph graph_check ~/ros-repo-2/fleet_msgs/docs/node_graph.yaml N11 D3
```

```
경로: N11 N12 N13 N14 N15 N16 D3
Move 1  FORWARD   N12 N13 N14
Move 2  REVERSE   N15 N16          # N14에서 돌아 후진으로 올라감
Move 3  REVERSE   D3               # 도킹은 따로 끊는다 (로봇이 정밀 도킹)
```

## 메인 서비스에서 쓰기

`package.xml`에 `<depend>fleet_graph</depend>`, `<depend>fleet_msgs</depend>`, CMake에:

```cmake
find_package(fleet_graph REQUIRED)
find_package(fleet_msgs REQUIRED)
target_link_libraries(main_service fleet_graph::fleet_graph rclcpp::rclcpp ${fleet_msgs_TARGETS})
```

코드 예시는 `src/example_server.cpp` (토픽 이름, QoS, Move · Display 보내기, Event 받기).

```bash
ros2 run fleet_graph example_server --ros-args -p robot_id:=pinky1 -p from:=N13 -p to:=D3
```
