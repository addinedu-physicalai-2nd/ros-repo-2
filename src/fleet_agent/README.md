# fleet_agent — 로봇 쪽 에이전트 (Python, rclpy)

메인 서비스가 보낸 Move를 받아 노드 단위로 주행하고, 결과를 `fleet/state` · `fleet/event`로 돌려준다.
규칙은 `fleet_msgs/README.md` 5장 (로봇 규칙)을 따른다.

```
            fleet_msgs (domain_bridge)                       로봇 안
 Main  ── /fleet/move · control · display ──▶ ┌──────────────┐ ──▶ Nav2 (navigate_to_pose)   FORWARD
       ◀── /fleet/state · event ───────────── │  fleet_agent │ ──▶ /cmd_vel                  REVERSE · 회전
                                              │  (Driver)    │ ──▶ /lcd/text                 LCD
                                              └──────────────┘
                     /docking/request ▲ ▼ /docking/result        /obstacle/direction ▼ ▲ /obstacle/status
                                 ┌──────────┐                        ┌──────────┐
                                 │ docking  │ ◀── CAM                │ obstacle │ ◀── /scan (LiDAR)
                                 └──────────┘                        └──────────┘
```

| 파일 | 노드 | 하는 일 |
|---|---|---|
| `fleet_agent/core.py` | (ROS 없음) | `Driver` 주행 상태기계, `StepQueue`, `ReverseControl`, `ObstacleClass`, `BlockPolicy` |
| `fleet_agent/agent_node.py` | `fleet_agent` | fleet 토픽 · Nav2 액션 · TF 위치 · LCD · 배터리를 `Driver`에 이어 줌 |
| `fleet_agent/obstacle_node.py` | `obstacle` | 진행 방향 LiDAR 부채꼴로 막힘 판단, 사람(DYNAMIC) / 물건(STATIC) 구분 |
| `fleet_agent/docking_node.py` | `docking` | 정밀 정차 요청을 받아 방법별로 맡김 (스레드에서 실행, 시간 초과 · 취소 처리) |
| `fleet_agent/docking/marker_dock.py` | (모듈 + `marker_dock` 실행) | 마커 도킹 v4.3 (`Docker`): dock(D1~D4) · align(N14 마커 5 → 6). 혼자 실행으로 보정 · 목표 저장 |
| `fleet_agent/docking/line_dock.py` | (모듈) | **구조만.** 라인트레이싱 tape (P1 픽업대 · T1~T6 테이블), 추후 구현 |

## 노드 사이 인터페이스 (로봇 안, std_msgs/String JSON)

| 토픽 | 방향 | 내용 |
|---|---|---|
| `/docking/request` | agent → docking | `{"id", "node", "method", ...precision_stops.yaml 값}` |
| `/docking/result` | docking → agent | `{"id", "node", "ok", "detail"}` (`timeout_sec` 안에 못 하면 ok + "timeout") |
| `/docking/cancel` | agent → docking | `{"id"}` (정지 · 취소 때) |
| `/obstacle/direction` | agent → obstacle | `FORWARD` / `REVERSE` / `STOP` (어느 쪽을 볼지) |
| `/obstacle/status` | obstacle → agent | `{"blocked", "kind", "distance", "direction"}` |
| `/lcd/text` | agent → LCD 드라이버 | 띄울 문구 (카운트다운 포함) |

## 주행 규칙 (Driver)

- Move의 waypoint를 줄(StepQueue)에 쌓는다. `REPLACE`면 비우고 새로, `APPEND`면 같은 cmd_id일 때만 뒤에
- `FORWARD`는 Nav2로 간다. 중간 노드는 `pass_tolerance` 안에 들어오면 바로 다음 목표로 넘어간다 (위치만 통과)
- `REVERSE`는 `/cmd_vel` 직선 후진 (방향 P제어). 카메라가 못 보므로 장애물은 LiDAR만
- 앞 Move와 drive가 다르면 그 노드에서 멈추고 진행 방향으로 제자리 회전한 뒤 출발 (후진이면 뒤가 진행 방향)
- 노드마다 `NODE_REACHED`. `align` 노드(N14)는 지날 때 docking에 맞춤을 맡기고 이어 간다
- 줄이 비면 `done_grace_sec`(0.5초) 기다린 뒤 끝 처리: 후진으로 왔으면 노드 yaw로 회전 → `dock` / `tape`면 docking → `CMD_DONE`
- 장애물 (obstacle `blocked`): 멈추고 `state = BLOCKED`
  - 사람: 3초 기다림 → 5초 LCD "지나갈게요" → 서버에 `BLOCKED` (`reason = "DYNAMIC:N14->N15"`)
  - 물건: 2초 확인 → `BLOCKED` (`reason = "STATIC:..."`)
  - 길이 비면 그 노드부터 다시 출발
- `PAUSE`는 멈춤(줄 유지), `RESUME`은 이어서, `CANCEL`은 줄 비우고 정지
- Nav2 실패면 `CMD_FAILED` + `state = ERROR`. 서버가 RESUME을 보내면 남은 경로를 다시 받는다

## 정밀 정차 (docking)

| method | 노드 | 구현 |
|---|---|---|
| `dock` | D1~D4 | `marker_dock.Docker.dock()` (마커 `marker`의 목표 파일로) → `back` m 후진 |
| `align` | N14 | 같은 `Docker`로 `markers`를 차례로 (마커가 안 보이면 30°씩 돌며 찾기) |
| `tape` | P1, T1~T6 | `line_dock.LineDocker.run()` — 추후 라인트레이싱 (지금은 실패로 돌려주고 Nav2 위치로 진행) |

마커 준비 (로봇마다, fleet_agent는 끄고. 카메라를 docking 노드가 쥐고 있어서):
```bash
ros2 run fleet_agent marker_dock --calibrate          # 처음 한 번: 카메라 거리 보정 -> ~/dock_cam.json
ros2 run fleet_agent marker_dock --record --id 3      # 도킹 자리마다: 로봇을 최종 위치에 두고 -> ~/dock_target_id3.json
ros2 run fleet_agent marker_dock --dry-run --id 3     # 계획만 보기
```
- 목표 파일은 마커 ID별이라, D3 자리에서 `--id 3`, N14에서 `--id 5`(동쪽 보고) · `--id 6`(남쪽 보고)처럼 저장한다
- 도킹 로그 · 사진은 `~/dock_logs/`에 도킹마다 남는다

## 한 번에 켜기 (robot_system.launch.xml)

bringup -> (2초) Nav2(맵 pinkytorang_map_v4, 설정 config4) -> 초기 위치 + fleet_agent 를 한 명령으로 켠다.
```bash
export ROS_DOMAIN_ID=52
ros2 launch fleet_agent robot_system.launch.xml robot_id:=pinky2 start_node:=D2
```
- 초기 위치: `initial_pose`가 node_graph.yaml의 start_node 좌표 · yaw로 /initialpose를 보낸다 (RViz 2D Pose Estimate 대신).
  D 좌표가 안 맞으면 `set_initial_pose:=false`로 끄고 RViz로 맞춘다
- 맵 · Nav2 설정 바꾸기: `map:=... params_file:=...` (기본 ~/maps/pinkytorang_map_v4.yaml, ~/params/config4.yaml)
- Nav2에서 tf 에러가 계속 나면 `nav_delay:=5`
- 도킹 없이 N 노드에서 시험: `start_yaw:=<rad>`(로봇 앞 방향), `spin_deg:=360`(초기 위치 뒤 제자리 한 바퀴로 AMCL 맞춤)
  서버는 `main_service_test.yaml`로 켠다 (켤 때 노드에서 대기, 작업 끝나면 거기로 돌아감)

## 안내 (guide_node) - 손님 확인

서버는 안내 Move에 `guide`만 켜서 보낸다 (fleet_msgs v8 `Move.guide`). 나머지는 fleet_agent가 정한다

| 언제 | 로봇 동작 |
|---|---|
| 안내 Move를 처음 받았을 때 (출발 전) | 앞 카메라로 손님 기다림 -> 찾으면 환영 효과 -> 출발 |
| 꺾이는 노드 (방향이 `guide_check_turn_deg` 45도 넘게 바뀜) | 그 노드에서 섬 -> 뒤로 돎 -> 손님 확인 -> 다음 노드 쪽으로 돎 -> 이어 감 |
| `GUIDE_END` Move 끝 (안내 테이블) | 도착 효과 (LCD 얼굴 · LED · 부저) |

- 바로 전 인사 · 확인 자리에서 `guide_check_gap_m`(0.8m) 안의 꺾임은 건너뛴다 (맵이 작아서). 지금 맵 기준 확인 자리:
  T1·T2 = N1 / T3 = N1 / T4 = N1, N3 / T5 = N1, N8 / T6 = N1, N3, N9
- 못 찾으면(`wait_sec` 30초) 그 자리에 멈추고 `CMD_FAILED reason="guest_lost:<노드>:<이유>"` -> 서버가 안내 실패 처리, 로봇은 집으로
- YOLO는 guide 노드(별도 프로세스)에서 확인하는 동안에만 `infer_hz`로 돈다. 주행(fleet_agent)을 막지 않는다
- 켜기: `ros2 launch fleet_agent robot_system.launch.xml ... use_guide:=true guide_model:=$HOME/models/guest.pt`
  - 카메라 토픽 `camera_topic`(기본 /camera/image_raw)을 내는 노드가 따로 있어야 한다
  - guide 노드를 안 켜면 손님 확인을 건너뛰고 바로 진행, 모델이 없으면 guide 노드가 바로 OK (흐름 시험용)
- 필요: `pip install ultralytics` (로봇 파이썬 환경)

## 노드 순회 시험 (node_tour) - 서버 없이 Nav2 node to node 확인

서버(main_service)는 끄고, 로봇에서 `frun N1 -1.57 360`으로 켠 뒤 다른 터미널에서:
```bash
ros2 run fleet_agent node_tour                     # 기본: 홀 N1 -> N13 지그재그 -> N1 (T 노드도 지나감)
ros2 run fleet_agent node_tour N1 N4 N13 N11 N1    # 목표 노드만 적으면 사이 노드는 길 찾기로 채움
ros2 run fleet_agent node_tour --step 2 N1 N4 N1   # 목표마다 멈춰 2초 쉬고
ros2 run fleet_agent node_tour --loops 3
```
노드마다 걸린 시간 · 도착 때 위치 오차(TF vs 노드 좌표)를 찍고, 끝나면 표로 정리한다. Ctrl+C = 멈추고 결과 출력.
후진 구간 · 도킹 · 픽업은 목표로 적지 않으면 안 지나간다 (전진 주행만).

## 실제 로봇 시험 (정밀 정차 끔)

지금은 도킹(마커 · 라인)을 빼고 **주문 → P1에서 5초 대기 → 테이블 → 도킹 자리로 복귀**만 한다.
서버 설정 `main_service.yaml`: `auto_ready: true`(주방 버튼 없이 바로 출발), `pickup_timeout_sec: 5`(적재 버튼 없이 5초 뒤 출발)

```bash
# ---- 로봇 (pinky1, ROS_DOMAIN_ID=1) : 도킹 자리(예: D1)에 올려 두고
cd ~/ros-repo-2 && colcon build --packages-up-to fleet_agent && source install/setup.bash
# 1) bringup (모터 · LiDAR · odom)    2) Nav2 + 맵 project_map_v2, RViz로 초기 위치를 D1에 맞춤
ros2 launch fleet_agent fleet_agent.launch.xml robot_id:=pinky1 start_node:=D1

# ---- 서버 노트북 (ROS_DOMAIN_ID=0)
cd ~/ros-repo-2 && colcon build --packages-up-to pinkytorang_main && source install/setup.bash
ros2 launch src/fleet_msgs/bridge/fleet_bridge.launch.xml pinky2:=false pinky3:=false
ros2 run pinkytorang_main main_service src/main_service/config/main_service.yaml
# 브라우저: http://<노트북 IP>:8080/c?table=T1 에서 주문 (매장 화면 /store)
```

확인할 로그 (fleet_agent 터미널)
```
Move cmd ... REPLACE FORWARD ['N1', 'N2']     ← 서버 명령이 들어옴
event NODE_REACHED N1 / N2 ...                 ← 노드마다
Move cmd ... APPEND REVERSE ['N15']            ← N14에서 돌아서 후진 시작
event CMD_DONE P1                              ← 픽업 도착 (5초 뒤 서버가 테이블로 보냄)
event CMD_DONE T1                              ← 테이블 도착 (수령 버튼 또는 30초 뒤 도킹 자리로)
```
- 서버가 켜질 때 **쉬는 로봇 자리를 맞춘다** (빈 테이블이 많으면 입구 D1 · D2에 2대). 1대만 시험하면 D1에서 켜는 게 편하다.
  켜자마자 움직이는 게 싫으면 서버 설정 `entrance_standby_quiet: 0`, `entrance_standby_busy: 0`
- 처음에는 `reverse_speed`(0.06 m/s)와 Nav2 속도를 낮게 두고 본다

## 빌드 · 실행

```bash
cd ~/ros-repo-2 && colcon build --packages-select fleet_msgs fleet_agent && source install/setup.bash

# 로봇마다 (ROS_DOMAIN_ID = 1 / 2 / 3). Nav2 · 맵(project_map_v2) · 카메라 · LiDAR는 먼저 띄워 둔다
ros2 launch fleet_agent fleet_agent.launch.xml robot_id:=pinky1 start_node:=D1
```

- `start_node`: 켤 때 서 있는 도킹. 서버는 처음 `last_node`가 도킹이어야 배정한다 (`require_dock_start`)
- 설정: `config/fleet_agent.yaml` (속도 · 허용 오차 · 장애물 거리 · 대기 시간)
- 정밀 정차 노드: `fleet_msgs/docs/precision_stops.yaml` (launch가 자동으로 읽음)
- 로직 테스트 (ROS 없이, 가짜 로봇): `python3 test/test_core.py`

## 로봇에 맞춰 확인할 것

- `/cmd_vel`, `/scan`, `navigate_to_pose` 이름과 TF `map -> base_link`
- `obstacle`의 `scan_yaw_offset` (LiDAR 0도가 로봇 앞인지), 앞 · 뒤 멈춤 거리
- 배터리 토픽 (`battery_topic`, 없으면 `battery_default` 100%)
- LCD 드라이버가 `/lcd/text`를 받게
