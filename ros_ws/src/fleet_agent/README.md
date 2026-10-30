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
| `fleet_agent/docking/marker_dock.py` | (모듈 + `marker_dock` 실행) | 마커 도킹 v4.3 (`Docker`): dock(D1). 마커 측정 · 카메라 보정 · 목표 저장 (corridor도 이 측정을 씀) |
| `fleet_agent/docking/line_dock.py` | (모듈) | tape: P1 E자 테이프로 바구니를 픽업대에 · tape_exit: 새 선까지 빼기 (바닥 IR 3개) |
| `fleet_agent/docking/undock.py` | (모듈) | undock: 통로 쪽 도크에서 마커로 N15/N16까지 + 좌90 (그다음 통로 넘겨받기, 방향은 라이다 벽) |
| `fleet_agent/docking/marker_align.py` | (`marker_align` 실행) | 정렬 기준 저장 (`--record`) · 시험 |
| `fleet_agent/docking/corridor.py` | (모듈) | 주방 통로: N14 정렬(마커 4 → 우90 → 5), 라이다 벽 따라 후진 + 바닥 가로선(N15 · N16)마다 진행 알림, 도크 후면 주차 |
| `fleet_agent/docking/entrance.py` | (모듈) | 입구 D1 (안내 로봇 자리): N1 → 통로 가운데 → 마커 6 · 1 교점 → 곧게 후진 / 빠지기: 교점에서 북쪽 |
| `fleet_agent/docking/dock_record.py` | (`dock_record` 실행) | 통로 · 도크 기록을 로봇이 직접 (`all` / `dock` / `d1` / `align` / `check` / `scan`) |

## 노드 사이 인터페이스 (로봇 안, std_msgs/String JSON)

| 토픽 | 방향 | 내용 |
|---|---|---|
| `/docking/request` | agent → docking | `{"id", "node", "method", ...precision_stops.yaml 값}` |
| `/docking/result` | docking → agent | `{"id", "node", "ok", "detail"}` (`timeout_sec` 안에 못 하면 ok + "timeout"). corridor는 `reached` · `then_done` |
| `/docking/cancel` | agent → docking | `{"id"}` (정지 · 취소 때) |
| `/docking/progress` | docking → agent | `{"id", "node"}` 통로 docking이 노드를 지남 (가로선 · 거리) → `NODE_REACHED` |
| `/docking/extend` | agent → docking | `{"id", "nodes", "then"}` 통로 docking 중 서버 APPEND로 길이 늘어남 |
| `/obstacle/direction` | agent → obstacle | `FORWARD` / `REVERSE` / `STOP` (어느 쪽을 볼지) |
| `/obstacle/status` | obstacle → agent | `{"blocked", "kind", "distance", "direction"}` |
| `/lcd/text` | agent → LCD 드라이버 | 띄울 문구 (카운트다운 포함) |

## 주행 규칙 (Driver)

- Move의 waypoint를 줄(StepQueue)에 쌓는다. `REPLACE`면 비우고 새로, `APPEND`면 같은 cmd_id일 때만 뒤에
- `FORWARD`는 Nav2로 간다. 중간 노드는 `pass_tolerance` 안에 들어오면 바로 다음 목표로 넘어간다 (위치만 통과)
- `REVERSE`는 `/cmd_vel` 직선 후진 (방향 P제어). 카메라가 못 보므로 장애물은 LiDAR만
- 앞 Move와 drive가 다르면 그 노드에서 멈추고 진행 방향으로 제자리 회전한 뒤 출발 (후진이면 뒤가 진행 방향)
- 노드마다 `NODE_REACHED`. `align` 노드(N14)는 지날 때 docking에 맞춤을 맡기고 이어 간다
  (`hold_axis`면 맞춘 방향이 이어지는 후진의 축. 출발 회전 생략)
- **통로 넘겨받기** (`corridor_handover: true`): 축이 정해진 뒤의 후진은 docking에 맡긴다 (`corridor_approach`, method `corridor`).
  docking이 바닥 가로선(N15 · N16)을 밟거나 거리(P2)로 노드를 지날 때마다 `/docking/progress` → 그 노드 `NODE_REACHED` (안 섬)
  → 서버가 다음 노드를 `APPEND` → `/docking/extend`로 달리는 docking에 이어 붙임. 도착지가 `tape` / `dock_in`이면 `then`으로 같이.
  이어 붙이기가 늦어 docking이 먼저 섰으면(`reached`) 못 간 노드는 큐로 돌려 그 자리에서 다시 넘겨받는다
  docking이 실패하거나 시간 초과(`ok` + `timeout`)면 도착으로 치지 않는다: progress로 지난 노드까지만 `NODE_REACHED`,
  `CMD_FAILED reason="corridor_failed:<맡긴 끝 노드>:<이유>"` -> 서버 고장 처리 (lock 유지). 재개하면 남은 노드로 다시 맡김
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
| `dock` | (예전 방식) | `marker_dock.Docker.dock()` (마커 `marker`의 목표 파일로) → `back` m 후진 |
| `entrance` | D1 | N1에서 D1 쪽을 보고 통째로 (`entrance.Entrance.dock`). leave `entrance_exit`: 교점까지 나와 북쪽 → Nav2로 N1 |
| `align` | N14 | `front`/`axis`가 있으면 `corridor.Steps.n14_align` (마커 4 → 우90 → 마커 5 벽면 수직). 없으면 예전 `markers` 방식 |
| `tape` | P1 | `line_dock.LineDocker.run()` — E자 테이프: 가로선 → X → 시계 90° → 동 선까지 후진. 통로로 오면 `corridor`의 `then` |
| `dock_in` | D2 · D3 (통로 쪽 도크) | 통로로 오면 `corridor`의 `then`: 도크 줄 가로선(N16 · N15)에서 섬 → `Steps.park_from_line` (우90 → 마커 2(3)으로 줄 자세 → 후면 주차) |
| `corridor` | N14 다음 통로 | `corridor.Steps.follow_bars` — 라이다 양옆 벽과 평행 · 서쪽 벽에서 가운데로 후진, 가로선마다 progress |
| leave `undock` | D2 · D3를 떠날 때 | `undock.undock()` — 마커로 N16/N15까지, 왼쪽 90°. 그 뒤 후진은 통로 docking이 벽으로 (`hold_axis`) |
| leave `tape_exit` | P1을 떠날 때 | `line_dock.LineDocker.leave()` — 앞으로 가며 선을 세서 새 선에서 정지 → Nav2 |

마커 준비 (로봇마다, fleet_agent는 끄고. 카메라를 docking 노드가 쥐고 있어서):
```bash
ros2 run fleet_agent marker_dock --calibrate          # 로봇마다 처음 한 번: 카메라 거리 보정 -> records/cam/<hostname>.json
ros2 run fleet_agent marker_dock --record --id 3      # 도킹 자리마다: 로봇을 최종 위치에 두고 -> ~/dock_target_id3.json
ros2 run fleet_agent marker_dock --dry-run --id 3     # 계획만 보기
```
- 목표 파일은 마커 ID별이다 (D1 `--id 1`). 주방 통로 쪽(N14 · D3 · D2)은 아래 `dock_record`로 기록한다
- 도킹 로그 · 사진은 `~/dock_logs/`에 도킹마다 남는다

## 주방 통로 · 픽업 · 도크 (N13 → N14 → N15/N16 → P2 → P1 → N4)

```
N13 ─Nav2─▶ N14 ─align─▶ (축) ─통로 docking: 라이다 벽 따라 후진─▶ N15 선 ─▶ N16 선 ─▶ P2(거리) ─▶ P1 tape ─ tape_exit ─Nav2─▶ N4
            마커 4 → 우90 → 마커 5         (선마다 NODE_REACHED, 안 섬)
                                           ├─▶ (도착지 D3) N15 선에서 섬 → 우90 → 마커 3 → 후면 주차
                                           └─▶ (도착지 D2) N16 선에서 섬 → 우90 → 마커 2 → 후면 주차
D3 ──undock(마커 3 → 좌90)──▶ N15 (선 위) ──통로 docking──▶ N16 ─▶ P2 ─▶ P1 ...      (D2도 같음: 마커 2 → N16)
```
- 서버 명령은 그대로다 (`lock_lookahead: 2`): `FORWARD [N14]` + `REVERSE [N15]` → N14 도착 때 `APPEND [N16]`
  → N15 선에서 `APPEND [P2]` → N16 선에서 `APPEND [P1]`. 로봇은 N14에서만 서고 P1까지 멈추지 않는다
- 바닥 가로선: N15 · N16에 짧은 검정 테이프 (로봇 폭 정도, 진행 방향에 수직). 도크에서 곧게 나와 좌90 한 자세
  (`dock_record` 끝 자세)에서 바닥 센서 밑에 붙임
  - 선 = 센서 3개 모두 3800 넘음이 연속 2번. 통로 바닥이 약 20 cm마다 한 번씩 3개 모두 ~3500으로 튀어서 (바닥 기록 실측)
  - 노드 사이 거리 (`corridor.py BAR_GAP`): N14→N15 40 cm, N15→N16 15 cm (실측), N16→P2 48 cm, P2→P1 44 cm
  - 예상의 절반도 안 갔는데 나온 선은 무시, 예상 + 10 cm까지 안 나오면 거리로 그 노드를 침 (로그 `!! 선 못 봄`)
- 후진 방향은 **양옆 벽(라이다)과 평행**, 옆은 서쪽 칸막이에서 17.3 cm(통로 가운데). 실측: 마커 5 벽에 수직인 축이
  통로와 약 3.8° 틀어져 있고 odom도 1 m에 3~4° 휨. 후진 중 마커 방위각으로는 방향을 고치지 않는다
- 도크 들어가기: 도크 줄 선에서 서고 우90 → 마커로 줄 기록 자세(n15 / n16)에 맞춤 (옆 8 mm 넘으면 옆걸음)
  → 1초에 15번 마커를 보며 칸 줄을 따라 후진 (최대 5°) → 정답(d3 / d2 기록)과 비교해 앞뒤 마무리
- 바닥 센서: `pinky_sensor_adc main_node` (robot_system이 `use_ir:=true`로 켬). 바닥 230~250, 테이프 4050~4095

E자 테이프 (지도 방향, 북쪽 위 · 픽업대 오른쪽)
```
  새 선   서 선  동 선
   ┃      ┃      ┃       세로선 3개 (사진상 간격 새~서 약 14 cm, 서~동 약 7 cm)
   ┗━━━━━━┻━━━━━━┛       가로선 = 검검검 (메인선은 1~2개만 검정이라 착각 안 함)
          ┃              메인선 (복도, 있어도 됨)
```

준비 (로봇마다 한 번, fleet_agent는 끄고 bringup + 바닥 센서만)
```bash
ros2 run fleet_agent marker_dock --calibrate     # 카메라 보정 (로봇마다 처음 한 번) -> records/cam/<hostname>.json
ros2 run fleet_agent dock_record all             # 로봇을 D3 칸에 손으로 반듯하게 넣고: 로봇이 직접
                                                 #   d3 · n15 · n15s · n14s · n14e 를 벽과 수직으로 맞추고 찍음
ros2 run fleet_agent dock_record dock --dock d2  # D2 칸에 손으로: d2 · n16 · n16s. 끝 자세(남쪽)에서 N16 가로선을 센서 밑에
ros2 run fleet_agent dock_record check           # 확인 (n15와 d3 차이 0이어야 함, --dock d2도)
ros2 run fleet_agent dock_record scan            # (필요하면) 지금 자리에서 라이다로 양옆 벽이 보이나
```
기록 파일은 레포 `ros_ws/src/fleet_agent/records/`에 저장되고 모든 로봇이 거기서 읽는다 (`records/README.md`, 형식: `fleet_msgs/docs/record_json_format.md`).
다시 찍으면 `git status`에 바뀐 json이 보이니 커밋 · push. 마커를 다시 붙이거나 벽을 옮기면 다시 기록한다.

켜기
```bash
ros2 launch fleet_agent robot_system.launch.xml robot_id:=pinky2 start_node:=D3 \
  use_docking:=true precision_stops:=$(ros2 pkg prefix fleet_msgs)/share/fleet_msgs/docs/precision_stops.yaml
```
값 바꾸기: `precision_stops.yaml`의 `N14`(기록 이름 · 회전), `D2`/`D3`(마커 · 기록 이름 · timeout · leave), `P1`.
그 밖의 기본값은 `docking/corridor.py` 맨 위 (가로선 문턱 · 노드 사이 거리 · 벽 따라가기 · 후면 주차),
`docking/line_dock.py`의 `DEFAULTS` (테이프), `docking/undock.py` 맨 위.
통로 넘겨받기를 끄려면 `fleet_agent.yaml`의 `corridor_handover: false` (예전 odom 축 후진).

## 입구 D1 (안내 로봇 자리, docking/entrance.py)

```
N2 ─Nav2─▶ N1 ─(D1 쪽을 봄)─ entrance: 통로 가운데로 남쪽 → 칸막이 시작 + 기록 거리 → 마커 6 거리
                             → 우90, 앞 벽(라이다) 수직 → 마커 1 = 교점 기록 → 곧게 후진, 마커 1 거리 = D1 ─▶ DOCKED · CMD_DONE
D1 ─ entrance_exit: 곧게 앞으로 교점 → 마커 1 = 기록 → 우90 북쪽 (통로 벽 평행) ─▶ UNDOCKED ─▶ (안내면 여기서 인사) ─Nav2─▶ N1 ...
```
- 교점 = D1에서 곧게 나온 줄 × 남쪽을 볼 때 마커 6이 정면인 줄. 서쪽을 볼 때 방향은 앞 벽을 라이다로 직선 맞춤
  (odom 90° 회전이 몇 ° 틀려서, 마커 1을 정면에 둬도 로봇 중심이 2 cm 비껴 섰음), 남 · 북을 볼 때는 양옆 벽
- 서버 명령은 그대로 (`FORWARD [.. N1]` + `REVERSE [D1]`). Driver가 N1에서 뒤로 돌지 않고 D1 쪽을 보고 맡긴다 (`from: N1`)
- 안내 Move를 D1에서 처음 받으면 빠지기(entrance_exit)를 먼저 하고, 빠진 자리(북쪽을 봄)에서 손님 인사
- 기록 (fleet_agent 끄고 bringup만): D1 칸에 손으로 반듯하게 (앞 = 마커 1, 바구니 = 칸막이)
```bash
ros2 run fleet_agent dock_record d1            # D1 · 교점(마커 1 · 6) · N1 -> ~/align_d1s.json 등. 끝: N1에서 남쪽
ros2 run fleet_agent dock_record check --dock d1
```

## 도킹 알림 (fleet_msgs v10)

dock / dock_in(D2 · D3) / entrance(D1) 정밀 정차가 끝나면 `RobotEvent.DOCKED`(node = 도크, reason = 실패면 이유)를
CMD_DONE 바로 앞에, undock / entrance_exit(빠지기)가 끝나면 `UNDOCKED`(node = 떠난 도크)를 보낸다.
로봇 로그 `===== DOCKED D1 ok =====`, 서버 로그 `FLEET pinky2 event DOCKED D1` + 매장 화면 알림 · 이력(DOCK).

## 위치 맞추기 (Control LOCALIZE, fleet_msgs v11)

시스템 시작 전에 서버가 한 대씩 보낸다 (`main_service/launch/system.launch.xml`). 쉬는 중일 때만 한다 (아니면 `LOCALIZED reason=busy`)
1. 앞으로 `fwd`(기본 0.15 m, `localize_speed` 0.05 m/s) — 도크 벽에서 떨어져 돌 자리 확보
2. 제자리 `spin_deg`(기본 360°) — AMCL이 주변을 다 보고 위치를 맞춤. 회전 · 직진 방향은 odom yaw (AMCL 보정이 안 섞임)
3. 처음 방향으로 맞추고 1초 멈춤 → 같은 거리만큼 뒤로 (거리 = 속도 x 시간, 지도 좌표는 AMCL 보정으로 튀어서 안 씀)
4. `RobotEvent LOCALIZED` (reason 비면 성공)
- 노드별 값: `precision_stops.yaml`의 `<도크>.localize: {fwd: 0.15, spin_deg: 360}` (없으면 `fleet_agent.yaml` 기본값)
- PAUSE면 멈췄다 RESUME에 이어서, CANCEL이면 그만두고 `LOCALIZED reason=cancel`. LiDAR 장애물 정지는 안 씀 (조금만 움직임)

## 한 번에 켜기 (robot_system.launch.xml)

bringup -> (2초) Nav2(맵 pinkytorang_map_v5, 설정 nav_v5: 둘 다 레포) -> 초기 위치 + fleet_agent 를 한 명령으로 켠다.
```bash
export ROS_DOMAIN_ID=52
ros2 launch fleet_agent robot_system.launch.xml robot_id:=pinky2 start_node:=D2
```
- 초기 위치: `initial_pose`가 node_graph.yaml의 start_node 좌표 · yaw로 /initialpose를 보낸다 (RViz 2D Pose Estimate 대신).
  D 좌표가 안 맞으면 `set_initial_pose:=false`로 끄고 RViz로 맞춘다
- 맵 · Nav2 설정 바꾸기: `map:=... params_file:=...`
  (기본 `fleet_msgs/docs/maps/pinkytorang_map_v5.yaml`, `fleet_agent/params/nav_v5.yaml`. 바꾸면 커밋 · push 후 로봇에서 pull + fbuild)
- Nav2에서 tf 에러가 계속 나면 `nav_delay:=5`
- 도킹 없이 N 노드에서 시험: `start_yaw:=<rad>`(로봇 앞 방향), `spin_deg:=360`(초기 위치 뒤 제자리 한 바퀴로 AMCL 맞춤)
  서버는 `main_service_test.yaml`로 켠다 (켤 때 노드에서 대기, 작업 끝나면 거기로 돌아감)

## 맵 · Nav2 설정 (레포)

| 파일 | 내용 |
|---|---|
| `fleet_msgs/docs/maps/pinkytorang_map_v5.pgm/.yaml` | SLAM 맵 2.5 cm (100 × 81칸, 약 2.5 × 2.0 m). 서버 · 로봇 공용. 노드 좌표(`node_graph.yaml`)는 이 맵 기준 |
| `fleet_agent/params/nav_v5.yaml` | **Nav2 기본.** config4 기반 튜닝 (바꾼 줄 `# v5:`): 서는 노드 5 cm · 6°, 라이다 차선 가운데로(inflation 0.15 · 5), 제자리 회전 23° 넘으면, local costmap 1 m 2D, AMCL 파티클 1000 · sigma 0.1, A* |
| `fleet_agent/params/config4.yaml` | 예전 Nav2 설정 (되돌리기용): costmap 2.5 cm, footprint 뒤 -0.14 m(바구니), 라이다 0.15 m 안 무시, 직진 0.11 m/s, inflation 0.12 m |
| `fleet_agent/params/mapper_small.yaml` | 매핑(slam_toolbox) 설정: 2.5 cm, 레이저 4 m, 0.2 m · 0.2 rad마다 스캔 |

다시 매핑 (pinky2, 시작 자리를 정해 두고 같은 자리에서. 바구니는 떼고)
```bash
ros2 launch pinky_bringup bringup_robot.launch.xml                      # 터미널 A
ros2 launch pinky_navigation map_building.launch.xml \
  slam_params_file:=$(ros2 pkg prefix fleet_agent)/share/fleet_agent/params/mapper_small.yaml   # 터미널 B
ros2 run teleop_twist_keyboard teleop_twist_keyboard                   # PC, 천천히 한 바퀴 + 통로
ros2 run nav2_map_server map_saver_cli -f ~/maps/pinkytorang_map_v6    # 저장 -> 레포 fleet_msgs/docs/maps/에 넣고 커밋
```
맵을 다시 만들면 원점 · 방향이 바뀌므로 `node_graph.yaml` 노드 좌표와 서버 키오스크 배치도 좌표도 다시 읽는다.

## 안내 (guide_node) - 손님 확인

서버는 안내 Move에 `guide: true`를 켜고, 모든 Move에 `goal_node`(최종 도착지)를 채운다 (fleet_msgs v9). 나머지는 fleet_agent가 정한다

| 언제 | 로봇 동작 |
|---|---|
| 안내 Move를 처음 받았을 때 (출발 전) | 앞 카메라로 손님 기다림 -> 찾으면 환영 효과 -> 출발 |
| 꺾이는 노드 (방향이 `guide_check_turn_deg` 45도 넘게 바뀜) | 그 노드에서 섬 -> 뒤로 돎 -> 손님 확인 -> 다음 노드 쪽으로 돎 -> 이어 감 |
| `goal_node`에 도착 (안내 자리, 예: T3 안내면 N6) | 테이블 쪽으로 돎 -> 도착 효과 (LCD 얼굴 · LED · 부저) |

- 정밀 정차(도킹 마커 · 테이프)와 나가기는 `goal_node`에 도착했을 때만 한다 (`goal_node`가 비면 예전처럼 waypoints 끝). 안내 중엔 안 한다
- 바로 전 인사 · 확인 자리에서 `guide_check_gap_m`(0.8m) 안의 꺾임은 건너뛴다 (맵이 작아서). 지금 맵 기준 확인 자리:
  T1·T2 = N1 / T3 = N1 / T4 = N1, N3 / T5 = N1, N8 / T6 = N1, N3, N9
- 못 찾으면(`wait_sec` 30초) 그 자리에 멈추고 `CMD_FAILED reason="guest_lost:<노드>:<이유>"` -> 서버가 안내 실패 처리, 로봇은 집으로
- YOLO는 guide 노드(별도 프로세스)에서 확인하는 동안에만 `infer_hz`로 돈다. 주행(fleet_agent)을 막지 않는다
- 켜기: `ros2 launch fleet_agent robot_system.launch.xml ... use_guide:=true`
  - 모델: 저장소 `models/best.pt`(yolov8n, 클래스 toy_person / toy_car / cone)가 빌드 때 `share/fleet_agent/models/`에 같이 설치되고 기본값으로 쓰인다.
    손님 = `class_name: toy_person` (config/fleet_agent.yaml guide 쪽)
  - 다른 모델: `guide_model:=<경로.pt>` (클래스 이름이 다르면 class_name도), 모델 없이 흐름만: `guide_model:=""`
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
ros2 run fleet_agent node_tour --preset hall       # D1 빠지기 -> N1에서 360° (AMCL) -> 홀 N1~N13 + 테이블 모서리 보기 (N3·N6·N9 양쪽)
#   로봇: robot_system start_node:=D1 use_docking:=true precision_stops:=$(ros2 pkg prefix fleet_msgs)/share/fleet_msgs/docs/precision_stops.yaml
#   로그: ~/tour_logs/tour_<시각>.log (Move · 이벤트 · 노드마다 앞뒤/옆 오차 · AMCL) / _trace.csv (0.2초 위치) / _result.csv
ros2 run fleet_agent node_tour --tables            # 테이블 앞 노드(N2~N10)마다 서서 가장 가까운 테이블 모서리(face)를 봄
ros2 run fleet_agent node_tour --tables all        # 그 노드가 맡은 테이블 모두 (N3 = T2, T1)
```
테이블: 노드 T1~T6은 차선 위 점(지나가는 길), 바라볼 테이블 모서리는 `precision_stops.yaml` T의 `face`.
테이블이 도착지면 fleet_agent가 직전 노드(N2 · N3 …)에서 face 쪽을 보고 서서 docking table에 넘긴다 (`core._nav_yaw`).
노드마다 걸린 시간 · 도착 때 위치 오차(TF vs 노드 좌표)를 찍고, 끝나면 표로 정리한다. Ctrl+C = 멈추고 결과 출력.
후진 구간 · 도킹 · 픽업은 목표로 적지 않으면 안 지나간다 (전진 주행만).

## 실제 로봇 시험 (정밀 정차 끔)

지금은 도킹(마커 · 라인)을 빼고 **주문 → P1에서 5초 대기 → 테이블 → 도킹 자리로 복귀**만 한다.
서버 설정 `main_service.yaml`: `auto_ready: true`(주방 버튼 없이 바로 출발), `pickup_timeout_sec: 5`(적재 버튼 없이 5초 뒤 출발)

```bash
# ---- 로봇 (pinky1, ROS_DOMAIN_ID=1) : 도킹 자리(예: D1)에 올려 두고
cd ~/ros-repo-2/ros_ws && colcon build --packages-up-to fleet_agent && source install/setup.bash
# 1) bringup (모터 · LiDAR · odom)    2) Nav2 + 맵 project_map_v2, RViz로 초기 위치를 D1에 맞춤
ros2 launch fleet_agent fleet_agent.launch.xml robot_id:=pinky1 start_node:=D1

# ---- 서버 노트북 (ROS_DOMAIN_ID=0)
cd ~/ros-repo-2/ros_ws && colcon build --packages-up-to pinkytorang_main && source install/setup.bash
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
- 서버가 켜질 때 **쉬는 로봇 자리를 맞춘다** (입구 D1에 안내 1대). 1대만 시험하면 D1에서 켜는 게 편하다.
  켜자마자 움직이는 게 싫으면 서버 설정 `entrance_standby_quiet: 0`, `entrance_standby_busy: 0`
- 처음에는 `reverse_speed`(0.06 m/s)와 Nav2 속도를 낮게 두고 본다

## 빌드 · 실행

```bash
cd ~/ros-repo-2/ros_ws && colcon build --packages-select fleet_msgs fleet_agent && source install/setup.bash

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
