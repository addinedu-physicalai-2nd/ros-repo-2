# 기록(json) — 레포 한 곳, 모든 로봇이 같이 씀

로봇은 역할이 고정이 아니다 (안내 · 서빙 · 픽업을 바꿔 가며 함). 그래서 기록은 로봇 홈이 아니라 **이 폴더 하나**에 두고,
모든 로봇의 docking 코드가 여기서 바로 읽는다 (`docking/records.py`). 로봇에 따로 깔 것 없음: `git pull` · `fbuild`면 끝.

- 읽는 순서: 레포 소스 `ros_ws/src/fleet_agent/records/` → 설치된 `share/fleet_agent/records/` → `~/` (예전에 홈에 찍은 것)
- 기록 도구(`dock_record` · `marker_dock --record` · `--calibrate` · `marker_align --record`)는 **이 폴더에 바로 저장**
  → 로봇에서 `git status`로 보이고, 커밋 · push하면 다른 로봇도 같은 값
- 레포 위치를 못 찾으면 `export FLEET_RECORDS_DIR=~/ros-repo-2/ros_ws/src/fleet_agent/records`

| 파일 | 기능 | 쓰는 코드 | 찍은 로봇 · 명령 |
|---|---|---|---|
| `cam/<hostname>.json` | **카메라 보정 fx (로봇마다 다름)** — 그 로봇 이름 파일만 읽음 | `marker_dock.load_cam_file` | 로봇마다 `marker_dock --calibrate` (pinky1 531.8, pinky2 513.2) |
| `table_target_id11 · 12 … 61 · 62` | 테이블 T1~T6 정밀 정차 (다리 마커 11~62) | `docking/table_dock.py` | pinky1, `table_align --record-lane` |
| `align_n14e` · `align_n14s` | N14 정렬 (마커 4 → 우90 → 마커 5) | `corridor.Steps.n14_align` | pinky2, `dock_record all` |
| `dock_target_n15` · `dock_target_d3` | D3 줄 자세 · D3 주차 (마커 3) + D3 빠지기 | `corridor.park` · `undock.py` | pinky2, `dock_record all` |
| `dock_target_n16` · `dock_target_d2` | D2 줄 자세 · D2 주차 (마커 2) + D2 빠지기 | `corridor.park` · `undock.py` | pinky2, `dock_record dock --dock d2` |
| `align_d1s` | D1 도킹 · 빠지기 (교점 · N1 거리 · 마커 1 · 6) | `docking/entrance.py` | pinky2, `dock_record d1` |
| `dock_target_d1` · `dock_target_d1line` | D1 · 교점 자세 (확인용, 코드는 `align_d1s`만 씀) | — | pinky2, `dock_record d1` |

**새 로봇 (예: pinky3)**: `ros2 run fleet_agent marker_dock --calibrate` 한 번 → `records/cam/pinky3.json`이 생김 → 커밋 · push.
나머지 기록은 그대로 씀 (마커 기준 m · rad 값이라 같은 Pinky Pro끼리 공통). 처음 한 번 테이블 · D1~D3을 돌려 보고
크게 어긋나면 카메라 붙은 각도 차이 -> 그 로봇에서 다시 찍어 비교.

**기록에 없는 것**: 바닥 가로선 · E자 테이프 문턱 (코드 상수 `corridor.py BAR_*`, `line_dock.py DEFAULTS`).
쓰지 않는 예전 파일 (로봇 홈에 있어도 됨): `goto_n14/15`, `dock_target_n14`, `align_n14`, `dock_target_id2`,
`align_n15s · n16s · p2s`, `lane_p1`, `dock_target_d1f`, `dock_logs/*`
