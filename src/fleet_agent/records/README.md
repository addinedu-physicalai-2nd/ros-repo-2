# 기록(json) — 모든 로봇 공통

로봇은 역할이 고정이 아니다 (안내 · 서빙 · 픽업을 바꿔 가며 함). 그래서 **모든 로봇이 모든 기록을 갖는다**.
기록은 마커 기준 m · rad 값이라 같은 Pinky Pro끼리 그대로 쓴다. 카메라 보정만 로봇마다 다르다.

```bash
cd ~/ros-repo-2 && git pull
bash src/fleet_agent/records/install_records.sh            # 없는 것만 ~/에 (이미 있는 건 그대로)
bash src/fleet_agent/records/install_records.sh --force    # 레포 값으로 덮어씀 (원래 것은 ~/records_backup_<시각>/)
```

| 파일 | 기능 | 쓰는 코드 | 찍은 로봇 · 명령 |
|---|---|---|---|
| `table_target_id11 · 12 … 61 · 62` | 테이블 T1~T6 정밀 정차 (다리 마커 11~62, 왼 · 오른쪽에서 들어올 때) | `docking/table_dock.py` | pinky1, `table_align --record-lane` |
| `align_n14e` · `align_n14s` | N14 정렬 (마커 4 → 우90 → 마커 5) | `corridor.Steps.n14_align` | pinky2, `dock_record all` |
| `dock_target_n15` · `dock_target_d3` | D3 줄 자세 · D3 주차 (마커 3) + D3 빠지기 | `corridor.park` · `undock.py` | pinky2, `dock_record all` |
| `dock_target_n16` · `dock_target_d2` | D2 줄 자세 · D2 주차 (마커 2) + D2 빠지기 | `corridor.park` · `undock.py` | pinky2, `dock_record dock --dock d2` |
| `align_d1s` | D1 도킹 · 빠지기 (교점 · N1 거리 · 마커 1 · 6) | `docking/entrance.py` | pinky2, `dock_record d1` |
| `dock_target_d1` · `dock_target_d1line` | D1 · 교점 자세 (확인용, 코드는 `align_d1s`만 씀) | — | pinky2, `dock_record d1` |

**로봇별 / 여기 없는 것**
- 카메라 보정 `dock_cam.json`은 로봇마다 다름 (pinky1 531.8, pinky2 513.2) -> `cam/<hostname>.json`에 로봇별로.
  `install_records.sh`가 그 로봇(`hostname`) 것만 `~/dock_cam.json`으로 깜 (없을 때만). 새 로봇은
  `ros2 run fleet_agent marker_dock --calibrate` 한 번 -> `~/dock_cam.json`을 `cam/<hostname>.json`으로 올림
- 바닥 가로선 · E자 테이프: json 없음 (코드 상수 `corridor.py BAR_*`, `line_dock.py DEFAULTS`)

**다른 로봇에 깔았을 때**
- 기록의 `fx`는 찍은 로봇 값이라 "기록 FX != 지금 FX" 주의 로그가 나오는데, 값은 m 단위라 그대로 써도 된다
- 카메라 붙은 각도가 로봇마다 조금 다르면 몇 mm · 몇 ° 차이가 날 수 있다. 처음 한 번은 그 로봇으로
  도킹 · 테이블을 돌려 보고, 크게 어긋나면 그 로봇에서 다시 찍어 레포 값을 바꾸거나 그 로봇 ~/ 파일만 다시 찍는다
- 쓰지 않는 예전 파일 (로봇에 있어도 됨, 레포에 안 올림): `goto_n14/15`, `dock_target_n14`, `align_n14`, `dock_target_id2`,
  `align_n15s · n16s · p2s`, `lane_p1`, `dock_target_d1f`, `dock_logs/*`
