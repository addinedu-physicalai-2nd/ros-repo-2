# pinkytorang_gz: 핑키토랑 Gazebo 가상 환경

실제 매장 세트를 Gazebo(Harmonic, ROS 2 Jazzy)로 옮긴 패키지.

| 항목 | 출처 | 방법 |
|---|---|---|
| 벽 | `fleet_msgs/docs/maps/pinkytorang_map_v5.pgm` (2.5 cm) | 점유 칸을 직사각형으로 묶어 박스로 세움 (높이 0.20 m, 90개) |
| 테이블 T1~T6 | 실제 도면 `table1.stl` (165 × 110 × 150 mm) | node_graph의 T 좌표(통로 쪽 짧은 변 가운데)에서 -y로 놓음. PGM 속 테이블 자국은 벽에서 지움 |
| 로봇 | `pinky_pro/pinky_description` | 몸체는 그대로 쓰고, 뒤에 `tray1.stl` 트레이를 붙임 |
| 좌표 | `node_graph.yaml` v7.1 | **Gazebo world 프레임 = map 프레임**이라 노드 좌표를 그대로 쓴다 |

```
pinkytorang_gz/
├── config/   robots.yaml(로봇 · 도메인 · 시작 노드) · spawn_poses.yaml(생성됨) · 맵 v5 · node_graph 사본
├── launch/   sim.launch.py
├── meshes/   tray1.stl
├── models/   pinky_table/ (table1.stl)
├── params/   nav_v5.yaml(원본 사본) · nav_v5_sim.yaml(생성됨, use_sim_time: true)
├── tools/    gen_world.py (월드 · 스폰 자리 · sim용 nav 설정 생성)
├── urdf/     pinky_tray.urdf.xacro
└── worlds/   pinkytorang_v5.sdf (생성됨)
```

## 설치 · 실행

`ros-repo-2/ros_ws/src/`에 `pinky_pro`와 이 폴더를 같이 둔다.

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-xacro
cd ~/ros-repo-2/ros_ws
colcon build --packages-up-to pinkytorang_gz pinky_navigation && source install/setup.bash

ros2 launch pinkytorang_gz sim.launch.py                    # 로봇 3대 (D1 · D2 · D3), GUI
ros2 launch pinkytorang_gz sim.launch.py only:=pinky1       # 한 대만
ros2 launch pinkytorang_gz sim.launch.py nav:=true          # + 로봇별 Nav2 + AMCL 초기 위치
ros2 launch pinkytorang_gz sim.launch.py agent:=true camera:=false   # + fleet_agent (관제 서버와 연결, 아래 "전체 시스템")
ros2 launch pinkytorang_gz sim.launch.py gui:=false         # 헤드리스
```

## 전체 시스템 (시뮬 로봇 3대 + 관제 서버 + 웹)

터미널 3개. 전부 같은 setup을 source한 뒤 실행한다 (`/opt/ros/jazzy` → `pinky_ws/install` → `ros_ws/install`).

```bash
# 1) 시뮬 + 로봇별 Nav2 + fleet_agent  (GUI는 3번에서 따로)
ros2 launch pinkytorang_gz sim.launch.py agent:=true camera:=false gui:=false

# 2) 관제 서버 + domain_bridge (ros_ws 폴더에서, 레포 README와 같음)
export ROS_DOMAIN_ID=50
ros2 launch src/main_service/launch/system.launch.xml localize:=false

# 3) Gazebo 화면 (노트북 NVIDIA)
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia QT_QPA_PLATFORM=xcb gz sim -g
```

웹: `http://localhost:8080/store` 매장 · `/kiosk` 입구 키오스크 · `/c?table=T3` 고객 주문

- `agent:=true`면 Nav2가 같이 켜지고, 로봇마다 fleet_agent + obstacle이 그 로봇 도메인(51~53)에서 뜬다.
  설정은 `params/fleet_agent_sim.yaml` (원본 fleet_agent.yaml에서 아래만 바꿈)
  - `use_sim_time: false`: 서버는 `/fleet/state`를 벽시계 3초(`comm_timeout_sec`)로 보는데, sim 시간이면
    시뮬이 실시간보다 느릴 때 state가 늦어져 통신 끊김 → Move 재전송 → Nav2 목표 취소가 반복된다
  - `scan_yaw_offset: π`: 실제 sllidar는 scan 0° = 로봇 앞, gz 라이다는 0° = 로봇 뒤
    (안 바꾸면 obstacle이 뒤의 트레이 · 벽을 앞으로 보고 출발하자마자 BLOCKED)
  - `corridor_handover: false`: 정밀 정차가 없으니 통로 후진을 docking에 안 넘김
- 카메라 · 정밀 정차(docking) · guide는 끔 (`use_camera/use_docking/use_guide:=false`). 안내 손님 확인을 흉내 내려면 `fake_guide:=true`
- `localize:=false`: 시작 위치 맞추기(앞으로 + 360° 회전)는 시뮬에서 필요 없음 (스폰 자리 = AMCL 초기 위치)
- 서버 설정 `auto_ready: true`, `pickup_timeout_sec: 5`라서 주문하면 조리 · 적재 버튼 없이 진행된다

### 시뮬에서 아직 안 되는 것

- **도크로 들어가기 (D1 · D2 · D3)**: 실제로는 정밀 정차(entrance · dock_in)가 LiDAR 장애물 판단을 끄고 후진으로 넣는데,
  시뮬엔 마커가 없어 Nav2로만 간다. 칸이 좁아 obstacle이 `BLOCKED`를 내고 도크 앞(N1 등)에서 멈춘다.
  안내 · 서빙의 테이블까지 가는 흐름은 되지만 작업 마지막 `TO_DOCK`이 안 끝난다.
- 테이블 근처에서 `BLOCKED ... DYNAMIC`이 가끔 난다 (테이블 앞치마가 라이다 높이에 걸림). 서버가 우회시켜서 결국 지나간다.

## 도메인 구조 (실제 로봇과 같음)

gz 서버는 하나만 띄우고, 로봇마다 `ROS_DOMAIN_ID`를 따로 둔다 (`config/robots.yaml`: pinky1=51, pinky2=52, pinky3=53).
각 도메인 안에는 실제 로봇처럼 네임스페이스 없이 토픽이 있다.

```
gz:  /pinky1/cmd_vel /pinky1/scan ...  ──(ros_gz_bridge, 도메인 51)──>  /cmd_vel /scan /odom /tf /imu
                                                                        /joint_states /camera/image_raw /clock
TF:  odom -> base_footprint -> base_link -> rplidar_link · tray_link · tray_top ...   (프레임 이름 접두어 없음)
```

그래서 서버 쪽은 바꿀 게 없다.

```bash
export ROS_DOMAIN_ID=50
ros2 launch src/main_service/launch/system.launch.xml       # domain_bridge 50 <-> 51/52/53 + 관제 서버
```

한 로봇 도메인에 들어가서 보기:

```bash
ROS_DOMAIN_ID=51 rviz2                                      # Fixed Frame: map (nav:=true) / odom
ROS_DOMAIN_ID=51 ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 트레이 (`urdf/pinky_tray.urdf.xacro`)

- STL 걸쇠 쪽(+x 끝)을 몸체 뒷면(`base_link` x = -0.0745)에 1 mm 띄워 붙임 → 트레이 끝이 `base_footprint` 기준 **x = -0.145**
  (fleet_agent obstacle "뒤 바구니 0.15", Nav2 footprint 뒤 -0.12와 비교해서 맞추면 됨)
- 트레이 바닥 높이 `tray_z` = base_link 기준 -0.012 (바닥에서 16 mm). 다르면 xacro 인자로
- `tray_top` 프레임 = 물건 놓는 자리
- 끄기: `with_tray:=false`

## 실물 세트 world (`twin:=true`)

```bash
ros2 launch pinkytorang_gz sim.launch.py twin:=true                 # 나머지 인자(only · nav · agent ...)는 그대로
python3 tools/gen_twin.py                                           # 고친 뒤 다시 생성 -> worlds/pinkytorang_twin.sdf · twin_meshes/
```

PGM 벽 대신 실제 세트 치수로 세운 world. 좌표 · 스폰 · 테이블 자리는 위와 같다.

- 벽 30 cm. N13~N14 통로 위는 칸막이(x 0.40)를 북쪽 벽까지 이은 윗부분만 (바닥에서 17 cm 비움, 로봇은 밑으로)
- P1 오픈 키친: 픽업 상자 바닥 슬롯(높이 5 · 깊이 6 cm)에 트레이가 들어감 + 카운터 65 × 19 + 안쪽 주방
- 인쇄물 텍스처 `tools/twin_tex/`: KITCHEN 띠 · OPEN KITCHEN 카운터 · 메뉴판 · 충전 도크 D1~D3 · E자 테두리 종이
- 바닥 테이프 E자 · N15 · N16 가로선 (검정), 마커 1~6 자리 (색 판, 아직 ArUco 무늬 아님)
- 테이블 T1~T6 + D1 옆 키오스크 테이블(K) = `model://pinky_table`

## 맵이나 노드를 바꿨을 때

```bash
python3 tools/gen_world.py --preview /tmp/world.png                 # config/ 사본 기준
python3 tools/gen_world.py --map ../fleet_msgs/docs/maps/<새 맵>.yaml --graph ../fleet_msgs/docs/node_graph.yaml
python3 tools/gen_world.py --wall-height 0.25                       # 벽 높이
```

다시 생성되는 것: `worlds/pinkytorang_v5.sdf`, `config/spawn_poses.yaml`, `params/nav_v5_sim.yaml`. 그다음 `colcon build`.

## 알아 둘 것

- **테이블 STL은 뒤집혀 그려져 있음** (z=0 판이 상판, 기둥이 다리). `models/pinky_table/model.sdf`에서 roll 180°로 바로 세웠다.
  라이다 높이(12.5 cm)에서는 상판 아래 앞치마(10~15 cm)가 잡혀서 PGM의 속 빈 사각형과 같다.
- **스폰 자리 보정**: 도크 D1 · D2 · D3에 로봇+트레이를 그대로 놓으면 맵 벽과 1~6 cm 겹친다 (맵이 실제보다 조금 어긋남).
  `gen_world.py`가 벽과 안 겹칠 때까지 앞(최대 10 cm) · 옆(±2 cm)으로 옮겨 `spawn_poses.yaml`에 적는다
  (D1 +6.0 cm, D2 +1.1 cm, D3 +5.5 cm). AMCL 초기 위치도 이 값으로 넣는다.
- 바닥 테이프(E자, 가로선) · ArUco 마커 · IR 바닥 센서는 아직 없다 → precision_stops의 dock / tape / table 같은 정밀 정차는 시뮬에서 안 된다.
  Nav2 이동 · 관제 · 교통 관리(lock) 시험용이다.
- 카메라는 3대 부하 때문에 640×480 15 Hz (xacro 인자 `cam_width/height/rate`). 안 쓰면 `camera:=false`
- Nav2는 `use_composition:=False`(노드마다 프로세스)로 띄운다. 컴포저블 로딩은 launch 프로세스의 ROS 노드 하나로
  서비스를 불러서, 로봇마다 도메인이 다르면 다른 로봇 컨테이너에 잘못 실린다.
- 램프 플러그인은 `pinky_gz_sim`을 빌드했을 때만 `lamp_plugin:=true`.
