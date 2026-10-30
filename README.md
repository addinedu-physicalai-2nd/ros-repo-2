# ros-repo-2
자율주행 프로젝트 2조 저장소. 핑키토랑 (로봇 3대 서빙 · 안내 관제)

```
ros-repo-2/
├── ros_ws/                 ROS 2 워크스페이스 (여기서 colcon build)
│   └── src/
│       ├── fleet_msgs/     서버 ↔ 로봇 메시지 · 노드 그래프 · 맵 · 브릿지
│       ├── main_service/   관제 서버 (C++17, 패키지 pinkytorang_main)
│       └── fleet_agent/    로봇 에이전트 (Python) · 정밀 정차 · 기록
└── web/                    고객 · 매장 · 입구 키오스크 UI (서버가 띄움)
```

```bash
# 빌드 (서버 PC: fleet_msgs pinkytorang_main / 로봇: --packages-up-to fleet_agent)
cd ~/ros-repo-2/ros_ws && colcon build && source install/setup.bash

# 서버 PC (ROS_DOMAIN_ID=50): 브릿지 + 서버 + 시작 전 위치 맞추기
ros2 launch src/main_service/launch/system.launch.xml            # pinky3:=false · config:=main_service_test.yaml · localize:=false

# 로봇 (ROS_DOMAIN_ID=51/52/53, 도크에 올려 두고)
ros2 launch fleet_agent robot_system.launch.xml robot_id:=$(hostname) start_node:=D1 use_docking:=true use_guide:=true \
  precision_stops:=$(ros2 pkg prefix fleet_msgs)/share/fleet_msgs/docs/precision_stops.yaml
```

화면: `http://<서버IP>:8080/store` 매장 · `/kiosk` 입구 · `/c?table=T1` 고객
