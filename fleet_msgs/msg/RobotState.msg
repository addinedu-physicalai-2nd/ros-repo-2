# 로봇 -> 서버 (fleet/state), 주기적 2~5Hz, Best Effort
# 서버는 위치 표시와 생존 확인에 쓴다. lock 판단에는 RobotEvent를 쓴다.

uint8 IDLE=0
uint8 MOVING=1
uint8 PAUSED=2
uint8 BLOCKED=3
uint8 ERROR=4

string robot_id                   # 로봇 ID (예: pinky1)
builtin_interfaces/Time stamp     # 보고 시각
float64 x                         # 현재 위치 (map, m)
float64 y
float64 yaw                       # (rad)
uint8 state                       # IDLE / MOVING / PAUSED / BLOCKED / ERROR
uint32 cmd_id                     # 수행 중인 명령 (없으면 마지막 명령)
string last_node                  # 마지막으로 도착한 노드
float32 battery                   # 배터리 (%) - 모니터링용 (도킹에서 충전하지 않음)
