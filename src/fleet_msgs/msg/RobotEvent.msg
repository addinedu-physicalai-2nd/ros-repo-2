# 로봇 -> 서버 (fleet/event), 일이 생길 때만, Reliable
# 서버의 lock 해제는 NODE_REACHED 기준:
#   노드 N의 lock은 로봇이 N 다음 노드에 대해 NODE_REACHED를 보냈을 때 해제한다.

uint8 NODE_REACHED=0    # 노드 도착 (허용 오차 이내). lock 여부와 상관없이 모든 노드마다 빠짐없이
uint8 CMD_DONE=1        # 명령의 마지막 노드 도착 (도킹 노드라면 정밀 도킹까지 끝난 뒤)
uint8 CMD_FAILED=2      # 이동 실패 (reason에 이유)
                        # 3: 사용 안 함 (v3의 LOAD_CHANGED. 적재 확인은 GUI -> 서버로 바뀜)
uint8 BLOCKED=4         # 일정 시간 진척 없음
uint8 DOCKED=5          # (v10) 도킹 정밀 정차 끝 (node_id = 도크, reason = 실패면 이유). 같은 도크의 CMD_DONE 바로 앞
uint8 UNDOCKED=6        # (v10) 도크에서 빠지기 끝 (node_id = 떠난 도크, reason = 실패면 이유). 그다음 이동 시작
uint8 LOCALIZED=7       # (v11) Control LOCALIZE 끝 (node_id = 지금 노드, reason = 못 했으면 이유: busy / cancel / ...)

string robot_id                   # 로봇 ID
builtin_interfaces/Time stamp     # 발생 시각
uint8 type                        # 위 상수 중 하나
uint32 cmd_id                     # 관련 명령
string node_id                    # 관련 노드
string reason                     # CMD_FAILED일 때 이유
