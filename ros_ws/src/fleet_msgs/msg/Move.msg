# 서버 -> 로봇 (fleet/move)
# - 서버는 lock을 획득한 노드까지만 보낸다. waypoints의 마지막 노드는 항상 lock 노드다.
# - lock이 없는 노드(코너)는 단독으로 보내지 않고, 그 다음 lock 노드와 묶어서 보낸다.
# - 로봇은 마지막 노드에서 정지하고 CMD_DONE을 보낸다.
# - drive: 이 Move의 waypoints 전체를 전진 / 후진 중 한 방향으로만 간다.
#          방향이 바뀌는 곳에서 서버는 Move를 끊어 보낸다 (끊는 노드는 항상 lock 노드).
#          앞 Move와 방향이 다르면 로봇은 앞 Move의 마지막 노드에서 멈추고 제자리 회전 후 출발한다.

uint8 REPLACE=0     # 기존 경로를 버리고 교체. 새 명령은 새 cmd_id
uint8 APPEND=1      # 기존 경로 뒤에 이어 붙임. 원래 명령과 같은 cmd_id. 같은 방향이면 멈추지 않고 이어서 주행

uint8 FORWARD=0     # 전진 (앞이 진행 방향)
uint8 REVERSE=1     # 후진 (뒤가 진행 방향)

# goal_node (v9): 이번 이동의 최종 도착지 (서버가 정한 목적지. 예: P1, T3, D4, 안내면 N6)
#   서버는 lock을 잡은 만큼씩 끊어 보내므로 waypoints 끝이 도착지가 아닐 수 있다 (길이 막혀 잠깐 섬)
#   로봇은 도착지에서만 도착 동작(정밀 정차 dock · tape, 안내 도착 효과, leave 준비)을 하고, 나머지 노드는 지나간다
# guide (v9): 안내 손님과 같이 가는 이동인지. 서빙 · 도킹 · 복귀는 false
#   true면 로봇이 알아서: 출발 전 손님 기다림(인사), 꺾이는 노드에서 뒤돌아 확인, 도착지에서 도착 효과
#   안내는 도착지에서 정밀 정차를 하지 않는다 (Nav2 위치 · 마지막 waypoint yaw 방향으로 섬)
#   손님을 놓치면 그 자리에 멈추고 RobotEvent CMD_FAILED, reason = "guest_lost:<노드>:<이유>"

uint32 cmd_id       # 명령 번호 (서버가 1씩 증가)
uint8 mode          # REPLACE / APPEND
uint8 drive         # FORWARD / REVERSE
string goal_node    # 최종 도착지 노드 ID ("" = 모름: 예전처럼 waypoints 끝을 도착지로 봄)
bool guide          # 안내 중
Waypoint[] waypoints  # 차례로 지나갈 노드
