# Move 안에 들어가는 노드 1개
# - 좌표는 map 프레임 기준 (로봇 3대가 같은 맵 파일, 같은 원점 사용)
# - yaw는 로봇 앞이 보는 방향이다. Move의 마지막 노드에서만 맞춘다. 중간 노드는 위치만 통과한다.
#   후진(Move.drive = REVERSE) 중인 PASS 노드의 yaw는 서버가 "진행 방향 + 180도"로 채운다.
# - 전진 / 후진은 Move.drive로 정한다 (v4의 reverse 필드는 v5에서 삭제).

string node_id      # 노드 ID (예: N7, T1)
float64 x           # 맵 좌표 (m)
float64 y           # 맵 좌표 (m)
float64 yaw         # 로봇 앞이 보는 방향 (rad)
