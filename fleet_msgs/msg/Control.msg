# 서버 -> 로봇 (fleet/control)
# 현재 수행 중인 명령에 적용된다.

uint8 PAUSE=0       # 그 자리에서 정지 (명령 유지)
uint8 RESUME=1      # 정지했던 명령 이어서 수행
uint8 CANCEL=2      # 명령 취소, 그 자리에서 정지 후 IDLE

uint8 command
