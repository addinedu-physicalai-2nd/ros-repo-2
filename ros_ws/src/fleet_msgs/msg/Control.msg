# 서버 -> 로봇 (fleet/control)
# 현재 수행 중인 명령에 적용된다.

uint8 PAUSE=0       # 그 자리에서 정지 (명령 유지)
uint8 RESUME=1      # 정지했던 명령 이어서 수행
uint8 CANCEL=2      # 명령 취소, 그 자리에서 정지 후 IDLE
uint8 LOCALIZE=3    # (v11) 위치 맞추기: 쉬는 중일 때만. 앞으로 조금 -> 제자리 360° (AMCL) -> 뒤로 같은 만큼 -> RobotEvent LOCALIZED
                    #       시스템 시작 전에 서버가 준비된 로봇에게 한 대씩 차례로 보낸다 (거리 · 각도는 로봇 precision_stops의 <노드>.localize)

uint8 command
