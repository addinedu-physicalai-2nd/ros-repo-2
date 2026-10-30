# 서버 -> 로봇 (fleet/display), Reliable + Transient Local(마지막 화면 유지)
# 로봇 LCD에 띄울 내용. 문구는 서버가 정하고, 로봇은 받은 그대로 띄우기만 한다.
# - 카운트다운: 로봇이 메시지를 받은 순간부터 1초씩 줄여 표시한다. 0이 되면 숫자만 지운다.
# - 실제 출발 판단은 서버가 한다. 서버는 countdown_sec보다 1~2초 늦게 Move를 보낸다.
# - 새 Display가 오면 이전 화면과 카운트다운은 즉시 버린다.

string text             # 띄울 문구. 빈 문자열이면 화면을 지움(기본 화면)
uint16 countdown_sec    # 0이면 카운트다운 없음
