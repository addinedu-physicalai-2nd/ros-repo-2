class TableDocker:
    def __init__(self, node, pub, cancelled, tf_buf, cam, ir): ...

    def run(self, req):
        """req = precision_stops T 설정 + node + from_node"""
        f = req['from'][req['from_node']]          # 들어온 노드의 마커 · 기록
        g = load(f'~/dock_target_{f["target"]}.json')
        # 1. 마커 찾기: 지금 방향 -> 기록에서 계산한 방향 -> ±10 · ±20 -> 가운데 맞추기 -> 측정
        # 2. 차선 계산 -> 차선 위 entry_ahead 앞 점으로 올라타기 -> 차선 방향
        # 3. 차선 방향 직진, 정렬 시작부터 req['lines']번째 선에서 정지
        return '성공'

    def leave(self, req):
        """서빙 뒤: 지금 방향으로 req['dist'] 전진 (Motion.drive)"""
        return '성공'