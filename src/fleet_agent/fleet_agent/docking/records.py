"""기록(json) 위치: 레포 src/fleet_agent/records/ 하나에서 모든 로봇이 같이 읽는다 (로봇 역할이 고정이 아니라서)

읽기  find(이름)  : 1) 레포 소스 records/ (git pull만 하면 바로 반영)  2) 설치된 share/fleet_agent/records/
                    3) ~/ (예전 방식, 레포에 아직 없는 기록)
카메라 cam_file() : records/cam/<hostname>.json (로봇마다 다름) -> 없으면 ~/dock_cam.json
쓰기  save_path(이름): 기록 도구(dock_record · marker_dock --record · --calibrate)는 레포 소스 records/에 바로 씀
                    -> git diff로 보이고, 커밋 · push하면 다른 로봇도 같은 값 (소스 폴더를 못 찾으면 ~/)
레포 위치를 못 찾을 때는 환경 변수 FLEET_RECORDS_DIR=<.../src/fleet_agent/records>
"""
import os
import socket

_HOME = os.path.expanduser('~')


def _source_dir():
    env = os.environ.get('FLEET_RECORDS_DIR')
    if env and os.path.isdir(env):
        return env
    cands = []
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('fleet_agent')            # <ws>/install/fleet_agent/share/fleet_agent
        ws = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
        cands.append(os.path.join(ws, 'src', 'fleet_agent', 'records'))
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))                  # 소스에서 바로 돌릴 때 (테스트 · 혼자 실행)
    cands.append(os.path.abspath(os.path.join(here, '..', '..', 'records')))
    cands.append(os.path.join(_HOME, 'ros-repo-2', 'src', 'fleet_agent', 'records'))
    for c in cands:
        if os.path.isdir(c):
            return c
    return None


def _share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory('fleet_agent'), 'records')
        return d if os.path.isdir(d) else None
    except Exception:
        return None


def dirs():
    return [d for d in (_source_dir(), _share_dir(), _HOME) if d]


def find(name):
    """기록 파일 경로 (name = 'dock_target_d3.json' 같은 파일 이름). 없으면 레포 소스 쪽 경로 (에러 메시지용)"""
    name = os.path.basename(name)
    for d in dirs():
        p = os.path.join(d, name)
        if os.path.exists(p):
            return p
    return os.path.join(_source_dir() or _HOME, name)


def save_path(name):
    """새 기록을 쓸 곳: 레포 소스 records/ (못 찾으면 ~/)"""
    return os.path.join(_source_dir() or _HOME, os.path.basename(name))


def cam_file(write=False):
    """이 로봇 카메라 보정: records/cam/<hostname>.json. 읽기는 없으면 ~/dock_cam.json"""
    host = socket.gethostname()
    src = _source_dir()
    if write:
        if src:
            os.makedirs(os.path.join(src, 'cam'), exist_ok=True)
            return os.path.join(src, 'cam', f'{host}.json')
        return os.path.join(_HOME, 'dock_cam.json')
    for d in (src, _share_dir()):
        if d and os.path.exists(os.path.join(d, 'cam', f'{host}.json')):
            return os.path.join(d, 'cam', f'{host}.json')
    return os.path.join(_HOME, 'dock_cam.json')
