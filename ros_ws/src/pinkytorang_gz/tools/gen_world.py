#!/usr/bin/env python3
"""핑키토랑 Gazebo 월드 생성기

  벽   : 맵 PGM(pinkytorang_map_v5)의 점유 칸(검정)을 직사각형으로 묶어 박스로 세움
  테이블: 실제 도면 STL(table1.stl, 165 x 110 x 150 mm)을 node_graph의 T1~T6 위치에 배치
          T 좌표 = 테이블 통로 쪽 짧은 변 가운데 (node_graph 주석) -> 테이블은 -y 쪽으로 165 mm
  PGM 안의 테이블 자국(작은 덩어리)은 벽에서 빼고 STL로 대체한다.

  map 프레임 = Gazebo world 프레임 (원점 같음). 그래서 node_graph 좌표 그대로 스폰 · AMCL 초기 위치로 쓴다.

사용:
  python3 tools/gen_world.py                               # config/ 안의 맵 · 노드그래프로 worlds/ 생성
  python3 tools/gen_world.py --wall-height 0.25 --preview /tmp/world.png
  python3 tools/gen_world.py --map <다른 맵.yaml> --graph <node_graph.yaml>   # 맵을 다시 만들었을 때
"""
import argparse
import math
import os
import re

import numpy as np
import yaml
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)

# 테이블 STL (mm): x 0~165 (긴 변), y 0~110, z 0~150 (뒤집혀 그려짐: z=0 판이 상판). model.sdf에서 바로 세움
TABLE_L, TABLE_W, TABLE_H = 0.165, 0.110, 0.150


def load_map(map_yaml):
    with open(map_yaml) as f:
        m = yaml.safe_load(f)
    img_path = m['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(map_yaml), img_path)
    img = np.array(Image.open(img_path).convert('L')).astype(np.float32)
    if m.get('negate', 0):
        img = 255 - img
    occ_p = (255 - img) / 255.0
    occ = occ_p > m.get('occupied_thresh', 0.65)
    return occ, float(m['resolution']), m['origin']


def run_lengths(occ):
    """각 칸의 가로 · 세로 연속 점유 길이 (칸 수)"""
    H, W = occ.shape
    hr = np.zeros_like(occ, dtype=int)
    vr = np.zeros_like(occ, dtype=int)
    for r in range(H):
        c = 0
        while c < W:
            if occ[r, c]:
                e = c
                while e < W and occ[r, e]:
                    e += 1
                hr[r, c:e] = e - c
                c = e
            else:
                c += 1
    for c in range(W):
        r = 0
        while r < H:
            if occ[r, c]:
                e = r
                while e < H and occ[e, c]:
                    e += 1
                vr[r:e, c] = e - r
                r = e
            else:
                r += 1
    return hr, vr


def merge_rects(occ):
    """점유 칸을 탐욕적으로 직사각형으로 묶음 -> [(r0, c0, r1, c1)] (끝 포함 안 함)"""
    H, W = occ.shape
    used = np.zeros_like(occ, dtype=bool)
    rects = []
    for r in range(H):
        for c in range(W):
            if not occ[r, c] or used[r, c]:
                continue
            c1 = c
            while c1 < W and occ[r, c1] and not used[r, c1]:
                c1 += 1
            r1 = r + 1
            while r1 < H and all(occ[r1, k] and not used[r1, k] for k in range(c, c1)):
                r1 += 1
            used[r:r1, c:c1] = True
            rects.append((r, c, r1, c1))
    return rects


def table_poses(graph):
    """T 노드 -> 테이블 중심 (x, y). 긴 변(165)은 y 방향, 테이블은 T 점에서 -y 쪽"""
    out = []
    for n in graph['nodes']:
        if n.get('stop_kind') == 'SERVE':
            out.append((n['table'], n['x'], n['y'] - TABLE_L / 2.0))
    return out


def box_xml(name, x, y, z, sx, sy, sz, rgba):
    col = ' '.join(f'{v:g}' for v in rgba)
    return f"""        <collision name="{name}_c">
          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>
        <visual name="{name}_v">
          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material><ambient>{col}</ambient><diffuse>{col}</diffuse></material>
        </visual>
"""


def build(args):
    occ, res, origin = load_map(args.map)
    H, W = occ.shape
    ox, oy = origin[0], origin[1]
    with open(args.graph) as f:
        graph = yaml.safe_load(f)
    tables = table_poses(graph)

    def cell_xy(r, c):
        return ox + (c + 0.5) * res, oy + (H - 1 - r + 0.5) * res

    # PGM 속 테이블 자국 지우기: 테이블 바닥 영역(+여유) 안이면서 긴 벽 줄(8칸 이상 연속)이 아닌 칸
    hr, vr = run_lengths(occ)
    wall_like = (hr >= args.wall_run) | (vr >= args.wall_run)
    walls = occ.copy()
    removed = 0
    m = args.table_margin
    for r in range(H):
        for c in range(W):
            if not occ[r, c] or wall_like[r, c]:
                continue
            x, y = cell_xy(r, c)
            for _, tx, ty in tables:
                if abs(x - tx) <= TABLE_W / 2 + m and abs(y - ty) <= TABLE_L / 2 + m:
                    walls[r, c] = False
                    removed += 1
                    break

    rects = merge_rects(walls)
    h = args.wall_height
    parts = []
    for i, (r0, c0, r1, c1) in enumerate(rects):
        x0 = ox + c0 * res
        x1 = ox + c1 * res
        y1 = oy + (H - r0) * res
        y0 = oy + (H - r1) * res
        parts.append(box_xml(f'w{i}', (x0 + x1) / 2, (y0 + y1) / 2, h / 2, x1 - x0, y1 - y0, h,
                             (0.93, 0.93, 0.92, 1)))

    # 테이블: 모델 원점 = 바닥 발자국 가운데 (models/pinky_table/model.sdf), yaw 90도 -> 긴 변이 map y
    table_inc = []
    for name, px, py in tables:
        table_inc.append(f"""    <include>
      <uri>model://pinky_table</uri>
      <name>table_{name}</name>
      <pose>{px:.4f} {py:.4f} 0 0 0 {math.pi/2:.6f}</pose>
    </include>
""")

    world = f"""<?xml version="1.0" ?>
<!-- 자동 생성: tools/gen_world.py (손으로 고치지 말고 다시 생성)
     맵 {os.path.basename(args.map)} (해상도 {res} m, 원점 {ox}, {oy}) · 노드그래프 {os.path.basename(args.graph)}
     벽 박스 {len(rects)}개 (높이 {h} m), 테이블 {len(tables)}개 (table1.stl), PGM 테이블 자국 {removed}칸 제거
     world 프레임 = map 프레임 -->
<sdf version="1.9">
  <world name="pinkytorang">
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.8 0.8 0.8 1</background>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>1 0 3 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -1</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.78 0.78 0.76 1</ambient><diffuse>0.78 0.78 0.76 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- 벽: PGM 점유 칸 -->
    <model name="walls">
      <static>true</static>
      <link name="link">
{''.join(parts)}      </link>
    </model>

    <!-- 테이블: 실제 도면 STL -->
{''.join(table_inc)}  </world>
</sdf>
"""
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, 'w') as f:
        f.write(world)
    print(f'[world] {args.out}: 벽 박스 {len(rects)}개, 테이블 {len(tables)}개, 테이블 자국 {removed}칸 제거')
    return occ, walls, res, origin, tables, graph


# 로봇 + 트레이 바닥 발자국 (base_footprint 기준, m). 트레이는 urdf/pinky_tray.urdf.xacro와 같은 값
ROBOT_POLY = [(0.040, 0.048), (0.040, -0.048), (-0.146, -0.048), (-0.146, 0.048)]


def spawn_poses(walls, res, origin, graph, out_path, clearance=0.005):
    """STOP 노드(도크 등)에 로봇을 놓았을 때 벽 박스와 겹치면 앞(+x 로봇 방향)으로 조금씩 밀어 스폰 자리 계산.
    실제로는 들어가는 자리인데 맵 벽이 1~3 cm 어긋나 있어서, 스폰 순간 겹쳐 튕기는 것만 막는다."""
    H, W = walls.shape
    ox, oy = origin[0], origin[1]
    rr, cc = np.nonzero(walls)
    wx = ox + (cc + 0.5) * res
    wy = oy + (H - 1 - rr + 0.5) * res
    half = res / 2 + clearance
    xs = [p[0] for p in ROBOT_POLY]
    ys = [p[1] for p in ROBOT_POLY]

    def hits(x, y, yaw):
        # 벽 칸 중심을 로봇 좌표로 옮겨 (칸 반 + 여유)만큼 키운 사각형과 비교
        dx, dy = wx - x, wy - y
        lx = dx * math.cos(yaw) + dy * math.sin(yaw)
        ly = -dx * math.sin(yaw) + dy * math.cos(yaw)
        return int(np.sum((lx > min(xs) - half) & (lx < max(xs) + half) &
                          (ly > min(ys) - half) & (ly < max(ys) + half)))

    poses = {}
    for n in graph['nodes']:
        if n.get('type') != 'STOP' or 'yaw' not in n:
            continue
        yaw = float(n['yaw'])
        # 앞(+x) 0~10 cm, 옆 -2~2 cm 중에서 가장 덜 옮기는 자리 (5 mm 간격)
        best = None
        for fwd in np.arange(0.0, 0.1001, 0.005):
            for lat in np.arange(-0.02, 0.0201, 0.005):
                d = math.hypot(fwd, lat)
                if best is not None and d >= best[0]:
                    continue
                x = n['x'] + fwd * math.cos(yaw) - lat * math.sin(yaw)
                y = n['y'] + fwd * math.sin(yaw) + lat * math.cos(yaw)
                if hits(x, y, yaw) == 0:
                    best = (d, x, y)
        if best is None:
            best = (0.0, n['x'], n['y'])
        shift, x, y = float(best[0]), float(best[1]), float(best[2])
        poses[n['id']] = {'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4),
                          'shift_m': round(shift, 3), 'ok': hits(x, y, yaw) == 0}
    with open(out_path, 'w') as f:
        f.write('# 자동 생성: tools/gen_world.py. 스폰 자리 (map = world 프레임)\n'
                '# shift_m: 벽과 안 겹치게 로봇 앞쪽으로 민 거리. AMCL 초기 위치는 이 값으로 넣는다\n')
        yaml.safe_dump(poses, f, sort_keys=False, allow_unicode=True)
    for k, v in poses.items():
        print(f'[spawn] {k}: ({v["x"]}, {v["y"]}, yaw {v["yaw"]}) shift {v["shift_m"]} m ok={v["ok"]}')
    return poses


def make_sim_nav_params(src, dst, overrides=None, sim_time=True):
    """실제 로봇용 ROS 파라미터 yaml -> use_sim_time: true 판 (nav_v5.yaml, fleet_agent.yaml)
    overrides: {키: 값} 이미 있는 키의 값만 바꿈 (예: corridor_handover: false)"""
    if not os.path.exists(src):
        print(f'[nav] {src} 없음 -> 건너뜀')
        return
    out = []
    for line in open(src):
        if re.match(r'^\s*use_sim_time\s*:', line):
            continue
        for k, v in (overrides or {}).items():
            mk = re.match(r'^(\s*)' + k + r'\s*:\s*[^#\n]*(#.*)?$', line)
            if mk:
                line = f'{mk.group(1)}{k}: {v}   # 시뮬 (gen_world.py)\n'
        out.append(line)
        mm = re.match(r'^(\s*)ros__parameters\s*:\s*(#.*)?$', line)
        if mm:
            out.append(f'{mm.group(1)}  use_sim_time: {"true" if sim_time else "false"}\n')
    with open(dst, 'w') as f:
        f.write(f'# 자동 생성: tools/gen_world.py ({os.path.basename(src)} -> use_sim_time: {str(sim_time).lower()})\n')
        f.writelines(out)
    print(f'[nav] {dst}')


def preview(path, occ, walls, res, origin, tables, graph):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = ['Noto Sans CJK KR', 'DejaVu Sans']
    from matplotlib.patches import Rectangle, Polygon
    H, W = occ.shape
    ox, oy = origin[0], origin[1]
    ext = [ox, ox + W * res, oy, oy + H * res]
    fig, ax = plt.subplots(1, 2, figsize=(16, 7))
    ax[0].imshow(~occ, cmap='gray', extent=ext)
    ax[0].set_title('PGM occupied (원본)')
    ax[1].imshow(~walls, cmap='gray', extent=ext, alpha=0.6)
    for name, tx, ty in tables:
        ax[1].add_patch(Rectangle((tx - TABLE_W / 2, ty - TABLE_L / 2), TABLE_W, TABLE_L, fc='orange', ec='k'))
        ax[1].text(tx, ty, name, ha='center', va='center', fontsize=8)
    for n in graph['nodes']:
        ax[1].plot(n['x'], n['y'], '.', c='b')
        ax[1].text(n['x'] + 0.01, n['y'] + 0.01, n['id'], fontsize=6, color='b')
        if n.get('stop_kind') == 'DOCK':   # 로봇 + 트레이 발자국
            yaw = n.get('yaw', 0.0)
            sp = yaml.safe_load(open(os.path.join(PKG, 'config', 'spawn_poses.yaml')))[n['id']]
            nx, ny = sp['x'], sp['y']
            pts = [(0.0387, 0.047), (0.0387, -0.047), (-0.0745, -0.047), (-0.0745, 0.047)]
            tray = [(-0.0756, 0.041), (-0.0756, -0.041), (-0.145, -0.041), (-0.145, 0.041)]
            for poly, col in ((pts, 'tab:blue'), (tray, 'tab:green')):
                g = [(nx + px * math.cos(yaw) - py * math.sin(yaw),
                      ny + px * math.sin(yaw) + py * math.cos(yaw)) for px, py in poly]
                ax[1].add_patch(Polygon(g, fc=col, alpha=0.6))
    ax[1].set_title('Gazebo 월드: 벽(PGM) + 테이블(STL) + 도크 로봇/트레이')
    for a in ax:
        a.set_xlim(ext[0], ext[1]); a.set_ylim(ext[2], ext[3]); a.set_aspect('equal'); a.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(path, dpi=90)
    print(f'[preview] {path}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=os.path.join(PKG, 'config', 'pinkytorang_map_v5.yaml'))
    ap.add_argument('--graph', default=os.path.join(PKG, 'config', 'node_graph.yaml'))
    ap.add_argument('--out', default=os.path.join(PKG, 'worlds', 'pinkytorang_v5.sdf'))
    ap.add_argument('--wall-height', type=float, default=0.20)
    ap.add_argument('--wall-run', type=int, default=8, help='이 칸 수 이상 이어지면 벽으로 봄 (테이블 자국 지울 때 보호)')
    ap.add_argument('--table-margin', type=float, default=0.03)
    ap.add_argument('--nav-src', default=os.path.join(PKG, 'params', 'nav_v5.yaml'))
    ap.add_argument('--nav-out', default=os.path.join(PKG, 'params', 'nav_v5_sim.yaml'))
    ap.add_argument('--agent-src', default=os.path.join(PKG, 'params', 'fleet_agent.yaml'))
    ap.add_argument('--agent-out', default=os.path.join(PKG, 'params', 'fleet_agent_sim.yaml'))
    ap.add_argument('--spawn-out', default=os.path.join(PKG, 'config', 'spawn_poses.yaml'))
    ap.add_argument('--preview', default='')
    args = ap.parse_args()
    res = build(args)
    occ, walls, r, origin, tables, graph = res
    spawn_poses(walls, r, origin, graph, args.spawn_out)
    make_sim_nav_params(args.nav_src, args.nav_out)
    # fleet_agent: 시뮬엔 마커 · 바닥선이 없어서 통로 후진을 docking에 넘기지 않음
    #   use_sim_time: false (벽시계). 서버는 /fleet/state를 벽시계 3초(comm_timeout_sec)로 보는데, sim 시간이면
    #   시뮬이 실시간보다 느려질 때 state가 늦게 나가 '통신 끊김 -> 복구 -> Move 재전송 -> Nav2 목표 취소'가 반복된다
    #   scan_yaw_offset: pi. 실제 sllidar 드라이버는 scan 0도 = 로봇 앞인데, gz 라이다는 0도 = rplidar_link +x
    #   (URDF에서 180도 돌아가 있음) = 로봇 뒤. 그대로 두면 obstacle 노드가 뒤(트레이 · 벽)를 앞으로 보고 BLOCKED
    make_sim_nav_params(args.agent_src, args.agent_out,
                        {'corridor_handover': 'false', 'scan_yaw_offset': '3.14159265'}, sim_time=False)
    if args.preview:
        preview(args.preview, *res)


if __name__ == '__main__':
    main()
