#!/usr/bin/env python3
"""핑키토랑 디지털 트윈 world (실물 세트 그대로): pinkytorang_v5.sdf(PGM 벽)와 같은 map 프레임

  - 벽 30 cm · N13~N14 통로 위 칸막이 (아래 17 cm 비움, 로봇은 밑으로 지나감)
  - P1 오픈 키친: 픽업 상자(바닥 슬롯 5 x 6 cm, 트레이가 들어감) + 카운터 65 x 19 + 안쪽 주방 (냉장고 · 싱크 · 화구 · 후드 · 팬트리)
  - 인쇄물 텍스처 (tools/twin_tex): KITCHEN 띠 · OPEN KITCHEN 카운터 · 메뉴판 · 충전 도크 D1~D3 · E자 테두리 종이
  - 바닥 테이프 (E자, N15 · N16 가로선), 손님 대기선 (D1 통로 입구, 노랑), AprilTag 마커 1~6 · 테이블 다리 11~62 (실제 인쇄물)
  - D1 옆 키오스크 테이블 + 키오스크 화면 (동쪽을 봄)
  - 테이블 T1~T6 + D1 옆 키오스크 테이블 = model://pinky_table (gen_world.py와 같은 자리), 상판에 주문 QR 카드

  python3 tools/gen_twin.py                       # -> worlds/pinkytorang_twin.sdf + worlds/twin_meshes/
  python3 tools/gen_twin.py --json /tmp/twin.json # (미리보기용 장면 데이터도)
  실행: ros2 launch pinkytorang_gz sim.launch.py twin:=true
"""
import argparse
import json
import math
import os
import shutil

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ap = argparse.ArgumentParser()
ap.add_argument('--graph', default=os.path.join(PKG, 'config', 'node_graph.yaml'))
ap.add_argument('--out', default=os.path.join(PKG, 'worlds'))
ap.add_argument('--json', default='')
ARGS = ap.parse_args()
GRAPH = yaml.safe_load(open(ARGS.graph))
WALL_H, WALL_T = 0.30, 0.02
X0, X1, Y0, Y1 = -0.15, 2.12, -1.10, 0.75     # 바깥 벽 중심선
KX = 0.07                                      # 주방 앞면 (= 주방 통로 서쪽 면)
CX = 0.235                                     # 주방 통로 가운데
P1 = (0.251, -0.905)

C = {'wall': '#f4f4f2', 'counter': '#fbfaf7', 'dark': '#1b1b1d', 'steel': '#b9b9be', 'stove': '#2b2b2f',
     'table': '#ffffff', 'leg': '#bdbdbd', 'tape': '#0a0a0a', 'robot': '#26262a', 'wheel': '#3fa3b5',
     'lidar': '#111111', 'basket': '#f2f2f0', 'bowl': '#ffffff', 'berry': '#d8322f', 'pot': '#9a9aa0',
     'box': '#d8d0c3', 'marker': '#e5487f', 'shelf': '#e9e5dd'}

boxes = []     # (이름, 색, cx, cy, cz, sx, sy, sz, yaw, 충돌)
cyls = []      # (이름, 색, cx, cy, cz, r, h)
quads = []     # (이름, 텍스처, cx, cy, cz, w, h, 향하는 방향 '+x' '-x' '+y' '-y' 'up', yaw(up일 때))


def box(n, col, x0, x1, y0, y1, z0, z1, coll=True):
    boxes.append((n, col, (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2, x1 - x0, y1 - y0, z1 - z0, 0.0, coll))


def wall(n, x0, y0, x1, y1, h=WALL_H):
    t = WALL_T / 2
    box(n, 'wall', min(x0, x1) - (t if x0 == x1 else 0), max(x0, x1) + (t if x0 == x1 else 0),
        min(y0, y1) - (t if y0 == y1 else 0), max(y0, y1) + (t if y0 == y1 else 0), 0, h)


def tape(n, cx, cy, sx, sy):
    box(n, 'tape', cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2, 0.0004, 0.0014, False)


# ================================================================ 홀 · 벽
wall('wall_south', X0, Y0, X1, Y0)
wall('wall_north', X0, Y1, X1, Y1)
wall('wall_west', X0, Y0, X0, Y1)
wall('wall_east', X1, Y0, X1, Y1)
wall('corridor_partition', 0.40, -0.85, 0.40, 0.33)
wall('entrance_wall', 1.67, -0.43, 1.67, Y1)

# D2 · D3 칸 (중심 간격 14.6 -> 칸 폭 14.6)
for n, y in (('dock_div_s', -0.094), ('dock_div_m', 0.052), ('dock_div_n', 0.198)):
    wall(n, X0, y, KX, y, h=0.19)

# 테이블: 실제 도면 table1.stl (165 x 110 x 150 mm) = model://pinky_table. 자리는 gen_world.py와 같음
#   (node_graph T 좌표 = 통로 쪽 짧은 변 가운데 -> -y로 165 mm). K = D1 옆 키오스크 테이블 (맵 v5 속 빈 사각형 자리)
TL, TW, TH = 0.165, 0.110, 0.150
T = {n['table']: (n['x'], n['y'] - TL / 2) for n in GRAPH['nodes'] if n.get('table')}
T['K'] = (1.68 + TW / 2 + 0.003, Y1 - WALL_T / 2 - TL / 2 - 0.003)   # D1 통로 북서 모서리
for k, (x, y) in T.items():     # 미리보기용 상자 (SDF에는 tbl_ 대신 pinky_table include). 치수 = table1.stl
    box(f'tbl_{k}_top', 'table', x - TW / 2, x + TW / 2, y - TL / 2, y + TL / 2, TH - 0.002, TH)
    box(f'tbl_{k}_apron', 'table', x - 0.045, x + 0.045, y - 0.0725, y + 0.0725, 0.10, TH - 0.002)
    for dx in (-0.038, 0.038):
        for dy in (-0.0655, 0.0655):
            box(f'tbl_{k}_leg', 'leg', x + dx - 0.011, x + dx + 0.011, y + dy - 0.011, y + dy + 0.011, 0, 0.10)

# 테이블 위 QR 카드 (table_qr_T1-T6_A4.pdf, 165 x 110 = 상판 그대로). 긴 변 = map y, 동쪽에서 읽힘
# 다리 마커: 통로 쪽(+y) 다리 두 개의 바깥면. 왼쪽(+1) = 동쪽 다리, 오른쪽(+2) = 서쪽 다리 (통로에서 테이블을 봤을 때)
for k, (x, y) in T.items():
    if k == 'K':
        continue
    quads.append((f'qr_{k}', f'qr_{k}', x, y, TH + 0.0004, TL, TW, 'up', math.pi / 2))
    num = int(k[1:]) * 10
    leg_face = y + 0.0765 + 0.0006
    quads.append((f'tag_{num + 1}', f'tag_{num + 1}', x + 0.038, leg_face, 0.065, 0.025 * 1.25, 0.025 * 1.25, '+y', 0))
    quads.append((f'tag_{num + 2}', f'tag_{num + 2}', x - 0.038, leg_face, 0.065, 0.025 * 1.25, 0.025 * 1.25, '+y', 0))

# 키오스크 (K 테이블 위): 받침 + 기둥 + 화면. 화면은 동쪽(통로 쪽)을 봄 -> 키오스크 기준 오른쪽(남쪽) = 손님 들어오는 쪽
KXc, KYc = T['K']
box('kiosk_base', 'dark', KXc - 0.03, KXc + 0.01, KYc - 0.02, KYc + 0.02, TH, TH + 0.006)
box('kiosk_pole', 'steel_d', KXc - 0.014, KXc - 0.006, KYc - 0.004, KYc + 0.004, TH + 0.006, TH + 0.05)
box('kiosk_body', 'dark', KXc - 0.006, KXc + 0.004, KYc - 0.034, KYc + 0.034, TH + 0.04, TH + 0.14)
quads.append(('kiosk_screen', 'kiosk_screen', KXc + 0.0046, KYc, TH + 0.09, 0.062, 0.093, '+x', 0))

# 손님 대기 구역: D1 통로가 끊기는 곳(입구 벽 남쪽 끝 y -0.43)을 가로지르는 노란 선 + 바닥 안내
#   N1(y -0.82)에서 기다리던 안내 로봇이 북쪽(선 너머 대기 손님)을 보고 사람을 인식하면 출발
WAIT_Y = -0.43
C['wait'] = '#f2c230'
box('wait_line', 'wait', 1.68, X1 - WALL_T / 2, WAIT_Y - 0.0095, WAIT_Y + 0.0095, 0.0004, 0.0014, False)
quads.append(('wait_sign', 'wait_sign', (1.68 + X1 - WALL_T / 2) / 2, WAIT_Y + 0.045, 0.0006, 0.215, 0.05, 'up', 0))

# ================================================================ 오픈 키친 (P1)
# 실물 앞면: [픽업 상자: 바닥 검은 슬롯(높이 5, 깊이 6) + KITCHEN 띠] + [긴 카운터 65 x 19 (OPEN KITCHEN 인쇄)]
# 뒤 (홀에서 넘겨다 보이는 곳) = 실제 주방처럼: 냉장고 · 조리대 · 싱크 · 화구+오븐+후드 · 팬트리, 패스(배식대) 조명
H = 0.19
SY0, SY1 = -1.03, -0.78            # 픽업 상자 (폭 25)
KW = X0 + WALL_T / 2               # 서쪽 벽 안쪽 면 (-0.14)
KS, KN = Y0 + WALL_T / 2, -0.16    # 주방 남 · 북 끝
FB = 0.055                         # 카운터 판 뒤 면 (판 두께 1.5)
CB = KW + 0.07                     # 벽 쪽 조리대 앞 끝 (깊이 7)
CH = 0.09                          # 조리대 높이 (실제 90 cm의 1/10)

C.update({'cabinet': '#3d4852', 'marble': '#eeece6', 'steel_d': '#7c7e84', 'glass': '#15181c', 'grate': '#151517',
          'wood': '#b8875a', 'handle': '#d4d5d9', 'lamp': '#e0892e', 'bulb': '#ffd98a', 'jar_r': '#c8453b',
          'jar_y': '#e2b33c', 'jar_g': '#5d9a4a', 'kraft': '#c9a77c', 'green': '#4c9a3f', 'tomato': '#d93b2b',
          'cup': '#f7f7f5', 'paper': '#fbfbf8', 'bin': '#5b6168'})

# --- 앞 (홀 쪽) : 픽업 상자 + 카운터 판
box('pickup_lip', 'counter', -0.01, KX, SY0, SY1, 0, 0.01)
box('pickup_slot_back', 'dark', -0.01, KX - 0.06, SY0 + 0.015, SY1 - 0.015, 0.01, 0.06)
box('pickup_slot_floor', 'dark', KX - 0.06, KX - 0.001, SY0 + 0.015, SY1 - 0.015, 0.0095, 0.0105, False)
box('pickup_cheek_s', 'counter', -0.01, KX, SY0, SY0 + 0.015, 0.01, 0.06)
box('pickup_cheek_n', 'counter', -0.01, KX, SY1 - 0.015, SY1, 0.01, 0.06)
box('pickup_body', 'counter', -0.01, KX, SY0, SY1, 0.06, H)
quads.append(('kitchen_sign', 'kitchen_sign', KX + 0.0006, (SY0 + SY1) / 2, 0.10, 0.25, 0.06, '+x', 0))
box('counter_panel', 'counter', FB, KX, SY1, -0.13, 0, H)
# 카운터 끝 ~ D2 남쪽 칸막이(dock_div_s) 사이 틈 막기: 같은 판으로 이어서 주방이 칸막이까지 막힘
KJ = -0.094 - WALL_T / 2
box('counter_join', 'counter', FB, KX, -0.13, KJ, 0, H)
quads.append(('counter_front', 'counter', KX + 0.0006, (SY1 - 0.13) / 2, H / 2, 0.65, 0.19, '+x', 0))
box('pass_shelf', 'marble', 0.035, KX + 0.008, SY1, KJ, H, H + 0.006)        # 배식대 상판 (판 위 좁은 턱)
box('pickup_top', 'marble', -0.01, KX + 0.008, SY0, SY1, H, H + 0.006)
cyls.append(('bell_base', 'steel_d', 0.03, SY1 - 0.04, H + 0.008, 0.009, 0.004))
cyls.append(('bell', 'jar_y', 0.03, SY1 - 0.04, H + 0.014, 0.007, 0.01))
for y in (-0.62, -0.50, -0.38):
    cyls.append(('bowl', 'bowl', 0.056, y, H + 0.012, 0.016, 0.012))
    cyls.append(('berry', 'berry', 0.056, y, H + 0.020, 0.010, 0.008))
# 배식대 안쪽 선반 (접시 · 쟁반)
box('pass_under', 'steel', 0.005, FB, SY1, -0.13, 0.10, 0.104)
for y in (-0.72, -0.66):
    for k in range(5):
        cyls.append(('plate_stack', 'bowl', 0.03, y, 0.106 + k * 0.004, 0.02, 0.003))
box('tray_stack', 'steel_d', 0.012, 0.05, -0.30, -0.20, 0.104, 0.112)
# --- 주방 바닥 (타일)
quads.append(('kitchen_floor', 'tile_floor', (KW + FB) / 2, (KS + KN) / 2, 0.0003, FB - KW, KN - KS, 'up', 0))


# --- 벽 쪽 라인 (남 -> 북)
def base_cab(n, y0, y1, top='marble'):
    box(n + '_cab', 'cabinet', KW, CB, y0, y1, 0.004, CH - 0.004)
    box(n + '_kick', 'grate', KW, CB - 0.006, y0, y1, 0, 0.004)
    box(n + '_top', top, KW, CB + 0.004, y0, y1, CH - 0.004, CH)
    if top == 'marble':
        quads.append((n + '_marble', 'marble', (KW + CB + 0.004) / 2, (y0 + y1) / 2, CH + 0.0004, CB + 0.004 - KW, y1 - y0, 'up', 0))
    w = y1 - y0
    k = max(1, round(w / 0.07))
    for i in range(k):                                         # 문짝 홈 + 손잡이
        yy = y0 + w * (i + 0.5) / k
        box(n + '_gap', 'grate', CB, CB + 0.0008, y0 + w * i / k - 0.0006, y0 + w * i / k + 0.0006, 0.01, CH - 0.01, False)
        box(n + '_handle', 'handle', CB, CB + 0.003, yy - 0.012, yy + 0.012, CH - 0.018, CH - 0.015, False)


# 냉장고 (2도어)
FY0, FY1 = KS + 0.005, -0.97
box('fridge', 'steel', KW, CB + 0.005, FY0, FY1, 0, 0.25)
box('fridge_gap', 'steel_d', CB + 0.005, CB + 0.006, FY0, FY1, 0.155, 0.158, False)
for z0, z1 in ((0.165, 0.24), (0.02, 0.145)):
    box('fridge_handle', 'handle', CB + 0.005, CB + 0.010, FY1 - 0.012, FY1 - 0.008, z0, z1, False)
# 조리대 1: 도마 · 칼 · 채소 상자
base_cab('prep1', FY1, -0.84)
box('board', 'wood', KW + 0.012, KW + 0.058, -0.955, -0.89, CH, CH + 0.005)
box('knife_blade', 'steel', KW + 0.03, KW + 0.034, -0.935, -0.905, CH + 0.005, CH + 0.006, False)
box('knife_grip', 'grate', KW + 0.029, KW + 0.035, -0.905, -0.893, CH + 0.005, CH + 0.008, False)
for i, y in enumerate((-0.92, -0.91)):
    cyls.append(('tomato', 'tomato', KW + 0.05, y - 0.01 * i, CH + 0.011, 0.006, 0.01))
box('veg_crate', 'kraft', KW + 0.008, KW + 0.05, -0.885, -0.848, CH, CH + 0.02)
for y in (-0.877, -0.866, -0.856):
    cyls.append(('veg', 'green', KW + 0.029, y, CH + 0.024, 0.006, 0.01))
# 싱크 + 수전 + 식기 건조대
base_cab('sink', -0.84, -0.68, top='steel')
quads.append(('sink_top', 'sink', (KW + CB + 0.004) / 2, -0.77, CH + 0.0005, CB + 0.004 - KW, 0.14, 'up', 0))
cyls.append(('faucet', 'handle', KW + 0.006, -0.79, CH + 0.02, 0.003, 0.04))
box('faucet_spout', 'handle', KW + 0.006, KW + 0.026, -0.792, -0.788, CH + 0.037, CH + 0.04, False)
box('rack', 'steel', KW + 0.01, KW + 0.06, -0.725, -0.69, CH, CH + 0.003)
for i in range(5):
    y = -0.72 + i * 0.006
    box('rack_plate', 'bowl', KW + 0.02, KW + 0.05, y, y + 0.002, CH + 0.003, CH + 0.03, False)
# 화구 + 오븐 + 후드
RY0, RY1 = -0.68, -0.46
box('range', 'steel', KW, CB + 0.003, RY0, RY1, 0, CH + 0.002)
box('range_top', 'grate', KW, CB + 0.003, RY0, RY1, CH + 0.002, CH + 0.004)
quads.append(('cooktop', 'cooktop', (KW + CB + 0.003) / 2, (RY0 + RY1) / 2, CH + 0.0045, CB + 0.003 - KW, RY1 - RY0, 'up', 0))
box('oven_door', 'glass', CB + 0.003, CB + 0.005, RY0 + 0.02, RY1 - 0.02, 0.012, 0.062, False)
box('oven_handle', 'handle', CB + 0.005, CB + 0.009, RY0 + 0.03, RY1 - 0.03, 0.066, 0.069, False)
for i in range(5):
    y = RY0 + 0.03 + i * (RY1 - RY0 - 0.06) / 4
    box('knob', 'grate', CB + 0.003, CB + 0.008, y - 0.004, y + 0.004, 0.074, 0.082, False)
cyls.append(('pot', 'pot', KW + 0.022, RY1 - 0.055, CH + 0.022, 0.016, 0.032))
cyls.append(('pot_lid', 'steel', KW + 0.022, RY1 - 0.055, CH + 0.039, 0.017, 0.002))
cyls.append(('pot_knob', 'grate', KW + 0.022, RY1 - 0.055, CH + 0.042, 0.003, 0.004))
cyls.append(('pan', 'grate', KW + 0.05, RY0 + 0.05, CH + 0.009, 0.017, 0.006))
box('pan_handle', 'grate', KW + 0.05 + 0.016, KW + 0.05 + 0.045, RY0 + 0.048, RY0 + 0.052, CH + 0.009, CH + 0.012, False)
cyls.append(('pan_egg', 'paper', KW + 0.05, RY0 + 0.05, CH + 0.0125, 0.008, 0.001))
# 조리대 2: 볼 · 양념통
base_cab('prep2', RY1, -0.31)
cyls.append(('mix_bowl', 'steel', KW + 0.035, -0.42, CH + 0.009, 0.016, 0.018))
cyls.append(('mix_bowl2', 'steel', KW + 0.035, -0.385, CH + 0.007, 0.012, 0.014))
for i, col in enumerate(('jar_r', 'jar_y', 'jar_g', 'jar_r')):
    cyls.append(('spice', col, KW + 0.008, -0.36 + i * 0.012, CH + 0.009, 0.004, 0.018))
box('oil_bottle_tray', 'wood', KW + 0.02, KW + 0.06, -0.355, -0.318, CH, CH + 0.003)
cyls.append(('oil', 'jar_y', KW + 0.032, -0.337, CH + 0.02, 0.005, 0.036))
cyls.append(('vinegar', 'glass', KW + 0.047, -0.337, CH + 0.018, 0.004, 0.032))
# 팬트리 (오픈 선반 4단)
PY0, PY1 = -0.31, KN
for y in (PY0 + 0.002, PY1 - 0.002):
    box('pantry_side', 'steel_d', KW, CB, y - 0.002, y + 0.002, 0, 0.26)
for z in (0.01, 0.075, 0.14, 0.205, 0.255):
    box('pantry_shelf', 'steel', KW, CB, PY0, PY1, z, z + 0.004)
for i, col in enumerate(('kraft', 'kraft', 'jar_r', 'kraft')):
    box('dry_box', col, KW + 0.008, KW + 0.05, PY0 + 0.01 + i * 0.036, PY0 + 0.04 + i * 0.036, 0.014, 0.05)
for i, col in enumerate(('jar_y', 'jar_g', 'jar_r', 'jar_y', 'jar_g', 'jar_r')):
    cyls.append(('jar', col, KW + 0.03, PY0 + 0.015 + i * 0.024, 0.079 + 0.013, 0.009, 0.026))
for i in range(6):
    cyls.append(('cup', 'cup', KW + 0.03, PY0 + 0.015 + i * 0.024, 0.144 + 0.009, 0.008, 0.018))
for y in (PY0 + 0.03, PY0 + 0.09):
    for k in range(6):
        cyls.append(('pantry_plate', 'bowl', KW + 0.035, y, 0.209 + k * 0.004 + 0.0015, 0.022, 0.003))

# 벽: 타일 백스플래시 + 선반 + 걸이 레일
quads.append(('backsplash', 'tile_wall', KW + 0.0006, (FY1 + PY0) / 2, (CH + 0.26) / 2, PY0 - FY1, 0.26 - CH, '+x', 0))
# 쓰레기통 (주방 통로 북쪽 끝)
cyls.append(('bin', 'bin', 0.02, -0.19, 0.035, 0.022, 0.07))
cyls.append(('bin_lid', 'grate', 0.02, -0.19, 0.072, 0.023, 0.004))

# ================================================================ N13 ~ N14 통로 위 칸막이 (로봇은 아래로 지나감)
# 칸막이 벽(x 0.40) 높이 30 그대로 북쪽 벽까지. 아래 17 cm는 비움 (Pinky 13 cm + 여유)
LINTEL_Z = 0.17
box('n13_lintel', 'wall', 0.40 - WALL_T / 2, 0.40 + WALL_T / 2, 0.33, Y1 - WALL_T / 2, LINTEL_Z, WALL_H)

# 바닥: E자 테이프 + 테두리 종이 (인쇄물)
EY = P1[1] + 0.035                 # 가로선 (통로를 가로지름)
ARMS = {'E_dong': 0.15, 'E_seo': 0.21, 'E_sae': 0.30}   # 동(정지, 바구니 6 cm 삽입) · 서 · 새(빠질 때)
tape('E_spine', 0.23, EY, 0.20, 0.019)
for n, x in ARMS.items():
    tape(n, x, EY - 0.075, 0.019, 0.15)
quads.append(('floor_frame', 'floor_frame', 0.23, EY - 0.075, 0.0003, 0.20, 0.25, 'up', math.pi / 2))
tape('bar_N15', CX, 0.157, 0.15, 0.02)
tape('bar_N16', CX, 0.034, 0.15, 0.02)

# ================================================================ 벽 인쇄물
quads.append(('menu', 'menu', 1.01, Y0 + WALL_T / 2 + 0.0006, 0.17, 0.30, 0.12, '+y', 0))
quads.append(('dock_D2', 'dock_D2', X0 + WALL_T / 2 + 0.0006, -0.021, 0.045, 0.14, 0.079, '+x', 0))
quads.append(('dock_D3', 'dock_D3', X0 + WALL_T / 2 + 0.0006, 0.125, 0.045, 0.14, 0.079, '+x', 0))
quads.append(('dock_D1', 'dock_D1', 1.67 + WALL_T / 2 + 0.0006, 0.386, 0.05, 0.16, 0.09, '+x', 0))

# 마커 = 실제 인쇄물 (pinkytorang_markers.pdf, AprilTag 36h11). 검은 사각형 한 변 = size, 흰 테두리 1칸 포함 1.25배
#   중심 높이 6.5 cm. 텍스처 tools/twin_tex/tag_<id>.png (PDF에서 잘라 OpenCV로 ID 확인)
def marker(n, size, x, y, face, z=0.065):
    tid = n[1:]
    quads.append((f'tag_{tid}', f'tag_{tid}', x, y, z, size * 1.25, size * 1.25, face, 0))


marker('m1', 0.025, X1 - 0.011, 0.386, '-x')
marker('m6', 0.05, 1.943, Y1 - 0.011, '-y')   # 교점(마커 1 줄 · 6 정면): N1에서 끝 벽을 볼 때 정면
marker('m2', 0.025, 0.389, -0.021, '-x')
marker('m3', 0.025, 0.389, 0.125, '-x')
marker('m4', 0.05, X0 + 0.011, 0.522, '+x')
marker('m5', 0.05, 0.15, Y1 - 0.011, '-y')


# ================================================================ 로봇 자리 표시 (Pinky 모델 대신 상자)
def robot(n, x, y, yaw):
    """미리보기 전용 자리 표시 (Gazebo에는 진짜 pinky_description 모델이 들어감). 치수 = pinky.urdf.xacro + tray1.stl (끝 x -0.145)"""
    c, s = math.cos(yaw), math.sin(yaw)

    def at(fx, fy):
        return x + c * fx - s * fy, y + s * fx + c * fy
    parts = [('body', 'robot', -0.02, 0.0, 0.10, 0.094, 0.012, 0.095),
             ('tray', 'basket', -0.1102, 0.0, 0.0694, 0.082, 0.016, 0.039)]
    for pn, col, fx, fy, lx, ly, z0, z1 in parts:
        px, py = at(fx, fy)
        boxes.append((f'{n}_{pn}', col, px, py, (z0 + z1) / 2, lx, ly, z1 - z0, yaw, False))
    for side in (-1, 1):
        px, py = at(0.0, side * 0.048)
        boxes.append((f'{n}_wheel', 'wheel', px, py, 0.028, 0.056, 0.012, 0.056, yaw, False))
    px, py = at(-0.017, 0.0)
    cyls.append((f'{n}_lidar', 'lidar', px, py, 0.116, 0.025, 0.042))


robot('pinky_P1', 0.155, P1[1], 0.0)         # 트레이 끝(-0.145) x = 0.01 -> 슬롯 안 6 cm
robot('pinky_D2', 0.029, -0.021, 0.0)
robot('pinky_D1', 1.824, 0.386, 0.0)
robot('pinky_N13', 0.40, 0.522, math.pi)      # 칸막이 아래 통과 중


# ================================================================ 출력
def hexrgb(h):
    h = h.lstrip('#')
    return tuple(int(h[i:i + 2], 16) / 255 for i in (0, 2, 4))


def write_quad_obj(path, name, w, h, face):
    """텍스처 사각형: 가운데가 원점. face에 맞춰 법선 · UV (보는 사람 기준 오른쪽 = 이미지 오른쪽)"""
    hw, hh = w / 2, h / 2
    if face == '+x':
        v = [(0, -hw, -hh), (0, hw, -hh), (0, hw, hh), (0, -hw, hh)]; n = (1, 0, 0)
    elif face == '-x':
        v = [(0, hw, -hh), (0, -hw, -hh), (0, -hw, hh), (0, hw, hh)]; n = (-1, 0, 0)
    elif face == '+y':
        v = [(hw, 0, -hh), (-hw, 0, -hh), (-hw, 0, hh), (hw, 0, hh)]; n = (0, 1, 0)
    elif face == '-y':
        v = [(-hw, 0, -hh), (hw, 0, -hh), (hw, 0, hh), (-hw, 0, hh)]; n = (0, -1, 0)
    else:   # up: 이미지 가로 = +x, 세로 = +y (pose yaw로 돌림)
        v = [(-hw, -hh, 0), (hw, -hh, 0), (hw, hh, 0), (-hw, hh, 0)]; n = (0, 0, 1)
    with open(path + '.mtl', 'w') as f:
        f.write(f'newmtl {name}\nKa 1 1 1\nKd 1 1 1\nmap_Kd {name}.png\n')
    with open(path + '.obj', 'w') as f:
        f.write(f'mtllib {name}.mtl\n')
        for p in v:
            f.write('v %.5f %.5f %.5f\n' % p)
        f.write('vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\n')
        f.write('vn %d %d %d\n' % n)
        f.write(f'usemtl {name}\nf 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n')


def sdf(out):
    os.makedirs(os.path.join(out, 'twin_meshes'), exist_ok=True)
    parts = []
    for i, (n, col, x, y, z, sx, sy, sz, yaw, coll) in enumerate(boxes):
        if n.startswith(('pinky_', 'tbl_')):
            continue
        r, g, b = hexrgb(C[col])
        cl = f'<collision name="c"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry></collision>' if coll else ''
        parts.append(f'<model name="{n}_{i}"><static>true</static><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose><link name="l">{cl}'
                     f'<visual name="v"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>'
                     f'<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient><diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse></material></visual></link></model>')
    for i, (n, col, x, y, z, rr, hh) in enumerate(cyls):
        if n.startswith('pinky_'):
            continue
        r, g, b = hexrgb(C[col])
        parts.append(f'<model name="{n}_c{i}"><static>true</static><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose><link name="l">'
                     f'<visual name="v"><geometry><cylinder><radius>{rr}</radius><length>{hh}</length></cylinder></geometry>'
                     f'<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient><diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse></material></visual></link></model>')
    for n, tex, x, y, z, w, h, face, yaw in quads:
        write_quad_obj(os.path.join(out, 'twin_meshes', n), n, w, h, face)
        shutil.copy(os.path.join(HERE, 'twin_tex', tex + '.png'), os.path.join(out, 'twin_meshes', n + '.png'))
        parts.append(f'<model name="print_{n}"><static>true</static><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose><link name="l">'
                     f'<visual name="v"><geometry><mesh><uri>model://pinkytorang_gz/worlds/twin_meshes/{n}.obj</uri></mesh></geometry></visual></link></model>')
    for k, (x, y) in T.items():
        parts.append(f'<include><uri>model://pinky_table</uri><name>table_{k}</name>'
                     f'<pose>{x:.4f} {y:.4f} 0 0 0 {math.pi / 2:.6f}</pose></include>')
    body = '\n    '.join(parts)
    with open(os.path.join(out, 'pinkytorang_twin.sdf'), 'w', encoding='utf-8') as f:
        f.write(f'''<?xml version="1.0"?>
<!-- 자동 생성: tools/gen_twin.py. 좌표 = map 프레임. 로봇은 sim.launch.py twin:=true 가 스폰 -->
<sdf version="1.9">
  <world name="pinkytorang">
    <physics name="2ms" type="ignored"><max_step_size>0.002</max_step_size><real_time_factor>1</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.85 0.85 0.85 1</background></scene>
    <light type="directional" name="sun"><pose>0 0 3 0 0 0</pose><direction>-0.3 0.2 -1</direction><diffuse>0.9 0.9 0.9 1</diffuse></light>
    <model name="floor"><static>true</static><pose>{(X0 + X1) / 2} {(Y0 + Y1) / 2} -0.005 0 0 0</pose><link name="l">
      <collision name="c"><geometry><box><size>3 2.5 0.01</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>3 2.5 0.01</size></box></geometry>
      <material><ambient>0.80 0.80 0.79 1</ambient><diffuse>0.80 0.80 0.79 1</diffuse></material></visual></link></model>
    {body}
  </world>
</sdf>
''')


if __name__ == '__main__':
    sdf(ARGS.out)
    if ARGS.json:
        json.dump({'boxes': boxes, 'cyls': cyls, 'quads': quads, 'colors': C, 'tables': T}, open(ARGS.json, 'w'))
    print(os.path.join(ARGS.out, 'pinkytorang_twin.sdf'), len(boxes), 'boxes', len(cyls), 'cyls', len(quads), 'prints',
          len(T), 'tables')
