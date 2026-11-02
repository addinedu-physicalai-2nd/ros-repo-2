// 핑키토랑 입구 키오스크 (7인치 1024x600 터치)
// - 좌석 배치도를 보여 주고, 손님이 빈 테이블을 고르면 POST /api/guide {table}
//   서버가 그 테이블을 예약하고 입구 로봇이 안내한다
// - 테이블 상태(빈자리 / 사용 중)는 매장 snapshot으로 실시간으로 바뀐다
// - 판단하지 않는다. 빈자리인지는 서버가 최종 확인한다 (이미 찼으면 409)
// - 배치도: 매장 관제 화면과 같은 방향 (입구 오른쪽 아래, 테이블 윗줄 T2 T4 T6 / 아랫줄 T1 T3 T5). 손님 화면이라 주방은 안 그림.
//   맵 좌표로 줄 · 칸 순서만 정하고, 테이블은 화면을 넉넉히 채우는 큰 정사각형으로 (실제 비율 아님, 누르기 쉽게)

const $ = (id) => document.getElementById(id);
const state = {
  board: { x: 0, y: 0, w: 2.28, h: 1.82 }, entrance: null, tables: [], map: null,
  selected: null, taskId: null, poll: null, resetTimer: null, resetTick: null, idleTimer: null,
};
const ENDED = ['DONE', 'FAILED', 'CANCELED'];
const ROW_TOL = 0.15;          // 이 거리(m) 안이면 같은 행 · 열로 봄
const IDLE_CLEAR_SEC = 30;     // 자리만 고르고 그대로 두면 이만큼 뒤 선택 풀기 (다음 손님)
const RESET_AFTER_GUIDE = 10;  // 로봇이 출발하면 이만큼 뒤 처음 화면 (다음 손님)
const RESET_AFTER_FAIL = 8;

$('guideBtn').onclick = requestGuide;
$('homeBtn').onclick = reset;
$('cancelBtn').onclick = cancelGuide;
window.addEventListener('resize', renderFloor);

// 매장 snapshot을 같이 받아서 테이블 상태를 바로 반영 (연결 표시도 겸함)
openSocket('/ws/store', (msg) => {
  if (msg.type !== 'snapshot' || !msg.tables) return;
  mergeTables(msg.tables);
}, (on) => {
  $('conn').className = 'conn ' + (on ? 'on' : 'off');
  $('conn').textContent = on ? '' : '연결 끊김';
  $('offline').hidden = on;
  if (on) loadLayout();
});

loadLayout();

async function loadLayout() {
  if (!state.map) {
    const m = await api('GET', '/api/map');      // 테이블 노드 · 픽업 위치 (배치도 모양)
    if (m.ok && m.data && Array.isArray(m.data.nodes)) state.map = m.data;
  }
  const r = await api('GET', '/api/tables');
  if (!r.ok) return;
  if (r.data.board) state.board = Object.assign({ x: 0, y: 0 }, r.data.board);
  state.entrance = r.data.entrance || null;
  state.tables = r.data.tables || [];
  renderFloor();
}

function mergeTables(list) {
  const byId = Object.fromEntries(list.map((t) => [t.id, t]));
  let changed = false;
  state.tables.forEach((t) => {
    const n = byId[t.id];
    if (n && n.state !== t.state) { t.state = n.state; changed = true; }
  });
  if (!changed) return;
  const sel = state.tables.find((t) => t.id === state.selected);
  if (sel && sel.state !== 'FREE' && !$('pickView').hidden) {
    state.selected = null;
    toast('고르신 자리가 방금 찼어요. 다른 자리를 골라 주세요');
  }
  renderFloor();
}

// ---------- 배치도 ----------
// 화면: 가로 = 맵 y, 세로 = 맵 x (매장 관제 화면과 같음 -> 입구 오른쪽 아래)
const TABLE_BODY_M = 0.08;   // T 노드(통로 쪽 모서리)에서 테이블 가운데까지 (맵 -y 쪽)

function toScreen(x, y) { return { sx: y, sy: x }; }

function tableCenters() {
  const nodes = state.map ? state.map.nodes : [];
  return state.tables.map((t) => {
    const n = nodes.find((m) => m.stop_kind === 'SERVE' && m.table === t.id);
    const p = n ? toScreen(n.x, n.y - TABLE_BODY_M) : toScreen(t.x, t.y);
    return { t, sx: p.sx, sy: p.sy };
  });
}

function renderFloor() {
  const floor = $('floor');
  if (!state.tables.length) { floor.innerHTML = '<p class="empty">배치도를 불러오는 중…</p>'; return; }
  // 실제 위치로 줄 · 칸 순서만 정하고(같은 줄 맞춤), 크기는 화면을 넉넉히 채우게 (실제 비율 아님)
  const pts = tableCenters();
  const colOf = rankOf(pts.map((p) => p.sx)), rowOf = rankOf(pts.map((p) => p.sy));
  const nCol = Math.max(...pts.map((p) => colOf(p.sx))) + 1, nRow = Math.max(...pts.map((p) => rowOf(p.sy))) + 1;

  const W = floor.clientWidth, H = floor.clientHeight;
  const pad = 18, doorH = state.entrance ? 70 : 0;              // 아래 띠: 입구 통로 (칸막이 아래)
  const cellW = (W - pad * 2) / nCol, cellH = (H - pad * 2 - doorH) / nRow;
  const size = Math.min(cellW, cellH) * 0.8;
  const fs = Math.round(Math.min(size * 0.26, 40));

  // 입구 칸막이 벽: 테이블 줄과 입구(현위치) 사이. 오른쪽 벽에서 시작해 ENTRY_WALL.y0 까지 (그 왼쪽이 홀로 들어가는 길)
  //   가로 위치는 실제 y를 테이블 열 가운데 사이로 보간해서 정함
  const colReal = [...new Set(pts.map((p) => colOf(p.sx)))].sort((a, b) => a - b)
    .map((c) => { const g = pts.filter((p) => colOf(p.sx) === c); return [g.reduce((a, p) => a + p.sx, 0) / g.length, pad + (c + 0.5) * cellW]; });
  const realToPx = (v) => {
    let i = 0;
    while (i < colReal.length - 2 && v > colReal[i + 1][0]) i++;
    const [a, pa] = colReal[i], [b, pb] = colReal[Math.min(i + 1, colReal.length - 1)];
    return b === a ? pa : pa + (v - a) / (b - a) * (pb - pa);
  };
  const r = 16, wy = H - doorH - 4;                                  // 칸막이 높이 (아래 띠 위)
  const wx0 = Math.max(pad, realToPx(ENTRY_WALL.y0));
  const stripY = (wy + H) / 2;                                        // 입구 띠 가운데
  const pinX = W - pad - 30;
  let html = `<svg class="walls" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}">
    <rect class="floor-fill" x="1" y="1" width="${W - 2}" height="${H - 2}" rx="${r}"/>
    ${state.entrance ? `<line class="wall" x1="${f0(wx0)}" y1="${f0(wy)}" x2="${W - 6}" y2="${f0(wy)}"/>` : ''}
  </svg>`;
  html += pts.map((p) => {
    const t = p.t, free = t.state === 'FREE', sel = t.id === state.selected;
    const label = sel ? '고른 자리' : free ? '빈자리' : '사용 중';
    const cx = pad + (colOf(p.sx) + 0.5) * cellW, cy = pad + (rowOf(p.sy) + 0.5) * cellH;
    return `<button class="tbl ${sel ? 'selected' : ''}" data-id="${esc(t.id)}" ${free ? '' : 'disabled'}
      style="left:${cx - size / 2}px; top:${cy - size / 2}px; width:${size}px; height:${size}px; font-size:${fs}px">
      ${esc(t.id)}<small>${label}</small></button>`;
  }).join('');
  if (state.entrance) {
    html += `<div class="here" style="left:${pinX}px; top:${stripY}px">
      <span class="pin"><i></i></span><span class="here-text"><b>현위치</b>입구</span></div>`;
  }
  floor.innerHTML = html;
  floor.querySelectorAll('.tbl').forEach((b) => { b.onclick = () => selectTable(b.dataset.id); });
  renderPanel();
}

// 값을 ROW_TOL 안에서 묶어 0, 1, 2 ... 순서로 (같은 줄 · 칸)
const f0 = (v) => v.toFixed(0);
// 입구 칸막이 벽 (맵 v5 실측: x 1.68 m 에서 y 0.76 (바깥 벽) ~ -0.44 m). 화면에서는 테이블 줄과 현위치 사이 가로 벽
const ENTRY_WALL = { x: 1.68, y0: -0.44, y1: 0.76 };

function rankOf(values) {
  const sorted = [...values].sort((a, b) => a - b);
  const mids = [];
  let g = null;
  sorted.forEach((v) => {
    if (!g || v - g.last > ROW_TOL) { g = { last: v, sum: v, n: 1 }; mids.push(g); }
    else { g.last = v; g.sum += v; g.n += 1; }
  });
  const centers = mids.map((m) => m.sum / m.n);
  return (v) => centers.reduce((best, c, i) => (Math.abs(c - v) < Math.abs(centers[best] - v) ? i : best), 0);
}

function renderPanel() {
  const sel = state.selected;
  const free = state.tables.filter((t) => t.state === 'FREE').length;
  $('freeCount').textContent = state.tables.length ? `빈자리 ${free} / ${state.tables.length}` : '';
  $('selCard').hidden = !sel;
  $('selName').textContent = sel || '';
  $('guideBtn').disabled = !sel;
  $('guideBtn').hidden = free === 0;   // 빈자리가 없으면 누를 것도 없음
  $('guideBtn').textContent = sel ? '이 자리로 안내 받기' : '자리를 골라 주세요';
  if (free === 0) {
    $('pickTitle').innerHTML = '지금은 빈자리가<br>없어요';
    $('pickSub').textContent = '자리가 나면 바로 표시돼요. 잠시만 기다려 주세요';
  } else {
    $('pickTitle').innerHTML = '원하시는 자리를<br>골라 주세요';
    $('pickSub').textContent = '빈자리를 누르면 로봇이 안내해 드려요';
  }
}

function selectTable(id) {
  state.selected = state.selected === id ? null : id;
  clearTimeout(state.idleTimer);
  if (state.selected) state.idleTimer = setTimeout(() => { state.selected = null; renderFloor(); }, IDLE_CLEAR_SEC * 1000);
  renderFloor();
}

async function requestGuide() {
  if (!state.selected) return;
  clearTimeout(state.idleTimer);
  $('guideBtn').disabled = true;
  $('guideBtn').textContent = '자리를 잡는 중…';
  const r = await api('POST', '/api/guide', { table: state.selected });
  if (!r.ok) {
    if (r.status === 409) { toast('방금 다른 손님이 고른 자리예요. 다른 자리를 골라 주세요'); state.selected = null; loadLayout(); }
    else toast(r.status === 0 ? '서버에 연결할 수 없어요' : '요청하지 못했어요. 직원을 불러 주세요');
    renderPanel();
    return;
  }
  state.taskId = r.data.task_id;
  state.selected = null;   // 이제 이 손님 자리는 예약됨 (snapshot에서 '찼다'고 알리지 않게)
  showStatus(r.data);
  clearInterval(state.poll);
  state.poll = setInterval(poll, 2000);
}

async function poll() {
  if (state.taskId == null) return;
  const r = await api('GET', `/api/guide/${state.taskId}`);
  if (r.ok) showStatus(r.data);
}

function setSteps(n) {   // n = 지금 단계 (1~3), 4 = 다 끝
  document.querySelectorAll('#steps li').forEach((li) => {
    const s = Number(li.dataset.step);
    li.className = s < n ? 'done' : s === n ? 'now' : '';
  });
}

function showStatus(g) {
  $('pickView').hidden = true;
  $('resultView').hidden = false;
  const badge = $('resultBadge');
  badge.className = 'result-badge';
  badge.textContent = g.table;
  $('steps').hidden = false;

  if (g.stage === 'CANCELED' || g.stage === 'FAILED') {
    badge.className = 'result-badge warn';
    badge.textContent = '!';
    $('resultTitle').textContent = g.stage === 'CANCELED' ? '안내를 취소했어요' : '안내하지 못했어요';
    $('resultSub').textContent = g.stage === 'CANCELED' ? '다시 고르시려면 처음으로를 눌러 주세요' : '직원을 불러 주세요';
    $('steps').hidden = true;
    $('cancelBtn').hidden = true;
    stopPolling();
    autoReset(RESET_AFTER_FAIL);
    return;
  }
  if (g.stage === 'TO_TABLE' || g.stage === 'AT_TABLE' || ENDED.includes(g.stage) || g.stage === 'TO_DOCK') {
    $('resultTitle').textContent = `${g.table} 자리로 안내할게요`;
    $('resultSub').textContent = '로봇 뒤를 따라와 주세요';
    setSteps(3);
    $('cancelBtn').hidden = true;
    stopPolling();          // 다음 손님이 쓸 수 있게 잠시 뒤 처음 화면으로
    autoReset(RESET_AFTER_GUIDE);
    return;
  }
  // WAITING / ASSIGNED: 입구 로봇을 기다리는 중
  $('resultTitle').textContent = `${g.table} 자리를 잡아 뒀어요`;
  $('resultSub').textContent = '안내 로봇이 곧 앞으로 나와요. 잠시만 기다려 주세요';
  setSteps(2);
  $('cancelBtn').hidden = false;
  $('resetIn').textContent = '';
}

async function cancelGuide() {
  if (state.taskId == null) return;
  $('cancelBtn').disabled = true;
  const r = await api('POST', `/api/tasks/${state.taskId}/cancel`);
  $('cancelBtn').disabled = false;
  if (r.ok || r.status === 409) reset();
  else toast('취소하지 못했어요. 직원을 불러 주세요');
}

function stopPolling() { clearInterval(state.poll); state.poll = null; }

function autoReset(sec) {
  clearTimeout(state.resetTimer);
  clearInterval(state.resetTick);
  let left = sec;
  $('resetIn').textContent = `${left}초`;
  state.resetTick = setInterval(() => { left -= 1; $('resetIn').textContent = left > 0 ? `${left}초` : ''; }, 1000);
  state.resetTimer = setTimeout(reset, sec * 1000);
}

function reset() {
  stopPolling();
  clearTimeout(state.resetTimer);
  clearInterval(state.resetTick);
  clearTimeout(state.idleTimer);
  $('resetIn').textContent = '';
  state.taskId = null;
  state.selected = null;
  $('resultView').hidden = true;
  $('pickView').hidden = false;
  loadLayout();
}
