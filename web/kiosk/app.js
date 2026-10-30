// 핑키토랑 입구 키오스크 (7인치 1024x600 터치)
// - 좌석 배치도를 보여 주고, 손님이 빈 테이블을 고르면 POST /api/guide {table}
//   서버가 그 테이블을 예약하고 입구 로봇이 안내한다
// - 테이블 상태(빈자리 / 사용 중)는 매장 snapshot으로 실시간으로 바뀐다
// - 판단하지 않는다. 빈자리인지는 서버가 최종 확인한다 (이미 찼으면 409)
// - 배치도: 테이블 맵 좌표를 실제 크기로 그리지 않고, 위치 순서(행 · 열)대로 큰 칸에 놓는다 (작은 화면에서 누르기 쉽게)
//   입구는 실제로 있는 쪽 가장자리에. board.flip이면 매장 그림처럼 180도 돌려서

const $ = (id) => document.getElementById(id);
const state = {
  board: { x: 0, y: 0, w: 2.28, h: 1.82 }, entrance: null, tables: [],
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

// 맵 좌표 -> 화면 방향 (오른쪽 = +sx, 아래 = +sy). flip이면 180도
function screenPos(p) {
  const f = state.board.flip;
  return { sx: f ? -p.x : p.x, sy: f ? p.y : -p.y };
}

// 값들을 ROW_TOL 안에서 묶어 0, 1, 2 ... 순서로
function clusterIndex(values) {
  const sorted = [...values].sort((a, b) => a - b);
  const centers = [];
  sorted.forEach((v) => {
    if (!centers.length || v - centers[centers.length - 1].last > ROW_TOL) centers.push({ last: v, sum: v, n: 1 });
    else { const c = centers[centers.length - 1]; c.last = v; c.sum += v; c.n += 1; }
  });
  const mids = centers.map((c) => c.sum / c.n);
  return { count: mids.length, of: (v) => mids.reduce((best, m, i) => (Math.abs(m - v) < Math.abs(mids[best] - v) ? i : best), 0) };
}

function renderFloor() {
  const floor = $('floor');
  if (!state.tables.length) { floor.innerHTML = '<p class="empty">배치도를 불러오는 중…</p>'; return; }
  const pts = state.tables.map((t) => Object.assign({ t }, screenPos(t)));
  const cols = clusterIndex(pts.map((p) => p.sx));
  const rows = clusterIndex(pts.map((p) => p.sy));

  // 입구: 테이블 묶음 바깥 어느 쪽에 있나 (가장 많이 벗어난 쪽)
  let door = null;
  if (state.entrance) {
    const e = screenPos(state.entrance);
    const minX = Math.min(...pts.map((p) => p.sx)), maxX = Math.max(...pts.map((p) => p.sx));
    const minY = Math.min(...pts.map((p) => p.sy)), maxY = Math.max(...pts.map((p) => p.sy));
    const out = { left: minX - e.sx, right: e.sx - maxX, top: minY - e.sy, bottom: e.sy - maxY };
    const side = Object.keys(out).reduce((a, b) => (out[b] > out[a] ? b : a));
    door = { side, along: side === 'left' || side === 'right' ? rows.of(e.sy) : cols.of(e.sx) };
  }
  const dl = door && door.side === 'left', dr = door && door.side === 'right';
  const dt = door && door.side === 'top', db = door && door.side === 'bottom';
  floor.style.gridTemplateColumns = `${dl ? '58px ' : ''}repeat(${cols.count}, 1fr)${dr ? ' 58px' : ''}`;
  floor.style.gridTemplateRows = `${dt ? '48px ' : ''}repeat(${rows.count}, 1fr)${db ? ' 48px' : ''}`;
  const c0 = dl ? 2 : 1, r0 = dt ? 2 : 1;

  let html = pts.map((p) => {
    const t = p.t, free = t.state === 'FREE', sel = t.id === state.selected;
    const label = sel ? '고른 자리' : free ? '빈자리' : '사용 중';
    return `<button class="tbl ${sel ? 'selected' : ''}" data-id="${esc(t.id)}" ${free ? '' : 'disabled'}
      style="grid-column:${c0 + cols.of(p.sx)}; grid-row:${r0 + rows.of(p.sy)}">${esc(t.id)}<small>${label}</small></button>`;
  }).join('');
  if (door) {
    const vertical = door.side === 'left' || door.side === 'right';
    const col = dl ? 1 : dr ? c0 + cols.count : c0 + door.along;
    const row = dt ? 1 : db ? r0 + rows.count : r0 + door.along;
    html += `<span class="door ${vertical ? 'v' : ''}" style="grid-column:${col}; grid-row:${row}">입구 · 지금 여기</span>`;
  }
  floor.innerHTML = html;
  floor.querySelectorAll('.tbl').forEach((b) => { b.onclick = () => selectTable(b.dataset.id); });
  renderPanel();
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
