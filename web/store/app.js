// 핑키토랑 매장관리 UI
// 규칙
// - 판단하지 않는다. 화면은 마지막으로 받은 snapshot과 confirm_needed를 그대로 그린다.
//   (취소 가능 여부처럼 버튼을 보여 줄지만 단계 이름으로 정하고, 최종 판단은 서버가 409로 한다)
// - 버튼은 응답이 올 때까지 비활성화, 409면 조용히 닫는다.
// - 로봇·테이블·노드 ID는 문자열 그대로 표시만 한다.

const ROBOT_COLORS = { pinky1: '#e5487f', pinky2: '#3b82f6', pinky3: '#8b5cf6' };   // 맵·상태판 공통 (픽업 초록 · 도크 주황 · 장애물 빨강과 안 겹치게)
const ROBOT_STATE = { IDLE: '대기', MOVING: '이동 중', PAUSED: '일시정지', BLOCKED: '막힘', ERROR: '오류' };
const STAGE_TEXT = {
  COOKING: '조리 중', WAITING: '대기', ASSIGNED: '배정됨', TO_PICKUP: '픽업 창구로 이동', AT_PICKUP: '픽업 창구 도착',
  TO_TABLE: '테이블로 이동', AT_TABLE: '테이블 도착', TO_DOCK: '도킹 복귀', DONE: '완료', FAILED: '실패', CANCELED: '취소',
};
const KIND_TEXT = { SERVE: '서빙', GUIDE: '안내' };
const TABLE_STATE = { FREE: '빈자리', RESERVED: '안내 중', OCCUPIED: '사용 중' };
const ENDED = ['DONE', 'FAILED', 'CANCELED'];

// 주방이 누르는 확인 (나머지는 손님 화면 버튼이라 여기서는 표시만)
const KITCHEN_CONFIRM = { LOADED: { kind: '픽업 창구 · 음식 적재', btn: '적재 완료' } };
const GUEST_CONFIRM = { RECEIVED: '손님 수령 대기', SEATED: '손님 착석 대기' };

const state = { robots: [], tasks: [], locks: [], tables: [], obstacles: [], confirms: {}, alerts: [] };
const $ = (id) => document.getElementById(id);
const robotColor = (id) => ROBOT_COLORS[id] || '#a1a1a6';

// 같은 내용이면 다시 그리지 않는다 (5 Hz snapshot에 버튼이 계속 바뀌면 깜빡이고 클릭이 씹힘)
const lastHtml = {};
function setHtml(id, html) {
  if (lastHtml[id] === html) return false;
  lastHtml[id] = html;
  $(id).innerHTML = html;
  return true;
}

// 취소 버튼을 보여 줄 단계 (서버 규칙과 같음: 서빙은 조리 중, 안내는 착석 전)
function cancellable(t) {
  if (!t) return false;
  if (t.kind === 'SERVE') return t.stage === 'COOKING';
  return ['WAITING', 'ASSIGNED', 'TO_TABLE'].includes(t.stage);
}

// ---------- 시작 ----------
$('stopAllBtn').addEventListener('click', () => control('all', 'PAUSE', $('stopAllBtn')));
$('historyBtn').addEventListener('click', () => { $('historyModal').hidden = false; updateCsvLink(); });
$('historyClose').addEventListener('click', () => { $('historyModal').hidden = true; });
$('historyModal').addEventListener('click', (e) => { if (e.target.id === 'historyModal') $('historyModal').hidden = true; });
$('historyForm').addEventListener('submit', (e) => { e.preventDefault(); searchHistory(); });
['hFrom', 'hTo', 'hType'].forEach((id) => $(id).addEventListener('change', updateCsvLink));

// 버튼은 한 곳에서 처리 (다시 그려도 핸들러가 사라지지 않게)
document.addEventListener('click', (e) => {
  const b = e.target.closest('button[data-act]');
  if (!b || b.disabled) return;
  const act = b.dataset.act;
  if (act === 'confirm') pressConfirm(b);
  else if (act === 'ready') post(b, `/api/tasks/${b.dataset.task}/ready`, '조리 완료');
  else if (act === 'cancel') {
    if (confirm(`작업 #${b.dataset.task}을(를) 취소할까요?`)) post(b, `/api/tasks/${b.dataset.task}/cancel`, '취소했어요', '취소할 수 없는 단계예요');
  } else if (act === 'clear') post(b, `/api/tables/${encodeURIComponent(b.dataset.table)}/clear`, `${b.dataset.table} 빈자리`);
  else if (act === 'occupy') post(b, `/api/tables/${encodeURIComponent(b.dataset.table)}/occupy`, `${b.dataset.table} 사용 중`);
  else if (act === 'robot') {
    if (b.dataset.cmd === 'CANCEL' && !confirm(`${b.dataset.robot}의 작업을 취소할까요?`)) return;
    control(b.dataset.robot, b.dataset.cmd, b);
  }
});

setInterval(tick, 1000);
tick();

openSocket('/ws/store', onMessage, (on) => {
  $('conn').className = 'conn ' + (on ? 'on' : 'off');
  $('conn').textContent = on ? '서버 연결됨' : '서버 끊김 · 다시 연결 중';
  if (!on) { state.confirms = {}; renderConfirms(); }   // 다시 연결되면 서버가 현재 상태를 다시 보내준다
});

function tick() {
  $('clock').textContent = new Date().toLocaleTimeString('ko-KR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
  let changed = false;
  Object.values(state.confirms).forEach((c) => {     // 손님 카운트다운 표시용
    if (c.left > 0) { c.left -= 1; changed = true; }
  });
  if (changed) renderTasks();
}

// ---------- 서버 알림 ----------
function onMessage(msg) {
  switch (msg.type) {
    case 'snapshot':
      state.robots = msg.robots || [];
      state.tasks = msg.tasks || [];
      state.locks = msg.locks || [];
      state.tables = msg.tables || [];
      state.obstacles = msg.obstacles || [];   // (예정) 서버가 장애물로 잠근 노드 · 길
      renderRobots();
      renderTasks();
      renderTables();
      renderConfirms();
      renderCtl();
      break;
    case 'confirm_needed':
      state.confirms[msg.task_id] = { ...msg, left: msg.countdown_sec || 0 };
      renderConfirms();
      renderTasks();
      break;
    case 'confirm_closed':
      delete state.confirms[msg.task_id];
      renderConfirms();
      renderTasks();
      break;
    case 'alert':
      pushAlert(msg);
      break;
  }
}

// ---------- 확인 필요 ----------
function taskById(id) { return state.tasks.find((t) => t.task_id === id); }
function itemsText(t) {
  if (!t) return '';
  if (t.kind === 'GUIDE') return '';
  return t.items ? t.items.map((i) => `${i.name ?? i.menu_id} ${i.qty}`).join(', ') : '';
}

function renderConfirms() {
  const list = Object.values(state.confirms).filter((c) => KITCHEN_CONFIRM[c.confirm]);
  $('confirmCount').textContent = list.length || '';
  setHtml('confirms', list.map((c) => {
    const def = KITCHEN_CONFIRM[c.confirm];
    const t = taskById(c.task_id) || {};
    const table = c.table ?? t.table ?? '';
    return `
      <div class="confirm ${esc(c.confirm)}">
        <div class="confirm-kind">${def.kind}</div>
        <div class="confirm-table">${esc(table || `작업 #${c.task_id}`)}</div>
        <div class="confirm-items">${esc(itemsText(t))}</div>
        ${t.robot_id ? `<div class="confirm-robot"><span class="dot" style="background:${robotColor(t.robot_id)}"></span>${esc(t.robot_id)}</div>` : ''}
        <button class="btn btn-primary" data-act="confirm" data-task="${esc(c.task_id)}" data-confirm="${esc(c.confirm)}">${def.btn}</button>
      </div>`;
  }).join(''));
}

async function pressConfirm(btn) {
  const taskId = btn.dataset.task;
  btn.disabled = true;
  const r = await api('POST', `/api/tasks/${encodeURIComponent(taskId)}/confirm`, { confirm: btn.dataset.confirm });
  if (r.ok || r.status === 409) {
    delete state.confirms[taskId];      // 409 = 이미 처리됨, 조용히 닫는다
    renderConfirms();
  } else {
    btn.disabled = false;
    toast(r.status === 0 ? '서버에 연결할 수 없어요' : '처리하지 못했어요. 다시 눌러주세요');
  }
}

// 버튼 하나로 POST 보내기 (409면 bad 문구)
async function post(btn, path, okText, badText) {
  btn.disabled = true;
  const r = await api('POST', path);
  if (r.ok) toast(okText);
  else if (r.status === 409) toast(badText || '이미 처리됐어요');
  else toast(r.status === 0 ? '서버에 연결할 수 없어요' : '처리하지 못했어요');
  btn.disabled = false;
  delete lastHtml.colCook; delete lastHtml.colWait; delete lastHtml.colRun; delete lastHtml.tables;  // 다음 snapshot에 새로 그림
}

// ---------- 주문·작업판 ----------
function renderTasks() {
  const cook = state.tasks.filter((t) => t.stage === 'COOKING');
  const wait = state.tasks.filter((t) => t.stage === 'WAITING');
  const done = state.tasks.filter((t) => ENDED.includes(t.stage));
  const run = state.tasks.filter((t) => !['COOKING', 'WAITING'].includes(t.stage) && !ENDED.includes(t.stage));
  [['colCook', 'cntCook', cook], ['colWait', 'cntWait', wait], ['colRun', 'cntRun', run], ['colDone', 'cntDone', done]]
    .forEach(([col, cnt, list]) => {
      $(cnt).textContent = list.length || '';
      setHtml(col, list.sort((a, b) => (col === 'colDone' ? b.task_id - a.task_id : a.task_id - b.task_id)).map(card).join(''));
    });
}

function card(t) {
  const c = state.confirms[t.task_id];
  const guestWait = c && GUEST_CONFIRM[c.confirm];
  const bad = t.stage === 'FAILED' || t.stage === 'CANCELED';
  const tableText = t.table || '';
  return `
    <div class="card">
      <div class="card-top">
        <span class="card-table">${esc(tableText)}</span>
        <span class="kind ${esc(t.kind)}">${KIND_TEXT[t.kind] || esc(t.kind)}</span>
        <span class="card-id num">#${esc(t.task_id)}</span>
      </div>
      ${itemsText(t) ? `<div class="card-items">${esc(itemsText(t))}</div>` : ''}
      <div class="card-foot">
        <span class="stage ${bad ? 'bad' : ''}">${STAGE_TEXT[t.stage] || esc(t.stage)}</span>
        ${guestWait ? `<span class="wait-badge num">${guestWait}${c.left > 0 ? ` · ${c.left}초` : ''}</span>` : ''}
        ${t.robot_id ? `<span class="robot-tag"><span class="dot" style="background:${robotColor(t.robot_id)}"></span>${esc(t.robot_id)}</span>` : ''}
      </div>
      ${t.stage === 'COOKING' || cancellable(t) ? `
      <div class="card-actions">
        ${t.stage === 'COOKING' ? `<button class="btn btn-primary" data-act="ready" data-task="${esc(t.task_id)}">조리 완료</button>` : ''}
        ${cancellable(t) ? `<button class="btn btn-ghost" data-act="cancel" data-task="${esc(t.task_id)}">취소</button>` : ''}
      </div>` : ''}
    </div>`;
}

// ---------- 테이블 ----------
function renderTables() {
  setHtml('tables', state.tables.map((tb) => `
    <div class="table-cell ${esc(tb.state)}">
      <div class="table-top"><span class="table-name">${esc(tb.id)}</span></div>
      <div class="table-state">${TABLE_STATE[tb.state] || esc(tb.state)}</div>
      ${tb.state === 'OCCUPIED' ? `<button class="btn btn-dark" data-act="clear" data-table="${esc(tb.id)}">정리 완료</button>` : ''}
      ${tb.state === 'FREE' ? `<button class="btn btn-ghost" data-act="occupy" data-table="${esc(tb.id)}">착석 처리</button>` : ''}
    </div>`).join('') || '<div class="empty">테이블 정보가 없어요</div>');
}

// ---------- 로봇 상태판 ----------
function renderRobots() {
  setHtml('robots', state.robots.map((r) => {
    const online = r.online !== false;
    const st = online ? r.state : 'OFFLINE';
    const t = r.task_id != null ? taskById(r.task_id) : null;
    const batt = Math.round(r.battery ?? 0);
    return `
      <div class="robot ${online ? '' : 'offline'}" style="--rc:${robotColor(r.id)}">
        <div class="robot-head">
          <span class="robot-name">${esc(r.id)}</span>
          <span class="badge ${esc(st)}">${online ? (ROBOT_STATE[r.state] || esc(r.state)) : '통신 끊김'}</span>
          ${r.charging ? '<span class="badge CHARGING">충전 중</span>' : ''}
        </div>
        <dl class="robot-meta">
          <dt>작업</dt><dd>${t ? `${esc(t.table || '')} ${KIND_TEXT[t.kind] || ''} · ${STAGE_TEXT[t.stage] || esc(t.stage)}` : (r.task_id != null ? `#${esc(r.task_id)}` : '없음')}</dd>
          <dt>마지막 노드</dt><dd class="num">${esc(r.last_node || '-')}</dd>
          <dt>다음 · 목적지</dt><dd class="num">${r.goal_node ? `${esc(r.next_node || '-')} → ${esc(r.goal_node)}` : '-'}</dd>
          <dt>배터리</dt><dd class="batt"><span class="batt-bar"><i class="${batt < 30 ? 'low' : ''}" style="width:${Math.max(0, Math.min(100, batt))}%"></i></span><span class="num">${batt}%</span></dd>
        </dl>
        <div class="robot-actions">
          ${r.state === 'PAUSED'
            ? `<button class="btn btn-ghost" data-act="robot" data-robot="${esc(r.id)}" data-cmd="RESUME">재개</button>`
            : `<button class="btn btn-ghost" data-act="robot" data-robot="${esc(r.id)}" data-cmd="PAUSE">일시정지</button>`}
          <button class="btn btn-ghost" data-act="robot" data-robot="${esc(r.id)}" data-cmd="CANCEL" ${cancellable(t) ? '' : 'disabled'}
            title="안내는 착석 전까지만 취소할 수 있어요. 조리가 끝난 서빙은 취소할 수 없어요">작업 취소</button>
        </div>
      </div>`;
  }).join('') || '<div class="empty">로봇 정보가 아직 없어요</div>');
}

async function control(robotId, command, btn) {
  btn.disabled = true;
  const r = await api('POST', `/api/robots/${encodeURIComponent(robotId)}/control`, { command });
  btn.disabled = false;
  if (r.ok) toast(robotId === 'all' ? '전체 정지를 보냈어요' : `${robotId} · ${command}`);
  else if (r.status === 409) toast('지금은 취소할 수 없는 단계예요');
  else toast(r.status === 0 ? '서버에 연결할 수 없어요' : '명령을 보내지 못했어요');
  delete lastHtml.robots;
}

// ---------- 알림 ----------
function pushAlert(a) {
  const id = Date.now() + Math.random();
  state.alerts.unshift({ ...a, id, time: new Date() });
  state.alerts = state.alerts.slice(0, 5);
  renderAlerts();
  if (a.level === 'INFO') setTimeout(() => dismissAlert(id), 6000);
}
function dismissAlert(id) {
  state.alerts = state.alerts.filter((a) => a.id !== id);
  renderAlerts();
}
function renderAlerts() {
  $('alerts').innerHTML = state.alerts.map((a) => `
    <div class="alert ${esc(a.level)}">
      ${a.robot_id ? `<b>${esc(a.robot_id)}</b>` : ''}<span>${esc(a.message)}</span>
      <span class="time num">${a.time.toLocaleTimeString('ko-KR')}</span>
      <button data-id="${a.id}" aria-label="닫기">✕</button>
    </div>`).join('');
  $('alerts').querySelectorAll('button').forEach((b) => { b.onclick = () => dismissAlert(Number(b.dataset.id)); });
}

// ---------- 이력 ----------
function historyQuery() {
  const p = new URLSearchParams();
  if ($('hFrom').value) p.set('from', new Date($('hFrom').value).toISOString());
  if ($('hTo').value) p.set('to', new Date($('hTo').value).toISOString());
  if ($('hType').value) p.set('type', $('hType').value);
  return p;
}
function updateCsvLink() {
  const p = historyQuery();
  p.set('format', 'csv');
  $('csvLink').href = '/api/history?' + p.toString();
}
async function searchHistory() {
  updateCsvLink();
  const r = await api('GET', '/api/history?' + historyQuery().toString());
  if (!r.ok) { toast('이력을 불러오지 못했어요'); return; }
  const rows = Array.isArray(r.data) ? r.data : (r.data.items || []);
  $('historyRows').innerHTML = rows.map((h) => `
    <tr><td class="num">${esc(h.time ? new Date(h.time).toLocaleString('ko-KR') : '')}</td>
        <td>${esc(h.type)}</td><td>${esc(h.message ?? '')}</td></tr>`).join('')
    || '<tr><td colspan="3" class="empty">결과가 없어요</td></tr>';
}


// ---------- 관제 탭 ----------
// 노드 그래프(/api/map) 위에 로봇마다 잠근 길(committed, 진한 선) · 예정 길(plan, 점선) · 잠근 노드(locks, 테두리) ·
// 목적지 · 현재 위치를 색으로 그린다. 장애물(snapshot.obstacles: [{node} | {from, to}])은 빨간색.
// 화면 방향: 입구(D1)가 오른쪽 아래, 픽업(P1)이 왼쪽 위 (테이블 윗줄 T2 T4 T6 / 아랫줄 T1 T3 T5)
// 노드는 정사각형 칸 격자에 줄 맞춰 놓는다 (CTL_GRID: [열, 행]). 목록에 없는 노드는 실제 좌표로 칸 위치를 추정
//   열 = 세로 통로 (N1~N4 | N5~N7 | N8~N10 | N11~N13), 행 = 위(주방)에서 아래(입구)로. 테이블 T는 통로 위 반 칸, 몸통은 왼쪽
const CTL_GRID = {
  D2: [2, -1], D3: [2.5, -1],
  P1: [0, 0], P2: [1, 0], N16: [2, 0], N15: [2.5, 0], N14: [3, 0],
  N4: [0, 1], N7: [1, 1], N10: [2, 1], N13: [3, 1],
  T2: [0, 1.5], T4: [1, 1.5], T6: [2, 1.5],
  N3: [0, 2], N6: [1, 2], N9: [2, 2], N12: [3, 2],
  T1: [0, 2.5], T3: [1, 2.5], T5: [2, 2.5],
  N2: [0, 3], N5: [1, 3], N8: [2, 3], N11: [3, 3],
  N1: [0, 4], D1: [3, 4],
};
const CTL = { map: null, nodes: {}, grid: {}, S: 120, SX: 120, SY: 120, PAD: 56, c0: 0, r0: 0, w: 0, h: 0, fit: null };

document.querySelectorAll('.tab').forEach((b) => b.addEventListener('click', () => showTab(b.dataset.tab)));
function showTab(tab) {
  document.querySelectorAll('.tab').forEach((b) => b.classList.toggle('on', b.dataset.tab === tab));
  $('opsView').hidden = tab !== 'ops';
  $('ctlView').hidden = tab !== 'ctl';
  try { localStorage.setItem('storeTab', tab); } catch (_) { /* 저장 안 돼도 됨 */ }
  if (tab === 'ctl') { renderCtl(); startAnim(); }
}
try { if (localStorage.getItem('storeTab') === 'ctl') showTab('ctl'); } catch (_) { /* 기본 운영 탭 */ }

loadMap();
async function loadMap() {
  const r = await api('GET', '/api/map');
  if (!r.ok || !r.data || !Array.isArray(r.data.nodes)) { setTimeout(loadMap, 3000); return; }
  CTL.map = r.data;
  CTL.nodes = {};
  r.data.nodes.forEach((n) => { CTL.nodes[n.id] = n; });
  // 실제 좌표 -> 칸: 열 = a·y + b, 행 = c·x + d (격자에 있는 노드로 최소제곱. 목록에 없는 노드 · 로봇 위치 추정용)
  const known = r.data.nodes.filter((n) => CTL_GRID[n.id]);
  const lsq = (pts) => {
    const n = pts.length, sx = pts.reduce((a, p) => a + p[0], 0), sy = pts.reduce((a, p) => a + p[1], 0);
    const sxx = pts.reduce((a, p) => a + p[0] * p[0], 0), sxy = pts.reduce((a, p) => a + p[0] * p[1], 0);
    const k = (n * sxy - sx * sy) / ((n * sxx - sx * sx) || 1);
    return [k, (sy - k * sx) / n];
  };
  CTL.fit = { col: lsq(known.map((n) => [n.y, CTL_GRID[n.id][0]])), row: lsq(known.map((n) => [n.x, CTL_GRID[n.id][1]])) };
  CTL.grid = {};
  r.data.nodes.forEach((n) => { CTL.grid[n.id] = CTL_GRID[n.id] || toCell(n.x, n.y); });
  const cells = Object.values(CTL.grid);
  CTL.c0 = Math.min(...cells.map((c) => c[0])) - 1;     // 왼쪽 한 칸: 테이블 몸통
  CTL.r0 = Math.min(...cells.map((c) => c[1])) - 0.5;
  const cols = Math.max(...cells.map((c) => c[0])) + 0.5 - CTL.c0;
  const rows = Math.max(...cells.map((c) => c[1])) + 0.5 - CTL.r0;
  // 전체를 정사각형으로: 세로 칸 = S, 가로 칸은 열 수에 맞춰 넓힘
  CTL.SY = CTL.S;
  CTL.SX = CTL.S * rows / cols;
  CTL.w = cols * CTL.SX + CTL.PAD * 2;
  CTL.h = rows * CTL.SY + CTL.PAD * 2;
  $('ctlMap').setAttribute('viewBox', `0 0 ${CTL.w.toFixed(0)} ${CTL.h.toFixed(0)}`);
  delete lastHtml.ctlStatic;
  renderCtl();
}

// 칸 [열, 행] -> 화면 (px)
function cp(c, r) { return [(c - CTL.c0) * CTL.SX + CTL.PAD, (r - CTL.r0) * CTL.SY + CTL.PAD]; }
function toCell(x, y) { return [CTL.fit.col[0] * y + CTL.fit.col[1], CTL.fit.row[0] * x + CTL.fit.row[1]]; }
function np(id) { const g = CTL.grid[id]; return g ? cp(g[0], g[1]) : null; }
// 로봇 실제 위치 -> 화면: 마지막 노드 -> 다음 노드 길 위에서 얼마나 왔는지(실제 거리 비율)로 칸 사이를 보간
function robotPoint(r) {
  const a = CTL.nodes[r.last_node];
  const nxt = (r.committed || [])[0] || r.next_node;
  const b = CTL.nodes[nxt];
  const hasPose = Number.isFinite(r.x) && Number.isFinite(r.y) && (r.x !== 0 || r.y !== 0);
  if (a && b && hasPose) {
    const vx = b.x - a.x, vy = b.y - a.y, L2 = vx * vx + vy * vy || 1;
    const t = Math.max(0, Math.min(1, ((r.x - a.x) * vx + (r.y - a.y) * vy) / L2));
    const pa = np(a.id), pb = np(b.id);
    return { p: [pa[0] + (pb[0] - pa[0]) * t, pa[1] + (pb[1] - pa[1]) * t], dir: Math.atan2(pb[1] - pa[1], pb[0] - pa[0]) };
  }
  if (a) return { p: np(a.id), dir: null };
  if (hasPose) { const c = toCell(r.x, r.y); return { p: cp(c[0], c[1]), dir: null }; }
  return null;
}
const f1 = (v) => v.toFixed(1);

function renderCtl() {
  if (!CTL.map || $('ctlView').hidden) return;
  renderCtlStatic();
  renderCtlDyn();
  renderCtlSide();
}

// 테이블 몸통 (정사각형, T 노드 왼쪽 = 통로 쪽 모서리에 붙음)
function tableBox(nodeId) {
  const [x, y] = np(nodeId);
  const w = Math.min(CTL.SX, CTL.SY) * 0.62;
  const gap = (CTL.SX - w) / 2;
  return { x: x - gap - w, y: y - w / 2, w, h: w };
}
// 로봇이 가는 테이블: 작업(서빙 · 안내)의 테이블, 작업이 없으면 목적지가 테이블 노드일 때
function robotTable(r) {
  const t = r.task_id != null ? taskById(r.task_id) : null;
  if (t && t.table && !ENDED.includes(t.stage)) return t.table;
  const g = r.goal_node && CTL.nodes[r.goal_node];
  return g && g.stop_kind === 'SERVE' ? g.table : '';
}

function renderCtlStatic() {
  const out = [];
  // 격자 (반 칸마다, 칸 경계는 조금 진하게)
  for (let k = 0; CTL.PAD + k * CTL.SX / 2 <= CTL.w - CTL.PAD + 0.5; k++) {
    const gx = CTL.PAD + k * CTL.SX / 2;
    out.push(`<line class="g-grid${k % 2 ? '' : ' major'}" x1="${f1(gx)}" y1="${CTL.PAD}" x2="${f1(gx)}" y2="${f1(CTL.h - CTL.PAD)}"/>`);
  }
  for (let k = 0; CTL.PAD + k * CTL.SY / 2 <= CTL.h - CTL.PAD + 0.5; k++) {
    const gy = CTL.PAD + k * CTL.SY / 2;
    out.push(`<line class="g-grid${k % 2 ? '' : ' major'}" x1="${CTL.PAD}" y1="${f1(gy)}" x2="${f1(CTL.w - CTL.PAD)}" y2="${f1(gy)}"/>`);
  }
  // 테이블 몸통 (상태색은 동적 층에서 덮지 않고 여기 class로 - 테이블 상태가 바뀌면 다시 그림)
  const tstate = {};
  state.tables.forEach((t) => { tstate[t.id] = t.state; });
  CTL.map.nodes.filter((n) => n.stop_kind === 'SERVE').forEach((n) => {
    const t = tableBox(n.id);
    out.push(`<rect class="g-table ${esc(tstate[n.table] || '')}" x="${f1(t.x)}" y="${f1(t.y)}" width="${f1(t.w)}" height="${f1(t.h)}" rx="10"/>`);
    out.push(`<text class="g-label big" x="${f1(t.x + t.w / 2)}" y="${f1(t.y + t.h / 2)}">${esc(n.table || n.id)}</text>`);
  });
  // 길 (양방향은 한 줄, 한쪽 방향만 있으면 가운데 화살표)
  const seen = new Set();
  const dir = new Set(CTL.map.edges.map((e) => e.from + '>' + e.to));
  CTL.map.edges.forEach((e) => {
    const key = [e.from, e.to].sort().join('|');
    if (seen.has(key)) return;
    seen.add(key);
    const a = np(e.from), b = np(e.to);
    if (!a || !b) return;
    out.push(`<line class="g-edge" x1="${f1(a[0])}" y1="${f1(a[1])}" x2="${f1(b[0])}" y2="${f1(b[1])}"/>`);
    if (!dir.has(e.to + '>' + e.from)) {
      const mx = (a[0] + b[0]) / 2, my = (a[1] + b[1]) / 2, ang = Math.atan2(b[1] - a[1], b[0] - a[0]) * 180 / Math.PI;
      out.push(`<path class="g-oneway" d="M6 0 L-5 -5 L-5 5 Z" transform="translate(${f1(mx)} ${f1(my)}) rotate(${f1(ang)})"/>`);
    }
  });
  // 노드
  CTL.map.nodes.forEach((n) => {
    const [x, y] = np(n.id);
    const r = n.stop_kind === 'DOCK' || n.stop_kind === 'PICKUP' || n.stop_kind === 'QUEUE' ? 15 : n.stop_kind === 'SERVE' ? 8 : 12;
    out.push(`<rect class="g-node ${esc(n.stop_kind || '')}" x="${f1(x - r)}" y="${f1(y - r)}" width="${2 * r}" height="${2 * r}" rx="5"/>`);
    if (n.stop_kind !== 'SERVE') out.push(`<text class="g-label" x="${f1(x + r + 4)}" y="${f1(y + r + 6)}" text-anchor="start">${esc(n.id)}</text>`);
  });
  // 구역 이름
  const zone = (id, text, cls, dx, dy) => { const p = np(id); if (p) out.push(`<text class="g-zone ${cls}" x="${f1(p[0] + dx)}" y="${f1(p[1] + dy)}">${text}</text>`); };
  zone('D1', '입구', '', 0, 40);
  zone('P1', '픽업', 'pick', 0, -26);
  zone('D2', '도킹스테이션', '', -62, 4);
  setHtml('ctlStatic', out.join(''));
}

// 같은 길을 여러 로봇이 지나도 겹치지 않게 로봇마다 옆으로 조금 비켜 그림
function offsetPath(ids, off) { return offsetPts(ids.map(np).filter(Boolean), off); }
function offsetPts(pts, off) {
  if (pts.length < 2) return '';
  const segs = [];
  for (let i = 0; i + 1 < pts.length; i++) {
    const [ax, ay] = pts[i], [bx, by] = pts[i + 1];
    const L = Math.hypot(bx - ax, by - ay) || 1;
    const ox = -(by - ay) / L * off, oy = (bx - ax) / L * off;
    segs.push(`M${f1(ax + ox)} ${f1(ay + oy)} L${f1(bx + ox)} ${f1(by + oy)}`);
  }
  return segs.join(' ');
}

function robotList() { return [...state.robots].sort((a, b) => a.id.localeCompare(b.id)); }

function renderCtlDyn() {
  const out = [];
  const robots = robotList();
  const mid = (robots.length - 1) / 2;
  // (로봇 · 길은 updateBots가 따로: 매 프레임 부드럽게 움직임)
  // 장애물 길
  state.obstacles.filter((o) => o.from && o.to).forEach((o) => {
    const a = np(o.from), b = np(o.to);
    if (a && b) out.push(`<line class="g-obst-edge" x1="${f1(a[0])}" y1="${f1(a[1])}" x2="${f1(b[0])}" y2="${f1(b[1])}"/>`);
  });
  // 잠근 노드
  state.locks.forEach((l) => {
    const p = np(l.node);
    if (!p) return;
    out.push(`<rect class="g-lock" x="${f1(p[0] - 16)}" y="${f1(p[1] - 16)}" width="32" height="32" rx="8" stroke="${robotColor(l.robot_id)}"/>`);
  });
  // 로봇이 가는 테이블: 테이블 테두리를 로봇 색으로 + 모서리에 로봇 번호
  robots.forEach((r) => {
    const table = r.online !== false && robotTable(r);
    const n = table && CTL.map.nodes.find((m) => m.stop_kind === 'SERVE' && m.table === table);
    if (!n) return;
    const t = tableBox(n.id), c = robotColor(r.id);
    out.push(`<rect class="g-table-target" x="${f1(t.x - 4)}" y="${f1(t.y - 4)}" width="${f1(t.w + 8)}" height="${f1(t.h + 8)}" rx="13" stroke="${c}"/>`);
    out.push(`<g class="g-table-badge" transform="translate(${f1(t.x + t.w)} ${f1(t.y)})"><circle r="10" fill="${c}"/>
      <text y="0.5">${esc(r.id.replace(/\D/g, '') || r.id)}</text></g>`);
  });
  // 막혀서 기다리는 로봇: 못 들어가는 다음 칸에 주황 점선 테두리
  robots.forEach((r) => {
    const w = waitInfo(r);
    const p = w && w.node && np(w.node);
    if (!p) return;
    out.push(`<rect class="g-blocked" x="${f1(p[0] - 21)}" y="${f1(p[1] - 21)}" width="42" height="42" rx="11"/>`);
  });
  // 목적지
  robots.forEach((r) => {
    const p = r.goal_node && np(r.goal_node);
    if (!p || r.online === false) return;
    out.push(`<rect class="g-goal" x="${f1(p[0] - 7)}" y="${f1(p[1] - 7)}" width="14" height="14" stroke="${robotColor(r.id)}"
      transform="rotate(45 ${f1(p[0])} ${f1(p[1])})"/>`);
  });
  // 장애물 노드
  state.obstacles.filter((o) => o.node).forEach((o) => {
    const p = np(o.node);
    if (!p) return;
    out.push(`<g class="g-obst" transform="translate(${f1(p[0])} ${f1(p[1])})"><circle class="halo" r="16"/><circle class="core" r="11"/>
      <path d="M-4 -4 L4 4 M4 -4 L-4 4"/><text class="g-obst-label" y="-20">${esc(OBST_TEXT[o.kind] || '장애물')}</text></g>`);
  });
  setHtml('ctlDyn', out.join(''));
  updateBots(robots, mid);
  const moving = robots.filter((r) => r.goal_node).length;
  // 장애물: 서버가 피하는 남은 시간 (block_avoid_sec). 맵 표시는 그대로 두고 남은 초는 여기만 (맵을 매초 다시 그리지 않게)
  const obstText = state.obstacles.filter((o) => o.node)
    .map((o) => `${o.node}${o.left_sec > 0 ? ` ${Math.ceil(o.left_sec)}초` : ''}`).join(', ');
  $('ctlInfo').textContent = `로봇 ${robots.length}대 · 이동 ${moving}대 · 잠근 노드 ${state.locks.length}개` +
    (obstText ? ` · 장애물 ${obstText}` : '');
}

function renderCtlSide() {
  setHtml('ctlRobots', robotList().map((r) => {
    const online = r.online !== false;
    const st = online ? r.state : 'OFFLINE';
    const committed = r.committed || [], plan = r.plan || [];
    const chip = (id, cls) => `<span class="n ${cls}${id === r.goal_node ? ' goal' : ''}">${esc(id)}</span>`;
    const parts = [];
    if (r.last_node) parts.push(chip(r.last_node, 'at'));
    committed.forEach((n) => parts.push(chip(n, 'lock')));
    plan.forEach((n) => parts.push(chip(n, 'plan')));
    const route = parts.length > 1 ? parts.join('<span class="arr">›</span>') : (parts[0] || '') + (r.goal_node ? '' : ' <span class="arr">대기 중</span>');
    return `
      <div class="ctl-robot ${online ? '' : 'offline'}" style="--rc:${robotColor(r.id)}">
        <div class="robot-head">
          <span class="robot-name">${esc(r.id)}</span>
          <span class="badge ${esc(st)}">${online ? (ROBOT_STATE[r.state] || esc(r.state)) : '통신 끊김'}</span>
          ${r.goal_node ? `<span class="count">→ ${esc(r.goal_node)}</span>` : ''}
        </div>
        <div class="route">${route}</div>
        ${waitInfo(r) ? `<div class="ctl-wait">${esc(waitInfo(r).text)}</div>` : ''}
        ${BOTS[r.id] && BOTS[r.id].reroutedUntil > Date.now() ? `<div class="ctl-reroute">↻ 경로 변경 · 예전 ${esc((BOTS[r.id].reroutedFrom || []).join(' › '))}</div>` : ''}
      </div>`;
  }).join('') || '<div class="empty">로봇 정보가 아직 없어요</div>');
}

// ---------- 관제 애니메이션 ----------
// snapshot(5 Hz)이 오면 로봇마다 목표 위치만 바꾸고, 화면은 매 프레임(requestAnimationFrame) 목표 쪽으로 부드럽게 따라간다.
// 잠근 길(진한 선)은 지금 로봇 위치에서 시작해서 로봇이 가는 만큼 줄어들고, 예정 길(점선)은 목적지 쪽으로 흐른다.
const SVGNS = 'http://www.w3.org/2000/svg';
const BOTS = {};          // id -> { el: {...}, pos, ang, tgt, tang, off, route }
let animOn = false, animLast = 0;

function svgEl(tag, attrs, parent) {
  const e = document.createElementNS(SVGNS, tag);
  Object.entries(attrs).forEach(([k, v]) => e.setAttribute(k, v));
  if (parent) parent.appendChild(e);
  return e;
}

function botFor(r) {
  let b = BOTS[r.id];
  if (b) return b;
  const c = robotColor(r.id);
  const paths = $('ctlPaths'), bots = $('ctlBots');
  b = BOTS[r.id] = { pos: null, ang: 0, tgt: null, tang: 0, off: 0, locked: [], plan: [], el: {} };
  b.el.ghost = svgEl('path', { class: 'g-path ghost', stroke: c }, paths);   // 경로가 바뀌면 예전 길이 잠깐 남았다 사라짐
  b.el.dash = svgEl('path', { class: 'g-path dash flow', stroke: c }, paths);
  b.el.solid = svgEl('path', { class: 'g-path solid', stroke: c }, paths);
  b.el.run = svgEl('path', { class: 'g-path run' }, paths);           // 잠근 길 위 흰 점이 앞으로 흐름 (점유 중 표시)
  b.el.chev = svgEl('g', { class: 'g-chevs' }, paths);                // 길 옆 >>> (구간 가운데마다)
  b.el.g = svgEl('g', { class: 'g-robot' }, bots);
  b.el.halo = svgEl('circle', { class: 'halo', r: 14, fill: c }, b.el.g);
  b.el.head = svgEl('path', { d: 'M21 0 L10 -7 L10 7 Z', fill: c }, b.el.g);
  svgEl('circle', { class: 'body', r: 14, fill: c }, b.el.g);
  const t = svgEl('text', { class: 'num-in', y: 0.5 }, b.el.g);
  t.textContent = r.id.replace(/\D/g, '') || r.id;
  b.el.pause = svgEl('g', { class: 'pause', transform: 'translate(14 -14)' }, b.el.g);   // 막혀서 대기: ⏸ 표시
  svgEl('circle', { r: 9 }, b.el.pause);
  svgEl('rect', { x: -3.5, y: -4, width: 2.4, height: 8, rx: 1 }, b.el.pause);
  svgEl('rect', { x: 1.1, y: -4, width: 2.4, height: 8, rx: 1 }, b.el.pause);
  return b;
}

function updateBots(robots, mid) {
  const seen = new Set();
  robots.forEach((r, i) => {
    const b = botFor(r);
    seen.add(r.id);
    const online = r.online !== false;
    b.online = online;
    b.moving = online && r.state === 'MOVING';
    b.off = (i - mid) * 6;
    const rp = robotPoint(r);
    if (rp) {
      const yaw = r.yaw || 0;   // 서 있을 때: 맵 방향 -> 화면 방향 (가로 ≈ 맵 y, 세로 ≈ 맵 x)
      b.tgt = rp.p;
      b.tang = rp.dir != null ? rp.dir : Math.atan2(Math.cos(yaw), Math.sin(yaw));
      if (!b.pos) { b.pos = [...b.tgt]; b.ang = b.tang; }
    }
    b.locked = online ? (r.committed || []) : [];
    b.plan = online ? (r.plan || []) : [];
    b.waiting = !!waitInfo(r);
    b.el.g.classList.toggle('waiting', b.waiting);
    checkReroute(b, r);
    b.planFrom = b.locked.length ? b.locked[b.locked.length - 1] : (r.goal_node ? r.last_node : '');
    b.el.g.classList.toggle('off', !online);
    b.el.g.classList.toggle('moving', b.moving);
    b.el.chev.classList.toggle('moving', b.moving);
    b.el.chev.classList.toggle('wait', online && (!b.moving || b.waiting));
    b.el.chev.classList.toggle('moving', b.moving && !b.waiting);
    b.el.dash.classList.toggle('still', !b.moving || b.waiting);
    drawChevrons(b, r, robotColor(r.id));
  });
  Object.keys(BOTS).forEach((id) => {     // 사라진 로봇 정리
    if (seen.has(id)) return;
    Object.values(BOTS[id].el).forEach((e) => e.remove());
    delete BOTS[id];
  });
  startAnim();
}

function startAnim() {
  if (animOn) return;
  animOn = true;
  animLast = performance.now();
  requestAnimationFrame(animFrame);
}

function animFrame(now) {
  if ($('ctlView').hidden) { animOn = false; return; }      // 관제 탭이 아니면 멈춤 (탭 열 때 다시 시작)
  const dt = Math.min(0.1, (now - animLast) / 1000);
  animLast = now;
  const k = 1 - Math.exp(-dt * 8);                           // 목표까지 남은 거리의 이만큼씩 (0.2초 정도에 거의 도착)
  Object.values(BOTS).forEach((b) => {
    if (!b.tgt) return;
    b.pos[0] += (b.tgt[0] - b.pos[0]) * k;
    b.pos[1] += (b.tgt[1] - b.pos[1]) * k;
    let da = b.tang - b.ang;
    da = Math.atan2(Math.sin(da), Math.cos(da));
    b.ang += da * k;
    b.el.g.setAttribute('transform', `translate(${f1(b.pos[0])} ${f1(b.pos[1])})`);
    b.el.head.setAttribute('transform', `rotate(${f1(b.ang * 180 / Math.PI)})`);
    // 잠근 길: 지금 위치 -> 보낸 노드들 (로봇이 가면 줄어듦)
    const lockedPts = b.locked.map(np).filter(Boolean);
    const solid = lockedPts.length ? offsetPts([b.pos, ...lockedPts], b.off) : '';
    b.el.solid.setAttribute('d', solid);
    b.el.run.setAttribute('d', b.moving ? solid : '');
    const planPts = [b.planFrom, ...b.plan].map(np).filter(Boolean);
    b.el.dash.setAttribute('d', planPts.length > 1 ? offsetPts(planPts, b.off) : '');
  });
  requestAnimationFrame(animFrame);
}

// 길 옆 방향 표시 >>>: 지금 지나는 구간(마지막 노드 -> 다음 노드) 가운데에 한 번만.
// 움직이는 중이면 앞으로 차례차례 켜지며 흐르고, 서 있으면(대기 · 막힘 · 정지) 흐리게 멈춰 있음. 구간이 바뀔 때만 다시 만듦
const CHEV_SIDE = 15;     // 길에서 옆으로 (px, 가는 방향 왼쪽)
function drawChevrons(b, r, color) {
  const route = r.last_node && b.locked.length ? [r.last_node, b.locked[0]] : [];
  const key = route.join(',') + '|' + b.off;
  if (key === b.chevKey) return;
  b.chevKey = key;
  const g = b.el.chev;
  while (g.firstChild) g.firstChild.remove();
  for (let i = 0; i + 1 < route.length; i++) {
    const a = np(route[i]), c = np(route[i + 1]);
    if (!a || !c) continue;
    const L = Math.hypot(c[0] - a[0], c[1] - a[1]);
    const ux = (c[0] - a[0]) / L, uy = (c[1] - a[1]) / L;
    const side = CHEV_SIDE + b.off;
    const mx = (a[0] + c[0]) / 2 - uy * side, my = (a[1] + c[1]) / 2 + ux * side;
    const ang = Math.atan2(uy, ux) * 180 / Math.PI;
    const set = svgEl('g', { class: 'chev-set',
      transform: `translate(${f1(mx)} ${f1(my)}) rotate(${f1(ang)})` }, g);
    [-9, 0, 9].forEach((dx, k) => svgEl('path', { d: `M${dx - 3} -5 L${dx + 3} 0 L${dx - 3} 5`, stroke: color,
      style: `animation-delay:${(k * 0.18).toFixed(2)}s` }, set));
  }
}

// 경로가 바뀜 (목적지는 같은데 남은 길이 예전 길의 뒷부분이 아님 = 우회 · 비켜 주기): 예전 길을 잠깐 흐리게 남기고
// 새 길을 반짝여 보여 준다. 오른쪽 패널에도 '경로 변경'을 몇 초 띄움
const REROUTE_MS = 4000;
const OBST_TEXT = { DYNAMIC: '사람', PERSON: '사람', STATIC: '물건', UNKNOWN: '장애물' };
// 로봇이 막혀서 기다리는 이유: 다른 로봇(waiting_for) 또는 앞 칸 장애물 (state BLOCKED 이거나 다음 칸이 장애물)
function waitInfo(r) {
  if (r.online === false) return null;
  const next = (r.committed || [])[0] || (r.plan || [])[0] || '';
  const obst = state.obstacles.find((o) => o.node === next || (o.from === r.last_node && o.to === next) || (o.to === r.last_node && o.from === next));
  if (obst) return { node: next, text: `${next} ${OBST_TEXT[obst.kind] || '장애물'} 앞에서 대기` };
  if (r.waiting_for) return { node: (r.plan || [])[0] || next, text: `${(r.plan || [])[0] || '다음 칸'} 앞에서 대기 · ${r.waiting_for}가 쓰는 중` };
  if (r.state === 'BLOCKED') return { node: next, text: `${next || '앞'} 막힘 · 확인 중` };
  return null;
}
function checkReroute(b, r) {
  const route = [...b.locked, ...b.plan];
  const prev = b.prevRoute, prevGoal = b.prevGoal;
  b.prevRoute = route;
  b.prevGoal = r.goal_node || '';
  if (!prev || !r.goal_node || prevGoal !== r.goal_node || !route.length) return;
  const isTail = route.length <= prev.length && route.every((n, i) => n === prev[prev.length - route.length + i]);
  const grew = route.length > prev.length && prev.every((n, i) => n === route[i]);   // 같은 길에 노드만 늘어남
  if (isTail || grew) return;
  const ghostPts = [r.last_node, ...prev].map(np).filter(Boolean);
  b.el.ghost.setAttribute('d', ghostPts.length > 1 ? offsetPts(ghostPts, b.off) : '');
  restartAnim(b.el.ghost, 'show');
  restartAnim(b.el.dash, 'reroute');
  restartAnim(b.el.solid, 'reroute');
  b.reroutedUntil = Date.now() + REROUTE_MS;
  b.reroutedFrom = prev;
}
function restartAnim(el, cls) {
  el.classList.remove(cls);
  void el.getBoundingClientRect();   // 다시 붙여서 애니메이션을 처음부터
  el.classList.add(cls);
}
