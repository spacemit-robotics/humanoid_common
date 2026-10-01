"use strict";
const $ = id => document.getElementById(id);
let socket, authenticated = false, status, catalog, id = 0, lastStatusAt = 0;
let token = new URLSearchParams(location.hash.slice(1)).get("token") || "";
try {
  if (token) sessionStorage.setItem("operator.session", token);
  else token = sessionStorage.getItem("operator.session") || "";
} catch (_) { /* Browser privacy mode can disable tab storage. */ }
let held = "", lastActionCatalog = "", lastPolicyCatalog = "", lastVelocityAt = 0;
let activeRequest = 0;
const pending = new Map();
const messages = {not_owner:"请先接管控制",busy:"其他客户端持有控制权，或请求正在处理",not_ready:"操作条件尚未满足",fault:"故障已锁存",unauthorized:"配对已失效，请重新扫描操作二维码",timeout:"确认超时，请检查当前状态"};
const operationNames = {state:"状态切换",policy:"策略切换",interaction:"开始动作",cancel:"取消动作",ack:"故障确认",reference_start:"开始参考动作"};
const phaseNames = {accepted:"待确认",completed:"已确认",rejected:"已拒绝",expired:"确认超时",cancelled:"已撤销",sent:"已发送"};
const actionPhases = {IDLE:"就绪",BLEND_IN:"进入",PLAYING:"播放",HOLDING:"保持",BLEND_OUT:"收回",FINISHED:"完成",REJECTED:"拒绝"};
const descriptions = {"control acquired":"已取得控制权","released":"控制权已释放","control renewed":"控制权已续期","velocity accepted":"速度请求已受理","velocity cleared; pending requests cancelled":"速度已清零，控制权及待处理请求已撤销","control entered requested state":"机器人已切换状态","control selected requested policy":"机器人已采用所选策略","control handled action request":"机器人已处理动作请求","control acknowledged fault":"故障已确认","already in requested state":"已经在请求状态"};
history.replaceState(null, "", location.pathname);
window.addEventListener('hashchange', () => {
  const paired = new URLSearchParams(location.hash.slice(1)).get('token');
  if (!paired) return;
  token = paired;
  try { sessionStorage.setItem('operator.session', token); } catch (_) {}
  history.replaceState(null, '', location.pathname);
  connect();
});
function tell(text) { $("message").textContent = text; }
function call(op, args = {}) {
  return new Promise((resolve, reject) => {
    if (!socket || socket.readyState !== WebSocket.OPEN) { reject(new Error("连接已断开")); return; }
    const request = ++id;
    const timer = setTimeout(() => { pending.delete(request); reject(new Error(messages.timeout)); }, 2000);
    pending.set(request, {resolve, reject, timer});
    socket.send(JSON.stringify({v:1,id:request,op,args}));
  });
}
async function request(op, args = {}) {
  try {
    const r = await call(op, args);
    if (r.data?.request?.phase === "accepted") activeRequest = r.data.request.sequence;
    tell(r.code === "accepted" ? (r.data?.request.phase === 'sent' ? "请求已发送" : "请求已受理，等待机器人确认") : (descriptions[r.message] || r.message || "操作完成"));
    return r;
  }
  catch (e) { tell(e.message); return null; }
}
function connect() {
  if (!token) { $("login").hidden = false; return; }
  if (socket) socket.close();
  for (const p of pending.values()) { clearTimeout(p.timer); p.reject(new Error("连接已替换")); }
  pending.clear(); held = "";
  id = 0; authenticated = false; activeRequest = 0;
  socket = new WebSocket(`${location.protocol === "https:" ? "wss:" : "ws:"}//${location.host}/api/v1/ws`);
  const current = socket;
  $("connection").textContent = "连接中";
  socket.onopen = async () => {
    if (socket !== current) return;
    try {
      const r = await call("hello", {token,name:navigator.userAgent.includes("SpaceMITOperatorApp") ? "android" : "web"});
      if (socket !== current) return;
      authenticated = true; catalog = r.data.catalog; status = r.data.status;
      lastStatusAt = performance.now();
      $("robot").textContent = catalog.robot === 'linglong' ? 'LingLong 灵龙' : catalog.robot;
      document.title = `${catalog.robot} · 操作台`;
      $("login").hidden = true;
      tell("已连接，尚未申请控制权"); render();
    } catch (e) { if (socket === current) { tell(e.message); $("login").hidden = false; current.close(); } }
  };
  socket.onmessage = event => {
    if (socket !== current) return;
    try {
      const r = JSON.parse(event.data);
      if (r.v !== 1) throw new Error("不支持的协议版本");
      if (r.event === "status") {
        status = r.data; lastStatusAt = performance.now();
        if (!status.owns_control || !status.online || status.state !== "RL") endMotion(false);
        render(); return;
      }
      const waiting = pending.get(r.id);
      if (!waiting) return;
      pending.delete(r.id); clearTimeout(waiting.timer);
      if (r.ok) waiting.resolve(r);
      else waiting.reject(new Error(`${messages[r.code] || r.code}: ${r.message}`));
    } catch (e) { tell(e.message); socket.close(); }
  };
  socket.onclose = () => {
    if (socket !== current) return;
    authenticated = false; held = "";
    if (status) { status.online = false; status.owns_control = false; }
    for (const p of pending.values()) { clearTimeout(p.timer); p.reject(new Error("连接已断开")); }
    pending.clear(); render();
  };
}
function activePolicy() { return catalog?.policies.find(p => p.name === status?.policy); }
function render() {
  const online = authenticated && status?.online && performance.now() - lastStatusAt < 1000;
  const owns = online && status.owns_control;
  const fault = status?.fault;
  const busy = status?.request.phase === "accepted";
  $("connection").textContent = online ? "控制链路在线" : authenticated ? "等待控制程序" : "未连接";
  $("connection").classList.toggle("online", !!online);
  $("owner").textContent = owns ? "本客户端" : (status?.owner || "无人持有");
  $("state").textContent = status?.state || "—";
  const stateLabels = {POWER_OFF:'电机未上电',DAMP:'阻尼保护',HOME:'复位姿态',ZERO:'准备姿态',RL:'策略运行中',SAFETY:'安全保护'};
  $("state-label").textContent = stateLabels[status?.state] || '等待状态';
  $("fsm-note").textContent = status?.state === 'ZERO' ? (status.zero_ready ? '准备就绪' : '姿态调整中') : '';
  $("ready").textContent = status?.state === "ZERO" ? (status.zero_ready ? "已到位" : "调整中") : "—";
  $("frequency").textContent = `${(status?.rl_hz || 0).toFixed(1)} Hz`;
  $("age").textContent = online ? `${status.age_ms.toFixed(0)} ms` : "—";
  $("active-policy").textContent = status?.policy || "—";
  $("acquire").disabled = !online || !!status?.owner;
  $("release").disabled = !owns;
  $("pair").disabled = !authenticated;
  $("disconnect").disabled = !authenticated;
  $("disconnect").hidden = !authenticated;
  $("reconnect").hidden = authenticated || !token;
  $("stop").disabled = !authenticated;
  $("fault").hidden = !fault?.latched;
  $("fault-title").textContent = fault ? `${fault.source}/${fault.code}${fault.active ? " · 活动" : " · 锁存"}` : "";
  $("fault-detail").textContent = fault?.detail || "";
  $("ack").disabled = !owns || status?.state !== "POWER_OFF" || fault?.active;
  const before = {HOME:"DAMP",ZERO:"HOME",RL:"ZERO"};
  document.querySelectorAll("[data-state]").forEach(b => {
    const target = b.dataset.state;
    b.classList.toggle("active", status?.state === target);
    b.disabled = !online || status.state === target ||
      (!['POWER_OFF','DAMP'].includes(target) && (!owns || busy || fault?.latched ||
        before[target] !== status.state || (target === 'RL' && !status.zero_ready))) ||
      (target === 'DAMP' && status.state === 'SAFETY');
  });
  if (catalog) {
    const signature = catalog.policies.map(p => p.name).join("|");
    if (signature !== lastPolicyCatalog) {
      $("policy").replaceChildren(...catalog.policies.map(p => new Option(p.name, p.name)));
      $("policy").value = status.policy; lastPolicyCatalog = signature;
    }
  }
  $("select-policy").disabled = !owns || busy || fault?.latched || !['POWER_OFF','DAMP'].includes(status?.state);
  const p = activePolicy();
  const signature = status?.policy || "";
  if (signature !== lastActionCatalog) {
    $("policy").value = status.policy;
    $("action").replaceChildren(...(p?.actions || []).map(a => new Option(a.display_name, a.key)));
    lastActionCatalog = signature;
    for (const axis of ['vx','vy','wz']) {
      const input = $("speed-" + axis);
      const max = Math.max(Math.abs(p?.minimum[axis] || 0), p?.maximum[axis] || 0);
      input.step = catalog.velocity_step?.[axis] || 0.1;
      input.max = max; input.value = Math.min(Number(input.value), max); input.disabled = !max;
    }
  }
  const rl = owns && status.state === 'RL' && !fault?.latched;
  const movable = p && ['vx','vy','wz'].some(axis => p.minimum[axis] < 0 || p.maximum[axis] > 0);
  $("motion-state").textContent = rl ? (movable ? '可操作' : '定点站立') : owns ? '等待 RL' : '等待接管';
  const actionBusy = ['BLEND_IN','PLAYING','HOLDING','BLEND_OUT'].includes(status?.interaction.phase);
  $("start-action").disabled = !rl || busy || actionBusy || !p?.actions.length;
  $("cancel-action").disabled = !rl || busy || !actionBusy;
  $("start-reference").hidden = !p?.manual_reference;
  $("start-reference").disabled = !rl || busy;
  const actionPhase = status?.interaction.phase || "IDLE";
  $("action-state").textContent = actionPhases[actionPhase] || actionPhase;
  $("action-state").dataset.phase = actionPhase;
  $("progress").value = status?.interaction.progress || 0;
  $("progress-label").textContent = `${Math.round((status?.interaction.progress || 0) * 100)}%`;
  const r = status?.request;
  if (activeRequest && r?.sequence === activeRequest && r.phase !== "accepted") {
    tell(`${operationNames[r.operation] || r.operation} · ${phaseNames[r.phase] || r.phase} · ${descriptions[r.message] || r.message}`);
    activeRequest = 0;
  }
  $("request").textContent = r?.sequence ? `#${r.sequence} ${operationNames[r.operation] || r.operation} · ${phaseNames[r.phase] || r.phase} · ${descriptions[r.message] || r.message}` : "—";
  const v = status?.velocity;
  $("velocity").textContent = `${(v?.vx || 0).toFixed(2)} / ${(v?.vy || 0).toFixed(2)} / ${(v?.wz || 0).toFixed(2)}`;
  document.querySelectorAll('[data-motion]').forEach(b => {
    const axis = ['w','s'].includes(b.dataset.motion) ? 'vx' : ['a','d'].includes(b.dataset.motion) ? 'vy' : 'wz';
    b.disabled = !rl || !Math.max(Math.abs(p?.minimum[axis] || 0), p?.maximum[axis] || 0);
    b.classList.toggle('active', held === b.dataset.motion);
  });
}
function velocity() {
  const v = {vx:0,vy:0,wz:0,valid_for_ms:300};
  const key = held;
  const axis = ['w','s'].includes(key) ? 'vx' : ['a','d'].includes(key) ? 'vy' : 'wz';
  const amount = Math.max(0, Number($("speed-" + axis).value));
  if (key && Number.isFinite(amount)) v[axis] = amount * (['w','a','j'].includes(key) ? 1 : -1);
  return v;
}
function beginMotion(key) {
  if (!status?.owns_control || !status.online || status.state !== 'RL' || status.fault?.latched || document.hidden) return;
  held = key; request('velocity', velocity()); render();
}
function endMotion(send = true) {
  const wasHeld = !!held; held = "";
  if (send && wasHeld && authenticated && status?.owns_control) request('velocity', {vx:0,vy:0,wz:0,valid_for_ms:300});
  document.querySelectorAll('[data-motion]').forEach(b => b.classList.remove('active'));
}
document.querySelectorAll('[data-state]').forEach(b => b.onclick = () => { endMotion(); request('state', {state:b.dataset.state}); });
document.querySelectorAll('[data-motion]').forEach(b => {
  b.onpointerdown = e => { e.preventDefault(); b.setPointerCapture(e.pointerId); beginMotion(b.dataset.motion); };
  b.onpointerup = b.onpointercancel = b.onlostpointercapture = () => endMotion();
});
$("acquire").onclick = () => request('acquire');
$("release").onclick = () => { endMotion(); request('release'); };
$("stop").onclick = () => { endMotion(false); request('stop'); };
$("select-policy").onclick = () => request('policy', {policy:$("policy").value});
$("start-action").onclick = () => request('interaction', {action:$("action").value});
$("cancel-action").onclick = () => request('cancel');
$("start-reference").onclick = () => request('reference_start');
$("ack").onclick = () => request('ack');
$("login").onsubmit = e => {
  e.preventDefault();
  try {
    const target = new URL($("pairing-link").value);
    if (!['http:', 'https:'].includes(target.protocol) || target.username || target.password) throw new Error();
    const paired = new URLSearchParams(target.hash.slice(1)).get('token');
    if (!paired) throw new Error();
    if (target.origin !== location.origin) { location.assign(target.href); return; }
    token = paired; $("pairing-link").value = '';
    try { sessionStorage.setItem('operator.session', token); } catch (_) {}
    connect();
  } catch (_) { $("login-message").textContent = '链接不完整，请使用二维码中的连接链接'; }
};
$("disconnect").onclick = () => window.operatorDisconnect();
$("reconnect").onclick = () => connect();
$("pair").onclick = async () => {
  const r = await request('pairing');
  if (r) {
    const image = 'data:image/svg+xml;charset=utf-8,' + encodeURIComponent(r.data.qr_svg);
    $("qr").src = image; $("download-qr").href = image;
    $("pair-address").textContent = new URL(r.data.url).origin;
    $("pair-dialog").showModal();
  }
};
$("close-pair").onclick = () => $("pair-dialog").close();
window.operatorDisconnect = () => { endMotion(false); if (authenticated) { call('release').catch(() => {}); } socket?.close(); };
window.operatorReconnect = () => { if (token && (!authenticated || socket?.readyState !== WebSocket.OPEN)) connect(); };
document.addEventListener('visibilitychange', () => { if (document.hidden) window.operatorDisconnect(); });
window.addEventListener('pagehide', () => window.operatorDisconnect());
window.addEventListener('blur', () => endMotion());
window.addEventListener('keydown', e => {
  if (['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName) || e.repeat) return;
  if (e.code === 'Space') { e.preventDefault(); endMotion(false); request('stop'); }
  if ('wsadjl'.includes(e.key) && e.key.length === 1) { e.preventDefault(); beginMotion(e.key); }
});
window.addEventListener('keyup', e => { if (e.key === held) endMotion(); });
document.querySelectorAll('[data-view]').forEach(button => {
  if (button.tagName !== 'BUTTON') return;
  button.onclick = () => {
    endMotion(); document.body.dataset.view = button.dataset.view;
    document.querySelectorAll('.mobile-views button').forEach(b => b.classList.toggle('active', b === button));
  };
});
setInterval(() => {
  if (!authenticated || document.hidden) return;
  if (performance.now() - lastStatusAt > 1000) { window.operatorDisconnect(); return; }
  if (status?.owns_control) call('renew').catch(e => { held = ""; tell(e.message); });
}, 300);
setInterval(() => {
  if (held && authenticated && status?.owns_control && status.online && !document.hidden &&
      performance.now() - lastVelocityAt >= 100) {
    lastVelocityAt = performance.now(); call('velocity', velocity()).catch(e => { held = ""; tell(e.message); });
  }
}, 100);
if (window.lucide) lucide.createIcons();
async function initialize() {
  if (!navigator.userAgent.includes('SpaceMITOperatorApp')) {
    fetch('/downloads/SpacemiT-Operator.apk', {method:'HEAD', cache:'no-store'})
      .then(response => { $("download-app").hidden = !response.ok; })
      .catch(() => {});
  }
  if (!token) {
    try {
      const response = await fetch('/api/v1/local-session', {cache:'no-store'});
      if (response.ok) {
        token = (await response.json()).token;
        try { sessionStorage.setItem('operator.session', token); } catch (_) {}
      }
    } catch (_) {}
  }
  if (!token) tell('等待设备配对');
  connect();
}
render(); initialize();
