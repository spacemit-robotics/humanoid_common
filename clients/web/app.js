"use strict";
const $ = id => document.getElementById(id);
let socket, authenticated = false, status, catalog, id = 0, lastStatusAt = 0;
const STATUS_PAUSE_MS = 1000, RECONNECT_AFTER_MS = 5000;
let connectionWanted = true, suspended = false, reconnectTimer = null, connectTimer = null;
let reconnectDelay = 500, statusStale = false;
let token = new URLSearchParams(location.hash.slice(1)).get("token") || "";
try {
  if (token) sessionStorage.setItem("operator.session", token);
  else token = sessionStorage.getItem("operator.session") || "";
} catch (_) { /* Browser privacy mode can disable tab storage. */ }
let lastActionCatalog = "", lastPolicyCatalog = "";
let lastVelocityCatalog = "", speedPresetsInitialized = false;
const motion = new window.OperatorTeleop.Motion();
const keys = new Set();
const pointers = new Map();
const stickPointers = new Map();
const stickValues = {move: {x:0,y:0}, turn: {x:0,y:0}};
let inputMode = 'buttons';
let motionTickAt = performance.now(), velocityDirty = false, motionEpoch = 0;
let activeRequest = 0, adminLoginWaiting = false;
const pending = new Map();
let lastClientEvent = "", lastClientEventAt = -Infinity;
function clientEvent(event, detail = "") {
  if (!authenticated || !socket || socket.readyState !== WebSocket.OPEN) return;
  const signature = `${event}:${detail}`, now = performance.now();
  if (signature === lastClientEvent && now - lastClientEventAt < 1000) return;
  lastClientEvent = signature; lastClientEventAt = now;
  call('client_event', {event, detail:String(detail).slice(0, 256)}).catch(() => {});
}
const messages = {not_owner:"请先接管控制",busy:"其他客户端持有控制权，或请求正在处理",not_ready:"操作条件尚未满足",fault:"故障已锁存",unauthorized:"配对已失效，请重新扫描操作二维码",timeout:"确认超时，请检查当前状态",admin_required:"请先登录管理员",rate_limited:"尝试次数过多，请稍后再试"};
const operationNames = {state:"状态切换",policy:"策略切换",interaction:"开始动作",cancel:"取消动作",ack:"故障确认",reference_start:"开始参考动作"};
const phaseNames = {accepted:"待确认",completed:"已确认",rejected:"已拒绝",expired:"确认超时",cancelled:"已撤销",sent:"已发送"};
const actionPhases = {IDLE:"就绪",BLEND_IN:"进入",PLAYING:"播放",HOLDING:"保持",BLEND_OUT:"收回",FINISHED:"完成",REJECTED:"拒绝"};
const descriptions = {"control acquired":"已取得控制权","released":"控制权已释放","control renewed":"控制权已续期","velocity accepted":"速度请求已受理","velocity cleared; pending requests cancelled":"速度已清零，控制权及待处理请求已撤销","control entered requested state":"机器人已切换状态","control selected requested policy":"机器人已采用所选策略","control handled action request":"机器人已处理动作请求","control acknowledged fault":"故障已确认","already in requested state":"已经在请求状态"};
const inApp = navigator.userAgent.includes("SpaceMITOperatorApp");
history.replaceState(null, "", location.pathname);
window.addEventListener('hashchange', () => {
  const paired = new URLSearchParams(location.hash.slice(1)).get('token');
  if (!paired) return;
  token = paired;
  try { sessionStorage.setItem('operator.session', token); } catch (_) {}
  history.replaceState(null, '', location.pathname);
  closeConnection(); window.operatorReconnect();
});
function tell(text) { $("message").textContent = text; }
function closeConnection() {
  const previous = socket;
  socket = null;
  clearTimeout(connectTimer); connectTimer = null;
  authenticated = false; activeRequest = 0; endMotion(false);
  statusStale = false;
  if (status) { status.online = false; status.owns_control = false; status.owner = ""; }
  for (const p of pending.values()) { clearTimeout(p.timer); p.reject(new Error("连接已断开")); }
  pending.clear(); previous?.close(); render();
}
function scheduleReconnect() {
  if (!connectionWanted || !token || suspended || document.hidden || reconnectTimer !== null) return;
  reconnectTimer = setTimeout(() => { reconnectTimer = null; connect(); }, reconnectDelay);
  reconnectDelay = Math.min(reconnectDelay * 2, 5000);
  tell("连接暂不可用，正在自动重连，无需重新扫码"); render();
}
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
  const current = socket;
  try {
    const r = await call(op, args);
    if (socket !== current) return null;
    if (r.data?.request?.phase === "accepted") activeRequest = r.data.request.sequence;
    tell(r.code === "accepted" ? (r.data?.request.phase === 'sent' ? "请求已发送" : "请求已受理，等待机器人确认") : (descriptions[r.message] || r.message || "操作完成"));
    return r;
  }
  catch (e) { if (socket === current) tell(e.message); return null; }
}
function connect() {
  if (!token) { $("login").hidden = false; return; }
  if (!connectionWanted || suspended || document.hidden) return;
  if (socket && [WebSocket.CONNECTING, WebSocket.OPEN].includes(socket.readyState)) return;
  clearTimeout(reconnectTimer); reconnectTimer = null;
  closeConnection(); id = 0;
  socket = new WebSocket(`${location.protocol === "https:" ? "wss:" : "ws:"}//${location.host}/api/v1/ws`);
  const current = socket;
  $("connection").textContent = "连接中";
  connectTimer = setTimeout(() => {
    if (socket === current) { closeConnection(); scheduleReconnect(); }
  }, RECONNECT_AFTER_MS);
  socket.onopen = async () => {
    if (socket !== current) return;
    try {
      const r = await call("hello", {token,name:navigator.userAgent.includes("SpaceMITOperatorApp") ? "android" : "web"});
      if (socket !== current) return;
      clearTimeout(connectTimer); connectTimer = null; reconnectDelay = 500;
      authenticated = true; catalog = r.data.catalog; status = r.data.status;
      lastStatusAt = performance.now();
      $("robot").textContent = catalog.robot;
      if ($("voice-robot")) $("voice-robot").textContent = catalog.robot;
      voicePort = Number.isInteger(catalog.voice_chat_port) ? catalog.voice_chat_port : 0;
      showPublicPage();
      document.title = `${catalog.robot} · 操作台`;
      $("login").hidden = true;
      tell("已连接，尚未申请控制权"); render();
    } catch (e) {
      if (socket !== current) return;
      if (e.code === "unauthorized") connectionWanted = false;
      closeConnection();
      if (connectionWanted) scheduleReconnect();
      else { tell(e.message); $("login").hidden = false; }
    }
  };
  socket.onmessage = event => {
    if (socket !== current) return;
    let r;
    try {
      r = JSON.parse(event.data);
      if (r.v !== 1) throw new Error("不支持的协议版本");
    } catch (e) {
      clientEvent('protocol_error', e.message);
      tell(e.message); closeConnection(); scheduleReconnect(); return;
    }
    try {
      if (r.event === "status") {
        status = r.data; lastStatusAt = performance.now();
        if (statusStale) { statusStale = false; tell("状态更新已恢复，旧遥控输入不会继续执行"); }
        if (!status.owns_control || !status.online || status.state !== "RL" || status.fault?.latched) {
          const stopForFault = motion.moving && status.owns_control && status.online && status.fault?.latched;
          endMotion(false);
          if (stopForFault) call('stop').catch(() => {});
        }
        render(); return;
      }
      const waiting = pending.get(r.id);
      if (!waiting) return;
      pending.delete(r.id); clearTimeout(waiting.timer);
      if (r.ok) waiting.resolve(r);
      else waiting.reject(Object.assign(new Error(`${messages[r.code] || r.code}: ${r.message}`), {code:r.code,remote:true}));
    } catch (e) {
      clientEvent('ui_error', e.message);
      endMotion(); tell(`界面更新异常：${e.message}；连接保留，请刷新页面`);
    }
  };
  socket.onclose = event => {
    if (socket !== current) return;
    adminLoginWaiting = false;
    $("admin-submit").disabled = false;
    if ($("admin-dialog").open) $("admin-message").textContent = "连接已断开，请重连后登录";
    console.warn('operator socket closed', event.code, event.reason);
    closeConnection(); scheduleReconnect();
  };
}
function activePolicy() { return catalog?.policies.find(p => p.name === status?.policy); }
function controlOwnerLabel(owner, owns) {
  if (owns) return "本客户端已接管";
  if (!owner) return "无人持有";
  const names = {gamepad:"手柄",web:"其他网页",android:"手机 App",terminal:"终端",voice:"语音"};
  const client = owner.replace(/#\d+$/, "");
  return Object.prototype.hasOwnProperty.call(names, client) ? `${names[client]}已接管控制权` : owner;
}
function render() {
  const trajectory = status?.trajectory_enabled ?? catalog?.trajectory_enabled ?? false;
  document.body.dataset.trajectory = String(trajectory);
  $("inference-status").hidden = trajectory;
  document.querySelector('.policy-section').hidden = trajectory;
  document.querySelector('.motion').hidden = trajectory;
  document.querySelector('.mobile-views').hidden = trajectory;
  if (trajectory) document.body.dataset.view = 'control';
  const activeMode = $("active-mode");
  activeMode.dataset.state = trajectory ? 'TRAJECTORY' : 'RL';
  activeMode.querySelector('strong').textContent = trajectory ? '动作' : '策略控制';
  activeMode.querySelector('span').textContent = trajectory ? 'TRAJECTORY' : 'RL';
  const fresh = performance.now() - lastStatusAt < STATUS_PAUSE_MS;
  const online = authenticated && status?.online && fresh;
  const admin = authenticated && (status?.admin || !catalog?.admin_required);
  const owns = online && admin && status.owns_control;
  const fault = status?.fault;
  const busy = status?.request.phase === "accepted";
  $("connection").textContent = online ? "控制链路在线" : authenticated
    ? (fresh ? "等待控制程序" : "网络延迟，等待恢复")
    : connectionWanted && token ? "连接中 · 自动重试" : "未连接";
  $("connection").classList.toggle("online", !!online);
  const ownerLabel = online ? controlOwnerLabel(status.owner, owns) : "等待状态确认";
  $("owner").textContent = ownerLabel;
  $("state").textContent = status?.state === 'TRAJECTORY' ? '动作' : status?.state || "—";
  const stateLabels = {POWER_OFF:'电机未上电',DAMP:'阻尼保护',HOME:'复位姿态',ZERO:'准备姿态',RL:'策略运行中',TRAJECTORY:'TRAJECTORY',SAFETY:'安全保护'};
  $("state-label").textContent = stateLabels[status?.state] || '等待状态';
  $("fsm-note").textContent = status?.state === 'ZERO' ? (status.zero_ready ? '准备就绪' : '姿态调整中') : '';
  $("ready").textContent = status?.state === "ZERO" ? (status.zero_ready ? "已到位" : "调整中") : "—";
  $("frequency").textContent = `${(status?.rl_hz || 0).toFixed(1)} Hz`;
  $("age").textContent = online ? `${status.age_ms.toFixed(0)} ms` : "—";
  $("active-policy").textContent = activePolicy()?.display_name || status?.policy || "—";
  $("acquire").disabled = !online || !admin || !!status?.owner;
  $("release").disabled = !owns;
  const managing = document.body.dataset.mode === 'control';
  const hasPublicPage = Boolean(catalog?.admin_required || voicePort > 0);
  $("admin-entry").hidden = managing || !hasPublicPage;
  $("admin-entry").disabled = !authenticated;
  $("admin-exit").hidden = !managing || !hasPublicPage;
  $("pair").disabled = !authenticated;
  $("disconnect").disabled = !authenticated;
  $("disconnect").hidden = !authenticated;
  $("reconnect").hidden = authenticated || !token;
  $("fault").hidden = !fault?.latched;
  $("fault-title").textContent = fault ? `${fault.source}/${fault.code}${fault.active ? " · 活动" : " · 锁存"}` : "";
  $("fault-detail").textContent = fault?.detail || "";
  $("ack").disabled = !owns || status?.state !== "POWER_OFF" || fault?.active;
  $("ack").hidden = !managing;
  const before = {HOME:"DAMP",ZERO:"HOME",RL:"ZERO",TRAJECTORY:"ZERO"};
  document.querySelectorAll("[data-state]").forEach(b => {
    const target = b.dataset.state;
    b.classList.toggle("active", status?.state === target);
    b.disabled = !online || !admin || status.state === target ||
      (!['POWER_OFF','DAMP'].includes(target) && (!owns || busy || fault?.latched ||
        before[target] !== status.state || (['RL','TRAJECTORY'].includes(target) && !status.zero_ready))) ||
      (target === 'DAMP' && status.state === 'SAFETY');
  });
  if (catalog) {
    const signature = JSON.stringify(catalog.policies.map(p => [p.name, p.display_name]));
    if (signature !== lastPolicyCatalog) {
      $("policy").replaceChildren(...catalog.policies.map(p => new Option(p.display_name || p.name, p.name)));
      $("policy").value = status.policy; lastPolicyCatalog = signature;
    }
  }
  const switchGroup = activePolicy()?.switch_group;
  const selectedPolicy = catalog?.policies.find(p => p.name === $("policy").value);
  const liveSwitch = status?.state === 'RL' && switchGroup && selectedPolicy?.switch_group === switchGroup;
  $("select-policy").disabled = trajectory || !owns || busy || fault?.latched ||
    (!['POWER_OFF','DAMP'].includes(status?.state) && !liveSwitch);
  const p = activePolicy();
  const actions = trajectory ? (catalog?.actions || []) : (p?.actions || []);
  const signature = JSON.stringify([trajectory, status?.policy || '', actions]);
  if (signature !== lastActionCatalog) {
    $("policy").value = status?.policy || '';
    $("action").replaceChildren(...actions.map(a => new Option(a.display_name, a.key)));
    lastActionCatalog = signature;
  }
  const velocityCatalog = JSON.stringify([p?.name, p?.minimum, p?.maximum, p?.client_smoothing, catalog?.teleop]);
  if (velocityCatalog !== lastVelocityCatalog) {
    lastVelocityCatalog = velocityCatalog;
    endMotion();
    motion.mode = 'slow';
    motion.configure(p, catalog?.teleop);
    updateSpeedPresets(!speedPresetsInitialized);
    if (p && ['vx','vy','wz'].some(axis => !$("speed-" + axis).disabled)) speedPresetsInitialized = true;
  }
  const rl = !trajectory && owns && status.state === 'RL' && !fault?.latched;
  const actionReady = owns && status.state === (trajectory ? 'TRAJECTORY' : 'RL') && !fault?.latched;
  const movable = p && ['vx','vy','wz'].some(axis => p.minimum[axis] < 0 || p.maximum[axis] > 0);
  $("motion-state").textContent = rl ? (movable ? '可操作' : '定点站立') : owns ? '等待 RL'
    : online && status.owner ? ownerLabel : '等待接管';
  const actionBusy = ['BLEND_IN','PLAYING','HOLDING','BLEND_OUT'].includes(status?.interaction.phase);
  $("start-action").disabled = !actionReady || busy || actionBusy || !actions.length;
  $("cancel-action").disabled = !actionReady || busy || !actionBusy;
  $("start-reference").hidden = trajectory || !p?.manual_reference;
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
  renderMotion();
  if (adminLoginWaiting && !status?.admin_login_pending) {
    adminLoginWaiting = false;
    $("admin-submit").disabled = false;
    if (status?.admin) {
      $("admin-dialog").close();
      setMode('control');
      tell('已进入管理员界面，操作前请接管控制');
    } else {
      $("admin-message").textContent = '密码不正确';
      $("admin-password").focus();
    }
  }
  // 语音面板据此显示现在能不能让机器人做动作（谁握着控制权、什么状态、在做什么动作）
  window.dispatchEvent(new CustomEvent("operator-status", {detail: {status, catalog, online: !!online, authenticated}}));
}
// 先把本地的持有标记清掉再发 release：否则服务端推来新状态前，每 300 ms 的续租可能紧跟着发出去，报「请先接管控制」
function releaseControl() {
  endMotion();
  if (status) status.owns_control = false;
  return request("release");
}
function motionAllowed() {
  return authenticated && status?.owns_control && status.online && status.state === 'RL' &&
    !status.fault?.latched && !suspended && !document.hidden &&
    performance.now() - lastStatusAt < STATUS_PAUSE_MS;
}
function motionInput() {
  if (inputMode === 'sticks') return {vx:-stickValues.move.y, vy:-stickValues.move.x, wz:-stickValues.turn.x};
  const held = new Set([...keys, ...[...pointers.values()].map(button => button.dataset.motion)]);
  const input = {vx: Number(held.has('w')) - Number(held.has('s')),
    vy: Number(held.has('a')) - Number(held.has('d')),
    wz: Number(held.has('j')) - Number(held.has('l'))};
  for (const axis of ['vx','vy','wz']) {
    const amount = Number($("speed-" + axis).value), limits = motion.limits(axis);
    const cap = input[axis] >= 0 ? limits.max : -limits.min;
    input[axis] *= cap > 0 && Number.isFinite(amount) ? Math.min(Math.max(0, amount), cap) / cap : 0;
  }
  return input;
}
function updateSpeedPresets(fill = false) {
  for (const axis of ['vx','vy','wz']) {
    const input = $("speed-" + axis), limits = motion.limits(axis);
    const cap = Math.max(-limits.min, limits.max);
    input.max = cap; input.step = catalog?.velocity_step?.[axis] || 0.1;
    input.disabled = !cap;
    // A stationary policy must not overwrite a nonzero movement preset.
    if (cap > 0) input.value = fill ? cap : Math.min(Math.max(0, Number(input.value) || 0), cap);
  }
}
function renderMotion() {
  const allowed = motionAllowed();
  const caps = ['vx','vy','wz'].map(axis => {
    const limits = motion.limits(axis);
    return Math.max(-limits.min, limits.max);
  });
  document.querySelectorAll('[data-motion]').forEach(button => {
    const key = button.dataset.motion;
    const axis = 'ws'.includes(key) ? 'vx' : 'ad'.includes(key) ? 'vy' : 'wz';
    const limits = motion.limits(axis), cap = 'waj'.includes(key) ? limits.max : -limits.min;
    button.disabled = !allowed || !cap;
    button.classList.toggle('active', keys.has(key) || [...pointers.values()].includes(button));
  });
  document.querySelectorAll('[data-stick]').forEach(stick => {
    const movable = stick.dataset.stick === 'move' ? caps[0] || caps[1] : caps[2];
    stick.setAttribute('aria-disabled', String(!allowed || !movable));
  });
  document.querySelectorAll('[data-speed-mode]').forEach(button => {
    const active = button.dataset.speedMode === motion.mode;
    button.classList.toggle('active', active);
    button.setAttribute('aria-pressed', String(active));
    button.disabled = !activePolicy();
  });
  $('speed-limits').textContent = activePolicy()
    ? `上限 · 前后 ${caps[0].toFixed(2)} / 横移 ${caps[1].toFixed(2)} m/s · 转向 ${caps[2].toFixed(2)} rad/s`
    : '等待策略限速';
  $('command-velocity').textContent = ['vx','vy','wz'].map(axis => motion.value[axis].toFixed(2)).join(' / ');
}
function endMotion(send = true) {
  const wasMoving = motion.moving || velocityDirty;
  velocityDirty = false; motionEpoch++;
  keys.clear();
  const held = [...pointers]; pointers.clear();
  for (const [pointer, button] of held) {
    button.classList.remove('active');
    if (button.hasPointerCapture(pointer)) button.releasePointerCapture(pointer);
  }
  const heldSticks = [...stickPointers]; stickPointers.clear();
  stickValues.move = {x:0,y:0}; stickValues.turn = {x:0,y:0};
  for (const [pointer, {element}] of heldSticks) {
    element.querySelector('.joystick-knob').style.transform = 'translate(-50%, -50%)';
    element.classList.remove('active');
    if (element.hasPointerCapture(pointer)) element.releasePointerCapture(pointer);
  }
  motion.reset();
  motionTickAt = performance.now();
  if (send && wasMoving && authenticated && status?.owns_control)
    call('velocity', {vx:0,vy:0,wz:0,valid_for_ms:300}).catch(() => {});
  $('command-velocity').textContent = '0.00 / 0.00 / 0.00';
}
document.querySelectorAll('[data-input-mode]').forEach(button => {
  if (button.tagName !== 'BUTTON') return;
  button.onclick = () => {
    if (inputMode === button.dataset.inputMode) return;
    endMotion(); inputMode = button.dataset.inputMode;
    document.body.dataset.inputMode = inputMode;
    document.querySelectorAll('.input-modes button').forEach(b => {
      b.classList.toggle('active', b === button);
      b.setAttribute('aria-pressed', String(b === button));
    });
    renderMotion();
  };
});
document.querySelectorAll('[data-stick]').forEach(element => {
  const update = e => {
    const held = stickPointers.get(e.pointerId);
    if (!held || held.element !== element) return;
    e.preventDefault();
    // Keep the gesture origin stable if mobile browser chrome resizes while held.
    const x = (e.clientX - held.x) / held.radius;
    const y = element.dataset.stick === 'turn' ? 0 : (e.clientY - held.y) / held.radius;
    stickValues[element.dataset.stick] = window.OperatorTeleop.stick(x, y, catalog?.teleop?.deadzone);
    const scale = Math.max(1, Math.hypot(x, y));
    element.querySelector('.joystick-knob').style.transform =
      `translate(calc(-50% + ${x / scale * held.radius}px), calc(-50% + ${y / scale * held.radius}px))`;
  };
  element.onpointerdown = e => {
    if (inputMode !== 'sticks' || e.button !== 0 || !motionAllowed() ||
        element.getAttribute('aria-disabled') === 'true' ||
        [...stickPointers.values()].some(p => p.element === element)) return;
    const rect = element.getBoundingClientRect();
    stickPointers.set(e.pointerId, {element, x:rect.left + rect.width / 2,
      y:rect.top + rect.height / 2, radius:Math.min(rect.width, rect.height) * 0.34});
    element.setPointerCapture(e.pointerId); element.classList.add('active'); update(e);
  };
  element.onpointermove = update;
  element.onpointerup = element.onpointercancel = element.onlostpointercapture = e => {
    if (stickPointers.get(e.pointerId)?.element !== element) return;
    stickPointers.delete(e.pointerId);
    stickValues[element.dataset.stick] = {x:0,y:0};
    element.querySelector('.joystick-knob').style.transform = 'translate(-50%, -50%)';
    element.classList.remove('active');
    if (element.hasPointerCapture(e.pointerId)) element.releasePointerCapture(e.pointerId);
  };
  element.oncontextmenu = e => e.preventDefault();
});
document.querySelectorAll('[data-state]').forEach(b => b.onclick = () => { endMotion(); request('state', {state:b.dataset.state}); });
document.querySelectorAll('[data-motion]').forEach(button => {
  button.onpointerdown = e => {
    if (e.button !== 0 || button.disabled || !motionAllowed()) return;
    clientEvent('direction_down', button.dataset.motion);
    e.preventDefault(); button.focus({preventScroll:true});
    pointers.set(e.pointerId, button); button.setPointerCapture(e.pointerId);
    renderMotion();
  };
  button.onpointerup = e => {
    if (pointers.get(e.pointerId) !== button) return;
    e.preventDefault(); pointers.delete(e.pointerId);
    if (button.hasPointerCapture(e.pointerId)) button.releasePointerCapture(e.pointerId);
    renderMotion();
  };
  button.onpointercancel = button.onlostpointercapture = e => {
    if (pointers.get(e.pointerId) === button) endMotion();
  };
  button.oncontextmenu = e => e.preventDefault();
});
document.querySelectorAll('[data-speed-mode]').forEach(button => {
  button.onclick = () => {
    const changed = motion.mode !== button.dataset.speedMode;
    motion.mode = button.dataset.speedMode; updateSpeedPresets(true); renderMotion();
    if (changed && status?.owns_control) clientEvent('teleop_preset', motion.mode);
  };
});
$("acquire").onclick = () => request('acquire');
$("release").onclick = () => releaseControl();
$("policy").onchange = () => render();
$("select-policy").onclick = () => { endMotion(); request('policy', {policy:$("policy").value}); };
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
    closeConnection(); window.operatorReconnect();
  } catch (_) { $("login-message").textContent = '链接不完整，请使用二维码中的连接链接'; }
};
$("disconnect").onclick = () => window.operatorDisconnect();
$("reconnect").onclick = () => window.operatorReconnect();
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
window.operatorDisconnect = () => {
  clientEvent('manual_disconnect');
  connectionWanted = false; clearTimeout(reconnectTimer); reconnectTimer = null;
  if (authenticated) call('release').catch(() => {});
  closeConnection();
};
window.operatorReconnect = () => {
  connectionWanted = true; suspended = false;
  if (token) connect();
};
function suspend(reason) {
  clientEvent(reason);
  suspended = true; clearTimeout(reconnectTimer); reconnectTimer = null;
  endMotion(false);
  if (authenticated) call('release').catch(() => {});
}
function resume() { suspended = false; if (connectionWanted) connect(); }
document.addEventListener('visibilitychange', () => { if (document.hidden) suspend('page_hidden'); else resume(); });
window.addEventListener('pagehide', () => suspend('pagehide'));
window.addEventListener('pageshow', resume);
window.addEventListener('online', resume);
window.addEventListener('blur', () => endMotion());
window.addEventListener('error', e => clientEvent('script_error', e.message));
window.addEventListener('unhandledrejection', e => clientEvent('promise_error', e.reason?.message || String(e.reason)));
window.addEventListener('keydown', e => {
  if (['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName) || e.repeat) return;
  if (document.body.dataset.mode !== 'control') return;
  if (e.code === 'Space') { e.preventDefault(); endMotion(false); request('stop'); }
  if (e.key.toLowerCase() === 'c' && !$("cancel-action").disabled) request('cancel');
  const key = e.key.toLowerCase();
  if ('wsadjl'.includes(key) && key.length === 1 && motionAllowed()) { e.preventDefault(); keys.add(key); }
});
window.addEventListener('keyup', e => keys.delete(e.key.toLowerCase()));
document.querySelectorAll('[data-view]').forEach(button => {
  if (button.tagName !== 'BUTTON') return;
  button.onclick = () => {
    endMotion(); document.body.dataset.view = button.dataset.view;
    document.querySelectorAll('.mobile-views button').forEach(b => b.classList.toggle('active', b === button));
  };
});
setInterval(() => {
  if (!authenticated || suspended || document.hidden) return;
  const age = performance.now() - lastStatusAt;
  if (age >= STATUS_PAUSE_MS) {
    endMotion();
    if (!statusStale) { statusStale = true; tell("状态更新延迟，遥控输入已暂停，等待自动恢复"); }
    render();
    if (age >= RECONNECT_AFTER_MS) { closeConnection(); scheduleReconnect(); }
    return;
  }
  const current = socket;
  if (status?.owns_control) call('renew').catch(e => {
    if (socket === current && e.remote) { endMotion(); tell(e.message); }
  });
}, 300);
setInterval(() => {
  const now = performance.now(), dt = (now - motionTickAt) / 1000;
  motionTickAt = now;
  if (!motionAllowed()) { endMotion(); return; }
  const wasMoving = motion.moving;
  const v = motion.step(motionInput(), dt);
  velocityDirty ||= wasMoving || motion.moving;
  renderMotion();
  // Advance every tick; slow replies must not throttle the velocity stream.
  // If transport is backed up, keep only the latest value, including final zero.
  if (!velocityDirty || socket?.bufferedAmount > 0) return;
  const current = socket, epoch = motionEpoch;
  velocityDirty = false;
  call('velocity', {...v,valid_for_ms:300}).catch(e => {
    if (socket === current && epoch === motionEpoch && e.remote) {
      clientEvent('velocity_rejected', e.message);
      endMotion(); tell(e.message);
    }
  });
}, 50);
if (window.lucide) lucide.createIcons();
// Voice runs through the existing HTTPS proxy. Control authorization is per WebSocket session.
let voicePort = 0;
function setMode(mode) {
  if (mode === 'control' && (!authenticated || (catalog?.admin_required && !status?.admin))) return;
  const voice = mode === "voice";
  document.body.dataset.mode = voice ? "voice" : "control";
  const hasPublicPage = Boolean(catalog?.admin_required || voicePort > 0);
  $("admin-entry").hidden = !voice || !hasPublicPage;
  $("admin-exit").hidden = voice || !hasPublicPage;
  $("voice-panel").hidden = !voice;
  window.voicePanel?.setActive(voice);
}
function showPublicPage() {
  if (!catalog?.admin_required && !voicePort) { setMode('control'); return; }
  setMode('voice');
  if (!window.voicePanel && voicePort > 0 && token) {
    const url = `http://${location.hostname}:${voicePort}/#token=${encodeURIComponent(token)}`;
    if ($("open-voice")) { $("open-voice").href = url; $("open-voice").hidden = window.isSecureContext || inApp; }
  }
  if ($("voice-message")) $("voice-message").textContent = inApp ? '语音会话在手机浏览器中打开' :
    window.isSecureContext ? '语音服务连接中' : '语音交互';
}
$("admin-entry").onclick = () => {
  if (status?.admin || !catalog?.admin_required) { setMode('control'); render(); return; }
  if (!window.isSecureContext && voicePort > 0 && !inApp) {
    location.assign(`http://${location.hostname}:${voicePort}/#token=${encodeURIComponent(token)}`);
    return;
  }
  $("admin-password").value = '';
  $("admin-message").textContent = '';
  $("admin-dialog").showModal();
  $("admin-password").focus();
};
$("close-admin").onclick = () => $("admin-dialog").close();
$("admin-dialog").onclose = () => {
  $("admin-password").value = '';
  if (adminLoginWaiting) call('admin_logout').catch(() => {});
  adminLoginWaiting = false;
  $("admin-submit").disabled = false;
};
$("admin-login").onsubmit = async e => {
  e.preventDefault();
  $("admin-submit").disabled = true;
  $("admin-message").textContent = '正在验证';
  let password = $("admin-password").value;
  $("admin-password").value = '';
  try {
    await call('admin_login', {password});
    password = '';
    if (!$("admin-dialog").open) { await call('admin_logout'); return; }
    adminLoginWaiting = true;
    const result = await call('status');
    status = result.data.status;
    render();
  } catch (error) {
    password = '';
    adminLoginWaiting = false;
    $("admin-submit").disabled = false;
    $("admin-message").textContent = error.message;
  }
};
$("admin-exit").onclick = async () => {
  if (status?.owns_control && ['BLEND_IN','PLAYING','HOLDING','BLEND_OUT'].includes(status.interaction.phase)) {
    tell('动作执行中，请先取消并等待收回完成'); return;
  }
  if (!await request('admin_logout')) return;
  if (status) { status.admin = false; status.owns_control = false; }
  setMode('voice'); render();
};
if (window.isSecureContext && !inApp) {
  const script = document.createElement("script");
  script.src = "/voice/panel.js";
  script.onload = () => { if (document.body.dataset.mode === 'voice') window.voicePanel?.setActive(true); };
  script.onerror = () => { if ($("voice-message")) $("voice-message").textContent = '语音服务暂不可用'; };
  document.head.append(script);
}
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
