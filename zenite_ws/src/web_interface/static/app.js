// Interface Web do Zênite — cliente WebSocket.
// Binário = frame JPEG; texto = JSON (pose, status, goal_ack, error).

const canvas = document.getElementById('video-canvas');
const ctx = canvas.getContext('2d');
const noVideo = document.getElementById('no-video');
const navButtons = Array.from(document.querySelectorAll('.navRow button[data-target]'));
const tabSections = Array.from(document.querySelectorAll('main > section > div[id]'));
const connBadge = document.getElementById('conn-badge');
const camBadge = document.getElementById('cam-badge');
const homogBadge = document.getElementById('homog-badge');
const locBadge = document.getElementById('loc-badge');
const poseValue = document.getElementById('pose-value');
const initialValue = document.getElementById('initial-value');
const goalValue = document.getElementById('goal-value');
const lastGoalValue = document.getElementById('last-goal-value');
const calibAreaValue = document.getElementById('calib-area-value');
const logList = document.getElementById('log');
const robotSelect = document.getElementById('robot-select');
const robotGrid = document.getElementById('robot-grid');
const robotEmpty = document.getElementById('robot-empty');
const robotCount = document.getElementById('robot-count');
const releaseAllBtn = document.getElementById('release-all-btn');
const robotBar = document.getElementById('robot-bar');
const robotBarName = document.getElementById('robot-bar-name');
const changeRobotBtn = document.getElementById('change-robot-btn');
const controlMain = document.getElementById('control-main');
const btScanBtn = document.getElementById('bt-scan-btn');
const btDevicesEl = document.getElementById('bt-devices');
const btHint = document.getElementById('bt-hint');
const calibrateToggle = document.getElementById('calibrate-toggle');
const switchPont = document.getElementById('switchPont');
const switchLoc = document.getElementById('switchLoc');
const switchPerc = document.getElementById('switchPerc');

const rangeInputs = Array.from(document.querySelectorAll('.slider-row input[type="range"]'));

const MAX_ROBOTS = 8;            // limite de robôs na lista
const RELEASE_CONFIRM_MS = 4000; // tempo para confirmar "tornar todos livres"
// Cores por ID (id % 6), na mesma ordem usadas pelo localization_node.
const ROBOT_COLORS = ['#22c55e', '#ef4444', '#3b82f6', '#eab308', '#d946ef', '#06b6d4'];
const STALE_MS = 5000; // tempo sem frame/pose até marcar badge como desconectado
const TRAIL_MAX = 80;  // pontos guardados no rastro de percurso
const CALIBRATION_HOLD_MS = 1000; // tempo que os pontos ficam verdes após o 4º clique

let ws = null;
let currentPose = null;   // {x, y, px?, py?}
let initialPose = null;   // primeira pose recebida {x, y}
let currentGoal = null;   // {x, y, px, py}
let hasFrame = false;
let lastFrameAt = 0;
let lastPoseAt = 0;
// Multiusuário: a tela de escolha só aparece quando o servidor envia a lista
// de robôs. Enquanto isso não acontece, a interface funciona como antes.
let multiUser = false;
let robots = [];          // [{id, name, status: 'free'|'busy'|'mine', bluetooth, marker}]
let myRobotId = null;     // robô reservado para esta sessão
let pendingRobotId = null; // clique em "Conectar" aguardando a resposta do servidor
let releaseAllTimer = null;
let btScanning = false;   // o servidor está procurando dispositivos Bluetooth
let btDevices = [];       // [{mac, name, likely_robot, connected}]
let btConnectingMac = null; // dispositivo com tentativa de conexão em andamento
let poseTrail = [];       // últimas posições (px, py) para o "percurso do robô"

// Calibração: modo de clique alternativo que marca os 4 cantos da arena
// (ordem: (0,0) -> (2.75,0) -> (2.75,2.25) -> (0,2.25)) em vez de enviar um alvo.
let calibrating = false;
let calibrationPoints = [];           // [{x, y}, ...] em progresso, até 4
let calibrationJustCompleted = false; // true entre o 4º clique e o fim do CALIBRATION_HOLD_MS
let lastCalibrationPoints = null;     // pontos da última calibração concluída
let calibrationHoldTimer = null;


function setupSectionTabs() {
  const buttons = navButtons;
  const sections = tabSections;

  if (!buttons.length || !sections.length) {
    return;
  }

  let transitionInProgress = false;
  let queuedTargetId = null;

  function activateSection(targetId) {
    const nextSection = sections.find((section) => section.id === targetId);
    if (!nextSection) {
      return;
    }

    buttons.forEach((button) => {
      button.classList.toggle('selected', button.dataset.target === targetId);
    });

    if (transitionInProgress) {
      queuedTargetId = targetId;
      return;
    }

    const currentSection = sections.find((section) => !section.classList.contains('hidden'));
    if (currentSection === nextSection) {
      return;
    }

    const showSection = () => {
      nextSection.classList.remove('hidden');
      nextSection.classList.add('block');
    };

    if (!currentSection || window.matchMedia('(prefers-reduced-motion: reduce)').matches) {
      sections.forEach((section) => {
        const isActive = section === nextSection;
        section.classList.toggle('block', isActive);
        section.classList.toggle('hidden', !isActive);
      });
      return;
    }

    transitionInProgress = true;
    const finishFadeOut = (event) => {
      if (event.target !== currentSection || event.animationName !== 'tabFadeOut') {
        return;
      }

      currentSection.removeEventListener('animationend', finishFadeOut);
      currentSection.classList.remove('tab-panel-leaving', 'block');
      currentSection.classList.add('hidden');
      showSection();
      transitionInProgress = false;

      if (queuedTargetId) {
        const queuedTarget = queuedTargetId;
        queuedTargetId = null;
        activateSection(queuedTarget);
      }
    };

    currentSection.addEventListener('animationend', finishFadeOut);
    currentSection.classList.add('tab-panel-leaving');
  }

  buttons.forEach((button) => {
    button.addEventListener('click', () => {
      activateSection(button.dataset.target);
    });
  });

  const initialTarget = buttons.find((button) => button.classList.contains('selected'))?.dataset.target
    || buttons[0]?.dataset.target;

  if (initialTarget) {
    activateSection(initialTarget);
  }
}

function updateRangeFill(range) {
  const min = Number(range.min || 0);
  const max = Number(range.max || 100);
  const value = Number(range.value || min);
  const percent = ((value - min) / (max - min || 1)) * 100;

  range.style.setProperty('--value', `${percent}%`);
}

rangeInputs.forEach((range) => {
  updateRangeFill(range);
  range.addEventListener('input', () => updateRangeFill(range));
});

document.addEventListener('DOMContentLoaded', setupSectionTabs);

// ------------------------------------------------------------------ //
// WebSocket
// ------------------------------------------------------------------ //

// Identificador deste navegador. O servidor usa o token para manter o robô
// reservado por alguns segundos se a conexão cair, e devolvê-lo na volta.
function getToken() {
  try {
    let token = localStorage.getItem('zenite-token');
    if (!token) {
      token = (crypto.randomUUID ? crypto.randomUUID()
        : `${Date.now()}-${Math.random().toString(36).slice(2)}`);
      localStorage.setItem('zenite-token', token);
    }
    return token;
  } catch (e) {
    // localStorage indisponível (modo privado, etc.): token só desta aba.
    if (!window.__zeniteToken) {
      window.__zeniteToken = `${Date.now()}-${Math.random().toString(36).slice(2)}`;
    }
    return window.__zeniteToken;
  }
}

function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.binaryType = 'blob';

  ws.onopen = () => {
    setBadge(connBadge, true, 'Conectado');
    send({ type: 'hello', token: getToken() });
  };

  ws.onclose = () => {
    setBadge(connBadge, false, 'Desconectado');
    setTimeout(connect, 1500); // reconexão automática
  };

  ws.onmessage = (event) => {
    if (event.data instanceof Blob) {
      drawFrame(event.data);
    } else {
      handleJson(JSON.parse(event.data));
    }
  };
}

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  }
}

function formatMeters(p) {
  return `(${p.x.toFixed(2)}, ${p.y.toFixed(2)}) m`;
}

// Pose do robô: posição em metros e, se o servidor enviar, o ângulo (theta).
function formatPose(p) {
  const base = formatMeters(p);
  if (typeof p.theta !== 'number') return base;
  return `${base} · ${(p.theta * 180 / Math.PI).toFixed(0)}°`;
}

// Ao trocar de robô a telemetria anterior deixa de valer.
function resetTelemetry() {
  currentPose = null;
  initialPose = null;
  currentGoal = null;
  lastPoseAt = 0;
  poseTrail = [];
  [poseValue, initialValue, goalValue, lastGoalValue].forEach((el) => {
    el.textContent = '—';
  });
}

function handleJson(msg) {
  switch (msg.type) {
    case 'pose':
      currentPose = msg;
      lastPoseAt = Date.now();
      poseValue.textContent = formatPose(msg);
      if (msg.px !== undefined) {
        poseTrail.push({ x: msg.px, y: msg.py });
        if (poseTrail.length > TRAIL_MAX) poseTrail.shift();
      }
      if (!initialPose) {
        initialPose = { x: msg.x, y: msg.y };
        initialValue.textContent = formatMeters(initialPose);
      }
      break;
    case 'robots':
      multiUser = true;
      robots = Array.isArray(msg.robots) ? msg.robots.slice(0, MAX_ROBOTS) : [];
      pendingRobotId = null;
      {
        const previous = myRobotId;
        myRobotId = robots.find((r) => r.status === 'mine')?.id ?? null;
        if (previous !== myRobotId) resetTelemetry();
      }
      renderRobotSelect();
      updateRobotView();
      break;
    case 'bluetooth':
      {
        const wasScanning = btScanning;
        btScanning = !!msg.scanning;
        btDevices = Array.isArray(msg.devices) ? msg.devices : [];
        if (wasScanning && !btScanning) {
          showBtHint(btDevices.length
            ? 'Escolha na lista o dispositivo do seu robô.'
            : 'Nenhum dispositivo encontrado. Confira se o robô está ligado.');
        }
      }
      renderBluetooth();
      break;
    case 'bt_event':
      handleBluetoothEvent(msg);
      break;
    case 'status':
      setBadge(homogBadge, msg.homography,
        msg.homography ? 'Calibrado' : 'Sem calibração');
      break;
    case 'goal_ack':
      if (currentGoal) lastGoalValue.textContent = formatMeters(currentGoal);
      currentGoal = msg;
      goalValue.textContent = formatMeters(msg);
      addLog(`Alvo enviado: ${formatMeters(msg)}`);
      break;
    case 'calibrate_ack':
      // O overlay verde já está visível desde o 4º clique e some sozinho
      // após CALIBRATION_HOLD_MS (ver finishCalibrationHold).
      addLog('Calibração salva com sucesso.');
      setBadge(homogBadge, msg.homography, msg.homography ? 'Calibrado' : 'Sem calibração');
      break;
    case 'error':
      addLog(msg.message, true);
      if (calibrating) {
        // Pontos inválidos: cancela o overlay verde e deixa tentar de novo.
        clearTimeout(calibrationHoldTimer);
        calibrationJustCompleted = false;
        calibrationPoints = [];
      }
      break;
  }
}

// ------------------------------------------------------------------ //
// Vídeo + overlays
// ------------------------------------------------------------------ //

async function drawFrame(blob) {
  let bitmap;
  try {
    bitmap = await createImageBitmap(blob);
  } catch {
    return; // frame corrompido, ignora
  }

  if (canvas.width !== bitmap.width || canvas.height !== bitmap.height) {
    canvas.width = bitmap.width;
    canvas.height = bitmap.height;
  }

  ctx.drawImage(bitmap, 0, 0);
  bitmap.close();

  lastFrameAt = Date.now();
  if (!hasFrame) {
    hasFrame = true;
    noVideo.classList.add('hidden');
  }

  drawMarkers();
}

function drawMarkers() {
  // Alvo (clique do usuário) — vermelho
  if (currentGoal) {
    drawCross(currentGoal.px, currentGoal.py, '#ff5566');
  }

  // Percurso do robô — toggle "Ver percurso do robô"
  if (switchPerc.checked && poseTrail.length > 1) {
    drawTrail(poseTrail, '#8b8bd8');
  }

  // Pose atual do robô — toggle "Ver localização do robô"
  if (switchLoc.checked && currentPose && currentPose.px !== undefined) {
    drawCircle(currentPose.px, currentPose.py, '#5fd38d');
  }

  // Pontos de calibração: durante a calibração aparecem sempre e ficam verdes
  // ao completar; depois, só os da última calibração via toggle.
  if (calibrating && calibrationPoints.length > 0) {
    drawCalibrationOverlay(calibrationPoints, calibrationJustCompleted);
  } else if (!calibrating && switchPont.checked && lastCalibrationPoints) {
    drawCalibrationOverlay(lastCalibrationPoints, false);
  }
}

function drawTrail(points, color) {
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(points[0].x, points[0].y);
  for (let i = 1; i < points.length; i++) ctx.lineTo(points[i].x, points[i].y);
  ctx.stroke();
}

// Espelha o overlay do calibration_node (C++): círculos nos pontos e linhas
// entre eles, fechando o quadrilátero no 4º ponto.
function drawCalibrationOverlay(points, complete) {
  ctx.fillStyle = complete ? '#22c55e' : '#ffcc00';
  ctx.strokeStyle = complete ? '#22c55e' : '#33bbff';
  ctx.lineWidth = 2;

  for (const p of points) {
    ctx.beginPath();
    ctx.arc(p.x, p.y, 5, 0, 2 * Math.PI);
    ctx.fill();
  }

  if (points.length > 1) {
    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);
    for (let i = 1; i < points.length; i++) ctx.lineTo(points[i].x, points[i].y);
    if (points.length === 4) ctx.closePath();
    ctx.stroke();
  }
}

function drawCross(x, y, color) {
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(x - 10, y); ctx.lineTo(x + 10, y);
  ctx.moveTo(x, y - 10); ctx.lineTo(x, y + 10);
  ctx.stroke();
}

function drawCircle(x, y, color) {
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(x, y, 12, 0, 2 * Math.PI);
  ctx.stroke();
}

function framePoint(e) {
  const rect = canvas.getBoundingClientRect();
  return {
    x: (e.clientX - rect.left) * (canvas.width / rect.width),
    y: (e.clientY - rect.top) * (canvas.height / rect.height),
  };
}

// Clique no vídeo -> alvo (modo normal) ou ponto de calibração (modo calibração).
canvas.addEventListener('click', (e) => {
  if (!hasFrame) return;
  const { x, y } = framePoint(e);

  if (calibrating) {
    if (calibrationPoints.length >= 4) return; // aguardando o overlay verde sumir
    calibrationPoints.push({ x, y });
    addLog(`Ponto ${calibrationPoints.length} de 4.`);

    if (calibrationPoints.length === 4) {
      calibrationJustCompleted = true;
      send({ type: 'calibrate', points: calibrationPoints.map((p) => [p.x, p.y]) });
      addLog('Enviando calibração…');
      calibrationHoldTimer = setTimeout(finishCalibrationHold, CALIBRATION_HOLD_MS);
    }
    return;
  }

  send({ type: 'goal', x, y });
});

// ------------------------------------------------------------------ //
// Sliders (0-255 na UI -> 0.0-1.0 no ROS, como no interface_node)
// Só brilho, saturação e matiz existem em image_params; os demais
// sliders do layout ficam desativados no HTML.
// ------------------------------------------------------------------ //

const sliders = ['brightness', 'saturation', 'hue'].map((id) => ({
  id,
  input: document.getElementById(id),
}));

let paramsTimer = null;

function sendParams() {
  const params = {};
  for (const s of sliders) params[s.id] = Number(s.input.value) / 255.0;
  send({ type: 'params', ...params });
}

for (const s of sliders) {
  s.input.addEventListener('input', () => {
    clearTimeout(paramsTimer);
    paramsTimer = setTimeout(sendParams, 100); // throttle
  });
}

// ------------------------------------------------------------------ //
// Calibração, localização do robô e homografia
// ------------------------------------------------------------------ //

document.getElementById('reload-homography').addEventListener('click', () => {
  send({ type: 'reload_homography' });
  addLog('Recarregando homografia…');
});

calibrateToggle.addEventListener('click', () => {
  if (calibrating) {
    exitCalibrationMode();
    return;
  }
  calibrating = true;
  calibrationPoints = [];
  canvas.classList.add('calibrating');
  calibrateToggle.textContent = 'Cancelar calibração';
  addLog('Calibração iniciada: clique nos 4 cantos da arena.');
});

// Depois do 4º ponto: guarda como "última calibração" e sai do modo.
function finishCalibrationHold() {
  lastCalibrationPoints = calibrationPoints;
  updateCalibrationArea();
  exitCalibrationMode();
}

function exitCalibrationMode() {
  clearTimeout(calibrationHoldTimer);
  calibrating = false;
  calibrationPoints = [];
  calibrationJustCompleted = false;
  canvas.classList.remove('calibrating');
  calibrateToggle.textContent = 'Calibrar pontos';
  addLog(`Ponto ${calibrationPoints.length} de 4.`)
}

// Telemetria "Área de calibragem": caixa envolvente (x mín, y mín, x máx, y máx) em px.
function updateCalibrationArea() {
  if (!lastCalibrationPoints) return;
  const xs = lastCalibrationPoints.map((p) => p.x);
  const ys = lastCalibrationPoints.map((p) => p.y);
  const f = (v) => Math.round(v);
  calibAreaValue.textContent =
    `(${f(Math.min(...xs))}, ${f(Math.min(...ys))}, ${f(Math.max(...xs))}, ${f(Math.max(...ys))}) px`;
}

// ------------------------------------------------------------------ //
// Escolha de robô (multiusuário)
//
// Protocolo:
//   servidor -> cliente  {type: 'robots', robots: [{id, name, status, bluetooth, marker}]}
//                        (reenviado a cada mudança; status 'mine' = reservado para esta sessão)
//   cliente -> servidor  {type: 'select_robot', id} | {type: 'release_robot'} | {type: 'release_all'}
// ------------------------------------------------------------------ //

function robotColor(id) {
  return ROBOT_COLORS[((id % ROBOT_COLORS.length) + ROBOT_COLORS.length) % ROBOT_COLORS.length];
}

function makeEl(tag, className, text) {
  const el = document.createElement(tag);
  if (className) el.className = className;
  if (text !== undefined) el.textContent = text;
  return el;
}

function makeStatusChip(robot) {
  if (robot.status === 'mine') return makeEl('span', 'success', 'Seu robô');
  if (robot.status === 'busy') return makeEl('span', 'error', 'Ocupado');
  if (!robot.bluetooth) return makeEl('span', 'neutral', 'Offline');
  return makeEl('span', 'success', 'Livre');
}

function makeCheck(ok, okText, failText) {
  return makeEl('li', ok ? 'ok' : 'fail', `${ok ? '✓' : '✕'} ${ok ? okText : failText}`);
}

function buildRobotCard(robot) {
  const mine = robot.status === 'mine';
  const busy = robot.status === 'busy';
  const offline = !robot.bluetooth && !mine;

  const card = makeEl('article', 'robot-card');
  if (mine) card.classList.add('mine');
  if (busy) card.classList.add('busy');
  if (offline) card.classList.add('offline');
  card.style.setProperty('--robot-color', robotColor(robot.id));

  const head = makeEl('div', 'robot-card-head');
  head.append(makeEl('span', 'robot-dot'), makeEl('h2', '', robot.name), makeEl('span', 'robot-id', `ID ${robot.id}`));

  const chips = makeEl('div', 'badges');
  chips.append(makeStatusChip(robot));

  const checks = makeEl('ul', 'robot-checks');
  checks.append(
    makeCheck(robot.bluetooth, 'Bluetooth conectado', 'Bluetooth desconectado'),
    makeCheck(robot.marker, 'Marcador visível na câmera', 'Marcador não visto'),
  );

  const actions = makeEl('div', 'robot-actions');
  const button = makeEl('button', 'buttonPrimary');
  button.type = 'button';

  if (mine) {
    button.textContent = 'Em uso por você';
    button.disabled = true;
  } else {
    button.textContent = busy ? 'Em uso' : (offline ? 'Indisponível' : 'Conectar');
    button.disabled = busy || offline || pendingRobotId !== null;
    button.addEventListener('click', () => selectRobot(robot));
  }
  actions.append(button);

  card.append(head, chips, checks, actions);
  return card;
}

function renderRobotSelect() {
  robotGrid.replaceChildren(...robots.map(buildRobotCard));
  robotEmpty.classList.toggle('hidden', robots.length > 0);

  const free = robots.filter((r) => r.status === 'free' && r.bluetooth).length;
  robotCount.textContent = `${robots.length} de ${MAX_ROBOTS} robôs · ${free} livres`;

  const mine = robots.find((r) => r.id === myRobotId);
  if (mine) {
    robotBarName.textContent = mine.name;
    robotBar.style.setProperty('--robot-color', robotColor(mine.id));
  }
}

// ------------------------------------------------------------------ //
// Conexão Bluetooth de robôs novos (painel da tela de escolha)
// ------------------------------------------------------------------ //

function showBtHint(text, isError = false) {
  btHint.textContent = text;
  btHint.classList.toggle('hidden', !text);
  btHint.classList.toggle('error', isError);
}

function renderBluetooth() {
  btScanBtn.disabled = btScanning;
  btScanBtn.textContent = btScanning ? 'Procurando…' : 'Procurar robôs';

  const items = btDevices.map((d) => {
    const li = makeEl('li', 'bt-device');
    const info = makeEl('div', 'bt-device-info');
    info.append(
      makeEl('strong', null, d.name || 'Sem nome'),
      makeEl('span', 'bt-mac', d.mac),
    );
    if (d.likely_robot) info.append(makeEl('span', 'bt-tag', 'Provável robô'));

    const btn = makeEl('button', 'buttonSecondary');
    btn.type = 'button';
    if (d.connected) {
      btn.textContent = 'Conectado';
      btn.disabled = true;
    } else if (btConnectingMac === d.mac) {
      btn.textContent = 'Conectando…';
      btn.disabled = true;
    } else {
      btn.textContent = 'Conectar';
      btn.disabled = btConnectingMac !== null || btScanning;
      btn.addEventListener('click', () => connectDevice(d));
    }
    li.append(info, btn);
    return li;
  });
  btDevicesEl.replaceChildren(...items);

  if (!btScanning && btDevices.length === 0 && btHint.classList.contains('hidden')) {
    showBtHint('Nenhum dispositivo listado. Ligue o robô e clique em "Procurar robôs".');
  }
}

function connectDevice(device) {
  btConnectingMac = device.mac;
  showBtHint(`Conectando a ${device.name || device.mac}… isso pode levar alguns segundos.`);
  renderBluetooth();
  send({ type: 'bt_connect', mac: device.mac });
}

btScanBtn.addEventListener('click', () => {
  btScanning = true;
  showBtHint('Procurando dispositivos Bluetooth próximos…');
  renderBluetooth();
  send({ type: 'bt_scan' });
});

// Avisos do servidor para todos os usuários: robô conectado/desconectado e
// motivos de recusa (dispositivo que não é robô, robô fora do cadastro etc.).
function handleBluetoothEvent(msg) {
  const failed = ['connect_failed', 'not_a_robot', 'not_registered', 'scan_error']
    .includes(msg.event);
  addLog(msg.message || msg.event, failed);
  if (msg.mac && msg.mac === btConnectingMac) btConnectingMac = null;
  if (msg.event === 'robot_connected') btConnectingMac = null;
  showBtHint(msg.message || '', failed);
  renderBluetooth();
}

// Mostra a tela de escolha enquanto o usuário não tem robô e a interface de
// controle quando tem.
function updateRobotView() {
  const gate = multiUser && myRobotId === null;
  robotSelect.classList.toggle('hidden', !gate);
  controlMain.classList.toggle('hidden', gate);
  robotBar.classList.toggle('hidden', !(multiUser && myRobotId !== null));
}

function selectRobot(robot) {
  pendingRobotId = robot.id;
  renderRobotSelect();
  send({ type: 'select_robot', id: robot.id });
  addLog(`Conectando ao robô ${robot.name}…`);
}

changeRobotBtn.addEventListener('click', () => {
  send({ type: 'release_robot' });
  addLog('Robô liberado.');
});

// "Tornar todos os robôs livres": qualquer usuário pode usar (fase de testes).
// Pede uma segunda confirmação para evitar clique acidental.
function resetReleaseAllButton() {
  clearTimeout(releaseAllTimer);
  releaseAllBtn.classList.remove('confirming');
  releaseAllBtn.textContent = 'Tornar todos os robôs livres';
}

releaseAllBtn.addEventListener('click', () => {
  if (!releaseAllBtn.classList.contains('confirming')) {
    releaseAllBtn.classList.add('confirming');
    releaseAllBtn.textContent = 'Confirmar: desconectar todos?';
    releaseAllTimer = setTimeout(resetReleaseAllButton, RELEASE_CONFIRM_MS);
    return;
  }
  resetReleaseAllButton();
  send({ type: 'release_all' });
  addLog('Todos os robôs foram liberados.');
});

// ------------------------------------------------------------------ //
// Util
// ------------------------------------------------------------------ //

function setBadge(el, ok, text) {
  el.textContent = text;
  el.classList.toggle('success', ok);
  el.classList.toggle('error', !ok);
}

function addLog(text, isError = false) {
  const row = document.createElement('div');
  row.textContent = `${new Date().toLocaleTimeString()} — ${text}`;
  if (isError) row.style.color = 'var(--color-red-400)';
  logList.prepend(row);
  while (logList.children.length > 30) logList.removeChild(logList.lastChild);
}

// Badges de atividade recente (câmera / robô localizado)
setInterval(() => {
  const now = Date.now();

  const camOk = hasFrame && (now - lastFrameAt) < STALE_MS;
  setBadge(camBadge, camOk, camOk ? 'Câmera conectada' : 'Câmera desconectada');

  const locOk = currentPose !== null && (now - lastPoseAt) < STALE_MS;
  setBadge(locBadge, locOk, locOk ? 'Robô localizado' : 'Robô não localizado');
}, 1000);

connect();
