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
const calibrateToggle = document.getElementById('calibrate-toggle');
const calibrationStatus = document.getElementById('calibration-status');
const switchPont = document.getElementById('switchPont');
const switchLoc = document.getElementById('switchLoc');
const switchPerc = document.getElementById('switchPerc');
const locateBtn = document.getElementById('locate-robot-btn');

const rangeInputs = Array.from(document.querySelectorAll('.slider-row input[type="range"]'));

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
let poseTrail = [];       // últimas posições (px, py) para o "percurso do robô"

// Calibração: modo de clique alternativo que marca os 4 cantos da arena
// (ordem: (0,0) -> (2.75,0) -> (2.75,2.25) -> (0,2.25)) em vez de enviar um alvo.
let calibrating = false;
let calibrationPoints = [];           // [{x, y}, ...] em progresso, até 4
let calibrationJustCompleted = false; // true entre o 4º clique e o fim do CALIBRATION_HOLD_MS
let lastCalibrationPoints = null;     // pontos da última calibração concluída
let calibrationHoldTimer = null;

// Localizar robô: seleção por marquee (arraste), equivalente ao cv::selectROI
// do localization_node — inicializa o CSRT no backend.
let locating = false;   // aguardando o arraste
let tracking = false;   // rastreamento ativo no backend
let dragging = false;
let marqueeStart = null;   // {x, y} em coordenadas do frame
let marqueeCurrent = null;

function setupSectionTabs() {
  const buttons = navButtons;
  const sections = tabSections;

  if (!buttons.length || !sections.length) {
    return;
  }

  function activateSection(targetId) {
    buttons.forEach((button) => {
      button.classList.toggle('selected', button.dataset.target === targetId);
    });

    sections.forEach((section) => {
      const isActive = section.id === targetId;
      section.classList.toggle('block', isActive);
      section.classList.toggle('hidden', !isActive);
    });
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

function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.binaryType = 'blob';

  ws.onopen = () => setBadge(connBadge, true, 'Conectado');

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

function handleJson(msg) {
  switch (msg.type) {
    case 'pose':
      currentPose = msg;
      lastPoseAt = Date.now();
      poseValue.textContent = formatMeters(msg);
      if (msg.px !== undefined) {
        poseTrail.push({ x: msg.px, y: msg.py });
        if (poseTrail.length > TRAIL_MAX) poseTrail.shift();
      }
      if (!initialPose) {
        initialPose = { x: msg.x, y: msg.y };
        initialValue.textContent = formatMeters(initialPose);
      }
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
    case 'locate_ack':
      tracking = true;
      exitLocateMode();
      addLog('Robô localizado — rastreamento iniciado.');
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
        updateCalibrationStatus();
      }
      if (locating) {
        dragging = false;
        marqueeStart = marqueeCurrent = null;
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

  // Seleção do robô por marquee — só durante o arraste
  if (locating && dragging && marqueeStart && marqueeCurrent) {
    drawMarquee(normalizedRect(marqueeStart, marqueeCurrent));
  }
}

function normalizedRect(a, b) {
  return {
    x: Math.min(a.x, b.x),
    y: Math.min(a.y, b.y),
    width: Math.abs(b.x - a.x),
    height: Math.abs(b.y - a.y),
  };
}

// Retângulo da marquee com cruz inscrita (+) que acompanha o formato.
function drawMarquee(rect) {
  ctx.strokeStyle = '#22d3ee';
  ctx.lineWidth = 2;
  ctx.strokeRect(rect.x, rect.y, rect.width, rect.height);

  const cx = rect.x + rect.width / 2;
  const cy = rect.y + rect.height / 2;
  ctx.beginPath();
  ctx.moveTo(rect.x, cy); ctx.lineTo(rect.x + rect.width, cy);
  ctx.moveTo(cx, rect.y); ctx.lineTo(cx, rect.y + rect.height);
  ctx.stroke();
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
// No modo "Localizar robô" o clique não faz nada: a seleção é por arraste.
canvas.addEventListener('click', (e) => {
  if (!hasFrame || locating) return;
  const { x, y } = framePoint(e);

  if (calibrating) {
    if (calibrationPoints.length >= 4) return; // aguardando o overlay verde sumir
    calibrationPoints.push({ x, y });
    updateCalibrationStatus();

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

// Marquee: mousedown inicia, mousemove atualiza, mouseup envia o ROI.
canvas.addEventListener('mousedown', (e) => {
  if (!locating || !hasFrame) return;
  e.preventDefault();
  dragging = true;
  marqueeStart = framePoint(e);
  marqueeCurrent = marqueeStart;
});

canvas.addEventListener('mousemove', (e) => {
  if (!locating || !dragging) return;
  marqueeCurrent = framePoint(e);
  drawMarkers(); // redesenho imediato para feedback fluido
});

canvas.addEventListener('mouseup', (e) => {
  if (!locating || !dragging) return;
  dragging = false;
  const rect = normalizedRect(marqueeStart, framePoint(e));
  marqueeStart = marqueeCurrent = null;

  if (rect.width < 5 || rect.height < 5) return; // clique acidental: tenta de novo

  send({ type: 'locate_robot', roi: rect });
  addLog('Localizando robô…');
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
  exitLocateMode();
  calibrating = true;
  calibrationPoints = [];
  canvas.classList.add('calibrating');
  calibrateToggle.textContent = 'Cancelar calibração';
  updateCalibrationStatus();
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
  calibrateToggle.textContent = 'Calibrar ponto';
  updateCalibrationStatus();
}

function updateCalibrationStatus() {
  calibrationStatus.textContent = `Ponto ${calibrationPoints.length} de 4.`;
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

// Botão "Localizar robô": idle -> selecionando (cancelável) -> rastreando
// (o mesmo botão passa a parar o rastreamento).
locateBtn.addEventListener('click', () => {
  if (tracking) {
    stopTracking();
  } else if (locating) {
    exitLocateMode();
  } else {
    exitCalibrationMode();
    locating = true;
    canvas.classList.add('locating');
    locateBtn.textContent = 'Cancelar seleção';
    addLog('Selecione o robô arrastando um retângulo sobre ele.');
  }
});

function exitLocateMode() {
  locating = false;
  dragging = false;
  marqueeStart = marqueeCurrent = null;
  canvas.classList.remove('locating');
  locateBtn.textContent = tracking ? 'Parar rastreamento' : 'Localizar robô';
}

function stopTracking() {
  send({ type: 'reset_tracking' });
  tracking = false;
  currentPose = null;
  initialPose = null;
  poseTrail = [];
  poseValue.textContent = '—';
  initialValue.textContent = '—';
  exitLocateMode();
  addLog('Rastreamento parado.');
}

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
