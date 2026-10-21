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
const logList = document.getElementById('log');

const rangeInputs = Array.from(document.querySelectorAll('.slider-row input[type="range"]'));

const STALE_MS = 5000; // tempo sem frame/pose até marcar badge como desconectado

let ws = null;
let currentPose = null;   // {x, y, px?, py?}
let initialPose = null;   // primeira pose recebida {x, y}
let currentGoal = null;   // {x, y, px, py}
let hasFrame = false;
let lastFrameAt = 0;
let lastPoseAt = 0;

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
    case 'error':
      addLog(msg.message, true);
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
}

function drawCross(x, y, color) {
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(x - 10, y); ctx.lineTo(x + 10, y);
  ctx.moveTo(x, y - 10); ctx.lineTo(x, y + 10);
  ctx.stroke();
}

function framePoint(e) {
  const rect = canvas.getBoundingClientRect();
  return {
    x: (e.clientX - rect.left) * (canvas.width / rect.width),
    y: (e.clientY - rect.top) * (canvas.height / rect.height),
  };
}

// Clique no vídeo -> alvo em coordenadas do frame original
canvas.addEventListener('click', (e) => {
  if (!hasFrame) return;
  const { x, y } = framePoint(e);
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
