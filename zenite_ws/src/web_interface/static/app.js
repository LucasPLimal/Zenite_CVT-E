// Interface Web do Zênite — cliente WebSocket.
// Binário = frame JPEG; texto = JSON (pose, status, goal_ack, error).

const canvas = document.getElementById('video-canvas');
const ctx = canvas.getContext('2d');
const noVideo = document.getElementById('no-video');
const connBadge = document.getElementById('conn-badge');
const homogBadge = document.getElementById('homog-badge');
const poseValue = document.getElementById('pose-value');
const goalValue = document.getElementById('goal-value');
const logList = document.getElementById('log');

let ws = null;
let currentPose = null;   // {x, y, px?, py?}
let currentGoal = null;   // {x, y, px, py}
let hasFrame = false;

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

function handleJson(msg) {
  switch (msg.type) {
    case 'pose':
      currentPose = msg;
      poseValue.textContent = `(${msg.x.toFixed(2)}, ${msg.y.toFixed(2)}) m`;
      break;
    case 'status':
      setBadge(homogBadge, msg.homography,
        msg.homography ? 'Calibrado' : 'Sem calibração');
      break;
    case 'goal_ack':
      currentGoal = msg;
      goalValue.textContent = `(${msg.x.toFixed(2)}, ${msg.y.toFixed(2)}) m`;
      addLog(`Alvo enviado: (${msg.x.toFixed(2)}, ${msg.y.toFixed(2)}) m`);
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
  // Pose atual do robô (telemetria) — verde
  if (currentPose && currentPose.px !== undefined) {
    drawCircle(currentPose.px, currentPose.py, '#5fd38d');
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

// Clique no vídeo -> alvo em coordenadas do frame original
canvas.addEventListener('click', (e) => {
  if (!hasFrame) return;
  const rect = canvas.getBoundingClientRect();
  const x = (e.clientX - rect.left) * (canvas.width / rect.width);
  const y = (e.clientY - rect.top) * (canvas.height / rect.height);
  send({ type: 'goal', x, y });
});

// ------------------------------------------------------------------ //
// Sliders (0-255 na UI -> 0.0-1.0 no ROS, como no interface_node)
// ------------------------------------------------------------------ //

const sliders = ['brightness', 'saturation', 'hue'].map((id) => ({
  id,
  input: document.getElementById(id),
  label: document.getElementById(`${id}-value`),
}));

let paramsTimer = null;

function sendParams() {
  const params = {};
  for (const s of sliders) params[s.id] = Number(s.input.value) / 255.0;
  send({ type: 'params', ...params });
}

for (const s of sliders) {
  s.input.addEventListener('input', () => {
    s.label.textContent = s.input.value;
    clearTimeout(paramsTimer);
    paramsTimer = setTimeout(sendParams, 100); // throttle
  });
}

// ------------------------------------------------------------------ //
// Calibração / util
// ------------------------------------------------------------------ //

document.getElementById('reload-homography').addEventListener('click', () => {
  send({ type: 'reload_homography' });
  addLog('Recarregando homografia…');
});

function setBadge(el, on, text) {
  el.textContent = text;
  el.classList.toggle('on', on);
  el.classList.toggle('off', !on);
}

function addLog(text, isError = false) {
  const li = document.createElement('li');
  li.textContent = `${new Date().toLocaleTimeString()} — ${text}`;
  if (isError) li.classList.add('error');
  logList.prepend(li);
  while (logList.children.length > 30) logList.removeChild(logList.lastChild);
}

connect();
