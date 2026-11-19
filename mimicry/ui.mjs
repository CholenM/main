const $ = id => document.getElementById(id);
const key = document.querySelector('meta[name="mimicry-key"]').content;
const virtual = document.querySelector('meta[name="mimicry-virtual"]').content === '1';
const headers = {'X-Mimicry-Key': key, 'X-Mimicry-Client': crypto.randomUUID()};
const canvas = $('preview');
const ctx = canvas.getContext('2d');
const keys = new Set();
const displayedFrames = [];
const movementKeys = ['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', 'Space'];
let state, sample = {}, lastFrame = 0, drawnFrame = 0, generation = 0;
let frameError = '', presenceError = '', actionError = '';
let pendingObservation, sendingObservation = false;
let active = !document.hidden && document.hasFocus();

async function api(path, data, keepalive = false) {
  const options = {headers, cache: 'no-store'};
  if (data !== undefined) Object.assign(options, {
    method: 'POST', keepalive, body: JSON.stringify(data),
    headers: {...headers, 'Content-Type': 'application/json'},
  });
  const response = await fetch(`/api/${path}`, options);
  if (!response.ok) throw new Error((await response.json()).error || `HTTP ${response.status}`);
  return response;
}

function renderStatus() {
  if (!state) return;
  const robot = state.robot;
  const idle = robot?.state === 'IDLE' && !state.homing && !state.armed && !state.pending;
  const mimic = robot?.mimicry;
  const ready = idle && mimic?.enabled && !robot.spindle;
  $('mode').textContent = !robot ? 'Disconnected' : robot.simulator ? 'Simulator' : 'Hardware';
  $('center').disabled = !active || state.armed || state.homing || !!state.pending;
  $('arm').disabled = !active || !ready || !mimic.z_referenced || !state.center;
  $('disarm').disabled = !state.armed && !state.pending;
  $('zero').disabled = !ready || robot.limits.z !== 0;
  $('home').disabled = !ready || !mimic.z_referenced || robot.limits.z !== 1;
  $('reset').disabled = !['HOLD', 'ESTOP', 'FAULT'].includes(robot?.state);
  $('message').textContent = state.message;
  $('tracking').hidden = !state.tracking;
  if (state.tracking) {
    const now = performance.now();
    while (displayedFrames.length && displayedFrames[0] <= now - 1000) displayedFrames.shift();
    const {fps, inference_ms, age_ms} = state.tracking;
    $('tracking').textContent = `Tracking: ${fps} FPS · Preview: ${displayedFrames.length} FPS · ` +
      `Inference: ${inference_ms} ms · Latest frame: ${age_ms == null ? 'waiting' : `${age_ms} ms old`}`;
  }
  $('error').textContent = actionError || frameError || presenceError || state.camera_error || state.error ||
    (robot && !mimic?.enabled ? 'Mimicry is not enabled in this controller profile.' : '');
  const rows = [
    ['Controller', robot?.state], ['Gestures', state.paused ? 'Paused' : state.armed ? 'Armed' :
      state.pending ? 'Waiting for palm' : 'Disarmed'],
    ['Z bottom switch', robot ? (robot.limits?.z === 0 ? 'Pressed' : 'Clear') : null],
    ['Z zero', mimic?.z_referenced ? 'Set' : 'Not set'],
    ['Z upper cap', mimic?.enabled ? `${mimic.z_max_steps} steps` : null],
    ['Rotary home', robot?.homed ? 'Homed' : 'Not homed'],
    ...['j1', 'j2', 'z'].map(axis => [axis.toUpperCase(), robot?.pos?.[axis]]),
    ['Fault', robot?.fault],
  ];
  if (robot?.home_axis) rows.push(['Homing phase', `${robot.home_axis.toUpperCase()} ${robot.home_phase ?? ''}`]);
  if (state.last_home) {
    const {message, axis, phase} = state.last_home;
    rows.push(['Last homing', [message, axis?.toUpperCase(), phase].filter(Boolean).join(' · ')]);
  }
  $('status').replaceChildren(...rows.flatMap(([name, value]) => {
    const label = document.createElement('dt'), description = document.createElement('dd');
    label.textContent = name;
    description.textContent = value ?? '—';
    return [label, description];
  }));
}

async function poll() {
  try {
    state = await (await api('status')).json();
    renderStatus();
  } catch (error) {
    $('error').textContent = error.message;
  } finally {
    setTimeout(poll, 300);
  }
}

function overlay() {
  if (state?.center) {
    const [x, y] = state.center;
    ctx.strokeStyle = '#70d9ef';
    ctx.lineWidth = 3;
    ctx.strokeRect((x - .09) * canvas.width, (y - .09) * canvas.height,
      .18 * canvas.width, .18 * canvas.height);
  }
  if (sample.point) {
    ctx.fillStyle = '#83f5be';
    ctx.beginPath();
    ctx.arc(sample.point[0] * canvas.width, sample.point[1] * canvas.height, 8, 0, Math.PI * 2);
    ctx.fill();
  }
}

function fromKeyboard() {
  ctx.fillStyle = '#0d131a';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.font = '22px system-ui';
  ctx.fillStyle = '#bbcbd9';
  ctx.textAlign = 'center';
  ctx.fillText('Simulator joystick', canvas.width / 2, 50);
  if (keys.has('Space')) return {hand: 'fist'};
  const dx = Number(keys.has('ArrowRight')) - Number(keys.has('ArrowLeft'));
  const dy = Number(keys.has('ArrowDown')) - Number(keys.has('ArrowUp'));
  return {hand: 'open', point: [.5 + dx * .25, .5 + dy * .25]};
}

async function sendObservation(observation) {
  // Keep at most one unsent observation. Never replace a pending stop with an open palm.
  if (!pendingObservation || pendingObservation.hand === 'open' || observation.hand !== 'open') {
    pendingObservation = observation;
  }
  if (sendingObservation) return;
  sendingObservation = true;
  try {
    while (active && pendingObservation) {
      const next = pendingObservation;
      pendingObservation = undefined;
      const {frame, hand, point} = next;
      await api('observe', virtual ? {frame, hand, point} : {frame});
      frameError = '';
    }
  } catch (error) {
    pendingObservation = undefined;
    frameError = error.message;
  } finally {
    sendingObservation = false;
  }
}

async function sendPresence() {
  try {
    if (active && drawnFrame) {
      await api('presence', {frame: drawnFrame});
      presenceError = '';
    }
  } catch (error) {
    presenceError = error.message;
  } finally {
    setTimeout(sendPresence, 100);
  }
}

async function nextFrame() {
  const epoch = generation;
  let delay = virtual ? 35 : 0;
  try {
    if (!active) {
      delay = 100;
      return;
    }
    const response = await api(virtual ? 'frame' : `frame?after=${lastFrame}`);
    if (!active || epoch !== generation) return;
    const seq = Number(response.headers.get('X-Frame'));
    if (!seq || seq <= lastFrame) {
      delay = state?.camera_error ? 150 : 20;
      return;
    }
    lastFrame = seq;
    sample = virtual ? fromKeyboard() : JSON.parse(response.headers.get('X-Hand-Observation'));
    if (virtual) sendObservation({frame: seq, ...sample});
    await drawFrame(response, epoch, seq);
    if (!virtual) frameError = '';
  } catch (error) {
    delay = 100;
    frameError = error.message;
    sample = {};
  } finally {
    renderStatus();
    setTimeout(nextFrame, delay);
  }
}

async function drawFrame(response, epoch, sequence) {
  if (!virtual) {
    const bitmap = await createImageBitmap(await response.blob());
    if (active && epoch === generation) {
      if (canvas.width !== bitmap.width || canvas.height !== bitmap.height) {
        canvas.width = bitmap.width;
        canvas.height = bitmap.height;
      }
      ctx.drawImage(bitmap, 0, 0);
      drawnFrame = sequence;
      displayedFrames.push(performance.now());
    }
    bitmap.close();
  }
  if (!active || epoch !== generation) return;
  overlay();
  $('input').textContent = virtual ? 'Arrows: jog · Release: center · Space: fist' :
    (sample.hand === 'open' ? 'Open palm tracked.' :
      sample.hand === 'fist' ? 'Fist detected.' : 'Show one open palm with fingers and wrist in view.');
}

function focusChanged() {
  const nowActive = !document.hidden && document.hasFocus();
  if (nowActive === active) return;
  active = nowActive;
  generation++;
  if (!active) {
    keys.clear();
    sample = {};
    drawnFrame = 0;
    pendingObservation = undefined;
    api('disarm', {}, true).catch(() => {});
  }
}

window.addEventListener('focus', focusChanged);
window.addEventListener('blur', focusChanged);
document.addEventListener('visibilitychange', focusChanged);
window.addEventListener('pagehide', () => {
  active = false;
  generation++;
  drawnFrame = 0;
  pendingObservation = undefined;
  api('disarm', {}, true).catch(() => {});
});
if (virtual) {
  for (const event of ['keydown', 'keyup']) {
    window.addEventListener(event, e => {
      if (movementKeys.includes(e.code) && !(e.code === 'Space' && e.target instanceof HTMLButtonElement)) {
        e.preventDefault();
        if (event === 'keydown') keys.add(e.code);
        else keys.delete(e.code);
      }
    });
  }
}
for (const action of ['center', 'arm', 'disarm', 'zero', 'home', 'reset', 'estop']) {
  $(action).addEventListener('click', async () => {
    // Space is a fist in the keyboard simulator, not a repeat click on this button.
    if (virtual) $(action).blur();
    try {
      await api(action, {});
      actionError = '';
    } catch (error) {
      actionError = error.message;
    }
    renderStatus();
  });
}

poll();
nextFrame();
if (!virtual) sendPresence();
