import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { Sounds } from './sounds.js';
import { setUpSettings } from './settings.js';

const $ = (id) => document.getElementById(id);
const DEG = Math.PI / 180;
let state = null;

// ------------------------------------------------------------------ scene

// low detail: lighter models, no shadows, no antialiasing. Saved per device, ?low forces it
function readDetail() {
  if (new URLSearchParams(window.location.search).has('low')) return 'low';
  try {
    return window.localStorage.getItem('simDetail') === 'low' ? 'low' : 'high';
  } catch {
    return 'high';
  }
}
const LOW_DETAIL = readDetail() === 'low';

const renderer = new THREE.WebGLRenderer({ antialias: !LOW_DETAIL });
renderer.setPixelRatio(LOW_DETAIL ? 1 : Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = !LOW_DETAIL;
$('view').appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x14171c);
const camera = new THREE.PerspectiveCamera(50, 1, 0.5, 3000);
camera.position.set(-70, 95, 95);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 6, 0);
controls.enableDamping = true;

scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 0.9));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(60, 160, 40);
sun.castShadow = true;
Object.assign(sun.shadow.camera, { left: -90, right: 90, top: 90, bottom: -90, near: 1, far: 400 });
sun.shadow.mapSize.set(2048, 2048);
scene.add(sun);

// field is Pedro's: 0-144 in, drawn centered. field (x, y) -> three (x - 72, 0, 72 - y)
const fieldToThree = (x, y) => new THREE.Vector3(x - 72, 0, 72 - y);

const field = new THREE.Mesh(new THREE.PlaneGeometry(144, 144),
  new THREE.MeshStandardMaterial({ color: 0x585d64, roughness: 0.95 }));
field.rotation.x = -Math.PI / 2;
field.receiveShadow = true;
scene.add(field);
const grid = new THREE.GridHelper(144, 6, 0x9aa3ad, 0x9aa3ad);
grid.position.y = 0.05;
scene.add(grid);

const wallMat = new THREE.MeshStandardMaterial({ color: 0xb8c4d0, transparent: true, opacity: 0.25 });
// the CAD has real walls but no floor tiles (dropped, too heavy), so only our walls get hidden
const placeholderField = [];
for (const [w, d, x, z] of [[146, 1, 0, -72.5], [146, 1, 0, 72.5], [1, 146, -72.5, 0], [1, 146, 72.5, 0]]) {
  const wall = new THREE.Mesh(new THREE.BoxGeometry(w, 12, d), wallMat);
  wall.position.set(x, 6, z);
  scene.add(wall);
  placeholderField.push(wall);
}

// the real field CAD, if there is one (see field.json). Its origin is the field center, like ours
const fieldNodes = {};
async function loadField() {
  let cfg;
  try {
    const res = await fetch('field.json', { cache: 'no-store' });
    if (!res.ok) return;
    cfg = await res.json();
  } catch {
    return;
  }
  new GLTFLoader().load(`models/${LOW_DETAIL ? 'low/' : ''}${cfg.file}`, (gltf) => {
    const model = gltf.scene;
    model.scale.setScalar(cfg.scale ?? 1);
    model.rotation.y = (cfg.rotationYDeg ?? 0) * DEG;
    model.traverse((o) => {
      if (o.isMesh) {
        o.receiveShadow = true;
        o.castShadow = !o.material.transparent; // clear polycarb shouldn't throw a solid shadow
      }
      if (o.name) fieldNodes[o.name] = o;
    });
    scene.add(model);
    placeholderField.forEach((m) => { m.visible = false; });
  }, undefined, (err) => console.warn(`couldn't load models/${cfg.file}`, err));
}
// sim.html?nofield hides the field CAD, handy for looking at just the robot
if (!new URLSearchParams(window.location.search).has('nofield')) loadField();

// ?colliders draws the shapes the physics actually uses, green wireframes over the CAD
async function drawColliders() {
  const data = await (await fetch('field_colliders.json')).json();
  const mat = new THREE.LineBasicMaterial({ color: 0x33ff88 });
  const H = 1.8288;
  const toThree = (p) => new THREE.Vector3(p[0], p[2], -p[1]); // field axes -> three axes, directions only
  for (const b of data.static) {
    const box = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(b.h[0] * 2, b.h[1] * 2, b.h[2] * 2)), mat);
    const [ax, ay, az] = b.axes.map(toThree);
    if (ax.clone().cross(ay).dot(az) < 0) az.negate();
    box.matrixAutoUpdate = false;
    box.matrix.makeBasis(ax, ay, az).setPosition((b.c[0] - H), b.c[2], (H - b.c[1]));
    const holder = new THREE.Group();
    holder.scale.setScalar(39.37);
    holder.add(box);
    scene.add(holder);
  }
}
if (new URLSearchParams(window.location.search).has('colliders')) drawColliders();
// Pedro axes at the field origin: red = +x, green = +y
const origin = fieldToThree(0, 0).setY(0.5);
scene.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), origin, 18, 0xff5555, 4, 3));
scene.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), origin, 18, 0x55ff77, 4, 3));

// ------------------------------------------------------------------ robot

const robot = new THREE.Group();
scene.add(robot);
const parts = {};
let config = null;
const balls = [];

function material(cfg) {
  const opacity = cfg.opacity ?? 1;
  return new THREE.MeshStandardMaterial({
    color: cfg.color || '#999999', roughness: 0.6, metalness: 0.1,
    transparent: opacity < 1, opacity,
  });
}

function placeholder(cfg) {
  const c = cfg.center || [0, 0, 0];
  let mesh;
  if (cfg.cylinder) {
    const { radius, width } = cfg.cylinder;
    mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, width, 28), material(cfg));
    // cylinders line up with the part's axis (wheels default to sideways)
    const axis = new THREE.Vector3(...(cfg.axis || [0, 0, 1])).normalize();
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis);
  } else {
    mesh = new THREE.Mesh(new THREE.BoxGeometry(...(cfg.box || [1, 1, 1])), material(cfg));
  }
  mesh.position.set(...c);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

function loadGlb(cfg, holder) {
  const file = LOW_DETAIL ? cfg.file.replace('robot/', 'robot/low/') : cfg.file;
  new GLTFLoader().load(`models/${file}`, (gltf) => {
    const model = gltf.scene;
    model.scale.setScalar(cfg.scale ?? 1);
    if (cfg.rotation) model.rotation.set(cfg.rotation[0] * DEG, cfg.rotation[1] * DEG, cfg.rotation[2] * DEG);
    model.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
    holder.clear();
    holder.add(model);
  }, undefined, (err) => {
    console.warn(`couldn't load models/${cfg.file}, using a placeholder`, err);
    holder.add(placeholder(cfg));
  });
}

async function buildRobot() {
  config = await (await fetch('robot.json', { cache: 'no-store' })).json();
  for (const cfg of config.parts) {
    const pivot = cfg.pivot || [0, 0, 0];
    const parent = cfg.parent ? parts[cfg.parent] : null;
    const parentPivot = parent ? (parent.cfg.pivot || [0, 0, 0]) : [0, 0, 0];
    // group sits on the hinge, holder shifts back so robot-frame coordinates (and CAD) line up
    const group = new THREE.Group();
    group.position.set(pivot[0] - parentPivot[0], pivot[1] - parentPivot[1], pivot[2] - parentPivot[2]);
    (parent ? parent.group : robot).add(group);
    const holder = new THREE.Group();
    holder.position.set(-pivot[0], -pivot[1], -pivot[2]);
    group.add(holder);
    if (cfg.file) loadGlb(cfg, holder);
    else holder.add(placeholder(cfg));
    parts[cfg.name] = { cfg, group, axis: new THREE.Vector3(...(cfg.axis || [0, 1, 0])).normalize(), spin: 0 };
  }

  const stack = config.ballStack;
  for (let i = 0; i < 5; i++) {
    const ball = new THREE.Mesh(new THREE.SphereGeometry(stack.radius, 20, 14), material({ color: stack.color }));
    const step = Array.isArray(stack.spacing) ? stack.spacing : [0, stack.spacing, 0];
    ball.position.set(stack.bottom[0] + i * step[0], stack.bottom[1] + i * step[1], stack.bottom[2] + i * step[2]);
    ball.castShadow = true;
    robot.add(ball);
    balls.push(ball);
  }

}

// ------------------------------------------------------------------ game pieces

// instanced so 40 balls cost about the same as one. Spheres until the real models load
const BALL_MATERIALS = [0xf2c81e, 0xd63a3a, 0x3a64d6];
const BALL_RADIUS = [1.4, 1.81, 1.81];
const BALL_FILES = ['pollen', 'redNectar', 'blueNectar'];
const ballMeshes = BALL_MATERIALS.map((color, i) => {
  const mesh = new THREE.InstancedMesh(new THREE.SphereGeometry(BALL_RADIUS[i], 20, 14),
    new THREE.MeshStandardMaterial({ color, roughness: 0.6 }), 64);
  mesh.castShadow = true;
  mesh.count = 0;
  mesh.frustumCulled = false;
  scene.add(mesh);
  return mesh;
});
if (!LOW_DETAIL) {
  BALL_FILES.forEach((name, i) => new GLTFLoader().load(`models/balls/${name}.glb`, (gltf) => {
    gltf.scene.traverse((o) => {
      if (!o.isMesh) return;
      // the shape only: our material colors it, and without normals the lighting makes it black
      const geometry = o.geometry;
      geometry.deleteAttribute('color');
      if (!geometry.attributes.normal) geometry.computeVertexNormals();
      ballMeshes[i].geometry = geometry;
    });
  }));
}
const ballMatrix = new THREE.Matrix4();

function updateBalls() {
  const p = state.pieces || [];
  const counts = [0, 0, 0];
  for (let i = 0; i + 3 < p.length; i += 4) {
    const type = p[i];
    ballMatrix.makeTranslation(p[i + 1] - 72, p[i + 3], 72 - p[i + 2]);
    ballMeshes[type].setMatrixAt(counts[type]++, ballMatrix);
  }
  ballMeshes.forEach((mesh, i) => {
    mesh.count = counts[i];
    mesh.instanceMatrix.needsUpdate = true;
  });
}

// tilts a Hive around its axle. The field model is in meters, the axle is 43.95 in up (manual 9.6.1)
const HIVE_AXLE_M = 43.95 * 0.0254;
const hiveMatrix = new THREE.Matrix4();
const hiveTurn = new THREE.Matrix4();
function tiltHive(name, angle) {
  const node = fieldNodes[name];
  if (!node) return;
  node.matrixAutoUpdate = false;
  hiveMatrix.makeTranslation(0, HIVE_AXLE_M, 0).multiply(hiveTurn.makeRotationX(angle));
  node.matrix.copy(hiveMatrix).multiply(hiveTurn.makeTranslation(0, -HIVE_AXLE_M, 0));
  node.matrixWorldNeedsUpdate = true;
}

// ------------------------------------------------------------------ apply state

const shownPose = { x: 72, y: 72, h: 0, ready: false };

function angleLerp(a, b, t) {
  let d = b - a;
  while (d > Math.PI) d -= 2 * Math.PI;
  while (d < -Math.PI) d += 2 * Math.PI;
  return a + d * t;
}

function updateRobot(dt) {
  if (!state || !config) return;
  const r = state.robot;
  const t = shownPose.ready ? 1 - Math.exp(-dt * 45) : 1;
  shownPose.x += (r.x - shownPose.x) * t;
  shownPose.y += (r.y - shownPose.y) * t;
  shownPose.h = angleLerp(shownPose.h, r.h, t);
  shownPose.ready = true;

  const before = robot.position.clone();
  robot.position.copy(fieldToThree(shownPose.x, shownPose.y));
  robot.rotation.y = shownPose.h;
  if ($('follow').checked && view === 'free') {
    const delta = robot.position.clone().sub(before);
    camera.position.add(delta);
    controls.target.add(delta);
  }

  for (const part of Object.values(parts)) {
    const d = part.cfg.driver;
    if (!d) continue;
    let angle = null;
    if (d.type === 'servo') {
      const s = state.servos[d.servo];
      if (s) angle = (d.angleAt0 + (d.angleAt1 - d.angleAt0) * s.pos) * DEG;
    } else if (d.type === 'spin') {
      const m = state.motors[d.motor];
      // slowed way down so you can see it turn instead of a strobe
      if (m) part.spin += (m.rpm * (d.scale ?? 1)) / 60 * 2 * Math.PI * dt * 0.08;
      angle = part.spin;
    } else if (d.type === 'turret') {
      angle = ((d.angleAtForward ?? 0) + (state.turretDeg - (d.forwardDeg ?? 0)) * (d.clockwisePositive ? -1 : 1)) * DEG;
    }
    if (angle !== null) part.group.quaternion.setFromAxisAngle(part.axis, angle);
  }

  balls.forEach((b, i) => { b.visible = i < state.balls; });

  updateBalls();
  if (state.hives) {
    tiltHive('red_hive', state.hives.redAngle);
    tiltHive('blue_hive', state.hives.blueAngle);
  }
}

// ------------------------------------------------------------------ camera views

// inches. From the driver's side but further back and higher than a real driver's eyes, so the whole field fits
const DRIVER_BACK = 84;
const DRIVER_EYE = 87;
const FIELD_CENTER = new THREE.Vector3(0, 8, 0);
const FIXED_VIEWS = {
  driverRed: fieldToThree(-DRIVER_BACK, 72).setY(DRIVER_EYE),
  driverBlue: fieldToThree(144 + DRIVER_BACK, 72).setY(DRIVER_EYE),
  top: new THREE.Vector3(0, 235, 0.1),
  audience: fieldToThree(72, -42).setY(58),
};
const VIEW_ORDER = ['free', 'driverRed', 'driverBlue', 'top', 'audience', 'chase', 'limelight'];
const lerpTarget = new THREE.Vector3();
const scratch = new THREE.Vector3();
// the Limelight rides on the turret, looking the way the shooter shoots, tilted up (TUNE with the robot)
const LIMELIGHT_POINT = [-5.5, 12.4, -5.0];
const LIMELIGHT_DIR = [-0.6, 0, -0.8];
const LIMELIGHT_TILT_DEG = 25;
let limelightMarker = null;

let view = 'driverRed';
try {
  const saved = window.localStorage.getItem('simView');
  if (VIEW_ORDER.includes(saved)) view = saved;
} catch { /* no storage, use the default */ }

function setView(next) {
  view = next;
  $('cameraView').value = next;
  $('cameraView').blur(); // so WASD goes back to driving
  $('follow').disabled = next !== 'free';
  controls.enabled = next !== 'chase' && next !== 'limelight';
  if (FIXED_VIEWS[next]) {
    camera.position.copy(FIXED_VIEWS[next]);
    controls.target.copy(FIELD_CENTER);
  }
  try { window.localStorage.setItem('simView', next); } catch { /* fine */ }
}
$('cameraView').addEventListener('change', () => setView($('cameraView').value));
window.addEventListener('keydown', (e) => {
  const i = Number(e.key) - 1;
  if (VIEW_ORDER[i]) setView(VIEW_ORDER[i]);
});

function updateCamera(dt) {
  if (view === 'driverRed' || view === 'driverBlue') {
    // your head follows the robot a bit, your feet stay put
    lerpTarget.copy(FIELD_CENTER).lerp(robot.position, 0.6);
    controls.target.lerp(lerpTarget, 1 - Math.exp(-dt * 4));
  } else if (view === 'chase') {
    camera.position.lerp(robot.localToWorld(scratch.set(-36, 24, 0)), 1 - Math.exp(-dt * 6));
    controls.target.copy(robot.localToWorld(scratch.set(14, 9, 0)));
  } else if (view === 'limelight') {
    if (!limelightMarker && parts.turret) {
      const tp = parts.turret.cfg.pivot;
      limelightMarker = new THREE.Object3D();
      limelightMarker.position.set(LIMELIGHT_POINT[0] - tp[0], LIMELIGHT_POINT[1] - tp[1], LIMELIGHT_POINT[2] - tp[2]);
      parts.turret.group.add(limelightMarker);
    }
    if (!limelightMarker) return;
    limelightMarker.getWorldPosition(camera.position);
    const tilt = LIMELIGHT_TILT_DEG * DEG;
    scratch.set(LIMELIGHT_DIR[0] * Math.cos(tilt), Math.sin(tilt), LIMELIGHT_DIR[2] * Math.cos(tilt))
      .transformDirection(limelightMarker.matrixWorld);
    controls.target.copy(camera.position).addScaledVector(scratch, 30);
  }
}

// ------------------------------------------------------------------ driver station UI

let opmodesShown = '';
let statsShown = null;

function renderUi() {
  if (!state) return;
  const names = JSON.stringify(state.opmodes);
  if (names !== opmodesShown) {
    opmodesShown = names;
    const sel = $('opmode');
    const keep = sel.value;
    sel.innerHTML = '';
    for (const group of [['TeleOp', false], ['Autonomous', true]]) {
      const og = document.createElement('optgroup');
      og.label = group[0];
      for (const o of state.opmodes.filter((o) => o.auto === group[1])) {
        const opt = document.createElement('option');
        opt.textContent = o.name;
        og.appendChild(opt);
      }
      sel.appendChild(og);
    }
    if (keep) sel.value = keep;
  }
  const phase = state.phase;
  $('phase').textContent = state.opmode ? `${phase} (${state.opmode})` : phase;
  $('phase').className = `phase-${phase}`;
  $('start').disabled = phase !== 'INIT';
  $('stop').disabled = !(phase === 'INIT' || phase === 'RUNNING');
  // the alliance menu is only for robots whose code cares (state.alliance is null otherwise)
  $('allianceBox').hidden = state.alliance === null || state.alliance === undefined;
  if (document.activeElement !== $('alliance')) $('alliance').value = state.alliance || '';
  $('robotName').textContent = state.robotName || '';
  $('battery').textContent = `${state.battery.toFixed(2)} V`;
  const stats = (state.stats || []).join('\n');
  if (stats !== statsShown) {
    statsShown = stats;
    $('stats').innerHTML = '';
    for (const line of state.stats || []) {
      const [label, value] = line.split('\t');
      const a = document.createElement('span');
      a.textContent = label;
      const b = document.createElement('span');
      b.textContent = value;
      $('stats').append(a, b);
    }
  }
  $('balls').textContent = state.balls;
  $('shots').textContent = state.shots;
  if (state.hives) {
    const h = state.hives; // 20 per tip, 2 per ball still in a cell (manual table 10-2)
    $('redScore').textContent = `${h.redTips} tips, ${h.redInCell} in cells, ${h.redTips * 20 + h.redInCell * 2} pts`;
    $('blueScore').textContent = `${h.blueTips} tips, ${h.blueInCell} in cells, ${h.blueTips * 20 + h.blueInCell * 2} pts`;
  }
  $('pose').textContent = `${state.robot.x.toFixed(1)}, ${state.robot.y.toFixed(1)}, ${(state.robot.h / DEG).toFixed(0)}°`;
  $('telemetry').textContent = state.telemetry.length ? state.telemetry.join('\n') : '-';
  $('warnings').innerHTML = '';
  for (const w of state.warnings) {
    const div = document.createElement('div');
    div.textContent = `⚠ ${w}`;
    $('warnings').appendChild(div);
  }
  $('error').textContent = state.error || '';
}

async function cmd(action) {
  const q = action === 'init' ? `&opmode=${encodeURIComponent($('opmode').value)}` : '';
  const res = await fetch(`cmd?action=${action}${q}`, { method: 'POST' });
  if (!res.ok) $('error').textContent = await res.text();
}
for (const a of ['init', 'start', 'stop', 'reset']) $(a).addEventListener('click', () => cmd(a));

// the alliance: same thing as B or X during INIT, and it can change any time here. Remembered in this browser
const pickAlliance = (side) => fetch(`cmd?action=alliance&side=${side}`, { method: 'POST' }).catch(() => {});
// starts on red unless this browser picked something else, "not picked" counts as picking
let savedAlliance = null;
try { savedAlliance = window.localStorage.getItem('simAlliance'); } catch { /* fine */ }
if (savedAlliance === null) savedAlliance = 'RED';
$('alliance').value = savedAlliance;
pickAlliance(savedAlliance);
$('alliance').addEventListener('change', () => {
  pickAlliance($('alliance').value);
  try { window.localStorage.setItem('simAlliance', $('alliance').value); } catch { /* fine */ }
  $('alliance').blur();
});

setUpSettings($);

const sounds = new Sounds();
const showSound = () => { $('soundOn').textContent = sounds.on ? 'Sound on' : 'Sound off'; };
$('soundVolume').value = sounds.volume;
$('soundMusical').checked = sounds.musical;
$('soundMusical').addEventListener('change', () => { sounds.setMusical($('soundMusical').checked); $('soundMusical').blur(); });
showSound();
$('soundOn').addEventListener('click', () => { sounds.setOn(!sounds.on); showSound(); $('soundOn').blur(); });
$('soundVolume').addEventListener('input', () => sounds.setVolume(Number($('soundVolume').value)));
window.addEventListener('keydown', (e) => {
  if (e.code === 'KeyM' && !e.repeat) { sounds.setOn(!sounds.on); showSound(); }
});

const events = new EventSource('events');
events.onmessage = (e) => { state = JSON.parse(e.data); renderUi(); sounds.update(state); };
events.onerror = () => { $('phase').textContent = 'sim not running'; $('phase').className = 'phase-STOPPED'; };

// ------------------------------------------------------------------ controller + keyboard

const keys = new Set();
// keys go to the robot, unless a menu or a settings slider has them
const typing = () => document.activeElement && ['SELECT', 'INPUT', 'TEXTAREA'].includes(document.activeElement.tagName);
window.addEventListener('keydown', (e) => {
  if (typing()) return;
  keys.add(e.code);
  if (['Space', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.code)) e.preventDefault();
});
window.addEventListener('keyup', (e) => keys.delete(e.code));
window.addEventListener('blur', () => keys.clear());

const dead = (v) => (Math.abs(v) < 0.06 ? 0 : v);

function blankInput() {
  return {
    lx: 0, ly: 0, rx: 0, ry: 0, lt: 0, rt: 0, a: false, b: false, x: false, y: false,
    lb: false, rb: false, up: false, down: false, left: false, right: false, start: false, back: false,
    ls: false, rs: false,
  };
}

function readInput() {
  const g = blankInput();
  const pad = [...(navigator.getGamepads ? navigator.getGamepads() : [])].find((p) => p && p.connected);
  if (pad) {
    const btn = (i) => !!pad.buttons[i] && pad.buttons[i].pressed;
    const val = (i) => (pad.buttons[i] ? pad.buttons[i].value : 0);
    Object.assign(g, {
      lx: dead(pad.axes[0] || 0), ly: dead(pad.axes[1] || 0), rx: dead(pad.axes[2] || 0), ry: dead(pad.axes[3] || 0),
      a: btn(0), b: btn(1), x: btn(2), y: btn(3), lb: btn(4), rb: btn(5), lt: val(6), rt: val(7),
      back: btn(8), start: btn(9), ls: btn(10), rs: btn(11),
      up: btn(12), down: btn(13), left: btn(14), right: btn(15),
    });
    $('pad').textContent = pad.id.slice(0, 32);
  } else {
    $('pad').textContent = 'keyboard (press a pad button to connect)';
  }
  // keyboard adds on top
  const k = (code) => keys.has(code);
  if (k('KeyW')) g.ly = -1;
  if (k('KeyS')) g.ly = 1;
  if (k('KeyA')) g.lx = -1;
  if (k('KeyD')) g.lx = 1;
  if (k('KeyQ')) g.rx = -1;
  if (k('KeyE')) g.rx = 1;
  if (k('ShiftLeft') || k('ShiftRight')) g.lt = 1;
  if (k('Space')) g.rt = 1;
  g.b ||= k('KeyF');
  g.x ||= k('KeyX');
  g.rb ||= k('KeyR');
  g.a ||= k('KeyG');
  g.up ||= k('ArrowUp');
  g.down ||= k('ArrowDown');
  g.left ||= k('ArrowLeft');
  g.right ||= k('ArrowRight');
  g.start ||= k('Enter');
  return g;
}

// send on every change, and keep re-sending while something is held. An idle page (another
// laptop just watching) never sends, so it can't fight whoever is driving
const NEUTRAL = JSON.stringify(blankInput());
let lastSent = NEUTRAL;
let lastSentAt = 0;
setInterval(() => {
  const body = JSON.stringify(readInput());
  const now = performance.now();
  if (body === lastSent && (body === NEUTRAL || now - lastSentAt < 250)) return;
  lastSent = body;
  lastSentAt = now;
  fetch('gamepad', { method: 'POST', body }).catch(() => {});
}, 20);

// ------------------------------------------------------------------ loop

function resize() {
  renderer.setSize(window.innerWidth, window.innerHeight);
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();

$('detail').value = LOW_DETAIL ? 'low' : 'high';
$('detail').addEventListener('change', () => {
  try { window.localStorage.setItem('simDetail', $('detail').value); } catch { /* ?low still works */ }
  window.location.href = window.location.pathname; // models and renderer are set up at load. The robot keeps running
});

window.simDebug = { sounds, THREE, scene, camera, controls, robot, parts, renderer, ballMeshes, fieldNodes, updateBalls, tiltHive }; // for poking at it from the browser console

const clock = new THREE.Clock();
let frames = 0;
let fpsTime = 0;
function frame() {
  const dt = Math.min(clock.getDelta(), 0.1);
  frames++;
  fpsTime += dt;
  if (fpsTime >= 0.5) {
    const fps = Math.round(frames / fpsTime);
    $('fps').textContent = fps < 25 && !LOW_DETAIL ? `${fps} fps, try Low detail` : `${fps} fps`;
    frames = 0;
    fpsTime = 0;
  }
  updateRobot(dt);
  updateCamera(dt);
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}

setView(view);
buildRobot().then(frame).catch((e) => { $('error').textContent = `robot.json: ${e}`; frame(); });
