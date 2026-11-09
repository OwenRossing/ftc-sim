// Robot and field sounds, made up on the spot with Web Audio from the state the sim already sends, so
// there are no audio files. They're off until you turn them on (the button or M).
//
// Sounds works out what's happening, a voice decides what that sounds like. There are two:
//   MusicalVoice: every pitch from one scale (A minor pentatonic, any notes of it sound fine together), soft
//     tones, one-off sounds on a steady beat so they come out even, motors step between notes.
//   RealisticVoice: motors whine up with their speed and hits are thumps. Rougher, closer to a real robot.

const KEY_ON = 'simSoundOn';
const KEY_VOLUME = 'simSoundVolume';
const KEY_MUSICAL = 'simSoundMusical';

function load(key, fallback) {
  try {
    const value = window.localStorage.getItem(key);
    return value === null ? fallback : value;
  } catch {
    return fallback;
  }
}

function save(key, value) {
  try {
    window.localStorage.setItem(key, value);
  } catch { /* fine, it just won't be remembered */ }
}

const clamp01 = (x) => Math.max(0, Math.min(1, x));

// ------------------------------------------------------------------ musical

const ROOT_MIDI = 45; // A2
const SCALE = [0, 3, 5, 7, 10];
const BEAT_SEC = 0.125; // sixteenth notes at 120 bpm

// a step of the scale as a frequency, 0 is A2, 5 is A3, 10 is A4. Negative goes lower
function note(step) {
  const octave = Math.floor(step / SCALE.length);
  const degree = step - octave * SCALE.length;
  return 440 * 2 ** ((ROOT_MIDI + 12 * octave + SCALE[degree] - 69) / 12);
}

class MusicalVoice {
  constructor(ctx, output) {
    this.ctx = ctx;
    this.output = output;
    this.shotStep = 0;
    this.lastShotAt = -10;
    this.beatsUsed = new Map();

    // a short echo in time with the beat, it's most of what makes the notes feel like they belong together
    this.echo = ctx.createDelay(1);
    this.echo.delayTime.value = BEAT_SEC * 3;
    const feedback = ctx.createGain();
    feedback.gain.value = 0.28;
    const echoLevel = ctx.createGain();
    echoLevel.gain.value = 0.3;
    this.echo.connect(feedback);
    feedback.connect(this.echo);
    this.echo.connect(echoLevel);
    echoLevel.connect(output);

    this.hums = { flywheel: this.hum(), drive: this.hum(), intake: this.hum(), turret: this.hum() };
  }

  // a steady soft tone, a sine with a quiet one an octave up for some body
  hum() {
    const ctx = this.ctx;
    const gain = ctx.createGain();
    gain.gain.value = 0;
    gain.connect(this.output);
    const low = ctx.createOscillator();
    low.type = 'sine';
    low.connect(gain);
    low.start();
    const high = ctx.createOscillator();
    high.type = 'triangle';
    const highLevel = ctx.createGain();
    highLevel.gain.value = 0.18;
    high.connect(highLevel);
    highLevel.connect(gain);
    high.start();
    return { gain, low, high, step: null };
  }

  setHum(name, level, step) {
    const hum = this.hums[name];
    const now = this.ctx.currentTime;
    hum.gain.gain.setTargetAtTime(level, now, 0.08);
    if (step !== hum.step) {
      hum.step = step;
      const hz = note(step);
      // a short glide between notes, not a slide through everything in between
      hum.low.frequency.setTargetAtTime(hz, now, 0.04);
      hum.high.frequency.setTargetAtTime(hz * 2, now, 0.04);
    }
  }

  // which step a speed lands on. It only moves once it's well into the next one, so a flywheel
  // dipping a little on each shot holds its note rather than warbling between two
  stepFor(name, fraction, steps) {
    const hum = this.hums[name];
    const exact = clamp01(fraction) * steps;
    const current = hum.held === undefined ? Math.round(exact) : hum.held;
    hum.held = Math.abs(exact - current) > 0.7 ? Math.round(exact) : current;
    return hum.held;
  }

  // all one fixed level once they're going, so nothing surges
  running(m) {
    this.setHum('flywheel', 0.07 * clamp01(m.fly / 800), this.stepFor('flywheel', m.fly / 4000, 10));
    this.setHum('drive', 0.06 * clamp01(m.drive / 60), -5 + this.stepFor('drive', m.drive / 450, 4));
    this.setHum('intake', 0.05 * clamp01(m.intake / 100), 3);
    this.setHum('turret', 0.025 * clamp01(m.turretRate / 60), 7);
  }

  quiet() {
    for (const name of Object.keys(this.hums)) this.hums[name].gain.gain.setTargetAtTime(0, this.ctx.currentTime, 0.08);
  }

  // the next beat that isn't already full, so one-off sounds come out even and never pile up in a flam
  nextBeat(limit) {
    const beat = Math.ceil((this.ctx.currentTime + 0.01) / BEAT_SEC);
    for (const key of this.beatsUsed.keys()) {
      if (key < beat) this.beatsUsed.delete(key);
    }
    for (let b = beat; b < beat + 4; b++) {
      const used = this.beatsUsed.get(b) || 0;
      if (used < limit) {
        this.beatsUsed.set(b, used + 1);
        return b * BEAT_SEC;
      }
    }
    return null; // too much going on, this one can sit out
  }

  // one soft note at a set time
  pluck(at, step, level, seconds, toEcho) {
    const ctx = this.ctx;
    const gain = ctx.createGain();
    gain.gain.setValueAtTime(0.0001, at);
    gain.gain.exponentialRampToValueAtTime(level, at + 0.012);
    gain.gain.exponentialRampToValueAtTime(0.0001, at + seconds);
    gain.connect(this.output);
    if (toEcho) gain.connect(this.echo);
    const osc = ctx.createOscillator();
    osc.type = 'triangle';
    osc.frequency.value = note(step);
    osc.connect(gain);
    osc.start(at);
    osc.stop(at + seconds + 0.02);
  }

  // a soft low note for anything with weight
  thud(at, step, level, seconds) {
    const ctx = this.ctx;
    const gain = ctx.createGain();
    gain.gain.setValueAtTime(0.0001, at);
    gain.gain.exponentialRampToValueAtTime(level, at + 0.008);
    gain.gain.exponentialRampToValueAtTime(0.0001, at + seconds);
    gain.connect(this.output);
    const osc = ctx.createOscillator();
    osc.type = 'sine';
    osc.frequency.value = note(step);
    osc.connect(gain);
    osc.start(at);
    osc.stop(at + seconds + 0.02);
  }

  // each shot in a burst goes one note up the scale, a new burst starts over
  shot() {
    const at = this.nextBeat(1);
    if (at === null) return;
    if (at - this.lastShotAt > 1.5) this.shotStep = 0;
    this.lastShotAt = at;
    this.thud(at, 0, 0.3, 0.16);
    this.pluck(at, 5 + (this.shotStep % 5), 0.16, 0.35, true);
    this.shotStep++;
  }

  ballIn() {
    const at = this.nextBeat(2);
    if (at !== null) this.pluck(at, 9, 0.1, 0.2, false);
  }

  bonk() {
    const at = this.nextBeat(2);
    if (at !== null) this.thud(at, -5, 0.3, 0.25);
  }

  // balls keep their own note, the same for the same ball, quiet and never more than one a beat
  bounce(ballIndex) {
    const at = this.nextBeat(1);
    if (at !== null) this.pluck(at, 10 + (ballIndex % 5), 0.045, 0.14, true);
  }

  // an A minor chord going up, all even
  tip() {
    const at = this.nextBeat(4);
    if (at === null) return;
    this.thud(at, -5, 0.35, 0.8);
    [5, 6, 8, 10].forEach((step, i) => this.pluck(at + i * BEAT_SEC, step, 0.16, 0.6, true));
  }
}

// ------------------------------------------------------------------ realistic

class RealisticVoice {
  constructor(ctx, output) {
    this.ctx = ctx;
    this.output = output;
    this.bounceTimes = [];

    // a second of white noise, every thump and rumble is this through a filter
    const length = ctx.sampleRate;
    this.noise = ctx.createBuffer(1, length, ctx.sampleRate);
    const data = this.noise.getChannelData(0);
    for (let i = 0; i < length; i++) data[i] = Math.random() * 2 - 1;

    this.hums = {
      flywheel: this.motorHum('sawtooth', 2, 1400),
      drive: this.motorHum('square', 2, 700),
      intake: this.motorHum('sawtooth', 1, 900),
      turret: this.motorHum('sawtooth', 1, 1600),
      servo: this.motorHum('sine', 1, 900), // quiet and low on purpose, a high whine got annoying fast
      rollers: this.noiseHum(500),
    };
  }

  motorHum(type, voices, cutoff) {
    const ctx = this.ctx;
    const gain = ctx.createGain();
    gain.gain.value = 0;
    const filter = ctx.createBiquadFilter();
    filter.type = 'lowpass';
    filter.frequency.value = cutoff;
    filter.connect(gain);
    gain.connect(this.output);
    const oscillators = [];
    for (let i = 0; i < voices; i++) {
      const osc = ctx.createOscillator();
      osc.type = type;
      osc.detune.value = i * 9; // two slightly off beat against each other, more like a real motor
      osc.connect(filter);
      osc.start();
      oscillators.push(osc);
    }
    return { gain, oscillators };
  }

  noiseHum(cutoff) {
    const ctx = this.ctx;
    const gain = ctx.createGain();
    gain.gain.value = 0;
    const filter = ctx.createBiquadFilter();
    filter.type = 'bandpass';
    filter.frequency.value = cutoff;
    const source = ctx.createBufferSource();
    source.buffer = this.noise;
    source.loop = true;
    source.connect(filter);
    filter.connect(gain);
    gain.connect(this.output);
    source.start();
    return { gain, oscillators: [] };
  }

  setHum(name, level, frequency) {
    const hum = this.hums[name];
    const now = this.ctx.currentTime;
    hum.gain.gain.setTargetAtTime(level, now, 0.04);
    for (const osc of hum.oscillators) osc.frequency.setTargetAtTime(Math.max(20, frequency), now, 0.03);
  }

  running(m) {
    this.setHum('flywheel', 0.11 * clamp01(m.fly / 4000) ** 1.3, 40 + m.fly / 60 * 5);
    this.setHum('drive', 0.05 * clamp01(m.drive / 400), 45 + m.drive * 0.55);
    this.setHum('rollers', 0.2 * clamp01(m.drive / 450), 0);
    this.setHum('intake', 0.07 * clamp01(m.intake / 312), 70 + m.intake * 0.5);
    this.setHum('turret', 0.05 * clamp01(m.turretRate / 150), 160 + m.turretRate * 1.2);
    this.setHum('servo', 0.012 * clamp01(m.servoMove * 2), 300 + m.servoMove * 60);
  }

  quiet() {
    for (const name of Object.keys(this.hums)) this.hums[name].gain.gain.setTargetAtTime(0, this.ctx.currentTime, 0.04);
  }

  // a low hit: a pitch that drops fast, plus a burst of noise
  thump(level, fromHz, toHz, seconds, noiseCutoff) {
    const ctx = this.ctx;
    const now = ctx.currentTime;
    const gain = ctx.createGain();
    gain.gain.setValueAtTime(level, now);
    gain.gain.exponentialRampToValueAtTime(0.001, now + seconds);
    gain.connect(this.output);
    const osc = ctx.createOscillator();
    osc.frequency.setValueAtTime(fromHz, now);
    osc.frequency.exponentialRampToValueAtTime(toHz, now + seconds);
    osc.connect(gain);
    osc.start(now);
    osc.stop(now + seconds);

    const hiss = ctx.createGain();
    hiss.gain.setValueAtTime(level * 0.6, now);
    hiss.gain.exponentialRampToValueAtTime(0.001, now + seconds * 0.5);
    const filter = ctx.createBiquadFilter();
    filter.type = 'lowpass';
    filter.frequency.value = noiseCutoff;
    const source = ctx.createBufferSource();
    source.buffer = this.noise;
    source.start(now, Math.random() * 0.8, seconds);
    source.connect(filter);
    filter.connect(hiss);
    hiss.connect(this.output);
  }

  shot() { this.thump(0.5, 170, 55, 0.16, 2500); }
  ballIn() { this.thump(0.22, 260, 120, 0.08, 900); }
  bonk() { this.thump(0.6, 95, 40, 0.2, 500); }

  bounce(ballIndex, fallSpeed) {
    const nowMs = performance.now();
    this.bounceTimes = this.bounceTimes.filter((t) => nowMs - t < 1000);
    if (this.bounceTimes.length >= 14) return;
    this.bounceTimes.push(nowMs);
    this.thump(0.05 + 0.25 * clamp01(fallSpeed / 200), 420, 200, 0.05, 1600);
  }

  tip() { this.thump(0.9, 75, 32, 0.6, 350); }
}

// ------------------------------------------------------------------ what's happening

export class Sounds {
  constructor() {
    this.on = load(KEY_ON, '0') === '1';
    this.musical = load(KEY_MUSICAL, '1') === '1';
    this.volume = Number(load(KEY_VOLUME, '0.7'));
    this.ctx = null;
    this.voice = null;
    this.last = null;
    this.lastAt = 0;
    this.pieces = null;

    const start = () => this.start();
    window.addEventListener('pointerdown', start);
    window.addEventListener('keydown', start);
    // no updates means the sim went away, don't leave a motor droning
    setInterval(() => {
      if (this.voice && performance.now() - this.lastAt > 500) this.voice.quiet();
    }, 250);
  }

  start() {
    if (!this.on) return;
    if (!this.ctx) this.build();
    if (this.ctx.state === 'suspended') this.ctx.resume();
  }

  build() {
    const ctx = new (window.AudioContext || window.webkitAudioContext)();
    this.ctx = ctx;
    this.master = ctx.createGain();
    this.master.gain.value = this.on ? this.volume : 0;
    this.analyser = ctx.createAnalyser();
    this.analyser.fftSize = 1024;
    // keeps a pile of sounds at once from clipping
    const limiter = ctx.createDynamicsCompressor();
    this.master.connect(limiter);
    limiter.connect(this.analyser);
    this.analyser.connect(ctx.destination);
    this.useVoice();
  }

  useVoice() {
    if (this.voice) this.voice.quiet();
    const Voice = this.musical ? MusicalVoice : RealisticVoice;
    this.voices = this.voices || new Map();
    if (!this.voices.has(Voice)) this.voices.set(Voice, new Voice(this.ctx, this.master));
    this.voice = this.voices.get(Voice);
  }

  // the state just came in, work out what changed since the last one
  update(state) {
    const previous = this.last;
    const nowMs = performance.now();
    const dt = (nowMs - this.lastAt) / 1000;
    this.last = state;
    this.lastAt = nowMs;
    if (!this.voice || !this.on || !previous || dt <= 0 || dt > 0.5) return;

    const motors = state.motors || {};
    let drive = 0;
    for (const name of ['frontLeft', 'frontRight', 'backLeft', 'backRight']) {
      if (motors[name]) drive += Math.abs(motors[name].rpm) / 4;
    }
    let servoMove = 0;
    for (const name of Object.keys(state.servos || {})) {
      const before = previous.servos && previous.servos[name];
      if (before) servoMove = Math.max(servoMove, Math.abs(state.servos[name].pos - before.pos) / dt);
    }
    this.voice.running({
      fly: Math.abs(state.flywheelRpm),
      drive,
      intake: Math.abs(state.intakeRpm),
      turretRate: Math.abs(state.turretDeg - previous.turretDeg) / dt,
      servoMove,
    });

    if (state.shots > previous.shots) this.voice.shot();
    else if (state.balls > previous.balls) this.voice.ballIn();

    const hives = state.hives, before = previous.hives;
    if (hives && before && (hives.redTips > before.redTips || hives.blueTips > before.blueTips)) this.voice.tip();

    // the robot stopping dead is it hitting something, braking takes longer than this
    const speed = Math.hypot(state.robot.x - previous.robot.x, state.robot.y - previous.robot.y) / dt;
    if (this.lastSpeed !== undefined && this.lastSpeed - speed > 22 && dt < 0.1) this.voice.bonk();
    this.lastSpeed = speed;

    this.ballBounces(state.pieces || [], dt);
  }

  // a ball coming down fast then not anymore hit something
  ballBounces(pieces, dt) {
    const old = this.pieces;
    this.pieces = pieces;
    if (!old || old.length !== pieces.length) {
      this.fallSpeeds = null;
      return;
    }
    const falls = new Array(pieces.length / 4);
    for (let i = 0; i < falls.length; i++) {
      const fall = (old[i * 4 + 3] - pieces[i * 4 + 3]) / dt; // inches/sec downward
      falls[i] = fall;
      const before = this.fallSpeeds ? this.fallSpeeds[i] : 0;
      if (before > 40 && fall < before * 0.3) this.voice.bounce(i, before);
    }
    this.fallSpeeds = falls;
  }

  setOn(on) {
    this.on = on;
    save(KEY_ON, on ? '1' : '0');
    if (on) this.start();
    if (this.ctx) {
      if (!on) this.voice.quiet();
      this.master.gain.setTargetAtTime(on ? this.volume : 0, this.ctx.currentTime, 0.02);
    }
  }

  setMusical(musical) {
    this.musical = musical;
    save(KEY_MUSICAL, musical ? '1' : '0');
    if (this.ctx) this.useVoice();
  }

  setVolume(volume) {
    this.volume = volume;
    save(KEY_VOLUME, String(volume));
    if (this.ctx && this.on) this.master.gain.setTargetAtTime(volume, this.ctx.currentTime, 0.02);
  }

  // how loud it is right now, 0 to 1. For checking it works
  level() {
    if (!this.analyser) return 0;
    const samples = new Float32Array(this.analyser.fftSize);
    this.analyser.getFloatTimeDomainData(samples);
    let sum = 0;
    for (const s of samples) sum += s * s;
    return Math.sqrt(sum / samples.length);
  }
}
