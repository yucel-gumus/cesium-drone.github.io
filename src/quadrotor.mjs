import {
  clamp,
  dot,
  norm,
  add,
  sub,
  mul,
  unit,
  cross,
  wrap,
  qmul,
  conjugate,
  rotate,
  attitudeFromForce,
  slerp,
} from "./flight-math.mjs";
import { referenceAt } from "./trajectory.mjs";

// Generic X quad, explicit engineering assumptions; not identified flight data.
export const AIRFRAME = Object.freeze({
  mass: 2,
  inertia: [0.035, 0.035, 0.06],
  armLength: 0.25,
  gravity: 9.80665,
  airDensity: 1.09,
  thrustCoefficient: 1.8e-5,
  reactionRatio: 0.018,
  maxRotorSpeed: 850,
  motorTimeConstant: 0.055,
  dragArea: 0.08,
  linearDrag: 0.1,
  angularDrag: [0.003, 0.003, 0.004],
  maxTilt: (25 * Math.PI) / 180,
  maxYawRate: (40 * Math.PI) / 180,
  maxYawAcceleration: (60 * Math.PI) / 180,
  physicsHz: 240,
  controlHz: 120,
  recordHz: 30,
});
export const SPIN = [1, -1, 1, -1];
export const STRIDE = 36;
export const PHASE_NAMES = [
  "arming",
  "takeoff",
  "survey",
  "return",
  "settle",
  "landing",
  "touchdown",
  "disarm",
  "complete",
];
export function validateAirframe(config) {
  for (const key of [
    "mass",
    "armLength",
    "gravity",
    "airDensity",
    "thrustCoefficient",
    "reactionRatio",
    "maxRotorSpeed",
    "motorTimeConstant",
    "physicsHz",
    "controlHz",
    "recordHz",
  ])
    if (!Number.isFinite(config[key]) || config[key] <= 0)
      throw new Error(`Geçersiz fizik parametresi: ${key}`);
  if (
    config.inertia?.length !== 3 ||
    config.inertia.some((x) => !Number.isFinite(x) || x <= 0)
  )
    throw new Error("Atalet tensörü pozitif olmalı.");
  if (config.physicsHz % config.controlHz || config.physicsHz % config.recordHz)
    throw new Error(
      "Fizik frekansı kontrol ve kayıt frekanslarının katı olmalı.",
    );
}
const kf = (c) => (c.thrustCoefficient * c.airDensity) / 1.225;
export function windAt(t, wind = 0) {
  return [
    wind / Math.hypot(1, 0.4) + 0.25 * wind * Math.sin(0.31 * t),
    (0.4 * wind) / Math.hypot(1, 0.4) + 0.15 * wind * Math.sin(0.53 * t + 0.7),
    0.04 * wind * Math.sin(0.7 * t),
  ];
}
export function wrench(motors, c = AIRFRAME) {
  const a = c.armLength / Math.sqrt(2),
    xs = [a, -a, -a, a],
    ys = [-a, -a, a, a];
  const f = motors.map((w) => kf(c) * w * w);
  return {
    thrust: f.reduce((a, b) => a + b, 0),
    torque: [dot(ys, f), -dot(xs, f), c.reactionRatio * dot(SPIN, f)],
  };
}
export function allocate(thrust, torque, c = AIRFRAME) {
  const a = c.armLength / Math.sqrt(2),
    xs = [a, -a, -a, a],
    ys = [-a, -a, a, a],
    max = kf(c) * c.maxRotorSpeed ** 2;
  const forces = xs.map(
    (x, i) =>
      thrust / 4 +
      (ys[i] * torque[0]) / (4 * a * a) -
      (x * torque[1]) / (4 * a * a) +
      (SPIN[i] * torque[2]) / (4 * c.reactionRatio),
  );
  return {
    commands: forces.map((f) => Math.sqrt(clamp(f, 0, max) / kf(c))),
    saturated: forces.some((f) => f < 0 || f > max),
  };
}
export function initialState(yaw = 0) {
  const s = new Float64Array(21);
  s[6] = Math.cos(yaw / 2);
  s[9] = Math.sin(yaw / 2);
  return s;
}
export function derivative(state, commands, time, c = AIRFRAME, wind = 0) {
  const s = state,
    q = Array.from(s.slice(6, 10)),
    w = Array.from(s.slice(10, 13));
  const { thrust, torque } = wrench(Array.from(s.slice(13, 17)), c);
  const thrustWorld = rotate(q, [0, 0, thrust]),
    relative = sub(Array.from(s.slice(3, 6)), windAt(time, wind));
  const drag = mul(
    relative,
    -0.5 * c.airDensity * c.dragArea * norm(relative) - c.linearDrag,
  );
  const acceleration = mul(add(thrustWorld, drag), 1 / c.mass);
  acceleration[2] -= c.gravity;
  const gyroscopic = cross(
    w,
    w.map((x, i) => x * c.inertia[i]),
  );
  const alpha = torque.map(
    (x, i) => (x - gyroscopic[i] - c.angularDrag[i] * w[i]) / c.inertia[i],
  );
  const qdot = mul(qmul(q, [0, ...w]), 0.5);
  const out = new Float64Array(21);
  out.set(s.slice(3, 6), 0);
  out.set(acceleration, 3);
  out.set(qdot, 6);
  out.set(alpha, 10);
  for (let i = 0; i < 4; i++) {
    out[13 + i] = (commands[i] - s[13 + i]) / c.motorTimeConstant;
    out[17 + i] = s[13 + i];
  }
  return out;
}
export function rk4Step(s, commands, t, dt, c = AIRFRAME, wind = 0) {
  const tmp = (k, f) => s.map((x, i) => x + k[i] * f);
  const a = derivative(s, commands, t, c, wind),
    b = derivative(tmp(a, dt / 2), commands, t + dt / 2, c, wind),
    d = derivative(tmp(b, dt / 2), commands, t + dt / 2, c, wind),
    e = derivative(tmp(d, dt), commands, t + dt, c, wind);
  const result = s.map(
    (x, i) => x + (dt * (a[i] + 2 * b[i] + 2 * d[i] + e[i])) / 6,
  );
  const qnorm = Math.hypot(...result.slice(6, 10));
  for (let i = 6; i < 10; i++) result[i] /= qnorm;
  return result;
}

export function makeController(yaw) {
  return { integral: [0, 0, 0], yaw, yawRate: 0, disarmTime: null };
}
export function control(state, ref, controller, dt, time, c = AIRFRAME) {
  const p = Array.from(state.slice(0, 3)),
    v = Array.from(state.slice(3, 6)),
    q = Array.from(state.slice(6, 10)),
    w = Array.from(state.slice(10, 13));
  const error = sub(ref.p, p);
  controller.integral = controller.integral.map((x, i) =>
    clamp(x + error[i] * dt, -15, 15),
  );
  let acc = ref.a.map(
    (x, i) =>
      x +
      [1.5, 1.5, 2.4][i] * error[i] +
      [2.5, 2.5, 3.1][i] * (ref.v[i] - v[i]) +
      [0.2, 0.2, 0.35][i] * controller.integral[i],
  );
  acc[2] = clamp(acc[2], -3, 4);
  const maxHorizontal = Math.min(4, Math.tan(c.maxTilt) * (c.gravity + acc[2]));
  const horizontal = Math.hypot(acc[0], acc[1]);
  if (horizontal > maxHorizontal) {
    acc[0] *= maxHorizontal / horizontal;
    acc[1] *= maxHorizontal / horizontal;
  }
  const yawTarget = clamp(
    ref.yawRate + 2 * wrap(ref.yaw - controller.yaw),
    -c.maxYawRate,
    c.maxYawRate,
  );
  controller.yawRate += clamp(
    yawTarget - controller.yawRate,
    -c.maxYawAcceleration * dt,
    c.maxYawAcceleration * dt,
  );
  controller.yaw += controller.yawRate * dt;
  const desiredForce = [acc[0], acc[1], acc[2] + c.gravity];
  const desired = attitudeFromForce(desiredForce, controller.yaw);
  let qe = qmul(conjugate(q), desired);
  if (qe[0] < 0) qe = mul(qe, -1);
  // Feed forward the desired attitude's full angular velocity, including the
  // roll/pitch needed by curve acceleration, rather than yaw alone.
  let feed = [0, 0, 0];
  if (controller.previousDesired) {
    let delta = qmul(conjugate(controller.previousDesired), desired);
    if (delta[0] < 0) delta = mul(delta, -1);
    const length = norm(delta.slice(1));
    const rate =
      length > 1e-10
        ? mul(
            delta.slice(1),
            (2 * Math.atan2(length, delta[0])) / (length * dt),
          )
        : [0, 0, 0];
    feed = rotate(conjugate(q), rotate(controller.previousDesired, rate));
  }
  controller.previousDesired = desired;
  const rate = qe.slice(1).map((x, i) => clamp(8 * x + feed[i], -2, 2));
  const gyro = cross(
    w,
    w.map((x, i) => x * c.inertia[i]),
  );
  const torque = w.map((x, i) =>
    clamp(
      c.inertia[i] * 12 * (rate[i] - x) + gyro[i] + c.angularDrag[i] * x,
      -[0.8, 0.8, 0.35][i],
      [0.8, 0.8, 0.35][i],
    ),
  );
  let thrust = c.mass * Math.max(0, dot(desiredForce, rotate(q, [0, 0, 1])));
  if (ref.phase === "arming") thrust = 0.2 * c.mass * c.gravity;
  // Touchdown is detected from measured state, not the reference clock.
  if (
    ["touchdown", "disarm"].includes(ref.phase) &&
    p[2] < 0.012 &&
    Math.hypot(p[0], p[1]) < 0.15 &&
    norm(v) < 0.25 &&
    controller.disarmTime === null
  )
    controller.disarmTime = time;
  if (controller.disarmTime !== null)
    thrust *= clamp(1 - (time - controller.disarmTime) / 0.5, 0, 1);
  const output = allocate(
    thrust,
    controller.disarmTime !== null ? [0, 0, 0] : torque,
    c,
  );
  return { ...output, referenceError: norm(error) };
}

export function simulate(reference, options = {}, onProgress = () => {}) {
  const c = { ...AIRFRAME, ...options.airframe };
  validateAirframe(c);
  const wind = options.wind ?? 2,
    dt = 1 / c.physicsHz,
    recordEvery = c.physicsHz / c.recordHz,
    controlEvery = c.physicsHz / c.controlHz;
  let state = initialState(reference.initialYaw),
    commands = [0, 0, 0, 0],
    distance = 0,
    previous = [0, 0, 0],
    output;
  const controller = makeController(reference.initialYaw);
  const count = Math.ceil(reference.duration * c.physicsHz);
  const data = new Float64Array((Math.floor(count / recordEvery) + 2) * STRIDE);
  let rows = 0,
    lastAcceleration = [0, 0, 0],
    contact = false;
  const metrics = {
    maxTrackingError: 0,
    maxGroundSpeed: 0,
    maxClimb: 0,
    maxDescent: 0,
    maxTilt: 0,
    maxBodyRate: 0,
    maxYawRate: 0,
    maxRotorSpeed: 0,
    maxAcceleration: 0,
    maxAirborneJerk: 0,
    saturationSeconds: 0,
    contactSpeed: 0,
    steps: count,
    physicsHz: c.physicsHz,
  };
  function record(t, ref) {
    const offset = rows++ * STRIDE,
      { thrust, torque } = wrench(Array.from(state.slice(13, 17)), c);
    data[offset] = t;
    data.set(state.slice(0, 13), offset + 1);
    data.set(lastAcceleration, offset + 14);
    data.set(state.slice(13, 17), offset + 17);
    data[offset + 21] = thrust;
    data.set(torque, offset + 22);
    data.set(windAt(t, wind), offset + 25);
    data[offset + 28] = distance;
    data[offset + 29] = norm(sub(Array.from(state.slice(0, 3)), ref.p));
    data[offset + 30] = PHASE_NAMES.indexOf(ref.phase);
    data[offset + 31] = Number(output?.saturated ?? false);
    data.set(state.slice(17, 21), offset + 32);
  }
  record(0, referenceAt(reference, 0));
  for (let step = 0; step < count; step++) {
    const t = step * dt,
      ref = referenceAt(reference, t);
    if (step % controlEvery === 0) {
      output = control(state, ref, controller, 1 / c.controlHz, t, c);
      commands = output.commands;
    }
    const before = state;
    state = rk4Step(state, commands, t, dt, c, wind);
    // Unilateral pad contact: normal impulse + Coulomb friction. No snapping to
    // the home XY or prescribed orientation; residual landing error is retained.
    if (state[2] < 0) {
      if (!contact && t > reference.surveyStart + reference.horizontalDuration)
        metrics.contactSpeed = Math.max(metrics.contactSpeed, -state[5]);
      const normalImpulse = Math.max(0, -state[5]) * c.mass;
      state[2] = 0;
      if (state[5] < 0) state[5] = 0;
      const planar = Math.hypot(state[3], state[4]);
      const friction = Math.min(
        planar,
        0.8 * (normalImpulse / c.mass + c.gravity * dt),
      );
      if (planar > 0) {
        state[3] *= 1 - friction / planar;
        state[4] *= 1 - friction / planar;
      }
      if (friction >= planar && Math.hypot(before[3], before[4]) < 0.01) {
        state[0] = before[0];
        state[1] = before[1];
      }
      contact = true;
    } else contact = false;
    if (
      !state.every(Number.isFinite) ||
      norm(Array.from(state.slice(0, 3))) > 15000
    )
      throw new Error("Fizik çözümü kararsız: uçuş hazırlanmadı.");
    const p = Array.from(state.slice(0, 3)),
      v = Array.from(state.slice(3, 6)),
      acc = Array.from(state.slice(3, 6)).map(
        (x, i) => (x - before[3 + i]) / dt,
      );
    distance += norm(sub(p, previous));
    previous = p;
    metrics.maxTrackingError = Math.max(
      metrics.maxTrackingError,
      norm(sub(Array.from(before.slice(0, 3)), ref.p)),
    );
    metrics.maxGroundSpeed = Math.max(
      metrics.maxGroundSpeed,
      Math.hypot(v[0], v[1]),
    );
    metrics.maxClimb = Math.max(metrics.maxClimb, v[2]);
    metrics.maxDescent = Math.max(metrics.maxDescent, -v[2]);
    const z = rotate(Array.from(state.slice(6, 10)), [0, 0, 1]);
    metrics.maxTilt = Math.max(metrics.maxTilt, Math.acos(clamp(z[2], -1, 1)));
    metrics.maxBodyRate = Math.max(
      metrics.maxBodyRate,
      norm(Array.from(state.slice(10, 13))),
    );
    metrics.maxYawRate = Math.max(
      metrics.maxYawRate,
      Math.abs(
        rotate(
          Array.from(state.slice(6, 10)),
          Array.from(state.slice(10, 13)),
        )[2],
      ),
    );
    metrics.maxRotorSpeed = Math.max(
      metrics.maxRotorSpeed,
      ...state.slice(13, 17),
    );
    metrics.maxAcceleration = Math.max(metrics.maxAcceleration, norm(acc));
    if (state[2] > 1 && before[2] > 1)
      metrics.maxAirborneJerk = Math.max(
        metrics.maxAirborneJerk,
        norm(sub(acc, lastAcceleration)) / dt,
      );
    if (output.saturated) metrics.saturationSeconds += dt;
    lastAcceleration = acc;
    if ((step + 1) % recordEvery === 0 || step === count - 1)
      record((step + 1) * dt, referenceAt(reference, (step + 1) * dt));
    if (step % 2400 === 0) onProgress(step / count);
  }
  metrics.landingError = Math.hypot(state[0], state[1]);
  metrics.finalSpeed = norm(Array.from(state.slice(3, 6)));
  metrics.disarmed =
    controller.disarmTime !== null && Math.max(...state.slice(13, 17)) < 1;
  if (
    !metrics.disarmed ||
    metrics.landingError > 0.2 ||
    metrics.finalSpeed > 0.05
  )
    throw new Error("İniş doğrulanamadı: kontrolcü hedefe yerleşemedi.");
  return {
    data: data.slice(0, rows * STRIDE),
    duration: count * dt,
    metrics,
    airframe: c,
    wind,
    rows,
  };
}

export function replayAt(sim, time) {
  const t = clamp(time, 0, sim.duration);
  let lo = 0,
    hi = sim.rows - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (sim.data[mid * STRIDE] < t) lo = mid + 1;
    else hi = mid;
  }
  const b = lo,
    a = Math.max(0, b - 1),
    ta = sim.data[a * STRIDE],
    tb = sim.data[b * STRIDE],
    f = tb > ta ? (t - ta) / (tb - ta) : 0;
  const row = Array.from(
    { length: STRIDE },
    (_, i) => sim.data[a * STRIDE + i] * (1 - f) + sim.data[b * STRIDE + i] * f,
  );
  row.splice(
    7,
    4,
    ...slerp(
      Array.from(sim.data.slice(a * STRIDE + 7, a * STRIDE + 11)),
      Array.from(sim.data.slice(b * STRIDE + 7, b * STRIDE + 11)),
      f,
    ),
  );
  return {
    p: row.slice(1, 4),
    v: row.slice(4, 7),
    q: row.slice(7, 11),
    omega: row.slice(11, 14),
    acceleration: row.slice(14, 17),
    motors: row.slice(17, 21),
    thrust: row[21],
    torque: row.slice(22, 25),
    wind: row.slice(25, 28),
    traveled: row[28],
    trackingError: row[29],
    phase:
      time >= sim.duration - 1e-7
        ? "complete"
        : PHASE_NAMES[Math.round(row[30])],
    rotorAngles: row.slice(32, 36),
  };
}
