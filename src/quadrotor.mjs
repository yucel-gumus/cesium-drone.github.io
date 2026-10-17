import {
  clamp,
  dot,
  norm,
  sub,
  mul,
  cross,
  wrap,
  qmul,
  conjugate,
  rotate,
  attitudeFromForce,
  slerp,
} from "./flight-math.mjs";
import { referenceAt } from "./trajectory.mjs";
import { makeRandom } from "./random.mjs";
import {
  BATTERY,
  electricalPower,
  batteryTerminal,
  dischargeStep,
  voltageAuthority,
} from "./battery.mjs";
import { makeNavigation } from "./navigation.mjs";
export { makeRandom } from "./random.mjs";

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
  // Spin inertia of one rotor (10 inch propeller plus motor bell), kg·m²; a
  // generic estimate, not a measured value. Drives the gyroscopic and rotor
  // acceleration reaction torques.
  rotorInertia: 6e-5,
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
// Rotor spin per motor (front right, back right, back left, front left):
// +1 clockwise seen from above, so a rotor's reaction torque on the body is
// +SPIN × reactionRatio × thrust about +z. Front right and back left turn
// counter-clockwise, as in the bundled model and the PX4 quad X layout.
export const SPIN = Object.freeze([-1, 1, -1, 1]);
// Record row layout (STRIDE numbers per 30 Hz row):
//  0 time · 1–3 position · 4–6 velocity · 7–10 quaternion · 11–13 body rates
//  14–16 acceleration · 17–20 rotor speeds · 21 thrust · 22–24 torque
//  25–27 wind · 28 distance · 29 tracking error · 30 phase · 31 saturated
//  32–35 rotor angles · 36 state of charge · 37 pack voltage · 38 current
//  39 electrical power · 40–42 estimated position · 43 EPH (1σ, m)
export const STRIDE = 44;
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
    "rotorInertia",
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
// Mean wind blows towards east-north-east, east:north = 1:0.4.
const WIND_EAST = 1 / Math.hypot(1, 0.4),
  WIND_NORTH = 0.4 / Math.hypot(1, 0.4);
export function meanWind(wind = 0) {
  return [wind * WIND_EAST, wind * WIND_NORTH, 0];
}
// Representative height for the Dryden scale lengths: the survey flies about
// 300 ft (91 m) above ground.
export const TURBULENCE_ALTITUDE = 300 * 0.3048;
// MIL-F-8785C low-altitude Dryden intensities and scale lengths (valid for
// 10–1000 ft). The specification is written in feet with h the height above
// ground and W20 the mean wind at 20 ft; options.wind is used as W20:
//   L_w = h,  L_u = L_v = h / (0.177 + 0.000823 h)^1.2
//   σ_w = 0.1 W20,  σ_u = σ_v = σ_w / (0.177 + 0.000823 h)^0.4
// Scale lengths are returned in metres; σ keeps the unit of `wind`.
export function drydenParameters(wind, altitude = TURBULENCE_ALTITUDE) {
  const FT = 0.3048,
    h = clamp(altitude / FT, 10, 1000),
    g = 0.177 + 0.000823 * h;
  const sigmaW = 0.1 * wind,
    sigmaU = sigmaW / g ** 0.4,
    horizontal = (h / g ** 1.2) * FT;
  return {
    sigma: [sigmaU, sigmaU, sigmaW],
    scale: [horizontal, horizontal, h * FT],
  };
}
// Exact zero-order-hold discretization of the Dryden lateral/vertical filter
// H(s) = σ (1 + √3 τ s) / (1 + τ s)², τ = L / V. In controllable canonical
// form with b = 1/τ, the scaled states z₁ = b^{3/2} x₁ and z₂ = b^{1/2} x₂
// obey ż = b [[0, 1], [−1, −2]] z + [0, √b] n (unit white noise n) and keep
// the stationary covariance ¼ I for every b, so the output σ (z₁ + √3 z₂)
// has variance σ² even while the airspeed V changes between steps. Over one
// step with x = b dt the transition and process noise are
//   Φ = e^{−x} [[1 + x, x], [−x, 1 − x]]
//   Q = ∫₀ˣ e^{−2s} [[s², s (1 − s)], [s (1 − s), (1 − s)²]] ds
// with Q₁₁ = (1 − e^{−2x}(1 + 2x + 2x²)) / 4, Q₁₂ = e^{−2x} x² / 2 and
// Q₂₂ = (1 − e^{−2x}(1 − 2x + 2x²)) / 4; Q₁₁ switches to its power series
// below x = 0.02 to avoid cancellation. `out` receives Φ (row-major) followed
// by the Cholesky factor of Q (l₁₁, l₂₁, l₂₂), ready for two standard normals.
export function drydenCoefficients(x, out = new Float64Array(7)) {
  const e = Math.exp(-x),
    e2 = e * e;
  out[0] = e * (1 + x);
  out[1] = e * x;
  out[2] = -e * x;
  out[3] = e * (1 - x);
  const q11 =
      x < 0.02
        ? x *
          x *
          x *
          (1 / 3 +
            x * (-1 / 2 + x * (2 / 5 + x * (-2 / 9 + x * (2 / 21)))))
        : (1 - e2 * (1 + 2 * x + 2 * x * x)) / 4,
    q12 = (e2 * x * x) / 2,
    q22 = (1 - e2 * (1 - 2 * x + 2 * x * x)) / 4;
  const l11 = Math.sqrt(Math.max(0, q11)),
    l21 = l11 > 0 ? q12 / l11 : 0;
  out[4] = l11;
  out[5] = l21;
  out[6] = Math.sqrt(Math.max(0, q22 - l21 * l21));
  return out;
}
// Mean wind plus Dryden turbulence, advanced once per physics step. The
// longitudinal gust u follows the first order Dryden filter as an exact
// Ornstein–Uhlenbeck step (a = e^{−V dt / L_u}, noise gain σ_u √(1 − a²));
// v and w use drydenCoefficients(). V is the vehicle's air-relative speed,
// as Taylor's frozen-turbulence hypothesis prescribes; at V = 0 the field is
// simply not sampled and the gust holds. Filter states start from their
// stationary distribution, so no warm-up is needed. `vector` is the ENU wind
// [east, north, up] and is updated in place.
export function makeTurbulence(wind = 0, options = {}) {
  const { seed = 1, turbulence = true, altitude = TURBULENCE_ALTITUDE } =
    options;
  if (!Number.isFinite(wind) || wind < 0)
    throw new Error("Geçersiz rüzgâr hızı: negatif olmayan sayı olmalı.");
  if (!Number.isFinite(seed))
    throw new Error("Geçersiz türbülans tohumu: sayı olmalı.");
  const { sigma, scale } = drydenParameters(wind, altitude),
    active = turbulence !== false && wind > 0,
    random = makeRandom(seed),
    vector = new Float64Array(3),
    coefficients = new Float64Array(7),
    SQRT3 = Math.sqrt(3);
  let u = 0,
    v1 = 0,
    v2 = 0,
    w1 = 0,
    w2 = 0;
  if (active) {
    u = sigma[0] * random.normal();
    v1 = 0.5 * random.normal();
    v2 = 0.5 * random.normal();
    w1 = 0.5 * random.normal();
    w2 = 0.5 * random.normal();
  }
  // u along the mean wind, v to its left, w up.
  const compose = () => {
    const v = sigma[1] * (v1 + SQRT3 * v2);
    vector[0] = (wind + u) * WIND_EAST - v * WIND_NORTH;
    vector[1] = (wind + u) * WIND_NORTH + v * WIND_EAST;
    vector[2] = sigma[2] * (w1 + SQRT3 * w2);
  };
  compose();
  const advance = (dt, airspeed = wind) => {
    if (!active) return vector;
    const speed = Math.max(0, airspeed),
      x = (speed * dt) / scale[0];
    u =
      Math.exp(-x) * u +
      sigma[0] * Math.sqrt(-Math.expm1(-2 * x)) * random.normal();
    let c = drydenCoefficients((speed * dt) / scale[1], coefficients),
      n1 = random.normal(),
      n2 = random.normal(),
      z1 = c[0] * v1 + c[1] * v2 + c[4] * n1,
      z2 = c[2] * v1 + c[3] * v2 + c[5] * n1 + c[6] * n2;
    v1 = z1;
    v2 = z2;
    c = drydenCoefficients((speed * dt) / scale[2], coefficients);
    n1 = random.normal();
    n2 = random.normal();
    z1 = c[0] * w1 + c[1] * w2 + c[4] * n1;
    z2 = c[2] * w1 + c[3] * w2 + c[5] * n1 + c[6] * n2;
    w1 = z1;
    w2 = z2;
    compose();
    return vector;
  };
  return { vector, sigma, scale, active, advance };
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
// Mechanical shaft power of the four rotors: rotor drag torque Q = κ f
// (reaction ratio κ) times rotor speed.
export function shaftPower(motors, c = AIRFRAME) {
  const k = kf(c) * c.reactionRatio;
  let sum = 0;
  for (const w of motors) sum += k * w * w * Math.abs(w);
  return sum;
}
export function allocate(
  thrust,
  torque,
  c = AIRFRAME,
  maxRotorSpeed = c.maxRotorSpeed,
) {
  const a = c.armLength / Math.sqrt(2),
    xs = [a, -a, -a, a],
    ys = [-a, -a, a, a],
    max = kf(c) * maxRotorSpeed ** 2;
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
// State layout: 0-2 position, 3-5 velocity, 6-9 quaternion w,x,y,z,
// 10-12 body rates, 13-16 rotor speeds, 17-20 rotor angles. Written with
// scalars and an optional output buffer: this runs ~10⁶ times per mission.
export function derivative(
  state,
  commands,
  time,
  c = AIRFRAME,
  wind = 0,
  out = new Float64Array(21),
) {
  const s = state;
  const qw = s[6],
    qx = s[7],
    qy = s[8],
    qz = s[9],
    wx = s[10],
    wy = s[11],
    wz = s[12];
  // Rotor wrench (same algebra as wrench()).
  const k = kf(c),
    a = c.armLength / Math.sqrt(2);
  const f0 = k * s[13] * s[13],
    f1 = k * s[14] * s[14],
    f2 = k * s[15] * s[15],
    f3 = k * s[16] * s[16];
  const thrust = f0 + f1 + f2 + f3;
  const tx = -a * f0 + -a * f1 + a * f2 + a * f3,
    ty = -(a * f0 + -a * f1 + -a * f2 + a * f3),
    tz =
      c.reactionRatio *
      (SPIN[0] * f0 + SPIN[1] * f1 + SPIN[2] * f2 + SPIN[3] * f3);
  // Thrust along body z rotated into ENU: v + 2 q.w (q.v × v) + 2 q.v × (q.v × v).
  const t0 = qy * thrust * 2,
    t1 = (0 - qx * thrust) * 2;
  const fx = t0 * qw + (0 - qz * t1),
    fy = t1 * qw + (qz * t0 - 0),
    fz = thrust + (qx * t1 - qy * t0);
  // Wind: a number is a steady mean wind of that speed (east-north-east), an
  // ENU array [east, north, up] is used as given. simulate() advances the
  // turbulence generator once per physics step and passes its vector, so all
  // four RK4 stages see one wind sample; `time` stays for signature
  // compatibility and no longer drives any gust model.
  let wind0, wind1, wind2;
  if (typeof wind === "number") {
    wind0 = wind * WIND_EAST;
    wind1 = wind * WIND_NORTH;
    wind2 = 0;
  } else {
    wind0 = wind[0];
    wind1 = wind[1];
    wind2 = wind[2];
  }
  // Air-relative velocity and quadratic plus linear drag.
  const r0 = s[3] - wind0,
    r1 = s[4] - wind1,
    r2 = s[5] - wind2;
  const dragFactor =
    -0.5 * c.airDensity * c.dragArea * Math.hypot(r0, r1, r2) - c.linearDrag;
  const inverseMass = 1 / c.mass;
  out[0] = s[3];
  out[1] = s[4];
  out[2] = s[5];
  out[3] = (fx + r0 * dragFactor) * inverseMass;
  out[4] = (fy + r1 * dragFactor) * inverseMass;
  out[5] = (fz + r2 * dragFactor) * inverseMass - c.gravity;
  // Quaternion kinematics q̇ = ½ q ⊗ [0, Ω].
  out[6] = (0 - qx * wx - qy * wy - qz * wz) * 0.5;
  out[7] = (qw * wx + qy * wz - qz * wy) * 0.5;
  out[8] = (qw * wy - qx * wz + qz * wx) * 0.5;
  out[9] = (qw * wz + qx * wy - qy * wx) * 0.5;
  // Rotor spin-up (first order motor lag), needed below for the reaction.
  const a0 = (commands[0] - s[13]) / c.motorTimeConstant,
    a1 = (commands[1] - s[14]) / c.motorTimeConstant,
    a2 = (commands[2] - s[15]) / c.motorTimeConstant,
    a3 = (commands[3] - s[16]) / c.motorTimeConstant;
  // Rotor inertia. A rotor with SPIN +1 turns clockwise from above, i.e. about
  // −z, so the rotors' spin momentum is h = −J_r Σ SPIN_i ω_i e_z. The body
  // equation I Ω̇ = τ − Ω × (I Ω) − Ω × h − ḣ − D Ω gains the gyroscopic term
  // −Ω × h = J_r Σ SPIN_i ω_i [Ω_y, −Ω_x, 0] and the spin-up reaction
  // −ḣ_z = J_r Σ SPIN_i ω̇_i about z (same sign rule as reactionRatio: a
  // clockwise rotor accelerating torques the body about +z).
  const spinMomentum =
      c.rotorInertia *
      (SPIN[0] * s[13] + SPIN[1] * s[14] + SPIN[2] * s[15] + SPIN[3] * s[16]),
    reaction =
      c.rotorInertia *
      (SPIN[0] * a0 + SPIN[1] * a1 + SPIN[2] * a2 + SPIN[3] * a3);
  // Euler: I Ω̇ = τ − Ω × (I Ω) − D Ω, plus the rotor terms above.
  const [ix, iy, iz] = c.inertia,
    [dx, dy, dz] = c.angularDrag;
  const gx = wy * (wz * iz) - wz * (wy * iy),
    gy = wz * (wx * ix) - wx * (wz * iz),
    gz = wx * (wy * iy) - wy * (wx * ix);
  out[10] = (tx - gx - dx * wx + spinMomentum * wy) / ix;
  out[11] = (ty - gy - dy * wy - spinMomentum * wx) / iy;
  out[12] = (tz - gz - dz * wz + reaction) / iz;
  out[13] = a0;
  out[14] = a1;
  out[15] = a2;
  out[16] = a3;
  out[17] = s[13];
  out[18] = s[14];
  out[19] = s[15];
  out[20] = s[16];
  return out;
}
const k1 = new Float64Array(21),
  k2 = new Float64Array(21),
  k3 = new Float64Array(21),
  k4 = new Float64Array(21),
  stage = new Float64Array(21);
export function rk4Step(s, commands, t, dt, c = AIRFRAME, wind = 0) {
  derivative(s, commands, t, c, wind, k1);
  for (let i = 0; i < 21; i++) stage[i] = s[i] + k1[i] * (dt / 2);
  derivative(stage, commands, t + dt / 2, c, wind, k2);
  for (let i = 0; i < 21; i++) stage[i] = s[i] + k2[i] * (dt / 2);
  derivative(stage, commands, t + dt / 2, c, wind, k3);
  for (let i = 0; i < 21; i++) stage[i] = s[i] + k3[i] * dt;
  derivative(stage, commands, t + dt, c, wind, k4);
  const result = new Float64Array(21);
  for (let i = 0; i < 21; i++)
    result[i] = s[i] + (dt * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])) / 6;
  const qnorm = Math.hypot(result[6], result[7], result[8], result[9]);
  for (let i = 6; i < 10; i++) result[i] /= qnorm;
  return result;
}

export function makeController(yaw) {
  return {
    integral: [0, 0, 0],
    yaw,
    yawRate: 0,
    disarmTime: null,
    landTimer: 0,
    // Navigation offset captured at the start of take-off (see control()).
    anchor: null,
    previousDesired: null,
  };
}
// `estimate` carries the navigation solution ({ position, velocity }) and
// the voltage-limited top rotor speed; without it the controller sees truth.
// Attitude and body rates come from an ideal AHRS in both cases.
export function control(
  state,
  ref,
  controller,
  dt,
  time,
  c = AIRFRAME,
  estimate = {},
) {
  const p = estimate.position
      ? Array.from(estimate.position)
      : Array.from(state.slice(0, 3)),
    v = estimate.velocity
      ? Array.from(estimate.velocity)
      : Array.from(state.slice(3, 6)),
    q = Array.from(state.slice(6, 10)),
    w = Array.from(state.slice(10, 13));
  // Take off from where the navigation solution says the vehicle is, as
  // flight stacks do: the horizontal offset between estimate and plan at the
  // start of take-off is held, then blended out with a smoothstep over the
  // climb, so the vehicle leaves the pad vertically and joins the survey
  // with a continuous reference.
  if (ref.phase === "takeoff") {
    controller.anchor ??= [p[0] - ref.p[0], p[1] - ref.p[1]];
    const u = ref.phaseProgress,
      blend = 1 - u * u * (3 - 2 * u),
      rate = (-6 * u * (1 - u)) / ref.phaseDuration;
    ref = {
      ...ref,
      p: [
        ref.p[0] + controller.anchor[0] * blend,
        ref.p[1] + controller.anchor[1] * blend,
        ref.p[2],
      ],
      v: [
        ref.v[0] + controller.anchor[0] * rate,
        ref.v[1] + controller.anchor[1] * rate,
        ref.v[2],
      ],
    };
  }
  const error = sub(ref.p, p);
  // On the pad, while arming or once the land detector suspects ground
  // contact, there is no horizontal correction and no integration, so a
  // navigation offset cannot wind up the integrators before lift-off. Once
  // airborne the position loop holds the vehicle against the wind.
  const groundPhase = ref.phase === "arming" || controller.landTimer > 0;
  if (!groundPhase)
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
  if (groundPhase) {
    acc[0] = 0;
    acc[1] = 0;
  }
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
  // Rotor gyroscopic torque J_r Σ SPIN_i ω_i [Ω_y, −Ω_x, 0] acts on the body
  // (see derivative()); cancel it under the same ideal-state assumption as
  // the Euler coupling, using the integrated rotor speeds.
  const spinMomentum =
      c.rotorInertia * dot(SPIN, Array.from(state.slice(13, 17))),
    rotorGyro = [spinMomentum * w[1], -spinMomentum * w[0], 0];
  const torque = w.map((x, i) =>
    clamp(
      c.inertia[i] * 12 * (rate[i] - x) +
        gyro[i] +
        c.angularDrag[i] * x -
        rotorGyro[i],
      -[0.8, 0.8, 0.35][i],
      [0.8, 0.8, 0.35][i],
    ),
  );
  let thrust = c.mass * Math.max(0, dot(desiredForce, rotate(q, [0, 0, 1])));
  if (ref.phase === "arming") thrust = 0.2 * c.mass * c.gravity;
  // Land detector in the style of flight stacks: during the landing descent
  // the vehicle is on the ground once the collective thrust the controller
  // asks for falls well below hover while the velocity stays near zero for a
  // short while. Neither absolute height nor home position is used, so it
  // works with an imperfect navigation solution.
  if (
    ["touchdown", "disarm"].includes(ref.phase) &&
    controller.disarmTime === null
  ) {
    const grounded =
      thrust < 0.8 * c.mass * c.gravity &&
      Math.abs(v[2]) < 0.15 &&
      Math.hypot(v[0], v[1]) < 0.25;
    controller.landTimer = grounded ? controller.landTimer + dt : 0;
    if (controller.landTimer >= 0.3) controller.disarmTime = time;
  }
  if (controller.disarmTime !== null)
    thrust *= clamp(1 - (time - controller.disarmTime) / 0.5, 0, 1);
  const output = allocate(
    thrust,
    controller.disarmTime !== null ? [0, 0, 0] : torque,
    c,
    estimate.maxRotorSpeed ?? c.maxRotorSpeed,
  );
  return { ...output, referenceError: norm(error) };
}

export function simulate(reference, options = {}, onProgress = () => {}) {
  const c = { ...AIRFRAME, ...options.airframe };
  validateAirframe(c);
  const wind = options.wind ?? 2,
    seed = options.seed ?? 1,
    turbulence = options.turbulence ?? true,
    gnssMode = options.gnss ?? "rtk",
    battery = { ...BATTERY, ...options.battery },
    dt = 1 / c.physicsHz,
    recordEvery = c.physicsHz / c.recordHz,
    controlEvery = c.physicsHz / c.controlHz;
  // One wind sample per physics step, shared by the four RK4 stages.
  const gust = makeTurbulence(wind, { seed, turbulence });
  const navigation = makeNavigation(gnssMode, seed);
  let soc = 1,
    pack = batteryTerminal(battery.avionicsPower, soc, battery),
    power = battery.avionicsPower;
  let state = initialState(reference.initialYaw),
    commands = [0, 0, 0, 0],
    distance = 0,
    px = 0,
    py = 0,
    pz = 0,
    output;
  const controller = makeController(reference.initialYaw);
  const count = Math.ceil(reference.duration * c.physicsHz);
  const data = new Float64Array((Math.floor(count / recordEvery) + 2) * STRIDE);
  let rows = 0,
    lastAcceleration = new Float64Array(3),
    contact = false;
  // Delta-velocity IMU: the navigation filter receives the mean acceleration
  // over each control interval, not one physics step's sample.
  const imuVelocity = [0, 0, 0],
    imuAcceleration = [0, 0, 0];
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
    energyWh: 0,
    peakPower: 0,
    minVoltage: pack.voltage,
    batteryLimitedSeconds: 0,
    maxNavigationError: 0,
    maxEph: 0,
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
    data[offset + 25] = gust.vector[0];
    data[offset + 26] = gust.vector[1];
    data[offset + 27] = gust.vector[2];
    data[offset + 28] = distance;
    // The landing reference continues below the pad; on the ground the
    // tracking error is measured against the reference clipped to the pad.
    data[offset + 29] = Math.hypot(
      state[0] - ref.p[0],
      state[1] - ref.p[1],
      state[2] - Math.max(0, ref.p[2]),
    );
    data[offset + 30] = PHASE_NAMES.indexOf(ref.phase);
    data[offset + 31] = Number(output?.saturated ?? false);
    data.set(state.slice(17, 21), offset + 32);
    data[offset + 36] = soc;
    data[offset + 37] = pack.voltage;
    data[offset + 38] = pack.current;
    data[offset + 39] = power;
    data[offset + 40] = navigation.position[0];
    data[offset + 41] = navigation.position[1];
    data[offset + 42] = navigation.position[2];
    data[offset + 43] = navigation.eph;
  }
  record(0, referenceAt(reference, 0));
  for (let step = 0; step < count; step++) {
    const t = step * dt,
      ref = referenceAt(reference, t);
    if (step % controlEvery === 0) {
      for (let i = 0; i < 3; i++) {
        imuAcceleration[i] = (state[3 + i] - imuVelocity[i]) * c.controlHz;
        imuVelocity[i] = state[3 + i];
      }
      navigation.step(
        [state[0], state[1], state[2]],
        [state[3], state[4], state[5]],
        imuAcceleration,
        1 / c.controlHz,
      );
      output = control(state, ref, controller, 1 / c.controlHz, t, c, {
        position: navigation.position,
        velocity: navigation.velocity,
        maxRotorSpeed: c.maxRotorSpeed * voltageAuthority(pack.voltage, battery),
      });
      commands = output.commands;
      if (state[2] > 0.5)
        metrics.maxNavigationError = Math.max(
          metrics.maxNavigationError,
          Math.hypot(
            navigation.position[0] - state[0],
            navigation.position[1] - state[1],
            navigation.position[2] - state[2],
          ),
        );
      metrics.maxEph = Math.max(metrics.maxEph, navigation.eph);
    }
    const before = state;
    state = rk4Step(state, commands, t, dt, c, gust.vector);
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
      // Landing gear on a rigid, level pad: pressed onto it the airframe
      // cannot roll or pitch, and gear friction stops any rotation. Yaw is
      // kept.
      const yaw = Math.atan2(
        2 * (state[6] * state[9] + state[7] * state[8]),
        1 - 2 * (state[8] * state[8] + state[9] * state[9]),
      );
      state[6] = Math.cos(yaw / 2);
      state[7] = 0;
      state[8] = 0;
      state[9] = Math.sin(yaw / 2);
      state[10] = 0;
      state[11] = 0;
      state[12] = 0;
      contact = true;
    } else contact = false;
    // Scalar check: no per-step allocation in the 240 Hz loop.
    if (
      !state.every(Number.isFinite) ||
      Math.hypot(state[0], state[1], state[2]) > 15000
    )
      throw new Error("Fizik çözümü kararsız: uçuş hazırlanmadı.");
    const ax = (state[3] - before[3]) / dt,
      ay = (state[4] - before[4]) / dt,
      az = (state[5] - before[5]) / dt;
    distance += Math.hypot(state[0] - px, state[1] - py, state[2] - pz);
    px = state[0];
    py = state[1];
    pz = state[2];
    metrics.maxTrackingError = Math.max(
      metrics.maxTrackingError,
      Math.hypot(
        before[0] - ref.p[0],
        before[1] - ref.p[1],
        before[2] - Math.max(0, ref.p[2]),
      ),
    );
    metrics.maxGroundSpeed = Math.max(
      metrics.maxGroundSpeed,
      Math.hypot(state[3], state[4]),
    );
    metrics.maxClimb = Math.max(metrics.maxClimb, state[5]);
    metrics.maxDescent = Math.max(metrics.maxDescent, -state[5]);
    const qw = state[6],
      qx = state[7],
      qy = state[8],
      qz = state[9],
      wx = state[10],
      wy = state[11],
      wz = state[12];
    // Body z axis in ENU (tilt) and body rate rotated into ENU (yaw rate).
    metrics.maxTilt = Math.max(
      metrics.maxTilt,
      Math.acos(clamp(1 + (qx * (-qx * 2) - qy * (qy * 2)), -1, 1)),
    );
    metrics.maxBodyRate = Math.max(metrics.maxBodyRate, Math.hypot(wx, wy, wz));
    const r0 = (qy * wz - qz * wy) * 2,
      r1 = (qz * wx - qx * wz) * 2,
      r2 = (qx * wy - qy * wx) * 2;
    metrics.maxYawRate = Math.max(
      metrics.maxYawRate,
      Math.abs(wz + (r2 * qw + (qx * r1 - qy * r0))),
    );
    metrics.maxRotorSpeed = Math.max(
      metrics.maxRotorSpeed,
      state[13],
      state[14],
      state[15],
      state[16],
    );
    metrics.maxAcceleration = Math.max(
      metrics.maxAcceleration,
      Math.hypot(ax, ay, az),
    );
    if (state[2] > 1 && before[2] > 1)
      metrics.maxAirborneJerk = Math.max(
        metrics.maxAirborneJerk,
        Math.hypot(
          ax - lastAcceleration[0],
          ay - lastAcceleration[1],
          az - lastAcceleration[2],
        ) / dt,
      );
    if (output.saturated) metrics.saturationSeconds += dt;
    // Pack: shaft power through the propulsion efficiency plus avionics,
    // Thevenin terminal voltage, Coulomb counting.
    power = electricalPower(shaftPower(state.subarray(13, 17), c), battery);
    pack = batteryTerminal(power, soc, battery);
    soc = dischargeStep(soc, pack.current, dt, battery);
    metrics.energyWh += (pack.voltage * pack.current * dt) / 3600;
    metrics.peakPower = Math.max(metrics.peakPower, power);
    metrics.minVoltage = Math.min(metrics.minVoltage, pack.voltage);
    if (pack.limited) metrics.batteryLimitedSeconds += dt;
    lastAcceleration[0] = ax;
    lastAcceleration[1] = ay;
    lastAcceleration[2] = az;
    // Next step's wind; the frozen field is traversed at air-relative speed.
    gust.advance(
      dt,
      Math.hypot(
        state[3] - gust.vector[0],
        state[4] - gust.vector[1],
        state[5] - gust.vector[2],
      ),
    );
    if ((step + 1) % recordEvery === 0 || step === count - 1)
      record((step + 1) * dt, referenceAt(reference, (step + 1) * dt));
    if (step % 2400 === 0) onProgress(step / count);
  }
  metrics.landingError = Math.hypot(state[0], state[1]);
  metrics.finalSpeed = norm(Array.from(state.slice(3, 6)));
  metrics.disarmed =
    controller.disarmTime !== null && Math.max(...state.slice(13, 17)) < 1;
  metrics.disarmTime = controller.disarmTime;
  metrics.finalSoc = soc;
  if (
    !metrics.disarmed ||
    metrics.landingError > 5 ||
    metrics.finalSpeed > 0.05
  )
    throw new Error("İniş doğrulanamadı: kontrolcü hedefe yerleşemedi.");
  return {
    data: data.slice(0, rows * STRIDE),
    duration: count * dt,
    metrics,
    airframe: c,
    wind,
    seed,
    turbulence,
    gnss: gnssMode,
    battery,
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
    battery: {
      soc: row[36],
      voltage: row[37],
      current: row[38],
      power: row[39],
    },
    estimate: row.slice(40, 43),
    eph: row[43],
  };
}
