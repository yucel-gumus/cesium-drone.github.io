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
import { makeRandom } from "./random.mjs";
import {
  BATTERY,
  electricalPower,
  batteryTerminal,
  dischargeStep,
  voltageAuthority,
} from "./battery.mjs";
import { makeNavigation } from "./navigation.mjs";
import { makeWindEstimator } from "./wind-estimator.mjs";
import {
  FAILSAFE,
  OPERATOR_COMMANDS,
  validateFault,
  validateCommands,
  makeGuidance,
  groundHeight,
  returnCharge,
} from "./failsafe.mjs";
import { obstacleText } from "./obstacles.mjs";
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
  // Rotor drag (H-force) coefficient at hover rotor speed, N·s/m; it scales
  // with the rotor speed sum and acts in the rotor plane.
  linearDrag: 0.1,
  // 10-inch propeller radius and the rotor plane's height above the ground
  // when the airframe rests on its gear, m (ground effect, induced velocity).
  rotorRadius: 0.127,
  rotorClearance: 0.15,
  // Blade flapping: moment per unit rotor speed sum and in-plane airspeed,
  // N·m / (rad/s · m/s); tilts the discs away from the relative wind.
  flappingMoment: 2.5e-6,
  // Peak thrust loss in the vortex ring state.
  vrsLoss: 0.25,
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
//  44–45 estimated wind (east, north)
export const STRIDE = 46;
// Phase codes of the record. Codes 0–8 keep their meaning; failsafe and
// area-survey phases follow: "hold" (stopped on the path by a failsafe),
// "rtl" (failsafe return to home) and "transit" (to and from a survey grid).
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
  "hold",
  "rtl",
  "transit",
];
// Rotor effectiveness of a healthy airframe (see FAULTS.motor).
const FULL = Object.freeze([1, 1, 1, 1]);
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
    "rotorRadius",
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
// Hover induced velocity of one rotor at the airframe's weight (momentum
// theory, v_h = √(T / 2ρA)), m/s: the scale of the vortex ring state.
export const inducedVelocity = (c = AIRFRAME) =>
  Math.sqrt(
    (c.mass * c.gravity) / 4 / (2 * c.airDensity * Math.PI * c.rotorRadius ** 2),
  );
// Thrust fraction lost to the vortex ring state for an axial descent rate
// and in-plane airspeed relative to a rotor's induced velocity v_h, as
// derivative() applies it.
export function vortexRingLoss(descent, inPlane, vh, c = AIRFRAME) {
  const ratio = descent / vh;
  if (!(ratio > 0.3 && ratio < 1.6)) return 0;
  const bump = Math.sin((Math.PI * (ratio - 0.3)) / 1.3),
    escape = inPlane / (0.5 * vh);
  return c.vrsLoss * bump * bump * Math.exp(-escape * escape);
}
// Sum of the four rotor speeds in hover, rad/s.
export const hoverRotors = (c = AIRFRAME) =>
  4 * Math.sqrt((c.mass * c.gravity) / (4 * kf(c)));
// Horizontal drag force (N) at an air-relative speed in level cruise at
// hover rotor speeds: body drag plus rotor drag, as derivative() integrates
// them.
export function dragForce(airspeed, c = AIRFRAME) {
  return 0.5 * c.airDensity * c.dragArea * airspeed * airspeed + c.linearDrag * airspeed;
}
// Mean-wind authority limit (m/s) of an airframe at a cruise speed. On the
// upwind leg the controller must hold the drag of cruise + W + 2σ_u (a
// typical Dryden gust, σ_u = 0.141 W at the 300 ft scale height) with at
// most 25° bank and 4 m/s². The demand is calibrated to the measured limit
// of the 2 kg profile, 3.5 m/s, at which every one of 40 turbulence seeds
// kept the full mission's tracking error under 1 m (README "Rüzgâr yetki
// sınırı"): drag grows with airspeed, the available acceleration shrinks
// with mass, so lighter airframes run out of authority in less wind.
// Returned in 0.1 m/s steps, at most `cap`.
export const WIND_REFERENCE = Object.freeze({ mass: 2, wind: 3.5 });
export function windLimit(c = AIRFRAME, cruise = 8, cap = 6) {
  const gust = 1 + 2 * (drydenParameters(1).sigma[0]),
    reference = { ...AIRFRAME, mass: WIND_REFERENCE.mass },
    demand =
      dragForce(cruise + gust * WIND_REFERENCE.wind, reference) /
      WIND_REFERENCE.mass;
  const a = 0.5 * c.airDensity * c.dragArea,
    airspeed =
      (-c.linearDrag + Math.sqrt(c.linearDrag ** 2 + 4 * a * c.mass * demand)) /
      (2 * a),
    wind = (airspeed - cruise) / gust;
  return Math.max(0, Math.min(cap, Math.floor(wind * 10 + 1e-6) / 10));
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
    ys = [-a, -a, a, a],
    e = c.effectiveness ?? FULL;
  const f = motors.map((w, i) => kf(c) * e[i] * w * w);
  return {
    thrust: f.reduce((a, b) => a + b, 0),
    torque: [dot(ys, f), -dot(xs, f), c.reactionRatio * dot(SPIN, f)],
  };
}
// Mechanical shaft power of the four rotors: rotor drag torque Q = κ f
// (reaction ratio κ) times rotor speed. A rotor with reduced effectiveness
// gives less thrust and torque at a given speed, so it must spin faster for
// the same thrust and draws more power for it.
export function shaftPower(motors, c = AIRFRAME) {
  const k = kf(c) * c.reactionRatio,
    e = c.effectiveness ?? FULL;
  let sum = 0;
  for (let i = 0; i < 4; i++) {
    const w = motors[i];
    sum += k * e[i] * w * w * Math.abs(w);
  }
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
  // Air-relative velocity, split along the rotor axis (body z in ENU) and in
  // the rotor plane.
  const r0 = s[3] - wind0,
    r1 = s[4] - wind1,
    r2 = s[5] - wind2;
  const zx = 2 * (qw * qy + qx * qz),
    zy = 2 * (qy * qz - qw * qx),
    zz = 1 - 2 * (qx * qx + qy * qy),
    axial = r0 * zx + r1 * zy + r2 * zz,
    p0 = r0 - axial * zx,
    p1 = r1 - axial * zy,
    p2 = r2 - axial * zz,
    inPlane = Math.hypot(p0, p1, p2);
  // Rotor wrench (same algebra as wrench()); `effectiveness` scales each
  // rotor's thrust and reaction torque (an injected motor fault).
  const k = kf(c),
    a = c.armLength / Math.sqrt(2),
    e = c.effectiveness ?? FULL,
    R = c.rotorRadius;
  let f0 = k * e[0] * s[13] * s[13],
    f1 = k * e[1] * s[14] * s[14],
    f2 = k * e[2] * s[15] * s[15],
    f3 = k * e[3] * s[16] * s[16];
  // Ground effect (Cheeseman–Bennett): T / T∞ = 1 / (1 − (R / 4h)²) for a
  // rotor plane h above the surface below (`groundLevel`, the CG's rest
  // height set by simulate()), limited at h = R / 2.
  let scale = 1;
  const h = s[2] - (c.groundLevel ?? -Infinity) + c.rotorClearance;
  if (h < 8 * R) {
    const x = R / (4 * Math.max(h, R / 2));
    scale = 1 / (1 - x * x);
  }
  // Vortex ring state: descending into its own downwash at 0.3–1.6 times
  // the hover induced velocity v_h = √(T / 2ρA), with little in-plane
  // airflow to carry the wake away, a rotor loses thrust (empirical bump
  // peaking near v_h, after Johnson's VRS model).
  const mean = (f0 + f1 + f2 + f3) / 4;
  if (mean > 0)
    scale *= 1 - vortexRingLoss(
      -axial,
      inPlane,
      Math.sqrt(mean / (2 * c.airDensity * Math.PI * R * R)),
      c,
    );
  if (scale !== 1) {
    f0 *= scale;
    f1 *= scale;
    f2 *= scale;
    f3 *= scale;
  }
  const thrust = f0 + f1 + f2 + f3;
  const tx = -a * f0 + -a * f1 + a * f2 + a * f3,
    ty = -(a * f0 + -a * f1 + -a * f2 + a * f3),
    tz =
      c.reactionRatio *
      (SPIN[0] * f0 + SPIN[1] * f1 + SPIN[2] * f2 + SPIN[3] * f3);
  // Thrust along the rotor axis.
  const fx = thrust * zx,
    fy = thrust * zy,
    fz = thrust * zz;
  // Body drag (quadratic in airspeed) and rotor drag: the H-force acts in
  // the rotor plane and grows with the rotor speeds, linearDrag at hover.
  const rotors = s[13] + s[14] + s[15] + s[16],
    hover = 4 * Math.sqrt((c.mass * c.gravity) / (4 * k)),
    body = -0.5 * c.airDensity * c.dragArea * Math.hypot(r0, r1, r2),
    rotorDrag = (-c.linearDrag * rotors) / hover;
  const inverseMass = 1 / c.mass;
  out[0] = s[3];
  out[1] = s[4];
  out[2] = s[5];
  out[3] = (fx + r0 * body + p0 * rotorDrag) * inverseMass;
  out[4] = (fy + r1 * body + p1 * rotorDrag) * inverseMass;
  out[5] = (fz + r2 * body + p2 * rotorDrag) * inverseMass - c.gravity;
  // Blade flapping: the in-plane airflow in body axes (attitude conjugate)
  // tilts the discs away from it, M = −k Σω (e_z × v_plane): forward flight
  // pitches the nose up, sideslip rolls away from the relative wind.
  const ux = -qx,
    uy = -qy,
    uz = -qz,
    cx = uy * p2 - uz * p1,
    cy = uz * p0 - ux * p2,
    cz = ux * p1 - uy * p0,
    bx = p0 + 2 * (qw * cx + (uy * cz - uz * cy)),
    by = p1 + 2 * (qw * cy + (uz * cx - ux * cz)),
    flap = c.flappingMoment * rotors,
    mx = flap * by,
    my = -flap * bx;
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
  out[10] = (tx + mx - gx - dx * wx + spinMomentum * wy) / ix;
  out[11] = (ty + my - gy - dy * wy - spinMomentum * wx) / iy;
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

// Integral gain of the body-rate loop, 1/s² (proportional gain 12 1/s).
const RATE_INTEGRAL_GAIN = 15;
export function makeController(yaw) {
  return {
    integral: [0, 0, 0],
    // Body-rate integrator and the lift-off latch that releases it (see
    // control()).
    rateIntegral: [0, 0, 0],
    airborne: false,
    yaw,
    yawRate: 0,
    disarmTime: null,
    landTimer: 0,
    // Navigation offset captured at the start of take-off (see control()).
    anchor: null,
    previousDesired: null,
  };
}
// `estimate` carries the navigation solution ({ position, velocity,
// attitude, rates }: the EKF's position, velocity, attitude and bias-
// corrected gyro rates), the voltage-limited top rotor speed, the land
// detector's horizontal speed limit and the wind estimate; whatever it
// leaves out the controller takes from the true state.
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
    q = estimate.attitude
      ? Array.from(estimate.attitude)
      : Array.from(state.slice(6, 10)),
    w = estimate.rates
      ? Array.from(estimate.rates)
      : Array.from(state.slice(10, 13));
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
  // Drag feed-forward: the planned velocity relative to the estimated wind
  // gives the drag the airframe will meet; the thrust tilts for it in
  // advance instead of waiting for the position error and the integrator.
  let feedDrag = [0, 0];
  if (estimate.wind && !groundPhase) {
    const ax = ref.v[0] - estimate.wind[0],
      ay = ref.v[1] - estimate.wind[1],
      airspeed = Math.hypot(ax, ay),
      k = (0.5 * c.airDensity * c.dragArea * airspeed + c.linearDrag) / c.mass;
    feedDrag = [k * ax, k * ay];
  }
  let acc = ref.a.map(
    (x, i) =>
      x +
      (i < 2 ? feedDrag[i] : 0) +
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
  // Body-rate integrator, as in PX4's rate controller (MC_*RATE_I): it
  // absorbs steady torque disturbances, such as a rotor that lost
  // effectiveness, which a proportional rate loop would leave as a standing
  // attitude error. Limited to 30 % of the torque authority (MC_RR_INT_LIM).
  // Pressed onto the pad the airframe cannot rotate, so the integrator only
  // runs once lift-off is seen (estimated climb) and stops with the land
  // detector, as flight stacks gate it with their landed state.
  const limits = [0.8, 0.8, 0.35];
  if (!controller.airborne && ref.phase !== "arming" && v[2] > 0.25)
    controller.airborne = true;
  if (controller.landTimer > 0) controller.airborne = false;
  if (controller.airborne)
    controller.rateIntegral = controller.rateIntegral.map((x, i) => {
      const bound = (0.3 * limits[i]) / (c.inertia[i] * RATE_INTEGRAL_GAIN);
      return clamp(x + (rate[i] - w[i]) * dt, -bound, bound);
    });
  const torque = w.map((x, i) =>
    clamp(
      c.inertia[i] *
        (12 * (rate[i] - x) +
          RATE_INTEGRAL_GAIN * controller.rateIntegral[i]) +
        gyro[i] +
        c.angularDrag[i] * x -
        rotorGyro[i],
      -limits[i],
      limits[i],
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
      Math.hypot(v[0], v[1]) < (estimate.landSpeed ?? 0.25);
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

// Longest extension of a flight beyond its planned duration that a failsafe
// may cause (holds, a longer return, a field landing), s.
const MAX_EXTENSION = 1800;
const LANDING_PHASES = new Set(["settle", "landing", "touchdown", "disarm"]);
const EN_ROUTE = new Set(["transit", "survey", "return"]);

// Solves the whole flight in advance. Besides the airframe, wind and
// navigation options, `options.faults` ([{ type, time }], see FAULTS; or a
// single `options.fault`) inject faults at mission times, `options.commands`
// ([{ time, action }], see OPERATOR_COMMANDS) are the operator's commands
// and `options.failsafe` arms the autopilot's failsafes (on by default with
// faults or commands). Failsafe legs need the planner's `options.settings`
// and the elevation model `options.terrain` (terrainForWorker()); without a
// model the ground is level with the pad. `options.rally` ([{ name, xy }])
// are safe landing points a return may choose (`options.rtlTarget`
// "nearest", the default, or "home").
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
  const faults = (options.faults ?? (options.fault ? [options.fault] : [])).map(
      validateFault,
    ),
    fault = faults[0] ?? null,
    orders = validateCommands(options.commands ?? []),
    failsafe = options.failsafe ?? (faults.length > 0 || orders.length > 0),
    settings = options.settings,
    terrain = options.terrain ?? null;
  if (new Set(faults.map((f) => f.type.replace(/-.*/, ""))).size !== faults.length)
    throw new Error("Aynı türden iki arıza birlikte seçilemez.");
  if (failsafe && !settings)
    throw new Error("Arıza ve failsafe için uçuş ayarları gerekli.");
  // The airframe the physics integrates; a motor fault changes only its
  // rotor effectiveness, the controller keeps flying the nominal model.
  const plant = faults.some((f) => f.type === "motor")
    ? { ...c, effectiveness: [1, 1, 1, 1] }
    : c;
  // One wind sample per physics step, shared by the four RK4 stages.
  const gust = makeTurbulence(wind, { seed, turbulence });
  const navigation = makeNavigation(gnssMode, seed, {
    drag: {
      mass: c.mass,
      area: 0.5 * c.airDensity * c.dragArea,
      linear: c.linearDrag,
    },
  });
  // Wind estimate: the EKF's wind states; with ideal navigation (no EKF)
  // the standalone drag-model estimator.
  const windEstimator = makeWindEstimator(c);
  const windEstimate = () =>
    navigation.ideal
      ? windEstimator.initialized
        ? windEstimator.wind
        : [0, 0]
      : navigation.wind;
  const guidance = makeGuidance(reference, {
    settings,
    terrain,
    rally: options.rally ?? [],
    rtlTarget: options.rtlTarget ?? "nearest",
  });
  // CG rest height of the surface under (x, y): the pad, or for a field
  // landing the elevation model's ground.
  const surfaceAt = (x, y) =>
    guidance.landing.site !== "home"
      ? terrain
        ? groundHeight(terrain, x, y) + settings.landingOffset
        : guidance.surface
      : 0;
  // Surface under the rotors for ground effect: the landing surface, the
  // pad near home, otherwise the elevation model where one was given.
  const landingOffset = settings?.landingOffset ?? 0.1;
  const groundUnder = (x, y) =>
    guidance.landing.site !== "home"
      ? surfaceAt(x, y)
      : Math.hypot(x, y) < 10
        ? 0
        : terrain
          ? groundHeight(terrain, x, y) + landingOffset
          : -Infinity;
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
  const steps = () => Math.ceil(guidance.endTime * c.physicsHz);
  let data = new Float64Array(
    (Math.floor(steps() / recordEvery) + 2) * STRIDE,
  );
  let rows = 0,
    lastAcceleration = new Float64Array(3),
    contact = false;
  // Delta-velocity and delta-angle IMU: the navigation filter receives the
  // mean acceleration and the mean body rate over each control interval
  // (the rotation actually made, including the gear levelling the airframe
  // on touchdown), not one physics step's sample.
  const imuVelocity = [0, 0, 0],
    imuAcceleration = [0, 0, 0],
    imuAttitude = Float64Array.from(state.subarray(6, 10)),
    imuRates = [0, 0, 0];
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
    contactTime: null,
    // Largest vortex-ring thrust loss met and the fastest descent with
    // little in-plane airflow (below half the induced velocity).
    maxVrsLoss: 0,
    maxSlowDescent: 0,
    // Largest airborne attitude estimation errors, radians.
    maxTiltError: 0,
    maxHeadingError: 0,
  };
  // Fault injection and the autopilot's failsafe monitors, evaluated at
  // the control rate. `events` is the failsafe log; `status` the operator
  // banner over time: the cause of the failsafe in force (the flight phase
  // names the action), null when the mission runs normally.
  const events = [],
    status = [];
  const monitor = {
    injected: false,
    restored: false,
    gnssLostAt: null,
    gnssBackAt: null,
    holdReason: null,
    holdSince: null,
    imbalance: [0, 0, 0, 0],
    imbalanceSince: null,
    motorFlagged: false,
    returnReason: null,
    returnNeed: null,
    action: null,
    operator: false,
    siteLogged: false,
    // Low-passed hover-equivalent shaft power (W), see returnCharge().
    hoverShaft: null,
  };
  const log = (time, kind, label, detail = "") =>
    events.push({ time, kind, label, detail });
  const banner = (time, text) => status.push({ time, text });
  const pct = (x) => `%${Math.round(x * 100)}`;
  function supervise(t) {
    for (const f of faults) {
      if (!f.injected && t >= f.time) {
        f.injected = true;
        monitor.injected = true;
        if (f.type === "battery") {
          const before = soc;
          soc = Math.max(0, soc - f.loss);
          log(t, "fault", "ARIZA · batarya hücresi", `şarj ${pct(before)} → ${pct(soc)}`);
        } else if (f.type === "motor") {
          plant.effectiveness[f.motor] = f.effectiveness;
          log(
            t,
            "fault",
            `ARIZA · motor ${f.motor + 1} verim kaybı`,
            `itki etkinliği ${pct(f.effectiveness)}`,
          );
        } else {
          navigation.gnssAvailable = false;
          log(
            t,
            "fault",
            "ARIZA · GNSS sinyali kesildi",
            Number.isFinite(f.duration) ? `${f.duration} s kesinti` : "kalıcı kayıp",
          );
        }
      }
      if (
        f.type.startsWith("gnss") &&
        f.injected &&
        !f.restored &&
        t >= f.time + f.duration
      ) {
        f.restored = true;
        navigation.gnssAvailable = true;
        log(t, "info", "GNSS geri geldi", `${(t - f.time).toFixed(0)} s sonra`);
      }
    }
    if (!failsafe) return;
    // A landing away from home has chosen its touchdown point.
    if (guidance.mode === "land" && !monitor.siteLogged) {
      monitor.siteLogged = true;
      const site = guidance.landing;
      if (site.moved)
        log(
          t,
          "failsafe",
          "İniş yeri engelden uzağa alındı",
          `${Math.round(site.moved)} m · ${obstacleText(site.obstacle)}`,
        );
      else if (site.obstacle && !monitor.operator)
        log(
          t,
          "failsafe",
          "Engelsiz iniş yeri bulunamadı",
          `${FAILSAFE.landingSearch} m içinde · ${obstacleText(site.obstacle)}`,
        );
    }
    const mode = guidance.mode,
      phase = guidance.missionPhase(t),
      // The mission can still be interrupted: climbing out, en route,
      // braking or holding. A failure during the take-off aborts it.
      actionable =
        ["mission", "brake", "hold", "resume"].includes(mode) &&
        (mode === "hold" || phase === "takeoff" || EN_ROUTE.has(phase)),
      final = mode === "rtl" || mode === "land";
    // GNSS: the fix is declared lost after the timeout without one.
    if (!navigation.gnssAvailable) monitor.gnssLostAt ??= t;
    else if (monitor.gnssLostAt !== null) {
      monitor.gnssLostAt = null;
      monitor.gnssBackAt = t;
    }
    const gnssLost =
      monitor.gnssLostAt !== null && t - monitor.gnssLostAt >= FAILSAFE.gnssTimeout;
    // Battery: remaining charge against a direct return with the reserve.
    // The measured shaft power, divided by the tilt factor of the drag at
    // the current estimated airspeed, gives the hover-equivalent power the
    // return prediction scales.
    const p = navigation.position,
      airspeed = Math.hypot(
        navigation.velocity[0] - windEstimate()[0],
        navigation.velocity[1] - windEstimate()[1],
      ),
      drag = dragForce(airspeed, c),
      shaft =
        ((power - battery.avionicsPower) * battery.propulsionEfficiency) /
        (1 + (drag / (c.mass * c.gravity)) ** 2) ** 0.75;
    monitor.hoverShaft =
      monitor.hoverShaft === null || t < 5
        ? shaft
        : monitor.hoverShaft +
          (shaft - monitor.hoverShaft) *
            (1 - Math.exp(-1 / (c.controlHz * 20)));
    const target = guidance.returnTarget(p),
      need = returnCharge({
      p: [p[0] - target.xy[0], p[1] - target.xy[1], p[2]],
      wind: windEstimate(),
      hoverShaft: monitor.hoverShaft,
      voltage: pack.voltage,
      homeHeight: reference.endHeight,
      speed: Math.hypot(navigation.velocity[0], navigation.velocity[1]),
      settings,
      battery,
      airframe: c,
    });
    // Motors: low-passed speed excess of each rotor over the others.
    const w = state.subarray(13, 17),
      total = w[0] + w[1] + w[2] + w[3],
      gain = 1 - Math.exp(-1 / (c.controlHz * FAILSAFE.motorFilter));
    let worst = 0,
      worstMotor = 0;
    for (let i = 0; i < 4; i++) {
      const others = (total - w[i]) / 3,
        excess = others > 50 ? w[i] / others - 1 : 0;
      monitor.imbalance[i] += (excess - monitor.imbalance[i]) * gain;
      if (monitor.imbalance[i] > worst) {
        worst = monitor.imbalance[i];
        worstMotor = i;
      }
    }
    // Airborne and armed only: pressed onto uneven ground or spinning down
    // after the land detector the rotors differ for no fault.
    if (
      worst > FAILSAFE.motorImbalance &&
      state[2] > surfaceAt(state[0], state[1]) + 2 &&
      controller.disarmTime === null
    )
      monitor.imbalanceSince ??= t;
    else monitor.imbalanceSince = null;
    if (
      !monitor.motorFlagged &&
      monitor.imbalanceSince !== null &&
      t - monitor.imbalanceSince >= FAILSAFE.motorPersistence
    ) {
      monitor.motorFlagged = true;
      log(
        t,
        "failsafe",
        "Motor dengesizliği algılandı",
        `M${worstMotor + 1} diğerlerinden ${pct(worst)} hızlı dönüyor`,
      );
    }
    // Operator commands take effect at their time; once the operator has
    // commanded, only the critical battery failsafe still acts on its own.
    for (const order of orders)
      if (!order.done && t >= order.time) {
        order.done = true;
        operate(t, order.action);
      }
    // A return or a field landing in progress is final.
    if (final) return;
    // Critical charge: land where the vehicle is, unless it already is
    // landing at the end of the mission.
    if (soc <= FAILSAFE.batteryCritical && !LANDING_PHASES.has(phase)) {
      if (request(t, "land")) {
        log(t, "failsafe", "FAILSAFE · kritik batarya", `şarj ${pct(soc)} · yerinde iniş`);
        banner(t, "kritik batarya · yerinde iniş");
      }
      return;
    }
    if (!actionable || monitor.operator) return;
    // Low battery or a failing motor: return home, or land without GNSS.
    const reason =
      soc - need <= battery.reserve
        ? "battery"
        : monitor.motorFlagged
          ? "motor"
          : null;
    if (reason) {
      const action = gnssLost ? "land" : "rtl";
      if (request(t, action)) {
        monitor.returnReason = reason;
        // Predicted return charge (with the safety factor) at the decision.
        monitor.returnNeed = need;
        log(
          t,
          "failsafe",
          action === "land"
            ? "FAILSAFE · yerinde iniş"
            : phase === "takeoff"
              ? "FAILSAFE · kalkış iptal, eve iniş"
              : "FAILSAFE · eve dönüş (RTL)",
          reason === "battery"
            ? `şarj ${pct(soc)} · dönüş için ${pct(need)} + rezerv ${pct(battery.reserve)}`
            : "motor arızasıyla görev kesildi",
        );
        banner(
          t,
          `${reason === "battery" ? "düşük batarya" : "motor arızası"}${action === "rtl" ? "" : " · GNSS yok · yerinde iniş"}`,
        );
      }
      return;
    }
    // GNSS lost: hold on dead reckoning; land if it does not come back.
    if (gnssLost && request(t, "hold")) {
      monitor.holdReason = "gnss";
      monitor.holdSince = t;
      log(t, "failsafe", "FAILSAFE · konum tutma", "GNSS yok · IMU ölü hesap + baro");
      banner(t, "GNSS yok · ölü hesap");
      return;
    }
    if (monitor.holdReason === "gnss") {
      if (gnssLost && t - monitor.holdSince >= FAILSAFE.holdTimeout) {
        if (request(t, "land")) {
          log(
            t,
            "failsafe",
            "FAILSAFE · yerinde iniş",
            `GNSS ${FAILSAFE.holdTimeout} s içinde dönmedi`,
          );
          banner(t, "GNSS yok · yerinde iniş");
        }
      } else if (
        mode === "hold" &&
        monitor.gnssLostAt === null &&
        t - monitor.gnssBackAt >= FAILSAFE.gnssSettle &&
        guidance.resume(t)
      ) {
        monitor.holdReason = null;
        monitor.action = null;
        log(t, "info", "Göreve devam", "GNSS fix'i oturdu");
        banner(t, null);
      }
    }
  }
  function operate(t, action) {
    const label = OPERATOR_COMMANDS[action];
    let applied;
    if (action === "resume") {
      applied = guidance.resume(t);
      if (applied) {
        monitor.action = null;
        monitor.holdReason = null;
        banner(t, null);
      }
    } else {
      applied = guidance.stop(t, action);
      if (applied) {
        monitor.action = action;
        if (action === "hold") monitor.holdReason = "operator";
        banner(t, "operatör komutu");
      }
    }
    monitor.operator = true;
    log(
      t,
      "operator",
      `OPERATÖR · ${label}`,
      applied ? "komut uygulandı" : "uygulanamadı: araç iniyor ya da beklemede değil",
    );
  }
  // Escalating failsafe actions: a hold may become a return or a landing,
  // a return a landing, never the other way.
  const RANK = { hold: 1, rtl: 2, land: 3 };
  // The autopilot's own landings move clear of mapped obstacles.
  function request(t, action) {
    if (RANK[action] <= (RANK[monitor.action] ?? 0)) return false;
    if (!guidance.stop(t, action, { avoid: true, uncertainty: navigation.eph }))
      return false;
    monitor.action = action;
    return true;
  }
  function record(t, ref) {
    if ((rows + 1) * STRIDE > data.length) {
      const grown = new Float64Array(Math.ceil(data.length * 1.5));
      grown.set(data);
      data = grown;
    }
    const offset = rows++ * STRIDE,
      { thrust, torque } = wrench(Array.from(state.slice(13, 17)), plant);
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
    // The landing reference continues below the surface; on the ground the
    // tracking error is measured against the reference clipped to it.
    data[offset + 29] = Math.hypot(
      state[0] - ref.p[0],
      state[1] - ref.p[1],
      state[2] - Math.max(guidance.surface, ref.p[2]),
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
    data[offset + 44] = windEstimate()[0];
    data[offset + 45] = windEstimate()[1];
  }
  record(0, guidance.at(0));
  for (let step = 0; step < steps(); step++) {
    const t = step * dt;
    if (t > reference.duration + MAX_EXTENSION)
      throw new Error("Failsafe uçuşu makul sürede bitmedi.");
    if (step % controlEvery === 0 && (fault || failsafe)) supervise(t);
    const ref = guidance.at(t);
    if (step % controlEvery === 0) {
      for (let i = 0; i < 3; i++) {
        imuAcceleration[i] = (state[3 + i] - imuVelocity[i]) * c.controlHz;
        imuVelocity[i] = state[3 + i];
      }
      {
        // Body rotation since the last control step, conj(q₀) ⊗ q₁.
        const [aw, ax, ay, az] = imuAttitude,
          bw = state[6],
          bx = state[7],
          by = state[8],
          bz = state[9];
        let dw = aw * bw + ax * bx + ay * by + az * bz,
          dx = aw * bx - ax * bw - ay * bz + az * by,
          dy = aw * by + ax * bz - ay * bw - az * bx,
          dz = aw * bz - ax * by + ay * bx - az * bw;
        if (dw < 0) {
          dw = -dw;
          dx = -dx;
          dy = -dy;
          dz = -dz;
        }
        const sine = Math.hypot(dx, dy, dz),
          scale = sine > 1e-12 ? (2 * Math.atan2(sine, dw) * c.controlHz) / sine : 2 * c.controlHz;
        imuRates[0] = dx * scale;
        imuRates[1] = dy * scale;
        imuRates[2] = dz * scale;
        imuAttitude.set(state.subarray(6, 10));
      }
      // Drag fusion runs while airborne: it estimates the wind with GNSS
      // and holds the velocity without it.
      const rotorRatio =
        (state[13] + state[14] + state[15] + state[16]) / hoverRotors(c);
      navigation.step(
        [state[0], state[1], state[2]],
        [state[3], state[4], state[5]],
        imuAcceleration,
        1 / c.controlHz,
        state.subarray(6, 10),
        imuRates,
        state[2] > surfaceAt(state[0], state[1]) + 2 ? { rotorRatio } : null,
      );
      output = control(state, ref, controller, 1 / c.controlHz, t, c, {
        position: navigation.position,
        velocity: navigation.velocity,
        attitude: navigation.ideal ? undefined : navigation.attitude,
        rates: navigation.ideal ? undefined : navigation.rates,
        maxRotorSpeed: c.maxRotorSpeed * voltageAuthority(pack.voltage, battery),
        // Without GNSS the dead-reckoned velocity wanders; the land detector
        // then accepts PX4's default horizontal speed limit.
        landSpeed: navigation.gnssAvailable ? 0.25 : 1.5,
        wind: windEstimate(),
      });
      commands = output.commands;
      // Wind from the drag the airframe feels, once clear of the ground.
      if (navigation.ideal && state[2] > surfaceAt(state[0], state[1]) + 2)
        windEstimator.step(
          navigation.velocity,
          imuAcceleration,
          navigation.ideal ? state.subarray(6, 10) : navigation.attitude,
          wrench(state.subarray(13, 17), c).thrust,
          1 / c.controlHz,
          (state[13] + state[14] + state[15] + state[16]) / hoverRotors(c),
        );
      // Airborne only: resting on the ground, a dead-reckoned estimate keeps
      // wandering without any consequence.
      if (state[2] > surfaceAt(state[0], state[1]) + 0.5) {
        metrics.maxNavigationError = Math.max(
          metrics.maxNavigationError,
          Math.hypot(
            navigation.position[0] - state[0],
            navigation.position[1] - state[1],
            navigation.position[2] - state[2],
          ),
        );
        metrics.maxEph = Math.max(metrics.maxEph, navigation.eph);
        if (!navigation.ideal) {
          const error = navigation.attitudeError(state.subarray(6, 10));
          metrics.maxTiltError = Math.max(metrics.maxTiltError, error.tilt);
          metrics.maxHeadingError = Math.max(metrics.maxHeadingError, error.yaw);
        }
      }
    }
    const before = state;
    plant.groundLevel = groundUnder(state[0], state[1]);
    state = rk4Step(state, commands, t, dt, plant, gust.vector);
    // Unilateral ground contact (the pad, or the elevation model's ground
    // after a field landing): normal impulse + Coulomb friction. No snapping
    // to the home XY or prescribed orientation; residual landing error is
    // retained.
    const surface = surfaceAt(state[0], state[1]);
    if (state[2] < surface) {
      if (!contact && LANDING_PHASES.has(ref.phase)) {
        metrics.contactSpeed = Math.max(metrics.contactSpeed, -state[5]);
        metrics.contactTime ??= t + dt;
      }
      const normalImpulse = Math.max(0, -state[5]) * c.mass;
      state[2] = surface;
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
      // Landing gear on a rigid, level surface: pressed onto it the
      // airframe cannot roll or pitch, and gear friction stops any
      // rotation. Yaw is kept.
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
    // Vortex ring exposure along the rotor axis (level attitude assumed for
    // the split; the physics resolves it exactly).
    if (state[2] > plant.groundLevel + 1 && state[13] > 50) {
      const vh = Math.sqrt(
          (kf(plant) * (state[13] ** 2 + state[14] ** 2 + state[15] ** 2 + state[16] ** 2)) /
            4 /
            (2 * plant.airDensity * Math.PI * plant.rotorRadius ** 2),
        ),
        descent = gust.vector[2] - state[5],
        inPlane = Math.hypot(state[3] - gust.vector[0], state[4] - gust.vector[1]);
      metrics.maxVrsLoss = Math.max(
        metrics.maxVrsLoss,
        vortexRingLoss(descent, inPlane, vh, plant),
      );
      if (inPlane < 0.5 * vh)
        metrics.maxSlowDescent = Math.max(metrics.maxSlowDescent, descent);
    }
    distance += Math.hypot(state[0] - px, state[1] - py, state[2] - pz);
    px = state[0];
    py = state[1];
    pz = state[2];
    metrics.maxTrackingError = Math.max(
      metrics.maxTrackingError,
      Math.hypot(
        before[0] - ref.p[0],
        before[1] - ref.p[1],
        before[2] - Math.max(guidance.surface, ref.p[2]),
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
    power = electricalPower(shaftPower(state.subarray(13, 17), plant), battery);
    pack = batteryTerminal(power, soc, battery);
    soc = dischargeStep(soc, pack.current, dt, battery);
    if (soc <= 0)
      throw new Error(
        "Batarya görev bitmeden tükeniyor; alanı küçültün, bindirmeyi azaltın veya kütleyi düşürün.",
      );
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
    if ((step + 1) % recordEvery === 0 || step === steps() - 1)
      record((step + 1) * dt, guidance.at((step + 1) * dt));
    if (step % 2400 === 0) onProgress(Math.min(1, step / steps()));
  }
  const count = steps(),
    landing = guidance.landing;
  metrics.landingError = Math.hypot(
    state[0] - landing.xy[0],
    state[1] - landing.xy[1],
  );
  metrics.landingSite = landing.site;
  metrics.landingName = landing.name ?? null;
  // Metres a failsafe landing moved clear of an obstacle, and the obstacle.
  metrics.landingMoved = landing.moved ?? 0;
  metrics.landingObstacle = landing.obstacle ?? null;
  metrics.homeDistance = Math.hypot(state[0], state[1]);
  metrics.landingPosition = [state[0], state[1]];
  metrics.finalSpeed = norm(Array.from(state.slice(3, 6)));
  metrics.disarmed =
    controller.disarmTime !== null && Math.max(...state.slice(13, 17)) < 1;
  metrics.disarmTime = controller.disarmTime;
  metrics.finalSoc = soc;
  // A field landing or a landing on dead reckoning cannot be judged against
  // the pad; it must still end disarmed and at rest.
  const padLanding = landing.site === "home" && navigation.gnssAvailable,
    settled =
      metrics.disarmed &&
      !(padLanding && metrics.landingError > 5) &&
      metrics.finalSpeed <= 0.05;
  // A pack driven to its power limit or nearly empty, or rotors saturated
  // for lack of authority, explain a landing that did not complete (the
  // checklist then rejects the plan); otherwise it is a controller failure.
  metrics.landingCompleted = settled;
  const exhausted =
    metrics.batteryLimitedSeconds > 0 || soc <= 0.05 || metrics.saturationSeconds > 0.5;
  if (!settled && !exhausted)
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
    failsafe: {
      armed: failsafe,
      fault: fault && { ...fault, duration: fault.duration ?? null },
      faults: faults.map((f) => ({
        type: f.type,
        time: f.time,
        label: f.label,
        duration: f.duration ?? null,
        loss: f.loss ?? null,
        motor: f.motor ?? null,
        effectiveness: f.effectiveness ?? null,
      })),
      commands: orders.map(({ time, action }) => ({ time, action })),
      operator: monitor.operator,
      events,
      status,
      transitions: guidance.transitions,
      rtl: guidance.rtl,
      landing,
      returnReason: monitor.returnReason,
      returnNeed: monitor.returnNeed,
    },
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
    windEstimate: row.slice(44, 46),
  };
}
