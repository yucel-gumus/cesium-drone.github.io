import test from "node:test";
import assert from "node:assert/strict";
import {
  AIRFRAME,
  SPIN,
  STRIDE,
  initialState,
  derivative,
  rk4Step,
  wrench,
  allocate,
  validateAirframe,
  makeController,
  control,
  meanWind,
  drydenParameters,
  drydenCoefficients,
  makeTurbulence,
  simulate,
  replayAt,
} from "../src/quadrotor.mjs";
import { rotate, norm, dot } from "../src/flight-math.mjs";
import { roundedPath, makeReference } from "../src/trajectory.mjs";
import { FLIGHT_DEFAULTS } from "../src/flight-planner.mjs";
const near = (a, b, t = 1e-7) =>
  assert.ok(Math.abs(a - b) <= t, `${a} != ${b}`);
const bare = {
  ...AIRFRAME,
  dragArea: 0,
  linearDrag: 0,
  angularDrag: [0, 0, 0],
};
const integrate = (initial, commands, seconds, hz = 240, config = bare) => {
  let s = initial;
  for (let i = 0; i < seconds * hz; i++)
    s = rk4Step(s, commands, i / hz, 1 / hz, config);
  return s;
};

test("zero thrust obeys analytical ballistic free fall", () => {
  const s = initialState();
  s[2] = 100;
  const end = integrate(s, [0, 0, 0, 0], 1);
  near(end[2], 100 - 0.5 * AIRFRAME.gravity, 1e-9);
  near(end[5], -AIRFRAME.gravity, 1e-9);
});

test("balanced rotor thrust supports weight with zero net torque", () => {
  const c = AIRFRAME,
    w = Math.sqrt(
      (c.mass * c.gravity) / ((4 * c.thrustCoefficient * c.airDensity) / 1.225),
    );
  const s = initialState();
  s[2] = 100;
  s.set([w, w, w, w], 13);
  const result = integrate(s, [w, w, w, w], 2, 240, AIRFRAME);
  near(result[2], 100, 1e-9);
  near(norm(Array.from(result.slice(3, 6))), 0, 1e-10);
  assert.ok(wrench([w, w, w, w]).torque.every((t) => Math.abs(t) < 1e-12));
});

test("tilted thrust gives exactly the lateral force and vertical weight balance", () => {
  const theta = 0.2,
    s = initialState();
  s[6] = Math.cos(theta / 2);
  s[8] = Math.sin(theta / 2);
  const thrust = (AIRFRAME.mass * AIRFRAME.gravity) / Math.cos(theta),
    w = Math.sqrt(
      thrust / ((4 * AIRFRAME.thrustCoefficient * AIRFRAME.airDensity) / 1.225),
    );
  s.set([w, w, w, w], 13);
  const d = derivative(s, [w, w, w, w], 0, AIRFRAME, 0);
  near(d[3], AIRFRAME.gravity * Math.tan(theta));
  near(d[5], 0);
});

test("control allocation recovers requested thrust and all three torques", () => {
  for (const torque of [
    [0.2, 0, 0],
    [0, -0.2, 0],
    [0, 0, 0.04],
    [0.15, -0.1, 0.02],
  ]) {
    const a = allocate(20, torque),
      actual = wrench(a.commands);
    assert.equal(a.saturated, false);
    near(actual.thrust, 20);
    torque.forEach((v, i) => near(actual.torque[i], v));
  }
});

test("motor step follows first order lag, not instantaneous actuator motion", () => {
  const s = initialState(),
    T = 0.1,
    result = integrate(s, [500, 500, 500, 500], T, 240);
  const expected = 500 * (1 - Math.exp(-T / AIRFRAME.motorTimeConstant));
  near(result[13], expected, 0.0001);
  assert.ok(result[13] < 500);
  const impossible = allocate(1e5, [100, 100, 100]);
  assert.equal(impossible.saturated, true);
  assert.ok(impossible.commands.every((w) => w <= AIRFRAME.maxRotorSpeed));
});

test("Euler rigid-body coupling conserves rotational energy without torque or damping", () => {
  const s = initialState();
  s[2] = 100;
  s.set([0.4, 0.7, 0.2], 10);
  const energy = (x) =>
    AIRFRAME.inertia.reduce((sum, I, i) => sum + 0.5 * I * x[10 + i] ** 2, 0);
  const result = integrate(s, [0, 0, 0, 0], 2);
  near(energy(s), energy(result), 1e-10);
  near(norm(Array.from(result.slice(6, 10))), 1, 1e-12);
});

test("wind perturbs force and differing rotor forces cause angular acceleration first", () => {
  const s = initialState();
  const still = derivative(s, [0, 0, 0, 0], 3, AIRFRAME, 0),
    wind = derivative(s, [0, 0, 0, 0], 3, AIRFRAME, 3);
  assert.ok(wind[3] > still[3]);
  assert.ok(wind[4] > still[4]);
  const motors = allocate(20, [0, 0, 0.04]).commands;
  s.set(motors, 13);
  const d = derivative(s, motors, 0, AIRFRAME, 0);
  near(d[12], 0.04 / AIRFRAME.inertia[2]);
  near(norm(Array.from(d.slice(6, 10))), 0);
});

test("RK4 converges under smaller time steps and quaternion remains normalized", () => {
  const s = initialState();
  s[2] = 100;
  s.set([520, 500, 510, 505], 13);
  const a = integrate(s, [520, 500, 510, 505], 0.5, 240),
    b = integrate(s, [520, 500, 510, 505], 0.5, 480);
  assert.ok(norm(Array.from(a.slice(0, 17)).map((v, i) => v - b[i])) < 1e-6);
});

test("invalid or inconsistent physics frequencies are rejected", () => {
  assert.throws(() => validateAirframe({ ...AIRFRAME, mass: -1 }));
  assert.throws(() => validateAirframe({ ...AIRFRAME, inertia: [1, 0, 1] }));
  assert.throws(() => validateAirframe({ ...AIRFRAME, controlHz: 113 }));
  assert.throws(() => validateAirframe({ ...AIRFRAME, rotorInertia: 0 }));
  assert.throws(() => makeTurbulence(-1));
  assert.throws(() => makeTurbulence(2, { seed: NaN }));
});

test("wind/payload scenario produces a new causal flight while deterministic reruns agree", () => {
  const path = roundedPath(
    [
      [0, 0],
      [50, 0],
      [50, 50],
      [0, 50],
      [0, 0],
    ],
    15,
    2,
  );
  const ref = makeReference(path, 12, { ...FLIGHT_DEFAULTS, cruiseSpeed: 5 });
  const a = simulate(ref, { wind: 0 }),
    b = simulate(ref, {
      wind: 4,
      airframe: {
        ...AIRFRAME,
        mass: 2.5,
        inertia: AIRFRAME.inertia.map((v) => v * 1.25),
      },
    });
  assert.ok(a.metrics.landingError < 0.2 && b.metrics.landingError < 0.2);
  const t = ref.surveyStart + 20,
    sa = replayAt(a, t),
    sb = replayAt(b, t);
  assert.ok(norm(sa.p.map((v, i) => v - sb.p[i])) > 0.005);
  assert.ok(Math.abs(sa.thrust - sb.thrust) > 2);
  const c = simulate(ref, { wind: 0 });
  assert.deepEqual(a.data, c.data);
});

test("Dryden second-order discretization keeps the stationary covariance ¼I at every step size", () => {
  // The scaled filter state has covariance ¼I in stationarity, so for any
  // x = dt/τ the discrete step must satisfy ¼ Φ Φᵀ + L Lᵀ = ¼ I. This spans
  // the power-series branch (x < 0.02), the closed form and the x = 0 limit.
  for (const x of [0, 1e-4, 1e-3, 0.019, 0.021, 0.2, 1, 4]) {
    const c = drydenCoefficients(x);
    near(0.25 * (c[0] ** 2 + c[1] ** 2) + c[4] ** 2, 0.25, 1e-14);
    near(0.25 * (c[0] * c[2] + c[1] * c[3]) + c[4] * c[5], 0, 1e-14);
    near(0.25 * (c[2] ** 2 + c[3] ** 2) + c[5] ** 2 + c[6] ** 2, 0.25, 1e-14);
  }
});

test("Dryden turbulence is seed-deterministic, holds the MIL-F-8785C intensities and keeps the mean wind", () => {
  const dt = 1 / 240,
    airspeed = 8,
    wind = 2;
  // Low-altitude form at 300 ft: σ_w = 0.1 W20, σ_u = σ_w / 0.4239^0.4.
  const { sigma, scale } = drydenParameters(wind);
  near(sigma[2], 0.2);
  near(sigma[0], 0.2 / (0.177 + 0.000823 * 300) ** 0.4);
  near(sigma[1], sigma[0]);
  near(scale[2], 300 * 0.3048);
  near(scale[0], (300 / (0.177 + 0.000823 * 300) ** 1.2) * 0.3048);
  // Same seed → bit-identical, different seed → different.
  const a = makeTurbulence(wind, { seed: 1 }),
    b = makeTurbulence(wind, { seed: 1 }),
    c = makeTurbulence(wind, { seed: 2 });
  assert.deepEqual(Array.from(a.vector), Array.from(b.vector));
  let differs = false;
  for (let i = 0; i < 1000; i++) {
    const x = Array.from(a.advance(dt, airspeed)),
      z = c.advance(dt, airspeed);
    assert.deepEqual(x, Array.from(b.advance(dt, airspeed)));
    differs ||= x.some((v, k) => v !== z[k]);
  }
  assert.ok(differs);
  // Two hours at 240 Hz flown at 8 m/s airspeed: sample deviation of each
  // component within ±35 % of σ and the mean vector within 0.1 m/s.
  const g = makeTurbulence(wind, { seed: 1 }),
    mean = meanWind(wind),
    east = mean[0] / wind,
    north = mean[1] / wind,
    n = 240 * 7200,
    sum = [0, 0, 0],
    square = [0, 0, 0],
    total = [0, 0, 0];
  for (let i = 0; i < n; i++) {
    const w = g.advance(dt, airspeed),
      de = w[0] - mean[0],
      dn = w[1] - mean[1];
    const u = de * east + dn * north,
      v = -de * north + dn * east;
    [u, v, w[2]].forEach((x, k) => {
      sum[k] += x;
      square[k] += x * x;
    });
    total.forEach((_, k) => (total[k] += w[k]));
  }
  for (let k = 0; k < 3; k++) {
    const m = sum[k] / n,
      deviation = Math.sqrt(square[k] / n - m * m);
    assert.ok(
      Math.abs(deviation - sigma[k]) < 0.35 * sigma[k],
      `component ${k}: ${deviation} vs ${sigma[k]}`,
    );
    assert.ok(Math.abs(total[k] / n - mean[k]) < 0.1);
  }
  // turbulence: false and zero wind are exactly the steady mean vector.
  const steady = makeTurbulence(wind, { turbulence: false });
  assert.equal(steady.active, false);
  for (let i = 0; i < 100; i++)
    assert.deepEqual(Array.from(steady.advance(dt, airspeed)), mean);
  assert.deepEqual(
    Array.from(makeTurbulence(0).advance(dt, airspeed)),
    [0, 0, 0],
  );
});

test("simulate records the turbulent wind, reruns per seed and honours turbulence: false", () => {
  const path = roundedPath(
    [
      [0, 0],
      [40, 0],
      [40, 40],
      [0, 0],
    ],
    15,
    2,
  );
  const ref = makeReference(path, 12, { ...FLIGHT_DEFAULTS, cruiseSpeed: 5 });
  const a = simulate(ref, { wind: 2, seed: 5 }),
    b = simulate(ref, { wind: 2, seed: 5 }),
    c = simulate(ref, { wind: 2, seed: 6 }),
    steady = simulate(ref, { wind: 2, turbulence: false });
  assert.deepEqual(a.data, b.data);
  assert.notDeepEqual(a.data, c.data);
  assert.equal(a.seed, 5);
  assert.equal(a.turbulence, true);
  assert.equal(steady.turbulence, false);
  const mean = meanWind(2);
  let gusty = 0;
  for (let i = 0; i < a.rows; i++) {
    const r = i * STRIDE;
    if (Math.hypot(a.data[r + 25] - mean[0], a.data[r + 26] - mean[1]) > 0.05)
      gusty++;
    for (let k = 0; k < 3; k++)
      assert.equal(steady.data[r + 25 + k], mean[k]);
  }
  assert.ok(gusty > a.rows / 4);
  assert.ok(a.metrics.landingError < 0.2 && steady.metrics.landingError < 0.2);
  // Scalar wind in derivative() is the steady mean vector.
  const s = initialState();
  assert.deepEqual(
    Array.from(derivative(s, [0, 0, 0, 0], 0, AIRFRAME, 3)),
    Array.from(derivative(s, [0, 0, 0, 0], 0, AIRFRAME, meanWind(3))),
  );
});

test("rotor gyroscopic torque is −J_r·SPIN_i·ω_i·Ω_x / I_yy and cancels for a balanced rotor set", () => {
  // A clockwise rotor (SPIN +1) spins about −z, so its angular momentum is
  // h = −J_r ω e_z. Rolling at Ω_x, the body feels −Ω × h =
  // −(Ω_x e_x) × (−J_r ω e_z) = J_r ω Ω_x (e_x × e_z) = −J_r ω Ω_x e_y.
  const i = 1,
    omega = 600,
    rate = 0.8,
    still = { ...bare, rotorInertia: 0 };
  assert.equal(SPIN[i], 1);
  const s = initialState();
  s[2] = 50;
  s[13 + i] = omega;
  s[10] = rate;
  const commands = [0, 0, 0, 0];
  commands[i] = omega; // no spin-up, so no reaction torque
  const d = derivative(s, commands, 0, bare),
    d0 = derivative(s, commands, 0, still);
  const expected =
    (-bare.rotorInertia * SPIN[i] * omega * rate) / bare.inertia[1];
  near(d[11] - d0[11], expected, 1e-12);
  assert.ok(expected < 0);
  near(
    d[11],
    wrench([0, omega, 0, 0]).torque[1] / bare.inertia[1] + expected,
    1e-12,
  );
  near(d[10], d0[10]);
  near(d[12], d0[12]);
  // Pitching at Ω_y instead gives +J_r SPIN ω Ω_y about x.
  s[10] = 0;
  s[11] = rate;
  const p = derivative(s, commands, 0, bare),
    p0 = derivative(s, commands, 0, still);
  near(
    p[10] - p0[10],
    (bare.rotorInertia * SPIN[i] * omega * rate) / bare.inertia[0],
    1e-12,
  );
  near(p[11], p0[11]);
  // Balanced set (Σ SPIN ω = 0) and rotors at rest add nothing.
  s.set([omega, omega, omega, omega], 13);
  s[10] = rate;
  const balanced = derivative(s, [omega, omega, omega, omega], 0, bare),
    balanced0 = derivative(s, [omega, omega, omega, omega], 0, still);
  near(balanced[10], balanced0[10]);
  near(balanced[11], balanced0[11]);
  s.set([0, 0, 0, 0], 13);
  assert.deepEqual(
    Array.from(derivative(s, [0, 0, 0, 0], 0, bare)),
    Array.from(derivative(s, [0, 0, 0, 0], 0, still)),
  );
  // The controller cancels the same term: its torque differs by −τ_gyro.
  const state = initialState();
  state[2] = 50;
  state.set([500, 600, 500, 600], 13);
  state.set([0.5, 0.3, 0], 10);
  const ref = {
    p: [0, 0, 50],
    v: [0, 0, 0],
    a: [0, 0, 0],
    yaw: 0,
    yawRate: 0,
    phase: "survey",
  };
  const spin = bare.rotorInertia * dot(SPIN, Array.from(state.slice(13, 17)));
  const torqueOf = (c) =>
    wrench(control(state, ref, makeController(0), 1 / 120, 0, c).commands, c)
      .torque;
  const withRotor = torqueOf(bare),
    without = torqueOf(still);
  near(withRotor[0] - without[0], -spin * 0.3, 1e-9);
  near(withRotor[1] - without[1], spin * 0.5, 1e-9);
  near(withRotor[2], without[2], 1e-9);
});

test("rotor spin-up reaction torque equals J_r Σ SPIN_i ω̇_i about z", () => {
  const s = initialState();
  s[2] = 50;
  // Clockwise motor 1 commanded from rest: ω̇ = ω_cmd / motorTimeConstant and
  // no aerodynamic torque yet, so the body yaws about +z.
  const d = derivative(s, [0, 600, 0, 0], 0, bare),
    expected =
      (bare.rotorInertia * SPIN[1] * 600) /
      bare.motorTimeConstant /
      bare.inertia[2];
  near(d[12], expected, 1e-12);
  assert.ok(d[12] > 0);
  near(derivative(s, [600, 0, 0, 0], 0, bare)[12], -expected, 1e-12);
  near(derivative(s, [600, 600, 600, 600], 0, bare)[12], 0, 1e-12);
  // Differential step on spinning rotors: aerodynamic plus inertial reaction.
  s.set([500, 500, 500, 500], 13);
  const step = derivative(s, [530, 470, 530, 470], 0, bare),
    rotorDot = [30, -30, 30, -30].map((x) => x / bare.motorTimeConstant);
  near(
    step[12],
    (wrench([500, 500, 500, 500]).torque[2] +
      bare.rotorInertia * dot(SPIN, rotorDot)) /
      bare.inertia[2],
    1e-12,
  );
});
