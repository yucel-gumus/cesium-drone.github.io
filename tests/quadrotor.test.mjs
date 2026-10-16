import test from "node:test";
import assert from "node:assert/strict";
import {
  AIRFRAME,
  initialState,
  derivative,
  rk4Step,
  wrench,
  allocate,
  validateAirframe,
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
