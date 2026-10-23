import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import * as C from "cesium";
import { createTerrainRoute, buildMission, FLIGHT_DEFAULTS } from "../src/flight-planner.mjs";
import {
  simulate,
  replayAt,
  meanWind,
  derivative,
  initialState,
  windLimit,
  dragForce,
  hoverRotors,
  AIRFRAME,
  WIND_REFERENCE,
} from "../src/quadrotor.mjs";
import { makeWindEstimator } from "../src/wind-estimator.mjs";
import { returnCharge } from "../src/failsafe.mjs";
import { BATTERY } from "../src/battery.mjs";
import { toCSV, CSV_HEADER } from "../src/exports.mjs";

const data = JSON.parse(
  fs.readFileSync(new URL("../public/points.json", import.meta.url)),
);
const near = (a, b, t = 1e-6) =>
  assert.ok(Math.abs(a - b) <= t, `${a} != ${b} (±${t})`);
const airframe = (mass) => ({
  ...AIRFRAME,
  mass,
  inertia: AIRFRAME.inertia.map((x) => (x * mass) / 2),
});
// The waypoint loop over level ground for a given mean wind.
const flat = (wind) => {
  const route = createTerrainRoute(C, data, { wind });
  route.samples.forEach((p) => (p.height = 1150));
  return buildMission(C, route);
};

test("wind authority limit: calibrated to 3.5 m/s at 2 kg, grows with mass, 0.1 m/s steps", () => {
  assert.equal(windLimit(airframe(WIND_REFERENCE.mass)), WIND_REFERENCE.wind);
  near(windLimit(airframe(1.5)), 2, 1e-12);
  let previous = -1;
  for (let mass = 1; mass <= 3.001; mass += 0.1) {
    const limit = windLimit(airframe(mass));
    assert.ok(limit > previous, `${mass} kg`);
    near(limit * 10, Math.round(limit * 10), 1e-9);
    previous = limit;
  }
  assert.equal(windLimit(airframe(50)), 6);
  // The drag the formula uses is the one the physics integrates: level, at
  // hover rotor speeds, moving east through still air, the airframe
  // decelerates by drag / mass.
  const state = initialState(0),
    hover = hoverRotors(AIRFRAME) / 4;
  state[3] = 9;
  state.fill(hover, 13, 17);
  const out = derivative(state, [hover, hover, hover, hover], 0, AIRFRAME, 0);
  near(out[3], -dragForce(9) / AIRFRAME.mass, 1e-12);
  near(out[5], 0, 1e-9);
});

test("wind estimate: the EKF's wind states track a steady wind and its gusts; the standalone estimator inverts the drag model", () => {
  const mission = flat(3),
    ref = mission.reference,
    truth = meanWind(3);
  const error = (sim) => {
    let worst = 0;
    for (let t = ref.surveyStart + 60; t < ref.surveyStart + ref.horizontalDuration; t += 10) {
      const e = replayAt(sim, t).windEstimate;
      worst = Math.max(worst, Math.hypot(e[0] - truth[0], e[1] - truth[1]));
    }
    return worst;
  };
  // Estimated attitude (about 0.5° of tilt error) sets the floor: a tilt
  // error looks like drag.
  assert.ok(error(simulate(ref, { wind: 3, turbulence: false })) < 0.5);
  // Dryden gusts (σ_u ≈ 0.42 m/s at 3 m/s) are followed, not averaged.
  assert.ok(error(simulate(ref, { wind: 3 })) < 1.5);
  // With ideal navigation the standalone estimator is exact.
  assert.ok(error(simulate(ref, { wind: 3, turbulence: false, gnss: "ideal" })) < 0.05);
  // Standalone: a vehicle hovering in a 4 m/s easterly wind holds still
  // against the drag, tilted into the wind.
  const estimator = makeWindEstimator(AIRFRAME),
    drag = dragForce(4),
    tilt = Math.atan2(drag, AIRFRAME.mass * AIRFRAME.gravity),
    thrust = Math.hypot(drag, AIRFRAME.mass * AIRFRAME.gravity),
    // Rotation about north (+y) by −tilt leans the thrust towards −east.
    q = [Math.cos(-tilt / 2), 0, Math.sin(-tilt / 2), 0];
  for (let i = 0; i < 200; i++)
    estimator.step([0, 0, 0], [0, 0, 0], q, thrust, 0.1);
  // The hover balance above uses level-flight drag; the estimator resolves
  // the rotor drag in the tilted rotor plane: within a centimetre a second.
  near(estimator.wind[0], 4, 0.01);
  near(estimator.wind[1], 0, 1e-9);
});

test("drag feed-forward from the wind estimate keeps light airframes on track, and the limit marks the authority edge", () => {
  const calm = flat(0);
  const light = simulate(calm.reference, { airframe: airframe(1), wind: 0 });
  // Without feed-forward the 1 kg airframe trailed by about a metre.
  assert.ok(light.metrics.maxTrackingError < 0.3, `${light.metrics.maxTrackingError}`);
  for (const mass of [1.5, 3]) {
    const limit = windLimit(airframe(mass)),
      at = simulate(flat(limit).reference, { airframe: airframe(mass), wind: limit, seed: 3 });
    assert.ok(at.metrics.maxTrackingError < 1, `${mass} kg at ${limit}: ${at.metrics.maxTrackingError}`);
    const beyond = limit + 1,
      over = simulate(flat(beyond).reference, { airframe: airframe(mass), wind: beyond, seed: 5 });
    assert.ok(over.metrics.maxTrackingError > 3, `${mass} kg at ${beyond}: ${over.metrics.maxTrackingError}`);
  }
});

test("return charge prices the estimated wind along the leg home", () => {
  const base = {
    p: [1000, 0, 80],
    hoverShaft: 210,
    voltage: 22,
    settings: FLIGHT_DEFAULTS,
    battery: BATTERY,
    airframe: AIRFRAME,
  };
  const calm = returnCharge(base),
    // Home is west: an easterly-blowing wind is a headwind on the way back.
    headwind = returnCharge({ ...base, wind: [4, 0] }),
    tailwind = returnCharge({ ...base, wind: [-4, 0] });
  assert.ok(headwind > calm && calm > tailwind, `${headwind} ${calm} ${tailwind}`);
  // Twice the distance, more charge; over home only the descent and the
  // landing remain.
  assert.ok(returnCharge({ ...base, p: [2000, 0, 80] }) > calm);
  const overHome = returnCharge({ ...base, p: [0, 0, 80] });
  assert.ok(overHome > 0 && overHome < calm / 2, `${overHome} ${calm}`);
});

test("CSV carries the wind estimate", () => {
  const mission = flat(2);
  mission.simulation = simulate(mission.reference, { wind: 2, turbulence: false });
  const lines = toCSV(mission).split("\n"),
    header = lines[0].split(",");
  assert.deepEqual(header.slice(-2), ["wind_est_e_ms", "wind_est_n_ms"]);
  assert.equal(header.length, CSV_HEADER.length);
  const row = lines[Math.floor(lines.length / 2)].split(",").map(Number);
  near(row.at(-2), meanWind(2)[0], 0.5);
  near(row.at(-1), meanWind(2)[1], 0.5);
});
