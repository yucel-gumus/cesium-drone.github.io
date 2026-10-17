import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import * as C from "cesium";
import {
  createTerrainRoute,
  sampleTerrainRoute,
  buildMission,
  attachSimulation,
  evaluateMission,
  validateWaypoints,
  validateBounds,
  validateGeoidUndulation,
} from "../src/flight-planner.mjs";
import { referenceAt } from "../src/trajectory.mjs";
import { simulate, STRIDE } from "../src/quadrotor.mjs";
import { dot, norm, sub, polynomial, rotate } from "../src/flight-math.mjs";
const data = JSON.parse(
  fs.readFileSync(new URL("../public/points.json", import.meta.url)),
);
const route = createTerrainRoute(C, data);
route.samples.forEach((p) => (p.height = 1150));
const mission = buildMission(C, route);
const simulation = simulate(mission.reference);
attachSimulation(mission, simulation);
const near = (a, b, t = 1e-6) =>
  assert.ok(Math.abs(a - b) <= t, `${a} != ${b}`);

test("validates region, missing/null coordinates, closed and degenerate routes", () => {
  assert.deepEqual(validateWaypoints(data)[0], validateWaypoints(data).at(-1));
  assert.deepEqual(
    validateWaypoints({
      ...data,
      dronePoints: data.dronePoints.slice(0, -1),
    }).at(-1),
    validateWaypoints(data)[0],
  );
  for (const bad of [
    null,
    { longitude: NaN, latitude: 40.62 },
    { longitude: 0, latitude: 0 },
  ])
    assert.throws(() =>
      validateWaypoints({ ...data, dronePoints: [bad, ...data.dronePoints] }),
    );
  // Region limits come from the data file, so messages name no place.
  assert.throws(
    () =>
      validateWaypoints({
        ...data,
        dronePoints: [{ longitude: 0, latitude: 0 }, ...data.dronePoints],
      }),
    /^Error: Rota noktası 1: rota sınırlarının dışında\.$/,
  );
  assert.throws(() =>
    createTerrainRoute(C, {
      ...data,
      dronePoints: Array(3).fill(data.dronePoints[0]),
    }),
  );
  assert.throws(() => createTerrainRoute(C, data, { wind: NaN }));
});

test("route bounds are mandatory, ordered, on the globe and small enough for one ENU frame", () => {
  const { bounds } = data;
  assert.deepEqual(validateBounds(data), bounds);
  assert.deepEqual(createTerrainRoute(C, data).bounds, bounds);
  const { bounds: _omitted, ...withoutBounds } = data;
  assert.throws(() => validateWaypoints(withoutBounds), /bounds/);
  for (const bad of [
    null,
    "40.25,40.35,40.59,40.65",
    { ...bounds, north: undefined },
    { ...bounds, east: NaN },
    { ...bounds, west: bounds.east, east: bounds.west },
    { ...bounds, south: bounds.north },
    { ...bounds, west: -181 },
    { ...bounds, north: 90.5 },
    { ...bounds, east: bounds.west + 0.51 },
    { ...bounds, south: bounds.north - 0.6 },
  ])
    assert.throws(
      () => validateWaypoints({ ...data, bounds: bad }),
      /^Error: Rota (dosyasında bounds|sınırları)/,
    );
  // A box at the 0.5° limit is accepted; the same points elsewhere are not.
  const limit = {
    ...bounds,
    east: bounds.west + 0.5,
    north: bounds.south + 0.5,
  };
  assert.equal(validateWaypoints({ ...data, bounds: limit }).length, 18);
  // 1.1 − 0.6 is 0.5000000000000001 in floating point: still at the limit.
  const noisy = { west: 0.6, east: 1.1, south: 40, north: 40.4 };
  assert.deepEqual(validateBounds({ bounds: noisy }), noisy);
  assert.throws(
    () =>
      validateWaypoints({
        ...data,
        bounds: { west: 29.0, east: 29.5, south: 41.0, north: 41.5 },
      }),
    /rota sınırlarının dışında/,
  );
});

test("terrain sampling covers the rounded path and both sides, not the old polygon", () => {
  assert.equal(route.samples.length, route.centers.length * 3);
  assert.ok(route.path.length < 4897.5 && route.path.length > 4500);
  const distances = route.centers
    .slice(1)
    .map(
      (x, i) => route.path.table[x].s - route.path.table[route.centers[i]].s,
    );
  assert.ok(Math.max(...distances) <= 6.01);
  assert.ok(route.path.pieces.some((p) => p.length === 6));
});

test("reference is C2 at every knot and phase boundary", () => {
  const ref = mission.reference;
  for (let i = 1; i < ref.horizontal.length; i++) {
    const a = ref.horizontal[i - 1],
      b = ref.horizontal[i];
    for (let axis = 0; axis < 3; axis++) {
      const left = polynomial(a.coefficients[axis], 1, a.duration),
        right = polynomial(b.coefficients[axis], 0, b.duration);
      for (const key of ["p", "v", "a"]) near(left[key], right[key], 1e-6);
    }
  }
  for (const phase of ref.phases.slice(1)) {
    const a = referenceAt(ref, phase.startTime - 1e-6),
      b = referenceAt(ref, phase.startTime + 1e-6);
    assert.ok(norm(sub(a.p, b.p)) < 1e-4);
    assert.ok(norm(sub(a.v, b.v)) < 1e-4);
    assert.ok(norm(sub(a.a, b.a)) < 1e-4);
  }
});

test("reference speed, acceleration and jerk bounds hold across all polynomials", () => {
  const s = mission.settings;
  for (const seg of mission.reference.horizontal)
    for (let i = 0; i <= 40; i++) {
      const p = seg.coefficients.map((c) =>
        polynomial(c, i / 40, seg.duration),
      );
      assert.ok(Math.hypot(p[0].v, p[1].v) <= s.cruiseSpeed + 1e-5);
      assert.ok(Math.hypot(p[0].a, p[1].a) <= s.acceleration + 1e-5);
      assert.ok(Math.hypot(p[0].j, p[1].j) <= s.jerk + 1e-5);
    }
});

test("fly-through bends retain speed and have actual bank instead of yaw teleportation", () => {
  let bends = 0,
    maxAngularStep = 0;
  for (let i = 1; i < simulation.rows; i++) {
    const r = i * STRIDE,
      p = (i - 1) * STRIDE;
    const q = Array.from(simulation.data.slice(r + 7, r + 11)),
      previous = Array.from(simulation.data.slice(p + 7, p + 11));
    const angle = 2 * Math.acos(Math.min(1, Math.abs(dot(q, previous))));
    maxAngularStep = Math.max(maxAngularStep, angle);
    if (simulation.data[r + 30] === 2) {
      const time = simulation.data[r],
        ref = referenceAt(mission.reference, time);
      if (Math.abs(ref.yawRate) > 0.06) {
        bends++;
        assert.ok(
          Math.hypot(simulation.data[r + 4], simulation.data[r + 5]) > 0.4,
        );
      }
    }
  }
  assert.ok(bends > 100);
  assert.ok(maxAngularStep < 0.025);
  assert.ok(
    simulation.metrics.maxTilt > 0.05 &&
      simulation.metrics.maxTilt < (25 * Math.PI) / 180,
  );
});

test("full force-driven flight tracks within one metre and lands by contact, without snapping home", () => {
  const m = simulation.metrics,
    s = mission.settings;
  assert.ok(m.maxTrackingError < 1);
  // RTK navigation: centimetre landing, never an exact copy of home.
  assert.ok(m.landingError < 0.1 && m.landingError > 0.001);
  // Constant-speed final descent until the land detector fires.
  assert.ok(Math.abs(m.contactSpeed - s.landingSpeed) < 0.1);
  assert.equal(m.disarmed, true);
  // Disarm follows the first pad contact by less than two seconds.
  let contact = Infinity;
  for (let i = 0; i < simulation.rows && contact === Infinity; i++) {
    const r = i * STRIDE;
    if (simulation.data[r + 30] >= 6 && simulation.data[r + 3] <= 0.002)
      contact = simulation.data[r];
  }
  assert.ok(m.disarmTime >= contact && m.disarmTime - contact < 2);
  near(m.finalSpeed, 0);
  assert.ok(m.maxRotorSpeed <= simulation.airframe.maxRotorSpeed + 1e-7);
  near(m.saturationSeconds, 0);
  assert.ok(mission.minClearance >= s.agl - s.clearanceTolerance);
  assert.ok(mission.maxAgl <= s.maxAgl);
  assert.ok(mission.maxCorridorDeviation < s.corridorHalfWidth - 2);
});

test("recorded airborne acceleration obeys Newton force balance and turns have roll", () => {
  const c = simulation.airframe;
  let banked = 0;
  for (let i = 1; i < simulation.rows - 1; i += 3) {
    const r = i * STRIDE;
    if (simulation.data[r + 3] < 2) continue;
    const q = Array.from(simulation.data.slice(r + 7, r + 11)),
      v = Array.from(simulation.data.slice(r + 4, r + 7)),
      wind = Array.from(simulation.data.slice(r + 25, r + 28));
    const relative = sub(v, wind),
      dragFactor =
        -(0.5 * c.airDensity * c.dragArea * norm(relative) + c.linearDrag) /
        c.mass;
    const predicted = rotate(q, [0, 0, simulation.data[r + 21] / c.mass]).map(
      (a, j) => a + dragFactor * relative[j] - (j === 2 ? c.gravity : 0),
    );
    const dt =
      simulation.data[(i + 1) * STRIDE] - simulation.data[(i - 1) * STRIDE];
    const measured = [0, 1, 2].map(
      (j) =>
        (simulation.data[(i + 1) * STRIDE + 4 + j] -
          simulation.data[(i - 1) * STRIDE + 4 + j]) /
        dt,
    );
    assert.ok(norm(sub(predicted, measured)) < 0.08);
    const [w, x, y, z] = q,
      roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    const ref = referenceAt(mission.reference, simulation.data[r]);
    if (Math.abs(ref.yawRate) > 0.06 && Math.abs(roll) > 0.01) banked++;
  }
  assert.ok(banked > 50);
});

test("replay preserves measured velocity and quaternion at pause/seek instead of inventing speed", () => {
  const t = mission.reference.surveyStart + 123.4,
    dt = 0.001;
  const a = evaluateMission(C, mission, t - dt),
    b = evaluateMission(C, mission, t + dt),
    s = evaluateMission(C, mission, t);
  assert.ok(
    norm(
      sub(
        sub(b.p, a.p).map((v) => v / (2 * dt)),
        s.v,
      ),
    ) < 0.005,
  );
  near(norm(s.q), 1, 1e-10);
  const end = evaluateMission(C, mission, mission.duration - 1e-10);
  assert.equal(end.phase, "complete");
  near(end.groundSpeed, 0);
  assert.ok(Math.hypot(end.p[0], end.p[1]) < 0.1);
  assert.deepEqual(evaluateMission(C, mission, t).q, s.q);
});

test("ENU state transforms correctly to ellipsoid and AGL references", () => {
  const s = evaluateMission(C, mission, mission.reference.surveyStart + 200);
  near(s.agl, s.position.height - s.terrainHeight, 1e-7);
  near(s.relativeAltitude, s.position.height - mission.groundHeight);
  const initial = evaluateMission(C, mission, 0);
  near(initial.agl, mission.settings.landingOffset, 1e-6);
  const at = C.Cartographic.fromCartesian(s.cartesian);
  near(at.longitude, s.position.longitude, 1e-12);
});

test("missing terrain blocks planning and service failures propagate", async () => {
  const bad = createTerrainRoute(C, data);
  bad.samples[0].height = undefined;
  assert.throws(() => buildMission(C, bad));
  await assert.rejects(
    sampleTerrainRoute(
      {
        sampleTerrainMostDetailed: async () => {
          throw new Error("network");
        },
      },
      {},
      bad,
    ),
  );
});

test("clearance is measured against the corridor's highest sampled ground, not the centre line", () => {
  const raised = createTerrainRoute(C, data);
  // Flat centre line with 20 m higher ground on both corridor edges.
  raised.samples.forEach((p, i) => (p.height = i % 3 === 0 ? 1150 : 1170));
  const m = buildMission(C, raised);
  attachSimulation(m, simulate(m.reference));
  const s = m.settings;
  // The profile follows the higher edges, so clearance over the corridor
  // floor sits at the target while the centre line is 20 m further below.
  assert.ok(m.minClearance >= s.agl - s.clearanceTolerance);
  assert.ok(m.minClearance <= s.agl + 1);
  const sample = evaluateMission(C, m, m.reference.surveyStart + 200);
  near(sample.agl, sample.position.height - sample.terrainHeight, 1e-7);
  assert.ok(Math.abs(sample.agl - (s.agl + 20)) < 1.5);
});

test("approximate MSL is ellipsoid height minus the geoid undulation, and optional", () => {
  assert.equal(typeof data.geoidUndulation, "number");
  assert.equal(validateGeoidUndulation(data), data.geoidUndulation);
  assert.equal(route.geoidUndulation, data.geoidUndulation);
  assert.equal(mission.geoidUndulation, data.geoidUndulation);
  for (const t of [0, mission.reference.surveyStart + 200, mission.duration]) {
    const s = evaluateMission(C, mission, t);
    near(s.mslAltitude, s.position.height - data.geoidUndulation, 1e-9);
  }
  // Without N the planner still works and reports no MSL value.
  const { geoidUndulation: _omitted, ...withoutGeoid } = data;
  for (const input of [withoutGeoid, { ...data, geoidUndulation: null }]) {
    assert.equal(validateGeoidUndulation(input), null);
    const r = createTerrainRoute(C, input);
    assert.equal(r.geoidUndulation, null);
    r.samples.forEach((p) => (p.height = 1150));
    const m = attachSimulation(buildMission(C, r), simulation);
    const s = evaluateMission(C, m, m.reference.surveyStart + 200);
    assert.equal(s.mslAltitude, null);
    near(s.relativeAltitude, s.position.height - m.groundHeight);
  }
  for (const bad of [NaN, Infinity, 500, -121, "27.26", {}])
    assert.throws(
      () => createTerrainRoute(C, { ...data, geoidUndulation: bad }),
      /geoidUndulation/,
    );
});
