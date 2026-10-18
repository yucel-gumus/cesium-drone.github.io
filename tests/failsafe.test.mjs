import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import * as C from "cesium";
import {
  createTerrainRoute,
  buildMission,
  attachSimulation,
  terrainForWorker,
  measureFlight,
  failsafeStatus,
} from "../src/flight-planner.mjs";
import { referenceAt } from "../src/trajectory.mjs";
import { simulate, STRIDE, PHASE_NAMES, replayAt } from "../src/quadrotor.mjs";
import {
  FAULTS,
  FAILSAFE,
  validateFault,
  makeGuidance,
  groundHeight,
} from "../src/failsafe.mjs";
import {
  rasterHeight,
  rasterFloor,
  corridorFloor,
  rasterLayout,
  distanceToConvex,
  convexHull,
  indexStations,
} from "../src/terrain.mjs";
import { analyzeMission } from "../src/mission-analysis.mjs";
import { toKML, toGeoJSON } from "../src/exports.mjs";

const data = JSON.parse(
  fs.readFileSync(new URL("../public/points.json", import.meta.url)),
);
const obstacles = JSON.parse(
  fs.readFileSync(new URL("../public/obstacles.json", import.meta.url)),
);
const near = (a, b, t = 1e-6) =>
  assert.ok(Math.abs(a - b) <= t, `${a} != ${b} (±${t})`);
// Synthetic relief: a 60 m ridge, a 40 m dip and a north–south slope.
const hilly = (p) => {
  const lon = C.Math.toDegrees(p.longitude),
    lat = C.Math.toDegrees(p.latitude);
  return (
    1150 +
    60 * Math.exp(-(((lon - 40.3) / 0.004) ** 2)) -
    40 * Math.exp(-(((lon - 40.287) / 0.003) ** 2)) +
    30 * Math.sin(lat * 900)
  );
};
const route = createTerrainRoute(C, data, {}, { obstacles });
route.samples.forEach((p) => (p.height = hilly(p)));
const mission = buildMission(C, route);
const nominal = simulate(mission.reference);
const context = {
  settings: mission.settings,
  terrain: terrainForWorker(mission),
};
const fly = (type, time, extra = {}) =>
  simulate(mission.reference, { ...context, fault: { type, time }, ...extra });
// A flight with its terrain measurements, as the app attaches it.
const measured = (sim) => ({ ...mission, ...measureFlight(mission, sim) });
const phases = (sim) => {
  const seconds = {};
  for (let i = 0; i < sim.rows; i++) {
    const name = PHASE_NAMES[sim.data[i * STRIDE + 30]];
    seconds[name] = (seconds[name] ?? 0) + 1 / 30;
  }
  return seconds;
};
const battery = fly("battery", 240),
  outage = fly("gnss-outage", 300),
  loss = fly("gnss-loss", 300),
  motor = fly("motor", 300);

test("fault scenarios are validated and need the planner's settings", () => {
  assert.equal(validateFault(null), null);
  assert.deepEqual(Object.keys(FAULTS).sort(), [
    "battery",
    "gnss-loss",
    "gnss-outage",
    "motor",
  ]);
  assert.throws(() => validateFault({ type: "fire", time: 10 }), /Bilinmeyen/);
  assert.throws(() => validateFault({ type: "motor", time: -1 }), /negatif/);
  assert.throws(
    () => simulate(mission.reference, { fault: { type: "motor", time: 10 } }),
    /uçuş ayarları gerekli/,
  );
});

test("elevation raster: bilinear surface, conservative floor and a hull that covers every direct leg home", () => {
  const raster = {
    x0: 0,
    y0: 0,
    spacing: 10,
    nx: 3,
    ny: 3,
    // z = 2x + 3y on the nodes.
    heights: Float64Array.from({ length: 9 }, (_, i) => 2 * (i % 3) * 10 + 3 * Math.floor(i / 3) * 10),
  };
  near(rasterHeight(raster, 7, 13), 2 * 7 + 3 * 13, 1e-9);
  assert.ok(rasterFloor(raster, 7, 13) >= rasterHeight(raster, 7, 13));
  assert.ok(Number.isNaN(rasterHeight(raster, -5, 5)));
  const hull = convexHull([[0, 0], [10, 0], [5, 5], [10, 10], [0, 10]]);
  assert.equal(hull.length, 4);
  assert.equal(distanceToConvex([5, 5], hull), 0);
  near(distanceToConvex([15, 5], hull), 5, 1e-9);
  // Every point of the planned path, and so every straight leg between two
  // of them, lies on raster cells whose four nodes were sampled.
  const model = indexStations(terrainForWorker(mission)),
    { raster: r } = model;
  for (let i = 0; i < mission.path.table.length; i += 50) {
    const p = mission.path.table[i].p;
    for (const f of [0.25, 0.5, 0.75]) {
      const x = p[0] * f,
        y = p[1] * f;
      assert.ok(Number.isFinite(rasterFloor(r, x, y)), `${x}, ${y}`);
      // The direct leg's floor is never below the ground under its centre.
      assert.ok(corridorFloor(model, [x, y], [1, 0]) >= rasterHeight(r, x, y) - 1e-9);
    }
  }
  const layout = rasterLayout([[0, 0], [100, 0], [0, 100]], 25, 50);
  assert.ok(layout.nodes.every((p) => distanceToConvex(p, convexHull([[0, 0], [100, 0], [0, 100]])) <= 50));
});

test("guidance time warp brakes along the path with continuous p, v and a, holds, and resumes on the shifted clock", () => {
  const ref = mission.reference,
    guidance = makeGuidance(ref, context),
    t0 = ref.surveyStart + 200;
  assert.equal(guidance.at(t0 - 1).p.join(), referenceAt(ref, t0 - 1).p.join());
  guidance.stop(t0, "hold");
  const before = referenceAt(ref, t0 - 1e-4),
    after = guidance.at(t0 + 1e-4);
  for (const key of ["p", "v", "a"])
    for (let i = 0; i < 3; i++) near(before[key][i], after[key][i], 2e-3);
  // Braking deceleration within the planner's acceleration limit.
  let previous = guidance.at(t0);
  for (let t = t0 + 0.05; t < t0 + 15; t += 0.05) {
    const r = guidance.at(t);
    assert.ok(Math.hypot(r.a[0], r.a[1]) <= mission.settings.acceleration + 0.3);
    near(Math.hypot(...r.p.map((x, i) => x - previous.p[i])), 0, 0.5);
    previous = r;
  }
  assert.equal(guidance.mode, "hold");
  const held = guidance.at(t0 + 20);
  assert.equal(held.phase, "hold");
  near(Math.hypot(...held.v), 0, 1e-12);
  near(guidance.at(t0 + 40).p[0], held.p[0], 1e-12);
  assert.equal(guidance.endTime, Infinity);
  guidance.resume(t0 + 40);
  for (let t = t0 + 40; t < t0 + 70; t += 0.5) guidance.at(t);
  assert.equal(guidance.mode, "mission");
  // Back on the mission clock, delayed by the pause.
  const delay = guidance.endTime - ref.duration;
  assert.ok(delay > 35 && delay < 50, `${delay}`);
  const late = guidance.at(t0 + 120);
  const nominalAt = referenceAt(ref, t0 + 120 - delay);
  for (let i = 0; i < 3; i++) near(late.p[i], nominalAt.p[i], 1e-6);
});

test("armed failsafes leave a healthy flight untouched and faulted solves are deterministic", () => {
  const armed = simulate(mission.reference, { ...context, failsafe: true });
  assert.deepEqual(armed.data, nominal.data);
  assert.deepEqual(armed.failsafe.events, []);
  assert.deepEqual(fly("battery", 240).data, battery.data);
});

test("low battery: cell fault, return-charge RTL over a direct terrain-following leg, landing on the pad with the reserve", () => {
  const events = battery.failsafe.events,
    m = battery.metrics;
  assert.equal(events[0].kind, "fault");
  near(events[0].time, 240, 1e-9);
  const rtl = events.find((e) => e.label.includes("RTL"));
  assert.ok(rtl && rtl.time > 240, JSON.stringify(events));
  assert.equal(battery.failsafe.returnReason, "battery");
  // The return replaced the rest of the mission.
  assert.ok(battery.duration < nominal.duration - 200);
  assert.ok(phases(battery).rtl > 60);
  assert.equal(m.landingSite, "home");
  assert.ok(m.disarmed && m.landingError < 0.5, `${m.landingError}`);
  assert.ok(m.finalSoc >= battery.battery.reserve, `${m.finalSoc}`);
  // The leg runs straight from the stop point to home over the elevation
  // model and keeps the target clearance over its own corridor floor.
  const leg = battery.failsafe.rtl;
  near(Math.hypot(...leg.points.at(-1)), 0, 1e-9);
  near(leg.distance, Math.hypot(...leg.points[0]), 1e-6);
  const flown = measureFlight(mission, battery);
  assert.ok(flown.returnClearance >= mission.settings.agl - mission.settings.clearanceTolerance);
  assert.ok(flown.returnDeviation < 3, `${flown.returnDeviation}`);
  assert.ok(m.maxTrackingError < 1.5);
  // The operator banner names the failsafe from the RTL on.
  assert.equal(failsafeStatus(battery, rtl.time - 1), null);
  assert.equal(failsafeStatus(battery, rtl.time + 1), "düşük batarya");
});

test("short GNSS outage: dead-reckoning hold on the path, then the mission resumes", () => {
  const labels = outage.failsafe.events.map((e) => e.label),
    m = outage.metrics;
  for (const label of [
    "ARIZA · GNSS sinyali kesildi",
    "FAILSAFE · konum tutma",
    "GNSS geri geldi",
    "Göreve devam",
  ])
    assert.ok(labels.includes(label), label);
  assert.ok(phases(outage).hold > 3);
  // The hold delays the whole mission, which then lands at home.
  assert.ok(outage.duration > nominal.duration + 5);
  assert.equal(m.landingSite, "home");
  assert.ok(m.landingError < 0.5);
  // Uncertainty grows while dead reckoning and shrinks once fixes return.
  const eph = (sim, t) => replayAt(sim, t).eph;
  assert.ok(eph(outage, 314) > 5 * eph(outage, 299), `${eph(outage, 314)}`);
  assert.ok(eph(outage, 340) < eph(outage, 314));
  assert.ok(m.maxNavigationError < 3);
});

test("permanent GNSS loss: hold, then a field landing on the elevation model's ground", () => {
  const m = loss.metrics,
    labels = loss.failsafe.events.map((e) => e.label);
  assert.ok(labels.includes("FAILSAFE · yerinde iniş"), JSON.stringify(labels));
  const land = loss.failsafe.events.find((e) => e.label === "FAILSAFE · yerinde iniş");
  near(land.time - 300, FAILSAFE.gnssTimeout + FAILSAFE.holdTimeout, 0.05);
  assert.equal(m.landingSite, "field");
  assert.ok(m.disarmed && m.finalSpeed < 0.05);
  assert.ok(m.homeDistance > 500, `${m.homeDistance}`);
  assert.ok(m.contactSpeed < 0.6);
  // The vehicle rests on the ground under its true position.
  const end = replayAt(loss, loss.duration).p;
  near(
    end[2],
    groundHeight(indexStations(context.terrain), end[0], end[1]) +
      mission.settings.landingOffset,
    0.05,
  );
  // Dead reckoning drifts; the filter's own uncertainty grows with it.
  assert.ok(m.maxNavigationError > 1 && m.maxEph > m.maxNavigationError / 3);
});

test("motor effectiveness loss: imbalance detected within seconds, RTL with the rate integrator holding the track", () => {
  const events = motor.failsafe.events,
    detected = events.find((e) => e.label === "Motor dengesizliği algılandı");
  assert.ok(detected && detected.time - 300 < 5, JSON.stringify(events));
  assert.equal(motor.failsafe.returnReason, "motor");
  assert.equal(motor.metrics.landingSite, "home");
  assert.ok(motor.metrics.maxTrackingError < 2, `${motor.metrics.maxTrackingError}`);
  // The weak rotor spins faster than the others for the same thrust.
  const after = replayAt(motor, 400).motors,
    others = (after[1] + after[2] + after[3]) / 3;
  assert.ok(after[0] / others > 1.1, JSON.stringify(after));
  // …and draws more power than the healthy flight did at that point.
  assert.ok(replayAt(motor, 400).battery.power > replayAt(nominal, 400).battery.power);
});

test("a fault during the final landing needs no failsafe action", () => {
  const late = fly("battery", mission.reference.duration - 60),
    analysis = analyzeMission(C, attachSimulation({ ...mission }, late, nominal));
  assert.equal(late.failsafe.events.filter((e) => e.kind === "failsafe").length, 0);
  assert.equal(late.metrics.landingSite, "home");
  assert.match(
    analysis.checks.find((c) => c.id === "fault-response").detail,
    /failsafe eylemi gerekmedi/,
  );
});

test("checklist judges the plan on the fault-free solve and reports the rehearsal in its own advisory group", () => {
  const faulted = attachSimulation({ ...mission }, loss, nominal),
    analysis = analyzeMission(C, faulted),
    plan = analysis.checks.filter((c) => c.group !== "fault"),
    scenario = analysis.checks.filter((c) => c.group === "fault");
  assert.equal(analysis.blocked, false);
  for (const c of plan) assert.ok(c.ok, `${c.label}: ${c.detail}`);
  assert.deepEqual(
    scenario.map((c) => c.id),
    ["fault-response", "fault-landing", "fault-clearance", "fault-obstacles", "fault-battery", "fault-navigation"],
  );
  assert.ok(scenario.every((c) => c.blocking === false));
  assert.match(
    scenario[0].detail,
    /GNSS sinyali kesildi 05:00 → konum tutma 05:01 → yerinde iniş 05:21/,
  );
  assert.match(scenario[1].detail, /^yerinde · evden/);
  // The flight log interleaves the solve's fault and failsafe events.
  const kinds = analysis.events.filter((e) => e.kind !== "info");
  assert.deepEqual(
    kinds.map((e) => e.kind),
    ["fault", "failsafe", "failsafe"],
  );
  for (let i = 1; i < analysis.events.length; i++)
    assert.ok(analysis.events[i].time >= analysis.events[i - 1].time);
  assert.ok(analysis.events.some((e) => e.label === "İniş noktası üzerinde"));
  // Exports mark the events where they happened.
  const kml = toKML(C, faulted, analysis);
  assert.ok(kml.includes("<name>Arıza ve failsafe</name>"));
  assert.equal(
    kml.split("<Placemark>").length - 1,
    analysis.capture.photos.length +
      3 +
      mission.rally.length +
      loss.failsafe.events.length,
  );
  const geo = toGeoJSON(C, faulted, analysis);
  assert.equal(
    geo.features.filter((f) => f.properties.kind === "event").length,
    loss.failsafe.events.length,
  );
  // A battery fault drains the faulted landing below the reserve without
  // blocking the plan, whose own landing keeps it.
  const drained = attachSimulation({ ...mission }, fly("battery", 420), nominal),
    result = analyzeMission(C, drained);
  assert.equal(result.blocked, false);
  assert.equal(result.checks.find((c) => c.id === "reserve").ok, true);
  assert.equal(result.checks.find((c) => c.id === "fault-battery").ok, false);
});
