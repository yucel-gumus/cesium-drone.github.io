import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import * as C from "cesium";
import {
  createTerrainRoute,
  buildMission,
  attachSimulation,
  terrainForWorker,
  validateRallyPoints,
} from "../src/flight-planner.mjs";
import { simulate, STRIDE, replayAt } from "../src/quadrotor.mjs";
import { validateCommands, groundHeight } from "../src/failsafe.mjs";
import { indexStations } from "../src/terrain.mjs";
import { analyzeMission } from "../src/mission-analysis.mjs";
import { toQGCPlan, toGeoJSON } from "../src/exports.mjs";

const data = JSON.parse(
  fs.readFileSync(new URL("../public/points.json", import.meta.url)),
);
const near = (a, b, t = 1e-6) =>
  assert.ok(Math.abs(a - b) <= t, `${a} != ${b} (±${t})`);
// Valley floor at 1150 m with the slopes rising north of it.
const valley = (p) =>
  1150 + 25 * Math.max(0, (C.Math.toDegrees(p.latitude) - 40.6205) / 0.001);
const route = createTerrainRoute(C, data);
route.samples.forEach((p) => (p.height = valley(p)));
const mission = buildMission(C, route);
const armed = {
  failsafe: true,
  settings: mission.settings,
  terrain: terrainForWorker(mission),
  rally: mission.rally.map(({ name, xy }) => ({ name, xy })),
};
const fly = (extra) => simulate(mission.reference, { ...armed, ...extra });
const nominal = simulate(mission.reference);
const labels = (sim) => sim.failsafe.events.map((e) => e.label);
// Rows recorded strictly before `time`.
const before = (sim, time) => {
  let rows = 0;
  while (rows < sim.rows && sim.data[rows * STRIDE] < time) rows++;
  return sim.data.subarray(0, rows * STRIDE);
};

test("rally points are optional, named and inside the bounds; commands are validated", () => {
  assert.deepEqual(validateRallyPoints({ ...data, rallyPoints: undefined }, data.bounds), []);
  assert.equal(mission.rally.length, 2);
  assert.ok(mission.rally.every((r) => r.name && r.xy.length === 2));
  assert.throws(
    () => validateRallyPoints({ ...data, rallyPoints: [{ longitude: 0, latitude: 0 }] }, data.bounds),
    /sınırlarının dışında/,
  );
  assert.throws(() => validateCommands([{ time: 10, action: "flip" }]), /Bilinmeyen/);
  assert.throws(() => validateCommands([{ time: -1, action: "rtl" }]), /negatif/);
  assert.deepEqual(
    validateCommands([{ time: 20, action: "land" }, { time: 10, action: "hold" }]).map((c) => c.time),
    [10, 20],
  );
  assert.throws(
    () => fly({ faults: [{ type: "gnss-loss", time: 100 }, { type: "gnss-outage", time: 200 }] }),
    /Aynı türden/,
  );
});

test("an operator RTL re-solves identically up to the command, then returns to the nearest safe point", () => {
  const at = 420,
    sim = fly({ commands: [{ time: at, action: "rtl" }] });
  // Determinism: the faulted solve equals the nominal one before the command.
  assert.deepEqual(before(sim, at), before(nominal, at));
  assert.ok(labels(sim).includes("OPERATÖR · eve dönüş (RTL)"));
  assert.equal(sim.failsafe.operator, true);
  // Far east along the loop, the east rally point is nearer than home.
  const east = mission.rally[0],
    from = replayAt(nominal, at).p;
  assert.ok(Math.hypot(from[0] - east.xy[0], from[1] - east.xy[1]) < Math.hypot(from[0], from[1]));
  const m = sim.metrics;
  assert.equal(m.landingSite, "rally");
  assert.equal(m.landingName, east.name);
  assert.ok(m.disarmed && m.landingError < 0.5, `${m.landingError}`);
  // It rests on the modelled ground at the rally point.
  const end = replayAt(sim, sim.duration).p;
  near(
    end[2],
    groundHeight(indexStations(armed.terrain), end[0], end[1]) + mission.settings.landingOffset,
    0.05,
  );
  // Asked for home only, the same command flies back to the pad.
  const home = fly({ commands: [{ time: at, action: "rtl" }], rtlTarget: "home" });
  assert.equal(home.metrics.landingSite, "home");
  assert.ok(home.metrics.landingError < 0.5);
  assert.ok(home.duration > sim.duration);
  // The checklist names it, and the QGC plan and GeoJSON list the rally
  // points.
  const flown = attachSimulation({ ...mission }, sim, nominal),
    analysis = analyzeMission(C, flown);
  const landing = analysis.checks.find((c) => c.id === "fault-landing");
  assert.match(landing.detail, /^rally noktası \(Doğu vadi tabanı\)/);
  assert.equal(
    analysis.checks.find((c) => c.id === "fault-response").label,
    "Operatör komutları",
  );
  assert.equal(toQGCPlan(C, flown, analysis).rallyPoints.points.length, 2);
  assert.equal(
    toGeoJSON(C, flown, analysis).features.filter((f) => f.properties.kind === "rally").length,
    2,
  );
});

test("operator hold and resume pause the mission on its path, then it completes", () => {
  const sim = fly({
    commands: [
      { time: 300, action: "hold" },
      { time: 330, action: "resume" },
    ],
  });
  assert.deepEqual(
    labels(sim).filter((l) => l.startsWith("OPERATÖR")),
    ["OPERATÖR · konum tut", "OPERATÖR · göreve devam"],
  );
  assert.equal(sim.metrics.landingSite, "home");
  // About the half-minute hold plus the brake and the restart.
  const delay = sim.duration - nominal.duration;
  assert.ok(delay > 25 && delay < 45, `${delay}`);
  // Held still at the pause, on the planned corridor.
  const held = replayAt(sim, 325);
  assert.ok(Math.hypot(...held.v) < 0.2, JSON.stringify(held.v));
  const measured = attachSimulation({ ...mission }, sim, nominal);
  assert.ok(measured.maxCorridorDeviation < 3);
});

test("an operator can land in place during a return, but not take over the final landing", () => {
  const sim = fly({
    commands: [
      { time: 420, action: "rtl" },
      { time: 470, action: "land" },
    ],
  });
  assert.equal(sim.metrics.landingSite, "field");
  assert.ok(sim.metrics.disarmed);
  const ignored = fly({
    commands: [{ time: mission.reference.duration - 40, action: "land" }],
  });
  const event = ignored.failsafe.events.find((e) => e.kind === "operator");
  assert.match(event.detail, /uygulanamadı/);
  assert.equal(ignored.metrics.landingSite, "home");
});

test("two simultaneous faults escalate together: without GNSS a low battery lands in place instead of returning", () => {
  const sim = fly({
    faults: [
      { type: "gnss-loss", time: 300 },
      { type: "battery", time: 310 },
    ],
  });
  const events = labels(sim);
  assert.ok(events.includes("ARIZA · GNSS sinyali kesildi"));
  assert.ok(events.includes("ARIZA · batarya hücresi"));
  assert.equal(sim.failsafe.faults.length, 2);
  // Battery drained below what a return needs while the position is dead
  // reckoned: the autopilot lands where it is rather than flying blind.
  assert.ok(events.includes("FAILSAFE · yerinde iniş"), JSON.stringify(events));
  assert.ok(!events.some((l) => l.includes("RTL")));
  assert.equal(sim.metrics.landingSite, "field");
  const analysis = analyzeMission(C, attachSimulation({ ...mission }, sim, nominal));
  assert.match(
    analysis.checks.find((c) => c.id === "fault-response").detail,
    /GNSS sinyali kesildi 05:00 → konum tutma 05:01 → batarya hücresi 05:10/,
  );
});
