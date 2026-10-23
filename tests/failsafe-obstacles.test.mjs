import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import * as C from "cesium";
import {
  createTerrainRoute,
  buildMission,
  attachSimulation,
  terrainForWorker,
} from "../src/flight-planner.mjs";
import { referenceAt } from "../src/trajectory.mjs";
import { simulate, replayAt } from "../src/quadrotor.mjs";
import {
  FAILSAFE,
  buildReturn,
  clearLandingSite,
  returnTiming,
} from "../src/failsafe.mjs";
import { indexStations } from "../src/terrain.mjs";
import { obstaclesNear, gridIndex } from "../src/obstacles.mjs";
import { analyzeMission, toGeodetic } from "../src/mission-analysis.mjs";

const data = JSON.parse(
  fs.readFileSync(new URL("../public/points.json", import.meta.url)),
);
const near = (a, b, t = 1e-6) =>
  assert.ok(Math.abs(a - b) <= t, `${a} != ${b} (±${t})`);
const valley = (p) =>
  1146 + 30 * Math.max(0, (C.Math.toDegrees(p.latitude) - 40.6195) / 0.001);
function plan(routeData = data, options = {}) {
  const route = createTerrainRoute(C, routeData, options, {});
  route.samples.forEach((p) => (p.height = valley(p)));
  return buildMission(C, route);
}
const armedOf = (mission) => ({
  failsafe: true,
  settings: mission.settings,
  terrain: terrainForWorker(mission),
  rally: mission.rally.map(({ name, xy }) => ({ name, xy })),
});
// A square footprint of half width `half` around a local point, in the
// worker's local obstacle format.
const localSquare = (xy, half, height, kind = "building") => ({
  kind,
  height,
  heightSource: "default",
  shape: "polygon",
  xy: [
    [xy[0] - half, xy[1] - half],
    [xy[0] + half, xy[1] - half],
    [xy[0] + half, xy[1] + half],
    [xy[0] - half, xy[1] + half],
  ],
  radius: 0,
  box: [xy[0] - half, xy[1] - half, xy[0] + half, xy[1] + half],
});
// The same square as a route-file obstacle (longitude, latitude corners).
function routeSquare(mission, xy, half, extra) {
  const corner = (dx, dy) => {
    const g = toGeodetic(C, mission, [xy[0] + dx, xy[1] + dy, 0]);
    return [g.longitude, g.latitude];
  };
  return {
    ...extra,
    polygon: [corner(-half, -half), corner(half, -half), corner(half, half), corner(-half, half)],
  };
}
const withObstacles = (model, items) => ({
  ...model,
  obstacles: { cell: 50, items },
  obstacleIndex: undefined,
});

const base = plan(),
  settings = base.settings,
  model = indexStations(terrainForWorker(base)),
  // A stop point on the survey, 300 s in, and its direct leg home.
  stop = referenceAt(base.reference, base.reference.surveyStart + 300).p,
  plain = buildReturn(stop, 0, model, settings);

test("a return leg climbs over an obstacle on its line to the obstacle clearance and is unchanged elsewhere", () => {
  assert.equal(base.obstacles.items.length, 0);
  assert.equal(terrainForWorker(base).obstacles, null);
  assert.equal(plain.leg.raised, null);
  const points = plain.leg.points,
    mid = Math.floor(points.length / 2),
    tower = localSquare(points[mid], 10, 90),
    raised = buildReturn(stop, 0, withObstacles(model, [tower]), settings);
  // Over the tower: its 90 m plus the 20 m obstacle clearance above the
  // corridor floor, instead of the 80 m target AGL.
  assert.ok(
    raised.leg.profile[mid] >= raised.leg.floor[mid] + 90 + settings.obstacleClearance - 1e-6,
    `${raised.leg.profile[mid] - raised.leg.floor[mid]}`,
  );
  assert.ok(plain.leg.profile[mid] < plain.leg.floor[mid] + 90);
  assert.equal(raised.leg.raised.obstacle.height, 90);
  near(raised.leg.raised.lift, 90 + settings.obstacleClearance - settings.agl, 1e-9);
  // The same leg away from the tower; the higher leg takes no less time.
  near(raised.leg.profile[0], plain.leg.profile[0], 1e-9);
  near(raised.leg.profile.at(-1), plain.leg.profile.at(-1), 1e-9);
  assert.ok(raised.ref.duration >= plain.ref.duration);
  // Obstacles off the leg, or lower than the target AGL needs, change
  // nothing: the reference is bit-identical.
  const off = localSquare([points[mid][0] + 300, points[mid][1] + 300], 10, 90),
    low = localSquare(points[mid], 10, settings.agl - settings.obstacleClearance),
    same = buildReturn(stop, 0, withObstacles(model, [off, low]), settings);
  assert.deepEqual(same.leg.profile, plain.leg.profile);
  assert.deepEqual(same.ref, plain.ref);
});

test("a failsafe landing site moves to the nearest clear point, widened by the position uncertainty", () => {
  const p = [stop[0], stop[1]],
    forest = withObstacles(model, [localSquare(p, 50, 30, "forest")]),
    buffer = settings.obstacleLateral;
  assert.deepEqual(clearLandingSite(withObstacles(model, []), p, settings), {
    xy: p,
    moved: 0,
    obstacle: null,
  });
  const site = clearLandingSite(forest, p, settings);
  assert.ok(
    site.moved >= 50 + buffer && site.moved <= 50 + buffer + FAILSAFE.landingSearchStep,
    `${site.moved}`,
  );
  near(Math.hypot(site.xy[0] - p[0], site.xy[1] - p[1]), site.moved, 1e-9);
  assert.equal(site.obstacle.kind, "forest");
  const index = gridIndex(forest.obstacles.items, 50);
  assert.equal(obstaclesNear(index, site.xy, buffer).length, 0);
  // On dead reckoning the gap must cover the horizontal uncertainty too.
  const unsure = clearLandingSite(forest, p, settings, 25);
  assert.ok(unsure.moved >= 50 + buffer + 25, `${unsure.moved}`);
  assert.equal(obstaclesNear(index, unsure.xy, buffer + 25).length, 0);
  // Nothing clear within reach: land where it is and name the obstacle.
  const huge = clearLandingSite(
    withObstacles(model, [localSquare(p, 400, 30, "forest")]),
    p,
    settings,
  );
  assert.equal(huge.moved, 0);
  assert.equal(huge.obstacle.kind, "forest");
});

test("GNSS-loss landing over a forest flies to a clear site; an operator's land command lands where commanded", () => {
  const lost = simulate(base.reference, {
      ...armedOf(base),
      faults: [{ type: "gnss-loss", time: 300 }],
    }),
    landAt = FAILSAFE.gnssTimeout + FAILSAFE.holdTimeout + 300,
    hold = replayAt(lost, landAt).estimate,
    forest = routeSquare(base, hold, 60, {
      name: "Deneme ormanı",
      kind: "forest",
      height: 30,
      heightSource: "assumed",
    }),
    wooded = plan({ ...data, obstacles: [forest] }),
    armed = armedOf(wooded);
  // 30 m canopy + 20 m stays under the 80 m target: the plan is unchanged.
  assert.deepEqual(wooded.profile, base.profile);
  const sim = simulate(wooded.reference, {
      ...armed,
      faults: [{ type: "gnss-loss", time: 300 }],
    }),
    m = sim.metrics,
    moved = sim.failsafe.events.find((e) => e.label === "İniş yeri engelden uzağa alındı");
  assert.ok(moved, JSON.stringify(sim.failsafe.events.map((e) => e.label)));
  near(moved.time, landAt, 0.05);
  assert.match(moved.detail, /Deneme ormanı \(orman örtüsü\) · 30 m \(kabul\)/);
  assert.equal(m.landingSite, "field");
  assert.ok(m.landingMoved >= 60 + settings.obstacleLateral, `${m.landingMoved}`);
  assert.equal(sim.failsafe.rtl.target.site, "field");
  near(sim.failsafe.rtl.distance, m.landingMoved, 1e-6);
  assert.ok(m.disarmed && m.finalSpeed < 0.05);
  const fault = analyzeMission(
    C,
    attachSimulation({ ...wooded }, sim, simulate(wooded.reference)),
  ).checks.find((c) => c.id === "fault-landing");
  assert.match(fault.detail, /engelden \d+ m uzağa alındı/);
  // The operator's "land" is literal: no search, no relocation.
  const commanded = simulate(wooded.reference, {
    ...armed,
    commands: [{ time: landAt, action: "land" }],
  });
  assert.equal(commanded.metrics.landingMoved, 0);
  assert.equal(commanded.failsafe.rtl, null);
  assert.ok(!commanded.failsafe.events.some((e) => e.label.startsWith("İniş yeri")));
});

test("a motor failure during the take-off aborts it and lands back on the pad", () => {
  const sim = simulate(base.reference, {
      ...armedOf(base),
      faults: [{ type: "motor", time: 15 }],
    }),
    labels = sim.failsafe.events.map((e) => e.label);
  assert.ok(labels.includes("FAILSAFE · kalkış iptal, eve iniş"), JSON.stringify(labels));
  assert.equal(sim.failsafe.returnReason, "motor");
  assert.equal(sim.metrics.landingSite, "home");
  assert.ok(
    sim.metrics.disarmed && sim.metrics.landingError < 0.5,
    `${sim.metrics.landingError}`,
  );
  // Never left the climb-out: no survey, back down within two minutes.
  assert.ok(sim.duration < 120, `${sim.duration}`);
  assert.equal(sim.failsafe.rtl, null);
  // The checklist sees no route flown, not a low route clearance.
  const clearance = analyzeMission(
    C,
    attachSimulation({ ...base }, sim, simulate(base.reference)),
  ).checks.find((c) => c.id === "fault-clearance");
  assert.ok(clearance.ok, clearance.detail);
  assert.equal(clearance.detail, "rota uçulmadı: kalkış iptal edildi");
  // A healthy take-off is untouched by the armed detector.
  assert.deepEqual(simulate(base.reference, armedOf(base)).failsafe.events, []);
});

test("short return legs (a failsafe stopping near its target) get a finite, growing duration", () => {
  let previous = 0;
  for (const d of [4, 8, 10, 15, 20, 25, 30, 40, 60]) {
    const leg = buildReturn([d, 0, settings.agl - settings.landingOffset], 0, null, settings)
      .ref.horizontalDuration;
    assert.ok(Number.isFinite(leg) && leg > previous, `${d} m: ${leg}`);
    previous = leg;
  }
});

test("the return-charge prediction is unbiased: the safety factor stays as margin above the reserve", () => {
  // The leg timing matches the planner's own leg.
  near(returnTiming(settings).leg(plain.leg.distance), plain.ref.horizontalDuration, 1.5);
  for (const rtlTarget of ["nearest", "home"]) {
    const sim = simulate(base.reference, {
        ...armedOf(base),
        rtlTarget,
        faults: [{ type: "battery", time: 240 }],
      }),
      trigger = sim.failsafe.events.find((e) => e.label === "FAILSAFE · eve dönüş (RTL)"),
      used =
        replayAt(sim, trigger.time).battery.soc -
        replayAt(sim, sim.metrics.disarmTime).battery.soc,
      predicted = sim.failsafe.returnNeed / FAILSAFE.batteryMargin;
    assert.ok(
      Math.abs(used / predicted - 1) < 0.08,
      `${rtlTarget}: used ${used} predicted ${predicted}`,
    );
    assert.ok(
      sim.metrics.finalSoc >= sim.battery.reserve + 0.015,
      `${rtlTarget}: ${sim.metrics.finalSoc}`,
    );
  }
});
