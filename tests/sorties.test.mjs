import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import * as C from "cesium";
import {
  createTerrainRoute,
  buildMission,
  attachSimulation,
  surveySorties,
  sortieLimit,
} from "../src/flight-planner.mjs";
import { simulate, AIRFRAME } from "../src/quadrotor.mjs";
import { BATTERY } from "../src/battery.mjs";
import { partitionLines, sortieSeconds } from "../src/survey.mjs";
import { analyzeMission, planPhotos } from "../src/mission-analysis.mjs";
import { toQGCPlan } from "../src/exports.mjs";

const data = JSON.parse(
  fs.readFileSync(new URL("../public/points.json", import.meta.url)),
);
const near = (a, b, t = 1e-6) =>
  assert.ok(Math.abs(a - b) <= t, `${a} != ${b} (±${t})`);
const valley = (p) =>
  1146 + 30 * Math.max(0, (C.Math.toDegrees(p.latitude) - 40.6195) / 0.001);
const airframe = (mass) => ({
  ...AIRFRAME,
  mass,
  inertia: AIRFRAME.inertia.map((x) => (x * mass) / 2),
});
const fly = (route) => {
  route.samples.forEach((p) => (p.height = valley(p)));
  const mission = buildMission(C, route);
  return attachSimulation(mission, simulate(mission.reference));
};

test("sortie limit follows hover power; the grid splits into the fewest balanced contiguous blocks", () => {
  const limit = (mass) => sortieLimit(airframe(mass), BATTERY);
  // About 20 minutes of usable flight at 2 kg, less for heavier airframes
  // (hover power grows with mass^1.5).
  assert.ok(limit(2) > 900 && limit(2) < 1800, `${limit(2)}`);
  near(limit(3) / limit(2), (2 / 3) ** 1.5, 0.02);
  const route = createTerrainRoute(C, data, {}, { type: "survey", sideOverlap: 0.8 }),
    s = route.survey,
    grid = { lines: s.allLines, direction: s.direction, normal: s.normal },
    home = s.waypoints[0],
    all = s.allLines.map((l) => l.index),
    whole = sortieSeconds(grid, all, home, route.settings);
  assert.deepEqual(partitionLines(grid, home, route.settings, Infinity), [all]);
  // Just below the whole grid's estimate: two halves.
  const halves = partitionLines(grid, home, route.settings, whole - 1);
  assert.equal(halves.length, 2);
  assert.deepEqual(halves.flat(), all);
  assert.ok(Math.abs(halves[0].length - halves[1].length) <= 1);
  for (const block of halves) {
    assert.ok(block.length >= 2);
    assert.ok(sortieSeconds(grid, block, home, route.settings) <= whole - 1);
  }
  assert.throws(() => partitionLines(grid, home, route.settings, 60), /bölünemiyor/);
});

test("a survey too long for one battery flies as sorties that each land with reserve and together cover the area", () => {
  const routes = surveySorties(
    C,
    data,
    {},
    { sideOverlap: 0.8 },
    sortieLimit(airframe(3), BATTERY),
  );
  assert.ok(routes.length >= 2, `${routes.length} sorties`);
  const count = routes[0].survey.allLines.length;
  // Every line is flown exactly once, in contiguous blocks.
  const flown = routes.flatMap((r) => r.survey.lines.map((l) => l.w)).sort((a, b) => a - b);
  assert.equal(flown.length, count);
  routes.forEach((r, k) => {
    assert.deepEqual(r.survey.sortie, { index: k, count: routes.length });
    assert.equal(r.survey.allLines.length, count);
  });
  const missions = routes.map(fly);
  for (const [k, m] of missions.entries()) {
    const metrics = m.simulation.metrics;
    assert.ok(metrics.disarmed && metrics.landingSite === "home", `sortie ${k}`);
    // The battery is sized for 3 kg; at 2 kg each sortie lands well above
    // the reserve.
    assert.ok(metrics.finalSoc > BATTERY.reserve + 0.1, `sortie ${k}: ${metrics.finalSoc}`);
  }
  const surveyPhotos = missions.flatMap((m) => planPhotos(m));
  const analyses = missions.map((m) => analyzeMission(C, m, {}, { surveyPhotos }));
  // Each sortie alone images only its block; the combined photos cover the
  // polygon with the planned side overlap.
  const alone = analyzeMission(C, missions[0]).planCoverage.areaCovered;
  for (const a of analyses) {
    assert.ok(a.planCoverage.areaCovered > 0.95, `${a.planCoverage.areaCovered}`);
    assert.ok(a.planCoverage.areaCovered > alone + 0.2);
    assert.ok(a.planCoverage.minSideOverlap > 0.7, `${a.planCoverage.minSideOverlap}`);
    assert.ok(a.checks.find((c) => c.id === "area").ok);
  }
});

// The keys QGroundControl's loader requires (SurveyComplexItem::_loadV4V5,
// TransectStyleComplexItem::_load, CameraCalc::load, CameraSpec::load,
// MissionItem::load), with their JSON types.
const REQUIRED = {
  survey: {
    version: "number",
    type: "string",
    complexItemType: "string",
    entryLocation: "number",
    angle: "number",
    splitConcavePolygons: "boolean",
    polygon: "array",
    TransectStyleComplexItem: "object",
  },
  transect: {
    version: "number",
    TurnAroundDistance: "number",
    CameraTriggerInTurnAround: "boolean",
    HoverAndCapture: "boolean",
    Refly90Degrees: "boolean",
    CameraCalc: "object",
    VisualTransectPoints: "array",
    Items: "array",
    CameraShots: "number",
    // Required in the calculated-above-terrain distance mode.
    TerrainAdjustTolerance: "number",
    TerrainAdjustMaxClimbRate: "number",
    TerrainAdjustMaxDescentRate: "number",
  },
  camera: {
    version: "number",
    CameraName: "string",
    AdjustedFootprintSide: "number",
    AdjustedFootprintFrontal: "number",
    DistanceToSurface: "number",
    DistanceMode: "number",
    ValueSetIsDistance: "boolean",
    ImageDensity: "number",
    FrontalOverlap: "number",
    SideOverlap: "number",
    SensorWidth: "number",
    SensorHeight: "number",
    ImageWidth: "number",
    ImageHeight: "number",
    FocalLength: "number",
    Landscape: "boolean",
    FixedOrientation: "boolean",
    MinTriggerInterval: "number",
  },
  item: {
    type: "string",
    frame: "number",
    command: "number",
    params: "array",
    autoContinue: "boolean",
    doJumpId: "number",
  },
};
const typeOf = (x) => (Array.isArray(x) ? "array" : typeof x);
const conforms = (object, keys, where) => {
  for (const [key, type] of Object.entries(keys))
    assert.equal(typeOf(object[key]), type, `${where}.${key}`);
};

test("QGC export: the survey grid is a Survey complex item QGroundControl loads with our terrain-following items", () => {
  const route = createTerrainRoute(C, data, {}, { type: "survey" }),
    mission = fly(route),
    analysis = analyzeMission(C, mission),
    plan = toQGCPlan(C, mission, analysis),
    flat = toQGCPlan(C, mission, analysis, { surveyItem: false }),
    complex = plan.mission.items.filter((x) => x.type === "ComplexItem");
  assert.equal(complex.length, 1);
  const [survey] = complex,
    inner = survey.TransectStyleComplexItem,
    camera = inner.CameraCalc;
  conforms(survey, REQUIRED.survey, "survey");
  conforms(inner, REQUIRED.transect, "TransectStyleComplexItem");
  conforms(camera, REQUIRED.camera, "CameraCalc");
  for (const item of inner.Items) {
    conforms(item, REQUIRED.item, "Items[]");
    assert.equal(item.type, "SimpleItem");
    assert.equal(item.params.length, 7);
  }
  assert.equal(survey.version, 5);
  assert.equal(survey.complexItemType, "survey");
  assert.equal(inner.version, 2);
  assert.equal(camera.version, 2);
  assert.equal(camera.DistanceMode, 3);
  // Unrolled, the plan is the plain-waypoint plan with consecutive jump ids.
  const unrolled = plan.mission.items.flatMap((x) =>
    x.type === "ComplexItem" ? x.TransectStyleComplexItem.Items : [x],
  );
  assert.equal(unrolled.length, flat.mission.items.length);
  unrolled.forEach((x, i) => {
    assert.equal(x.doJumpId, i + 1);
    assert.equal(x.command, flat.mission.items[i].command);
    assert.deepEqual(x.params, flat.mission.items[i].params);
  });
  // Inside: the grid's first waypoint, the trigger on, ..., the trigger off.
  assert.equal(inner.Items[0].command, 16);
  assert.equal(inner.Items[1].command, 206);
  assert.ok(inner.Items[1].params[0] > 10);
  assert.deepEqual(inner.Items.at(-1).params, [0, 0, 0, 0, 0, 0, 0]);
  // Geometry: the polygon's vertices, two transect points per line, the
  // transect bearing as QGC's angle (clockwise from north).
  const s = mission.survey;
  assert.equal(survey.polygon.length, data.surveyArea.length);
  assert.equal(inner.VisualTransectPoints.length, 2 * s.count);
  const [a, b] = inner.VisualTransectPoints,
    rad = Math.PI / 180,
    east = (b[1] - a[1]) * Math.cos(a[0] * rad),
    north = b[0] - a[0];
  let bearing = Math.atan2(east, north) / rad;
  bearing = ((bearing % 180) + 180) % 180;
  if (bearing > 90) bearing -= 180;
  near(survey.angle, bearing, 0.2);
  // Camera: spacing and trigger distance as planned; QGC's own footprint
  // formula (landscape: image width across track) reproduces the spacing.
  near(camera.AdjustedFootprintSide, s.spacing, 0.01);
  near(camera.AdjustedFootprintFrontal, analysis.capture.spacing, 0.01);
  const side =
    ((camera.ImageWidth * camera.ImageDensity) / 100) * (1 - camera.SideOverlap / 100);
  near(side, camera.AdjustedFootprintSide, 0.05);
  assert.equal(inner.CameraShots, analysis.capture.photos.length);
  // A reconnaissance plan has no complex item.
  const recon = createTerrainRoute(C, data);
  const reconMission = fly(recon);
  assert.ok(
    toQGCPlan(C, reconMission, analyzeMission(C, reconMission)).mission.items.every(
      (x) => x.type === "SimpleItem",
    ),
  );
});
