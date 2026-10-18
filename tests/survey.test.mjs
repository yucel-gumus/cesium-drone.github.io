import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import * as C from "cesium";
import {
  createTerrainRoute,
  buildMission,
  attachSimulation,
} from "../src/flight-planner.mjs";
import { roundedPath, referenceAt } from "../src/trajectory.mjs";
import { simulate, STRIDE, PHASE_NAMES } from "../src/quadrotor.mjs";
import { polynomial } from "../src/flight-math.mjs";
import {
  validateSurveyArea,
  preparePolygon,
  polygonArea,
  pointInPolygon,
  lawnmower,
  areaCoverage,
} from "../src/survey.mjs";
import { acrossTrackWidth, CAMERA } from "../src/payload.mjs";
import { analyzeMission } from "../src/mission-analysis.mjs";
import { toQGCPlan, toKML, toGeoJSON } from "../src/exports.mjs";

const data = JSON.parse(
  fs.readFileSync(new URL("../public/points.json", import.meta.url)),
);
const obstacles = JSON.parse(
  fs.readFileSync(new URL("../public/obstacles.json", import.meta.url)),
);
const near = (a, b, t = 1e-6) =>
  assert.ok(Math.abs(a - b) <= t, `${a} != ${b} (±${t})`);
// Cosine of the turn at b between legs a→b and b→c.
const turn = (a, b, c) => {
  const u = [b[0] - a[0], b[1] - a[1]],
    v = [c[0] - b[0], c[1] - b[1]];
  return (u[0] * v[0] + u[1] * v[1]) / (Math.hypot(...u) * Math.hypot(...v));
};
// Lake surface with the valley side rising north of it.
const valley = (p) =>
  1146 + 30 * Math.max(0, (C.Math.toDegrees(p.latitude) - 40.6195) / 0.001);
function plan(task = {}, options = {}, simOptions = {}) {
  const route = createTerrainRoute(C, data, options, { type: "survey", obstacles, ...task });
  route.samples.forEach((p) => (p.height = valley(p)));
  const mission = buildMission(C, route);
  attachSimulation(mission, simulate(mission.reference, simOptions));
  return mission;
}
const survey = plan();
const surveyAnalysis = analyzeMission(C, survey);

test("survey area is optional, inside the bounds and a simple polygon of sensible size", () => {
  const bounds = data.bounds;
  assert.equal(validateSurveyArea({ ...data, surveyArea: undefined }, bounds), null);
  assert.equal(validateSurveyArea(data, bounds).length, 5);
  // A closing duplicate is dropped.
  const closed = [...data.surveyArea, data.surveyArea[0]];
  assert.equal(validateSurveyArea({ ...data, surveyArea: closed }, bounds).length, 5);
  assert.throws(
    () => validateSurveyArea({ ...data, surveyArea: data.surveyArea.slice(0, 2) }, bounds),
    /3–50 köşe/,
  );
  assert.throws(
    () =>
      validateSurveyArea(
        { ...data, surveyArea: [{ longitude: 0, latitude: 0 }, ...data.surveyArea] },
        bounds,
      ),
    /rota sınırlarının dışında/,
  );
  // A bow tie crosses itself; a 10 m square is too small.
  assert.throws(
    () => preparePolygon([[0, 0], [300, 300], [300, 0], [0, 300]]),
    /kendini kesmemeli/,
  );
  assert.throws(() => preparePolygon([[0, 0], [10, 0], [10, 10], [0, 10]]), /ha arasında/);
  // Clockwise input comes back counter-clockwise.
  const ccw = preparePolygon([[0, 0], [0, 200], [300, 200], [300, 0]]);
  assert.ok(polygonArea(ccw) > 0);
  // Recon routes do not need the polygon; survey routes do.
  assert.throws(
    () =>
      createTerrainRoute(C, { ...data, surveyArea: undefined }, {}, { type: "survey" }),
    /surveyArea poligonu yok/,
  );
  for (const sideOverlap of [0.5, 0.85])
    assert.throws(
      () => createTerrainRoute(C, data, {}, { type: "survey", sideOverlap }),
      /Yan bindirme %60–80/,
    );
});

test("line spacing comes from the camera's across-track footprint and the side overlap", () => {
  // Nadir: the image's long side, AGL × sensor width / focal length.
  near(acrossTrackWidth(80, -90), (80 * CAMERA.sensorWidth) / CAMERA.focalLength, 1e-9);
  // Tilted forward by θ the near (bottom) edge's rays fall
  // f sin θ + (h/2) cos θ per unit of sensor depth: on flat ground the
  // near edge is 2 (w/2) AGL / (f sin θ + (h/2) cos θ) wide.
  const theta = Math.PI / 3,
    { sensorWidth: w, sensorHeight: h, focalLength: f } = CAMERA;
  near(
    acrossTrackWidth(80, -60),
    (w * 80) / (f * Math.sin(theta) + (h / 2) * Math.cos(theta)),
    1e-9,
  );
  const s = survey.survey;
  near(s.swath, 120, 1e-9);
  near(s.spacing, 0.3 * 120, 1e-9);
  const dense = createTerrainRoute(C, data, {}, { type: "survey", sideOverlap: 0.8 });
  near(dense.survey.spacing, 0.2 * 120, 1e-9);
  assert.ok(dense.survey.count > s.count);
  const higher = createTerrainRoute(C, data, { agl: 110 }, { type: "survey" });
  near(higher.survey.spacing, 0.3 * 1.5 * 110, 1e-9);
  assert.ok(higher.survey.count < s.count);
});

test("lawnmower lines run along the longest edge, cover the width and turn with two 90° corners", () => {
  // A 500 × 220 m rectangle rotated by 30°, home 300 m away.
  const a = Math.PI / 6,
    rotate = ([x, y]) => [x * Math.cos(a) - y * Math.sin(a), x * Math.sin(a) + y * Math.cos(a)];
  const polygon = preparePolygon([[0, 0], [500, 0], [500, 220], [0, 220]].map(rotate)),
    spacing = 36,
    grid = lawnmower(polygon, spacing, { home: [-300, -50] });
  // Parallel to the 500 m edges.
  near(Math.abs(grid.direction[0] * Math.cos(a) + grid.direction[1] * Math.sin(a)), 1, 1e-9);
  assert.equal(grid.count, Math.ceil(220 / spacing) + 1);
  const ws = grid.lines.map((l) => l.w).sort((x, y) => x - y);
  for (let i = 1; i < ws.length; i++) near(ws[i] - ws[i - 1], spacing, 1e-9);
  // The outermost lines sit on or outside the polygon's edges.
  const across = polygon.map((p) => p[0] * grid.normal[0] + p[1] * grid.normal[1]);
  assert.ok(ws[0] <= Math.min(...across) + 1e-9 && ws.at(-1) >= Math.max(...across) - 1e-9);
  // Every line overruns the polygon by the turnaround distance at both ends.
  for (const line of grid.lines) {
    const lo = Math.min(line.from, line.to),
      hi = Math.max(line.from, line.to);
    near(hi - lo, 500 + 2 * 15, 1e-6);
  }
  // Inside the grid every corner is a right angle.
  const w = grid.waypoints;
  for (let i = grid.gridStart + 1; i < grid.gridEnd; i++)
    near(turn(w[i - 1], w[i], w[i + 1]), 0, 1e-9);
  // No corner of the whole route is sharper than 150°, so it can be flown.
  for (let i = 1; i < w.length - 1; i++)
    assert.ok(turn(w[i - 1], w[i], w[i + 1]) >= Math.cos((150 * Math.PI) / 180) - 1e-9);
  assert.doesNotThrow(() => roundedPath(w, 45, 2, 0.5));
  // Home on the first line's axis: a reversal is avoided through a point
  // outside the grid, and the turn onto the line stays a right angle.
  const inline = lawnmower(polygon, spacing, { home: rotate([250, -16]) });
  assert.ok(inline.gridStart === 2 || inline.gridEnd === inline.waypoints.length - 3);
  const v = inline.waypoints;
  for (let i = 1; i < v.length - 1; i++)
    assert.ok(turn(v[i - 1], v[i], v[i + 1]) >= Math.cos((150 * Math.PI) / 180) - 1e-9);
});

test("area coverage counts polygon samples inside at least one footprint", () => {
  const square = [[0, 0], [100, 0], [100, 100], [0, 100]];
  assert.ok(pointInPolygon([50, 50], square) && !pointInPolygon([150, 50], square));
  near(areaCoverage(square, [[[-10, -10], [110, -10], [110, 110], [-10, 110]]]), 1);
  near(areaCoverage(square, [[[-10, -10], [50, -10], [50, 110], [-10, 110]]]), 0.5, 0.01);
  assert.equal(areaCoverage(square, []), 0);
});

test("survey reference is C2 and within the speed, acceleration and jerk limits, and turns keep speed", () => {
  const ref = survey.reference,
    s = survey.settings;
  for (let i = 0; i < ref.horizontal.length; i++) {
    const seg = ref.horizontal[i];
    for (let k = 0; k <= 20; k++) {
      const p = seg.coefficients.map((c) => polynomial(c, k / 20, seg.duration));
      assert.ok(Math.hypot(p[0].v, p[1].v) <= s.cruiseSpeed + 1e-5);
      assert.ok(Math.hypot(p[0].a, p[1].a) <= s.acceleration + 1e-5);
      assert.ok(Math.hypot(p[0].j, p[1].j) <= s.jerk + 1e-5);
    }
    if (i) {
      const prev = ref.horizontal[i - 1];
      for (let axis = 0; axis < 3; axis++) {
        const a = polynomial(prev.coefficients[axis], 1, prev.duration),
          b = polynomial(seg.coefficients[axis], 0, seg.duration);
        for (const key of ["p", "v", "a"]) near(a[key], b[key], 1e-6);
      }
    }
  }
  // Smoothed speed profile: the grid is flown at more than 70 % of cruise
  // speed on average, turnarounds included.
  const mean = survey.path.length / ref.horizontalDuration;
  assert.ok(mean > 0.7 * s.cruiseSpeed, `mean speed ${mean}`);
  // Transit, grid and return labels follow each other once.
  const labels = [];
  for (let t = ref.surveyStart + 1; t < ref.surveyStart + ref.horizontalDuration; t += 2) {
    const phase = referenceAt(ref, t).phase;
    if (labels.at(-1) !== phase) labels.push(phase);
  }
  assert.deepEqual(labels, ["transit", "survey", "return"]);
});

test("survey mission: camera fires only on the grid, covers the polygon with the planned side overlap and passes the checklist", () => {
  const { capture, coverage, checks, events } = surveyAnalysis,
    sim = survey.simulation;
  assert.ok(capture.photos.length > 200);
  // Photos only in the grid phase, never in the transits.
  const gridPhase = PHASE_NAMES.indexOf("survey");
  for (const photo of capture.photos) {
    let i = 0;
    while (i < sim.rows - 1 && sim.data[(i + 1) * STRIDE] <= photo.time) i++;
    const phases = [sim.data[i * STRIDE + 30], sim.data[Math.min(sim.rows - 1, i + 1) * STRIDE + 30]];
    assert.ok(phases.includes(gridPhase), `photo ${photo.index} outside the grid`);
  }
  assert.ok(coverage.areaCovered >= 0.98, `covered ${coverage.areaCovered}`);
  assert.equal(coverage.linesImaged, survey.survey.count);
  assert.ok(Math.abs(coverage.minSideOverlap - 0.7) < 0.03, `${coverage.minSideOverlap}`);
  assert.ok(coverage.minOverlap >= 0.75);
  near(coverage.areaHectares, Math.abs(survey.survey.areaM2) / 1e4, 1e-9);
  assert.equal(surveyAnalysis.blocked, false);
  for (const c of checks) assert.ok(c.ok, `${c.label}: ${c.detail}`);
  assert.ok(checks.some((c) => c.id === "area"));
  const labels = events.map((e) => e.label);
  for (const label of ["Görev irtifası", "Izgara başladı", "Izgara tamamlandı", "Zemin teması"])
    assert.ok(labels.includes(label), label);
  assert.ok(labels.indexOf("Izgara başladı") < labels.indexOf("Izgara tamamlandı"));
});

test("survey exports: QGC waypoints switch the trigger at the grid, KML and GeoJSON carry the polygon", () => {
  const planFile = toQGCPlan(C, survey, surveyAnalysis, { surveyItem: false }),
    items = planFile.mission.items;
  items.forEach((item, i) => assert.equal(item.doJumpId, i + 1));
  assert.equal(items[0].command, 22);
  assert.equal(items.at(-1).command, 20);
  const triggers = items
    .map((item, i) => ({ item, i }))
    .filter(({ item }) => item.command === 206);
  assert.equal(triggers.length, 2);
  const [on, off] = triggers;
  assert.ok(on.item.params[0] > 10 && off.item.params[0] === 0);
  // Both switches follow a waypoint: transit waypoints before, grid between.
  assert.equal(items[on.i - 1].command, 16);
  assert.equal(items[off.i - 1].command, 16);
  const before = items.slice(0, on.i).filter((x) => x.command === 16).length,
    grid = items.slice(on.i, off.i).filter((x) => x.command === 16).length;
  assert.ok(before >= 1);
  // Every line contributes at least its two ends.
  assert.ok(grid >= 2 * survey.survey.count - 1, `${grid} grid waypoints`);
  const kml = toKML(C, survey, surveyAnalysis);
  assert.ok(kml.includes("<name>Haritalama alanı</name>"));
  assert.equal(
    kml.split("<Placemark>").length - 1,
    surveyAnalysis.capture.photos.length + 4 + survey.rally.length,
  );
  const geo = toGeoJSON(C, survey, surveyAnalysis),
    area = geo.features.find((f) => f.properties.kind === "surveyArea");
  const ring = area.geometry.coordinates[0];
  assert.deepEqual(ring[0], ring.at(-1));
  assert.equal(ring.length, data.surveyArea.length + 1);
  assert.equal(area.properties.lines, survey.survey.count);
});
