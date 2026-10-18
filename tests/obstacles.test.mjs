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
import { simulate } from "../src/quadrotor.mjs";
import {
  osmObstacleHeight,
  fromOverpass,
  validateObstacles,
  obstacleIndex,
  footprintDistance,
  obstaclesNear,
  obstacleAt,
  OSM_HEIGHTS,
} from "../src/obstacles.mjs";
import { analyzeMission, toGeodetic } from "../src/mission-analysis.mjs";

const data = JSON.parse(
  fs.readFileSync(new URL("../public/points.json", import.meta.url)),
);
const snapshot = JSON.parse(
  fs.readFileSync(new URL("../public/obstacles.json", import.meta.url)),
);
const near = (a, b, t = 1e-6) =>
  assert.ok(Math.abs(a - b) <= t, `${a} != ${b} (±${t})`);
const valley = (p) =>
  1146 + 30 * Math.max(0, (C.Math.toDegrees(p.latitude) - 40.6195) / 0.001);
function plan(options = {}, routeData = data, task = { obstacles: snapshot }) {
  const route = createTerrainRoute(C, routeData, options, task);
  route.samples.forEach((p) => (p.height = valley(p)));
  return buildMission(C, route);
}
const fly = (mission, extra = {}) =>
  attachSimulation(mission, simulate(mission.reference, extra));
const check = (analysis, id) => analysis.checks.find((c) => c.id === id);

test("OpenStreetMap heights: tags first, then storeys, then documented defaults; Overpass features become footprints", () => {
  assert.deepEqual(osmObstacleHeight("building", { height: "15 m" }), { height: 15, source: "tag" });
  assert.deepEqual(osmObstacleHeight("building", { "building:levels": "4" }), {
    height: 4 * OSM_HEIGHTS.storey + OSM_HEIGHTS.roof,
    source: "levels",
  });
  assert.equal(osmObstacleHeight("building", { building: "hotel" }).height, OSM_HEIGHTS.hotel);
  assert.deepEqual(osmObstacleHeight("building", { building: "yes" }), {
    height: OSM_HEIGHTS.building,
    source: "default",
  });
  assert.equal(osmObstacleHeight("tower", { man_made: "tower", "tower:type": "minaret" }).height, 25);
  assert.equal(osmObstacleHeight("mast", { man_made: "mast" }).height, OSM_HEIGHTS.mast);
  assert.deepEqual(osmObstacleHeight("forest", {}), { height: OSM_HEIGHTS.canopy, source: "assumed" });
  const square = [
    { lon: 40.3, lat: 40.62 },
    { lon: 40.3001, lat: 40.62 },
    { lon: 40.3001, lat: 40.6201 },
    { lon: 40.3, lat: 40.6201 },
    { lon: 40.3, lat: 40.62 },
  ];
  const list = fromOverpass({
    elements: [
      { type: "node", id: 1, lon: 40.29, lat: 40.61, tags: { man_made: "mast", name: "Verici" } },
      { type: "way", id: 2, geometry: square, tags: { building: "yes", "building:levels": "2" } },
      { type: "way", id: 3, geometry: square.slice(0, 3), tags: { natural: "tree_row" } },
      { type: "way", id: 4, geometry: square, tags: { highway: "track" } },
      {
        type: "relation",
        id: 5,
        tags: { landuse: "forest", type: "multipolygon" },
        members: [{ type: "way", ref: 6, role: "outer", geometry: square }],
      },
    ],
  });
  assert.deepEqual(
    list.map((o) => [o.id, o.kind, o.point ? "point" : o.line ? "line" : "polygon"]),
    [
      ["node/1", "mast", "point"],
      ["way/2", "building", "polygon"],
      ["way/3", "tree_row", "line"],
      ["relation/5/6", "forest", "polygon"],
    ],
  );
  // Closed rings drop the repeated vertex; the mast keeps its name.
  assert.equal(list[1].polygon.length, 4);
  assert.equal(list[0].name, "Verici");
  assert.equal(list[1].height, 9);
});

test("obstacle data is validated; the snapshot is attributed and lies in the route box", () => {
  const b = data.bounds,
    mast = { kind: "mast", height: 30, point: [40.29, 40.61] };
  assert.deepEqual(validateObstacles(undefined, b), []);
  assert.throws(() => validateObstacles([{ ...mast, kind: "kite" }], b), /bilinmeyen tür/);
  assert.throws(() => validateObstacles([{ ...mast, height: 0 }], b), /yükseklik/);
  assert.throws(() => validateObstacles([{ ...mast, line: [[40.29, 40.61], [40.3, 40.61]] }], b), /tek bir/);
  assert.throws(() => validateObstacles([{ ...mast, point: [41, 40.61] }], b), /dışında/);
  assert.deepEqual(validateObstacles([{ ...mast, point: [41, 40.61] }, mast], b, { drop: true }), [mast]);
  assert.equal(snapshot.license, "ODbL 1.0");
  assert.match(snapshot.attribution, /OpenStreetMap/);
  assert.deepEqual(snapshot.bounds, data.bounds);
  assert.equal(validateObstacles(snapshot.obstacles, b).length, snapshot.obstacles.length);
  const counts = {};
  for (const o of snapshot.obstacles) counts[o.kind] = (counts[o.kind] ?? 0) + 1;
  assert.deepEqual(counts, snapshot.counts);
  assert.ok(counts.building > 500 && counts.forest > 10);
});

test("footprint distances: inside a polygon is zero, points and lines count their radius", () => {
  const toLocal = ({ longitude, latitude }) => [longitude, latitude];
  const index = obstacleIndex(
    [
      { kind: "building", height: 10, polygon: [[0, 0], [20, 0], [20, 10], [0, 10]] },
      { kind: "mast", height: 40, point: [100, 0], radius: 2 },
      { kind: "tree_row", height: 15, line: [[0, 50], [40, 50]], radius: 5 },
    ],
    toLocal,
    25,
  );
  const [building, mast, row] = index.items;
  assert.deepEqual(footprintDistance(building, [5, 5]), { distance: 0, nearest: [5, 5] });
  near(footprintDistance(building, [25, 5]).distance, 5);
  near(footprintDistance(mast, [110, 0]).distance, 8);
  assert.deepEqual(footprintDistance(mast, [110, 0]).nearest, [102, 0]);
  near(footprintDistance(row, [20, 60]).distance, 5);
  assert.equal(obstacleAt(index, [5, 5]), building);
  assert.equal(obstacleAt(index, [30, 30]), null);
  assert.deepEqual(
    obstaclesNear(index, [50, 25], 35).map((h) => h.obstacle.kind).sort(),
    ["building", "tree_row"],
  );
  assert.equal(index.tallest, 40);
});

test("the profile climbs over a tall building on the path and the flown trace keeps the clearance", () => {
  const base = plan({}, data, {}),
    table = base.path.table,
    knot = table[Math.floor(table.length / 3)],
    // A 70 m block, 20 m square, centred on the path a third of the way.
    corner = (dx, dy) => {
      const g = toGeodetic(C, base, [knot.p[0] + dx, knot.p[1] + dy, 0]);
      return [g.longitude, g.latitude];
    },
    block = {
      name: "Deneme kulesi",
      kind: "building",
      height: 70,
      polygon: [corner(-10, -10), corner(10, -10), corner(10, 10), corner(-10, 10)],
    };
  const raised = fly(plan({}, { ...data, obstacles: [block] }, {})),
    plain = fly(base);
  const i = Math.floor(table.length / 3),
    ground = raised.terrain.find((x) => x.index >= i).floor;
  // Above the block: its 70 m plus the 20 m clearance, not the 80 m AGL.
  assert.ok(raised.profile[i] >= ground + 70 + 20 - 0.5, `${raised.profile[i] - ground}`);
  assert.ok(plain.profile[i] < ground + 85);
  near(raised.plannedObstacleClearance.clearance, 20, 1);
  assert.equal(raised.plannedObstacleClearance.obstacle.name, "Deneme kulesi");
  // Elsewhere the profile is unchanged.
  near(raised.profile[table.length - 50], plain.profile[table.length - 50], 1e-9);
  const flown = raised.obstacleClearance;
  assert.ok(flown.clearance > 15 && flown.clearance < 25, `${flown.clearance}`);
  const item = check(analyzeMission(C, raised), "obstacles");
  assert.ok(item.ok && item.blocking, item.detail);
  assert.match(item.detail, /Deneme kulesi \(bina\) · 70 m \(dosya\)/);
  // Without the climb (the same block, profile planned without it) the
  // flown trace passes 10 m over the roof: the checklist blocks it.
  const blind = attachSimulation({ ...base, obstacles: raised.obstacles, obstacleData: raised.obstacleData }, plain.simulation);
  const blocked = analyzeMission(C, blind);
  assert.equal(check(blocked, "obstacles").ok, false);
  assert.equal(blocked.blocked, true);
});

test("on the OSM snapshot: the default flight clears forest canopy and buildings, a low flight is lifted over the canopy", () => {
  const normal = fly(plan()),
    analysis = analyzeMission(C, normal),
    item = check(analysis, "obstacles");
  assert.ok(item.ok, item.detail);
  assert.match(item.detail, /OSM 20\d\d-\d\d-\d\d/);
  // 80 m AGL over 30 m canopy.
  near(normal.obstacleClearance.clearance, 50, 1);
  assert.equal(normal.obstacleClearance.obstacle.kind, "forest");
  const sites = check(analysis, "sites");
  assert.ok(sites.ok, sites.detail);
  assert.match(sites.detail, /^3 alan · en yakın engel kalkış: bina/);
  // At 30 m AGL the profile rises to canopy + 20 m over the forest.
  const low = fly(plan({ agl: 30 }));
  near(low.obstacleClearance.clearance, 20, 1);
  assert.ok(low.maxAgl > 49, `${low.maxAgl}`);
  assert.ok(check(analyzeMission(C, low), "obstacles").ok);
  // No snapshot and no obstacles in the route file: an advisory.
  const none = check(analyzeMission(C, fly(plan({}, data, {}))), "obstacles");
  assert.equal(none.ok, false);
  assert.equal(none.blocking, false);
  assert.match(none.detail, /engel verisi yok/);
});

test("a failsafe landing in place onto trees is flagged", () => {
  const mission = plan(),
    armed = {
      failsafe: true,
      settings: mission.settings,
      terrain: terrainForWorker(mission),
      rally: mission.rally.map(({ name, xy }) => ({ name, xy })),
    },
    nominal = simulate(mission.reference),
    lost = simulate(mission.reference, { ...armed, faults: [{ type: "gnss-loss", time: 300 }] });
  assert.equal(lost.metrics.landingSite, "field");
  const clear = analyzeMission(C, attachSimulation({ ...mission }, lost, nominal));
  assert.ok(check(clear, "fault-landing").ok, check(clear, "fault-landing").detail);
  // The same landing with a stand of trees mapped around the spot.
  const [x, y] = lost.metrics.landingPosition,
    corner = (dx, dy) => {
      const g = toGeodetic(C, mission, [x + dx, y + dy, 0]);
      return [g.longitude, g.latitude];
    },
    trees = obstacleIndex(
      [
        {
          kind: "forest",
          height: 30,
          heightSource: "assumed",
          polygon: [corner(-30, -30), corner(30, -30), corner(30, 30), corner(-30, 30)],
        },
      ],
      (p) => {
        const local = C.Matrix4.multiplyByPoint(
          C.Matrix4.inverseTransformation(mission.frame, new C.Matrix4()),
          C.Cartesian3.fromDegrees(p.longitude, p.latitude, mission.groundHeight),
          new C.Cartesian3(),
        );
        return [local.x, local.y];
      },
    );
  const wooded = analyzeMission(
    C,
    attachSimulation({ ...mission, obstacles: trees }, lost, nominal),
  );
  const landing = check(wooded, "fault-landing");
  assert.equal(landing.ok, false);
  assert.match(landing.detail, /⚠ engel üzerine: orman örtüsü · 30 m \(kabul\)/);
  // Descending into the canopy also breaks the obstacle clearance.
  assert.equal(check(wooded, "fault-obstacles").ok, false);
});
