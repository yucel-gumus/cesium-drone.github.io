import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import * as C from "cesium";
import {
  createTerrainRoute,
  buildMission,
  attachSimulation,
  terrainFollowingProfile,
  FLIGHT_DEFAULTS,
} from "../src/flight-planner.mjs";
import { referenceAt } from "../src/trajectory.mjs";
import { simulate, AIRFRAME, STRIDE, shaftPower } from "../src/quadrotor.mjs";
import { polynomial } from "../src/flight-math.mjs";
import {
  BATTERY,
  cellOpenCircuitVoltage,
  batteryTerminal,
  dischargeStep,
  electricalPower,
  voltageAuthority,
} from "../src/battery.mjs";
import { makeNavigation, GNSS_MODES } from "../src/navigation.mjs";
import {
  CAMERA,
  fieldOfView,
  groundSampleDistance,
  footprint,
  alongTrackExtent,
  triggerDistance,
} from "../src/payload.mjs";
import { analyzeMission } from "../src/mission-analysis.mjs";
import {
  toKML,
  toQGCPlan,
  toGeoJSON,
  toCSV,
  CSV_HEADER,
  simplifyPath,
} from "../src/exports.mjs";

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
function plan(heightAt, options = {}, simOptions = {}) {
  const route = createTerrainRoute(C, data, options, { obstacles });
  route.samples.forEach((p, i) => (p.height = heightAt(p, i)));
  const mission = buildMission(C, route);
  attachSimulation(mission, simulate(mission.reference, simOptions));
  return mission;
}
const ridge = plan(hilly);

test("terrain-following profile never dips below floor + AGL and respects climb/descent slopes", () => {
  const s = FLIGHT_DEFAULTS,
    n = 600,
    spacing = 2,
    requirement = new Float64Array(n);
  // A 40 m step up, a plateau and a cliff down.
  for (let i = 0; i < n; i++)
    requirement[i] = 100 + (i > 200 && i < 380 ? 40 : 0) + 3 * Math.sin(i / 7);
  const z = terrainFollowingProfile(requirement, spacing, s);
  const up = (s.followClimbRate / s.cruiseSpeed) * spacing,
    down = (s.followDescentRate / s.cruiseSpeed) * spacing;
  for (let i = 0; i < n; i++) assert.ok(z[i] >= requirement[i] - 1e-9);
  for (let i = 1; i < n; i++) {
    assert.ok(z[i] - z[i - 1] <= up + 1e-9);
    assert.ok(z[i - 1] - z[i] <= down + 1e-9);
  }
  // Look-ahead: the climb is already under way before the step.
  assert.ok(z[195] > requirement[195] + 5);
  // Smooth: bounded second difference.
  for (let i = 1; i < n - 1; i++)
    assert.ok(Math.abs(z[i + 1] - 2 * z[i] + z[i - 1]) < 0.05);
});

test("ridge mission: 3D reference is C2, within vertical limits and flown above the floor", () => {
  const ref = ridge.reference,
    s = ridge.settings;
  let climb = 0,
    descent = 0;
  for (let i = 0; i < ref.horizontal.length; i++) {
    const seg = ref.horizontal[i];
    for (let k = 0; k <= 20; k++) {
      const z = polynomial(seg.coefficients[2], k / 20, seg.duration);
      climb = Math.max(climb, z.v);
      descent = Math.max(descent, -z.v);
      assert.ok(Math.abs(z.a) <= s.verticalAcceleration + 1e-6);
      assert.ok(Math.abs(z.j) <= s.jerk + 1e-6);
    }
    if (i) {
      const a = polynomial(ref.horizontal[i - 1].coefficients[2], 1, ref.horizontal[i - 1].duration),
        b = polynomial(seg.coefficients[2], 0, seg.duration);
      for (const key of ["p", "v", "a"]) near(a[key], b[key], 1e-6);
    }
  }
  assert.ok(climb > 0.5 && climb <= s.followClimbRate + 1e-6);
  assert.ok(descent > 0.5 && descent <= s.followDescentRate + 1e-6);
  for (let i = 0; i < ridge.profile.length; i++)
    assert.ok(ridge.profile[i] >= ridge.requirement[i] - 1e-9);
  assert.ok(ridge.minClearance >= s.agl - s.clearanceTolerance);
  assert.ok(ridge.maxAgl > s.agl && ridge.maxAgl <= s.maxAgl);
  assert.ok(ridge.simulation.metrics.maxTrackingError < 1);
  // Survey reference height really changes with the terrain.
  const zs = [];
  for (let t = ref.surveyStart; t < ref.surveyStart + ref.horizontalDuration; t += 5)
    zs.push(referenceAt(ref, t).p[2]);
  assert.ok(Math.max(...zs) - Math.min(...zs) > 40);
});

test("LiPo pack: Thevenin terminal state satisfies P = V·I and V = E − I·R", () => {
  near(cellOpenCircuitVoltage(1), 4.2);
  near(cellOpenCircuitVoltage(0), 3.27);
  for (let s = 0.05; s <= 1; s += 0.05)
    assert.ok(cellOpenCircuitVoltage(s) > cellOpenCircuitVoltage(s - 0.05));
  for (const soc of [1, 0.6, 0.3]) {
    const p = 300,
      t = batteryTerminal(p, soc);
    near(t.voltage * t.current, p, 1e-6);
    near(t.voltage, t.openCircuit - t.current * BATTERY.internalResistance, 1e-9);
    assert.equal(t.limited, false);
  }
  const overload = batteryTerminal(1e6, 0.5);
  assert.equal(overload.limited, true);
  near(overload.voltage, overload.openCircuit / 2, 1e-9);
  // One hour at 1 C empties the pack.
  near(dischargeStep(1, BATTERY.capacityAh, 3600), 0, 1e-12);
  near(voltageAuthority(BATTERY.cells * 4.2), 1);
  assert.ok(voltageAuthority(22) < 0.9);
});

test("energy: 2 kg hover draws 230–320 W and the record integrates current like a Coulomb counter", () => {
  const c = AIRFRAME,
    k = (c.thrustCoefficient * c.airDensity) / 1.225,
    w = Math.sqrt((c.mass * c.gravity) / (4 * k)),
    hover = electricalPower(shaftPower([w, w, w, w], c));
  assert.ok(hover > 230 && hover < 320, `hover ${hover} W`);
  const sim = ridge.simulation,
    m = sim.metrics;
  let charge = 0,
    energy = 0;
  for (let i = 1; i < sim.rows; i++) {
    const a = (i - 1) * STRIDE,
      b = i * STRIDE,
      dt = sim.data[b] - sim.data[a];
    charge += ((sim.data[a + 38] + sim.data[b + 38]) / 2) * dt;
    energy += ((sim.data[a + 37] * sim.data[a + 38] + sim.data[b + 37] * sim.data[b + 38]) / 2) * dt;
  }
  near((1 - m.finalSoc) * BATTERY.capacityAh * 3600, charge, charge * 0.005);
  near(m.energyWh, energy / 3600, m.energyWh * 0.005);
  assert.ok(m.finalSoc > BATTERY.reserve && m.finalSoc < 0.7);
  assert.ok(m.minVoltage > BATTERY.cells * BATTERY.minCellVoltage);
});

test("heavier airframe needs more energy and falls below the landing reserve at 3 kg", () => {
  const heavy = plan(() => 1150, {}, {
    airframe: { ...AIRFRAME, mass: 3, inertia: AIRFRAME.inertia.map((x) => x * 1.5) },
  });
  const light = plan(() => 1150);
  assert.ok(heavy.simulation.metrics.energyWh > 1.5 * light.simulation.metrics.energyWh);
  const result = analyzeMission(C, heavy);
  assert.equal(result.blocked, true);
  assert.equal(result.checks.find((c) => c.id === "reserve").ok, false);
  assert.equal(analyzeMission(C, light).blocked, false);
});

test("navigation filter: ideal is truth, RTK holds centimetres, standard GNSS drifts metres", () => {
  const ideal = makeNavigation("ideal");
  ideal.step([1, 2, 3], [4, 5, 6], [0, 0, 0], 1 / 120);
  assert.deepEqual(ideal.position, [1, 2, 3]);
  const errors = {};
  for (const mode of ["rtk", "standard"]) {
    const nav = makeNavigation(mode, 7);
    let worst = 0;
    // A vehicle circling at 8 m/s for five minutes.
    for (let k = 0; k < 120 * 300; k++) {
      const t = k / 120,
        w = 8 / 100,
        p = [100 * Math.cos(w * t), 100 * Math.sin(w * t), 80],
        v = [-8 * Math.sin(w * t), 8 * Math.cos(w * t), 0],
        a = [-8 * w * Math.cos(w * t), -8 * w * Math.sin(w * t), 0];
      if (k === 0) nav.step(p, v, a, 1 / 120);
      nav.step(p, v, a, 1 / 120);
      if (t > 10)
        worst = Math.max(worst, Math.hypot(...nav.position.map((x, i) => x - p[i])));
    }
    errors[mode] = { worst, eph: nav.eph };
  }
  assert.ok(errors.rtk.worst < 0.1, JSON.stringify(errors));
  assert.ok(errors.standard.worst > 0.3 && errors.standard.worst < 5, JSON.stringify(errors));
  assert.ok(errors.standard.eph > 10 * errors.rtk.eph);
  assert.ok(GNSS_MODES.rtk.horizontalNoise < GNSS_MODES.standard.horizontalNoise);
});

test("standard GNSS mission still lands by the land detector, metres off the pad", () => {
  const m = plan(() => 1150, {}, { gnss: "standard" });
  const s = m.simulation.metrics;
  assert.equal(s.disarmed, true);
  assert.ok(s.landingError > 0.05 && s.landingError < 3, `${s.landingError}`);
  assert.ok(s.maxNavigationError > 0.3);
  // Same seed, same sensors: deterministic.
  const again = simulate(m.reference, { gnss: "standard" });
  assert.deepEqual(again.data, m.simulation.data);
});

test("camera geometry: GSD, nadir footprint and oblique along-track extent", () => {
  const fov = fieldOfView();
  near((fov.horizontal * 180) / Math.PI, 73.74, 0.01);
  near((fov.vertical * 180) / Math.PI, 53.13, 0.01);
  near(groundSampleDistance(80), (80 * 13.2) / (8.8 * 5472), 1e-12);
  const f = footprint([0, 0, 80], 0, -Math.PI / 2, 0);
  const xs = f.corners.map((c) => c[0]),
    ys = f.corners.map((c) => c[1]);
  // Long side across track (east for a northbound heading).
  near(Math.max(...xs) - Math.min(...xs), (80 * CAMERA.sensorWidth) / CAMERA.focalLength, 1e-9);
  near(Math.max(...ys) - Math.min(...ys), (80 * CAMERA.sensorHeight) / CAMERA.focalLength, 1e-9);
  near(alongTrackExtent(80, -90), 80, 1e-9);
  // −45°: edges 26.57° either side of the axis, so 80/tan(18.43°) − 80/tan(71.57°).
  near(alongTrackExtent(80, -45), 80 * 3 - 80 / 3, 1e-6);
  near(triggerDistance(80, -90, 0.8), 16, 1e-9);
  // Oblique footprint lies ahead of the camera.
  const oblique = footprint([0, 0, 80], 0, -Math.PI / 4, 0);
  assert.ok(oblique.corners.every((c) => c[1] > 0));
});

test("photos are triggered by distance with the planned forward overlap and no motion blur", () => {
  const result = analyzeMission(C, ridge),
    { photos, spacing } = result.capture;
  near(spacing, triggerDistance(ridge.settings.agl, -90, 0.8), 1e-9);
  assert.ok(photos.length > 250);
  for (let i = 1; i < photos.length; i++) {
    assert.ok(photos[i].time > photos[i - 1].time);
    near(photos[i].distance - photos[i - 1].distance, spacing, 1e-6);
  }
  assert.ok(result.coverage.minOverlap >= 0.75);
  assert.ok(result.coverage.maxBlur < 0.5);
  assert.ok(result.coverage.gsdMin >= groundSampleDistance(ridge.settings.agl - 1));
  // Oblique imaging: longer spacing, coarser GSD.
  const oblique = analyzeMission(C, ridge, { gimbalPitch: -45, frontOverlap: 0.7 });
  assert.ok(oblique.capture.spacing > 3 * spacing);
  assert.ok(oblique.coverage.gsdMean > result.coverage.gsdMean);
});

test("preflight checklist passes the default mission and the timeline is ordered", () => {
  const result = analyzeMission(C, ridge);
  assert.equal(result.blocked, false);
  for (const c of result.checks) assert.ok(c.ok, `${c.label}: ${c.detail}`);
  const labels = result.events.map((e) => e.label);
  // The mission happens in this order.
  const sequence = [
    "Motorlar hazırlanıyor",
    "Kalkış",
    "Görev irtifası",
    "İlk fotoğraf",
    "Dönüş bacağı",
    "Son fotoğraf",
    "Ev noktası üzerinde",
    "İniş alçalması",
    "Son yaklaşma",
    "Zemin teması",
    "Motorlar durdu",
    "Görev tamamlandı",
  ].map((label) => labels.indexOf(label));
  assert.ok(sequence.every((i) => i >= 0), JSON.stringify(labels));
  for (let i = 1; i < sequence.length; i++)
    assert.ok(sequence[i] > sequence[i - 1], JSON.stringify(labels));
});

test("regulatory ceiling blocks a plan that would exceed 120 m AGL", () => {
  const high = plan(hilly, { agl: 118 });
  const result = analyzeMission(C, high);
  assert.ok(high.maxAgl > 120);
  assert.equal(result.checks.find((c) => c.id === "ceiling").ok, false);
  assert.equal(result.blocked, true);
});

test("exports: KML, QGroundControl plan, GeoJSON and CSV are complete and consistent", () => {
  const result = analyzeMission(C, ridge);
  const kml = toKML(C, ridge, result);
  assert.ok(kml.startsWith('<?xml version="1.0" encoding="UTF-8"?>'));
  for (const tag of ["Placemark", "Folder", "Document", "LineString", "Polygon"])
    assert.equal(
      kml.split(`<${tag}>`).length,
      kml.split(`</${tag}>`).length,
      tag,
    );
  // Home, planned route, flown track, the rally points and every photo.
  assert.equal(
    kml.split("<Placemark>").length - 1,
    result.capture.photos.length + 3 + ridge.rally.length,
  );
  const plan = toQGCPlan(C, ridge, result);
  const items = plan.mission.items;
  assert.equal(plan.fileType, "Plan");
  assert.equal(items[0].command, 22);
  assert.equal(items.at(-1).command, 20);
  items.forEach((item, i) => assert.equal(item.doJumpId, i + 1));
  const waypoints = items.filter((i) => i.command === 16);
  assert.ok(waypoints.length > 20);
  const b = ridge.bounds,
    offset = ridge.settings.landingOffset,
    // Geodetic height differences add the Earth's curvature (< 0.5 m here).
    lowest = Math.min(...ridge.profile) + offset,
    highest = Math.max(...ridge.profile) + offset + 0.5;
  for (const w of waypoints) {
    const [, , , , lat, lon, alt] = w.params;
    assert.ok(lat > b.south && lat < b.north && lon > b.west && lon < b.east);
    // Heights relative to home follow the terrain-following profile, which
    // dips below home where the valley does.
    assert.ok(alt >= lowest - 0.01 && alt <= highest + 0.01);
  }
  assert.equal(items.find((i) => i.command === 206).params[0], Number(result.capture.spacing.toFixed(2)));
  const geo = toGeoJSON(C, ridge, result);
  assert.equal(geo.type, "FeatureCollection");
  const footprints = geo.features.filter((f) => f.properties.kind === "footprint");
  assert.equal(footprints.length, result.capture.photos.length);
  for (const f of footprints) {
    const ring = f.geometry.coordinates[0];
    assert.equal(ring.length, 5);
    assert.deepEqual(ring[0], ring[4]);
  }
  const csv = toCSV(ridge).split("\n");
  assert.equal(csv[0].split(",").length, CSV_HEADER.length);
  assert.equal(csv[1].split(",").length, CSV_HEADER.length);
  assert.ok(csv.length > 9000);
});

test("Ramer–Douglas–Peucker keeps every point within tolerance", () => {
  const points = Array.from({ length: 200 }, (_, i) => [i, 5 * Math.sin(i / 20), 0]);
  const keep = simplifyPath(points, 0.5);
  assert.ok(keep.length < 60 && keep[0] === 0 && keep.at(-1) === 199);
  for (let k = 1; k < keep.length; k++) {
    const a = points[keep[k - 1]],
      b = points[keep[k]];
    for (let i = keep[k - 1] + 1; i < keep[k]; i++) {
      const f = (i - keep[k - 1]) / (keep[k] - keep[k - 1]),
        y = a[1] + (b[1] - a[1]) * f;
      assert.ok(Math.abs(points[i][1] - y) < 0.6);
    }
  }
});

// First recorded row on the pad after the landing descent began.
function contactTime(sim) {
  for (let i = 0; i < sim.rows; i++) {
    const r = i * STRIDE;
    if (sim.data[r + 30] >= 6 && sim.data[r + 3] <= 0.002) return sim.data[r];
  }
  return Infinity;
}

test("land detector disarms on the pad within two seconds of contact, never in the air", () => {
  for (const gnss of ["rtk", "standard"])
    for (const seed of [1, 2, 3]) {
      const sim = simulate(ridge.reference, { gnss, seed }),
        contact = contactTime(sim),
        disarm = sim.metrics.disarmTime;
      assert.ok(Number.isFinite(contact), `${gnss}/${seed} no contact`);
      assert.ok(disarm >= contact, `${gnss}/${seed} disarmed in the air`);
      assert.ok(disarm - contact < 2, `${gnss}/${seed} late by ${disarm - contact} s`);
    }
});

test("an offset navigation fix cannot tip the airframe on the pad, saturate the motors or wind up a lift-off jump", () => {
  // Standard GNSS starts metres off. On the pad the airframe stays level;
  // after lift-off it moves at most by the navigation offset.
  const sim = simulate(ridge.reference, { gnss: "standard", seed: 7 });
  let tiltArming = 0,
    tiltLiftoff = 0,
    drift = 0;
  for (let i = 0; i < sim.rows; i++) {
    const r = i * STRIDE,
      phase = sim.data[r + 30],
      z = sim.data[r + 3],
      qx = sim.data[r + 8],
      qy = sim.data[r + 9],
      tilt = Math.acos(Math.min(1, 1 - 2 * (qx * qx + qy * qy)));
    if (phase === 0) tiltArming = Math.max(tiltArming, tilt);
    if (phase === 1 && z < 0.01) tiltLiftoff = Math.max(tiltLiftoff, tilt);
    if (phase === 1 && z < 10)
      drift = Math.max(drift, Math.hypot(sim.data[r + 1], sim.data[r + 2]));
  }
  // Pressed onto the pad the gear holds it level; leaving it, the position
  // loop only leans into the wind.
  assert.ok(tiltArming < 1e-9, `tilt while arming ${tiltArming}`);
  assert.ok(tiltLiftoff < (5 * Math.PI) / 180, `tilt at lift-off ${tiltLiftoff}`);
  assert.ok(drift < 2.5, `drift below 10 m: ${drift}`);
  assert.ok(sim.metrics.saturationSeconds <= 0.5);
  assert.ok(sim.metrics.maxTrackingError < 3);
});

test("an airframe that runs out of authority before touchdown is blocked by the checklist, not reported as a solver failure", () => {
  const mass = 3,
    heavy = { ...AIRFRAME, mass, inertia: AIRFRAME.inertia.map((x) => (x * mass) / 2) },
    // A pack just large enough to reach home: the sagging voltage costs rotor
    // authority on the final descent and the landing does not settle.
    mission = plan(hilly, {}, { airframe: heavy, battery: { ...BATTERY, capacityAh: 5.75 } });
  const m = mission.simulation.metrics;
  assert.equal(m.landingCompleted, false);
  assert.ok(m.finalSoc < 0.05 || m.saturationSeconds > 0.5, `${m.finalSoc} ${m.saturationSeconds}`);
  const analysis = analyzeMission(C, mission),
    landing = analysis.checks.find((c) => c.id === "landing");
  assert.equal(analysis.blocked, true);
  assert.equal(landing.ok, false);
  assert.equal(landing.blocking, true);
  assert.match(landing.detail, /^iniş tamamlanamadı/);
});
