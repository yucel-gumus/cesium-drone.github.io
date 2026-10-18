import { roundedPath, makeReference, JERK_SMOOTHING } from "./trajectory.mjs";
import { replayAt, STRIDE, PHASE_NAMES } from "./quadrotor.mjs";
import { rotate, clamp } from "./flight-math.mjs";
import {
  terrainFollowingProfile,
  terrainUnder,
  rasterLayout,
  indexStations,
} from "./terrain.mjs";
import {
  SURVEY_DEFAULTS,
  validateSurveyArea,
  preparePolygon,
  polygonArea,
  lawnmower,
  partitionLines,
} from "./survey.mjs";
import { acrossTrackWidth } from "./camera.mjs";
import {
  OBSTACLE_DEFAULTS,
  validateObstacles,
  obstacleIndex,
  obstaclesNear,
  tallestNear,
} from "./obstacles.mjs";
export { terrainFollowingProfile } from "./terrain.mjs";
export const FLIGHT_DEFAULTS = Object.freeze({
  // Terrain following: nominal height above the highest ground across the
  // sampled corridor, and the regulatory ceiling above the ground below.
  agl: 80,
  maxAgl: 120,
  // Flown clearance may dip this far below `agl` (tracking, gusts) before
  // the preflight check fails.
  clearanceTolerance: 5,
  // Look-ahead window (dilation) and Gaussian σ of the height profile, metres.
  // Smoothing support (3σ) must stay inside the dilation window, which keeps
  // the smoothed profile above the requirement everywhere.
  profileDilation: 40,
  profileSmoothing: 12,
  // Climb/descent rates while following terrain in forward flight; the
  // vertical take-off and landing rates are climbSpeed and descentSpeed.
  followClimbRate: 3,
  followDescentRate: 2.5,
  verticalAcceleration: 1,
  sampleSpacing: 6,
  corridorHalfWidth: 12,
  cruiseSpeed: 8,
  climbSpeed: 2.5,
  descentSpeed: 1.5,
  acceleration: 1.5,
  lateralAcceleration: 1.2,
  jerk: 2,
  landingOffset: 0.1,
  landingCrawlHeight: 3,
  landingSpeed: 0.35,
  landingOvershoot: 2.5,
  turnTrim: 45,
  wind: 2,
  // Elevation raster over the route's convex hull for failsafe legs that
  // leave the planned corridor: node spacing and margin around the hull, m.
  rasterSpacing: 25,
  rasterMargin: 50,
  // Obstacles (buildings, masts, trees): vertical clearance over their
  // tops within a horizontal buffer, m (see obstacles.mjs).
  obstacleClearance: OBSTACLE_DEFAULTS.clearance,
  obstacleLateral: OBSTACLE_DEFAULTS.lateral,
});
export const MISSION_TYPES = Object.freeze(["recon", "survey"]);
// Largest route box (degrees) the single launch-point ENU tangent plane and
// the flat-earth corridor geometry are allowed to span.
export const MAX_BOUNDS_SPAN = 0.5;
// Largest plausible geoid undulation magnitude (metres); EGM96 stays within
// about -107…+86 m worldwide.
export const MAX_GEOID_UNDULATION = 120;
export function validateBounds(data) {
  const b = data?.bounds;
  if (
    !b ||
    typeof b !== "object" ||
    ["west", "east", "south", "north"].some((k) => !Number.isFinite(b[k]))
  )
    throw new Error(
      "Rota dosyasında bounds nesnesi (west, east, south, north) zorunlu.",
    );
  if (b.west >= b.east || b.south >= b.north)
    throw new Error("Rota sınırları: west < east ve south < north olmalı.");
  if (b.west < -180 || b.east > 180 || b.south < -90 || b.north > 90)
    throw new Error(
      "Rota sınırları: boylam −180…180, enlem −90…90 derece olmalı.",
    );
  // Tolerate floating-point noise so a nominally 0.5° wide box is accepted
  // wherever it sits on the globe (1.1 − 0.6 evaluates to 0.5000000000000001).
  if (
    b.east - b.west > MAX_BOUNDS_SPAN + 1e-9 ||
    b.north - b.south > MAX_BOUNDS_SPAN + 1e-9
  )
    throw new Error("Rota sınırları: kutu en fazla 0,5° × 0,5° olabilir.");
  return { west: b.west, east: b.east, south: b.south, north: b.north };
}
// Geoid undulation N (metres) of the route area; MSL ≈ ellipsoid height − N.
// Optional: null when the route file omits it, so the app still runs.
export function validateGeoidUndulation(data) {
  const n = data?.geoidUndulation;
  if (n === undefined || n === null) return null;
  if (
    typeof n !== "number" ||
    !Number.isFinite(n) ||
    Math.abs(n) > MAX_GEOID_UNDULATION
  )
    throw new Error("geoidUndulation sonlu bir sayı ve ±120 m içinde olmalı.");
  return n;
}
export function validateWaypoints(data) {
  const bounds = validateBounds(data);
  if (
    !Array.isArray(data?.dronePoints) ||
    data.dronePoints.length < 3 ||
    data.dronePoints.length > 200
  )
    throw new Error("Rota 3–200 nokta içermeli.");
  const points = data.dronePoints.map((p, i) => {
    if (!p || !Number.isFinite(p.longitude) || !Number.isFinite(p.latitude))
      throw new Error(`Rota noktası ${i + 1}: geçersiz koordinat.`);
    if (
      p.longitude < bounds.west ||
      p.longitude > bounds.east ||
      p.latitude < bounds.south ||
      p.latitude > bounds.north
    )
      throw new Error(`Rota noktası ${i + 1}: rota sınırlarının dışında.`);
    return { longitude: p.longitude, latitude: p.latitude };
  });
  if (
    points[0].longitude !== points.at(-1).longitude ||
    points[0].latitude !== points.at(-1).latitude
  )
    points.push({ ...points[0] });
  return points;
}
// `task` selects the mission: { type: "recon" } flies the waypoint loop of
// the route file, { type: "survey", sideOverlap, gimbalPitch } a lawnmower
// over its `surveyArea` polygon, with the line spacing taken from the
// camera's across-track footprint at the target AGL. `task.obstacles` is
// an obstacle snapshot ({ obstacles, bounds, timestamp, source }, see
// public/obstacles.json), added to the route file's own `obstacles`.
// An area survey split into sorties that each fit one battery: the grid's
// lines are partitioned into contiguous blocks whose estimated flight time
// stays within `maxSeconds` (see partitionLines()), and each block becomes
// its own route from and back to home. One route when the grid fits.
export function surveySorties(C, data, options = {}, task = {}, maxSeconds = Infinity) {
  const full = createTerrainRoute(C, data, options, { ...task, type: "survey" }),
    s = full.survey,
    blocks = partitionLines(
      { lines: s.allLines, direction: s.direction, normal: s.normal },
      s.waypoints[0],
      full.settings,
      maxSeconds,
    );
  if (blocks.length === 1) return [full];
  return blocks.map((lines, index) =>
    createTerrainRoute(C, data, options, {
      ...task,
      type: "survey",
      lines,
      sortie: { index, count: blocks.length },
    }),
  );
}
// Flight time a battery allows with the landing reserve and a 5 % margin,
// from the airframe's hover power (cruise adds about 2 % in this model).
export function sortieLimit(airframe, battery) {
  const k = (airframe.thrustCoefficient * airframe.airDensity) / 1.225,
    rotor = Math.sqrt((airframe.mass * airframe.gravity) / (4 * k)),
    shaft = 4 * k * airframe.reactionRatio * rotor ** 3,
    power = shaft / battery.propulsionEfficiency + battery.avionicsPower,
    usable = (1 - battery.reserve - 0.05) * battery.cells * 3.7 * battery.capacityAh;
  return (usable * 3600) / (1.02 * power);
}
// Optional `rallyPoints` of the route file: up to ten named safe landing
// points (WGS84, inside the bounds) a failsafe return may choose instead of
// home. Returns [] when the file has none.
export function validateRallyPoints(data, bounds) {
  const points = data?.rallyPoints;
  if (points === undefined || points === null) return [];
  if (!Array.isArray(points) || points.length > 10)
    throw new Error("rallyPoints en fazla 10 nokta içermeli.");
  return points.map((p, i) => {
    if (!p || !Number.isFinite(p.longitude) || !Number.isFinite(p.latitude))
      throw new Error(`Rally noktası ${i + 1}: geçersiz koordinat.`);
    if (
      p.longitude < bounds.west ||
      p.longitude > bounds.east ||
      p.latitude < bounds.south ||
      p.latitude > bounds.north
    )
      throw new Error(`Rally noktası ${i + 1}: rota sınırlarının dışında.`);
    return {
      name: typeof p.name === "string" && p.name ? p.name : `Rally ${i + 1}`,
      longitude: p.longitude,
      latitude: p.latitude,
    };
  });
}
export function createTerrainRoute(C, data, options = {}, task = {}) {
  const settings = { ...FLIGHT_DEFAULTS, ...options };
  for (const [key, value] of Object.entries(settings))
    if (!Number.isFinite(value) || value < 0 || (key !== "wind" && value === 0))
      throw new Error(`Geçersiz uçuş ayarı: ${key}`);
  if (
    settings.agl < 20 ||
    settings.agl > 150 ||
    3 * settings.profileSmoothing > settings.profileDilation ||
    settings.sampleSpacing > 20 ||
    settings.cruiseSpeed > 15 ||
    settings.wind > 8 ||
    settings.rasterSpacing > 50 ||
    settings.rasterMargin < settings.corridorHalfWidth + settings.rasterSpacing
  )
    throw new Error("Uçuş ayarı modelin sınırları dışında.");
  const type = task.type ?? "recon";
  if (!MISSION_TYPES.includes(type))
    throw new Error(`Bilinmeyen görev tipi: ${type}`);
  const points = validateWaypoints(data),
    bounds = validateBounds(data),
    geoidUndulation = validateGeoidUndulation(data),
    rallyPoints = validateRallyPoints(data, bounds),
    snapshot = task.obstacles ?? null,
    fromSnapshot = validateObstacles(snapshot?.obstacles, bounds, { drop: true }),
    fromRoute = validateObstacles(data.obstacles, bounds),
    obstacleList = [...fromSnapshot, ...fromRoute],
    home = C.Cartographic.fromDegrees(points[0].longitude, points[0].latitude);
  const zeroFrame = C.Transforms.eastNorthUpToFixedFrame(
    C.Cartesian3.fromRadians(home.longitude, home.latitude, 0),
  );
  const inverse = C.Matrix4.inverseTransformation(zeroFrame, new C.Matrix4());
  const toLocal = (p) => {
    const v = C.Matrix4.multiplyByPoint(
      inverse,
      C.Cartesian3.fromDegrees(p.longitude, p.latitude, 0),
      new C.Cartesian3(),
    );
    return [v.x, v.y];
  };
  const toCartographic = (x, y) =>
    C.Cartographic.fromCartesian(
      C.Matrix4.multiplyByPoint(
        zeroFrame,
        new C.Cartesian3(x, y, 0),
        new C.Cartesian3(),
      ),
    );
  const local = [];
  for (const p of points) {
    const v = toLocal(p);
    if (
      !local.length ||
      Math.hypot(v[0] - local.at(-1)[0], v[1] - local.at(-1)[1]) > 0.1
    )
      local.push(v);
  }
  if (local.length < 3) throw new Error("Rota en az üç farklı nokta içermeli.");
  let waypoints = local,
    survey = null;
  if (type === "survey") {
    const area = validateSurveyArea(data, bounds);
    if (!area)
      throw new Error(
        "Rota dosyasında surveyArea poligonu yok; alan haritalama planlanamaz.",
      );
    const sideOverlap = task.sideOverlap ?? SURVEY_DEFAULTS.sideOverlap,
      gimbalPitch = task.gimbalPitch ?? -90;
    if (
      !Number.isFinite(sideOverlap) ||
      sideOverlap < SURVEY_DEFAULTS.minSideOverlap ||
      sideOverlap > SURVEY_DEFAULTS.maxSideOverlap
    )
      throw new Error("Yan bindirme %60–80 arasında olmalı.");
    const corners = area.map(toLocal),
      polygon = preparePolygon(corners),
      swath = acrossTrackWidth(settings.agl, gimbalPitch),
      grid = lawnmower(polygon, (1 - sideOverlap) * swath, {
        home: local[0],
        lines: task.lines,
      });
    waypoints = grid.waypoints;
    survey = {
      ...grid,
      polygon,
      // WGS84 corners, counter-clockwise like the ENU polygon (RFC 7946).
      vertices: polygonArea(corners) > 0 ? area : area.slice().reverse(),
      areaM2: polygonArea(polygon),
      sideOverlap,
      gimbalPitch,
      swath,
      sortie: task.sortie ?? { index: 0, count: 1 },
    };
  }
  const path = roundedPath(waypoints, settings.turnTrim, 2, survey ? 0.5 : 0.24),
    samples = [],
    centers = [];
  if (survey) {
    // Imaging runs from the end of the turn onto the first line to the
    // start of the turn off the last one.
    const corner = (i) => path.corners.find((c) => c.waypoint === i);
    survey.legs = {
      start: corner(survey.gridStart).end,
      end: corner(survey.gridEnd).start,
    };
  }
  for (
    let i = 0;
    i < path.table.length;
    i += Math.max(1, Math.floor(settings.sampleSpacing / 2))
  ) {
    centers.push(i);
  }
  if (centers.at(-1) !== path.table.length - 1)
    centers.push(path.table.length - 1);
  for (const index of centers) {
    const k = path.table[index];
    for (const side of [0, -1, 1])
      samples.push(
        toCartographic(
          k.p[0] - k.tangent[1] * side * settings.corridorHalfWidth,
          k.p[1] + k.tangent[0] * side * settings.corridorHalfWidth,
        ),
      );
  }
  // The raster follows the corridor samples in `samples`; its hull takes in
  // the rally points, so a return to any of them is covered too.
  const rally = rallyPoints.map((p) => ({ ...p, xy: toLocal(p) })),
    corridorSamples = samples.length,
    raster = rasterLayout(
      [
        ...path.table.filter((_, i) => i % 5 === 0).map((k) => k.p),
        ...rally.map((p) => p.xy),
      ],
      settings.rasterSpacing,
      settings.rasterMargin,
    );
  for (const [x, y] of raster.nodes) samples.push(toCartographic(x, y));
  return {
    type,
    obstacles: obstacleIndex(obstacleList, toLocal),
    // Where the obstacles come from and whether the snapshot's area takes
    // in the whole route box.
    obstacleData: {
      snapshot: fromSnapshot.length,
      route: fromRoute.length,
      source: snapshot?.source ?? null,
      timestamp: snapshot?.timestamp ?? null,
      covers:
        !!snapshot?.bounds &&
        snapshot.bounds.west <= bounds.west &&
        snapshot.bounds.east >= bounds.east &&
        snapshot.bounds.south <= bounds.south &&
        snapshot.bounds.north >= bounds.north,
    },
    settings,
    bounds,
    geoidUndulation,
    samples,
    corridorSamples,
    raster,
    path,
    centers,
    home,
    survey,
    rally,
    horizontalDistance: path.length,
  };
}
// Samples the route's terrain. `cache` (a Map from a sample's radian
// coordinates to its height) lets re-plans query only new positions.
export async function sampleTerrainRoute(
  C,
  provider,
  route,
  onProgress = () => {},
  cache = null,
) {
  const key = (p) => `${p.longitude},${p.latitude}`;
  const missing = cache
    ? route.samples.filter((p) => !cache.has(key(p)))
    : route.samples;
  for (let offset = 0; offset < missing.length; offset += 192) {
    await C.sampleTerrainMostDetailed(
      provider,
      missing.slice(offset, offset + 192),
      true,
    );
    onProgress(Math.min(1, (offset + 192) / missing.length));
  }
  if (cache) {
    for (const p of missing)
      if (Number.isFinite(p.height)) cache.set(key(p), p.height);
    for (const p of route.samples) p.height = cache.get(key(p)) ?? NaN;
  }
  if (route.samples.some((p) => !Number.isFinite(p.height)))
    throw new Error("Eksik arazi yüksekliği; uçuş hazırlanmadı.");
  return route;
}
export function buildMission(C, route) {
  if (route.samples.some((p) => !Number.isFinite(p.height)))
    throw new Error("Eksik arazi yüksekliği.");
  const home = C.Cartographic.clone(route.samples[0]),
    groundHeight = home.height;
  const frame = C.Transforms.eastNorthUpToFixedFrame(
    C.Cartesian3.fromRadians(
      home.longitude,
      home.latitude,
      groundHeight + route.settings.landingOffset,
    ),
  );
  const inverse = C.Matrix4.inverseTransformation(frame, new C.Matrix4());
  const grounds = route.samples.map(
    (p) =>
      C.Matrix4.multiplyByPoint(
        inverse,
        C.Cartesian3.fromRadians(p.longitude, p.latitude, p.height),
        new C.Cartesian3(),
      ).z,
  );
  const s = route.settings,
    terrain = route.centers.map((index, i) => ({
      index,
      p: route.path.table[index].p,
      z: grounds[i * 3],
      floor: Math.max(...grounds.slice(i * 3, i * 3 + 3)),
      // Tallest obstacle across the corridor plus the lateral buffer.
      obstacle: tallestNear(
        route.obstacles,
        route.path.table[index].p,
        s.corridorHalfWidth + s.obstacleLateral,
      ),
    }));
  // Corridor floor at every path knot, linear between sampled stations,
  // plus the target AGL, or the obstacle clearance over the tallest
  // obstacle of either station where that is higher.
  const table = route.path.table,
    requirement = new Float64Array(table.length);
  for (let k = 1; k < terrain.length; k++) {
    const a = terrain[k - 1],
      b = terrain[k],
      above = Math.max(s.agl, Math.max(a.obstacle, b.obstacle) + s.obstacleClearance);
    for (let i = a.index; i <= b.index; i++) {
      const f = b.index > a.index ? (i - a.index) / (b.index - a.index) : 0;
      requirement[i] = a.floor + (b.floor - a.floor) * f + above;
    }
  }
  const spacing = table.length > 1 ? table[1].s - table[0].s : 1,
    profile = terrainFollowingProfile(requirement, spacing, route.settings),
    reference = makeReference(route.path, profile, route.settings, {
      legs: route.survey?.legs,
      jerkSmoothing: route.survey ? JERK_SMOOTHING : 0,
    });
  // Raster ground heights in the mission frame (NaN off the sampled hull).
  const layout = route.raster,
    heights = new Float64Array(layout.nx * layout.ny).fill(NaN);
  layout.slots.forEach((slot, i) => {
    if (slot >= 0) heights[i] = grounds[route.corridorSamples + slot];
  });
  const raster = {
    x0: layout.x0,
    y0: layout.y0,
    spacing: layout.spacing,
    nx: layout.nx,
    ny: layout.ny,
    heights,
  };
  const terrainModel = indexStations({
    stations: terrain,
    raster,
    halfWidth: route.settings.corridorHalfWidth,
    cell: 30,
  });
  // The planned path's clearance over obstacles within the lateral buffer.
  let planned = { clearance: Infinity, obstacle: null, s: 0 };
  for (const [i, k] of table.entries()) {
    if (i % 3) continue;
    for (const hit of obstaclesNear(route.obstacles, k.p, s.obstacleLateral)) {
      const c =
        profile[i] -
        terrainUnder(terrainModel, hit.nearest).height -
        hit.obstacle.height;
      if (c < planned.clearance)
        planned = { clearance: c, obstacle: hit.obstacle, s: k.s, nearest: hit.nearest };
    }
  }
  return {
    ...route,
    plannedObstacleClearance: planned,
    home,
    groundHeight,
    frame,
    reference,
    profile,
    requirement,
    duration: reference.duration,
    terrain,
    terrainModel,
    grid: terrainModel.grid,
    cell: terrainModel.cell,
  };
}
// Obstacles within `radius` of the planned path (for the map), each once
// with its footprint point nearest to the path and the modelled ground
// there (ENU z): long features (tree rows) may reach beyond the terrain
// model, their nearest point does not.
export function obstaclesAlongPath(mission, radius = 40) {
  const found = new Map();
  mission.path.table.forEach((k, i) => {
    if (i % 5) return;
    for (const hit of obstaclesNear(mission.obstacles, k.p, radius)) {
      const best = found.get(hit.obstacle);
      if (!best || hit.distance < best.distance) found.set(hit.obstacle, hit);
    }
  });
  return [...found.values()].map((hit) => ({
    obstacle: hit.obstacle,
    nearest: hit.nearest,
    ground: terrainAt(mission, hit.nearest).height,
  }));
}
// Ground under an ENU point: { height (centre-line ground, or the raster
// beside the corridor), floor (highest ground across the corridor),
// distance (to the nearest corridor station) }.
export function terrainAt(mission, p) {
  return terrainUnder(mission.terrainModel, p);
}
// The elevation model as plain data for the physics worker, which needs it
// to plan failsafe legs (direct return, landing in place).
export function terrainForWorker(mission) {
  const m = mission.terrainModel;
  return {
    stations: m.stations.map((x) => ({ p: x.p, z: x.z, floor: x.floor })),
    raster: m.raster,
    halfWidth: m.halfWidth,
    cell: m.cell,
  };
}
const AIRBORNE = new Set([
  "takeoff",
  "survey",
  "return",
  "settle",
  "landing",
  "transit",
  "hold",
  "rtl",
]);
// Phases flown along the planned path (checked against its corridor).
const ON_ROUTE = new Set(["survey", "return", "transit", "hold", "rtl"]);
// Measures a flown trace against the terrain: the lowest clearance over the
// corridor floor along the planned path, the highest height above the
// ground directly below at any airborne moment (regulatory ceiling) and the
// largest distance from the sampled centre line. A failsafe's direct return
// leg is measured against its own corridor (simulation.failsafe.rtl).
export function measureFlight(mission, simulation) {
  const rtl = simulation.failsafe?.rtl,
    lateral = mission.settings.obstacleLateral;
  let obstacle = { clearance: Infinity, obstacle: null, time: 0, distance: 0, nearest: null },
    nextObstacleCheck = 0;
  let minClearance = Infinity,
    maxAgl = 0,
    maxDeviation = 0,
    returnClearance = Infinity,
    returnDeviation = 0,
    knot = 0;
  for (let i = 0; i < simulation.rows; i++) {
    const r = i * STRIDE,
      name = PHASE_NAMES[simulation.data[r + 30]];
    if (!AIRBORNE.has(name)) continue;
    const p = [
        simulation.data[r + 1],
        simulation.data[r + 2],
        simulation.data[r + 3],
      ],
      terrain = terrainAt(mission, p);
    maxAgl = Math.max(maxAgl, p[2] - terrain.height);
    // Obstacles within the lateral buffer, five times a second.
    const time = simulation.data[r];
    if (mission.obstacles?.items.length && time >= nextObstacleCheck) {
      nextObstacleCheck = time + 0.2;
      for (const hit of obstaclesNear(mission.obstacles, p, lateral)) {
        const c = p[2] - terrainAt(mission, hit.nearest).height - hit.obstacle.height;
        if (c < obstacle.clearance)
          obstacle = {
            clearance: c,
            obstacle: hit.obstacle,
            time,
            distance: hit.distance,
            nearest: hit.nearest,
          };
      }
    }
    if (name === "rtl" && rtl && simulation.data[r] >= rtl.start) {
      // The vehicle advances along the leg: search near the last knot.
      let best = Infinity;
      const from = knot ? Math.max(0, knot - 30) : 0,
        to = knot ? Math.min(rtl.points.length, knot + 30) : rtl.points.length;
      for (let k = from; k < to; k++) {
        const d = Math.hypot(p[0] - rtl.points[k][0], p[1] - rtl.points[k][1]);
        if (d < best) {
          best = d;
          knot = k;
        }
      }
      returnClearance = Math.min(returnClearance, p[2] - rtl.floor[knot]);
      returnDeviation = Math.max(returnDeviation, best);
      continue;
    }
    if (!ON_ROUTE.has(name)) continue;
    // The flown trace may sit anywhere inside the corridor, so clearance is
    // measured against the corridor's highest sampled ground, not the centre.
    minClearance = Math.min(minClearance, p[2] - terrain.floor);
    maxDeviation = Math.max(maxDeviation, terrain.distance);
  }
  return {
    minClearance,
    maxAgl,
    maxCorridorDeviation: maxDeviation,
    returnClearance,
    returnDeviation,
    obstacleClearance: obstacle,
  };
}
// Attaches the flown simulation and its terrain measurements. `nominal` is
// the fault-free solve of the same plan when the flown one injects a fault:
// the preflight checklist judges the plan on it.
export function attachSimulation(mission, simulation, nominal = simulation) {
  mission.simulation = simulation;
  mission.duration = simulation.duration;
  Object.assign(mission, measureFlight(mission, simulation));
  mission.nominal =
    nominal === simulation
      ? {
          simulation,
          minClearance: mission.minClearance,
          maxAgl: mission.maxAgl,
          maxCorridorDeviation: mission.maxCorridorDeviation,
          obstacleClearance: mission.obstacleClearance,
        }
      : { simulation: nominal, ...measureFlight(mission, nominal) };
  return mission;
}
export function evaluateMission(C, mission, seconds) {
  const state = replayAt(mission.simulation, seconds),
    local = new C.Cartesian3(...state.p);
  const cartesian = C.Matrix4.multiplyByPoint(
    mission.frame,
    local,
    new C.Cartesian3(),
  );
  const position = C.Cartographic.fromCartesian(cartesian),
    terrain = terrainAt(mission, state.p),
    forward = rotate(state.q, [1, 0, 0]);
  const [w, x, y, z] = state.q;
  // Time left until the landing reserve at the mean current drawn so far;
  // charge an injected cell fault took was never drawn.
  const battery = mission.simulation.battery,
    failsafe = mission.simulation.failsafe,
    lost = (failsafe?.faults ?? [])
      .filter((f) => f.type === "battery" && seconds >= f.time)
      .reduce((sum, f) => sum + f.loss, 0),
    usedAh = Math.max(0, 1 - state.battery.soc - lost) * battery.capacityAh,
    meanCurrent =
      seconds > 5 ? (usedAh * 3600) / seconds : state.battery.current,
    reserveAh = (state.battery.soc - battery.reserve) * battery.capacityAh;
  return {
    ...state,
    navigationError: Math.hypot(
      state.estimate[0] - state.p[0],
      state.estimate[1] - state.p[1],
      state.estimate[2] - state.p[2],
    ),
    remainingTime:
      meanCurrent > 0 ? Math.max(0, (reserveAh * 3600) / meanCurrent) : 0,
    cartesian,
    position,
    agl: state.p[2] - terrain.height,
    terrainHeight: position.height - (state.p[2] - terrain.height),
    relativeAltitude: position.height - mission.groundHeight,
    // Cesium heights are ellipsoidal; subtracting the geoid undulation gives
    // an approximate orthometric (mean sea level) height, or null without N.
    mslAltitude: Number.isFinite(mission.geoidUndulation)
      ? position.height - mission.geoidUndulation
      : null,
    heading: (Math.atan2(forward[0], forward[1]) + 2 * Math.PI) % (2 * Math.PI),
    roll: Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    pitch: -Math.asin(clamp(2 * (w * y - z * x), -1, 1)),
    speed: Math.hypot(...state.v),
    groundSpeed: Math.hypot(state.v[0], state.v[1]),
    verticalSpeed: state.v[2],
    elapsed: clamp(seconds, 0, mission.duration),
    progress: clamp(seconds / mission.duration, 0, 1),
    failsafe: failsafeStatus(mission.simulation, seconds),
  };
}
// Operator banner of the failsafe in force at `seconds`, or null.
export function failsafeStatus(simulation, seconds) {
  let text = null;
  for (const s of simulation.failsafe?.status ?? [])
    if (s.time <= seconds) text = s.text;
  return text;
}
