import { roundedPath, makeReference } from "./trajectory.mjs";
import { replayAt, STRIDE } from "./quadrotor.mjs";
import { rotate, clamp } from "./flight-math.mjs";
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
});
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
export function createTerrainRoute(C, data, options = {}) {
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
    settings.wind > 8
  )
    throw new Error("Uçuş ayarı modelin sınırları dışında.");
  const points = validateWaypoints(data),
    bounds = validateBounds(data),
    geoidUndulation = validateGeoidUndulation(data),
    home = C.Cartographic.fromDegrees(points[0].longitude, points[0].latitude);
  const zeroFrame = C.Transforms.eastNorthUpToFixedFrame(
    C.Cartesian3.fromRadians(home.longitude, home.latitude, 0),
  );
  const inverse = C.Matrix4.inverseTransformation(zeroFrame, new C.Matrix4());
  const local = [];
  for (const p of points) {
    const v = C.Matrix4.multiplyByPoint(
      inverse,
      C.Cartesian3.fromDegrees(p.longitude, p.latitude, 0),
      new C.Cartesian3(),
    );
    if (
      !local.length ||
      Math.hypot(v.x - local.at(-1)[0], v.y - local.at(-1)[1]) > 0.1
    )
      local.push([v.x, v.y]);
  }
  if (local.length < 3) throw new Error("Rota en az üç farklı nokta içermeli.");
  const path = roundedPath(local, settings.turnTrim, 2),
    samples = [],
    centers = [];
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
    for (const side of [0, -1, 1]) {
      const v = new C.Cartesian3(
        k.p[0] - k.tangent[1] * side * settings.corridorHalfWidth,
        k.p[1] + k.tangent[0] * side * settings.corridorHalfWidth,
        0,
      );
      samples.push(
        C.Cartographic.fromCartesian(
          C.Matrix4.multiplyByPoint(zeroFrame, v, new C.Cartesian3()),
        ),
      );
    }
  }
  return {
    settings,
    bounds,
    geoidUndulation,
    samples,
    path,
    centers,
    home,
    horizontalDistance: path.length,
  };
}
export async function sampleTerrainRoute(
  C,
  provider,
  route,
  onProgress = () => {},
) {
  for (let offset = 0; offset < route.samples.length; offset += 192) {
    await C.sampleTerrainMostDetailed(
      provider,
      route.samples.slice(offset, offset + 192),
      true,
    );
    onProgress(Math.min(1, (offset + 192) / route.samples.length));
  }
  if (route.samples.some((p) => !Number.isFinite(p.height)))
    throw new Error("Eksik arazi yüksekliği; uçuş hazırlanmadı.");
  return route;
}
// Terrain-following height profile over evenly spaced path knots. Each
// knot's requirement is the corridor floor plus the target AGL. The profile
//   1. dilates the requirement over ±profileDilation (look-ahead: climb before
//      a ridge, descend only after it),
//   2. raises points until the climb and descent slopes the cruise speed
//      allows are respected (raise-only, so clearance is never reduced),
//   3. smooths with a Gaussian truncated at 3σ ≤ profileDilation.
// Averaging a raise-only, dilated sequence over a window no wider than the
// dilation keeps every point at or above its own requirement, and averaging
// cannot steepen slopes, so the result is smooth, flyable and safe.
export function terrainFollowingProfile(requirement, spacing, settings) {
  const n = requirement.length,
    window = Math.max(1, Math.round(settings.profileDilation / spacing)),
    sigma = settings.profileSmoothing / spacing,
    support = Math.min(window, Math.ceil(3 * sigma));
  const dilated = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    let max = -Infinity;
    for (let j = Math.max(0, i - window); j <= Math.min(n - 1, i + window); j++)
      max = Math.max(max, requirement[j]);
    dilated[i] = max;
  }
  const up = (settings.followClimbRate / settings.cruiseSpeed) * spacing,
    down = (settings.followDescentRate / settings.cruiseSpeed) * spacing;
  for (let i = n - 2; i >= 0; i--)
    dilated[i] = Math.max(dilated[i], dilated[i + 1] - up);
  for (let i = 1; i < n; i++)
    dilated[i] = Math.max(dilated[i], dilated[i - 1] - down);
  const weights = Array.from({ length: support + 1 }, (_, j) =>
    Math.exp((-j * j) / (2 * sigma * sigma)),
  );
  const profile = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    let sum = 0,
      total = 0;
    for (let j = -support; j <= support; j++) {
      const w = weights[Math.abs(j)];
      sum += w * dilated[clamp(i + j, 0, n - 1)];
      total += w;
    }
    profile[i] = sum / total;
  }
  return profile;
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
  const terrain = route.centers.map((index, i) => ({
    index,
    p: route.path.table[index].p,
    z: grounds[i * 3],
    floor: Math.max(...grounds.slice(i * 3, i * 3 + 3)),
  }));
  // Corridor floor at every path knot, linear between sampled stations.
  const table = route.path.table,
    requirement = new Float64Array(table.length);
  for (let k = 1; k < terrain.length; k++) {
    const a = terrain[k - 1],
      b = terrain[k];
    for (let i = a.index; i <= b.index; i++) {
      const f = b.index > a.index ? (i - a.index) / (b.index - a.index) : 0;
      requirement[i] = a.floor + (b.floor - a.floor) * f + route.settings.agl;
    }
  }
  const spacing = table.length > 1 ? table[1].s - table[0].s : 1,
    profile = terrainFollowingProfile(requirement, spacing, route.settings),
    reference = makeReference(route.path, profile, route.settings);
  const grid = new Map(),
    cell = 30;
  terrain.forEach((x, i) => {
    const key = `${Math.floor(x.p[0] / cell)},${Math.floor(x.p[1] / cell)}`;
    if (!grid.has(key)) grid.set(key, []);
    grid.get(key).push(i);
  });
  return {
    ...route,
    home,
    groundHeight,
    frame,
    reference,
    profile,
    requirement,
    duration: reference.duration,
    terrain,
    grid,
    cell,
  };
}
export function terrainAt(mission, p) {
  const x = Math.floor(p[0] / mission.cell),
    y = Math.floor(p[1] / mission.cell);
  let best = Infinity,
    index = 0;
  for (let a = -1; a <= 1; a++)
    for (let b = -1; b <= 1; b++)
      for (const i of mission.grid.get(`${x + a},${y + b}`) || []) {
        const k = mission.terrain[i];
        const d = (p[0] - k.p[0]) ** 2 + (p[1] - k.p[1]) ** 2;
        if (d < best) {
          best = d;
          index = i;
        }
      }
  if (!Number.isFinite(best))
    throw new Error("Drone doğrulanmış arazi koridorunun dışına çıktı.");
  const k = mission.terrain[index];
  let ground = k.z,
    floor = k.floor;
  for (const j of [index - 1, index + 1])
    if (mission.terrain[j]) {
      const n = mission.terrain[j],
        dx = n.p[0] - k.p[0],
        dy = n.p[1] - k.p[1],
        l = dx * dx + dy * dy;
      const f =
        l > 0
          ? clamp(((p[0] - k.p[0]) * dx + (p[1] - k.p[1]) * dy) / l, 0, 1)
          : 0;
      const d = (p[0] - k.p[0] - f * dx) ** 2 + (p[1] - k.p[1] - f * dy) ** 2;
      if (d < best) {
        best = d;
        ground = k.z + (n.z - k.z) * f;
        floor = k.floor + (n.floor - k.floor) * f;
      }
    }
  // height: centre-line ground under the drone (AGL display).
  // floor: highest ground across the sampled corridor at that station.
  return { height: ground, floor, distance: Math.sqrt(best) };
}
// Measures the flown trace against the terrain: the lowest clearance over
// the corridor floor during the survey, the highest height above the ground
// directly below at any airborne moment (regulatory ceiling) and the largest
// distance from the sampled centre line. The preflight checklist judges them.
export function attachSimulation(mission, simulation) {
  mission.simulation = simulation;
  mission.duration = simulation.duration;
  let minClearance = Infinity,
    maxAgl = 0,
    maxDeviation = 0;
  for (let i = 0; i < simulation.rows; i++) {
    const r = i * STRIDE,
      phase = simulation.data[r + 30];
    if (phase < 1 || phase > 5) continue;
    const p = [
        simulation.data[r + 1],
        simulation.data[r + 2],
        simulation.data[r + 3],
      ],
      terrain = terrainAt(mission, p);
    maxAgl = Math.max(maxAgl, p[2] - terrain.height);
    if (phase !== 2 && phase !== 3) continue;
    // The flown trace may sit anywhere inside the corridor, so clearance is
    // measured against the corridor's highest sampled ground, not the centre.
    minClearance = Math.min(minClearance, p[2] - terrain.floor);
    maxDeviation = Math.max(maxDeviation, terrain.distance);
  }
  mission.minClearance = minClearance;
  mission.maxAgl = maxAgl;
  mission.maxCorridorDeviation = maxDeviation;
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
  // Time left until the landing reserve at the mean current drawn so far.
  const battery = mission.simulation.battery,
    usedAh = (1 - state.battery.soc) * battery.capacityAh,
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
  };
}
