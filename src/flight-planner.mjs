import { roundedPath, makeReference } from "./trajectory.mjs";
import { replayAt, STRIDE } from "./quadrotor.mjs";
import { rotate, clamp } from "./flight-math.mjs";
export const FLIGHT_DEFAULTS = Object.freeze({
  clearance: 70,
  terrainMargin: 10,
  trackingMargin: 5,
  sampleSpacing: 6,
  corridorHalfWidth: 12,
  cruiseSpeed: 8,
  climbSpeed: 2.5,
  descentSpeed: 1.5,
  acceleration: 1.5,
  lateralAcceleration: 1.2,
  jerk: 2,
  landingOffset: 0.1,
  turnTrim: 45,
  wind: 2,
});
export function validateWaypoints(data) {
  if (
    !Array.isArray(data?.dronePoints) ||
    data.dronePoints.length < 3 ||
    data.dronePoints.length > 200
  )
    throw new Error("Rota 3–200 nokta içermeli.");
  const points = data.dronePoints.map((p, i) => {
    if (
      !p ||
      !Number.isFinite(p.longitude) ||
      !Number.isFinite(p.latitude) ||
      p.longitude < 40.25 ||
      p.longitude > 40.35 ||
      p.latitude < 40.59 ||
      p.latitude > 40.65
    )
      throw new Error(`Rota noktası ${i + 1}: geçersiz Uzungöl koordinatı.`);
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
    settings.clearance < 20 ||
    settings.sampleSpacing > 20 ||
    settings.cruiseSpeed > 15 ||
    settings.wind > 8
  )
    throw new Error("Uçuş ayarı modelin sınırları dışında.");
  const points = validateWaypoints(data),
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
    samples,
    path,
    centers,
    home,
    zeroFrame,
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
  const cruiseHeight =
    Math.max(...grounds) +
    route.settings.clearance +
    route.settings.terrainMargin +
    route.settings.trackingMargin;
  const reference = makeReference(route.path, cruiseHeight, route.settings);
  const grid = new Map(),
    cell = 30;
  const terrain = route.centers.map((index, i) => ({
    p: route.path.table[index].p,
    z: grounds[i * 3],
    floor: Math.max(...grounds.slice(i * 3, i * 3 + 3)),
  }));
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
    duration: reference.duration,
    distance: route.path.length + 2 * cruiseHeight,
    minCruiseClearance:
      route.settings.clearance +
      route.settings.terrainMargin +
      route.settings.trackingMargin,
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
  let ground = k.z;
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
      }
    }
  return { height: ground, distance: Math.sqrt(best) };
}
export function attachSimulation(mission, simulation) {
  mission.simulation = simulation;
  mission.duration = simulation.duration;
  let min = Infinity,
    maxDeviation = 0;
  for (let i = 0; i < simulation.rows; i++) {
    const r = i * STRIDE,
      phase = simulation.data[r + 30];
    if (phase !== 2 && phase !== 3) continue;
    const p = Array.from(simulation.data.slice(r + 1, r + 4)),
      terrain = terrainAt(mission, p);
    min = Math.min(min, p[2] - terrain.height);
    maxDeviation = Math.max(maxDeviation, terrain.distance);
    if (
      p[2] - terrain.height < mission.settings.clearance ||
      terrain.distance > mission.settings.corridorHalfWidth - 2
    )
      throw new Error(
        "Dinamik uçuş arazi açıklığı/koridor denetimini geçemedi.",
      );
  }
  mission.minCruiseClearance = min;
  mission.maxCorridorDeviation = maxDeviation;
  mission.distance = simulation.data[(simulation.rows - 1) * STRIDE + 28];
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
  return {
    ...state,
    cartesian,
    position,
    agl: state.p[2] - terrain.height,
    terrainHeight: position.height - (state.p[2] - terrain.height),
    relativeAltitude: position.height - mission.groundHeight,
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
