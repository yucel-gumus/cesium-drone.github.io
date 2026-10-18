import { clamp } from "./flight-math.mjs";

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

// The elevation model of a planned mission, in its ENU frame, is plain data
// so it can be posted to the physics worker:
//   stations  [{ p: [x, y], z, floor }] along the path: centre-line ground
//             and the highest of the three cross-corridor samples,
//   raster    { x0, y0, spacing, nx, ny, heights } regular grid over the
//             route's convex hull (NaN outside it), or null,
//   halfWidth corridor half width (m), cell lookup cell size (m).
// Stations are dense along the planned path; the raster lets a failsafe
// leave it (direct return to home, landing beside a drifting estimate).
export function indexStations(model) {
  if (model.grid) return model;
  const grid = new Map();
  model.stations.forEach((x, i) => {
    const key = `${Math.floor(x.p[0] / model.cell)},${Math.floor(x.p[1] / model.cell)}`;
    if (!grid.has(key)) grid.set(key, []);
    grid.get(key).push(i);
  });
  model.grid = grid;
  return model;
}

// Nearest corridor station to `p` within one lookup cell, refined along the
// neighbouring stations; null when no station is that close.
export function stationAt(model, p) {
  indexStations(model);
  const x = Math.floor(p[0] / model.cell),
    y = Math.floor(p[1] / model.cell);
  let best = Infinity,
    index = 0;
  for (let a = -1; a <= 1; a++)
    for (let b = -1; b <= 1; b++)
      for (const i of model.grid.get(`${x + a},${y + b}`) || []) {
        const k = model.stations[i];
        const d = (p[0] - k.p[0]) ** 2 + (p[1] - k.p[1]) ** 2;
        if (d < best) {
          best = d;
          index = i;
        }
      }
  if (!Number.isFinite(best)) return null;
  const k = model.stations[index];
  let ground = k.z,
    floor = k.floor;
  for (const j of [index - 1, index + 1])
    if (model.stations[j]) {
      const n = model.stations[j],
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
  return { height: ground, floor, distance: Math.sqrt(best) };
}

const node = (raster, i, j) =>
  i < 0 || j < 0 || i >= raster.nx || j >= raster.ny
    ? NaN
    : raster.heights[j * raster.nx + i];

// Bilinear ground height of the raster; NaN outside its sampled hull.
export function rasterHeight(raster, x, y) {
  if (!raster) return NaN;
  const u = (x - raster.x0) / raster.spacing,
    v = (y - raster.y0) / raster.spacing,
    i = Math.floor(u),
    j = Math.floor(v),
    fu = u - i,
    fv = v - j;
  return (
    (node(raster, i, j) * (1 - fu) + node(raster, i + 1, j) * fu) * (1 - fv) +
    (node(raster, i, j + 1) * (1 - fu) + node(raster, i + 1, j + 1) * fu) * fv
  );
}

// Highest of the four raster nodes around (x, y): a conservative ground
// height for clearance, never below the bilinear surface.
export function rasterFloor(raster, x, y) {
  if (!raster) return NaN;
  const i = Math.floor((x - raster.x0) / raster.spacing),
    j = Math.floor((y - raster.y0) / raster.spacing);
  return Math.max(
    node(raster, i, j),
    node(raster, i + 1, j),
    node(raster, i, j + 1),
    node(raster, i + 1, j + 1),
  );
}

// Ground under `p`: { height (centre-line or raster ground), floor (highest
// ground across the corridor or around the raster cell), distance (to the
// nearest corridor station, Infinity when none is near) }. Inside the
// sampled corridor the stations answer, as the preflight checks measure;
// beside it the raster does.
export function terrainUnder(model, p) {
  const station = stationAt(model, p);
  if (station && station.distance <= model.halfWidth + 2) return station;
  const height = rasterHeight(model.raster, p[0], p[1]),
    floor = rasterFloor(model.raster, p[0], p[1]);
  if (Number.isFinite(height) && Number.isFinite(floor))
    return { height, floor, distance: station?.distance ?? Infinity };
  if (station) return station;
  throw new Error("Drone doğrulanmış arazi koridorunun dışına çıktı.");
}

// Corridor floor of a path that leaves the planned route (the direct return
// leg): the highest raster ground under the centre and both corridor edges,
// as the planned corridor takes the highest of its three samples, raised to
// the station floor where the leg runs inside the planned corridor, so the
// leg is never lower than either survey of the ground. The bilinear surface
// keeps the floor continuous; a per-cell maximum would step it every
// raster spacing and slow the leg down for nothing.
export function corridorFloor(model, p, tangent) {
  const w = model.halfWidth,
    across = [-tangent[1] * w, tangent[0] * w];
  let floor = -Infinity;
  for (const side of [0, -1, 1]) {
    const x = p[0] + across[0] * side,
      y = p[1] + across[1] * side,
      value = rasterHeight(model.raster, x, y);
    if (Number.isFinite(value)) floor = Math.max(floor, value);
    else floor = NaN;
  }
  const station = stationAt(model, p);
  if (station && station.distance <= w)
    floor = Number.isNaN(floor) ? station.floor : Math.max(floor, station.floor);
  if (!Number.isFinite(floor))
    throw new Error("Eve dönüş hattı doğrulanmış arazinin dışında kalıyor.");
  return floor;
}

// Convex hull (counter-clockwise, Andrew's monotone chain) of 2D points.
export function convexHull(points) {
  const sorted = points
    .map((p) => [p[0], p[1]])
    .sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) =>
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower = [],
    upper = [];
  for (const p of sorted) {
    while (lower.length >= 2 && cross(lower.at(-2), lower.at(-1), p) <= 0)
      lower.pop();
    lower.push(p);
  }
  for (let i = sorted.length - 1; i >= 0; i--) {
    const p = sorted[i];
    while (upper.length >= 2 && cross(upper.at(-2), upper.at(-1), p) <= 0)
      upper.pop();
    upper.push(p);
  }
  return lower.slice(0, -1).concat(upper.slice(0, -1));
}

// Distance from `p` to a counter-clockwise convex polygon (0 inside).
export function distanceToConvex(p, hull) {
  let inside = true,
    best = Infinity;
  for (let i = 0; i < hull.length; i++) {
    const a = hull[i],
      b = hull[(i + 1) % hull.length],
      dx = b[0] - a[0],
      dy = b[1] - a[1],
      l = dx * dx + dy * dy,
      f = l > 0 ? clamp(((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l, 0, 1) : 0;
    if (dx * (p[1] - a[1]) - dy * (p[0] - a[0]) < 0) inside = false;
    best = Math.min(best, Math.hypot(p[0] - a[0] - f * dx, p[1] - a[1] - f * dy));
  }
  return inside ? 0 : best;
}

// Raster nodes to sample: a lattice of `spacing` metres (aligned to the ENU
// origin, so re-plans reuse cached heights) covering the convex hull of the
// path widened by `margin`. Any straight leg between two path points stays
// inside the hull, so the direct return home is always covered.
export function rasterLayout(points, spacing, margin) {
  const hull = convexHull(points);
  let minX = Infinity,
    minY = Infinity,
    maxX = -Infinity,
    maxY = -Infinity;
  for (const p of hull) {
    minX = Math.min(minX, p[0]);
    minY = Math.min(minY, p[1]);
    maxX = Math.max(maxX, p[0]);
    maxY = Math.max(maxY, p[1]);
  }
  const i0 = Math.floor((minX - margin) / spacing),
    j0 = Math.floor((minY - margin) / spacing),
    nx = Math.floor((maxX + margin) / spacing) - i0 + 1,
    ny = Math.floor((maxY + margin) / spacing) - j0 + 1,
    slots = new Int32Array(nx * ny).fill(-1),
    nodes = [];
  for (let j = 0; j < ny; j++)
    for (let i = 0; i < nx; i++) {
      const p = [(i0 + i) * spacing, (j0 + j) * spacing];
      if (distanceToConvex(p, hull) <= margin) {
        slots[j * nx + i] = nodes.length;
        nodes.push(p);
      }
    }
  return { x0: i0 * spacing, y0: j0 * spacing, spacing, nx, ny, slots, nodes };
}
