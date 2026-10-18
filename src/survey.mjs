import { clamp } from "./flight-math.mjs";

// Area mapping (lawnmower / boustrophedon) over a polygon, as photogrammetry
// planners fly it: parallel lines spaced from the camera's across-track
// footprint and the side overlap, turns outside the polygon.
export const SURVEY_DEFAULTS = Object.freeze({
  sideOverlap: 0.7,
  minSideOverlap: 0.6,
  maxSideOverlap: 0.8,
  // Lines run this far past the polygon so the turns happen outside it
  // (QGroundControl's "turnaround distance").
  turnaround: 15,
  maxLines: 60,
  // Polygon area limits, m².
  minArea: 5000,
  maxArea: 2e6,
});

// Optional `surveyArea` of the route file: 3–50 WGS84 vertices inside the
// route bounds. Returns null when the file has none.
export function validateSurveyArea(data, bounds) {
  const area = data?.surveyArea;
  if (area === undefined || area === null) return null;
  if (!Array.isArray(area) || area.length < 3 || area.length > 51)
    throw new Error("surveyArea poligonu 3–50 köşe içermeli.");
  const points = area.map((p, i) => {
    if (!p || !Number.isFinite(p.longitude) || !Number.isFinite(p.latitude))
      throw new Error(`surveyArea köşesi ${i + 1}: geçersiz koordinat.`);
    if (
      p.longitude < bounds.west ||
      p.longitude > bounds.east ||
      p.latitude < bounds.south ||
      p.latitude > bounds.north
    )
      throw new Error(`surveyArea köşesi ${i + 1}: rota sınırlarının dışında.`);
    return { longitude: p.longitude, latitude: p.latitude };
  });
  if (
    points.length > 3 &&
    points[0].longitude === points.at(-1).longitude &&
    points[0].latitude === points.at(-1).latitude
  )
    points.pop();
  if (points.length < 3 || points.length > 50)
    throw new Error("surveyArea poligonu 3–50 köşe içermeli.");
  return points;
}

// Signed shoelace area (m², positive counter-clockwise).
export function polygonArea(polygon) {
  let sum = 0;
  for (let i = 0; i < polygon.length; i++) {
    const a = polygon[i],
      b = polygon[(i + 1) % polygon.length];
    sum += a[0] * b[1] - b[0] * a[1];
  }
  return sum / 2;
}

// Even–odd ray casting; points on an edge may fall either way.
export function pointInPolygon(p, polygon) {
  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const a = polygon[i],
      b = polygon[j];
    if (
      a[1] > p[1] !== b[1] > p[1] &&
      p[0] < ((b[0] - a[0]) * (p[1] - a[1])) / (b[1] - a[1]) + a[0]
    )
      inside = !inside;
  }
  return inside;
}

const orientation = (a, b, c) =>
  Math.sign((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]));
function segmentsCross(a, b, c, d) {
  return (
    orientation(a, b, c) * orientation(a, b, d) < 0 &&
    orientation(c, d, a) * orientation(c, d, b) < 0
  );
}

// ENU polygon ready for planning: duplicate vertices removed, simple (no
// self-intersection), within the area limits, counter-clockwise.
export function preparePolygon(points, limits = SURVEY_DEFAULTS) {
  const polygon = [];
  for (const p of points)
    if (
      !polygon.length ||
      Math.hypot(p[0] - polygon.at(-1)[0], p[1] - polygon.at(-1)[1]) > 0.5
    )
      polygon.push([p[0], p[1]]);
  if (
    polygon.length > 3 &&
    Math.hypot(
      polygon[0][0] - polygon.at(-1)[0],
      polygon[0][1] - polygon.at(-1)[1],
    ) <= 0.5
  )
    polygon.pop();
  if (polygon.length < 3)
    throw new Error("surveyArea en az üç farklı köşe içermeli.");
  const n = polygon.length;
  for (let i = 0; i < n; i++)
    for (let j = i + 1; j < n; j++) {
      // Adjacent edges share a vertex and never count as crossing.
      if (j === i + 1 || (i === 0 && j === n - 1)) continue;
      if (
        segmentsCross(
          polygon[i],
          polygon[(i + 1) % n],
          polygon[j],
          polygon[(j + 1) % n],
        )
      )
        throw new Error("surveyArea poligonu kendini kesmemeli.");
    }
  const area = polygonArea(polygon);
  if (Math.abs(area) < limits.minArea || Math.abs(area) > limits.maxArea)
    throw new Error(
      `surveyArea alanı ${limits.minArea / 1e4}–${limits.maxArea / 1e4} ha arasında olmalı.`,
    );
  return area < 0 ? polygon.reverse() : polygon;
}

// Extent along `direction` of the part of the polygon whose cross
// coordinate lies in [w1, w2]: extremes occur at the strip's two chords or
// at vertices inside it.
function stripExtent(polygon, direction, normal, w1, w2) {
  const us = [];
  for (let i = 0; i < polygon.length; i++) {
    const a = polygon[i],
      b = polygon[(i + 1) % polygon.length],
      wa = a[0] * normal[0] + a[1] * normal[1],
      wb = b[0] * normal[0] + b[1] * normal[1],
      ua = a[0] * direction[0] + a[1] * direction[1],
      ub = b[0] * direction[0] + b[1] * direction[1];
    if (wa >= w1 && wa <= w2) us.push(ua);
    for (const w of [w1, w2])
      if ((wa - w) * (wb - w) < 0) us.push(ua + ((ub - ua) * (w - wa)) / (wb - wa));
  }
  return [Math.min(...us), Math.max(...us)];
}

// Lawnmower over a counter-clockwise ENU polygon. Lines run parallel to the
// polygon's longest edge, `spacing` apart and centred on the polygon's
// width across them; their count covers the full width, so the outermost
// lines sit on or just outside the edges and the boundary keeps the
// planned side overlap. Each line spans the polygon over its own strip
// (± half a spacing), is extended by `turnaround` past it and joins the
// next one with two 90° corners (roundedPath rejects
// 180° reversals): the shared turn coordinate is the farther of the two line
// ends, so the connecting leg is perpendicular to both. Of the four ways to
// fly the pattern (from the first or the last line, in either direction)
// the one with the shortest transits from and to `home` whose corners stay
// under 150° wins; a transit that would reverse onto a line arrives through
// a point outside the grid instead.
// The grid's lines over a polygon: direction along the longest edge, cross
// coordinates `spacing` apart covering the width, each line's extent over
// its own strip plus the turnaround distance (see lawnmower()).
export function gridLines(polygon, spacing, options = {}) {
  const turnaround = options.turnaround ?? SURVEY_DEFAULTS.turnaround,
    maxLines = options.maxLines ?? SURVEY_DEFAULTS.maxLines;
  if (!(spacing > 1)) throw new Error("Hat aralığı geçersiz.");
  let longest = 0,
    direction;
  for (let i = 0; i < polygon.length; i++) {
    const a = polygon[i],
      b = polygon[(i + 1) % polygon.length],
      length = Math.hypot(b[0] - a[0], b[1] - a[1]);
    if (length > longest) {
      longest = length;
      direction = [(b[0] - a[0]) / length, (b[1] - a[1]) / length];
    }
  }
  const normal = [-direction[1], direction[0]];
  const ws = polygon.map((p) => p[0] * normal[0] + p[1] * normal[1]),
    wMin = Math.min(...ws),
    wMax = Math.max(...ws),
    width = wMax - wMin,
    count = Math.max(2, Math.ceil(width / spacing - 1e-9) + 1);
  if (count > maxLines)
    throw new Error(
      `Izgara ${count} hat gerektiriyor (en fazla ${maxLines}). Yan bindirmeyi azaltın veya yüksekliği artırın.`,
    );
  const lines = [];
  for (let k = 0; k < count; k++) {
    const w = (wMin + wMax) / 2 + (k - (count - 1) / 2) * spacing,
      span = stripExtent(
        polygon,
        direction,
        normal,
        clamp(w - spacing / 2, wMin, wMax),
        clamp(w + spacing / 2, wMin, wMax),
      );
    lines.push({
      index: k,
      w,
      from: span[0] - turnaround,
      to: span[1] + turnaround,
    });
  }
  return { lines, direction, normal, count, width };
}

export function lawnmower(polygon, spacing, options = {}) {
  const home = options.home ?? [0, 0],
    turnaround = options.turnaround ?? SURVEY_DEFAULTS.turnaround,
    grid = gridLines(polygon, spacing, options),
    { direction, normal, count, width } = grid,
    // A sortie of a multi-flight survey flies a contiguous block of lines.
    lines = options.lines ? options.lines.map((i) => grid.lines[i]) : grid.lines;
  if (lines.length < 2 || lines.some((l) => !l))
    throw new Error("Bir uçuş en az iki ardışık ızgara hattı içermeli.");
  const xy = (u, w) => [
    u * direction[0] + w * normal[0],
    u * direction[1] + w * normal[1],
  ];
  // Cosine of the turn at b between legs a→b and b→c.
  const corner = (a, b, c) => {
    const ux = b[0] - a[0],
      uy = b[1] - a[1],
      vx = c[0] - b[0],
      vy = c[1] - b[1];
    return (ux * vx + uy * vy) / (Math.hypot(ux, uy) * Math.hypot(vx, vy));
  };
  const sharpest = Math.cos((150 * Math.PI) / 180),
    lead = 2 * turnaround,
    offset = (p, n) => [p[0] + n[0] * lead, p[1] + n[1] * lead],
    length = (a, b) => Math.hypot(b[0] - a[0], b[1] - a[1]);
  let best = null;
  for (const reverse of [false, true])
    for (const sense of [1, -1]) {
      const order = reverse ? lines.slice().reverse() : lines,
        points = [],
        legs = [];
      let s = sense;
      for (let k = 0; k < order.length; k++) {
        const line = order[k],
          previous = order[k - 1],
          next = order[k + 1];
        // Entry: the turn shared with the previous line, else the line end.
        const entry =
          s > 0
            ? previous
              ? Math.min(line.from, previous.from)
              : line.from
            : previous
              ? Math.max(line.to, previous.to)
              : line.to;
        const exit =
          s > 0
            ? next
              ? Math.max(line.to, next.to)
              : line.to
            : next
              ? Math.min(line.from, next.from)
              : line.from;
        points.push(xy(entry, line.w), xy(exit, line.w));
        legs.push({ w: line.w, from: entry, to: exit, sense: s });
        s = -s;
      }
      // Normals pointing out of the grid at its first and last line.
      const outward = (line, neighbour) =>
        Math.sign(line.w - neighbour.w) > 0 ? normal : [-normal[0], -normal[1]];
      const first = points[0],
        last = points.at(-1);
      // Join home directly when the turn onto the first line stays under
      // 150°; otherwise approach from a point a lead distance outside the
      // grid, which makes that turn 90°. The same for leaving the grid.
      let approach = [];
      if (corner(home, first, points[1]) < sharpest || length(home, first) < 1) {
        const a = offset(first, outward(order[0], order[1]));
        if (length(home, a) < 1 || corner(home, a, first) < sharpest) continue;
        approach = [a];
      }
      let departure = [];
      if (corner(points.at(-2), last, home) < sharpest || length(last, home) < 1) {
        const d = offset(last, outward(order.at(-1), order.at(-2)));
        if (length(d, home) < 1 || corner(last, d, home) < sharpest) continue;
        departure = [d];
      }
      const route = [home, ...approach, first],
        back = [last, ...departure, home];
      const cost =
        route.slice(1).reduce((sum, p, i) => sum + length(route[i], p), 0) +
        back.slice(1).reduce((sum, p, i) => sum + length(back[i], p), 0);
      if (!best || cost < best.cost)
        best = { cost, points, legs, approach, departure };
    }
  if (!best)
    throw new Error(
      "Kalkış noktası ızgaraya güvenli bir giriş/çıkış dönüşü bırakmıyor.",
    );
  const waypoints = [
    home.slice(),
    ...best.approach,
    ...best.points,
    ...best.departure,
    home.slice(),
  ];
  const gridStart = 1 + best.approach.length;
  return {
    waypoints,
    lines: best.legs.map((leg) => ({
      ...leg,
      a: xy(leg.from, leg.w),
      b: xy(leg.to, leg.w),
    })),
    // Every line of the grid, flown in this sortie or another.
    allLines: grid.lines,
    direction,
    normal,
    spacing,
    count,
    // Waypoint indices of the first line's start and the last line's end.
    gridStart,
    gridEnd: gridStart + best.points.length - 1,
    width,
  };
}

// Flight time estimate (s) of a sortie flying `lines` (a contiguous block
// of gridLines()) from and back to `home`: take-off and landing, transits
// at 70 % of cruise speed, lines at 95 % with half a minute per turnaround.
// Calibrated against solved grids (within a few percent); the solve itself
// then checks the landing reserve.
export function sortieSeconds(grid, block, home, settings) {
  const height = settings.agl + 10,
    overhead =
      2 + (1.9 * height) / settings.climbSpeed + 4 + (1.9 * height) / settings.descentSpeed + 15;
  const lines = block.map((i) => grid.lines[i]),
    at = (line, u) => [
      u * grid.direction[0] + line.w * grid.normal[0],
      u * grid.direction[1] + line.w * grid.normal[1],
    ],
    // In to the nearest end, out from (at worst) the farthest.
    reach = [
      at(lines[0], lines[0].from),
      at(lines[0], lines[0].to),
      at(lines.at(-1), lines.at(-1).from),
      at(lines.at(-1), lines.at(-1).to),
    ].map((p) => Math.hypot(p[0] - home[0], p[1] - home[1])),
    transit = Math.min(...reach) + Math.max(...reach);
  return (
    overhead +
    transit / (0.7 * settings.cruiseSpeed) +
    lines.reduce((sum, l) => sum + (l.to - l.from) / (0.95 * settings.cruiseSpeed) + 30, 0)
  );
}

// Splits a grid into sorties of contiguous line blocks, each estimated to
// fly within `maxSeconds` (at least two lines per sortie). Balanced: the
// number of sorties is the smallest that fits, the lines spread evenly.
export function partitionLines(grid, home, settings, maxSeconds) {
  const n = grid.lines.length;
  for (let sorties = 1; sorties <= Math.floor(n / 2); sorties++) {
    const blocks = [];
    for (let k = 0, start = 0; k < sorties; k++) {
      const size = Math.floor(n / sorties) + (k < n % sorties ? 1 : 0);
      blocks.push(Array.from({ length: size }, (_, i) => start + i));
      start += size;
    }
    if (blocks.every((b) => sortieSeconds(grid, b, home, settings) <= maxSeconds))
      return blocks;
  }
  throw new Error(
    "Izgara iki hatlık uçuşlara bile bölünemiyor; alanı küçültün veya kütleyi düşürün.",
  );
}

// Fraction of the polygon imaged at least once: the polygon is sampled on a
// `cell` metre lattice and each sample is tested against the (convex)
// footprint quadrilaterals, bucketed by their bounding boxes.
export function areaCoverage(polygon, footprints, cell = 5) {
  const bucket = 50,
    buckets = new Map();
  footprints.forEach((corners, index) => {
    const xs = corners.map((c) => c[0]),
      ys = corners.map((c) => c[1]);
    for (
      let i = Math.floor(Math.min(...xs) / bucket);
      i <= Math.floor(Math.max(...xs) / bucket);
      i++
    )
      for (
        let j = Math.floor(Math.min(...ys) / bucket);
        j <= Math.floor(Math.max(...ys) / bucket);
        j++
      ) {
        const key = `${i},${j}`;
        if (!buckets.has(key)) buckets.set(key, []);
        buckets.get(key).push(index);
      }
  });
  const xs = polygon.map((p) => p[0]),
    ys = polygon.map((p) => p[1]);
  let total = 0,
    covered = 0;
  for (let x = Math.min(...xs) + cell / 2; x < Math.max(...xs); x += cell)
    for (let y = Math.min(...ys) + cell / 2; y < Math.max(...ys); y += cell) {
      const p = [x, y];
      if (!pointInPolygon(p, polygon)) continue;
      total++;
      const candidates =
        buckets.get(`${Math.floor(x / bucket)},${Math.floor(y / bucket)}`) || [];
      if (candidates.some((i) => pointInPolygon(p, footprints[i]))) covered++;
    }
  return total ? covered / total : 0;
}
