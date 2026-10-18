import { pointInPolygon } from "./survey.mjs";

// Obstacles above the ground (buildings, masts, trees and forest canopy)
// as vertical prisms: a footprint (point with a radius, line with a half
// width, or polygon) and a height above the ground beneath it. The flight
// must stay `clearance` metres above any obstacle top within `lateral`
// metres horizontally (the planner raises the terrain-following profile
// where the target AGL alone would not, the checklist measures the flown
// trace against it).
export const OBSTACLE_DEFAULTS = Object.freeze({
  clearance: 20,
  lateral: 10,
  // Spatial index cell, m.
  cell: 50,
});
export const OBSTACLE_KINDS = Object.freeze([
  "building",
  "mast",
  "tower",
  "tree",
  "tree_row",
  "forest",
  "other",
]);
// Heights for OpenStreetMap features without a height tag, metres. Storeys
// are 3 m plus a 3 m roof; untagged buildings are taken as about four
// storeys (the hotels around the lake), forest canopy as mature oriental
// spruce stands. These are assumptions, flagged as such in the data.
export const OSM_HEIGHTS = Object.freeze({
  storey: 3,
  roof: 3,
  building: 12,
  hotel: 15,
  mast: 40,
  minaret: 25,
  tower: 25,
  chimney: 30,
  antenna: 15,
  tree: 15,
  treeRadius: 5,
  treeRow: 15,
  canopy: 30,
});

// "15", "15 m", "15.5m" → 15.5; anything else → NaN.
const metresTag = (value) => {
  const match = /^\s*(\d+(?:\.\d+)?)\s*m?\s*$/.exec(value ?? "");
  return match ? Number(match[1]) : NaN;
};
// Height (m above ground) of an OpenStreetMap feature and where it comes
// from: "tag" (height), "levels" (building:levels), "default" (by type) or
// "assumed" (tree canopy).
export function osmObstacleHeight(kind, tags = {}) {
  const tagged = metresTag(tags.height);
  if (Number.isFinite(tagged) && tagged > 0) return { height: tagged, source: "tag" };
  if (kind === "building") {
    const levels = Number(tags["building:levels"]),
      roof = Number(tags["roof:levels"]);
    if (Number.isFinite(levels) && levels > 0)
      return {
        height:
          (levels + (Number.isFinite(roof) && roof > 0 ? roof : 0)) * OSM_HEIGHTS.storey +
          OSM_HEIGHTS.roof,
        source: "levels",
      };
    return {
      height: tags.building === "hotel" ? OSM_HEIGHTS.hotel : OSM_HEIGHTS.building,
      source: "default",
    };
  }
  if (kind === "forest") return { height: OSM_HEIGHTS.canopy, source: "assumed" };
  if (kind === "tree_row") return { height: OSM_HEIGHTS.treeRow, source: "assumed" };
  if (kind === "tree") return { height: OSM_HEIGHTS.tree, source: "assumed" };
  const type = tags["tower:type"] === "minaret" ? "minaret" : tags.man_made;
  return { height: OSM_HEIGHTS[type] ?? OSM_HEIGHTS.tower, source: "default" };
}
function osmKind(tags) {
  if (tags.building) return "building";
  if (tags.landuse === "forest" || tags.natural === "wood") return "forest";
  if (tags.natural === "tree_row") return "tree_row";
  if (tags.natural === "tree") return "tree";
  if (tags.man_made === "mast" || tags.man_made === "antenna") return "mast";
  if (tags.man_made) return "tower";
  return null;
}

// Ramer–Douglas–Peucker on a [lon, lat] ring, tolerance in metres.
function simplifyRing(ring, tolerance) {
  if (ring.length <= 4) return ring;
  const k = Math.cos((ring[0][1] * Math.PI) / 180) * 111320,
    xy = ring.map(([lon, lat]) => [lon * k, lat * 110540]),
    keep = new Uint8Array(ring.length);
  keep[0] = keep[ring.length - 1] = 1;
  const stack = [[0, ring.length - 1]];
  while (stack.length) {
    const [a, b] = stack.pop(),
      [ax, ay] = xy[a],
      dx = xy[b][0] - ax,
      dy = xy[b][1] - ay,
      len2 = dx * dx + dy * dy;
    let worst = -1,
      index = -1;
    for (let i = a + 1; i < b; i++) {
      const u = len2 > 0 ? Math.max(0, Math.min(1, ((xy[i][0] - ax) * dx + (xy[i][1] - ay) * dy) / len2)) : 0,
        e = Math.hypot(xy[i][0] - ax - u * dx, xy[i][1] - ay - u * dy);
      if (e > worst) {
        worst = e;
        index = i;
      }
    }
    if (worst > tolerance) {
      keep[index] = 1;
      stack.push([a, index], [index, b]);
    }
  }
  const out = ring.filter((_, i) => keep[i]);
  return out.length >= 4 ? out : ring;
}
const round = (x) => Number(x.toFixed(7));

// Obstacles from an Overpass API answer (`out tags geom`): buildings,
// forests and woods, tree rows and single trees, masts and towers.
// Closed rings drop their repeated last vertex; large polygons (forests)
// are thinned to `tolerance` metres. Multipolygon relations contribute
// their outer rings (holes ignored, which is conservative).
export function fromOverpass(json, { tolerance = 3 } = {}) {
  const obstacles = [];
  const add = (element, tags, geometry) => {
    const kind = osmKind(tags);
    if (!kind) return;
    const { height, source } = osmObstacleHeight(kind, tags),
      base = {
        id: `${element.type}/${element.id}`,
        kind,
        ...(tags.name && { name: tags.name }),
        height,
        heightSource: source,
      };
    if (geometry.length === 1) {
      const radius =
        kind === "tree"
          ? (metresTag(tags.diameter_crown) || 2 * OSM_HEIGHTS.treeRadius) / 2
          : 3;
      obstacles.push({ ...base, point: geometry[0].map(round), radius });
      return;
    }
    const closed =
      geometry.length >= 4 &&
      geometry[0][0] === geometry.at(-1)[0] &&
      geometry[0][1] === geometry.at(-1)[1];
    if (closed && kind !== "tree_row") {
      const ring = simplifyRing(geometry, kind === "forest" ? tolerance : 0.2).slice(0, -1);
      obstacles.push({ ...base, polygon: ring.map((v) => v.map(round)) });
    } else
      obstacles.push({
        ...base,
        line: geometry.map((v) => v.map(round)),
        radius: kind === "tree_row" ? OSM_HEIGHTS.treeRadius : 3,
      });
  };
  for (const e of json.elements ?? []) {
    const tags = e.tags ?? {};
    if (e.type === "node") add(e, tags, [[e.lon, e.lat]]);
    else if (e.type === "way" && e.geometry)
      add(e, tags, e.geometry.map((g) => [g.lon, g.lat]));
    else if (e.type === "relation")
      for (const m of e.members ?? [])
        if (m.role === "outer" && m.geometry)
          add({ type: "relation", id: `${e.id}/${m.ref}` }, tags, m.geometry.map((g) => [g.lon, g.lat]));
  }
  return obstacles;
}

// Validates obstacles (from the obstacle snapshot or the route file):
// a known kind, a height of 0.5–600 m and one footprint whose box meets
// the route bounds. Returns them unchanged; with `drop` (a snapshot of a
// larger area) obstacles outside the bounds are left out instead.
export function validateObstacles(list, bounds, { drop = false } = {}) {
  if (list === undefined || list === null) return [];
  if (!Array.isArray(list)) throw new Error("obstacles bir dizi olmalı.");
  const lonLat = (v) =>
    Array.isArray(v) && v.length >= 2 && Number.isFinite(v[0]) && Number.isFinite(v[1]);
  const outside = (o) => {
    const vertices = o?.point ? [o.point] : o?.line ?? o?.polygon;
    if (!Array.isArray(vertices) || !vertices.every(lonLat)) return false;
    const lons = vertices.map((v) => v[0]),
      lats = vertices.map((v) => v[1]);
    return (
      Math.max(...lons) < bounds.west ||
      Math.min(...lons) > bounds.east ||
      Math.max(...lats) < bounds.south ||
      Math.min(...lats) > bounds.north
    );
  };
  return (drop ? list.filter((o) => !outside(o)) : list).map((o, i) => {
    const where = `Engel ${i + 1}${o?.name ? ` (${o.name})` : ""}`;
    if (!o || !OBSTACLE_KINDS.includes(o.kind ?? "other"))
      throw new Error(`${where}: bilinmeyen tür.`);
    if (!Number.isFinite(o.height) || o.height < 0.5 || o.height > 600)
      throw new Error(`${where}: yükseklik 0.5–600 m olmalı.`);
    const shapes = ["point", "line", "polygon"].filter((k) => o[k] !== undefined);
    if (shapes.length !== 1) throw new Error(`${where}: tek bir point, line ya da polygon olmalı.`);
    const vertices = o.point ? [o.point] : o.line ?? o.polygon;
    if (
      !vertices.every(lonLat) ||
      (o.line && vertices.length < 2) ||
      (o.polygon && vertices.length < 3)
    )
      throw new Error(`${where}: geçersiz koordinat.`);
    if (o.radius !== undefined && !(o.radius >= 0 && o.radius <= 100))
      throw new Error(`${where}: yarıçap 0–100 m olmalı.`);
    if (outside(o)) throw new Error(`${where}: rota sınırlarının dışında.`);
    return o;
  });
}

// Obstacles in a local frame (`toLocal({longitude, latitude}) → [x, y]`)
// with a grid index of their padded bounding boxes.
export function obstacleIndex(list, toLocal, cell = OBSTACLE_DEFAULTS.cell) {
  const items = list.map((o) => {
    const vertices = (o.point ? [o.point] : o.line ?? o.polygon).map(([longitude, latitude]) =>
        toLocal({ longitude, latitude }),
      ),
      radius = o.polygon ? 0 : o.radius ?? (o.kind === "tree" ? OSM_HEIGHTS.treeRadius : 3),
      xs = vertices.map((v) => v[0]),
      ys = vertices.map((v) => v[1]);
    return {
      ...o,
      shape: o.point ? "point" : o.line ? "line" : "polygon",
      xy: vertices,
      radius,
      box: [Math.min(...xs) - radius, Math.min(...ys) - radius, Math.max(...xs) + radius, Math.max(...ys) + radius],
    };
  });
  const grid = new Map();
  items.forEach((o, i) => {
    for (let x = Math.floor(o.box[0] / cell); x <= Math.floor(o.box[2] / cell); x++)
      for (let y = Math.floor(o.box[1] / cell); y <= Math.floor(o.box[3] / cell); y++) {
        const key = `${x},${y}`;
        if (!grid.has(key)) grid.set(key, []);
        grid.get(key).push(i);
      }
  });
  return { items, grid, cell, tallest: Math.max(0, ...items.map((o) => o.height)) };
}

// Nearest point of segment ab to p.
function nearestOnSegment(p, a, b) {
  const dx = b[0] - a[0],
    dy = b[1] - a[1],
    len2 = dx * dx + dy * dy,
    u = len2 > 0 ? Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2)) : 0;
  return [a[0] + u * dx, a[1] + u * dy];
}
// Horizontal distance from p to an obstacle's footprint (0 inside it) and
// the footprint point nearest to p.
export function footprintDistance(o, p) {
  if (o.shape === "polygon" && pointInPolygon(p, o.xy)) return { distance: 0, nearest: [p[0], p[1]] };
  let best = Infinity,
    nearest = o.xy[0];
  const consider = (q) => {
    const d = Math.hypot(p[0] - q[0], p[1] - q[1]);
    if (d < best) {
      best = d;
      nearest = q;
    }
  };
  if (o.shape === "point") consider(o.xy[0]);
  else {
    const n = o.xy.length,
      segments = o.shape === "polygon" ? n : n - 1;
    for (let i = 0; i < segments; i++) consider(nearestOnSegment(p, o.xy[i], o.xy[(i + 1) % n]));
  }
  // Points and lines have a radius: the footprint's edge is that much nearer.
  if (o.radius > 0 && best > 0) {
    const reach = Math.min(o.radius, best),
      f = reach / best;
    nearest = [nearest[0] + (p[0] - nearest[0]) * f, nearest[1] + (p[1] - nearest[1]) * f];
    best -= reach;
  }
  return { distance: best, nearest };
}
// Obstacles within `radius` metres of p: [{ obstacle, distance, nearest }].
export function obstaclesNear(index, p, radius) {
  if (!index?.items.length) return [];
  const { cell, grid, items } = index,
    seen = new Set(),
    out = [];
  for (let x = Math.floor((p[0] - radius) / cell); x <= Math.floor((p[0] + radius) / cell); x++)
    for (let y = Math.floor((p[1] - radius) / cell); y <= Math.floor((p[1] + radius) / cell); y++)
      for (const i of grid.get(`${x},${y}`) ?? []) {
        if (seen.has(i)) continue;
        seen.add(i);
        const o = items[i];
        if (
          p[0] < o.box[0] - radius ||
          p[0] > o.box[2] + radius ||
          p[1] < o.box[1] - radius ||
          p[1] > o.box[3] + radius
        )
          continue;
        const hit = footprintDistance(o, p);
        if (hit.distance <= radius) out.push({ obstacle: o, ...hit });
      }
  return out;
}
// Tallest obstacle (m above its ground) within `radius` of p, 0 if none.
export function tallestNear(index, p, radius) {
  let tallest = 0;
  for (const hit of obstaclesNear(index, p, radius)) tallest = Math.max(tallest, hit.obstacle.height);
  return tallest;
}
// Obstacle whose footprint covers p (the tallest when several do), or null.
export function obstacleAt(index, p) {
  let found = null;
  for (const hit of obstaclesNear(index, p, 0))
    if (!found || hit.obstacle.height > found.height) found = hit.obstacle;
  return found;
}

const KIND_TEXT = {
  building: "bina",
  mast: "direk",
  tower: "kule",
  tree: "ağaç",
  tree_row: "ağaç sırası",
  forest: "orman örtüsü",
  other: "engel",
};
const SOURCE_TEXT = {
  tag: "etiket",
  levels: "kat sayısı",
  default: "varsayılan",
  assumed: "kabul",
};
export const obstacleText = (o) =>
  `${o.name ? `${o.name} (${KIND_TEXT[o.kind] ?? o.kind})` : KIND_TEXT[o.kind] ?? o.kind} · ${Math.round(o.height)} m (${SOURCE_TEXT[o.heightSource] ?? "dosya"})`;
