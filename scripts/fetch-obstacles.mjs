// Refreshes public/obstacles.json, the obstacle snapshot of the route
// bounds, from OpenStreetMap through the Overpass API:
//   node scripts/fetch-obstacles.mjs            query overpass-api.de
//   node scripts/fetch-obstacles.mjs raw.json   convert a saved answer
// The data is © OpenStreetMap contributors, ODbL 1.0.
import fs from "node:fs";
import { fromOverpass, validateObstacles, OSM_HEIGHTS } from "../src/obstacles.mjs";

const root = new URL("../public/", import.meta.url),
  route = JSON.parse(fs.readFileSync(new URL("points.json", root))),
  b = route.bounds,
  box = `${b.south},${b.west},${b.north},${b.east}`,
  query = `[out:json][timeout:90];(
way["building"](${box});relation["building"](${box});
way["landuse"="forest"](${box});relation["landuse"="forest"](${box});
way["natural"="wood"](${box});relation["natural"="wood"](${box});
way["natural"="tree_row"](${box});node["natural"="tree"](${box});
node["man_made"~"^(mast|tower|communications_tower|chimney|antenna)$"](${box});
way["man_made"~"^(mast|tower)$"](${box});
);out tags geom;`;

let answer;
if (process.argv[2]) answer = JSON.parse(fs.readFileSync(process.argv[2]));
else {
  const response = await fetch("https://overpass-api.de/api/interpreter", {
    method: "POST",
    headers: {
      "Content-Type": "application/x-www-form-urlencoded",
      "User-Agent": "cesium-drone-planner (obstacle snapshot)",
    },
    body: new URLSearchParams({ data: query }),
  });
  if (!response.ok) throw new Error(`Overpass: HTTP ${response.status}`);
  answer = await response.json();
}
const obstacles = validateObstacles(fromOverpass(answer), b),
  counts = {};
for (const o of obstacles) counts[o.kind] = (counts[o.kind] ?? 0) + 1;
const snapshot = {
  source: "OpenStreetMap (Overpass API)",
  license: "ODbL 1.0",
  attribution: "© OpenStreetMap katkıda bulunanlar",
  timestamp: answer.osm3s?.timestamp_osm_base ?? null,
  bounds: b,
  heights: OSM_HEIGHTS,
  counts,
  obstacles,
};
// One obstacle per line keeps diffs of a refresh readable.
fs.writeFileSync(
  new URL("obstacles.json", root),
  `${JSON.stringify({ ...snapshot, obstacles: [] }, null, 2).replace(
    '"obstacles": []',
    `"obstacles": [\n${obstacles.map((o) => `    ${JSON.stringify(o)}`).join(",\n")}\n  ]`,
  )}\n`,
);
console.log(`${obstacles.length} engel yazıldı`, counts, snapshot.timestamp);
