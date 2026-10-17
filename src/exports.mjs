import { STRIDE } from "./quadrotor.mjs";
import { toGeodetic } from "./mission-analysis.mjs";

const xml = (s) =>
  String(s)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
const fixed = (x, n) => Number(x.toFixed(n));
// Home ground in the ENU frame: the origin is the CG resting on the pad,
// landingOffset above the ground.
const homeGround = (C, mission) =>
  toGeodetic(C, mission, [0, 0, -mission.settings.landingOffset]);
// Orthometric (mean sea level) height when the route file gives the geoid
// undulation, otherwise the ellipsoidal height.
const seaLevel = (mission, h) =>
  Number.isFinite(mission.geoidUndulation) ? h - mission.geoidUndulation : h;

// Ramer–Douglas–Peucker on 3D points: keeps the knots needed to stay within
// `tolerance` metres of the full path.
export function simplifyPath(points, tolerance) {
  if (points.length < 3) return points.map((_, i) => i);
  const keep = new Uint8Array(points.length);
  keep[0] = keep[points.length - 1] = 1;
  const stack = [[0, points.length - 1]];
  while (stack.length) {
    const [a, b] = stack.pop(),
      A = points[a],
      B = points[b],
      d = B.map((x, k) => x - A[k]),
      len2 = d.reduce((s, x) => s + x * x, 0);
    let worst = -1,
      index = -1;
    for (let i = a + 1; i < b; i++) {
      const P = points[i],
        u =
          len2 > 0
            ? Math.max(
                0,
                Math.min(1, P.reduce((s, x, k) => s + (x - A[k]) * d[k], 0) / len2),
              )
            : 0,
        e = Math.hypot(...P.map((x, k) => x - (A[k] + u * d[k])));
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
  return [...keep.keys()].filter((i) => keep[i]);
}

// Planned survey waypoints: path knots with the terrain-following height,
// thinned to 1.5 m deviation.
// Heights relative to home are geodetic height differences, as MAVLink's
// relative-altitude frame defines them (the flat ENU z would miss the
// Earth's curvature, d² / 2R).
export function plannedWaypoints(C, mission) {
  const table = mission.path.table,
    points = table.map((k, i) => [k.p[0], k.p[1], mission.profile[i]]),
    keep = simplifyPath(points, 1.5),
    home = homeGround(C, mission);
  return keep.map((i) => {
    const g = toGeodetic(C, mission, points[i]);
    return {
      ...g,
      relative: g.height - home.height,
      seaLevel: seaLevel(mission, g.height),
    };
  });
}

function flownTrack(C, mission, everyRows = 30) {
  const sim = mission.simulation,
    track = [];
  for (let i = 0; i < sim.rows; i += everyRows) {
    const r = i * STRIDE;
    track.push({
      time: sim.data[r],
      ...toGeodetic(C, mission, [
        sim.data[r + 1],
        sim.data[r + 2],
        sim.data[r + 3],
      ]),
    });
  }
  return track;
}

// KML 2.2: absolute altitudes are above mean sea level by definition.
export function toKML(C, mission, analysis, name = "Uzungöl keşif uçuşu") {
  const home = homeGround(C, mission);
  const coord = (g, h) =>
    `${g.longitude.toFixed(8)},${g.latitude.toFixed(8)},${h.toFixed(2)}`;
  const planned = plannedWaypoints(C, mission)
    .map((w) => coord(w, w.seaLevel))
    .join(" ");
  const flown = flownTrack(C, mission)
    .map((g) => coord(g, seaLevel(mission, g.height)))
    .join(" ");
  const photos = analysis.capture.photos
    .map((p) => {
      const g = toGeodetic(C, mission, p.position),
        foot = p.corners
          .concat([p.corners[0]])
          .map((c) => {
            const q = toGeodetic(C, mission, c);
            return `${q.longitude.toFixed(8)},${q.latitude.toFixed(8)},0`;
          })
          .join(" ");
      return `      <Placemark>
        <name>IMG_${String(p.index).padStart(4, "0")}</name>
        <description>t = ${p.time.toFixed(1)} s · AGL ${p.agl.toFixed(1)} m · GSD ${(p.gsd * 100).toFixed(2)} cm</description>
        <styleUrl>#photo</styleUrl>
        <MultiGeometry>
          <Point><altitudeMode>absolute</altitudeMode><coordinates>${coord(g, seaLevel(mission, g.height))}</coordinates></Point>
          <Polygon><altitudeMode>clampToGround</altitudeMode><outerBoundaryIs><LinearRing><coordinates>${foot}</coordinates></LinearRing></outerBoundaryIs></Polygon>
        </MultiGeometry>
      </Placemark>`;
    })
    .join("\n");
  return `<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>${xml(name)}</name>
    <description>${xml(`Hedef ${mission.settings.agl} m AGL · ${analysis.coverage.count} fotoğraf · irtifalar ortalama deniz seviyesine göredir${Number.isFinite(mission.geoidUndulation) ? ` (N = ${mission.geoidUndulation} m)` : " (jeoit verisi yok: elipsoit yüksekliği)"}`)}</description>
    <Style id="planned"><LineStyle><color>ff8bedb5</color><width>2</width></LineStyle></Style>
    <Style id="flown"><LineStyle><color>ffd1da7b</color><width>3</width></LineStyle></Style>
    <Style id="photo"><IconStyle><scale>0.5</scale></IconStyle><LineStyle><color>80d1da7b</color></LineStyle><PolyStyle><color>2ed1da7b</color></PolyStyle></Style>
    <Placemark>
      <name>Kalkış / iniş</name>
      <Point><altitudeMode>clampToGround</altitudeMode><coordinates>${coord(home, 0)}</coordinates></Point>
    </Placemark>
    <Placemark>
      <name>Planlanan rota</name>
      <styleUrl>#planned</styleUrl>
      <LineString><altitudeMode>absolute</altitudeMode><coordinates>${planned}</coordinates></LineString>
    </Placemark>
    <Placemark>
      <name>Uçulan iz</name>
      <styleUrl>#flown</styleUrl>
      <LineString><altitudeMode>absolute</altitudeMode><coordinates>${flown}</coordinates></LineString>
    </Placemark>
    <Folder>
      <name>Fotoğraflar</name>
${photos}
    </Folder>
  </Document>
</kml>
`;
}

// QGroundControl mission plan (.plan, JSON). Items use MAV_FRAME_GLOBAL_
// RELATIVE_ALT (3): heights relative to home, taken from the terrain-
// following profile at each retained knot.
export function toQGCPlan(C, mission, analysis) {
  const home = homeGround(C, mission),
    s = mission.settings;
  let id = 0;
  const item = (command, frame, params, altitude = 0) => ({
    AMSLAltAboveTerrain: null,
    Altitude: altitude,
    AltitudeMode: 1,
    autoContinue: true,
    command,
    doJumpId: ++id,
    frame,
    params,
    type: "SimpleItem",
  });
  const waypoints = plannedWaypoints(C, mission);
  const items = [
    // MAV_CMD_NAV_TAKEOFF
    item(
      22,
      3,
      [0, 0, 0, null, home.latitude, home.longitude, fixed(waypoints[0].relative, 2)],
      fixed(waypoints[0].relative, 2),
    ),
    // MAV_CMD_DO_CHANGE_SPEED: ground speed
    item(178, 2, [1, s.cruiseSpeed, -1, 0, 0, 0, 0]),
    // MAV_CMD_DO_MOUNT_CONTROL: gimbal pitch, MAVLink targeting
    item(205, 2, [analysis.capture.pitchDegrees, 0, 0, 0, 0, 0, 2]),
    // MAV_CMD_DO_SET_CAM_TRIGG_DIST: distance, shutter, trigger once now
    item(206, 2, [fixed(analysis.capture.spacing, 2), 0, 1, 0, 0, 0, 0]),
    ...waypoints.map((w) =>
      // MAV_CMD_NAV_WAYPOINT
      item(
        16,
        3,
        [0, 0, 0, null, fixed(w.latitude, 8), fixed(w.longitude, 8), fixed(w.relative, 2)],
        fixed(w.relative, 2),
      ),
    ),
    item(206, 2, [0, 0, 0, 0, 0, 0, 0]),
    // MAV_CMD_NAV_RETURN_TO_LAUNCH
    item(20, 2, [0, 0, 0, 0, 0, 0, 0]),
  ];
  const b = mission.bounds;
  return {
    fileType: "Plan",
    geoFence: {
      circles: [],
      polygons: [
        {
          inclusion: true,
          polygon: [
            [b.south, b.west],
            [b.south, b.east],
            [b.north, b.east],
            [b.north, b.west],
          ],
          version: 1,
        },
      ],
      version: 2,
    },
    groundStation: "QGroundControl",
    mission: {
      cruiseSpeed: s.cruiseSpeed,
      firmwareType: 12,
      globalPlanAltitudeMode: 1,
      hoverSpeed: 5,
      items,
      plannedHomePosition: [
        fixed(home.latitude, 8),
        fixed(home.longitude, 8),
        fixed(seaLevel(mission, home.height), 2),
      ],
      vehicleType: 2,
      version: 2,
    },
    rallyPoints: { points: [], version: 2 },
    version: 1,
  };
}

// GeoJSON (RFC 7946): longitude, latitude and height above the WGS 84
// ellipsoid.
export function toGeoJSON(C, mission, analysis) {
  const home = homeGround(C, mission);
  const position = (g) => [
    fixed(g.longitude, 8),
    fixed(g.latitude, 8),
    fixed(g.height, 2),
  ];
  const features = [
    {
      type: "Feature",
      properties: { kind: "home" },
      geometry: { type: "Point", coordinates: position(home) },
    },
    {
      type: "Feature",
      properties: { kind: "planned", aglTarget: mission.settings.agl },
      geometry: {
        type: "LineString",
        coordinates: plannedWaypoints(C, mission).map(position),
      },
    },
    {
      type: "Feature",
      properties: { kind: "flown" },
      geometry: {
        type: "LineString",
        coordinates: flownTrack(C, mission).map(position),
      },
    },
    ...analysis.capture.photos.map((p) => {
      const ring = p.corners.concat([p.corners[0]]).map((c) => {
        const g = toGeodetic(C, mission, c);
        return [fixed(g.longitude, 8), fixed(g.latitude, 8)];
      });
      return {
        type: "Feature",
        properties: {
          kind: "footprint",
          index: p.index,
          time: fixed(p.time, 2),
          agl: fixed(p.agl, 2),
          gsdCm: fixed(p.gsd * 100, 3),
          headingDeg: fixed((p.heading * 180) / Math.PI, 1),
          gimbalPitchDeg: analysis.capture.pitchDegrees,
        },
        geometry: { type: "Polygon", coordinates: [ring] },
      };
    }),
  ];
  return { type: "FeatureCollection", features };
}

export const CSV_HEADER = [
  "time_s",
  "east_m",
  "north_m",
  "up_m",
  "v_e_ms",
  "v_n_ms",
  "v_u_ms",
  "qw",
  "qx",
  "qy",
  "qz",
  "p_rads",
  "q_rads",
  "r_rads",
  "a_e_ms2",
  "a_n_ms2",
  "a_u_ms2",
  "m1_rads",
  "m2_rads",
  "m3_rads",
  "m4_rads",
  "thrust_N",
  "torque_x_Nm",
  "torque_y_Nm",
  "torque_z_Nm",
  "wind_e_ms",
  "wind_n_ms",
  "wind_u_ms",
  "distance_m",
  "tracking_error_m",
  "phase_code",
  "motor_saturated",
  "battery_soc",
  "battery_v",
  "battery_a",
  "power_w",
  "est_east_m",
  "est_north_m",
  "est_up_m",
  "eph_m",
];
// 10 Hz CSV from the 30 Hz record: everything except the rotor phases.
export function toCSV(mission) {
  const log = mission.simulation,
    lines = [CSV_HEADER.join(",")];
  for (let i = 0; i < log.rows; i += 3) {
    const r = i * STRIDE,
      row = [
        ...log.data.subarray(r, r + 32),
        ...log.data.subarray(r + 36, r + 44),
      ];
    lines.push(row.map((x) => x.toFixed(6)).join(","));
  }
  return lines.join("\n");
}
