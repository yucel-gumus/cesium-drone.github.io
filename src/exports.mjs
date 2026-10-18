import { STRIDE, replayAt } from "./quadrotor.mjs";
import { toGeodetic } from "./mission-analysis.mjs";
import { CAMERA, groundSampleDistance } from "./camera.mjs";
import { SURVEY_DEFAULTS } from "./survey.mjs";

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
// `tolerance` metres of the full path, and every index in `required`.
export function simplifyPath(points, tolerance, required = []) {
  if (points.length < 3) return points.map((_, i) => i);
  const keep = new Uint8Array(points.length);
  keep[0] = keep[points.length - 1] = 1;
  const anchors = [
    ...new Set([0, ...required.filter((i) => i > 0 && i < points.length - 1), points.length - 1]),
  ].sort((a, b) => a - b);
  for (const i of anchors) keep[i] = 1;
  const stack = anchors.slice(1).map((b, k) => [anchors[k], b]);
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

// Path knot at or after arc length s.
const knotAt = (table, s) => {
  let i = 0;
  while (i < table.length - 1 && table[i].s < s) i++;
  return i;
};
// Planned survey waypoints: path knots with the terrain-following height,
// thinned to 1.5 m deviation. An area survey keeps the knots where its grid
// starts and ends (`gridStart` / `gridEnd` mark those waypoints), so the
// camera trigger can be switched there.
// Heights relative to home are geodetic height differences, as MAVLink's
// relative-altitude frame defines them (the flat ENU z would miss the
// Earth's curvature, d² / 2R).
export function plannedWaypoints(C, mission) {
  const table = mission.path.table,
    points = table.map((k, i) => [k.p[0], k.p[1], mission.profile[i]]),
    legs = mission.survey?.legs,
    grid = legs ? [knotAt(table, legs.start), knotAt(table, legs.end)] : [],
    keep = simplifyPath(points, 1.5, grid),
    home = homeGround(C, mission);
  return keep.map((i) => {
    const g = toGeodetic(C, mission, points[i]);
    return {
      ...g,
      relative: g.height - home.height,
      seaLevel: seaLevel(mission, g.height),
      ...(i === grid[0] && { gridStart: true }),
      ...(i === grid[1] && { gridEnd: true }),
    };
  });
}

// Fault and failsafe events of the flown solve at the flown position.
function failsafeEvents(C, mission) {
  return (mission.simulation.failsafe?.events ?? []).map((e) => ({
    ...e,
    ...toGeodetic(C, mission, replayAt(mission.simulation, e.time).p),
  }));
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
  const area = mission.survey
    ? `
    <Placemark>
      <name>Haritalama alanı</name>
      <description>${xml(`${(Math.abs(mission.survey.areaM2) / 1e4).toFixed(2)} ha · ${mission.survey.count} hat · ${mission.survey.spacing.toFixed(1)} m aralık · %${Math.round(mission.survey.sideOverlap * 100)} yan bindirme`)}</description>
      <styleUrl>#area</styleUrl>
      <Polygon><altitudeMode>clampToGround</altitudeMode><outerBoundaryIs><LinearRing><coordinates>${mission.survey.vertices
        .concat([mission.survey.vertices[0]])
        .map((v) => `${v.longitude.toFixed(8)},${v.latitude.toFixed(8)},0`)
        .join(" ")}</coordinates></LinearRing></outerBoundaryIs></Polygon>
    </Placemark>`
    : "";
  const rally = (mission.rally ?? [])
    .map(
      (r) => `
    <Placemark>
      <name>${xml(`Rally · ${r.name}`)}</name>
      <styleUrl>#rally</styleUrl>
      <Point><altitudeMode>clampToGround</altitudeMode><coordinates>${r.longitude.toFixed(8)},${r.latitude.toFixed(8)},0</coordinates></Point>
    </Placemark>`,
    )
    .join("");
  const failsafe = failsafeEvents(C, mission)
    .map(
      (e) => `      <Placemark>
        <name>${xml(e.label)}</name>
        <description>${xml(`t = ${e.time.toFixed(1)} s · ${e.detail}`)}</description>
        <styleUrl>#event</styleUrl>
        <Point><altitudeMode>absolute</altitudeMode><coordinates>${coord(e, seaLevel(mission, e.height))}</coordinates></Point>
      </Placemark>`,
    )
    .join("\n");
  return `<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>${xml(name)}</name>
    <description>${xml(`Hedef ${mission.settings.agl} m AGL · ${analysis.coverage.count} fotoğraf · irtifalar ortalama deniz seviyesine göredir${Number.isFinite(mission.geoidUndulation) ? ` (N = ${mission.geoidUndulation} m)` : " (jeoit verisi yok: elipsoit yüksekliği)"}`)}</description>
    <Style id="planned"><LineStyle><color>ff8bedb5</color><width>2</width></LineStyle></Style>
    <Style id="flown"><LineStyle><color>ffd1da7b</color><width>3</width></LineStyle></Style>
    <Style id="photo"><IconStyle><scale>0.5</scale></IconStyle><LineStyle><color>80d1da7b</color></LineStyle><PolyStyle><color>2ed1da7b</color></PolyStyle></Style>
    <Style id="area"><LineStyle><color>ff7ad2ff</color><width>2</width></LineStyle><PolyStyle><color>1a7ad2ff</color></PolyStyle></Style>
    <Style id="event"><IconStyle><color>ff7a8fff</color></IconStyle></Style>
    <Style id="rally"><IconStyle><color>ff7ad2ff</color></IconStyle></Style>${area}${rally}
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
    </Folder>${
      failsafe
        ? `
    <Folder>
      <name>Arıza ve failsafe</name>
${failsafe}
    </Folder>`
        : ""
    }
  </Document>
</kml>
`;
}

// QGroundControl mission plan (.plan, JSON). Items use MAV_FRAME_GLOBAL_
// RELATIVE_ALT (3): heights relative to home, taken from the terrain-
// following profile at each retained knot. An area survey is written as
// plain waypoints too (its lines and turns), with the distance trigger
// switched on at the first line and off after the last one.
// QGroundControl survey complex item (Survey v5 / TransectStyleComplexItem
// v2 / CameraCalc v2, the keys QGC's loader validates) around the grid's
// simple items `grid`. The distance mode is "calculated above terrain", for
// which QGC flies the loaded items as they are: our terrain-following
// heights. Editing the survey in QGC regenerates the transects from the
// polygon, angle and camera with QGC's own terrain query.
function surveyComplexItem(C, mission, analysis, grid, camera = CAMERA) {
  const sv = mission.survey,
    s = mission.settings,
    latLon = (xy) => {
      const g = toGeodetic(C, mission, [xy[0], xy[1], 0]);
      return [fixed(g.latitude, 8), fixed(g.longitude, 8)];
    };
  // QGC's grid angle: transect bearing clockwise from north, folded into
  // (−90°, 90°] (the ENU tangent plane's north is the local meridian).
  let angle = (Math.atan2(sv.direction[0], sv.direction[1]) * 180) / Math.PI;
  angle = ((angle % 180) + 180) % 180;
  if (angle > 90) angle -= 180;
  return {
    TransectStyleComplexItem: {
      CameraCalc: {
        AdjustedFootprintFrontal: fixed(analysis.capture.spacing, 2),
        AdjustedFootprintSide: fixed(sv.spacing, 2),
        CameraName: "Custom Camera",
        // QGroundControlQmlGlobal::AltitudeFrameCalcAboveTerrain
        DistanceMode: 3,
        DistanceToSurface: s.agl,
        FixedOrientation: false,
        FocalLength: camera.focalLength,
        FrontalOverlap: fixed(analysis.capture.overlap * 100, 1),
        ImageDensity: fixed(groundSampleDistance(s.agl, camera) * 100, 3),
        ImageHeight: camera.imageHeight,
        ImageWidth: camera.imageWidth,
        // Landscape: the image width (sensorWidth) lies across track.
        Landscape: true,
        MinTriggerInterval: camera.minInterval,
        SensorHeight: camera.sensorHeight,
        SensorWidth: camera.sensorWidth,
        SideOverlap: fixed(sv.sideOverlap * 100, 1),
        ValueSetIsDistance: true,
        version: 2,
      },
      CameraShots: analysis.capture.photos.length,
      // The trigger stays armed through the turns between lines.
      CameraTriggerInTurnAround: true,
      HoverAndCapture: false,
      Items: grid.map(({ autoContinue, command, frame, params, type }) => ({
        autoContinue,
        command,
        doJumpId: 0,
        frame,
        params,
        type,
      })),
      Refly90Degrees: false,
      TerrainAdjustMaxClimbRate: s.followClimbRate,
      TerrainAdjustMaxDescentRate: s.followDescentRate,
      TerrainAdjustTolerance: s.clearanceTolerance,
      TerrainFlightSpeed: s.cruiseSpeed,
      TurnAroundDistance: SURVEY_DEFAULTS.turnaround,
      // Line ends in flight order.
      VisualTransectPoints: sv.lines.flatMap((l) => [latLon(l.a), latLon(l.b)]),
      version: 2,
    },
    angle: fixed(angle, 2),
    complexItemType: "survey",
    entryLocation: 0,
    flyAlternateTransects: false,
    // QGCMapPolygon::saveToJson: [latitude, longitude] vertices.
    polygon: sv.vertices.map((v) => [fixed(v.latitude, 8), fixed(v.longitude, 8)]),
    splitConcavePolygons: false,
    type: "ComplexItem",
    version: 5,
  };
}

// QGroundControl .plan. An area survey's grid goes into a Survey complex
// item unless `surveyItem` is false, which lists it as plain waypoints.
export function toQGCPlan(C, mission, analysis, { surveyItem = true } = {}) {
  const home = homeGround(C, mission),
    s = mission.settings;
  const item = (command, frame, params, altitude = 0) => ({
    AMSLAltAboveTerrain: null,
    Altitude: altitude,
    AltitudeMode: 1,
    autoContinue: true,
    command,
    doJumpId: 0,
    frame,
    params,
    type: "SimpleItem",
  });
  const waypoints = plannedWaypoints(C, mission),
    trigger = () =>
      // MAV_CMD_DO_SET_CAM_TRIGG_DIST: distance, shutter, trigger once now
      item(206, 2, [fixed(analysis.capture.spacing, 2), 0, 1, 0, 0, 0, 0]),
    triggerOff = () => item(206, 2, [0, 0, 0, 0, 0, 0, 0]),
    // MAV_CMD_NAV_WAYPOINT
    waypoint = (w) =>
      item(
        16,
        3,
        [0, 0, 0, null, fixed(w.latitude, 8), fixed(w.longitude, 8), fixed(w.relative, 2)],
        fixed(w.relative, 2),
      );
  let imaging;
  if (mission.survey) {
    // Grid: from the first line's start to the last line's end, the trigger
    // armed after the first waypoint and disarmed after the last.
    const start = waypoints.findIndex((w) => w.gridStart),
      end = waypoints.findIndex((w) => w.gridEnd),
      grid = [
        waypoint(waypoints[start]),
        trigger(),
        ...waypoints.slice(start + 1, end + 1).map(waypoint),
        triggerOff(),
      ];
    imaging = [
      ...waypoints.slice(0, start).map(waypoint),
      ...(surveyItem ? [surveyComplexItem(C, mission, analysis, grid)] : grid),
      ...waypoints.slice(end + 1).map(waypoint),
    ];
  } else imaging = [trigger(), ...waypoints.map(waypoint), triggerOff()];
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
    ...imaging,
    // MAV_CMD_NAV_RETURN_TO_LAUNCH
    item(20, 2, [0, 0, 0, 0, 0, 0, 0]),
  ];
  // Jump ids follow the mission item order, through the complex item.
  let jump = 0;
  for (const x of items)
    for (const y of x.type === "ComplexItem" ? x.TransectStyleComplexItem.Items : [x])
      y.doJumpId = ++jump;
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
    // Rally points relative to home, at the planned return height.
    rallyPoints: {
      points: (mission.rally ?? []).map((r) => [
        fixed(r.latitude, 8),
        fixed(r.longitude, 8),
        fixed(mission.reference.endHeight, 2),
      ]),
      version: 2,
    },
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
    ...(mission.survey
      ? [
          {
            type: "Feature",
            properties: {
              kind: "surveyArea",
              areaHa: fixed(Math.abs(mission.survey.areaM2) / 1e4, 3),
              lines: mission.survey.count,
              lineSpacingM: fixed(mission.survey.spacing, 2),
              sideOverlap: mission.survey.sideOverlap,
            },
            geometry: {
              type: "Polygon",
              // RFC 7946: exterior rings are counter-clockwise.
              coordinates: [
                mission.survey.vertices
                  .concat([mission.survey.vertices[0]])
                  .map((v) => [fixed(v.longitude, 8), fixed(v.latitude, 8)]),
              ],
            },
          },
        ]
      : []),
    ...(mission.rally ?? []).map((r) => ({
      type: "Feature",
      properties: { kind: "rally", name: r.name },
      geometry: {
        type: "Point",
        coordinates: [fixed(r.longitude, 8), fixed(r.latitude, 8)],
      },
    })),
    ...failsafeEvents(C, mission).map((e) => ({
      type: "Feature",
      properties: {
        kind: "event",
        event: e.kind,
        label: e.label,
        detail: e.detail,
        time: fixed(e.time, 2),
      },
      geometry: { type: "Point", coordinates: position(e) },
    })),
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
  "wind_est_e_ms",
  "wind_est_n_ms",
];
// 10 Hz CSV from the 30 Hz record: everything except the rotor phases.
export function toCSV(mission) {
  const log = mission.simulation,
    lines = [CSV_HEADER.join(",")];
  for (let i = 0; i < log.rows; i += 3) {
    const r = i * STRIDE,
      row = [
        ...log.data.subarray(r, r + 32),
        ...log.data.subarray(r + 36, r + 46),
      ];
    lines.push(row.map((x) => x.toFixed(6)).join(","));
  }
  return lines.join("\n");
}

// PX4 ULog (.ulg, file format version 1) of the flown solve: PX4 frames
// (NED world, FRD body) and the quad X motor numbering, so the flight opens
// in PX4 Flight Review, PlotJuggler or pyulog, and the calibration tool
// (sysid.mjs fromULog) reads it back. Topics at the 30 Hz record rate:
// vehicle_attitude, vehicle_local_position, sensor_combined (gyro and
// specific force), esc_status (rotor speeds as RPM) and battery_status.
const ULOG_FORMATS = [
  "vehicle_attitude:uint64_t timestamp;float[4] q;",
  "vehicle_local_position:uint64_t timestamp;float x;float y;float z;float vx;float vy;float vz;",
  "sensor_combined:uint64_t timestamp;float[3] gyro_rad;float[3] accelerometer_m_s2;",
  "esc_report:int32_t esc_rpm;float esc_voltage;float esc_current;",
  "esc_status:uint64_t timestamp;uint8_t esc_count;uint8_t[7] _padding0;esc_report[8] esc;",
  "battery_status:uint64_t timestamp;float voltage_v;float current_a;float remaining;",
];
// Our motor order (FR, BR, BL, FL) as PX4 ESC channels (1 FR, 2 BL, 3 FL,
// 4 BR): channel i carries our motor ESC_FROM[i].
const ESC_FROM = [0, 2, 3, 1];
export function toULog(mission) {
  const sim = mission.simulation,
    d = sim.data,
    encoder = new TextEncoder(),
    chunks = [];
  let length = 0;
  const push = (bytes) => {
    chunks.push(bytes);
    length += bytes.length;
  };
  const message = (type, body) => {
    const head = new Uint8Array(3);
    new DataView(head.buffer).setUint16(0, body.length, true);
    head[2] = type.charCodeAt(0);
    push(head);
    push(body);
  };
  const header = new Uint8Array(16);
  header.set([0x55, 0x4c, 0x6f, 0x67, 0x01, 0x12, 0x35, 0x01]);
  new DataView(header.buffer).setBigUint64(8, 0n, true);
  push(header);
  message("B", new Uint8Array(40));
  for (const [key, value] of [
    ["sys_name", "cesium-drone-sim"],
    ["ver_hw", "SIMULATION"],
  ]) {
    const k = encoder.encode(`char[${value.length}] ${key}`),
      v = encoder.encode(value),
      body = new Uint8Array(1 + k.length + v.length);
    body[0] = k.length;
    body.set(k, 1);
    body.set(v, 1 + k.length);
    message("I", body);
  }
  for (const format of ULOG_FORMATS) message("F", encoder.encode(format));
  const topics = ["vehicle_attitude", "vehicle_local_position", "sensor_combined", "esc_status", "battery_status"];
  topics.forEach((name, id) => {
    const n = encoder.encode(name),
      body = new Uint8Array(3 + n.length);
    new DataView(body.buffer).setUint16(1, id, true);
    body.set(n, 3);
    message("A", body);
  });
  const data = (id, size, write) => {
    const body = new Uint8Array(2 + size),
      view = new DataView(body.buffer);
    view.setUint16(0, id, true);
    write(view, 2);
    message("D", body);
  };
  const ENU_NED = [0, Math.SQRT1_2, Math.SQRT1_2, 0],
    mul = (a, b) => [
      a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
      a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
      a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
      a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ],
    rotateBack = (q, v) => {
      // v_body = q* v q
      const c = [q[0], -q[1], -q[2], -q[3]],
        r = mul(mul(c, [0, ...v]), q);
      return [r[1], r[2], r[3]];
    };
  for (let i = 0; i < sim.rows; i++) {
    const r = i * STRIDE,
      us = BigInt(Math.round(d[r] * 1e6) + 1_000_000),
      q = [d[r + 7], d[r + 8], d[r + 9], d[r + 10]],
      qNed = mul(mul(ENU_NED, q), [0, 1, 0, 0]),
      force = rotateBack(q, [d[r + 14], d[r + 15], d[r + 16] + 9.80665]);
    data(0, 8 + 16, (v, o) => {
      v.setBigUint64(o, us, true);
      qNed.forEach((x, k) => v.setFloat32(o + 8 + 4 * k, x, true));
    });
    data(1, 8 + 24, (v, o) => {
      v.setBigUint64(o, us, true);
      [d[r + 2], d[r + 1], -d[r + 3], d[r + 5], d[r + 4], -d[r + 6]].forEach((x, k) =>
        v.setFloat32(o + 8 + 4 * k, x, true),
      );
    });
    data(2, 8 + 24, (v, o) => {
      v.setBigUint64(o, us, true);
      [d[r + 11], -d[r + 12], -d[r + 13], force[0], -force[1], -force[2]].forEach((x, k) =>
        v.setFloat32(o + 8 + 4 * k, x, true),
      );
    });
    data(3, 8 + 8 + 8 * 12, (v, o) => {
      v.setBigUint64(o, us, true);
      v.setUint8(o + 8, 4);
      for (let k = 0; k < 4; k++) {
        const at = o + 16 + 12 * k;
        v.setInt32(at, Math.round((d[r + 17 + ESC_FROM[k]] * 60) / (2 * Math.PI)), true);
        v.setFloat32(at + 4, d[r + 37], true);
        v.setFloat32(at + 8, d[r + 38] / 4, true);
      }
    });
    data(4, 8 + 12, (v, o) => {
      v.setBigUint64(o, us, true);
      v.setFloat32(o + 8, d[r + 37], true);
      v.setFloat32(o + 12, d[r + 38], true);
      v.setFloat32(o + 16, d[r + 36], true);
    });
  }
  const out = new Uint8Array(length);
  let at = 0;
  for (const c of chunks) {
    out.set(c, at);
    at += c.length;
  }
  return out;
}
