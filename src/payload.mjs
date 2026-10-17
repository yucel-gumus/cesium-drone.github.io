import { STRIDE } from "./quadrotor.mjs";
import { terrainAt } from "./flight-planner.mjs";
import { rotate, slerp } from "./flight-math.mjs";

// Generic 1-inch, 20 MP mapping camera on a 3-axis stabilised gimbal. The
// image's long side lies across the flight direction.
export const CAMERA = Object.freeze({
  sensorWidth: 13.2, // mm, across track
  sensorHeight: 8.8, // mm, along track
  focalLength: 8.8, // mm
  imageWidth: 5472, // px
  imageHeight: 3648, // px
  minInterval: 0.7, // s, fastest sustained capture
  exposure: 1 / 1000, // s
});
export const PAYLOAD_DEFAULTS = Object.freeze({
  gimbalPitch: -90, // degrees, −90 = nadir
  frontOverlap: 0.8,
});

export function fieldOfView(camera = CAMERA) {
  return {
    horizontal: 2 * Math.atan(camera.sensorWidth / 2 / camera.focalLength),
    vertical: 2 * Math.atan(camera.sensorHeight / 2 / camera.focalLength),
  };
}
// Ground sampling distance (m/px) at a slant range along the optical axis.
export function groundSampleDistance(range, camera = CAMERA) {
  return (range * camera.sensorWidth) / (camera.focalLength * camera.imageWidth);
}
// Camera axes in ENU for a heading (radians clockwise from north) and a
// gimbal pitch (radians, negative below the horizon); roll is stabilised to
// zero. Returns the optical axis, image right and image up.
export function cameraAxes(heading, pitch) {
  const h = [Math.sin(heading), Math.cos(heading), 0],
    right = [Math.cos(heading), -Math.sin(heading), 0],
    c = Math.cos(pitch),
    s = Math.sin(pitch);
  return {
    forward: [c * h[0], c * h[1], s],
    right,
    up: [-s * h[0], -s * h[1], c],
  };
}
// Footprint of one exposure on the local ground plane z = groundZ: rays
// through the four sensor corners, clipped at 6 × AGL for rays at or above
// the horizon. Corners run left-back, right-back, right-front, left-front.
export function footprint(position, heading, pitch, groundZ, camera = CAMERA) {
  const { forward, right, up } = cameraAxes(heading, pitch),
    height = Math.max(0.1, position[2] - groundZ),
    w = camera.sensorWidth / 2,
    hgt = camera.sensorHeight / 2;
  const hit = (dir) => {
    const reach = 6 * height;
    if (dir[2] < -1e-6) {
      const t = (groundZ - position[2]) / dir[2],
        x = position[0] + t * dir[0],
        y = position[1] + t * dir[1];
      if (Math.hypot(x - position[0], y - position[1]) <= reach)
        return [x, y, groundZ];
    }
    const horizontal = Math.hypot(dir[0], dir[1]) || 1;
    return [
      position[0] + (reach * dir[0]) / horizontal,
      position[1] + (reach * dir[1]) / horizontal,
      groundZ,
    ];
  };
  const ray = (x, y) =>
    [0, 1, 2].map(
      (i) => forward[i] * camera.focalLength + right[i] * x + up[i] * y,
    );
  const corners = [
    hit(ray(-w, -hgt)),
    hit(ray(w, -hgt)),
    hit(ray(w, hgt)),
    hit(ray(-w, hgt)),
  ];
  const range = forward[2] < -1e-6 ? -height / forward[2] : 6 * height;
  return { corners, range, gsd: groundSampleDistance(range, camera) };
}
// Along-track ground extent of one image on flat ground at a given AGL.
// The image's top and bottom edges leave the camera at pitch ± half the
// vertical field of view; a ray depressed by angle δ below the horizon meets
// the ground agl / tan δ ahead (negative behind, for δ beyond 90°). Rays at
// or above the horizon are clipped at 6 × AGL.
export function alongTrackExtent(agl, pitchDegrees, camera = CAMERA) {
  const pitch = (pitchDegrees * Math.PI) / 180,
    half = fieldOfView(camera).vertical / 2;
  const ahead = (depression) =>
    depression <= 0 ? 6 * agl : Math.min(6 * agl, agl / Math.tan(depression));
  return Math.abs(ahead(-(pitch + half)) - ahead(-(pitch - half)));
}
// Distance-based trigger spacing, as mission planners set it.
export function triggerDistance(agl, pitchDegrees, overlap, camera = CAMERA) {
  return (1 - overlap) * alongTrackExtent(agl, pitchDegrees, camera);
}

const headingOf = (q) => {
  const f = rotate(q, [1, 0, 0]);
  return (Math.atan2(f[0], f[1]) + 2 * Math.PI) % (2 * Math.PI);
};
// Replays the flown trace and fires the camera every `spacing` metres of
// horizontal travel during the survey and return legs (never faster than
// the camera's minimum interval). Each exposure records time, position,
// heading, AGL, slant range, GSD, motion blur and its ground footprint.
export function capturePhotos(mission, settings = {}, camera = CAMERA) {
  const sim = mission.simulation,
    pitchDegrees = settings.gimbalPitch ?? PAYLOAD_DEFAULTS.gimbalPitch,
    overlap = settings.frontOverlap ?? PAYLOAD_DEFAULTS.frontOverlap,
    pitch = (pitchDegrees * Math.PI) / 180,
    spacing = triggerDistance(mission.settings.agl, pitchDegrees, overlap, camera);
  const photos = [];
  let traveled = 0,
    next = 0,
    lastTime = -Infinity,
    previous = null;
  for (let i = 0; i < sim.rows; i++) {
    const r = i * STRIDE,
      phase = sim.data[r + 30];
    if (phase !== 2 && phase !== 3) {
      previous = null;
      continue;
    }
    const row = {
      t: sim.data[r],
      p: [sim.data[r + 1], sim.data[r + 2], sim.data[r + 3]],
      v: [sim.data[r + 4], sim.data[r + 5], sim.data[r + 6]],
      q: [sim.data[r + 7], sim.data[r + 8], sim.data[r + 9], sim.data[r + 10]],
    };
    if (!previous) {
      previous = row;
      continue;
    }
    const step = Math.hypot(row.p[0] - previous.p[0], row.p[1] - previous.p[1]);
    while (step > 0 && traveled + step >= next) {
      const f = (next - traveled) / step,
        t = previous.t + (row.t - previous.t) * f;
      if (t - lastTime >= camera.minInterval) {
        const p = previous.p.map((x, k) => x + (row.p[k] - x) * f),
          v = previous.v.map((x, k) => x + (row.v[k] - x) * f),
          q = slerp(previous.q, row.q, f),
          heading = headingOf(q),
          ground = terrainAt(mission, p).height,
          shot = footprint(p, heading, pitch, ground, camera),
          speed = Math.hypot(...v);
        photos.push({
          index: photos.length + 1,
          time: t,
          position: p,
          heading,
          pitch,
          agl: p[2] - ground,
          ground,
          range: shot.range,
          gsd: shot.gsd,
          blur: (speed * camera.exposure) / shot.gsd,
          corners: shot.corners,
          distance: next,
        });
        lastTime = t;
      }
      next += spacing;
    }
    traveled += step;
    previous = row;
  }
  return { photos, spacing, pitchDegrees, overlap };
}
// Coverage statistics: GSD spread, achieved forward overlap between
// consecutive exposures and the imaged corridor area.
export function coverageSummary(capture, camera = CAMERA) {
  const { photos } = capture;
  if (!photos.length)
    return { count: 0, gsdMean: 0, gsdMin: 0, gsdMax: 0, minOverlap: 0, meanOverlap: 0, areaHectares: 0, maxBlur: 0 };
  const gsd = photos.map((p) => p.gsd),
    overlaps = [];
  let area = 0;
  for (let i = 1; i < photos.length; i++) {
    const a = photos[i - 1],
      b = photos[i],
      step = Math.hypot(
        b.position[0] - a.position[0],
        b.position[1] - a.position[1],
      ),
      extent = alongTrackExtent(a.agl, capture.pitchDegrees, camera);
    overlaps.push(Math.max(0, 1 - step / extent));
    const swath =
      (2 * a.range * Math.tan(fieldOfView(camera).horizontal / 2));
    area += step * swath;
  }
  return {
    count: photos.length,
    gsdMean: gsd.reduce((s, x) => s + x, 0) / gsd.length,
    gsdMin: Math.min(...gsd),
    gsdMax: Math.max(...gsd),
    minOverlap: overlaps.length ? Math.min(...overlaps) : 0,
    meanOverlap: overlaps.length
      ? overlaps.reduce((s, x) => s + x, 0) / overlaps.length
      : 0,
    areaHectares: area / 10000,
    maxBlur: Math.max(...photos.map((p) => p.blur)),
  };
}
