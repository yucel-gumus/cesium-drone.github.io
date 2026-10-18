import { STRIDE, PHASE_NAMES } from "./quadrotor.mjs";
import { terrainAt } from "./flight-planner.mjs";
import { rotate, slerp } from "./flight-math.mjs";
import {
  CAMERA,
  fieldOfView,
  footprint,
  alongTrackExtent,
  triggerDistance,
} from "./camera.mjs";
import { areaCoverage } from "./survey.mjs";

export {
  CAMERA,
  fieldOfView,
  groundSampleDistance,
  cameraAxes,
  footprint,
  alongTrackExtent,
  triggerDistance,
  acrossTrackWidth,
} from "./camera.mjs";
export const PAYLOAD_DEFAULTS = Object.freeze({
  gimbalPitch: -90, // degrees, −90 = nadir
  frontOverlap: 0.8,
});

const headingOf = (q) => {
  const f = rotate(q, [1, 0, 0]);
  return (Math.atan2(f[0], f[1]) + 2 * Math.PI) % (2 * Math.PI);
};
// Phases in which the camera's distance trigger is armed: the whole loop
// of a reconnaissance route, only the grid of an area survey (not the
// transits). Failsafe holds and returns switch it off.
const IMAGING = {
  recon: new Set(["survey", "return"].map((n) => PHASE_NAMES.indexOf(n))),
  survey: new Set([PHASE_NAMES.indexOf("survey")]),
};
// Replays the flown trace and fires the camera every `spacing` metres of
// horizontal travel while imaging (never faster than the camera's minimum
// interval). Each exposure records time, position, heading, AGL, slant
// range, GSD, motion blur and its ground footprint.
export function capturePhotos(mission, settings = {}, camera = CAMERA) {
  const sim = mission.simulation,
    pitchDegrees = settings.gimbalPitch ?? PAYLOAD_DEFAULTS.gimbalPitch,
    overlap = settings.frontOverlap ?? PAYLOAD_DEFAULTS.frontOverlap,
    pitch = (pitchDegrees * Math.PI) / 180,
    spacing = triggerDistance(mission.settings.agl, pitchDegrees, overlap, camera),
    imaging = IMAGING[mission.type ?? "recon"];
  const photos = [];
  let traveled = 0,
    next = 0,
    lastTime = -Infinity,
    previous = null;
  for (let i = 0; i < sim.rows; i++) {
    const r = i * STRIDE,
      phase = sim.data[r + 30];
    if (!imaging.has(phase)) {
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
// consecutive exposures and the imaged corridor area. With an area survey
// (`survey`, see lawnmower()) also the achieved side overlap between
// neighbouring lines and the fraction of the polygon imaged.
export function coverageSummary(capture, camera = CAMERA, survey = null) {
  const { photos } = capture;
  if (!photos.length)
    return {
      count: 0,
      gsdMean: 0,
      gsdMin: 0,
      gsdMax: 0,
      minOverlap: 0,
      meanOverlap: 0,
      areaHectares: 0,
      maxBlur: 0,
      ...(survey && { areaCovered: 0, minSideOverlap: 0, meanSideOverlap: 0 }),
    };
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
  const summary = {
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
  if (survey) {
    Object.assign(summary, sideOverlap(photos, survey));
    summary.areaHectares = Math.abs(survey.areaM2) / 10000;
    summary.areaCovered = areaCoverage(
      survey.polygon,
      photos.map((p) => p.corners),
    );
  }
  return summary;
}

// Achieved side overlap of an area survey: photos taken on a line (heading
// within 20° of it, within half a spacing of its axis) give the line's
// flown cross position (median) and image width (near edge, mean); the
// overlap of two neighbouring lines is 1 − separation / mean width.
export function sideOverlap(photos, survey) {
  const { direction, normal, spacing, lines } = survey,
    across = lines.map(() => []),
    widths = lines.map(() => []);
  for (const photo of photos) {
    const forward = [Math.sin(photo.heading), Math.cos(photo.heading)];
    if (
      Math.abs(forward[0] * direction[0] + forward[1] * direction[1]) <
      Math.cos((20 * Math.PI) / 180)
    )
      continue;
    const u = photo.position[0] * direction[0] + photo.position[1] * direction[1],
      w = photo.position[0] * normal[0] + photo.position[1] * normal[1];
    const k = lines.findIndex(
      (line) =>
        Math.abs(w - line.w) < spacing / 2 &&
        u >= Math.min(line.from, line.to) &&
        u <= Math.max(line.from, line.to),
    );
    if (k < 0) continue;
    across[k].push(w);
    const [a, b] = photo.corners;
    widths[k].push(Math.hypot(b[0] - a[0], b[1] - a[1]));
  }
  const median = (xs) => {
    const sorted = xs.slice().sort((a, b) => a - b);
    return sorted[sorted.length >> 1];
  };
  const flown = lines
    .map((_, k) =>
      across[k].length
        ? {
            w: median(across[k]),
            width: widths[k].reduce((s, x) => s + x, 0) / widths[k].length,
          }
        : null,
    )
    .filter(Boolean)
    .sort((a, b) => a.w - b.w);
  const overlaps = [];
  for (let k = 1; k < flown.length; k++)
    overlaps.push(
      1 - (flown[k].w - flown[k - 1].w) / ((flown[k].width + flown[k - 1].width) / 2),
    );
  return {
    linesImaged: flown.length,
    minSideOverlap: overlaps.length ? Math.min(...overlaps) : 0,
    meanSideOverlap: overlaps.length
      ? overlaps.reduce((s, x) => s + x, 0) / overlaps.length
      : 0,
  };
}
