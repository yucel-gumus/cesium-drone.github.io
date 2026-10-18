// Camera geometry shared by the payload replay and the area-survey planner.
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

// Across-track ground width of one image on flat ground at a given AGL:
// the near (bottom) edge, the narrowest one for a forward-tilted camera and
// equal to the far edge at nadir. Area surveys space their lines from it.
export function acrossTrackWidth(agl, pitchDegrees, camera = CAMERA) {
  const [leftBack, rightBack] = footprint(
    [0, 0, agl],
    0,
    (pitchDegrees * Math.PI) / 180,
    0,
    camera,
  ).corners;
  return Math.hypot(rightBack[0] - leftBack[0], rightBack[1] - leftBack[1]);
}
