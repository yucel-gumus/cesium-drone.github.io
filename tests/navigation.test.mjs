import test from "node:test";
import assert from "node:assert/strict";
import { makeNavigation, MAG } from "../src/navigation.mjs";
import { attitudeFromForce, qmul, conjugate } from "../src/flight-math.mjs";
import { AIRFRAME, dragForce, meanWind } from "../src/quadrotor.mjs";

const hz = 120,
  dt = 1 / hz,
  g = AIRFRAME.gravity,
  m = AIRFRAME.mass;
// A quadrotor circling at 8 m/s on a 100 m radius at 80 m through a steady
// wind: the thrust holds the centripetal acceleration and the drag of the
// air-relative velocity, so its tilt is consistent with what drag fusion
// expects. Returns truth samples at the control rate.
function circle(seconds, wind = meanWind(2)) {
  const samples = [],
    w = 8 / 100;
  let previous = null;
  for (let k = 0; k <= seconds * hz; k++) {
    const t = k * dt,
      p = [100 * Math.cos(w * t), 100 * Math.sin(w * t), 80],
      v = [-8 * Math.sin(w * t), 8 * Math.cos(w * t), 0],
      a = [-8 * w * Math.cos(w * t), -8 * w * Math.sin(w * t), 0],
      air = [v[0] - wind[0], v[1] - wind[1], 0],
      speed = Math.hypot(...air),
      drag = air.map((x) => (-dragForce(speed) * x) / Math.max(speed, 1e-9)),
      force = [0, 1, 2].map((i) => m * a[i] - drag[i] + (i === 2 ? m * g : 0)),
      q = attitudeFromForce(force, Math.atan2(v[1], v[0]));
    // Body rate from the rotation over the step (delta angle).
    let rates = [0, 0, 0];
    if (previous) {
      let d = qmul(conjugate(previous), q);
      if (d[0] < 0) d = d.map((x) => -x);
      const s = Math.hypot(d[1], d[2], d[3]);
      rates = s > 1e-12 ? d.slice(1).map((x) => (x * 2 * Math.atan2(s, d[0])) / s / dt) : [0, 0, 0];
    }
    previous = q;
    samples.push({ t, p, v, a, q, rates });
  }
  return samples;
}
const drag = {
  mass: m,
  area: 0.5 * AIRFRAME.airDensity * AIRFRAME.dragArea,
  linear: AIRFRAME.linearDrag,
};
function fly(samples, { mode = "rtk", seed = 3, outage = null, withDrag = true } = {}) {
  const nav = makeNavigation(mode, seed, withDrag ? { drag } : {}),
    log = [];
  for (const s of samples) {
    if (outage) nav.gnssAvailable = !(s.t >= outage[0] && s.t < outage[1]);
    nav.step(s.p, s.v, s.a, dt, s.q, s.rates, { rotorRatio: 1 });
    const error = nav.attitudeError(s.q);
    log.push({
      t: s.t,
      position: Math.hypot(...nav.position.map((x, i) => x - s.p[i])),
      tilt: error.tilt,
      yaw: error.yaw,
      eph: nav.eph,
      wind: nav.wind.slice(),
    });
  }
  return { nav, log };
}
const deg = Math.PI / 180;

test("error-state EKF: attitude, heading and gyro biases converge in a banked circle with RTK", () => {
  const { nav, log } = fly(circle(240));
  const settled = log.filter((x) => x.t > 30);
  assert.ok(Math.max(...settled.map((x) => x.tilt)) < 1 * deg, "tilt");
  // Heading rests on the magnetometer: within its noise and disturbance.
  assert.ok(Math.max(...settled.map((x) => x.yaw)) < 3 * (MAG.noise + MAG.drift), "yaw");
  assert.ok(Math.max(...settled.map((x) => x.position)) < 0.1, "position");
  // Roll and pitch gyro biases are observable through the GNSS velocity.
  const truth = nav.sensorBias().gyro;
  for (const i of [0, 1])
    assert.ok(Math.abs(nav.gyroBias[i] - truth[i]) < 0.3 * Math.abs(truth[i]) + 2e-4, `gyro ${i}`);
});

test("drag fusion estimates the wind with GNSS and bounds dead reckoning without it, with a consistent EPH", () => {
  const wind = meanWind(3),
    samples = circle(300, wind),
    outage = [180, 240];
  const { log } = fly(samples, { outage });
  const before = log.find((x) => x.t >= 179);
  assert.ok(Math.hypot(before.wind[0] - wind[0], before.wind[1] - wind[1]) < 0.3, JSON.stringify(before.wind));
  const lost = log.filter((x) => x.t >= outage[0] && x.t < outage[1]),
    worst = Math.max(...lost.map((x) => x.position));
  // The filter knows how uncertain it has become.
  for (const x of lost) assert.ok(x.position < 3 * x.eph + 0.5, `${x.t}: ${x.position} vs ${x.eph}`);
  assert.ok(lost.at(-1).eph > 10 * before.eph);
  // Without drag fusion pure inertial navigation drifts much further.
  const inertial = fly(samples, { outage, withDrag: false }).log.filter(
    (x) => x.t >= outage[0] && x.t < outage[1],
  );
  const worstInertial = Math.max(...inertial.map((x) => x.position));
  assert.ok(worstInertial > 3 * worst, `${worstInertial} vs ${worst}`);
  // GNSS back: the estimate snaps back within seconds.
  assert.ok(log.find((x) => x.t >= outage[1] + 5).position < 0.3);
});
