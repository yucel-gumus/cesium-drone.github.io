import { makeRandom, makeGaussMarkov } from "./random.mjs";

// Navigation sensor grades. GNSS errors are a fast white part plus a slowly
// wandering, exponentially correlated part (multipath, atmosphere), which is
// what makes real fixes drift rather than jitter. Values are typical
// datasheet figures, not a specific receiver.
export const GNSS_MODES = Object.freeze({
  rtk: Object.freeze({
    label: "RTK FIX",
    horizontalNoise: 0.008,
    verticalNoise: 0.015,
    horizontalDrift: 0.01,
    verticalDrift: 0.02,
    driftTime: 60,
    velocityNoise: 0.02,
    rate: 10,
    satellites: 24,
    hdop: 0.6,
  }),
  standard: Object.freeze({
    label: "3D FIX",
    horizontalNoise: 0.25,
    verticalNoise: 0.5,
    horizontalDrift: 0.7,
    verticalDrift: 1.2,
    driftTime: 40,
    velocityNoise: 0.08,
    rate: 10,
    satellites: 16,
    hdop: 0.9,
  }),
});
// MEMS IMU of a flight controller, per 120 Hz sample: white noise (with
// airframe vibration), turn-on bias left after the pre-arm calibration and a
// slow bias random walk. Accelerometer m/s², m/s²/√s; gyroscope rad/s,
// rad/s/√s (0.002 rad/s ≈ 0.11 °/s).
export const IMU = Object.freeze({
  accelNoise: 0.03,
  accelBias: 0.03,
  accelBiasWalk: 0.0005,
  gyroNoise: 0.002,
  gyroBias: 0.001,
  gyroBiasWalk: 2e-5,
});
// Barometric altimeter: white noise plus a weather/temperature drift.
export const BARO = Object.freeze({
  noise: 0.08,
  drift: 0.4,
  driftTime: 120,
  rate: 40,
});
// Magnetometer heading: white noise plus a slowly varying disturbance
// (motor currents, local field), radians.
export const MAG = Object.freeze({
  noise: 0.02,
  drift: 0.03,
  driftTime: 300,
  rate: 20,
});

const GRAVITY = 9.80665;
// Random walk of the horizontal mean-wind states, m/s/√s; turbulence
// (GUST, m/s) is treated as drag-fusion measurement noise.
const WIND_WALK = 0.03,
  GUST = 0.3;
// 17-state error vector: δp 0–2, δv 3–5, δθ 6–8 (body frame), δb_a 9–11,
// δb_g 12–14, δwind 15–16 (east, north), as PX4's EKF2 carries wind.
const N = 17;

function rotation(q, out = new Float64Array(9)) {
  const [w, x, y, z] = q;
  out[0] = 1 - 2 * (y * y + z * z);
  out[1] = 2 * (x * y - w * z);
  out[2] = 2 * (x * z + w * y);
  out[3] = 2 * (x * y + w * z);
  out[4] = 1 - 2 * (x * x + z * z);
  out[5] = 2 * (y * z - w * x);
  out[6] = 2 * (x * z - w * y);
  out[7] = 2 * (y * z + w * x);
  out[8] = 1 - 2 * (x * x + y * y);
  return out;
}
// q ← q ⊗ [cos(|θ|/2), sin(|θ|/2) θ/|θ|], normalised.
function rotateBy(q, tx, ty, tz) {
  const angle = Math.hypot(tx, ty, tz),
    half = angle / 2,
    s = angle > 1e-12 ? Math.sin(half) / angle : 0.5,
    dw = Math.cos(half),
    dx = tx * s,
    dy = ty * s,
    dz = tz * s;
  const [w, x, y, z] = q;
  q[0] = w * dw - x * dx - y * dy - z * dz;
  q[1] = w * dx + x * dw + y * dz - z * dy;
  q[2] = w * dy - x * dz + y * dw + z * dx;
  q[3] = w * dz + x * dy - y * dx + z * dw;
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  for (let i = 0; i < 4; i++) q[i] /= n;
}
const yawOf = (q) =>
  Math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
const wrap = (x) => Math.atan2(Math.sin(x), Math.cos(x));

// `mode` is a GNSS_MODES key or "ideal" (estimate = truth, no sensors).
// `options.drag` ({ mass, area: ½ ρ CdA, linear }) enables drag fusion:
// with GNSS it estimates the wind states, without GNSS it observes the
// velocity relative to them, as ArduPilot's EKF3 dead reckons (step()).
export function makeNavigation(mode = "rtk", seed = 1, options = {}) {
  if (mode === "ideal")
    return {
      mode,
      ideal: true,
      gnss: null,
      gnssAvailable: true,
      position: [0, 0, 0],
      velocity: [0, 0, 0],
      attitude: [1, 0, 0, 0],
      rates: [0, 0, 0],
      wind: [0, 0],
      eph: 0,
      epv: 0,
      step(truthPosition, truthVelocity, truthAcceleration, dt, truthAttitude, truthRates) {
        for (let i = 0; i < 3; i++) {
          this.position[i] = truthPosition[i];
          this.velocity[i] = truthVelocity[i];
          if (truthRates) this.rates[i] = truthRates[i];
        }
        if (truthAttitude) for (let i = 0; i < 4; i++) this.attitude[i] = truthAttitude[i];
      },
    };
  const gnss = GNSS_MODES[mode];
  if (!gnss) throw new Error(`Bilinmeyen GNSS modu: ${mode}`);
  // A stream of its own, so sensor noise never shifts the turbulence draws.
  const random = makeRandom((seed ^ 0x9e3779b9) >>> 0);
  const drift = [
    makeGaussMarkov(random, gnss.horizontalDrift, gnss.driftTime),
    makeGaussMarkov(random, gnss.horizontalDrift, gnss.driftTime),
    makeGaussMarkov(random, gnss.verticalDrift, gnss.driftTime),
  ];
  const baroDrift = makeGaussMarkov(random, BARO.drift, BARO.driftTime);
  const magDrift = makeGaussMarkov(random, MAG.drift, MAG.driftTime);
  const accelBias = [0, 1, 2].map(() => IMU.accelBias * random.normal());
  const gyroBias = [0, 1, 2].map(() => IMU.gyroBias * random.normal());
  const drag = options.drag ?? null;
  // Nominal state.
  const p = [0, 0, 0],
    v = [0, 0, 0],
    q = [1, 0, 0, 0],
    ba = [0, 0, 0],
    bg = [0, 0, 0],
    wind = [0, 0];
  // Error covariance (row-major 15 × 15), scratch buffers.
  const P = new Float64Array(N * N),
    A = new Float64Array(N * N),
    AP = new Float64Array(N * N),
    PHt = new Float64Array(N),
    dx = new Float64Array(N),
    R = new Float64Array(9);
  const initial = [
    gnss.horizontalNoise ** 2 + 1e-4,
    gnss.horizontalNoise ** 2 + 1e-4,
    gnss.verticalNoise ** 2 + 1e-4,
    1e-4,
    1e-4,
    1e-4,
    // Levelled on the pad from gravity; heading from the magnetometer.
    0.005 ** 2,
    0.005 ** 2,
    (MAG.noise + MAG.drift) ** 2,
    IMU.accelBias ** 2,
    IMU.accelBias ** 2,
    IMU.accelBias ** 2,
    IMU.gyroBias ** 2,
    IMU.gyroBias ** 2,
    IMU.gyroBias ** 2,
    // Wind unknown at take-off.
    3 ** 2,
    3 ** 2,
  ];
  initial.forEach((x, i) => (P[i * N + i] = x));
  let gnssClock = 0,
    baroClock = 0,
    magClock = 0,
    dragClock = 0,
    initialized = false;
  // Scalar measurement: H has entries `h` at columns `cols`.
  function update(cols, h, residual, variance) {
    let s = variance;
    for (let i = 0; i < N; i++) {
      let sum = 0;
      for (let k = 0; k < cols.length; k++) sum += P[i * N + cols[k]] * h[k];
      PHt[i] = sum;
    }
    for (let k = 0; k < cols.length; k++) s += h[k] * PHt[cols[k]];
    let predicted = 0;
    for (let k = 0; k < cols.length; k++) predicted += h[k] * dx[cols[k]];
    const innovation = residual - predicted;
    for (let i = 0; i < N; i++) dx[i] += (PHt[i] / s) * innovation;
    for (let i = 0; i < N; i++) {
      const ki = PHt[i] / s;
      for (let j = 0; j < N; j++) P[i * N + j] -= ki * PHt[j];
    }
  }
  // Move the estimated error into the nominal state and reset it.
  function inject() {
    for (let i = 0; i < 3; i++) {
      p[i] += dx[i];
      v[i] += dx[3 + i];
      ba[i] += dx[9 + i];
      bg[i] += dx[12 + i];
    }
    wind[0] += dx[15];
    wind[1] += dx[16];
    rotateBy(q, dx[6], dx[7], dx[8]);
    dx.fill(0);
    // Keep the covariance symmetric against round-off.
    for (let i = 0; i < N; i++)
      for (let j = i + 1; j < N; j++) {
        const m = (P[i * N + j] + P[j * N + i]) / 2;
        P[i * N + j] = m;
        P[j * N + i] = m;
      }
  }
  const navigation = {
    mode,
    ideal: false,
    gnss,
    // Cleared during an injected outage: the filter then dead-reckons on
    // the IMU with barometric height (and drag fusion when configured),
    // and its covariance grows.
    gnssAvailable: true,
    position: p,
    velocity: v,
    attitude: q,
    rates: [0, 0, 0],
    accelBias: ba,
    gyroBias: bg,
    wind,
    eph: 0,
    epv: 0,
    // One control-rate step: propagate on the IMU, then fuse what is due.
    // Truth arrays are ENU (acceleration: kinematic, over the step), the
    // attitude body → ENU and the rates in body axes. `aiding`
    // ({ rotorRatio }) is given while airborne and enables drag fusion.
    step(
      truthPosition,
      truthVelocity,
      truthAcceleration,
      dt,
      truthAttitude = [1, 0, 0, 0],
      truthRates = [0, 0, 0],
      aiding = null,
    ) {
      // The filter starts from the first GNSS fix and a levelled attitude
      // with the magnetometer's heading, as autopilots initialise.
      if (!initialized) {
        initialized = true;
        for (let i = 0; i < 3; i++) {
          p[i] = truthPosition[i] + drift[i].value;
          v[i] = truthVelocity[i];
        }
        for (let i = 0; i < 4; i++) q[i] = truthAttitude[i];
        rotateBy(q, 0, 0, magDrift.value);
      }
      // Sensors: bias walks, then body-frame specific force and rates.
      const Rt = rotation(truthAttitude);
      const f = [0, 1, 2].map(
        (i) =>
          Rt[i] * truthAcceleration[0] +
          Rt[3 + i] * truthAcceleration[1] +
          Rt[6 + i] * (truthAcceleration[2] + GRAVITY),
      );
      for (let i = 0; i < 3; i++) {
        accelBias[i] += IMU.accelBiasWalk * Math.sqrt(dt) * random.normal();
        gyroBias[i] += IMU.gyroBiasWalk * Math.sqrt(dt) * random.normal();
        f[i] += accelBias[i] + IMU.accelNoise * random.normal();
      }
      const w = [0, 1, 2].map(
        (i) => truthRates[i] + gyroBias[i] + IMU.gyroNoise * random.normal(),
      );
      // Propagate the nominal state.
      const fx = f[0] - ba[0],
        fy = f[1] - ba[1],
        fz = f[2] - ba[2],
        wx = w[0] - bg[0],
        wy = w[1] - bg[1],
        wz = w[2] - bg[2];
      rotation(q, R);
      const a = [
        R[0] * fx + R[1] * fy + R[2] * fz,
        R[3] * fx + R[4] * fy + R[5] * fz,
        R[6] * fx + R[7] * fy + R[8] * fz - GRAVITY,
      ];
      for (let i = 0; i < 3; i++) {
        p[i] += v[i] * dt + 0.5 * a[i] * dt * dt;
        v[i] += a[i] * dt;
      }
      rotateBy(q, wx * dt, wy * dt, wz * dt);
      this.rates[0] = wx;
      this.rates[1] = wy;
      this.rates[2] = wz;
      // Covariance: P ← (I + A) P (I + A)ᵀ + Q with A = F dt, F sparse:
      // δṗ = δv, δv̇ = −R [f]× δθ − R δb_a, δθ̇ = −[ω]× δθ − δb_g.
      A.fill(0);
      for (let i = 0; i < 3; i++) A[i * N + 3 + i] = dt;
      // −R [f]×, with [f]× = [[0, −fz, fy], [fz, 0, −fx], [−fy, fx, 0]].
      for (let i = 0; i < 3; i++) {
        const r0 = R[i * 3],
          r1 = R[i * 3 + 1],
          r2 = R[i * 3 + 2];
        A[(3 + i) * N + 6] = -(r1 * fz - r2 * fy) * dt;
        A[(3 + i) * N + 7] = -(-r0 * fz + r2 * fx) * dt;
        A[(3 + i) * N + 8] = -(r0 * fy - r1 * fx) * dt;
        A[(3 + i) * N + 9] = -r0 * dt;
        A[(3 + i) * N + 10] = -r1 * dt;
        A[(3 + i) * N + 11] = -r2 * dt;
      }
      // −[ω]×
      A[6 * N + 7] = wz * dt;
      A[6 * N + 8] = -wy * dt;
      A[7 * N + 6] = -wz * dt;
      A[7 * N + 8] = wx * dt;
      A[8 * N + 6] = wy * dt;
      A[8 * N + 7] = -wx * dt;
      for (let i = 0; i < 3; i++) A[(6 + i) * N + 12 + i] = -dt;
      // AP (rows of A have at most six entries).
      for (let i = 0; i < N; i++)
        for (let j = 0; j < N; j++) {
          let sum = 0;
          for (let k = 0; k < N; k++) {
            const aik = A[i * N + k];
            if (aik !== 0) sum += aik * P[k * N + j];
          }
          AP[i * N + j] = sum;
        }
      // P + AP + (AP)ᵀ + A P Aᵀ, the last as (AP) Aᵀ.
      for (let i = 0; i < N; i++)
        for (let j = i; j < N; j++) {
          let apa = 0;
          for (let k = 0; k < N; k++) {
            const ajk = A[j * N + k];
            if (ajk !== 0) apa += AP[i * N + k] * ajk;
          }
          const value = P[i * N + j] + AP[i * N + j] + AP[j * N + i] + apa;
          P[i * N + j] = value;
          P[j * N + i] = value;
        }
      const qa = (IMU.accelNoise * dt) ** 2,
        qg = (IMU.gyroNoise * dt) ** 2,
        qba = IMU.accelBiasWalk ** 2 * dt,
        qbg = IMU.gyroBiasWalk ** 2 * dt,
        qw = WIND_WALK ** 2 * dt;
      for (let i = 0; i < 3; i++) {
        P[(3 + i) * N + 3 + i] += qa;
        P[(6 + i) * N + 6 + i] += qg;
        P[(9 + i) * N + 9 + i] += qba;
        P[(12 + i) * N + 12 + i] += qbg;
      }
      P[15 * N + 15] += qw;
      P[16 * N + 16] += qw;
      for (const d of drift) d.step(dt);
      baroDrift.step(dt);
      magDrift.step(dt);
      // GNSS position and velocity.
      gnssClock += dt;
      if (gnssClock >= 1 / gnss.rate - 1e-9 && this.gnssAvailable) {
        gnssClock = 0;
        for (let i = 0; i < 3; i++) {
          const noise = i < 2 ? gnss.horizontalNoise : gnss.verticalNoise,
            fix = truthPosition[i] + drift[i].value + noise * random.normal();
          // The filter cannot tell drift from position; it is told the
          // receiver's total uncertainty.
          const sigma = Math.hypot(
            noise,
            i < 2 ? gnss.horizontalDrift : gnss.verticalDrift,
          );
          update([i], [1], fix - p[i], sigma * sigma);
          update(
            [3 + i],
            [1],
            truthVelocity[i] + gnss.velocityNoise * random.normal() - v[i],
            gnss.velocityNoise ** 2,
          );
        }
        inject();
      }
      // Barometric height.
      baroClock += dt;
      if (baroClock >= 1 / BARO.rate - 1e-9) {
        baroClock = 0;
        const altitude =
          truthPosition[2] + baroDrift.value + BARO.noise * random.normal();
        update([2], [1], altitude - p[2], BARO.noise ** 2 + BARO.drift ** 2);
        inject();
      }
      // Magnetometer heading: yaw error ≈ (R δθ)_z, the third row of R.
      magClock += dt;
      if (magClock >= 1 / MAG.rate - 1e-9) {
        magClock = 0;
        rotation(q, R);
        const heading =
          yawOf(truthAttitude) + magDrift.value + MAG.noise * random.normal();
        update(
          [6, 7, 8],
          [R[6], R[7], R[8]],
          wrap(heading - yawOf(q)),
          MAG.noise ** 2 + MAG.drift ** 2,
        );
        inject();
      }
      // Drag fusion: the accelerometer's body x/y axes feel only
      // aerodynamic force, the drag of the air-relative velocity v − wind,
      // linearised with k = (a |v_a| + c n) / m. With GNSS fixing the
      // velocity it estimates the wind; without GNSS it holds the velocity
      // relative to the estimated wind, whose uncertainty keeps growing.
      dragClock += dt;
      if (drag && aiding && dragClock >= 1 / 30 - 1e-9) {
        dragClock = 0;
        rotation(q, R);
        const air = [v[0] - wind[0], v[1] - wind[1], v[2]],
          speed = Math.hypot(...air),
          k = (drag.area * speed + drag.linear * aiding.rotorRatio) / drag.mass,
          // Air velocity in body axes, Rᵀ v_a.
          body = [0, 1, 2].map(
            (i) => R[i] * air[0] + R[3 + i] * air[1] + R[6 + i] * air[2],
          );
        for (let axis = 0; axis < 2; axis++) {
          // ∂f/∂δv = −k Rᵀ row, ∂f/∂δθ = −k [v_b]× row, ∂f/∂δb_a = 1,
          // ∂f/∂δwind = +k Rᵀ row (east, north).
          const skew =
            axis === 0 ? [0, -body[2], body[1]] : [body[2], 0, -body[0]];
          update(
            [3, 4, 5, 6, 7, 8, 9 + axis, 15, 16],
            [
              -k * R[axis],
              -k * R[3 + axis],
              -k * R[6 + axis],
              -k * skew[0],
              -k * skew[1],
              -k * skew[2],
              1,
              k * R[axis],
              k * R[3 + axis],
            ],
            f[axis] - (-k * body[axis] + ba[axis]),
            // Sensor noise, a 15 % drag-model error and gusts.
            IMU.accelNoise ** 2 +
              (0.15 * k * speed) ** 2 +
              (k * GUST) ** 2 +
              0.02 ** 2,
          );
        }
        inject();
      }
      // Reported accuracy: the filter's own variance plus the correlated
      // receiver drift that averaging many fixes cannot remove.
      this.eph = Math.sqrt(
        P[0] + P[N + 1] + 2 * gnss.horizontalDrift ** 2,
      );
      this.epv = Math.sqrt(P[2 * N + 2] + gnss.verticalDrift ** 2);
    },
    // The simulated sensors' true biases (diagnostics and tests only; the
    // filter never reads them).
    sensorBias() {
      return { accel: accelBias.slice(), gyro: gyroBias.slice() };
    },
    // Attitude error of the estimate against truth, radians: tilt and yaw.
    attitudeError(truthAttitude) {
      const [w, x, y, z] = truthAttitude,
        [ew, ex, ey, ez] = q;
      // Error quaternion q̂⁻¹ ⊗ q_t.
      const rw = ew * w + ex * x + ey * y + ez * z,
        rx = ew * x - ex * w - ey * z + ez * y,
        ry = ew * y + ex * z - ey * w - ez * x,
        rz = ew * z - ex * y + ey * x - ez * w;
      return {
        tilt: 2 * Math.asin(Math.min(1, Math.hypot(rx, ry))),
        yaw: Math.abs(wrap(yawOf(truthAttitude) - yawOf(q))),
        total: 2 * Math.acos(Math.min(1, Math.abs(rw))),
      };
    },
  };
  return navigation;
}
