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
// MEMS IMU accelerometer: white noise per control sample, turn-on bias left
// after calibration, and a slow bias random walk (m/s², m/s²/√s).
export const IMU = Object.freeze({
  accelNoise: 0.03,
  accelBias: 0.03,
  accelBiasWalk: 0.0005,
});
// Barometric altimeter: white noise plus a weather/temperature drift.
export const BARO = Object.freeze({
  noise: 0.08,
  drift: 0.4,
  driftTime: 120,
  rate: 40,
});

// Three decoupled ENU Kalman filters, one per axis, with state
// [position, velocity, accelerometer bias]. The accelerometer drives the
// prediction at the control rate; GNSS position and velocity (and the
// barometer on the vertical axis) correct it. Attitude comes from an ideal
// AHRS, so specific force is rotated into ENU without attitude error.
function makeAxisFilter(positionVariance, velocityVariance, biasVariance) {
  const x = [0, 0, 0];
  // Symmetric covariance stored as six numbers.
  let pp = positionVariance,
    pv = 0,
    pb = 0,
    vv = velocityVariance,
    vb = 0,
    bb = biasVariance;
  return {
    x,
    get positionVariance() {
      return pp;
    },
    predict(measuredAcceleration, dt, accelNoise, biasWalk) {
      const a = measuredAcceleration - x[2];
      x[0] += x[1] * dt + 0.5 * a * dt * dt;
      x[1] += a * dt;
      // P ← F P Fᵀ + Q with F = [[1, dt, −dt²/2], [0, 1, −dt], [0, 0, 1]].
      const h = -0.5 * dt * dt;
      const fpp = pp + dt * pv + h * pb,
        fpv = pv + dt * vv + h * vb,
        fpb = pb + dt * vb + h * bb,
        fvv = vv - dt * vb,
        fvb = vb - dt * bb;
      const npp = fpp + dt * fpv + h * fpb,
        npv = fpv - dt * fpb,
        nvv = fvv - dt * fvb;
      const q = accelNoise * accelNoise;
      pp = npp + (q * dt ** 4) / 4;
      pv = npv + (q * dt ** 3) / 2;
      pb = fpb;
      vv = nvv + q * dt * dt;
      vb = fvb;
      bb += biasWalk * biasWalk * dt;
    },
    // Scalar update of state component k (0 position, 1 velocity).
    update(k, measurement, variance) {
      const column =
        k === 0 ? [pp, pv, pb] : [pv, vv, vb];
      const s = column[k] + variance,
        gain = column.map((c) => c / s),
        innovation = measurement - x[k];
      for (let i = 0; i < 3; i++) x[i] += gain[i] * innovation;
      // P ← P − K Hᵀ P, i.e. subtract gain_i × column_j.
      pp -= gain[0] * column[0];
      pv -= gain[0] * column[1];
      pb -= gain[0] * column[2];
      vv -= gain[1] * column[1];
      vb -= gain[1] * column[2];
      bb -= gain[2] * column[2];
    },
  };
}

// `mode` is a GNSS_MODES key or "ideal" (estimate = truth, no sensors).
export function makeNavigation(mode = "rtk", seed = 1) {
  if (mode === "ideal")
    return {
      mode,
      ideal: true,
      gnss: null,
      position: [0, 0, 0],
      velocity: [0, 0, 0],
      eph: 0,
      epv: 0,
      step(truthPosition, truthVelocity) {
        for (let i = 0; i < 3; i++) {
          this.position[i] = truthPosition[i];
          this.velocity[i] = truthVelocity[i];
        }
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
  const accelBias = [0, 1, 2].map(() => IMU.accelBias * random.normal());
  const filters = [0, 1, 2].map((axis) =>
    makeAxisFilter(
      (axis < 2 ? gnss.horizontalNoise : gnss.verticalNoise) ** 2 + 1e-4,
      1e-4,
      IMU.accelBias ** 2,
    ),
  );
  let gnssClock = 0,
    baroClock = 0,
    initialized = false;
  const navigation = {
    mode,
    ideal: false,
    gnss,
    position: [0, 0, 0],
    velocity: [0, 0, 0],
    eph: 0,
    epv: 0,
    // One control-rate step: integrate the IMU, then apply any GNSS or
    // barometer sample that is due. Truth arrays are ENU.
    step(truthPosition, truthVelocity, truthAcceleration, dt) {
      // The filter starts from the first GNSS fix, as autopilots do.
      if (!initialized) {
        initialized = true;
        for (let i = 0; i < 3; i++) {
          filters[i].x[0] = truthPosition[i] + drift[i].value;
          filters[i].x[1] = truthVelocity[i];
        }
      }
      for (let i = 0; i < 3; i++) {
        accelBias[i] += IMU.accelBiasWalk * Math.sqrt(dt) * random.normal();
        const measured =
          truthAcceleration[i] + accelBias[i] + IMU.accelNoise * random.normal();
        filters[i].predict(measured, dt, IMU.accelNoise, IMU.accelBiasWalk);
      }
      for (const d of drift) d.step(dt);
      baroDrift.step(dt);
      gnssClock += dt;
      if (gnssClock >= 1 / gnss.rate - 1e-9) {
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
          filters[i].update(0, fix, sigma * sigma);
          filters[i].update(
            1,
            truthVelocity[i] + gnss.velocityNoise * random.normal(),
            gnss.velocityNoise ** 2,
          );
        }
      }
      baroClock += dt;
      if (baroClock >= 1 / BARO.rate - 1e-9) {
        baroClock = 0;
        const altitude =
          truthPosition[2] + baroDrift.value + BARO.noise * random.normal();
        filters[2].update(0, altitude, BARO.noise ** 2 + BARO.drift ** 2);
      }
      for (let i = 0; i < 3; i++) {
        this.position[i] = filters[i].x[0];
        this.velocity[i] = filters[i].x[1];
      }
      // Reported accuracy: the filter's own variance plus the correlated
      // receiver drift that averaging many fixes cannot remove.
      this.eph = Math.sqrt(
        filters[0].positionVariance +
          filters[1].positionVariance +
          2 * gnss.horizontalDrift ** 2,
      );
      this.epv = Math.sqrt(
        filters[2].positionVariance + gnss.verticalDrift ** 2,
      );
    },
  };
  return navigation;
}
