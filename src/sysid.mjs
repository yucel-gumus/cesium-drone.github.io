import { rotate, conjugate, qmul } from "./flight-math.mjs";
import { SPIN } from "./quadrotor.mjs";
import { cellOpenCircuitVoltage } from "./battery.mjs";

// System identification from a flight log: the airframe and battery
// parameters the simulator needs, estimated from what an autopilot logs
// (velocity, attitude, gyro, accelerometer, ESC rotor speeds, pack voltage
// and current), so the model can be calibrated against real flights.
//
// A log is normalised to rows { t, v (ENU m/s), q (body FLU → ENU),
// w (body rad/s), f (specific force, body m/s²), rotors (rad/s, motor order
// front right, back right, back left, front left), voltage, current, soc,
// height (m above take-off) } — see fromSimulation(), fromCSV(), fromULog().
//
// Identifiable from such a log, with the take-off mass given:
//   thrust coefficient, body drag area, rotor drag, the mean wind
//     (Levenberg–Marquardt on the body-frame force balance
//      m f = k Σω² e_z − ½ρ CdA |v_a| v_a − c_lin (Σω/Σω_h) v_a,plane);
//   roll and pitch inertia, angular drag and the blade-flapping moment
//     (linear least squares on I ω̇ = τ(rotors) + flapping − D ω);
//   yaw: only the ratios reactionRatio / I_z and rotor inertia / I_z,
//   power: reactionRatio / efficiency and the avionics power (P = V I),
//     so reactionRatio stays nominal and I_z and the efficiency follow
//     from the ratios — the closed loop depends on nothing else;
//   pack: internal resistance and open-circuit offset (V = n OCV − R I).
// Not identifiable without further signals: the motor time constant
// (needs logged rotor speed commands) and the battery capacity (needs a
// state of charge independent of the voltage).

const G = 9.80665;

// ---------------------------------------------------------------- loaders

// Deterministic Gaussian noise (mulberry32 + Box–Muller).
function noiseSource(seed) {
  let a = seed >>> 0;
  const uniform = () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  return () => Math.sqrt(-2 * Math.log(1 - uniform())) * Math.cos(2 * Math.PI * uniform());
}
// Specific force in body axes from the kinematic ENU acceleration.
const specificForce = (q, a) => rotate(conjugate(q), [a[0], a[1], a[2] + G]);

// A simulated flight as a log, with sensor noise of a typical autopilot
// (GNSS velocity 5 cm/s, attitude 0.3°, gyro 0.003 rad/s, accelerometer
// 0.05 m/s², ESC speed 2 rad/s, voltage 10 mV, current 50 mA).
export function fromSimulation(sim, { noise = true, seed = 7 } = {}) {
  const n = noise ? noiseSource(seed) : () => 0,
    d = sim.data,
    S = sim.stride ?? 46,
    rows = [],
    z0 = d[3];
  for (let i = 0; i < sim.rows; i++) {
    const r = i * S,
      q = [d[r + 7], d[r + 8], d[r + 9], d[r + 10]],
      half = (0.3 * Math.PI) / 180 / 2,
      dq = [1, half * n(), half * n(), half * n()],
      qn = qmul(q, dq),
      qnorm = Math.hypot(...qn);
    const f = specificForce(q, [d[r + 14], d[r + 15], d[r + 16]]);
    rows.push({
      t: d[r],
      v: [d[r + 4] + 0.05 * n(), d[r + 5] + 0.05 * n(), d[r + 6] + 0.05 * n()],
      q: qn.map((x) => x / qnorm),
      w: [d[r + 11] + 0.003 * n(), d[r + 12] + 0.003 * n(), d[r + 13] + 0.003 * n()],
      f: f.map((x) => x + 0.05 * n()),
      rotors: [d[r + 17], d[r + 18], d[r + 19], d[r + 20]].map((x) => x + 2 * n()),
      voltage: d[r + 37] + 0.01 * n(),
      current: d[r + 38] + 0.05 * n(),
      soc: d[r + 36],
      height: d[r + 3] - z0,
    });
  }
  return rows;
}

// The simulator's CSV export (see exports.mjs). Only the measurable
// columns are read; the true wind, thrust and torque columns are not.
export function fromCSV(text) {
  const lines = text.trim().split(/\r?\n/),
    header = lines[0].split(","),
    col = (name) => {
      const i = header.indexOf(name);
      if (i < 0) throw new Error(`CSV sütunu eksik: ${name}`);
      return i;
    },
    c = Object.fromEntries(
      [
        "time_s", "up_m", "v_e_ms", "v_n_ms", "v_u_ms", "qw", "qx", "qy", "qz",
        "p_rads", "q_rads", "r_rads", "a_e_ms2", "a_n_ms2", "a_u_ms2",
        "m1_rads", "m2_rads", "m3_rads", "m4_rads", "battery_v", "battery_a", "battery_soc",
      ].map((k) => [k, col(k)]),
    );
  const rows = [];
  let z0 = null;
  for (const line of lines.slice(1)) {
    const x = line.split(",").map(Number);
    if (x.length < header.length || x.some((v) => !Number.isFinite(v))) continue;
    const q = [x[c.qw], x[c.qx], x[c.qy], x[c.qz]];
    z0 ??= x[c.up_m];
    rows.push({
      t: x[c.time_s],
      v: [x[c.v_e_ms], x[c.v_n_ms], x[c.v_u_ms]],
      q,
      w: [x[c.p_rads], x[c.q_rads], x[c.r_rads]],
      f: specificForce(q, [x[c.a_e_ms2], x[c.a_n_ms2], x[c.a_u_ms2]]),
      rotors: [x[c.m1_rads], x[c.m2_rads], x[c.m3_rads], x[c.m4_rads]],
      voltage: x[c.battery_v],
      current: x[c.battery_a],
      soc: x[c.battery_soc],
      height: x[c.up_m] - z0,
    });
  }
  if (rows.length < 100) throw new Error("CSV kaydında yeterli satır yok.");
  return rows;
}

// ------------------------------------------------------------------- ULog

// PX4 ULog (.ulg) reader after the format specification: definitions
// (F, I, M, P, A) and data (D) messages; nested types are flattened to
// pyulog-style names ("esc[0].esc_rpm"). Returns { startTime, info,
// params, topics: { "name" or "name.multiId": { [field]: Float64Array } } }.
const ULOG_MAGIC = [0x55, 0x4c, 0x6f, 0x67, 0x01, 0x12, 0x35];
const TYPE_SIZE = {
  int8_t: 1, uint8_t: 1, bool: 1, char: 1,
  int16_t: 2, uint16_t: 2,
  int32_t: 4, uint32_t: 4, float: 4,
  int64_t: 8, uint64_t: 8, double: 8,
};
export function parseULog(buffer) {
  const bytes = buffer instanceof Uint8Array ? buffer : new Uint8Array(buffer),
    view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength),
    text = new TextDecoder();
  if (bytes.length < 16 || ULOG_MAGIC.some((b, i) => bytes[i] !== b))
    throw new Error("ULog başlığı bulunamadı (PX4 .ulg dosyası değil).");
  const startTime = Number(view.getBigUint64(8, true));
  const formats = new Map(),
    subscriptions = new Map(),
    raw = new Map(),
    info = {},
    params = {};
  const scalar = (type, offset) => {
    switch (type) {
      case "int8_t": return view.getInt8(offset);
      case "uint8_t": case "char": return view.getUint8(offset);
      case "bool": return view.getUint8(offset) ? 1 : 0;
      case "int16_t": return view.getInt16(offset, true);
      case "uint16_t": return view.getUint16(offset, true);
      case "int32_t": return view.getInt32(offset, true);
      case "uint32_t": return view.getUint32(offset, true);
      case "float": return view.getFloat32(offset, true);
      case "double": return view.getFloat64(offset, true);
      case "int64_t": return Number(view.getBigInt64(offset, true));
      case "uint64_t": return Number(view.getBigUint64(offset, true));
      default: throw new Error(`ULog: bilinmeyen tür ${type}`);
    }
  };
  // "float[4]" → { type: "float", count: 4 }
  const parseField = (field) => {
    const [typeText, name] = field.trim().split(/\s+/),
      match = /^([\w]+)(?:\[(\d+)\])?$/.exec(typeText);
    return { type: match[1], count: match[2] ? Number(match[2]) : 0, name };
  };
  const sizeOf = (type) => {
    if (TYPE_SIZE[type]) return TYPE_SIZE[type];
    const f = formats.get(type);
    if (!f) throw new Error(`ULog: tanımsız iç içe tür ${type}`);
    if (f.size === undefined)
      f.size = f.fields.reduce((s, x) => s + sizeOf(x.type) * Math.max(1, x.count), 0);
    return f.size;
  };
  // Flattened (name, type, offset) leaves of a format.
  const leaves = (type, prefix = "", base = 0, out = []) => {
    let offset = base;
    for (const field of formats.get(type).fields) {
      const size = sizeOf(field.type),
        n = Math.max(1, field.count);
      for (let i = 0; i < n; i++) {
        const name = `${prefix}${field.name}${field.count ? `[${i}]` : ""}`;
        if (!field.name.startsWith("_padding")) {
          if (TYPE_SIZE[field.type]) out.push({ name, type: field.type, offset: offset + i * size });
          else leaves(field.type, `${name}.`, offset + i * size, out);
        }
      }
      offset += size * n;
    }
    return out;
  };
  const keyValue = (offset, end) => {
    const length = bytes[offset],
      key = text.decode(bytes.subarray(offset + 1, offset + 1 + length)),
      [type, name] = key.split(" "),
      at = offset + 1 + length,
      base = /^(\w+)(?:\[(\d+)\])?$/.exec(type);
    let value;
    if (base[1] === "char") value = text.decode(bytes.subarray(at, end));
    else value = scalar(base[1], at);
    return { name, value };
  };
  let offset = 16;
  while (offset + 3 <= bytes.length) {
    const size = view.getUint16(offset, true),
      type = String.fromCharCode(bytes[offset + 2]),
      body = offset + 3,
      end = body + size;
    if (end > bytes.length) break;
    switch (type) {
      case "F": {
        const definition = text.decode(bytes.subarray(body, end)),
          colon = definition.indexOf(":"),
          name = definition.slice(0, colon),
          fields = definition
            .slice(colon + 1)
            .split(";")
            .filter((x) => x.trim())
            .map(parseField);
        formats.set(name, { name, fields });
        break;
      }
      case "I": {
        const { name, value } = keyValue(body, end);
        info[name] = value;
        break;
      }
      case "P": {
        const { name, value } = keyValue(body, end);
        params[name] = value;
        break;
      }
      case "A": {
        const multi = bytes[body],
          id = view.getUint16(body + 1, true),
          name = text.decode(bytes.subarray(body + 3, end));
        subscriptions.set(id, { name, multi, leaves: null });
        break;
      }
      case "D": {
        const id = view.getUint16(body, true),
          sub = subscriptions.get(id);
        if (!sub || !formats.has(sub.name)) break;
        sub.leaves ??= leaves(sub.name);
        const key = sub.multi ? `${sub.name}.${sub.multi}` : sub.name;
        if (!raw.has(key)) raw.set(key, Object.fromEntries(sub.leaves.map((l) => [l.name, []])));
        const columns = raw.get(key),
          data = body + 2;
        for (const leaf of sub.leaves) {
          // Trailing padding may be cut from the logged struct.
          if (data + leaf.offset + TYPE_SIZE[leaf.type] > end) continue;
          columns[leaf.name].push(scalar(leaf.type, data + leaf.offset));
        }
        break;
      }
      default:
        break;
    }
    offset = end;
  }
  const topics = {};
  for (const [key, columns] of raw)
    topics[key] = Object.fromEntries(
      Object.entries(columns).map(([k, v]) => [k, Float64Array.from(v)]),
    );
  return { startTime, info, params, topics };
}

// Linear interpolation of a column at time t (seconds) from a topic's
// timestamp (µs). `cursor` keeps the search position for monotone t.
function sampler(topic, field) {
  const time = topic.timestamp,
    values = topic[field];
  if (!time || !values) return null;
  let i = 0;
  return (t) => {
    const us = t * 1e6;
    while (i < time.length - 2 && time[i + 1] < us) i++;
    const t0 = time[i],
      t1 = time[i + 1] ?? t0,
      u = t1 > t0 ? Math.min(1, Math.max(0, (us - t0) / (t1 - t0))) : 0;
    return values[i] + ((values[i + 1] ?? values[i]) - values[i]) * u;
  };
}
// PX4 frames (NED world, FRD body) to the simulator's (ENU, FLU), and the
// quad X motor order 1 FR, 2 BL, 3 FL, 4 BR to FR, BR, BL, FL.
const NED_TO_ENU = [0, Math.SQRT1_2, Math.SQRT1_2, 0],
  FRD_TO_FLU = [0, 1, 0, 0],
  PX4_MOTOR = [0, 3, 1, 2];
export function fromULog(buffer, { rate = 50 } = {}) {
  const log = parseULog(buffer),
    T = log.topics,
    topic = (name) => T[name] ?? T[`${name}.0`];
  const attitude = topic("vehicle_attitude"),
    position = topic("vehicle_local_position"),
    imu = topic("sensor_combined"),
    esc = topic("esc_status"),
    battery = topic("battery_status");
  for (const [name, t] of Object.entries({ vehicle_attitude: attitude, vehicle_local_position: position, sensor_combined: imu, esc_status: esc }))
    if (!t) throw new Error(`ULog kaydında ${name} konusu yok.`);
  const s = (t, f) => sampler(t, f),
    q = [0, 1, 2, 3].map((i) => s(attitude, `q[${i}]`)),
    v = ["vx", "vy", "vz"].map((f) => s(position, f)),
    z = s(position, "z"),
    gyro = [0, 1, 2].map((i) => s(imu, `gyro_rad[${i}]`)),
    accel = [0, 1, 2].map((i) => s(imu, `accelerometer_m_s2[${i}]`)),
    rpm = [0, 1, 2, 3].map((i) => s(esc, `esc[${i}].esc_rpm`)),
    volt = battery && (s(battery, "voltage_filtered_v") ?? s(battery, "voltage_v")),
    amp = battery && (s(battery, "current_filtered_a") ?? s(battery, "current_a")),
    remaining = battery && s(battery, "remaining");
  if (rpm.some((x) => !x)) throw new Error("ULog kaydında dört ESC devri (esc_rpm) yok.");
  const span = (t) => [t.timestamp[0] / 1e6, t.timestamp.at(-1) / 1e6],
    ranges = [attitude, position, imu, esc].map(span),
    start = Math.max(...ranges.map((r) => r[0])),
    stop = Math.min(...ranges.map((r) => r[1])),
    rows = [];
  let z0 = null;
  for (let t = start; t <= stop; t += 1 / rate) {
    const qNed = q.map((f) => f(t)),
      norm = Math.hypot(...qNed),
      qEnu = qmul(qmul(NED_TO_ENU, qNed.map((x) => x / norm)), FRD_TO_FLU),
      [vn, ve, vd] = v.map((f) => f(t)),
      flu = (x) => [x[0], -x[1], -x[2]],
      speeds = rpm.map((f) => (Math.abs(f(t)) * 2 * Math.PI) / 60);
    z0 ??= z(t);
    rows.push({
      t: t - start,
      v: [ve, vn, -vd],
      q: qEnu,
      w: flu(gyro.map((f) => f(t))),
      f: flu(accel.map((f) => f(t))),
      rotors: PX4_MOTOR.map((i) => speeds[i]),
      voltage: volt ? volt(t) : NaN,
      current: amp ? amp(t) : NaN,
      soc: remaining ? remaining(t) : NaN,
      height: -(z(t) - z0),
    });
  }
  if (rows.length < 100) throw new Error("ULog kaydı kalibrasyon için çok kısa.");
  return { rows, airDensity: topic("vehicle_air_data")?.rho?.[0] ?? null, params: log.params };
}

// ---------------------------------------------------------------- solvers

// Solves the symmetric positive system A x = b (Gaussian elimination with
// partial pivoting) and returns x and A⁻¹.
function solve(A, b) {
  const n = b.length,
    M = A.map((row, i) => [...row, b[i], ...Array.from({ length: n }, (_, j) => (i === j ? 1 : 0))]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    if (Math.abs(M[c][c]) < 1e-300) throw new Error("Tanılama denklemleri tekil: kayıtta yeterli uyarım yok.");
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = M[r][c] / M[c][c];
      for (let k = c; k < 2 * n + 1; k++) M[r][k] -= f * M[c][k];
    }
  }
  return {
    x: M.map((row, i) => row[n] / M[i][i]),
    inverse: M.map((row, i) => row.slice(n + 1).map((v) => v / M[i][i])),
  };
}
// Linear least squares y ≈ X β over rows; returns β, its formal standard
// errors, the residual RMS and R².
export function leastSquares(X, y) {
  const p = X[0].length,
    A = Array.from({ length: p }, () => new Array(p).fill(0)),
    b = new Array(p).fill(0);
  for (let i = 0; i < X.length; i++)
    for (let j = 0; j < p; j++) {
      b[j] += X[i][j] * y[i];
      for (let k = 0; k < p; k++) A[j][k] += X[i][j] * X[i][k];
    }
  const { x, inverse } = solve(A, b);
  let ss = 0,
    total = 0;
  const mean = y.reduce((s, v) => s + v, 0) / y.length;
  for (let i = 0; i < X.length; i++) {
    const e = y[i] - X[i].reduce((s, v, j) => s + v * x[j], 0);
    ss += e * e;
    total += (y[i] - mean) ** 2;
  }
  const variance = ss / Math.max(1, X.length - p);
  return {
    beta: x,
    sigma: inverse.map((row, i) => Math.sqrt(Math.max(0, row[i] * variance))),
    rms: Math.sqrt(ss / X.length),
    r2: total > 0 ? 1 - ss / total : 1,
  };
}
// Instrumental-variable least squares β = (Zᵀ X)⁻¹ Zᵀ y: unbiased when
// the regressors X carry measurement noise that the instruments Z (same
// size, correlated with the true regressors) do not share.
export function instrumental(X, Z, y) {
  const p = X[0].length,
    A = Array.from({ length: p }, () => new Array(p).fill(0)),
    b = new Array(p).fill(0),
    ZZ = Array.from({ length: p }, () => new Array(p).fill(0));
  for (let i = 0; i < X.length; i++)
    for (let j = 0; j < p; j++) {
      b[j] += Z[i][j] * y[i];
      for (let k = 0; k < p; k++) {
        A[j][k] += Z[i][j] * X[i][k];
        ZZ[j][k] += Z[i][j] * Z[i][k];
      }
    }
  const { x, inverse } = solve(A, b);
  let ss = 0,
    total = 0;
  const mean = y.reduce((s, v) => s + v, 0) / y.length;
  for (let i = 0; i < X.length; i++) {
    const e = y[i] - X[i].reduce((s, v, j) => s + v * x[j], 0);
    ss += e * e;
    total += (y[i] - mean) ** 2;
  }
  // Cov β = σ² (ZᵀX)⁻¹ ZᵀZ (XᵀZ)⁻¹.
  const variance = ss / Math.max(1, X.length - p),
    middle = inverse.map((row) => ZZ[0].map((_, k) => row.reduce((s, v, j) => s + v * ZZ[j][k], 0))),
    cov = middle.map((row) => inverse.map((col) => row.reduce((s, v, j) => s + v * col[j], 0)));
  return {
    beta: x,
    sigma: cov.map((row, i) => Math.sqrt(Math.max(0, row[i] * variance))),
    rms: Math.sqrt(ss / X.length),
    r2: total > 0 ? 1 - ss / total : 1,
  };
}
// Levenberg–Marquardt on residual(θ, push) (push(r) per residual), with a
// forward-difference Jacobian accumulated without storing it.
function levenberg(residual, theta0, { iterations = 30, steps } = {}) {
  let theta = theta0.slice(),
    lambda = 1e-3;
  const p = theta.length,
    cost = (t) => {
      let s = 0;
      residual(t, (r) => (s += r * r));
      return s;
    };
  let current = cost(theta),
    count = 0;
  residual(theta, () => count++);
  let A, g;
  for (let it = 0; it < iterations; it++) {
    const base = [];
    residual(theta, (r) => base.push(r));
    const J = Array.from({ length: p }, (_, j) => {
      const h = steps[j],
        shifted = theta.slice(),
        column = [];
      shifted[j] += h;
      let i = 0;
      residual(shifted, (r) => column.push((r - base[i++]) / h));
      return column;
    });
    A = Array.from({ length: p }, (_, j) => Array.from({ length: p }, (_, k) => {
      let s = 0;
      for (let i = 0; i < base.length; i++) s += J[j][i] * J[k][i];
      return s;
    }));
    g = Array.from({ length: p }, (_, j) => {
      let s = 0;
      for (let i = 0; i < base.length; i++) s += J[j][i] * base[i];
      return s;
    });
    let improved = false;
    for (let tries = 0; tries < 10 && !improved; tries++) {
      const damped = A.map((row, j) => row.map((v, k) => (j === k ? v * (1 + lambda) : v))),
        { x: delta } = solve(damped, g.map((v) => -v)),
        candidate = theta.map((v, j) => v + delta[j]),
        next = cost(candidate);
      if (next < current) {
        const relative = (current - next) / current;
        theta = candidate;
        current = next;
        lambda = Math.max(1e-9, lambda / 5);
        improved = true;
        if (relative < 1e-10) it = iterations;
      } else lambda *= 10;
    }
    if (!improved) break;
  }
  const variance = current / Math.max(1, count - p),
    { inverse } = solve(A, g);
  return {
    theta,
    sigma: inverse.map((row, i) => Math.sqrt(Math.max(0, row[i] * variance))),
    rms: Math.sqrt(current / count),
    count,
  };
}

// ------------------------------------------------------------- estimators

// Samples usable for the force balance: airborne clear of ground effect,
// rotors well above idle, not descending into the vortex ring band.
export function flightMask(rows, airframe) {
  const hover = 4 * Math.sqrt((airframe.mass * G) / (4 * airframe.thrustCoefficient * (airframe.airDensity / 1.225)));
  return rows.map((r) => {
    const sum = r.rotors[0] + r.rotors[1] + r.rotors[2] + r.rotors[3],
      speed = Math.hypot(r.v[0], r.v[1]);
    return (
      sum > 0.6 * hover &&
      (!Number.isFinite(r.height) || r.height > 5) &&
      (r.v[2] > -1 || speed > 3) &&
      Math.hypot(...r.w) < 1.5
    );
  });
}

// Thrust coefficient (at the log's air density, and referred to sea level),
// drag area, rotor drag at hover rotor speed and the mean horizontal wind.
export function identifyAerodynamics(rows, airframe, { airDensity = airframe.airDensity } = {}) {
  const m = airframe.mass,
    mask = flightMask(rows, airframe),
    use = rows.filter((_, i) => mask[i]),
    // Thinned to about 10 Hz: neighbouring samples carry little new
    // information and the fit cost scales with them.
    step = Math.max(1, Math.round(use.length / Math.max(1, (use.at(-1)?.t - use[0]?.t) * 10 || 1))),
    samples = use.filter((_, i) => i % step === 0).map((r) => ({
      ...r,
      squares: r.rotors.reduce((s, x) => s + x * x, 0),
      sum: r.rotors.reduce((s, x) => s + x, 0),
    }));
  if (samples.length < 50) throw new Error("Kayıtta yeterli uçuş örneği yok (yerden 5 m üstü, havada).");
  const k0 = (airframe.thrustCoefficient * airDensity) / 1.225;
  // θ = [k / k0, CdA, c_lin, wind east, wind north]
  const residual = (θ, push) => {
    const k = θ[0] * k0,
      hover = 4 * Math.sqrt((m * G) / (4 * k)),
      halfRhoCdA = 0.5 * airDensity * θ[1];
    for (const r of samples) {
      const va = [r.v[0] - θ[3], r.v[1] - θ[4], r.v[2]],
        vb = rotate(conjugate(r.q), va),
        speed = Math.hypot(...va),
        rotorDrag = (θ[2] * r.sum) / hover;
      push(m * r.f[0] - (-halfRhoCdA * speed * vb[0] - rotorDrag * vb[0]));
      push(m * r.f[1] - (-halfRhoCdA * speed * vb[1] - rotorDrag * vb[1]));
      push(m * r.f[2] - (k * r.squares - halfRhoCdA * speed * vb[2]));
    }
  };
  const fit = levenberg(residual, [1, airframe.dragArea, airframe.linearDrag, 0, 0], {
    steps: [1e-6, 1e-6, 1e-6, 1e-5, 1e-5],
  });
  const [ratio, dragArea, linearDrag, windEast, windNorth] = fit.theta;
  return {
    samples: samples.length,
    airDensity,
    // Referred to the 1.225 kg/m³ the airframe model scales from.
    thrustCoefficient: ratio * airframe.thrustCoefficient,
    dragArea,
    linearDrag,
    wind: [windEast, windNorth],
    sigma: {
      thrustCoefficient: fit.sigma[0] * airframe.thrustCoefficient,
      dragArea: fit.sigma[1],
      linearDrag: fit.sigma[2],
      wind: [fit.sigma[3], fit.sigma[4]],
    },
    // Force residual per axis, N.
    rms: fit.rms,
  };
}

// Angular dynamics in integral form over short windows of the log, so the
// gyro noise lands on the left-hand side (unbiased) instead of in a
// differentiated regressor (which biases inertia low):
//   Δω_x = (1/I_x) ∫τ_x + (c_f/I_x) ∫Σω v_y − (d_x/I_x) ∫ω_x
//   Δω_y = (1/I_y) ∫τ_y − (c_f/I_y) ∫Σω v_x − (d_y/I_y) ∫ω_y
//   Δω_z = (κ/I_z) ∫Σ s_i f_i + (J_r/I_z) Σ s_i Δω_i − (d_z/I_z) ∫ω_z
// with rotor thrusts f_i = k ω_i² from the identified k, the in-plane
// airspeed v in body axes from the identified wind, c_f the blade-flapping
// coefficient and J_r the rotor inertia (spin-up reaction).
export function identifyRotation(rows, airframe, aero, { window = 3 } = {}) {
  const k = (aero.thrustCoefficient * aero.airDensity) / 1.225,
    a = airframe.armLength / Math.SQRT2,
    mask = flightMask(rows, airframe);
  // Per-sample integrands.
  const terms = rows.map((r) => {
    const f = r.rotors.map((x) => k * x * x),
      vb = rotate(conjugate(r.q), [r.v[0] - aero.wind[0], r.v[1] - aero.wind[1], r.v[2]]),
      sum = r.rotors[0] + r.rotors[1] + r.rotors[2] + r.rotors[3];
    return {
      tx: a * (-f[0] - f[1] + f[2] + f[3]),
      ty: -a * (f[0] - f[1] - f[2] + f[3]),
      tz: SPIN.reduce((s, sp, i) => s + sp * f[i], 0),
      fx: sum * vb[1],
      fy: -sum * vb[0],
    };
  });
  const roll = [],
    rollZ = [],
    rollY = [],
    pitch = [],
    pitchZ = [],
    pitchY = [],
    yaw = [],
    yawZ = [],
    yawY = [];
  const wide = window + 1;
  for (let i = wide; i < rows.length - wide; i += window) {
    let ok = true;
    for (let j = i - wide; j <= i + wide && ok; j++) ok = mask[j];
    const t0 = rows[i - window].t,
      t1 = rows[i + window].t;
    if (!ok || !(t1 > t0) || t1 - t0 > 2 * window * 0.1) continue;
    // Trapezoidal integrals over the window.
    const integral = (get) => {
      let sum = 0;
      for (let j = i - window; j < i + window; j++)
        sum += 0.5 * (get(j) + get(j + 1)) * (rows[j + 1].t - rows[j].t);
      return sum;
    };
    const first = rows[i - window],
      last = rows[i + window],
      before = rows[i - wide],
      after = rows[i + wide],
      dw = [0, 1, 2].map((n) => last.w[n] - first.w[n]),
      // Instrument for the torque integrals: the rate change over a window
      // one sample wider (independent gyro noise, no rotor noise).
      dwWide = [0, 1, 2].map((n) => after.w[n] - before.w[n]),
      spin = (a, b) => SPIN.reduce((s, sp, n) => s + sp * (b.rotors[n] - a.rotors[n]), 0),
      flapX = integral((j) => terms[j].fx),
      flapY = integral((j) => terms[j].fy),
      dragX = -integral((j) => rows[j].w[0]),
      dragY = -integral((j) => rows[j].w[1]),
      dragZ = -integral((j) => rows[j].w[2]);
    roll.push([integral((j) => terms[j].tx), flapX, dragX]);
    rollZ.push([dwWide[0], flapX, dragX]);
    rollY.push(dw[0]);
    pitch.push([integral((j) => terms[j].ty), flapY, dragY]);
    pitchZ.push([dwWide[1], flapY, dragY]);
    pitchY.push(dw[1]);
    yaw.push([integral((j) => terms[j].tz), spin(first, last), dragZ]);
    yawZ.push([dwWide[2], spin(before, after), dragZ]);
    yawY.push(dw[2]);
  }
  if (roll.length < 100) throw new Error("Açısal tanılama için yeterli ardışık örnek yok.");
  const rx = instrumental(roll, rollZ, rollY),
    ry = instrumental(pitch, pitchZ, pitchY),
    rz = instrumental(yaw, yawZ, yawY),
    ix = 1 / rx.beta[0],
    iy = 1 / ry.beta[0],
    // Flapping from both axes, weighted by their precision.
    fx = rx.beta[1] * ix,
    fy = ry.beta[1] * iy,
    sx = rx.sigma[1] * ix,
    sy = ry.sigma[1] * iy,
    wx = 1 / (sx * sx || 1),
    wy = 1 / (sy * sy || 1);
  return {
    samples: roll.length,
    inertia: [ix, iy],
    angularDrag: [rx.beta[2] * ix, ry.beta[2] * iy],
    flappingMoment: (wx * fx + wy * fy) / (wx + wy),
    reactionPerInertia: rz.beta[0],
    rotorInertiaPerInertia: rz.beta[1],
    yawDragPerInertia: rz.beta[2],
    sigma: {
      // σ(1/β) ≈ σβ / β².
      inertia: [rx.sigma[0] * ix * ix, ry.sigma[0] * iy * iy],
      flappingMoment: 1 / Math.sqrt(wx + wy),
      reactionPerInertia: rz.sigma[0],
    },
    // Rate-change residuals over a window, rad/s.
    rms: { roll: rx.rms, pitch: ry.rms, yaw: rz.rms },
  };
}

// Electrical power P = V I against rotor speed cubed: P = (κ/η) k Σ|ω|³ +
// avionics; and the pack: V = n OCV(soc) + offset − R I. Rotor speed and
// current noise would bias both slopes low (errors in variables), so the
// regressors are instrumented by the mean of the neighbouring samples.
export function identifyPower(rows, airframe, aero, cells) {
  const k = (aero.thrustCoefficient * aero.airDensity) / 1.225,
    mask = flightMask(rows, airframe),
    cube = rows.map((r) => k * r.rotors.reduce((s, x) => s + Math.abs(x) ** 3, 0)),
    X = [],
    Z = [],
    y = [],
    VX = [],
    VZ = [],
    Vy = [];
  for (let i = 1; i < rows.length - 1; i++) {
    const r = rows[i],
      p = rows[i - 1],
      n = rows[i + 1];
    if (![r, p, n].every((x) => Number.isFinite(x.voltage) && Number.isFinite(x.current))) continue;
    if (mask[i - 1] && mask[i] && mask[i + 1]) {
      X.push([cube[i], 1]);
      Z.push([(cube[i - 1] + cube[i + 1]) / 2, 1]);
      y.push(r.voltage * r.current);
    }
    if (Number.isFinite(r.soc) && r.current > 0.5) {
      VX.push([1, -r.current]);
      VZ.push([1, -(p.current + n.current) / 2]);
      Vy.push(r.voltage - cells * cellOpenCircuitVoltage(r.soc));
    }
  }
  if (X.length < 50) return null;
  const power = instrumental(X, Z, y),
    pack = VX.length >= 50 ? instrumental(VX, VZ, Vy) : null;
  return {
    samples: X.length,
    reactionPerEfficiency: power.beta[0],
    avionicsPower: power.beta[1],
    internalResistance: pack ? pack.beta[1] : null,
    voltageOffset: pack ? pack.beta[0] : null,
    sigma: {
      reactionPerEfficiency: power.sigma[0],
      avionicsPower: power.sigma[1],
      internalResistance: pack ? pack.sigma[1] : null,
    },
    rms: { power: power.rms, voltage: pack?.rms ?? null },
  };
}

// One pass of all estimators.
function estimate(rows, airframe, battery, options) {
  const aero = identifyAerodynamics(rows, airframe, options),
    rotation = identifyRotation(rows, airframe, aero),
    power = identifyPower(rows, airframe, aero, battery.cells),
    kappa = airframe.reactionRatio,
    iz = kappa / rotation.reactionPerInertia;
  return {
    aero,
    rotation,
    power,
    values: {
      thrustCoefficient: aero.thrustCoefficient,
      dragArea: aero.dragArea,
      linearDrag: aero.linearDrag,
      inertiaX: rotation.inertia[0],
      inertiaY: rotation.inertia[1],
      inertiaZ: iz,
      flappingMoment: rotation.flappingMoment,
      ...(power && {
        propulsionEfficiency: kappa / power.reactionPerEfficiency,
        avionicsPower: power.avionicsPower,
        ...(Number.isFinite(power.internalResistance) && {
          internalResistance: power.internalResistance,
        }),
      }),
    },
  };
}
const PARAMETERS = [
  ["thrustCoefficient", "İtki katsayısı k", "N·s²", (c) => c.thrustCoefficient],
  ["dragArea", "Gövde sürükleme alanı CdA", "m²", (c) => c.dragArea],
  ["linearDrag", "Rotor sürüklemesi", "N·s/m", (c) => c.linearDrag],
  ["inertiaX", "Atalet Ix", "kg·m²", (c) => c.inertia[0]],
  ["inertiaY", "Atalet Iy", "kg·m²", (c) => c.inertia[1]],
  ["inertiaZ", "Atalet Iz (κ nominal)", "kg·m²", (c) => c.inertia[2]],
  ["flappingMoment", "Kanat çırpma momenti", "N·m·s²", (c) => c.flappingMoment],
  ["propulsionEfficiency", "İtki verimi η (κ nominal)", "", null, (b) => b.propulsionEfficiency],
  ["avionicsPower", "Aviyonik gücü", "W", null, (b) => b.avionicsPower],
  ["internalResistance", "Paket iç direnci", "Ω", null, (b) => b.internalResistance],
];

// Full identification with a split-half consistency check: the log is
// also identified on two interleaved halves (alternate 20 s blocks, so
// both span the whole flight), and a parameter is applied only when the
// halves agree within `tolerance` (relative) — one the flight hardly
// excites, or that noise and gusts drive, differs between them.
// The angular drag and the rotor inertia are reported but never applied:
// a mission flight excites them too little. Returns the estimates, the
// report rows and the calibrated airframe and battery.
export function identify(rows, airframe, battery, options = {}) {
  const tolerance = options.tolerance ?? 0.15,
    full = estimate(rows, airframe, battery, options),
    block = (r) => Math.floor((r.t - rows[0].t) / 20) % 2,
    halves = [0, 1].map((b) => rows.filter((r) => block(r) === b)).map((part) => {
      try {
        return estimate(part, airframe, battery, options).values;
      } catch {
        return null;
      }
    });
  const report = PARAMETERS.filter(([key]) => full.values[key] !== undefined).map(
    ([key, label, unit, fromAirframe, fromBattery]) => {
      const value = full.values[key],
        spread =
          halves[0] && halves[1] && halves[0][key] !== undefined && halves[1][key] !== undefined
            ? Math.abs(halves[0][key] - halves[1][key]) / Math.abs(value)
            : Infinity;
      return {
        key,
        label,
        unit,
        nominal: fromAirframe ? fromAirframe(airframe) : fromBattery(battery),
        estimate: value,
        spread,
        applied: Number.isFinite(value) && value > 0 && spread <= tolerance,
      };
    },
  );
  const use = (key, fallback) => {
    const row = report.find((r) => r.key === key);
    return row?.applied ? row.estimate : fallback;
  };
  const calibrated = {
    ...airframe,
    thrustCoefficient: use("thrustCoefficient", airframe.thrustCoefficient),
    dragArea: use("dragArea", airframe.dragArea),
    linearDrag: use("linearDrag", airframe.linearDrag),
    inertia: [
      use("inertiaX", airframe.inertia[0]),
      use("inertiaY", airframe.inertia[1]),
      use("inertiaZ", airframe.inertia[2]),
    ],
    flappingMoment: use("flappingMoment", airframe.flappingMoment),
  };
  const calibratedBattery = {
    ...battery,
    propulsionEfficiency: use("propulsionEfficiency", battery.propulsionEfficiency),
    avionicsPower: use("avionicsPower", battery.avionicsPower),
    internalResistance: use("internalResistance", battery.internalResistance),
  };
  return {
    aero: full.aero,
    rotation: full.rotation,
    power: full.power,
    wind: full.aero.wind,
    report,
    airframe: calibrated,
    battery: calibratedBattery,
  };
}
