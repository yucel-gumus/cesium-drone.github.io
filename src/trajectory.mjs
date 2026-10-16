import {
  clamp,
  norm,
  sub,
  add,
  mul,
  unit,
  quintic,
  polynomial,
} from "./flight-math.mjs";

function bezier(points, u) {
  let p = points.map((x) => x.slice());
  while (p.length > 1)
    p = p.slice(0, -1).map((x, i) => add(mul(x, 1 - u), mul(p[i + 1], u)));
  return p[0];
}
function geometryAt(piece, u) {
  const p = bezier(piece, u);
  const first = piece
    .slice(0, -1)
    .map((x, i) => mul(sub(piece[i + 1], x), piece.length - 1));
  const second = first
    .slice(0, -1)
    .map((x, i) => mul(sub(first[i + 1], x), first.length - 1));
  const v = bezier(first, u),
    a = second.length ? bezier(second, u) : [0, 0];
  const speed = norm(v);
  return {
    p,
    tangent: unit(v),
    curvature: (v[0] * a[1] - v[1] * a[0]) / Math.max(1e-12, speed ** 3),
  };
}

export function roundedPath(points, trim = 45, spacing = 2) {
  const pieces = [];
  let last = points[0];
  const line = (a, b) => {
    if (norm(sub(b, a)) > 0.01) pieces.push([a, b]);
  };
  for (let i = 1; i < points.length - 1; i++) {
    const incoming = sub(points[i], points[i - 1]),
      outgoing = sub(points[i + 1], points[i]);
    const d = Math.min(trim, norm(incoming) * 0.24, norm(outgoing) * 0.24);
    const u = unit(incoming),
      v = unit(outgoing);
    if (norm(add(u, v)) < 0.1)
      throw new Error("180° dönüş için ara rota noktası gerekli.");
    const a = sub(points[i], mul(u, d)),
      b = add(points[i], mul(v, d));
    line(last, a);
    // Collinear first/last three controls give zero endpoint curvature: C2
    // geometric joins to straight legs, no instantaneous lateral acceleration.
    pieces.push([
      a,
      add(a, mul(u, 0.4 * d)),
      add(a, mul(u, 0.8 * d)),
      sub(b, mul(v, 0.8 * d)),
      sub(b, mul(v, 0.4 * d)),
      b,
    ]);
    last = b;
  }
  line(last, points.at(-1));
  const dense = [];
  let distance = 0,
    previous;
  for (const piece of pieces) {
    const length = piece
      .slice(1)
      .reduce((s, p, i) => s + norm(sub(p, piece[i])), 0);
    const count = Math.max(8, Math.ceil(length / 0.4));
    for (let i = 0; i <= count; i++) {
      if (i === 0 && dense.length) continue;
      const x = geometryAt(piece, i / count);
      if (previous) distance += norm(sub(x.p, previous));
      dense.push({ ...x, s: distance });
      previous = x.p;
    }
  }
  const table = [];
  let cursor = 0;
  const count = Math.ceil(distance / spacing);
  for (let i = 0; i <= count; i++) {
    const s = (distance * i) / count;
    while (cursor < dense.length - 2 && dense[cursor + 1].s < s) cursor++;
    const a = dense[cursor],
      b = dense[cursor + 1],
      f = (s - a.s) / (b.s - a.s);
    table.push({
      s,
      p: add(mul(a.p, 1 - f), mul(b.p, f)),
      tangent: unit(add(mul(a.tangent, 1 - f), mul(b.tangent, f))),
      curvature: a.curvature + (b.curvature - a.curvature) * f,
    });
  }
  return { pieces, table, length: distance };
}

export function makeReference(path, cruiseHeight, settings) {
  const table = path.table,
    n = table.length;
  let speeds = table.map((k) =>
    Math.min(
      settings.cruiseSpeed,
      Math.sqrt(
        settings.lateralAcceleration / Math.max(Math.abs(k.curvature), 1e-7),
      ),
    ),
  );
  // Anticipate bends, then smooth only below the original curvature speed cap.
  const eroded = speeds.map((v, i) =>
    Math.min(...speeds.slice(Math.max(0, i - 7), Math.min(n, i + 8))),
  );
  speeds = eroded.map((v, i) => {
    let sum = 0,
      w = 0;
    for (let j = -7; j <= 7; j++) {
      const k = clamp(i + j, 0, n - 1),
        weight = Math.exp((-j * j) / 12);
      sum += eroded[k] * weight;
      w += weight;
    }
    return Math.min(v, sum / w);
  });
  speeds[0] = speeds[n - 1] = 0;
  for (let i = 1; i < n; i++)
    speeds[i] = Math.min(
      speeds[i],
      Math.sqrt(speeds[i - 1] ** 2 + 2 * 0.65 * (table[i].s - table[i - 1].s)),
    );
  for (let i = n - 2; i >= 0; i--)
    speeds[i] = Math.min(
      speeds[i],
      Math.sqrt(speeds[i + 1] ** 2 + 2 * 0.65 * (table[i + 1].s - table[i].s)),
    );
  function buildHorizontal() {
    const accelerations = table.map((k, i) => {
      const a = Math.max(0, i - 1),
        b = Math.min(n - 1, i + 1);
      const along =
        (speeds[b] ** 2 - speeds[a] ** 2) / (2 * (table[b].s - table[a].s));
      return [
        along * k.tangent[0] - k.curvature * speeds[i] ** 2 * k.tangent[1],
        along * k.tangent[1] + k.curvature * speeds[i] ** 2 * k.tangent[0],
      ];
    });
    accelerations[0] = accelerations[n - 1] = [0, 0];
    let time = 0;
    const horizontal = [];
    for (let i = 1; i < n; i++) {
      const a = table[i - 1],
        b = table[i],
        T = (2 * (b.s - a.s)) / (speeds[i - 1] + speeds[i]);
      horizontal.push({
        startTime: time,
        duration: T,
        coefficients: [0, 1].map((axis) =>
          quintic(
            a.p[axis],
            speeds[i - 1] * a.tangent[axis],
            accelerations[i - 1][axis],
            b.p[axis],
            speeds[i] * b.tangent[axis],
            accelerations[i][axis],
            T,
          ),
        ),
      });
      time += T;
    }
    return { horizontal, time };
  }
  // Reduce speed locally around excessive jerk, preserving useful cruise
  // speed on straights instead of slowing the entire mission for one bend.
  for (let pass = 0; pass < 5; pass++) {
    const { horizontal } = buildHorizontal();
    const factors = Array(n).fill(1);
    for (let i = 0; i < horizontal.length; i++) {
      const seg = horizontal[i];
      let ratio = 1;
      for (let k = 0; k <= 12; k++) {
        const a = seg.coefficients.map((c) =>
          polynomial(c, k / 12, seg.duration),
        );
        ratio = Math.max(
          ratio,
          Math.sqrt(Math.hypot(a[0].a, a[1].a) / settings.acceleration),
          Math.cbrt(Math.hypot(a[0].j, a[1].j) / settings.jerk),
        );
      }
      if (ratio < 1.04) continue;
      for (let j = Math.max(1, i - 18); j < Math.min(n - 1, i + 20); j++) {
        const weight =
          (1 + Math.cos(Math.PI * Math.min(1, Math.abs(j - i - 0.5) / 20))) / 2;
        factors[j] = Math.min(factors[j], 1 / (1 + (ratio - 1) * weight));
      }
    }
    if (Math.min(...factors) > 0.98) break;
    speeds = speeds.map((v, i) => v * factors[i]);
  }
  const { horizontal, time } = buildHorizontal();
  // Retiming preserves geometry, C2 joins and all derivatives. Evaluate every
  // quintic densely and add headroom for the tracking controller.
  let scale = 1;
  for (const seg of horizontal)
    for (let k = 0; k <= 20; k++) {
      const a = seg.coefficients.map((c) =>
        polynomial(c, k / 20, seg.duration),
      );
      scale = Math.max(
        scale,
        Math.hypot(a[0].v, a[1].v) / settings.cruiseSpeed,
        Math.sqrt(Math.hypot(a[0].a, a[1].a) / settings.acceleration),
        Math.cbrt(Math.hypot(a[0].j, a[1].j) / settings.jerk),
      );
    }
  scale *= 1.03;
  horizontal.forEach((seg) => {
    seg.startTime *= scale;
    seg.duration *= scale;
  });
  const horizontalDuration = time * scale;
  const initialYaw = Math.atan2(table[0].tangent[1], table[0].tangent[0]);
  const finalYaw = Math.atan2(table.at(-1).tangent[1], table.at(-1).tangent[0]);
  const verticalDuration = (dz, speed) =>
    Math.max(
      3,
      (1.875 * Math.abs(dz)) / speed,
      Math.sqrt((5.774 * Math.abs(dz)) / settings.acceleration),
      Math.cbrt((60 * Math.abs(dz)) / settings.jerk),
    );
  const phases = [];
  let clock = 0;
  const append = (phase, duration, z0, z1, yaw) => {
    phases.push({
      phase,
      startTime: clock,
      endTime: clock + duration,
      duration,
      z: quintic(z0, 0, 0, z1, 0, 0, duration),
      yaw,
    });
    clock += duration;
  };
  append("arming", 2, 0, 0, initialYaw);
  append(
    "takeoff",
    verticalDuration(cruiseHeight, settings.climbSpeed),
    0,
    cruiseHeight,
    initialYaw,
  );
  const surveyStart = clock;
  phases.push({
    phase: "survey",
    startTime: clock,
    endTime: clock + horizontalDuration,
    duration: horizontalDuration,
  });
  clock += horizontalDuration;
  append("settle", 4, cruiseHeight, cruiseHeight, finalYaw);
  append(
    "landing",
    verticalDuration(cruiseHeight - 3, settings.descentSpeed),
    cruiseHeight,
    3,
    finalYaw,
  );
  append("touchdown", 18, 3, -0.08, finalYaw);
  append("disarm", 4, -0.08, -0.08, finalYaw);
  return {
    horizontal,
    phases,
    duration: clock,
    surveyStart,
    horizontalDuration,
    cruiseHeight,
    initialYaw,
    finalYaw,
    pathLength: path.length,
    timeScale: scale,
  };
}

export function referenceAt(ref, time) {
  const t = clamp(time, 0, ref.duration);
  const phase =
    ref.phases.find((p) => t <= p.endTime + 1e-9) || ref.phases.at(-1);
  if (phase.phase !== "survey") {
    const z = polynomial(
      phase.z,
      clamp((t - phase.startTime) / phase.duration, 0, 1),
      phase.duration,
    );
    return {
      p: [0, 0, z.p],
      v: [0, 0, z.v],
      a: [0, 0, z.a],
      yaw: phase.yaw,
      yawRate: 0,
      phase: phase.phase,
      progress: phase.startTime > ref.surveyStart ? 1 : 0,
    };
  }
  const local = t - ref.surveyStart;
  let lo = 0,
    hi = ref.horizontal.length - 1;
  while (lo < hi) {
    const m = (lo + hi) >> 1;
    if (ref.horizontal[m].startTime + ref.horizontal[m].duration < local)
      lo = m + 1;
    else hi = m;
  }
  const seg = ref.horizontal[lo],
    u = clamp((local - seg.startTime) / seg.duration, 0, 1);
  const [x, y] = seg.coefficients.map((c) => polynomial(c, u, seg.duration));
  const speed2 = x.v * x.v + y.v * y.v;
  return {
    p: [x.p, y.p, ref.cruiseHeight],
    v: [x.v, y.v, 0],
    a: [x.a, y.a, 0],
    yaw:
      speed2 > 1e-8
        ? Math.atan2(y.v, x.v)
        : local < 1
          ? ref.initialYaw
          : ref.finalYaw,
    yawRate: speed2 > 1e-8 ? (x.v * y.a - y.v * x.a) / speed2 : 0,
    phase: local > ref.horizontalDuration * 0.88 ? "return" : "survey",
    progress: local / ref.horizontalDuration,
  };
}
