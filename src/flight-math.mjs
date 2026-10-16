// Right-handed ENU world, FLU body (forward/left/up). Quaternion order w,x,y,z.
export const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
export const dot = (a, b) => a.reduce((s, x, i) => s + x * b[i], 0);
export const norm = (a) => Math.hypot(...a);
export const add = (a, b) => a.map((x, i) => x + b[i]);
export const sub = (a, b) => a.map((x, i) => x - b[i]);
export const mul = (a, k) => a.map((x) => x * k);
export const unit = (a) => mul(a, 1 / Math.max(1e-12, norm(a)));
export const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const wrap = (x) => Math.atan2(Math.sin(x), Math.cos(x));
export const conjugate = (q) => [q[0], -q[1], -q[2], -q[3]];
export const qmul = (a, b) => [
  a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
  a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
  a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
  a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
];
export function rotate(q, v) {
  const t = mul(cross(q.slice(1), v), 2);
  return add(v, add(mul(t, q[0]), cross(q.slice(1), t)));
}
export function quaternionFromBasis(x, y, z) {
  const m = [x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]];
  const tr = m[0] + m[4] + m[8];
  let q;
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2;
    q = [s / 4, (m[7] - m[5]) / s, (m[2] - m[6]) / s, (m[3] - m[1]) / s];
  } else if (m[0] > m[4] && m[0] > m[8]) {
    const s = Math.sqrt(1 + m[0] - m[4] - m[8]) * 2;
    q = [(m[7] - m[5]) / s, s / 4, (m[1] + m[3]) / s, (m[2] + m[6]) / s];
  } else if (m[4] > m[8]) {
    const s = Math.sqrt(1 + m[4] - m[0] - m[8]) * 2;
    q = [(m[2] - m[6]) / s, (m[1] + m[3]) / s, s / 4, (m[5] + m[7]) / s];
  } else {
    const s = Math.sqrt(1 + m[8] - m[0] - m[4]) * 2;
    q = [(m[3] - m[1]) / s, (m[2] + m[6]) / s, (m[5] + m[7]) / s, s / 4];
  }
  return unit(q);
}
export function slerp(a, b, u) {
  let d = dot(a, b);
  if (d < 0) {
    b = mul(b, -1);
    d = -d;
  }
  if (d > 0.9995) return unit(add(mul(a, 1 - u), mul(b, u)));
  const theta = Math.acos(clamp(d, -1, 1));
  return add(
    mul(a, Math.sin((1 - u) * theta) / Math.sin(theta)),
    mul(b, Math.sin(u * theta) / Math.sin(theta)),
  );
}
export function attitudeFromForce(force, yaw) {
  const z = unit(force),
    y = unit(cross(z, [Math.cos(yaw), Math.sin(yaw), 0]));
  return quaternionFromBasis(cross(y, z), y, z);
}
// Quintic Hermite coefficients with position, velocity, acceleration at both ends.
export function quintic(p0, v0, a0, p1, v1, a1, T) {
  const c0 = p0,
    c1 = v0 * T,
    c2 = (a0 * T * T) / 2;
  const A = p1 - c0 - c1 - c2,
    B = v1 * T - c1 - 2 * c2,
    D = a1 * T * T - 2 * c2;
  return [
    c0,
    c1,
    c2,
    10 * A - 4 * B + D / 2,
    -15 * A + 7 * B - D,
    6 * A - 3 * B + D / 2,
  ];
}
export function polynomial(c, u, T) {
  const p = c[0] + u * (c[1] + u * (c[2] + u * (c[3] + u * (c[4] + u * c[5]))));
  const v =
    (c[1] + u * (2 * c[2] + u * (3 * c[3] + u * (4 * c[4] + u * 5 * c[5])))) /
    T;
  const a =
    (2 * c[2] + u * (6 * c[3] + u * (12 * c[4] + u * 20 * c[5]))) / (T * T);
  const j = (6 * c[3] + u * (24 * c[4] + u * 60 * c[5])) / (T * T * T);
  return { p, v, a, j };
}
