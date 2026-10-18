import { rotate } from "./flight-math.mjs";

// In-flight wind estimate from what an autopilot measures, in the spirit of
// PX4's drag-fusion wind estimation: the aerodynamic force is the measured
// acceleration times mass minus the rotor thrust vector and gravity (thrust
// from ESC-reported rotor speeds and the nominal thrust coefficient, its
// direction from the attitude). The airframe's drag model, body drag
// ½ ρ CdA |v_a| v_a plus rotor drag c_lin n (v_a in the rotor plane; n is
// the rotor speed sum relative to hover), is inverted for the horizontal
// air-relative velocity v_a by a short fixed-point iteration (vertical wind
// taken as zero), and wind = ground velocity − v_a. A first-order low-pass
// (time constant `tau`) averages out gusts and sensor noise. Horizontal
// only; the estimate holds while the vehicle is on the ground.
export function makeWindEstimator(c, tau = 10) {
  const a = 0.5 * c.airDensity * c.dragArea;
  const estimator = {
    wind: [0, 0],
    initialized: false,
    // velocity: navigation estimate (ENU), acceleration: measured (ENU),
    // q: attitude, thrust: estimated collective thrust (N), rotorRatio: n.
    step(velocity, acceleration, q, thrust, dt, rotorRatio = 1) {
      const z = rotate(q, [0, 0, 1]),
        fx = c.mass * acceleration[0] - thrust * z[0],
        fy = c.mass * acceleration[1] - thrust * z[1],
        linear = c.linearDrag * rotorRatio,
        force = Math.hypot(fx, fy);
      // Level-flight solution as the starting point.
      const speed = (-linear + Math.sqrt(linear ** 2 + 4 * a * force)) / (2 * a);
      let rx = force > 1e-9 ? (-fx / force) * speed : 0,
        ry = force > 1e-9 ? (-fy / force) * speed : 0;
      const rz = velocity[2];
      for (let i = 0; i < 4; i++) {
        const axial = rx * z[0] + ry * z[1] + rz * z[2],
          denominator = a * Math.hypot(rx, ry, rz) + linear;
        rx = -(fx - linear * axial * z[0]) / denominator;
        ry = -(fy - linear * axial * z[1]) / denominator;
      }
      const wx = velocity[0] - rx,
        wy = velocity[1] - ry;
      if (!estimator.initialized) {
        estimator.wind[0] = wx;
        estimator.wind[1] = wy;
        estimator.initialized = true;
        return estimator.wind;
      }
      const gain = 1 - Math.exp(-dt / tau);
      estimator.wind[0] += (wx - estimator.wind[0]) * gain;
      estimator.wind[1] += (wy - estimator.wind[1]) * gain;
      return estimator.wind;
    },
  };
  return estimator;
}
