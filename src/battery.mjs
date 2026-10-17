// Generic 6S lithium-polymer pack and propulsion chain for a 2 kg class
// quadrotor. Textbook values, not a measured pack: the take-off mass entered
// in the UI includes this battery.
export const BATTERY = Object.freeze({
  cells: 6,
  capacityAh: 6,
  // Pack resistance: six cells of about 4 mΩ plus leads and connectors, Ω.
  internalResistance: 0.03,
  // Electrical to shaft power through ESC and motor.
  propulsionEfficiency: 0.78,
  // Flight controller, GNSS, datalink and the gimbal camera, W.
  avionicsPower: 18,
  // State of charge that must remain at touchdown (operator landing reserve).
  reserve: 0.25,
  // Loaded cell voltage below which the pack is considered over-discharged.
  minCellVoltage: 3.3,
  fullCellVoltage: 4.2,
});

// Open-circuit voltage of one LiPo cell over state of charge, typical
// discharge curve at room temperature.
const OPEN_CIRCUIT = [
  [0, 3.27],
  [0.05, 3.5],
  [0.1, 3.62],
  [0.2, 3.7],
  [0.3, 3.75],
  [0.4, 3.79],
  [0.5, 3.83],
  [0.6, 3.87],
  [0.7, 3.92],
  [0.8, 4.0],
  [0.9, 4.08],
  [1, 4.2],
];
export function cellOpenCircuitVoltage(soc) {
  const s = Math.min(1, Math.max(0, soc));
  for (let i = 1; i < OPEN_CIRCUIT.length; i++) {
    const [s1, v1] = OPEN_CIRCUIT[i];
    if (s <= s1) {
      const [s0, v0] = OPEN_CIRCUIT[i - 1];
      return v0 + ((v1 - v0) * (s - s0)) / (s1 - s0);
    }
  }
  return OPEN_CIRCUIT.at(-1)[1];
}

// Electrical power drawn from the pack for a given total shaft power.
export function electricalPower(shaftPower, battery = BATTERY) {
  return shaftPower / battery.propulsionEfficiency + battery.avionicsPower;
}

// Terminal state for a power demand P on a Thevenin pack (open-circuit E,
// resistance R): P = V I and V = E − I R give I = (E − √(E² − 4 R P)) / 2R.
// Above E² / 4R the pack cannot deliver the power; it then supplies its
// maximum, E² / 4R at V = E / 2, and `limited` is set.
export function batteryTerminal(power, soc, battery = BATTERY) {
  const E = battery.cells * cellOpenCircuitVoltage(soc),
    R = battery.internalResistance,
    disc = E * E - 4 * R * power;
  if (disc < 0)
    return { voltage: E / 2, current: E / (2 * R), openCircuit: E, limited: true };
  const current = (E - Math.sqrt(disc)) / (2 * R);
  return { voltage: E - current * R, current, openCircuit: E, limited: false };
}

// Coulomb counting: state of charge after drawing `current` for `dt` seconds.
export function dischargeStep(soc, current, dt, battery = BATTERY) {
  return soc - (current * dt) / (3600 * battery.capacityAh);
}

// Ratio of the available to the nominal top motor speed: brushless motor
// speed scales with supply voltage (Kv), so a sagging pack loses authority.
export function voltageAuthority(voltage, battery = BATTERY) {
  return Math.min(1, voltage / (battery.cells * battery.fullCellVoltage));
}

export function nominalEnergyWh(battery = BATTERY) {
  return battery.cells * 3.7 * battery.capacityAh;
}
