// mulberry32: a tiny portable 32-bit generator; the same seed gives the same
// sequence on every engine. Box–Muller turns uniform pairs into standard
// normals, keeping the second sample for the next call.
export function makeRandom(seed = 1) {
  let a = seed >>> 0,
    spare = NaN;
  const uniform = () => {
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  const normal = () => {
    if (!Number.isNaN(spare)) {
      const z = spare;
      spare = NaN;
      return z;
    }
    const r = Math.sqrt(-2 * Math.log(1 - uniform())),
      theta = 2 * Math.PI * uniform();
    spare = r * Math.sin(theta);
    return r * Math.cos(theta);
  };
  return { uniform, normal };
}

// First-order Gauss–Markov process: exponentially correlated noise with
// standard deviation `sigma` and correlation time `tau`, stepped exactly.
// Starts from its stationary distribution.
export function makeGaussMarkov(random, sigma, tau) {
  let value = sigma * random.normal();
  return {
    get value() {
      return value;
    },
    step(dt) {
      const a = Math.exp(-dt / tau);
      value = a * value + sigma * Math.sqrt(1 - a * a) * random.normal();
      return value;
    },
  };
}
