import { simulate } from "./quadrotor.mjs";
self.onmessage = (event) => {
  try {
    const simulation = simulate(
      event.data.reference,
      event.data.options,
      (progress) => self.postMessage({ progress }),
    );
    self.postMessage({ simulation }, [simulation.data.buffer]);
  } catch (error) {
    self.postMessage({ error: error.message });
  }
};
