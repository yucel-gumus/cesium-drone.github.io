/* global Cesium */
import {
  createTerrainRoute,
  surveySorties,
  sortieLimit,
  sampleTerrainRoute,
  buildMission,
  evaluateMission,
  attachSimulation,
  terrainForWorker,
  terrainAt,
  obstaclesAlongPath,
} from "./flight-planner.mjs";
import DroneUIController from "./ui-controller.js";
import { createPositionProperty } from "./position-property.mjs";
import {
  ROTOR_NODES,
  MODEL_ROOT_NODE,
  modelScale,
  modelGroundOffset,
} from "./drone-model.mjs";
import "./style.css";
import { AIRFRAME, SPIN, STRIDE, windLimit } from "./quadrotor.mjs";
import { analyzeMission, planPhotos, toGeodetic } from "./mission-analysis.mjs";
import { BATTERY } from "./battery.mjs";
import { groundSampleDistance, fieldOfView } from "./payload.mjs";
import { toCSV, toKML, toQGCPlan, toGeoJSON, toULog } from "./exports.mjs";
import { fromCSV, fromULog, identify } from "./sysid.mjs";
import { FAULTS } from "./failsafe.mjs";
import { obstacleText } from "./obstacles.mjs";
import { formatTime, formatDistance } from "./ui-controller.js";

const C = globalThis.Cesium;
// Bearings are relative to the drone's heading; negative pitch keeps the camera above it.
const FOLLOW_VIEWS = {
  rear: { bearing: 0, pitch: -0.32 },
  front: { bearing: Math.PI, pitch: -0.32 },
  left: { bearing: Math.PI / 2, pitch: -0.22 },
  right: { bearing: -Math.PI / 2, pitch: -0.22 },
  top: { bearing: 0, pitch: -Math.PI / 2 + 0.015 },
  diagonal: { bearing: Math.PI / 4, pitch: -0.6 },
};
const buildToken = process.env.CESIUM_ION_TOKEN;
let viewer, mission, analysis, drone, provider;
// A multi-flight survey: every sortie's { mission, analysis }; `mission`
// and `analysis` are the selected one.
let sorties = [],
  sortieIndex = 0,
  // Planned photos of all sorties, for the survey's area coverage.
  surveyPhotos;
// Calibration from a flight log (sysid.mjs): the identification result and
// whether the planner flies the calibrated airframe and battery.
let calibration = null,
  calibrationApplied = false;
// Airframe for a take-off mass: nominal with inertia scaled by mass, or the
// calibrated one (identified at the log's mass) scaled the same way.
function airframeFor(mass) {
  const base = calibrationApplied ? calibration.result.airframe : AIRFRAME,
    from = calibrationApplied ? calibration.mass : 2;
  return { ...base, mass, inertia: base.inertia.map((x) => (x * mass) / from) };
}
const batteryModel = () => (calibrationApplied ? calibration.result.battery : BATTERY);
// Obstacles near the route (buildings, masts, tree rows) in their own data
// source; the OSM snapshot is loaded once.
let obstacleLayer,
  obstacleSnapshot;
// Image footprints live in their own data source so the gimbal view can
// hide them: the payload camera sees the ground, not the plan overlay.
let footprints;
let currentState = "loading";
let cameraMode = "overview";
let runId = 0;
let startTime;
let lastUiUpdate = 0;
let missionEntities = [];
let cancelSolve;
// Sampled terrain heights by position for the current terrain provider, so
// re-plans (another height, side overlap or mission type) query only the
// positions they have not seen.
let terrainCache;
let cachedSample, cachedTime;
let followHeading = 0,
  followPitch = FOLLOW_VIEWS.rear.pitch,
  followView = "rear",
  followRange = 8,
  targetFollowRange = 8,
  lastCameraFrame = 0;
// Field of view and controller state to restore when leaving the gimbal view.
let savedFov;

const ui = new DroneUIController({
  start() {
    if (
      !mission ||
      analysis?.blocked ||
      currentState === "running" ||
      currentState === "paused"
    )
      return;
    viewer.clock.currentTime = startTime.clone();
    setState("running");
    viewer.clock.shouldAnimate = true;
    setCamera("follow");
    ui.message(
      mission.survey
        ? "Haritalama uçuşu başladı. Izgara hatları sabit AGL'de uçuluyor, kamera mesafeye göre tetikleniyor."
        : "Uçuş başladı. Drone araziyi sabit AGL'de izliyor, kamera mesafeye göre tetikleniyor.",
    );
    render();
  },
  pause() {
    if (!["running", "paused"].includes(currentState)) return;
    const running = currentState === "running";
    viewer.clock.shouldAnimate = !running;
    setState(running ? "paused" : "running");
    render();
  },
  reset() {
    if (!mission || analysis?.blocked) return;
    viewer.clock.shouldAnimate = false;
    viewer.clock.currentTime = startTime.clone();
    setState("ready");
    ui.message("Kalkış noktasında hazır. Keşif uçuşunu başlatabilirsiniz.");
    render();
  },
  plan(agl) {
    prepareMission(agl);
  },
  token(token) {
    if (!token) {
      ui.message("Önce Cesium ion erişim tokenınızı girin.", true);
      return;
    }
    try {
      sessionStorage.setItem("uzungol-ion-token", token);
    } catch {
      /* Private storage may be disabled. */
    }
    ui.elements.ionToken.value = "";
    provider = undefined;
    prepareMission(Number(ui.elements.aglInput.value), token);
  },
  seek(progress) {
    seekTo(progress * (mission?.duration ?? 0));
  },
  seekTime: seekTo,
  operator: operatorCommand,
  sortie: selectSortie,
  rate(multiplier) {
    if (viewer) viewer.clock.multiplier = multiplier;
  },
  camera: setCamera,
  followView: setFollowView,
  followDistance: setFollowDistance,
  // Identifies the airframe from a flight log (PX4 .ulg or the simulator's
  // CSV) at the current take-off mass.
  async calibrate(file) {
    const source = file.name;
    ui.setCalibration({ busy: true, source });
    await new Promise((resolve) => setTimeout(resolve, 30));
    try {
      const mass = Number(ui.elements.massInput.value);
      if (!Number.isFinite(mass) || mass < 1 || mass > 3)
        throw new Error("Kalkış kütlesi 1–3 kg arasında olmalı.");
      const bytes = new Uint8Array(await file.arrayBuffer()),
        ulog = bytes[0] === 0x55 && bytes[1] === 0x4c && bytes[2] === 0x6f && bytes[3] === 0x67,
        parsed = ulog ? fromULog(bytes) : { rows: fromCSV(new TextDecoder().decode(bytes)) },
        nominal = { ...AIRFRAME, mass, inertia: AIRFRAME.inertia.map((x) => (x * mass) / 2) },
        result = identify(parsed.rows, nominal, BATTERY, {
          ...(Number.isFinite(parsed.airDensity) && { airDensity: parsed.airDensity }),
        });
      calibration = { source, mass, result };
      calibrationApplied = false;
      ui.setCalibration({ source, result, applied: false });
    } catch (error) {
      ui.setCalibration({ error: `${source}: ${error.message}`, applied: calibrationApplied });
    }
  },
  applyCalibration(on) {
    if (on && !calibration) return;
    calibrationApplied = on;
    ui.syncWindLimit();
    if (calibration) ui.setCalibration({ ...calibration, applied: on });
    else ui.setCalibration({});
    prepareMission(Number(ui.elements.aglInput.value));
  },
  export(kind) {
    if (!mission?.simulation || !analysis) return;
    const files = {
      csv: () => [toCSV(mission), "text/csv;charset=utf-8", "csv"],
      kml: () => [
        toKML(C, mission, analysis),
        "application/vnd.google-earth.kml+xml",
        "kml",
      ],
      plan: () => [
        JSON.stringify(toQGCPlan(C, mission, analysis), null, 2),
        "application/json",
        "plan",
      ],
      geojson: () => [
        JSON.stringify(toGeoJSON(C, mission, analysis)),
        "application/geo+json",
        "geojson",
      ],
      ulog: () => [toULog(mission), "application/octet-stream", "ulg"],
    };
    if (!files[kind]) return;
    const [content, type, extension] = files[kind]();
    const url = URL.createObjectURL(new Blob([content], { type }));
    const a = document.createElement("a");
    a.href = url;
    a.download = `uzungol-${mission.survey ? "haritalama" : "kesif"}${sorties.length > 1 ? `-ucus${sortieIndex + 1}` : ""}.${extension}`;
    a.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  },
});
ui.airframeModel = airframeFor;

function setState(state) {
  currentState = state;
  ui.setState(state);
}
// Scrubbing a blocked plan keeps it blocked: it can be inspected, not flown.
function seekTo(seconds) {
  if (!mission) return;
  viewer.clock.shouldAnimate = false;
  const t = Math.min(mission.duration, Math.max(0, seconds));
  viewer.clock.currentTime = C.JulianDate.addSeconds(
    startTime,
    t,
    new C.JulianDate(),
  );
  if (!analysis?.blocked)
    setState(t >= mission.duration - 1e-6 ? "complete" : "paused");
  render();
}
// Photos taken by `seconds`, the live ground sampling distance and the last
// exposure time, for telemetry and the gimbal HUD.
function payloadAt(sample) {
  const photos = analysis.capture.photos;
  let lo = 0,
    hi = photos.length;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (photos[mid].time <= sample.elapsed) lo = mid + 1;
    else hi = mid;
  }
  const pitch = (analysis.capture.pitchDegrees * Math.PI) / 180;
  return {
    taken: lo,
    total: photos.length,
    last: lo ? photos[lo - 1].time : null,
    gsd: groundSampleDistance(Math.max(0, sample.agl) / Math.sin(-pitch)),
    pitchDegrees: analysis.capture.pitchDegrees,
  };
}
function elapsed(time = viewer.clock.currentTime) {
  return C.JulianDate.secondsDifference(time, startTime);
}
function sampleAt(time) {
  if (!cachedSample || !C.JulianDate.equals(time, cachedTime)) {
    cachedSample = evaluateMission(C, mission, elapsed(time));
    cachedTime = C.JulianDate.clone(time, cachedTime);
  }
  return cachedSample;
}
function toCartesian(position, result) {
  return C.Cartesian3.fromRadians(
    position.longitude,
    position.latitude,
    position.height,
    C.Ellipsoid.WGS84,
    result,
  );
}

function render() {
  if (!mission) return;
  const sample = sampleAt(viewer.clock.currentTime);
  if (sample.phase === "complete" && currentState === "running") {
    viewer.clock.shouldAnimate = false;
    setState("complete");
    const m = mission.simulation.metrics;
    ui.message(
      m.landingSite === "field"
        ? `Uçuş failsafe ile bitti: drone evden ${formatDistance(m.homeDistance)} uzakta yerinde indi.`
        : m.landingSite === "rally"
          ? `Uçuş rally noktasında (${m.landingName}) bitti.`
          : mission.simulation.failsafe?.returnReason ||
              mission.simulation.failsafe?.operator
            ? "Görev failsafe ya da operatör komutuyla kesildi; drone eve dönüp kalktığı noktaya indi."
            : mission.survey
              ? "Haritalama tamamlandı. Drone kalktığı noktaya indi."
              : "Keşif tamamlandı. Drone kalktığı noktaya indi.",
    );
  }
  ui.render(sample, C, payloadAt(sample));
}

function setCamera(mode) {
  if (!viewer || !["overview", "follow", "gimbal"].includes(mode)) return;
  if (mode !== "overview" && cameraMode === mode) return;
  viewer.camera.cancelFlight();
  const leavingGimbal = cameraMode === "gimbal" && mode !== "gimbal";
  cameraMode = mode;
  viewer.trackedEntity = undefined;
  viewer.camera.lookAtTransform(C.Matrix4.IDENTITY);
  ui.setCamera(mode);
  if (footprints) footprints.show = mode !== "gimbal";
  const controller = viewer.scene.screenSpaceCameraController;
  controller.enableZoom = mode === "overview";
  controller.enableInputs = mode !== "gimbal";
  viewer.camera.frustum.near = mode === "overview" ? 1 : 0.1;
  if (mode === "gimbal") {
    // Match the payload camera: the frustum's fov spans the canvas's longer
    // side, so use the sensor's matching angle.
    savedFov ??= viewer.camera.frustum.fov;
    const fov = fieldOfView(),
      canvas = viewer.canvas;
    viewer.camera.frustum.fov =
      canvas.clientWidth >= canvas.clientHeight ? fov.horizontal : fov.vertical;
  } else if (leavingGimbal && savedFov !== undefined) {
    viewer.camera.frustum.fov = savedFov;
    savedFov = undefined;
  }
  if (!mission || mode === "gimbal") return;
  if (mode === "follow") {
    followHeading =
      sampleAt(viewer.clock.currentTime).heading + FOLLOW_VIEWS[followView].bearing;
    followPitch = FOLLOW_VIEWS[followView].pitch;
    followRange = 80;
  } else if (mode === "overview") {
    const bounds = C.BoundingSphere.fromPoints(
      mission.path.table.map((k, i) =>
        C.Matrix4.multiplyByPoint(
          mission.frame,
          new C.Cartesian3(k.p[0], k.p[1], mission.profile[i]),
          new C.Cartesian3(),
        ),
      ),
    );
    viewer.camera.flyToBoundingSphere(bounds, {
      duration: 1.2,
      offset: new C.HeadingPitchRange(
        C.Math.toRadians(15),
        C.Math.toRadians(-55),
        Math.max(1500, bounds.radius * 3.1),
      ),
    });
  }
}

function setFollowView(view) {
  if (!Object.hasOwn(FOLLOW_VIEWS, view)) return;
  followView = view;
  ui.setFollowView(view);
  setCamera("follow");
}

function setFollowDistance(distance) {
  if (!Number.isFinite(distance)) return;
  targetFollowRange = Math.max(1.2, Math.min(80, distance));
  ui.setFollowDistance(targetFollowRange);
}

function initializeFollowZoom() {
  const canvas = viewer.canvas;
  let pinchDistance;
  canvas.addEventListener(
    "wheel",
    (event) => {
      if (cameraMode !== "follow" || !mission) return;
      event.preventDefault();
      const pixels =
        event.deltaY *
        (event.deltaMode === 1
          ? 16
          : event.deltaMode === 2
            ? canvas.clientHeight
            : 1);
      setFollowDistance(
        targetFollowRange *
          Math.exp(Math.max(-2, Math.min(2, pixels * 0.0015))),
      );
    },
    { passive: false },
  );
  const span = (touches) =>
    Math.hypot(
      touches[0].clientX - touches[1].clientX,
      touches[0].clientY - touches[1].clientY,
    );
  canvas.addEventListener(
    "touchstart",
    (event) => {
      if (cameraMode === "follow" && event.touches.length === 2)
        pinchDistance = span(event.touches);
    },
    { passive: true },
  );
  canvas.addEventListener(
    "touchmove",
    (event) => {
      if (cameraMode !== "follow" || !mission || event.touches.length !== 2) {
        pinchDistance = undefined;
        return;
      }
      event.preventDefault();
      const distance = span(event.touches);
      if (pinchDistance > 0 && distance > 0)
        setFollowDistance((targetFollowRange * pinchDistance) / distance);
      pinchDistance = distance;
    },
    { passive: false },
  );
  const endPinch = () => {
    pinchDistance = undefined;
  };
  canvas.addEventListener("touchend", endPinch);
  canvas.addEventListener("touchcancel", endPinch);
}

// Solves flights in Web Workers, one per job ({ reference, options }), in
// parallel; resolves with the simulations in job order. A failed job
// resolves to its Error when `tolerant` is set (the fault rehearsal must
// not sink a valid plan).
function solveDynamics(jobs, tolerant = []) {
  cancelSolve?.();
  const workers = jobs.map(
    () => new Worker(new URL("./simulation-worker.mjs", import.meta.url)),
  );
  const progress = jobs.map(() => 0);
  let cancelled = false;
  const cancel = () => {
    cancelled = true;
    workers.forEach((w) => w.terminate());
  };
  cancelSolve = cancel;
  const solve = (job, i) =>
    new Promise((resolve, reject) => {
      const fail = (error) =>
        tolerant[i] && !cancelled ? resolve(error) : reject(error);
      workers[i].onmessage = ({ data }) => {
        if (data.error) fail(new Error(data.error));
        else if (data.simulation) resolve(data.simulation);
        else {
          progress[i] = data.progress;
          if (currentState === "loading")
            ui.message(
              `240 Hz uçuş dinamiği çözülüyor${jobs.length > 1 ? " · nominal + arıza senaryosu" : ""} · %${Math.round((100 * progress.reduce((a, b) => a + b, 0)) / jobs.length)}`,
            );
        }
      };
      workers[i].onerror = () =>
        fail(
          new Error("Fizik işlemcisi başlatılamadı. Sayfayı yeniden yükleyin."),
        );
      workers[i].postMessage(job);
    });
  return withTimeout(
    new Promise((resolve, reject) => {
      // A newer plan supersedes this solve: stop the workers and settle now
      // instead of leaving the promise pending until the timeout.
      cancelSolve = () => {
        cancel();
        reject(new Error("Fizik çözümü yeni bir planla iptal edildi."));
      };
      Promise.all(jobs.map(solve)).then(resolve, reject);
    }),
    120000,
    "Fizik çözümü zaman aşımına uğradı.",
  ).finally(() => {
    workers.forEach((w) => w.terminate());
    if (!cancelled) cancelSolve = undefined;
  });
}

async function withTimeout(promise, milliseconds, message) {
  let timer;
  try {
    return await Promise.race([
      promise,
      new Promise((_, reject) => {
        timer = setTimeout(() => reject(new Error(message)), milliseconds);
      }),
    ]);
  } finally {
    clearTimeout(timer);
  }
}

async function prepareMission(agl = 80, overrideToken) {
  if (!viewer) return;
  if (!Number.isFinite(agl) || agl < 30 || agl > 120) {
    ui.message("Hedef yükseklik 30–120 m AGL arasında olmalı.", true);
    return;
  }
  const id = ++runId;
  setState("loading");
  viewer.clock.shouldAnimate = false;
  viewer.trackedEntity = undefined;
  cancelSolve?.();
  mission = undefined;
  analysis = undefined;
  sorties = [];
  sortieIndex = 0;
  surveyPhotos = undefined;
  ui.setSorties(0, 0);
  cachedSample = undefined;
  clearMission();
  ui.elements.mapError.hidden = true;
  ui.message("Gerçek arazi yükseklikleri yükleniyor…");
  try {
    let savedToken = "";
    try {
      savedToken = sessionStorage.getItem("uzungol-ion-token") || "";
    } catch {
      /* Optional storage. */
    }
    const token = overrideToken || savedToken || buildToken;
    if (!token)
      throw new Error(
        "Arazi verisi için Cesium ion token gerekli. “Harita erişim ayarı” bölümünden token girin.",
      );
    C.Ion.defaultAccessToken = token;
    if (!provider) {
      // Globe lighting is off, so vertex normals would only add tile bytes.
      provider = await withTimeout(
        C.createWorldTerrainAsync({ requestVertexNormals: false }),
        25000,
        "Arazi servisi yanıt vermedi. Bağlantınızı veya tokenınızı kontrol edin.",
      );
      viewer.terrainProvider = provider;
    }
    const response = await fetch(new URL("./points.json", document.baseURI));
    if (!response.ok)
      throw new Error(`Rota dosyası yüklenemedi (HTTP ${response.status}).`);
    const waypointText = await response.text();
    // Obstacle snapshot (OpenStreetMap buildings, trees, masts); without it
    // the checklist reports that obstacles were not checked.
    if (obstacleSnapshot === undefined) {
      try {
        const answer = await fetch(new URL("./obstacles.json", document.baseURI));
        obstacleSnapshot = answer.ok ? await answer.json() : null;
      } catch {
        obstacleSnapshot = null;
      }
    }
    const mass = Number(ui.elements.massInput.value),
      wind = Number(ui.elements.windInput.value),
      seed = Number(ui.elements.seedInput.value),
      turbulence = ui.elements.turbulenceInput.checked,
      gimbalPitch = Number(ui.elements.gimbalInput.value),
      frontOverlap = Number(ui.elements.overlapInput.value) / 100,
      gnss = ui.elements.gnssInput.value,
      type = ui.elements.missionTypeInput.value,
      sideOverlap = Number(ui.elements.sideOverlapInput.value) / 100,
      faultType = ui.elements.faultInput.value,
      faultTime = Number(ui.elements.faultTimeInput.value),
      secondType = ui.elements.secondFaultInput.value,
      secondTime = Number(ui.elements.secondFaultTimeInput.value),
      rtlTarget = ui.elements.rtlTargetInput.value;
    if (!Number.isFinite(mass) || mass < 1 || mass > 3)
      throw new Error("Kalkış kütlesi 1–3 kg arasında olmalı.");
    // Mean-wind authority limit of this airframe (see windLimit()).
    const limit = windLimit(airframeFor(mass));
    if (!Number.isFinite(wind) || wind < 0 || wind > limit)
      throw new Error(
        `${mass.toLocaleString("tr")} kg gövdede rüzgâr 0–${limit.toLocaleString("tr")} m/s arasında olmalı (yetki sınırı).`,
      );
    // mulberry32 takes a 32-bit seed; larger or fractional values would alias.
    if (!Number.isInteger(seed) || seed < 0 || seed > 4294967295)
      throw new Error(
        "Türbülans tohumu 0–4294967295 arasında bir tam sayı olmalı.",
      );
    if (
      !Number.isFinite(frontOverlap) ||
      frontOverlap < 0.6 ||
      frontOverlap > 0.9 ||
      ![-90, -70, -60, -45].includes(gimbalPitch) ||
      !["rtk", "standard"].includes(gnss)
    )
      throw new Error(
        "Bindirme %60–90, gimbal −90…−45° ve navigasyon RTK/standart olmalı.",
      );
    if (
      !["recon", "survey"].includes(type) ||
      !Number.isFinite(sideOverlap) ||
      sideOverlap < 0.6 ||
      sideOverlap > 0.8
    )
      throw new Error("Görev tipi keşif/haritalama, yan bindirme %60–80 olmalı.");
    const faults = [
      [faultType, faultTime],
      [secondType, secondTime],
    ]
      .filter(([type]) => type !== "none")
      .map(([type, time]) => {
        if (!Object.hasOwn(FAULTS, type) || !Number.isFinite(time) || time < 10 || time > 3600)
          throw new Error("Arıza zamanı 10–3600 s arasında olmalı.");
        return { type, time };
      });
    if (
      new Set(faults.map((f) => f.type.replace(/-.*/, ""))).size !== faults.length
    )
      throw new Error("Aynı türden iki arıza (ör. iki GNSS arızası) birlikte seçilemez.");
    if (!["nearest", "home"].includes(rtlTarget))
      throw new Error("RTL hedefi en yakın güvenli nokta ya da kalkış noktası olmalı.");
    const airframe = airframeFor(mass),
      battery = batteryModel();
    // An area survey larger than one battery flies as several sorties.
    const data = JSON.parse(waypointText),
      routes =
        type === "survey"
          ? surveySorties(
              C,
              data,
              { agl, wind },
              { sideOverlap, gimbalPitch, obstacles: obstacleSnapshot },
              sortieLimit(airframe, battery),
            )
          : [createTerrainRoute(C, data, { agl, wind }, { type, obstacles: obstacleSnapshot })];
    // Heights are cached by position: re-planning with another height,
    // mass, wind, payload or fault reuses them, a new grid samples only its
    // new positions.
    if (terrainCache?.provider !== provider)
      terrainCache = { provider, heights: new Map() };
    for (const [k, route] of routes.entries())
      await withTimeout(
        sampleTerrainRoute(
          C,
          provider,
          route,
          (progress) => {
            if (id === runId && currentState === "loading")
              ui.message(
                `Arazi koridoru örnekleniyor${routes.length > 1 ? ` · uçuş ${k + 1}/${routes.length}` : ""} · %${Math.round(progress * 100)}`,
              );
          },
          terrainCache.heights,
        ),
        90000,
        "Arazi örneklemesi tamamlanamadı. Yeniden hesaplayın veya erişim tokenını kontrol edin.",
      );
    if (id !== runId) return;
    const candidates = routes.map((route) => buildMission(C, route));
    const options = { airframe, battery, wind, seed, turbulence, gnss };
    for (const candidate of candidates)
      candidate.model = calibrationApplied
        ? `kalibre · ${calibration.source} (${calibration.result.report.filter((r) => r.applied).length} parametre)`
        : "nominal";
    // Everything a failsafe or an operator command needs to re-plan in
    // flight; kept so an operator command can re-solve the flight. Faults
    // are injected into the first sortie.
    for (const [k, candidate] of candidates.entries())
      candidate.flight = {
        options,
        armed: {
          failsafe: true,
          settings: candidate.settings,
          terrain: terrainForWorker(candidate),
          rally: candidate.rally.map(({ name, xy }) => ({ name, xy })),
          rtlTarget,
        },
        faults: k === 0 ? faults : [],
        commands: [],
      };
    // Each sortie solved fault-free for the preflight checklist; with faults
    // the first one also faulted (failsafes armed) for the flight shown. All
    // in parallel workers.
    const jobs = candidates.map((candidate) => ({
      reference: candidate.reference,
      options,
    }));
    if (faults.length)
      jobs.push({
        reference: candidates[0].reference,
        options: { ...options, ...candidates[0].flight.armed, faults },
      });
    const solved = await solveDynamics(
      jobs,
      jobs.map((_, k) => k >= candidates.length),
    );
    if (id !== runId) return;
    const faulted = faults.length ? solved.at(-1) : null,
      scenarioError = faulted instanceof Error ? faulted.message : null;
    candidates.forEach((candidate, k) =>
      attachSimulation(
        candidate,
        k === 0 && faulted && !scenarioError ? faulted : solved[k],
        solved[k],
      ),
    );
    const payload = { gimbalPitch, frontOverlap };
    surveyPhotos =
        candidates.length > 1
          ? candidates.flatMap((candidate) => planPhotos(candidate, payload))
          : undefined;
    sorties = candidates.map((candidate) => ({
      mission: candidate,
      analysis: analyzeMission(C, candidate, payload, { surveyPhotos }),
    }));
    sortieIndex = 0;
    mission = sorties[0].mission;
    analysis = sorties[0].analysis;
    cachedSample = undefined;
    cachedTime = undefined;
    installMission();
    const blocked = sorties.some((x) => x.analysis.blocked);
    setState(blocked ? "blocked" : "ready");
    ui.setSorties(sorties.length, sortieIndex);
    ui.setMission(mission, analysis);
    const failed = sorties.flatMap((x, k) =>
      x.analysis.checks
        .filter((c) => c.blocking && !c.ok)
        .map((c) => `${sorties.length > 1 ? `uçuş ${k + 1}: ` : ""}${c.label.toLocaleLowerCase("tr")}`),
    );
    const plan = analysis.planCoverage,
      photos = sorties.reduce((sum, x) => sum + x.analysis.planCoverage.count, 0),
      lowest = Math.min(...sorties.map((x) => x.mission.nominal.simulation.metrics.finalSoc));
    ui.message(
      (failed.length
        ? `Görev onaylanmadı: ${failed.join(", ")}. Planı inceleyip parametreleri değiştirin.`
        : `Görev onaylandı: ${sorties.length > 1 ? `${sorties.length} uçuş (batarya değişimiyle), ` : ""}${mission.survey ? `${mission.survey.count} hat, ` : ""}${photos} fotoğraf, ortalama GSD ${(plan.gsdMean * 100).toFixed(1)} cm${mission.survey ? `, alanın %${(plan.areaCovered * 100).toFixed(1)}'i` : ""}, inişte batarya ${sorties.length > 1 ? "en az " : ""}%${Math.round(lowest * 100)}.`) +
        (scenarioError
          ? ` Arıza senaryosu çözülemedi: ${scenarioError}`
          : faultSummary(mission)),
      failed.length > 0 || Boolean(scenarioError),
    );
    setCamera(cameraMode);
    render();
  } catch (error) {
    if (id !== runId) return;
    setState("error");
    ui.message(error.message || "Uçuş planı hazırlanamadı.", true);
    ui.elements.mapError.textContent =
      error.message || "Arazi verisi alınamadı.";
    ui.elements.mapError.hidden = false;
    if (!provider) ui.elements.ionToken.closest("details").open = true;
    // Do not print service errors: request URLs can contain access tokens.
  }
}

// Where the flown solve landed, for messages.
function landingText(metrics) {
  return metrics.landingSite === "field"
    ? `evden ${formatDistance(metrics.homeDistance)} uzakta yerinde iniş`
    : metrics.landingSite === "rally"
      ? `rally noktasına (${metrics.landingName}) iniş`
      : "evde iniş";
}
// One-line outcome of injected faults for the status message.
function faultSummary(m) {
  const f = m.simulation.failsafe;
  if (!f?.faults?.length) return "";
  const actions = f.events
    .filter((e) => e.label.startsWith("FAILSAFE"))
    .map((e) => `${e.label.replace("FAILSAFE · ", "")} ${formatTime(e.time)}`);
  const metrics = m.simulation.metrics;
  return ` Arıza senaryosu: ${f.faults.map((x) => `${FAULTS[x.type].label} ${formatTime(x.time)}`).join(" + ")}${actions.length ? ` → ${actions.join(" → ")}` : " → failsafe eylemi gerekmedi"}; ${landingText(metrics)}, batarya %${Math.round(metrics.finalSoc * 100)}.`;
}

// Show another sortie of a multi-flight survey: its plan, flight and
// checklist, from the pad.
function selectSortie(index) {
  if (!sorties[index] || index === sortieIndex || currentState === "loading")
    return;
  viewer.clock.shouldAnimate = false;
  sortieIndex = index;
  mission = sorties[index].mission;
  analysis = sorties[index].analysis;
  cachedSample = undefined;
  cachedTime = undefined;
  clearMission();
  installMission();
  setState(analysis.blocked ? "blocked" : "ready");
  ui.setSorties(sorties.length, sortieIndex);
  ui.setMission(mission, analysis);
  ui.message(
    `Uçuş ${index + 1}/${sorties.length}: ${mission.survey.lines.length} hat, ${analysis.planCoverage.count} fotoğraf, inişte batarya %${Math.round(mission.nominal.simulation.metrics.finalSoc * 100)}.`,
  );
  setCamera(cameraMode);
  render();
}

function clearMission() {
  missionEntities.forEach((entity) => viewer.entities.remove(entity));
  missionEntities = [];
  footprints?.entities.removeAll();
  obstacleLayer?.entities.removeAll();
}

// Operator command during playback. The flight is solved in advance and
// deterministic, so the command is applied by re-solving it with the
// command at the current mission time: up to that moment the new solve is
// identical, from it the autopilot obeys. Playback resumes where it was.
let operatorBusy = false;
async function operatorCommand(action) {
  if (!mission?.flight || operatorBusy || !["running", "paused"].includes(currentState))
    return;
  const flight = mission.flight,
    seconds = elapsed(),
    wasRunning = currentState === "running";
  if (seconds >= mission.duration - 1) return;
  operatorBusy = true;
  viewer.clock.shouldAnimate = false;
  ui.message(`Operatör komutu (${formatTime(seconds)}): uçuş bu andan itibaren yeniden çözülüyor…`);
  const id = runId,
    commands = [
      ...flight.commands.filter((c) => c.time < seconds),
      { time: seconds, action },
    ];
  try {
    const [simulation] = await solveDynamics([
      {
        reference: mission.reference,
        options: {
          ...flight.options,
          ...flight.armed,
          faults: flight.faults,
          commands,
        },
      },
    ]);
    if (id !== runId) return;
    flight.commands = commands;
    const nominal = mission.nominal.simulation;
    attachSimulation(mission, simulation, nominal);
    analysis = analyzeMission(
      C,
      mission,
      {
        gimbalPitch: analysis.capture.pitchDegrees,
        frontOverlap: analysis.capture.overlap,
      },
      { surveyPhotos },
    );
    sorties[sortieIndex].analysis = analysis;
    cachedSample = undefined;
    cachedTime = undefined;
    clearMission();
    installMission();
    ui.setMission(mission, analysis);
    viewer.clock.currentTime = C.JulianDate.addSeconds(
      startTime,
      seconds,
      new C.JulianDate(),
    );
    setState(wasRunning ? "running" : "paused");
    viewer.clock.shouldAnimate = wasRunning;
    const event = simulation.failsafe.events.findLast((e) => e.kind === "operator");
    ui.message(
      `${event?.label ?? "Operatör komutu"} · ${formatTime(seconds)} — ${event?.detail ?? ""}; ${landingText(simulation.metrics)} bekleniyor, batarya %${Math.round(simulation.metrics.finalSoc * 100)}.`,
    );
    render();
  } catch (error) {
    if (id !== runId) return;
    viewer.clock.shouldAnimate = wasRunning;
    ui.message(`Operatör komutu uygulanamadı: ${error.message}`, true);
  } finally {
    operatorBusy = false;
  }
}

function installMission() {
  startTime = C.JulianDate.fromIso8601("2026-06-21T09:00:00Z");
  const stop = C.JulianDate.addSeconds(
    startTime,
    mission.duration,
    new C.JulianDate(),
  );
  Object.assign(viewer.clock, {
    startTime: startTime.clone(),
    stopTime: stop,
    currentTime: startTime.clone(),
    clockRange: C.ClockRange.CLAMPED,
    clockStep: C.ClockStep.SYSTEM_CLOCK_MULTIPLIER,
    multiplier: Number(ui.elements.playbackSpeed.value),
    shouldAnimate: false,
  });
  const add = (options) => {
    const entity = viewer.entities.add(options);
    missionEntities.push(entity);
    return entity;
  };
  const position = createPositionProperty(C, (time, result) =>
    C.Cartesian3.clone(sampleAt(time).cartesian, result),
  );
  const frameRotation = C.Quaternion.fromRotationMatrix(
    C.Matrix4.getMatrix3(mission.frame, new C.Matrix3()),
  );
  const orientation = new C.CallbackProperty((time, result) => {
    const q = sampleAt(time).q;
    return C.Quaternion.multiply(
      frameRotation,
      new C.Quaternion(q[1], q[2], q[3], q[0]),
      result || new C.Quaternion(),
    );
  }, false);
  const { armLength } = mission.simulation.airframe;
  const nodeTransformations = {
    // Drop the model so its base plate rests on the pad at touchdown; the
    // entity position itself stays the physical CG.
    [MODEL_ROOT_NODE]: new C.TranslationRotationScale(
      new C.Cartesian3(
        0,
        modelGroundOffset(armLength, mission.settings.landingOffset),
        0,
      ),
    ),
  };
  // Rotor nodes spin about their local +Y (glTF up). A positive angle about
  // +Y is counter-clockwise seen from above, so clockwise (SPIN +1) is negated.
  ROTOR_NODES.forEach((name, i) => {
    nodeTransformations[name] = new C.CallbackProperty(
      (time) =>
        new C.TranslationRotationScale(
          undefined,
          C.Quaternion.fromAxisAngle(
            C.Cartesian3.UNIT_Y,
            -SPIN[i] * sampleAt(time).rotorAngles[i],
          ),
        ),
      false,
    );
  });
  drone = add({
    name: "Keşif drone",
    position,
    orientation,
    model: {
      // The payload camera hangs under the airframe; hide the model in the
      // gimbal view instead of rendering it from inside.
      show: new C.CallbackProperty(() => cameraMode !== "gimbal", false),
      uri: new URL("./drone.glb", document.baseURI).href,
      scale: modelScale(armLength),
      minimumPixelSize: 0,
      runAnimations: false,
      nodeTransformations,
    },
    point: {
      pixelSize: 7,
      color: C.Color.fromCssColorString("#b5ed8b"),
      outlineColor: C.Color.BLACK,
      outlineWidth: 1,
      distanceDisplayCondition: new C.DistanceDisplayCondition(80, 20000),
    },
  });
  const world = (p) =>
    C.Matrix4.multiplyByPoint(
      mission.frame,
      new C.Cartesian3(...p),
      new C.Cartesian3(),
    );
  const path = [
    world([0, 0, 0]),
    ...mission.path.table.map((k, i) => world([...k.p, mission.profile[i]])),
    world([0, 0, 0]),
  ];
  add({
    name: "Rota referansı",
    polyline: {
      positions: path,
      width: 2,
      arcType: C.ArcType.NONE,
      material: C.Color.fromCssColorString("#b5ed8b").withAlpha(0.7),
    },
  });
  const log = mission.simulation,
    trace = [];
  for (let i = 0; i < log.rows; i += 15)
    trace.push(
      world(Array.from(log.data.slice(i * STRIDE + 1, i * STRIDE + 4))),
    );
  add({
    name: "Gerçekleşen iz",
    polyline: {
      positions: new C.CallbackProperty((time) => {
        const index = Math.floor(Math.max(0, elapsed(time)) * 2),
          start = Math.max(0, index - 240);
        return trace.slice(start, Math.min(trace.length, index + 1));
      }, false),
      width: 3,
      arcType: C.ArcType.NONE,
      material: C.Color.fromCssColorString("#7bdad1"),
    },
  });
  if (mission.survey) {
    // Survey polygon outline draped on the terrain.
    const ring = mission.survey.vertices.concat([mission.survey.vertices[0]]);
    add({
      name: "Haritalama alanı",
      polyline: {
        positions: ring.map((v) =>
          C.Cartesian3.fromDegrees(v.longitude, v.latitude),
        ),
        width: 2,
        clampToGround: true,
        material: C.Color.fromCssColorString("#ffd27a").withAlpha(0.85),
      },
    });
  }
  const failsafe = mission.simulation.failsafe;
  if (failsafe?.rtl) {
    // The direct return leg the failsafe planned over the elevation model.
    add({
      name: "Failsafe dönüş bacağı",
      polyline: {
        positions: failsafe.rtl.points.map((p, i) =>
          world([p[0], p[1], failsafe.rtl.profile[i]]),
        ),
        width: 2,
        arcType: C.ArcType.NONE,
        material: new C.PolylineDashMaterialProperty({
          color: C.Color.fromCssColorString("#ff8f7a"),
          dashLength: 12,
        }),
      },
    });
  }
  // Obstacles within 40 m of the planned path as prisms on the modelled
  // ground (forest canopy is left to the imagery), and the one that set the
  // lowest clearance of the flown trace.
  const groundAt = (xy) => toGeodetic(C, mission, [xy[0], xy[1], terrainAt(mission, xy).height]).height,
    lonLat = (xy) => {
      const g = toGeodetic(C, mission, [xy[0], xy[1], 0]);
      return [g.longitude, g.latitude];
    };
  const OBSTACLE_COLORS = {
    building: "#d8c3a5",
    mast: "#ff8f7a",
    tower: "#ff8f7a",
    tree: "#7fb069",
    tree_row: "#7fb069",
    other: "#d8c3a5",
  };
  for (const { obstacle: o, nearest, ground } of obstaclesAlongPath(mission)) {
    if (o.kind === "forest") continue;
    const height = toGeodetic(C, mission, [nearest[0], nearest[1], ground]).height,
      base = height - 2,
      top = height + o.height,
      material = C.Color.fromCssColorString(OBSTACLE_COLORS[o.kind] ?? "#d8c3a5").withAlpha(0.8),
      name = `Engel · ${obstacleText(o)}`;
    if (o.shape === "polygon")
      obstacleLayer.entities.add({
        name,
        polygon: {
          hierarchy: C.Cartesian3.fromDegreesArray(o.xy.flatMap(lonLat)),
          height: base,
          extrudedHeight: top,
          material,
          outline: false,
        },
      });
    else if (o.shape === "line")
      obstacleLayer.entities.add({
        name,
        corridor: {
          positions: C.Cartesian3.fromDegreesArray(o.xy.flatMap(lonLat)),
          width: 2 * o.radius,
          height: base,
          extrudedHeight: top,
          material,
        },
      });
    else {
      const [lon, lat] = lonLat(o.xy[0]);
      obstacleLayer.entities.add({
        name,
        position: C.Cartesian3.fromDegrees(lon, lat, (base + top) / 2),
        cylinder: {
          length: top - base,
          topRadius: Math.max(0.5, o.radius / 3),
          bottomRadius: Math.max(1, o.radius),
          material,
        },
      });
    }
  }
  const closest = mission.obstacleClearance;
  if (closest?.obstacle && closest.nearest) {
    const [lon, lat] = lonLat(closest.nearest);
    obstacleLayer.entities.add({
      name: "En düşük engel açıklığı",
      position: C.Cartesian3.fromDegrees(lon, lat, groundAt(closest.nearest) + closest.obstacle.height),
      point: { pixelSize: 7, color: C.Color.fromCssColorString("#ff8f7a"), disableDepthTestDistance: 15000 },
      label: {
        text: `▲ ${obstacleText(closest.obstacle)} · açıklık ${Math.round(closest.clearance)} m`,
        font: "11px sans-serif",
        pixelOffset: new C.Cartesian2(0, -14),
        fillColor: C.Color.WHITE,
        showBackground: true,
        backgroundColor: C.Color.fromCssColorString("#2b1d16").withAlpha(0.85),
        disableDepthTestDistance: 15000,
        distanceDisplayCondition: new C.DistanceDisplayCondition(30, 6000),
      },
    });
  }
  // Rally points: safe landing sites a return may choose.
  for (const point of mission.rally ?? []) {
    const ground = C.Cartographic.fromDegrees(point.longitude, point.latitude);
    add({
      name: `Rally · ${point.name}`,
      position: C.Cartesian3.fromRadians(ground.longitude, ground.latitude),
      point: {
        pixelSize: 10,
        color: C.Color.fromCssColorString("#ffd27a"),
        outlineColor: C.Color.BLACK,
        outlineWidth: 1,
        heightReference: C.HeightReference.CLAMP_TO_GROUND,
        disableDepthTestDistance: 15000,
      },
      label: {
        text: `R  ${point.name}`,
        font: "bold 11px sans-serif",
        pixelOffset: new C.Cartesian2(0, -18),
        fillColor: C.Color.WHITE,
        showBackground: true,
        backgroundColor: C.Color.fromCssColorString("#101b20").withAlpha(0.85),
        heightReference: C.HeightReference.CLAMP_TO_GROUND,
        disableDepthTestDistance: 15000,
        distanceDisplayCondition: new C.DistanceDisplayCondition(30, 15000),
      },
    });
  }
  // Fault and failsafe events where they happened in the flown solve.
  for (const event of failsafe?.events ?? []) {
    if (event.kind === "info") continue;
    const state = evaluateMission(C, mission, event.time);
    add({
      name: event.label,
      position: state.cartesian,
      availability: new C.TimeIntervalCollection([
        new C.TimeInterval({
          start: C.JulianDate.addSeconds(startTime, event.time, new C.JulianDate()),
          stop,
        }),
      ]),
      point: {
        pixelSize: 9,
        color: C.Color.fromCssColorString(
          event.kind === "fault"
            ? "#ff8f7a"
            : event.kind === "operator"
              ? "#8fd3ff"
              : "#ffd27a",
        ),
        outlineColor: C.Color.BLACK,
        outlineWidth: 1,
        disableDepthTestDistance: 15000,
      },
      // The fault's label above its marker, the failsafe's below, so the
      // two stay readable when they happen close together.
      label: {
        text: event.label
          .replace("FAILSAFE · ", "")
          .replace("ARIZA · ", "⚠ ")
          .replace("OPERATÖR · ", "◆ "),
        font: "bold 11px sans-serif",
        verticalOrigin:
          event.kind === "fault"
            ? C.VerticalOrigin.BOTTOM
            : C.VerticalOrigin.TOP,
        pixelOffset: new C.Cartesian2(0, event.kind === "fault" ? -10 : 10),
        fillColor: C.Color.WHITE,
        showBackground: true,
        backgroundColor: C.Color.fromCssColorString("#3a1d18").withAlpha(0.85),
        disableDepthTestDistance: 15000,
        distanceDisplayCondition: new C.DistanceDisplayCondition(30, 12000),
      },
    });
  }
  // Image footprints draped on the terrain; each appears when its photo is
  // taken and stays until the end of the mission.
  const footprintColor = C.Color.fromCssColorString("#7bdad1").withAlpha(0.09);
  for (const photo of analysis.capture.photos) {
    const corners = photo.corners.map((c) => {
      const g = toGeodetic(C, mission, c);
      return C.Cartesian3.fromDegrees(g.longitude, g.latitude);
    });
    footprints.entities.add({
      name: `Görüntü izi ${photo.index}`,
      availability: new C.TimeIntervalCollection([
        new C.TimeInterval({
          start: C.JulianDate.addSeconds(startTime, photo.time, new C.JulianDate()),
          stop,
        }),
      ]),
      polygon: {
        hierarchy: corners,
        material: footprintColor,
        classificationType: C.ClassificationType.TERRAIN,
      },
    });
  }
  const home = C.Cartographic.clone(mission.home);
  home.height += 0.015;
  add({
    name: "Kalkış / iniş noktası",
    position: toCartesian(home),
    ellipse: {
      semiMajorAxis: 2,
      semiMinorAxis: 2,
      height: home.height,
      material: C.Color.fromCssColorString("#b5ed8b").withAlpha(0.55),
    },
    label: {
      text: "H  KALKIŞ / İNİŞ",
      font: "bold 12px sans-serif",
      pixelOffset: new C.Cartesian2(0, -24),
      fillColor: C.Color.WHITE,
      showBackground: true,
      backgroundColor: C.Color.fromCssColorString("#101b20").withAlpha(0.85),
      disableDepthTestDistance: 15000,
      // Hide the label at close follow ranges (1.2–5 m) so it does not cover
      // the drone model, and beyond 15 km where it would be unreadable anyway.
      distanceDisplayCondition: new C.DistanceDisplayCondition(6, 15000),
    },
  });
  // Useful for local inspection and numerical/browser regression checks.
  window.droneSimulation = Object.freeze({
    get mission() {
      return mission;
    },
    get analysis() {
      return analysis;
    },
    get state() {
      return currentState;
    },
    get drone() {
      return drone;
    },
    get cameraMode() {
      return cameraMode;
    },
    get followDistance() {
      return { target: targetFollowRange, current: followRange };
    },
    get followView() {
      return followView;
    },
    evaluate: (seconds) => evaluateMission(C, mission, seconds),
  });
}

function boot() {
  if (!C) {
    setState("error");
    ui.message("3D harita motoru yüklenemedi. Sayfayı yeniden yükleyin.", true);
    return;
  }
  try {
    viewer = new C.Viewer("cesiumContainer", {
      baseLayer: new C.ImageryLayer(
        new C.UrlTemplateImageryProvider({
          url: "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
          // Level 18 (≈0.45 m/px at 40° N) is the finest Esri serves over
          // Uzungöl; deeper levels return a "map data not yet available" tile.
          maximumLevel: 18,
          credit:
            "Tiles © Esri — Source: Esri, Maxar, Earthstar Geographics, and the GIS User Community",
        }),
      ),
      animation: false,
      timeline: false,
      baseLayerPicker: false,
      geocoder: false,
      homeButton: false,
      sceneModePicker: false,
      navigationHelpButton: false,
      fullscreenButton: false,
      infoBox: false,
      selectionIndicator: false,
      shouldAnimate: false,
      shadows: false,
    });
    window.viewer = viewer;
    footprints = new C.CustomDataSource("footprints");
    viewer.dataSources.add(footprints);
    obstacleLayer = new C.CustomDataSource("obstacles");
    viewer.dataSources.add(obstacleLayer);
    viewer.scene.globe.depthTestAgainstTerrain = true;
    viewer.scene.globe.enableLighting = false;
    // A coarse pointer marks touch/mobile devices: a looser tile error keeps
    // terrain detail affordable on their weaker GPUs and mobile bandwidth.
    const coarsePointer =
      globalThis.matchMedia?.("(pointer: coarse)")?.matches === true;
    viewer.scene.globe.maximumScreenSpaceError = coarsePointer ? 2 : 1.5;
    viewer.scene.screenSpaceCameraController.minimumZoomDistance = 1;
    initializeFollowZoom();
    viewer.scene.camera.setView({
      destination: C.Cartesian3.fromDegrees(40.3, 40.603, 2900),
      orientation: { heading: 0, pitch: C.Math.toRadians(-48), roll: 0 },
    });
    viewer.screenSpaceEventHandler.removeInputAction(
      C.ScreenSpaceEventType.LEFT_DOUBLE_CLICK,
    );
    // Exactly one lifetime tick listener; replays never add another listener.
    viewer.clock.onTick.addEventListener(() => {
      const now = performance.now();
      if (
        mission &&
        (now - lastUiUpdate > 90 || elapsed() >= mission.duration)
      ) {
        render();
        lastUiUpdate = now;
      }
    });
    viewer.scene.preRender.addEventListener(() => {
      const now = performance.now(),
        dt = Math.min(0.1, (now - lastCameraFrame) / 1000);
      lastCameraFrame = now;
      if (!mission?.simulation || cameraMode === "overview") return;
      const sample = sampleAt(viewer.clock.currentTime);
      if (cameraMode === "gimbal") {
        // Stabilised gimbal: heading follows the airframe, pitch is the
        // payload setting, roll is held level.
        viewer.camera.setView({
          destination: sample.cartesian,
          orientation: {
            heading: sample.heading,
            pitch: C.Math.toRadians(analysis.capture.pitchDegrees),
            roll: 0,
          },
        });
        return;
      }
      const preset = FOLLOW_VIEWS[followView];
      const targetHeading = sample.heading + preset.bearing;
      const delta = Math.atan2(
        Math.sin(targetHeading - followHeading),
        Math.cos(targetHeading - followHeading),
      );
      followHeading += delta * (1 - Math.exp(-dt / 0.65));
      followPitch += (preset.pitch - followPitch) * (1 - Math.exp(-dt / 0.4));
      followRange +=
        (targetFollowRange - followRange) * (1 - Math.exp(-dt / 0.3));
      viewer.camera.lookAt(
        sample.cartesian,
        new C.HeadingPitchRange(followHeading, followPitch, followRange),
      );
    });
    viewer.scene.renderError.addEventListener(() => {
      viewer.clock.shouldAnimate = false;
      ui.elements.mapError.textContent =
        "3D görüntü oluşturulamadı. WebGL desteğini kontrol edip sayfayı yeniden yükleyin.";
      ui.elements.mapError.hidden = false;
    });
    prepareMission();
  } catch {
    setState("error");
    ui.message(
      "3D harita başlatılamadı. Tarayıcınızda WebGL desteğini kontrol edin.",
      true,
    );
    ui.elements.mapError.textContent = ui.elements.message.textContent;
    ui.elements.mapError.hidden = false;
  }
}
boot();
