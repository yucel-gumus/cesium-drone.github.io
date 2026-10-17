/* global Cesium */
import {
  createTerrainRoute,
  sampleTerrainRoute,
  buildMission,
  evaluateMission,
  attachSimulation,
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
import { AIRFRAME, SPIN, STRIDE } from "./quadrotor.mjs";
import { analyzeMission, toGeodetic } from "./mission-analysis.mjs";
import { groundSampleDistance, fieldOfView } from "./payload.mjs";
import { toCSV, toKML, toQGCPlan, toGeoJSON } from "./exports.mjs";

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
// Mean-wind authority limit of the 2 kg profile (m/s), the same as windInput
// max in index.html: the highest 0.5 m/s step at which the full mission keeps
// its tracking error under 1 m for every turbulence seed tried. See README.
const WIND_LIMIT = 3.5;
let viewer, mission, analysis, drone, provider;
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
// Sampled terrain heights for the current waypoint file and terrain provider.
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
      "Uçuş başladı. Drone araziyi sabit AGL'de izliyor, kamera mesafeye göre tetikleniyor.",
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
  rate(multiplier) {
    if (viewer) viewer.clock.multiplier = multiplier;
  },
  camera: setCamera,
  followView: setFollowView,
  followDistance: setFollowDistance,
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
    };
    if (!files[kind]) return;
    const [content, type, extension] = files[kind]();
    const url = URL.createObjectURL(new Blob([content], { type }));
    const a = document.createElement("a");
    a.href = url;
    a.download = `uzungol-kesif.${extension}`;
    a.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  },
});

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
    ui.message("Keşif tamamlandı. Drone kalktığı noktaya indi.");
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

function solveDynamics(reference, options) {
  cancelSolve?.();
  const worker = new Worker(
    new URL("./simulation-worker.mjs", import.meta.url),
  );
  return withTimeout(
    new Promise((resolve, reject) => {
      // A newer plan supersedes this solve: stop the worker and settle now
      // instead of leaving the promise pending until the timeout.
      cancelSolve = Object.assign(
        () => {
          worker.terminate();
          reject(new Error("Fizik çözümü yeni bir planla iptal edildi."));
        },
        { worker },
      );
      worker.onmessage = ({ data }) => {
        if (data.error) reject(new Error(data.error));
        else if (data.simulation) resolve(data.simulation);
        else if (currentState === "loading")
          ui.message(
            `240 Hz uçuş dinamiği çözülüyor · %${Math.round(data.progress * 100)}`,
          );
      };
      worker.onerror = () =>
        reject(
          new Error("Fizik işlemcisi başlatılamadı. Sayfayı yeniden yükleyin."),
        );
      worker.postMessage({ reference, options });
    }),
    90000,
    "Fizik çözümü zaman aşımına uğradı.",
  ).finally(() => {
    worker.terminate();
    if (cancelSolve && worker === cancelSolve.worker) cancelSolve = undefined;
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
  cachedSample = undefined;
  missionEntities.forEach((entity) => viewer.entities.remove(entity));
  missionEntities = [];
  footprints?.entities.removeAll();
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
    const mass = Number(ui.elements.massInput.value),
      wind = Number(ui.elements.windInput.value),
      seed = Number(ui.elements.seedInput.value),
      turbulence = ui.elements.turbulenceInput.checked,
      gimbalPitch = Number(ui.elements.gimbalInput.value),
      frontOverlap = Number(ui.elements.overlapInput.value) / 100,
      gnss = ui.elements.gnssInput.value;
    if (
      !Number.isFinite(mass) ||
      mass < 1 ||
      mass > 3 ||
      !Number.isFinite(wind) ||
      wind < 0 ||
      wind > WIND_LIMIT
    )
      throw new Error(
        `Kütle 1–3 kg, rüzgâr 0–${WIND_LIMIT.toLocaleString("tr")} m/s arasında olmalı.`,
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
    const route = createTerrainRoute(C, JSON.parse(waypointText), {
      agl,
      wind,
    });
    // Sample positions depend only on the waypoints and corridor constants, so
    // re-planning with another height, mass, wind or payload reuses them.
    if (
      terrainCache?.provider === provider &&
      terrainCache.waypointText === waypointText &&
      terrainCache.heights.length === route.samples.length
    ) {
      route.samples.forEach((p, i) => (p.height = terrainCache.heights[i]));
    } else {
      await withTimeout(
        sampleTerrainRoute(C, provider, route, (progress) => {
          if (id === runId && currentState === "loading")
            ui.message(
              `Arazi koridoru örnekleniyor · %${Math.round(progress * 100)}`,
            );
        }),
        90000,
        "Arazi örneklemesi tamamlanamadı. Yeniden hesaplayın veya erişim tokenını kontrol edin.",
      );
      terrainCache = {
        provider,
        waypointText,
        heights: route.samples.map((p) => p.height),
      };
    }
    if (id !== runId) return;
    const candidate = buildMission(C, route);
    const airframe = {
      ...AIRFRAME,
      mass,
      inertia: AIRFRAME.inertia.map((x) => (x * mass) / 2),
    };
    const simulation = await solveDynamics(candidate.reference, {
      airframe,
      wind,
      seed,
      turbulence,
      gnss,
    });
    if (id !== runId) return;
    attachSimulation(candidate, simulation);
    const result = analyzeMission(C, candidate, { gimbalPitch, frontOverlap });
    mission = candidate;
    analysis = result;
    cachedSample = undefined;
    cachedTime = undefined;
    installMission();
    setState(analysis.blocked ? "blocked" : "ready");
    ui.setMission(mission, analysis);
    const failed = analysis.checks.filter((c) => c.blocking && !c.ok);
    ui.message(
      failed.length
        ? `Görev onaylanmadı: ${failed.map((c) => c.label.toLocaleLowerCase("tr")).join(", ")}. Planı inceleyip parametreleri değiştirin.`
        : `Görev onaylandı: ${analysis.coverage.count} fotoğraf, ortalama GSD ${(analysis.coverage.gsdMean * 100).toFixed(1)} cm, inişte batarya %${Math.round(simulation.metrics.finalSoc * 100)}.`,
      failed.length > 0,
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
