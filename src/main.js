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
import "./style.css";
import { AIRFRAME, SPIN, STRIDE } from "./quadrotor.mjs";

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
let viewer, mission, drone, provider;
let currentState = "loading";
let cameraMode = "overview";
let runId = 0;
let startTime;
let lastUiUpdate = 0;
let missionEntities = [];
let activeWorker;
let cachedSample, cachedTime;
let followHeading = 0,
  followPitch = FOLLOW_VIEWS.rear.pitch,
  followView = "rear",
  followRange = 8,
  targetFollowRange = 8,
  lastCameraFrame = 0;

const ui = new DroneUIController({
  start() {
    if (!mission || currentState === "running" || currentState === "paused")
      return;
    viewer.clock.currentTime = startTime.clone();
    setState("running");
    viewer.clock.shouldAnimate = true;
    setCamera("follow");
    ui.message("Uçuş başladı. Drone araziye göre planlanan rotayı izliyor.");
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
    if (!mission) return;
    viewer.clock.shouldAnimate = false;
    viewer.clock.currentTime = startTime.clone();
    setState("ready");
    ui.message("Kalkış noktasında hazır. Keşif uçuşunu başlatabilirsiniz.");
    render();
  },
  plan(clearance) {
    prepareMission(clearance);
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
    prepareMission(Number(ui.elements.clearance.value), token);
  },
  seek(progress) {
    if (!mission) return;
    viewer.clock.shouldAnimate = false;
    viewer.clock.currentTime = C.JulianDate.addSeconds(
      startTime,
      progress * mission.duration,
      new C.JulianDate(),
    );
    setState(progress >= 1 ? "complete" : "paused");
    render();
  },
  rate(multiplier) {
    if (viewer) viewer.clock.multiplier = multiplier;
  },
  camera: setCamera,
  followView: setFollowView,
  followDistance: setFollowDistance,
  export() {
    if (!mission?.simulation) return;
    const log = mission.simulation;
    const lines = [
      "time_s,east_m,north_m,up_m,v_e_ms,v_n_ms,v_u_ms,qw,qx,qy,qz,p_rads,q_rads,r_rads,a_e_ms2,a_n_ms2,a_u_ms2,m1_rads,m2_rads,m3_rads,m4_rads,thrust_N,torque_x_Nm,torque_y_Nm,torque_z_Nm,wind_e_ms,wind_n_ms,wind_u_ms,distance_m,tracking_error_m,phase_code,motor_saturated",
    ];
    for (let i = 0; i < log.rows; i += 3)
      lines.push(
        Array.from(log.data.slice(i * STRIDE, i * STRIDE + 32))
          .map((x) => x.toFixed(6))
          .join(","),
      );
    const url = URL.createObjectURL(
      new Blob([lines.join("\n")], { type: "text/csv;charset=utf-8" }),
    );
    const a = document.createElement("a");
    a.href = url;
    a.download = "uzungol-flight-log.csv";
    a.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  },
});

function setState(state) {
  currentState = state;
  ui.setState(state);
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
  ui.render(sample, C);
}

function setCamera(mode) {
  if (!viewer || !["overview", "follow"].includes(mode)) return;
  if (mode === "follow" && cameraMode === "follow") return;
  viewer.camera.cancelFlight();
  cameraMode = mode;
  viewer.trackedEntity = undefined;
  viewer.camera.lookAtTransform(C.Matrix4.IDENTITY);
  ui.setCamera(mode);
  viewer.scene.screenSpaceCameraController.enableZoom = mode !== "follow";
  viewer.camera.frustum.near = mode === "follow" ? 0.1 : 1;
  if (!mission) return;
  if (mode === "follow") {
    followHeading =
      sampleAt(viewer.clock.currentTime).heading + FOLLOW_VIEWS[followView].bearing;
    followPitch = FOLLOW_VIEWS[followView].pitch;
    followRange = 80;
  } else if (mode === "overview") {
    const bounds = C.BoundingSphere.fromPoints(
      mission.path.table.map((k) =>
        C.Matrix4.multiplyByPoint(
          mission.frame,
          new C.Cartesian3(k.p[0], k.p[1], mission.reference.cruiseHeight),
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
  activeWorker?.terminate();
  const worker = new Worker(
    new URL("./simulation-worker.mjs", import.meta.url),
  );
  activeWorker = worker;
  return withTimeout(
    new Promise((resolve, reject) => {
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
    if (activeWorker === worker) activeWorker = undefined;
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

async function prepareMission(clearance = 70, overrideToken) {
  if (!viewer) return;
  if (!Number.isFinite(clearance) || clearance < 30 || clearance > 150) {
    ui.message("Arazi açıklığı 30–150 metre arasında olmalı.", true);
    return;
  }
  const id = ++runId;
  setState("loading");
  viewer.clock.shouldAnimate = false;
  viewer.trackedEntity = undefined;
  activeWorker?.terminate();
  mission = undefined;
  cachedSample = undefined;
  missionEntities.forEach((entity) => viewer.entities.remove(entity));
  missionEntities = [];
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
      provider = await withTimeout(
        C.createWorldTerrainAsync({ requestVertexNormals: true }),
        25000,
        "Arazi servisi yanıt vermedi. Bağlantınızı veya tokenınızı kontrol edin.",
      );
      viewer.terrainProvider = provider;
    }
    const response = await fetch(new URL("./points.json", document.baseURI));
    if (!response.ok)
      throw new Error(`Rota dosyası yüklenemedi (HTTP ${response.status}).`);
    const mass = Number(ui.elements.massInput.value),
      wind = Number(ui.elements.windInput.value);
    if (
      !Number.isFinite(mass) ||
      mass < 1 ||
      mass > 3 ||
      !Number.isFinite(wind) ||
      wind < 0 ||
      wind > 6
    )
      throw new Error("Kütle 1–3 kg, rüzgâr 0–6 m/s arasında olmalı.");
    const route = createTerrainRoute(C, await response.json(), {
      clearance,
      wind,
    });
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
    });
    if (id !== runId) return;
    attachSimulation(candidate, simulation);
    mission = candidate;
    cachedSample = undefined;
    cachedTime = undefined;
    installMission();
    setState("ready");
    ui.setMission(mission);
    ui.message(
      `Dinamik uçuş doğrulandı. Keşifte en az ${mission.minCruiseClearance.toFixed(1)} m model açıklığı.`,
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
  const rotorNames = [
    "PropClockwiseFront_low",
    "PropClockwiseBack_low",
    "PropCounterClockwiseBack_low",
    "PropCounterClockwiseFront_low",
  ];
  const nodeTransformations = {};
  rotorNames.forEach((name, i) => {
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
      uri: new URL("./drone.glb", document.baseURI).href,
      scale: (2 * mission.simulation.airframe.armLength) / Math.sqrt(2) / 3.4,
      minimumPixelSize: 0,
      shadows: C.ShadowMode.ENABLED,
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
    ...mission.path.table.map((k) =>
      world([...k.p, mission.reference.cruiseHeight]),
    ),
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
    },
  });
  // Useful for local inspection and numerical/browser regression checks.
  window.droneSimulation = Object.freeze({
    get mission() {
      return mission;
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
          maximumLevel: 17,
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
    viewer.scene.globe.depthTestAgainstTerrain = true;
    viewer.scene.globe.enableLighting = false;
    viewer.scene.globe.maximumScreenSpaceError = 1.5;
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
      if (!mission?.simulation || cameraMode !== "follow") return;
      const sample = sampleAt(viewer.clock.currentTime);
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
