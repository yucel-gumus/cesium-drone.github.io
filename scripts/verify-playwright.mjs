// Browser regression for the production build, driven by Playwright
// Chromium against real Cesium World Terrain. Serve dist/ first
// (`npm run preview`); this script never starts a server, and no credentials
// are written to the console or to the report.
import fs from "node:fs/promises";
import path from "node:path";
import assert from "node:assert/strict";
import { createRequire } from "node:module";
import { chromium } from "playwright";

const require = createRequire(import.meta.url);
const base =
  process.env.PREVIEW_URL || "http://127.0.0.1:9001/cesium-drone.github.io/";
const artifacts = path.resolve("artifacts");
const TOKEN_KEY = "uzungol-ion-token";
// Mean-wind authority limit of the 2 kg profile, as in src/index.html and
// src/main.js prepareMission(); see README "Kütleye bağlı rüzgâr yetki sınırı".
const WIND_LIMIT = "3.5";
// Inputs that must stay locked while a plan is solved or a flight runs.
const ENVIRONMENT_FIELDS = [
  "aglInput",
  "massInput",
  "windInput",
  "seedInput",
  "turbulenceInput",
  "gimbalInput",
  "overlapInput",
  "gnssInput",
  "missionTypeInput",
  "faultInput",
  "secondFaultInput",
  "rtlTargetInput",
  "calibrationFile",
  "planButton",
];
// Inputs that only apply to an area survey or with a fault scenario.
const DEPENDENT_FIELDS = ["sideOverlapInput", "faultTimeInput", "secondFaultTimeInput"];
const LOCKED = Array(ENVIRONMENT_FIELDS.length).fill(true),
  UNLOCKED = Array(ENVIRONMENT_FIELDS.length).fill(false);
// Terrain corridor sampling plus the 240 Hz physics solve may take this long.
const READY_TIMEOUT = 100000;
// Drone-relative bearing and pitch of each follow preset, as in src/main.js.
const FOLLOW_VIEWS = [
  ["rear", 0, -0.32],
  ["front", Math.PI, -0.32],
  ["left", Math.PI / 2, -0.22],
  ["right", -Math.PI / 2, -0.22],
  ["top", 0, -Math.PI / 2 + 0.015],
  ["diagonal", Math.PI / 4, -0.6],
];
await fs.mkdir(artifacts, { recursive: true });
const report = {
  date: new Date().toISOString(),
  url: base,
  playwright: require("playwright/package.json").version,
  checks: [],
};
const pageErrors = [];
const failedResponses = [];
const check = (name, details = true) => {
  report.checks.push({ name, details });
  console.log(`PASS ${name}`);
};
// Strip anything token-like before an error message reaches the report.
const redact = (text) =>
  String(text)
    .replace(/access_token=[^&\s"']*/g, "access_token=[redacted]")
    .replace(/\beyJ[\w-]+\.[\w-]+\.[\w-]+/g, "[redacted-jwt]");

// Runs in every document before the app: error capture and shared helpers
// that read the real camera, clock and drone state inside the page.
function installPageHelpers() {
  window.__flightErrors = [];
  addEventListener("error", (event) => __flightErrors.push(event.message));
  addEventListener("unhandledrejection", (event) =>
    __flightErrors.push(String(event.reason)),
  );
  window.__verify = {
    elapsed: () =>
      Cesium.JulianDate.secondsDifference(
        viewer.clock.currentTime,
        viewer.clock.startTime,
      ),
    dronePosition: () =>
      droneSimulation.drone.position.getValue(viewer.clock.currentTime),
    cameraDistance: () =>
      Cesium.Cartesian3.distance(
        viewer.camera.positionWC,
        __verify.dronePosition(),
      ),
    setRange: (id, value) => {
      const slider = document.getElementById(id);
      slider.value = value;
      slider.dispatchEvent(new Event("input", { bubbles: true }));
    },
    // Scrub the flight bar to a mission time (1/1000 of the flight).
    seek: (seconds) =>
      __verify.setRange(
        "flightProgress",
        String(Math.round((1000 * seconds) / droneSimulation.mission.duration)),
      ),
    wheel: (deltaY) =>
      viewer.canvas.dispatchEvent(
        new WheelEvent("wheel", { deltaY, bubbles: true, cancelable: true }),
      ),
    // Camera position in the drone's ENU frame, independent of preset state.
    followGeometry: (bearing) => {
      const C = Cesium;
      const local = C.Matrix4.multiplyByPoint(
        C.Matrix4.inverseTransformation(
          C.Transforms.eastNorthUpToFixedFrame(__verify.dronePosition()),
          new C.Matrix4(),
        ),
        viewer.camera.positionWC,
        new C.Cartesian3(),
      );
      const range = C.Cartesian3.magnitude(local);
      const heading = droneSimulation.evaluate(__verify.elapsed()).heading;
      const actual = Math.atan2(-local.x, -local.y) - heading;
      return {
        view: droneSimulation.followView,
        range,
        pitch: -Math.asin(local.z / range),
        bearingError: Math.atan2(
          Math.sin(actual - bearing),
          Math.cos(actual - bearing),
        ),
        pressed: document.querySelectorAll(
          '[data-follow-view][aria-pressed="true"]',
        ).length,
        target: droneSimulation.followDistance.target,
      };
    },
    followViewMatches: (bearing, pitch) => {
      const g = __verify.followGeometry(bearing);
      return (
        Math.abs(g.bearingError) < 0.025 &&
        Math.abs(g.pitch - pitch) < 0.015 &&
        Math.abs(g.range - 8) < 0.05
      );
    },
    // Disabled flag of each environment field, in ENVIRONMENT_FIELDS order.
    environmentLocked: (ids) =>
      ids.map((id) => document.getElementById(id).disabled),
    // Phase names of the flown record at the given times.
    phaseAt: (times) => times.map((t) => droneSimulation.evaluate(t).phase),
    // Plan, then report a summary of the mission in the page.
    summary: () => {
      const m = droneSimulation.mission,
        a = droneSimulation.analysis,
        s = m.simulation;
      return {
        state: droneSimulation.state,
        type: m.type,
        duration: m.duration,
        photos: a.capture.photos.length,
        finalSoc: s.metrics.finalSoc,
        landingSite: s.metrics.landingSite,
        landingError: s.metrics.landingError,
        homeDistance: s.metrics.homeDistance,
        maxTrackingError: s.metrics.maxTrackingError,
        returnReason: s.failsafe?.returnReason ?? null,
        events: (s.failsafe?.events ?? []).map((e) => ({
          time: e.time,
          kind: e.kind,
          label: e.label,
        })),
        plan: a.checks
          .filter((c) => c.group !== "fault")
          .map((c) => ({ id: c.id, ok: c.ok })),
        scenario: a.checks
          .filter((c) => c.group === "fault")
          .map((c) => ({ id: c.id, ok: c.ok, detail: c.detail })),
        blocked: a.blocked,
        listed: document.querySelectorAll("#checklist li[data-check]").length,
        group: document.querySelectorAll('#checklist li[data-group="fault"]').length,
        status: document.getElementById("checklistStatus").textContent,
        logged: [...document.querySelectorAll("#eventLog li.fault, #eventLog li.failsafe")].map(
          (li) => li.className.split(" ")[0],
        ),
        entities: viewer.entities.values.length,
        message: document.getElementById("message").textContent,
      };
    },
    footprints: () => viewer.dataSources.getByName("footprints")[0],
    // Text of the file an export button would download.
    exported: async (id) => {
      let captured;
      const createObjectURL = URL.createObjectURL,
        click = HTMLAnchorElement.prototype.click;
      try {
        URL.createObjectURL = (blob) => {
          captured = blob;
          return createObjectURL.call(URL, blob);
        };
        HTMLAnchorElement.prototype.click = () => {};
        document.getElementById(id).click();
      } finally {
        URL.createObjectURL = createObjectURL;
        HTMLAnchorElement.prototype.click = click;
      }
      return captured.text();
    },
  };
}

function observe(page, label) {
  page.on("pageerror", (error) =>
    pageErrors.push(`${label}: ${redact(error.message)}`),
  );
  page.on("response", (response) => {
    const status = response.status();
    if (status < 400) return;
    const url = new URL(response.url());
    // Path only: query strings can carry the access token.
    failedResponses.push({
      page: label,
      host: url.host,
      path: url.pathname,
      status,
      expected: url.host === "api.cesium.com" && status === 401,
    });
  });
}
const wait = (page, predicate, arg, timeout = 30000) =>
  page.waitForFunction(predicate, arg, { timeout, polling: 200 });
const state = (page) => page.evaluate(() => droneSimulation.state);
// Resolves once planning settles (optionally with the given AGL, mass,
// mission type and fault) and fails fast with the page's own message unless
// the expected state ("ready", or "blocked" for a plan the checklist
// rejects) is reached.
async function waitPlanned(page, agl, expected = "ready", mass, type, fault) {
  await wait(
    page,
    ([agl, mass, type, fault]) =>
      (["ready", "blocked"].includes(window.droneSimulation?.state) &&
        (agl === null || droneSimulation.mission.settings.agl === agl) &&
        (mass === null ||
          droneSimulation.mission.simulation.airframe.mass === mass) &&
        (type === null || droneSimulation.mission.type === type) &&
        (fault === null ||
          (droneSimulation.mission.simulation.failsafe?.fault?.type ?? "none") ===
            fault)) ||
      !document.getElementById("mapError").hidden,
    [agl ?? null, mass ?? null, type ?? null, fault ?? null],
    READY_TIMEOUT,
  );
  const status = await page.evaluate(() => ({
    state: window.droneSimulation?.state ?? "loading",
    message: document.getElementById("message").textContent,
  }));
  assert.equal(status.state, expected, status.message);
}
// Camera flights have finished and the terrain tiles of the view are loaded.
const settleCamera = (page, timeout = 45000) =>
  wait(
    page,
    () => !viewer.camera._currentFlight && viewer.scene.globe.tilesLoaded,
    undefined,
    timeout,
  );
const restoreToken = (page, token) =>
  page.evaluate(
    ([key, value]) => {
      if (value === null) sessionStorage.removeItem(key);
      else sessionStorage.setItem(key, value);
    },
    [TOKEN_KEY, token],
  );

const browser = await chromium.launch({ headless: true });
report.browser = `chromium ${browser.version()}`;
let page, mobile;
// undefined until read; null when the tab had no session token.
let priorIonToken;
try {
  const context = await browser.newContext({
    viewport: { width: 1440, height: 1000 },
    deviceScaleFactor: 1,
  });
  context.setDefaultTimeout(30000);
  await context.addInitScript(installPageHelpers);
  page = await context.newPage();
  observe(page, "desktop");
  console.log("Opening production preview");
  await page.goto(base);
  console.log("Waiting for terrain plan");
  await waitPlanned(page);
  await page.evaluate(() =>
    viewer.scene.renderError.addEventListener((scene, error) =>
      __flightErrors.push(String(error)),
    ),
  );
  const initial = await page.evaluate(() => {
    const m = droneSimulation.mission;
    return {
      duration: m.duration,
      horizontalDistance: m.horizontalDistance,
      samples: m.samples.length,
      homeHeight: m.groundHeight,
      maxTerrain: Math.max(...m.samples.map((p) => p.height)),
      minClearance: m.minClearance,
      maxAgl: m.maxAgl,
      photos: droneSimulation.analysis.capture.photos.length,
      footprints: __verify.footprints().entities.values.length,
      listeners: viewer.clock.onTick.numberOfListeners,
      entities: viewer.entities.values.length,
      metrics: m.simulation.metrics,
      airframe: m.simulation.airframe,
    };
  });
  assert.ok(initial.samples > 1000 && initial.duration > 600);
  // Drone, reference, flown track, home and the two rally points.
  assert.equal(initial.entities, 6);
  assert.ok(initial.photos > 200);
  assert.equal(initial.footprints, initial.photos);
  check(
    "Worker solves 240 Hz rigid-body flight with actual terrain under a nested deployment path",
    initial,
  );

  // Terrain following and the preflight checklist on real terrain.
  const preflight = await page.evaluate(() => {
    const m = droneSimulation.mission,
      a = droneSimulation.analysis;
    let below = Infinity;
    for (let i = 0; i < m.profile.length; i++)
      below = Math.min(below, m.profile[i] - m.requirement[i]);
    return {
      blocked: a.blocked,
      failed: a.checks.filter((c) => !c.ok).map((c) => c.label),
      listed: document.querySelectorAll("#checklist li[data-check]").length,
      status: document.getElementById("checklistStatus").textContent,
      profileMinusRequirement: below,
      profileSpan: Math.max(...m.profile) - Math.min(...m.profile),
      minClearance: m.minClearance,
      maxAgl: m.maxAgl,
      agl: m.settings.agl,
      finalSoc: m.simulation.metrics.finalSoc,
      navigationError: m.simulation.metrics.maxNavigationError,
      tiltError: m.simulation.metrics.maxTiltError,
      events: document.querySelectorAll("#eventLog li[data-time]").length,
      obstacles: a.checks.find((c) => c.id === "obstacles").detail,
      sites: a.checks.find((c) => c.id === "sites").detail,
      obstacleClearance: m.obstacleClearance.clearance,
      obstacleLayer: viewer.dataSources.getByName("obstacles")[0].entities.values.length,
      obstacleLegend: !document.getElementById("legendObstacle").hidden,
    };
  });
  assert.equal(preflight.blocked, false);
  assert.deepEqual(preflight.failed, []);
  assert.equal(preflight.listed, 15);
  assert.equal(
    await page.evaluate(
      () => document.querySelectorAll('#checklist li[data-group="fault"]').length,
    ),
    0,
  );
  assert.equal(preflight.status, "Tümü geçti");
  assert.ok(preflight.profileMinusRequirement >= -1e-9);
  assert.ok(preflight.profileSpan > 20);
  assert.ok(preflight.minClearance >= preflight.agl - 5);
  assert.ok(preflight.maxAgl <= 120);
  assert.ok(preflight.finalSoc > 0.25);
  assert.ok(preflight.navigationError < 0.2);
  // ESKF attitude from the IMU and magnetometer, not the true attitude.
  assert.ok(preflight.tiltError > 0 && preflight.tiltError < (1.5 * Math.PI) / 180);
  assert.ok(preflight.events >= 10);
  // OpenStreetMap obstacles: 80 m AGL clears the 30 m canopy and the
  // buildings by at least the 20 m margin; buildings near the path are drawn.
  assert.match(preflight.obstacles, /OSM 20\d\d-\d\d-\d\d/);
  assert.ok(preflight.obstacleClearance >= 40, `${preflight.obstacleClearance}`);
  assert.ok(preflight.obstacleLayer > 50 && preflight.obstacleLegend);
  assert.match(preflight.sites, /^3 alan/);
  check(
    "Terrain-following plan passes all fifteen preflight checks (OSM obstacle clearance and landing sites included) with RTK/ESKF navigation and battery reserve",
    preflight,
  );

  // Turbulence controls: defaults now, lock states at later moments, one check.
  const environment = {
    defaults: await page.evaluate((ids) => {
      const field = (id) => document.getElementById(id);
      const s = droneSimulation.mission.simulation;
      return {
        windMax: field("windInput").max,
        wind: field("windInput").value,
        seed: field("seedInput").value,
        seedStep: field("seedInput").step,
        turbulence: field("turbulenceInput").checked,
        simulation: { wind: s.wind, seed: s.seed, turbulence: s.turbulence },
        summary: field("physicsSummary").textContent,
        locked: __verify.environmentLocked(ids),
      };
    }, ENVIRONMENT_FIELDS),
  };
  assert.equal(environment.defaults.windMax, WIND_LIMIT);
  assert.ok(Number(environment.defaults.wind) <= Number(WIND_LIMIT));
  assert.equal(environment.defaults.seed, "1");
  assert.equal(environment.defaults.seedStep, "1");
  assert.equal(environment.defaults.turbulence, true);
  assert.deepEqual(environment.defaults.simulation, {
    wind: Number(environment.defaults.wind),
    seed: 1,
    turbulence: true,
  });
  assert.ok(
    environment.defaults.summary.includes("Dryden") &&
      environment.defaults.summary.includes("tohum 1"),
  );
  assert.deepEqual(environment.defaults.locked, UNLOCKED);
  // Recon without a fault: side overlap and fault times do not apply.
  assert.deepEqual(
    await page.evaluate(
      (ids) => __verify.environmentLocked(ids),
      DEPENDENT_FIELDS,
    ),
    [true, true, true],
  );
  // The wind field's limit follows the take-off mass (rotor and body drag
  // against the thrust margin): 2.0 m/s at 1.5 kg, back to 3.5 at 2 kg.
  await page.fill("#massInput", "1.5");
  environment.lighter = await page.evaluate(() => [
    document.getElementById("windInput").max,
    document.getElementById("windLimitValue").textContent,
  ]);
  await page.fill("#massInput", "2");
  assert.deepEqual(environment.lighter, ["2", "2,0"]);
  assert.equal(await page.evaluate(() => document.getElementById("windInput").max), WIND_LIMIT);

  const dynamics = await page.evaluate(() => {
    const log = droneSimulation.mission.simulation;
    const STRIDE = log.data.length / log.rows;
    let maxStep = 0,
      turnSamples = 0;
    for (let i = 1; i < log.rows; i++) {
      const a = (i - 1) * STRIDE + 7,
        b = i * STRIDE + 7;
      let dot = 0;
      for (let j = 0; j < 4; j++) dot += log.data[a + j] * log.data[b + j];
      maxStep = Math.max(maxStep, 2 * Math.acos(Math.min(1, Math.abs(dot))));
      const phase = log.data[i * STRIDE + 30];
      if (
        (phase === 2 || phase === 3) &&
        Math.abs(log.data[i * STRIDE + 13]) > 0.06
      )
        turnSamples++;
    }
    return {
      maxQuaternionStep: maxStep,
      turnSamples,
      trackingError: log.metrics.maxTrackingError,
      maxBank: log.metrics.maxTilt,
      motorSaturation: log.metrics.saturationSeconds,
    };
  });
  assert.ok(
    dynamics.maxQuaternionStep < 0.025 &&
      dynamics.turnSamples > 100 &&
      dynamics.trackingError < 1,
  );
  assert.ok(dynamics.maxBank > 0.05 && dynamics.maxBank < (27 * Math.PI) / 180);
  check(
    "Continuous physical attitude, banked fly-through turns and bounded tracking error",
    dynamics,
  );

  const csv = await page.evaluate(async () => {
    const lines = (await __verify.exported("exportCsvButton"))
      .trim()
      .split("\n");
    return {
      rows: lines.length - 1,
      columns: lines[0].split(",").length,
      header: lines[0].split(",").slice(-10),
      firstTime: Number(lines[1].split(",")[0]),
      lastTime: Number(lines.at(-1).split(",")[0]),
    };
  });
  assert.ok(
    csv.rows > 5000 &&
      csv.columns === 42 &&
      csv.firstTime === 0 &&
      csv.lastTime > initial.duration - 0.2,
  );
  assert.deepEqual(csv.header, [
    "battery_soc",
    "battery_v",
    "battery_a",
    "power_w",
    "est_east_m",
    "est_north_m",
    "est_up_m",
    "eph_m",
    "wind_est_e_ms",
    "wind_est_n_ms",
  ]);
  check("CSV exports physical, battery, navigation and wind-estimate states at 10 Hz", csv);

  const files = await page.evaluate(async () => {
    const kml = await __verify.exported("exportKmlButton"),
      plan = JSON.parse(await __verify.exported("exportPlanButton")),
      geo = JSON.parse(await __verify.exported("exportGeojsonButton")),
      xml = new DOMParser().parseFromString(kml, "application/xml");
    return {
      kmlParses: !xml.querySelector("parsererror"),
      kmlPlacemarks: xml.getElementsByTagName("Placemark").length,
      planType: plan.fileType,
      planFirst: plan.mission.items[0].command,
      planLast: plan.mission.items.at(-1).command,
      planWaypoints: plan.mission.items.filter((i) => i.command === 16).length,
      triggerDistance: plan.mission.items.find((i) => i.command === 206)
        .params[0],
      geoType: geo.type,
      footprints: geo.features.filter((f) => f.properties.kind === "footprint")
        .length,
      photos: droneSimulation.analysis.capture.photos.length,
      rally: droneSimulation.mission.rally.length,
      planRally: plan.rallyPoints.points.length,
    };
  });
  assert.equal(files.kmlParses, true);
  assert.equal(files.kmlPlacemarks, files.photos + 3 + files.rally);
  assert.equal(files.planRally, 2);
  assert.equal(files.planType, "Plan");
  assert.equal(files.planFirst, 22);
  assert.equal(files.planLast, 20);
  assert.ok(files.planWaypoints > 20);
  assert.ok(files.triggerDistance > 10);
  assert.equal(files.geoType, "FeatureCollection");
  assert.equal(files.footprints, files.photos);
  check(
    "KML parses, QGroundControl plan and GeoJSON carry every waypoint and footprint",
    files,
  );

  const liveTerrain = await page.evaluate(async () => {
    const m = droneSimulation.mission;
    const states = Array.from({ length: 289 }, (_, i) =>
      droneSimulation.evaluate(
        m.reference.surveyStart +
          (m.reference.horizontalDuration * (i + 0.5)) / 289,
      ),
    );
    const terrain = states.map((s) => Cesium.Cartographic.clone(s.position));
    await Cesium.sampleTerrainMostDetailed(
      viewer.terrainProvider,
      terrain,
      true,
    );
    return {
      points: terrain.length,
      minAGL: Math.min(
        ...terrain.map((p, i) => states[i].position.height - p.height),
      ),
      allFinite: terrain.every((p) => Number.isFinite(p.height)),
    };
  });
  assert.ok(
    liveTerrain.allFinite && liveTerrain.minAGL >= initial.minClearance - 10,
  );
  check(
    "Independent terrain queries between route samples confirm clearance",
    liveTerrain,
  );

  await page.click("#overviewButton");
  await settleCamera(page);
  await page.screenshot({ path: path.join(artifacts, "desktop-overview.png") });
  await page.click("#startButton");
  await wait(
    page,
    () => droneSimulation.state === "running" && __verify.elapsed() >= 1,
  );
  const takeoff = await page.evaluate(() => {
    const elapsed = __verify.elapsed();
    const sample = droneSimulation.evaluate(elapsed);
    return {
      state: droneSimulation.state,
      elapsed,
      agl: sample.agl,
      groundSpeed: sample.groundSpeed,
      speed: document.getElementById("speedStatus").textContent,
    };
  });
  assert.equal(takeoff.state, "running");
  assert.ok(takeoff.groundSpeed < 0.25);
  check("Force-driven takeoff has bounded wind drift", takeoff);
  environment.running = await page.evaluate(
    (ids) => __verify.environmentLocked(ids),
    ENVIRONMENT_FIELDS,
  );
  assert.deepEqual(environment.running, LOCKED);
  await page.click("#pauseButton");
  environment.paused = await page.evaluate(
    (ids) => __verify.environmentLocked(ids),
    ENVIRONMENT_FIELDS,
  );
  assert.deepEqual(environment.paused, LOCKED);
  const paused = await page.evaluate(() => ({
    time: __verify.elapsed(),
    frame: viewer.scene.frameState.frameNumber,
    now: performance.now(),
  }));
  // Let the render loop tick for a while before reading the frozen clock.
  await wait(
    page,
    ({ frame, now }) =>
      viewer.scene.frameState.frameNumber > frame + 5 &&
      performance.now() - now > 700,
    paused,
  );
  assert.equal(await page.evaluate(() => __verify.elapsed()), paused.time);
  await page.click("#pauseButton");
  assert.equal(await state(page), "running");
  check("Pause freezes simulation time and resume continues");

  await page.evaluate(() => __verify.setRange("flightProgress", "350"));
  assert.equal(await state(page), "paused");
  await page.click("#followButton");
  await settleCamera(page);
  assert.equal(
    await page.evaluate(() => droneSimulation.cameraMode),
    "follow",
  );
  await page.screenshot({ path: path.join(artifacts, "drone-follow.png") });
  assert.equal(
    await page.evaluate(
      () => document.getElementById("freeButton") !== null,
    ),
    false,
  );
  await page.evaluate(() => __verify.setRange("followDistance", "2"));
  await wait(
    page,
    () =>
      Math.abs(__verify.cameraDistance() - 2) < 0.08 &&
      Math.abs(droneSimulation.followDistance.current - 2) < 0.05,
  );
  const closeDistance = await page.evaluate(() => ({
    target: droneSimulation.followDistance.target,
    current: droneSimulation.followDistance.current,
    actual: __verify.cameraDistance(),
  }));
  assert.ok(Math.abs(closeDistance.current - 2) < 0.05);
  await page.evaluate(() => __verify.wheel(500));
  assert.ok(
    await page.evaluate(() => droneSimulation.followDistance.target > 3),
  );
  await page.evaluate(() => __verify.wheel(-500));
  assert.ok(
    await page.evaluate(
      () => Math.abs(droneSimulation.followDistance.target - 2) < 0.02,
    ),
  );
  await page.click("#zoomInButton");
  assert.ok(
    await page.evaluate(() => droneSimulation.followDistance.target < 2),
  );
  await page.click("#zoomOutButton");
  assert.ok(
    await page.evaluate(() => droneSimulation.followDistance.target > 1.8),
  );
  await page.evaluate(() => __verify.setRange("followDistance", "2"));
  await page.click("#overviewButton");
  assert.equal(
    await page.evaluate(
      () =>
        document.getElementById("followZoomControls").hidden &&
        viewer.scene.screenSpaceCameraController.enableZoom,
    ),
    true,
  );
  await page.click("#followButton");
  assert.equal(
    await page.evaluate(() => droneSimulation.followDistance.target),
    2,
  );
  await page.evaluate(() => __verify.setRange("followDistance", "8"));
  check(
    "Follow zoom accepts slider, wheel and buttons, persists selection, and Free mode is removed",
    closeDistance,
  );

  const cameraViews = [];
  for (const [view, bearing, pitch] of FOLLOW_VIEWS) {
    await page.click(`[data-follow-view="${view}"]`);
    await wait(
      page,
      ([bearing, pitch]) => __verify.followViewMatches(bearing, pitch),
      [bearing, pitch],
    );
    const actual = await page.evaluate(
      (bearing) => __verify.followGeometry(bearing),
      bearing,
    );
    assert.equal(actual.view, view);
    assert.equal(actual.pressed, 1);
    assert.equal(actual.target, 8);
    assert.ok(
      Math.abs(actual.bearingError) < 0.025 &&
        Math.abs(actual.pitch - pitch) < 0.015 &&
        Math.abs(actual.range - 8) < 0.05,
      JSON.stringify(actual),
    );
    cameraViews.push(actual);
  }
  await page
    .locator(".camera-controls")
    .screenshot({ path: path.join(artifacts, "follow-camera-angles.png") });
  await page.click("#overviewButton");
  await page.click("#followButton");
  assert.equal(
    await page.evaluate(() => droneSimulation.followView),
    "diagonal",
  );
  check(
    "Six drone-relative camera angles match measured ENU geometry and preserve zoom and selection",
    cameraViews,
  );
  // Payload camera view: at the drone, stabilised to the gimbal pitch,
  // footprints and the airframe hidden, HUD shown; overview restores all.
  await page.click("#gimbalButton");
  await settleCamera(page);
  const gimbal = await page.evaluate(() => {
    const C = Cesium,
      sample = droneSimulation.evaluate(__verify.elapsed());
    return {
      mode: droneSimulation.cameraMode,
      offset: C.Cartesian3.distance(viewer.camera.positionWC, sample.cartesian),
      pitch: C.Math.toDegrees(viewer.camera.pitch),
      headingError: Math.abs(
        Math.atan2(
          Math.sin(viewer.camera.heading - sample.heading),
          Math.cos(viewer.camera.heading - sample.heading),
        ),
      ),
      fov: C.Math.toDegrees(viewer.camera.frustum.fov),
      footprintsShown: __verify.footprints().show,
      modelShown: droneSimulation.drone.model.show.getValue(
        viewer.clock.currentTime,
      ),
      hud: !document.getElementById("gimbalHud").hidden,
      hudText: document.getElementById("hudPhoto").textContent,
      inputs: viewer.scene.screenSpaceCameraController.enableInputs,
    };
  });
  assert.equal(gimbal.mode, "gimbal");
  assert.ok(gimbal.offset < 0.5, JSON.stringify(gimbal));
  assert.ok(Math.abs(gimbal.pitch + 90) < 0.5, JSON.stringify(gimbal));
  assert.ok(gimbal.headingError < 0.02);
  assert.ok(Math.abs(gimbal.fov - 73.74) < 0.1);
  assert.equal(gimbal.footprintsShown, false);
  assert.equal(gimbal.modelShown, false);
  assert.equal(gimbal.hud, true);
  assert.match(gimbal.hudText, /^IMG \d{4} \/ \d{4}$/);
  assert.equal(gimbal.inputs, false);
  await page.screenshot({ path: path.join(artifacts, "gimbal-camera.png") });
  await page.click("#overviewButton");
  // Cesium disables camera input for the duration of the overview flight.
  await settleCamera(page);
  const restored = await page.evaluate(() => ({
    fov: Math.round(Cesium.Math.toDegrees(viewer.camera.frustum.fov) * 100) / 100,
    footprintsShown: __verify.footprints().show,
    hud: !document.getElementById("gimbalHud").hidden,
    inputs: viewer.scene.screenSpaceCameraController.enableInputs,
  }));
  assert.deepEqual(restored, {
    fov: 60,
    footprintsShown: true,
    hud: false,
    inputs: true,
  });
  check(
    "Gimbal camera sits at the drone, looks at the commanded pitch with the sensor's field of view, and overview restores the map",
    { gimbal, restored },
  );
  check("Scrubbing and the three camera modes work independently of telemetry");

  await page.click("#resetButton");
  await page.click("#startButton");
  // Speed up only the playback clock; the recorded physics stays identical.
  await page.evaluate(() => {
    viewer.clock.multiplier = 180;
  });
  await wait(page, () => droneSimulation.state === "complete", undefined, 40000);
  const landing = await page.evaluate(() => {
    const m = droneSimulation.mission;
    const position = __verify.dronePosition();
    const actual = Cesium.Cartographic.fromCartesian(position);
    const start = droneSimulation.evaluate(0).position;
    return {
      landingErrorMeters: Cesium.Cartesian3.distance(
        position,
        Cesium.Cartesian3.fromRadians(
          start.longitude,
          start.latitude,
          start.height,
        ),
      ),
      heightError: actual.height - start.height,
      disarmed: m.simulation.metrics.disarmed,
      speed: document.getElementById("speedStatus").textContent,
      animate: viewer.clock.shouldAnimate,
      phase: document.getElementById("flightPhase").textContent,
      listeners: viewer.clock.onTick.numberOfListeners,
      entities: viewer.entities.values.length,
    };
  });
  assert.ok(
    landing.landingErrorMeters < 0.2 && Math.abs(landing.heightError) < 0.02,
  );
  assert.equal(landing.disarmed, true);
  assert.equal(landing.speed, "0.0 km/h");
  assert.equal(landing.animate, false);
  check(
    "Force-driven flight lands within 20 cm, contacts ground and disarms",
    landing,
  );
  await page.click("#startButton");
  await page.click("#resetButton");
  assert.equal(
    await page.evaluate(() => viewer.clock.onTick.numberOfListeners),
    initial.listeners,
  );
  assert.equal(
    await page.evaluate(() => viewer.entities.values.length),
    initial.entities,
  );
  check("Replay does not accumulate event listeners or entities");

  // Operator takeover: an RTL commanded during playback re-solves the
  // flight from that moment; up to it the solve is bitwise the planned one.
  await page.click("#startButton");
  await wait(page, () => droneSimulation.state === "running");
  await page.evaluate(() => __verify.seek(420));
  await wait(page, () => droneSimulation.state === "paused");
  const before = await page.evaluate(() => droneSimulation.mission.duration);
  await page.click("#operatorRtlButton");
  await wait(
    page,
    () => droneSimulation.mission.simulation.failsafe?.commands?.length === 1 && droneSimulation.state === "paused",
    undefined,
    60000,
  );
  const operator = await page.evaluate(() => {
    const m = droneSimulation.mission,
      sim = m.simulation,
      nominal = m.nominal.simulation,
      stride = sim.data.length / sim.rows;
    let identical = true,
      rows = 0;
    for (let i = 0; i < Math.min(sim.rows, nominal.rows); i++) {
      if (sim.data[i * stride] >= sim.failsafe.commands[0].time) break;
      rows++;
      for (let j = 0; j < stride; j++)
        if (sim.data[i * stride + j] !== nominal.data[i * stride + j]) identical = false;
    }
    return {
      command: sim.failsafe.commands[0],
      identical,
      rows,
      landing: sim.metrics.landingSite,
      landingName: sim.metrics.landingName,
      duration: m.duration,
      message: document.getElementById("message").textContent,
      events: sim.failsafe.events.filter((e) => e.kind === "operator").map((e) => e.label),
      response: droneSimulation.analysis.checks.find((c) => c.id === "fault-response")?.label,
    };
  });
  assert.ok(Math.abs(operator.command.time - 420) < 2, JSON.stringify(operator.command));
  assert.ok(operator.identical && operator.rows > 12000, JSON.stringify(operator));
  assert.deepEqual(operator.events, ["OPERATÖR · eve dönüş (RTL)"]);
  assert.equal(operator.landing, "rally");
  assert.equal(operator.landingName, "Doğu vadi tabanı");
  assert.ok(operator.duration < before);
  assert.equal(operator.response, "Operatör komutları");
  check(
    `Operator RTL at 07:00 during playback re-solves the flight (bitwise identical for ${operator.rows} rows before it) and lands at the nearest rally point`,
    operator,
  );
  await page.click("#resetButton");

  await page.fill("#aglInput", "90");
  await page.click("#planButton");
  // Planning starts synchronously on the click, well before the solve ends.
  environment.loading = await page.evaluate(
    (ids) => ({
      state: droneSimulation.state,
      locked: __verify.environmentLocked(ids),
    }),
    ENVIRONMENT_FIELDS,
  );
  assert.deepEqual(environment.loading, {
    state: "loading",
    locked: LOCKED,
  });
  await waitPlanned(page, 90);
  const higher = await page.evaluate(() => ({
    minClearance: droneSimulation.mission.minClearance,
    entities: viewer.entities.values.length,
    footprints: __verify.footprints().entities.values.length,
    photos: droneSimulation.analysis.capture.photos.length,
    spacing: droneSimulation.analysis.capture.spacing,
  }));
  assert.ok(higher.minClearance >= 85);
  assert.equal(higher.entities, 6);
  assert.equal(higher.footprints, higher.photos);
  // Trigger spacing scales with height: 20% of a 90 m along-track footprint.
  assert.ok(Math.abs(higher.spacing - 18) < 1e-6);
  check(
    "Changing the target AGL rebuilds the terrain-following mission and its photo plan without duplicates",
    higher,
  );
  // An overweight airframe drains the pack below the landing reserve: the
  // checklist blocks the plan, which stays inspectable but not flyable.
  await page.fill("#aglInput", "80");
  await page.fill("#massInput", "3");
  await page.click("#planButton");
  await waitPlanned(page, 80, "blocked", 3);
  const blocked = await page.evaluate(() => ({
    state: droneSimulation.state,
    start: document.getElementById("startButton").disabled,
    progress: document.getElementById("flightProgress").disabled,
    failed: droneSimulation.analysis.checks
      .filter((c) => c.blocking && !c.ok)
      .map((c) => c.id),
    status: document.getElementById("checklistStatus").textContent,
    message: document.getElementById("message").textContent,
  }));
  assert.equal(blocked.start, true);
  assert.equal(blocked.progress, false);
  assert.ok(blocked.failed.includes("reserve"));
  assert.match(blocked.status, /engel/);
  await page.evaluate(() => __verify.setRange("flightProgress", "500"));
  assert.equal(await state(page), "blocked");
  check(
    "Preflight checklist blocks an overweight plan below the battery reserve but keeps it inspectable",
    blocked,
  );
  await page.fill("#massInput", "2");
  await page.click("#planButton");
  await waitPlanned(page, 80, "ready", 2);
  environment.ready = await page.evaluate(
    (ids) => __verify.environmentLocked(ids),
    ENVIRONMENT_FIELDS,
  );
  assert.deepEqual(environment.ready, UNLOCKED);
  check(
    `Turbulence seed and switch default to 1/on, lock while planning or flying, and the wind field stops at the ${WIND_LIMIT} m/s authority limit`,
    environment,
  );

  // Area survey: a lawnmower over the route file's polygon, lines along its
  // longest edge, spacing from the camera footprint and the side overlap.
  await page.selectOption("#missionTypeInput", "survey");
  const surveyForm = await page.evaluate(
    (ids) => ({
      dependent: __verify.environmentLocked(ids),
      title: document.getElementById("missionTitle").textContent,
      stage: document.querySelector('[data-phase="survey"]').textContent,
    }),
    DEPENDENT_FIELDS,
  );
  assert.deepEqual(surveyForm.dependent, [false, true, true]);
  assert.equal(surveyForm.title, "Alan haritalama");
  assert.match(surveyForm.stage, /Haritalama/);
  await page.click("#planButton");
  await waitPlanned(page, 80, "ready", 2, "survey");
  const survey = await page.evaluate(async () => {
    const m = droneSimulation.mission,
      a = droneSimulation.analysis,
      s = m.survey;
    let longest = 0,
      edge;
    for (let i = 0; i < s.polygon.length; i++) {
      const p = s.polygon[i],
        q = s.polygon[(i + 1) % s.polygon.length],
        l = Math.hypot(q[0] - p[0], q[1] - p[1]);
      if (l > longest) {
        longest = l;
        edge = [(q[0] - p[0]) / l, (q[1] - p[1]) / l];
      }
    }
    const plan = JSON.parse(await __verify.exported("exportPlanButton")),
      kml = await __verify.exported("exportKmlButton"),
      geo = JSON.parse(await __verify.exported("exportGeojsonButton")),
      // The grid is a QGC Survey complex item around our own waypoints.
      complex = plan.mission.items.filter((i) => i.type === "ComplexItem"),
      items = plan.mission.items.flatMap((i) =>
        i.type === "ComplexItem" ? i.TransectStyleComplexItem.Items : [i],
      );
    const photoPhases = __verify.phaseAt(a.capture.photos.map((p) => p.time));
    const labels = a.events.map((e) => e.label);
    return {
      lines: s.count,
      spacing: s.spacing,
      swath: s.swath,
      parallel: Math.abs(edge[0] * s.direction[0] + edge[1] * s.direction[1]),
      areaHa: Math.abs(s.areaM2) / 1e4,
      covered: a.coverage.areaCovered,
      minSideOverlap: a.coverage.minSideOverlap,
      minForwardOverlap: a.coverage.minOverlap,
      photos: a.capture.photos.length,
      photoPhases: [...new Set(photoPhases)],
      listed: document.querySelectorAll("#checklist li[data-check]").length,
      failed: a.checks.filter((c) => !c.ok).map((c) => c.label),
      status: document.getElementById("checklistStatus").textContent,
      entities: viewer.entities.values.length,
      footprints: __verify.footprints().entities.values.length,
      triggers: items.filter((i) => i.command === 206).map((i) => i.params[0]),
      waypoints: items.filter((i) => i.command === 16).length,
      jumpIds: items.every((i, k) => i.doJumpId === k + 1),
      surveyItem: complex.map((i) => ({
        type: i.complexItemType,
        version: i.version,
        polygon: i.polygon.length,
        transects: i.TransectStyleComplexItem.VisualTransectPoints.length,
        distanceMode: i.TransectStyleComplexItem.CameraCalc.DistanceMode,
        side: i.TransectStyleComplexItem.CameraCalc.AdjustedFootprintSide,
      })),
      kmlArea: kml.includes("<name>Haritalama alanı</name>"),
      geoArea: geo.features.some((f) => f.properties.kind === "surveyArea"),
      minClearance: m.minClearance,
      maxAgl: m.maxAgl,
      duration: m.duration,
      finalSoc: m.simulation.metrics.finalSoc,
      grid: labels.includes("Izgara başladı") && labels.includes("Izgara tamamlandı"),
      legend: !document.getElementById("legendArea").hidden,
    };
  });
  assert.ok(survey.lines >= 4, JSON.stringify(survey));
  assert.ok(Math.abs(survey.swath - 120) < 1e-6);
  assert.ok(Math.abs(survey.spacing - 0.3 * survey.swath) < 1e-6);
  assert.ok(survey.parallel > 1 - 1e-9);
  assert.ok(survey.covered >= 0.98 && Math.abs(survey.minSideOverlap - 0.7) < 0.03);
  assert.ok(survey.minForwardOverlap >= 0.75);
  assert.deepEqual(survey.photoPhases, ["survey"]);
  assert.equal(survey.listed, 16);
  assert.deepEqual(survey.failed, []);
  assert.equal(survey.status, "Tümü geçti");
  // Plus the polygon outline.
  assert.equal(survey.entities, 7);
  assert.equal(survey.footprints, survey.photos);
  assert.equal(survey.triggers.length, 2);
  assert.ok(survey.triggers[0] > 10 && survey.triggers[1] === 0);
  assert.ok(survey.waypoints >= 2 * survey.lines);
  assert.ok(survey.jumpIds);
  assert.equal(survey.surveyItem.length, 1);
  assert.equal(survey.surveyItem[0].type, "survey");
  assert.equal(survey.surveyItem[0].version, 5);
  assert.equal(survey.surveyItem[0].transects, 2 * survey.lines);
  assert.equal(survey.surveyItem[0].distanceMode, 3);
  assert.ok(Math.abs(survey.surveyItem[0].side - survey.spacing) < 0.01);
  assert.ok(survey.kmlArea && survey.geoArea && survey.grid && survey.legend);
  assert.ok(survey.minClearance >= 75 && survey.maxAgl <= 120);
  assert.ok(survey.finalSoc > 0.25);
  await page.click("#overviewButton");
  await settleCamera(page);
  await page.screenshot({ path: path.join(artifacts, "survey-overview.png") });
  await page.click("#startButton");
  await page.evaluate(() => {
    viewer.clock.multiplier = 180;
  });
  await wait(page, () => droneSimulation.state === "complete", undefined, 40000);
  survey.landing = await page.evaluate(() => ({
    landingError: droneSimulation.mission.simulation.metrics.landingError,
    phase: document.getElementById("flightPhase").textContent,
    message: document.getElementById("message").textContent,
  }));
  assert.ok(survey.landing.landingError < 0.2);
  assert.match(survey.landing.message, /Haritalama tamamlandı/);
  check(
    `Area survey flies ${survey.lines} lawnmower lines along the polygon's longest edge, images ${(survey.covered * 100).toFixed(1)} % of it with ${Math.round(survey.minSideOverlap * 100)} % side overlap, passes sixteen checks and exports a QGC Survey item`,
    survey,
  );
  // A higher side overlap narrows the line spacing and adds lines; the grid
  // no longer fits one battery, so it is split into sorties (battery swap)
  // that each keep the landing reserve and together cover the area.
  await page.fill("#sideOverlapInput", "80");
  await page.click("#planButton");
  await wait(
    page,
    () =>
      ["ready", "blocked"].includes(droneSimulation.state) &&
      droneSimulation.mission.survey?.sideOverlap === 0.8,
    undefined,
    READY_TIMEOUT,
  );
  const sortie = () =>
    page.evaluate(() => ({
      state: droneSimulation.state,
      sortie: droneSimulation.mission.survey.sortie,
      lines: droneSimulation.mission.survey.lines.length,
      count: droneSimulation.mission.survey.count,
      spacing: droneSimulation.mission.survey.spacing,
      duration: droneSimulation.mission.duration,
      finalSoc: droneSimulation.mission.simulation.metrics.finalSoc,
      covered: droneSimulation.analysis.planCoverage.areaCovered,
      minSideOverlap: droneSimulation.analysis.planCoverage.minSideOverlap,
      buttons: document.querySelectorAll("#sortieControls button").length,
      message: document.getElementById("message").textContent,
    }));
  const denser = { first: await sortie() };
  await page.click('#sortieControls button[data-sortie="1"]');
  await wait(page, () => droneSimulation.mission.survey.sortie.index === 1);
  denser.second = await sortie();
  await page.click('#sortieControls button[data-sortie="0"]');
  const { first, second } = denser;
  assert.ok(Math.abs(first.spacing - 24) < 1e-6 && first.count > survey.lines);
  assert.equal(first.state, "ready");
  assert.deepEqual([first.sortie, second.sortie], [
    { index: 0, count: 2 },
    { index: 1, count: 2 },
  ]);
  assert.equal(first.buttons, 2);
  assert.equal(first.lines + second.lines, first.count);
  assert.ok(first.finalSoc > 0.25 && second.finalSoc > 0.25);
  assert.ok(first.covered >= 0.98 && Math.abs(first.minSideOverlap - 0.8) < 0.03);
  assert.match(first.message, /2 uçuş \(batarya değişimiyle\)/);
  assert.match(second.message, /^Uçuş 2\/2/);
  check(
    `Raising the side overlap to 80 % re-plans ${first.count} lines at 24 m as two battery sorties (${first.lines} + ${second.lines} lines) landing with ${Math.round(first.finalSoc * 100)} % and ${Math.round(second.finalSoc * 100)} %, together covering ${(first.covered * 100).toFixed(1)} %`,
    denser,
  );
  await page.fill("#sideOverlapInput", "70");
  await page.selectOption("#missionTypeInput", "recon");

  // In-flight fault injection: each scenario is solved fault-free for the
  // checklist and faulted for the flight, in parallel workers.
  const scenario = async (fault, time) => {
    await page.selectOption("#faultInput", fault);
    assert.equal(
      await page.evaluate(() => document.getElementById("faultTimeInput").disabled),
      false,
    );
    await page.fill("#faultTimeInput", String(time));
    await page.click("#planButton");
    await waitPlanned(page, 80, "ready", 2, "recon", fault);
    return page.evaluate(() => __verify.summary());
  };
  const flyToEnd = async () => {
    await page.click("#resetButton");
    await page.click("#startButton");
    await page.evaluate(() => {
      viewer.clock.multiplier = 180;
    });
    await wait(page, () => droneSimulation.state === "complete", undefined, 40000);
  };
  const lowBattery = await scenario("battery", 240);
  const rtl = lowBattery.events.find((e) => e.label.includes("RTL"));
  assert.ok(rtl && rtl.time > 240, JSON.stringify(lowBattery.events));
  assert.equal(lowBattery.returnReason, "battery");
  assert.equal(lowBattery.blocked, false);
  assert.ok(lowBattery.plan.every((c) => c.ok));
  assert.equal(lowBattery.listed, 15 + 5);
  assert.equal(lowBattery.group, 1);
  assert.ok(lowBattery.scenario.every((c) => c.ok), JSON.stringify(lowBattery.scenario));
  // The return goes to the nearest safe point: the east rally point.
  assert.equal(lowBattery.landingSite, "rally");
  assert.ok(lowBattery.finalSoc >= 0.25 && lowBattery.duration < initial.duration - 200);
  assert.deepEqual(lowBattery.logged, ["fault", "failsafe"]);
  // Fault and RTL markers plus the dashed direct return leg.
  assert.equal(lowBattery.entities, 6 + 1 + 2);
  // Scrubbed past the RTL, the flight bar names the failsafe.
  await page.evaluate((t) => __verify.seek(t + 20), rtl.time);
  lowBattery.banner = await page.evaluate(() => ({
    phase: document.getElementById("flightPhase").textContent,
    failsafe: document.getElementById("flightPhase").classList.contains("failsafe"),
  }));
  assert.ok(
    lowBattery.banner.failsafe &&
      /Eve dönüş · RTL — ⚠ düşük batarya/.test(lowBattery.banner.phase),
    JSON.stringify(lowBattery.banner),
  );
  await flyToEnd();
  lowBattery.end = await page.evaluate(() => ({
    message: document.getElementById("message").textContent,
    landing: droneSimulation.mission.simulation.metrics.landingError,
  }));
  assert.match(lowBattery.end.message, /rally noktasında \(Doğu vadi tabanı\)/);
  assert.ok(lowBattery.end.landing < 0.2);
  check(
    `Low-battery fault at 04:00 triggers a wind-aware return-charge RTL at ${Math.floor(rtl.time / 60)}:${String(Math.round(rtl.time % 60)).padStart(2, "0")} over a direct terrain-following leg to the nearest rally point, landing with ${Math.round(lowBattery.finalSoc * 100)} %`,
    lowBattery,
  );

  const outage = await scenario("gnss-outage", 300);
  const outageLabels = outage.events.map((e) => e.label);
  for (const label of [
    "ARIZA · GNSS sinyali kesildi",
    "FAILSAFE · konum tutma",
    "GNSS geri geldi",
    "Göreve devam",
  ])
    assert.ok(outageLabels.includes(label), label);
  assert.equal(outage.landingSite, "home");
  assert.ok(outage.duration > initial.duration);
  assert.ok(outage.scenario.every((c) => c.ok), JSON.stringify(outage.scenario));
  // During the outage telemetry shows the lost fix and the hold.
  await page.evaluate(() =>
    __verify.seek(droneSimulation.mission.simulation.failsafe.fault.time + 8),
  );
  outage.during = await page.evaluate(() => ({
    gnss: document.getElementById("gnssStatus").textContent,
    alert: document.getElementById("gnssStatus").classList.contains("alert"),
    phase: document.getElementById("flightPhase").textContent,
  }));
  assert.match(outage.during.gnss, /FIX YOK/);
  assert.ok(outage.during.alert);
  assert.match(outage.during.phase, /— ⚠ GNSS yok · ölü hesap/);
  // Scrubbing paused the flight, which locks the plan inputs; back to the
  // pad unlocks them.
  assert.equal(
    await page.evaluate(() => document.getElementById("faultInput").disabled),
    true,
  );
  await page.click("#resetButton");
  check(
    "A 15 s GNSS outage holds position on dead reckoning, shows the lost fix, then resumes the mission and lands home",
    outage,
  );

  const loss = await scenario("gnss-loss", 300);
  assert.equal(loss.landingSite, "field");
  assert.ok(loss.homeDistance > 100);
  assert.ok(
    loss.scenario.find((c) => c.id === "fault-landing").detail.startsWith("yerinde"),
  );
  await flyToEnd();
  // The vehicle rests on the real terrain at its touchdown point.
  loss.ground = await page.evaluate(async () => {
    const position = Cesium.Cartographic.fromCartesian(__verify.dronePosition()),
      ground = Cesium.Cartographic.clone(position);
    await Cesium.sampleTerrainMostDetailed(viewer.terrainProvider, [ground], true);
    return {
      cg: position.height,
      ground: ground.height,
      message: document.getElementById("message").textContent,
    };
  });
  assert.ok(
    Math.abs(loss.ground.cg - loss.ground.ground) < 3,
    JSON.stringify(loss.ground),
  );
  assert.match(loss.ground.message, /yerinde indi/);
  check(
    `Permanent GNSS loss holds, then lands in place ${Math.round(loss.homeDistance)} m from home on the terrain (CG ${(loss.ground.cg - loss.ground.ground).toFixed(2)} m above the live ground)`,
    loss,
  );

  const motor = await scenario("motor", 300);
  const detected = motor.events.find(
    (e) => e.label === "Motor dengesizliği algılandı",
  );
  assert.ok(detected && detected.time - 300 < 5, JSON.stringify(motor.events));
  assert.equal(motor.returnReason, "motor");
  assert.equal(motor.landingSite, "rally");
  assert.ok(motor.maxTrackingError < 2);
  check(
    "A 30 % loss of rotor 1 effectiveness is detected from the motor imbalance and flown to the nearest rally point by RTL within 2 m tracking",
    motor,
  );

  // Two faults at once escalate together: without GNSS the low battery
  // cannot be flown home blind, so the vehicle lands where it is.
  await page.selectOption("#faultInput", "gnss-loss");
  await page.fill("#faultTimeInput", "300");
  await page.selectOption("#secondFaultInput", "battery");
  await page.fill("#secondFaultTimeInput", "310");
  await page.click("#planButton");
  await waitPlanned(page, 80, "ready", 2, "recon", "gnss-loss");
  const double = await page.evaluate(() => __verify.summary());
  const doubleLabels = double.events.map((e) => e.label);
  assert.ok(doubleLabels.includes("ARIZA · GNSS sinyali kesildi"));
  assert.ok(doubleLabels.includes("ARIZA · batarya hücresi"));
  assert.ok(doubleLabels.includes("FAILSAFE · yerinde iniş"), JSON.stringify(doubleLabels));
  assert.ok(!doubleLabels.some((l) => l.includes("RTL")));
  assert.equal(double.landingSite, "field");
  check(
    "Simultaneous GNSS loss and battery fault escalate to a landing in place instead of a blind return",
    double,
  );
  await page.selectOption("#secondFaultInput", "none");
  await page.selectOption("#faultInput", "none");
  await page.click("#planButton");
  await waitPlanned(page, 80, "ready", 2, "recon", "none");
  assert.equal(
    await page.evaluate(() => viewer.entities.values.length),
    initial.entities,
  );

  // Calibration from a flight log: the flight exported as PX4 ULog is read
  // back by the calibration panel; applied, the next plan flies the
  // identified model, and the nominal model can be restored.
  const [download] = await Promise.all([
    page.waitForEvent("download"),
    page.click("#exportUlogButton"),
  ]);
  const ulogPath = path.join(artifacts, "flight.ulg");
  await download.saveAs(ulogPath);
  await page.evaluate(() => {
    document.getElementById("calibrationDetails").open = true;
  });
  await page.setInputFiles("#calibrationFile", ulogPath);
  await wait(
    page,
    () => /uçuş örneği|error/.test(document.getElementById("calibrationResult").innerHTML),
    undefined,
    60000,
  );
  const calibration = await page.evaluate(() => ({
    file: document.getElementById("calibrationFile").files[0].size,
    rows: document.querySelectorAll("#calibrationResult tbody tr").length,
    kept: document.querySelectorAll("#calibrationResult tbody tr.skipped").length,
    summary: document.querySelector("#calibrationResult p")?.textContent,
    apply: document.getElementById("calibrationApply").disabled,
  }));
  assert.equal(calibration.rows, 10, JSON.stringify(calibration));
  assert.ok(calibration.file > 1e6 && calibration.apply === false);
  await page.click("#calibrationApply");
  await wait(
    page,
    () =>
      droneSimulation.mission?.model?.startsWith("kalibre") &&
      ["ready", "blocked"].includes(droneSimulation.state),
    undefined,
    READY_TIMEOUT,
  );
  calibration.applied = await page.evaluate(() => ({
    state: droneSimulation.state,
    model: droneSimulation.mission.model,
    thrustCoefficient: droneSimulation.mission.simulation.airframe.thrustCoefficient,
    dragArea: droneSimulation.mission.simulation.airframe.dragArea,
    status: document.getElementById("checklistStatus").textContent,
  }));
  assert.equal(calibration.applied.state, "ready");
  // The log was flown with the nominal model: the identification returns it.
  assert.ok(Math.abs(calibration.applied.thrustCoefficient / 1.8e-5 - 1) < 0.01);
  assert.ok(Math.abs(calibration.applied.dragArea / 0.08 - 1) < 0.1);
  await page.click("#calibrationReset");
  await wait(
    page,
    () => droneSimulation.mission?.model === "nominal" && droneSimulation.state === "ready",
    undefined,
    READY_TIMEOUT,
  );
  check(
    "The flight exports as PX4 ULog, the calibration panel identifies it (thrust and drag within 1 % and 10 %), applies it to the next plan and restores the nominal model",
    calibration,
  );

  // A separate context gives the mobile page a real device-width viewport
  // with touch, which cannot be switched on an open desktop page.
  console.log("Opening mobile preview");
  const mobileContext = await browser.newContext({
    viewport: { width: 390, height: 844 },
    deviceScaleFactor: 1,
    isMobile: true,
    hasTouch: true,
  });
  mobileContext.setDefaultTimeout(30000);
  await mobileContext.addInitScript(installPageHelpers);
  mobile = await mobileContext.newPage();
  observe(mobile, "mobile");
  await mobile.goto(base);
  await waitPlanned(mobile);
  await mobile.evaluate(() =>
    viewer.scene.renderError.addEventListener((scene, error) =>
      __flightErrors.push(String(error)),
    ),
  );
  const layout = await mobile.evaluate(() => {
    const sidebar = document.querySelector(".sidebar");
    return {
      width: innerWidth,
      bodyWidth: document.body.scrollWidth,
      mapHeight: document
        .getElementById("cesiumContainer")
        .getBoundingClientRect().height,
      panelHeight: sidebar.clientHeight,
      scrollHeight: sidebar.scrollHeight,
    };
  });
  assert.equal(layout.width, 390);
  assert.ok(layout.bodyWidth <= 390);
  assert.ok(
    layout.mapHeight >= 300 &&
      layout.panelHeight > 300 &&
      layout.scrollHeight > layout.panelHeight,
  );
  await mobile.screenshot({
    path: path.join(artifacts, "mobile-overview.png"),
  });
  check(
    "Mobile map and independently scrollable controls fit 390px without overflow",
    layout,
  );
  await mobile.click("#followButton");
  await mobile.click('[data-follow-view="top"]');
  const mobileCamera = await mobile.evaluate(() => {
    const rect = (selector) =>
      document.querySelector(selector).getBoundingClientRect();
    const c = rect(".camera-controls"),
      h = rect(".attitude-instrument"),
      f = rect(".flight-bar");
    return {
      buttons: document.querySelectorAll("[data-follow-view]").length,
      overflow: document.body.scrollWidth > innerWidth,
      panelBottom: c.bottom,
      horizonBottom: h.bottom,
      flightTop: f.top,
      overlap: !(
        c.right <= h.left ||
        c.left >= h.right ||
        c.bottom <= h.top ||
        c.top >= h.bottom
      ),
    };
  });
  assert.equal(mobileCamera.buttons, 6);
  assert.equal(mobileCamera.overflow, false);
  assert.equal(mobileCamera.overlap, false);
  assert.ok(
    mobileCamera.panelBottom < mobileCamera.flightTop &&
      mobileCamera.horizonBottom < mobileCamera.flightTop,
  );
  await mobile.screenshot({
    path: path.join(artifacts, "mobile-follow-angles.png"),
  });
  check(
    "Six camera controls fit mobile without overlapping the horizon or flight bar",
    mobileCamera,
  );
  const mobileErrors = await mobile.evaluate(() => __flightErrors);
  await mobileContext.close();
  mobile = undefined;

  const errors = {
    desktop: await page.evaluate(() => __flightErrors),
    mobile: mobileErrors,
    uncaught: pageErrors,
  };
  assert.deepEqual(errors.desktop, []);
  assert.deepEqual(errors.mobile, []);
  assert.deepEqual(errors.uncaught, []);
  check(
    "No JavaScript or Cesium render errors during the exercised flows",
    errors,
  );

  priorIonToken = await page.evaluate((key) => {
    try {
      return sessionStorage.getItem(key);
    } catch {
      return null;
    }
  }, TOKEN_KEY);
  await page.evaluate(() => {
    document.getElementById("ionToken").closest("details").open = true;
  });
  await page.fill("#ionToken", "deliberately-invalid-test-token");
  await page.click("#tokenButton");
  await wait(page, () => droneSimulation.state === "error");
  assert.ok(
    await page.evaluate(
      () =>
        document.getElementById("startButton").disabled &&
        !document.getElementById("mapError").hidden &&
        viewer.entities.values.length === 0,
    ),
  );
  check("Invalid terrain credentials display an error and block flight");
  await restoreToken(page, priorIonToken);
  await page.goto(base);
  await waitPlanned(page);
  check("Valid terrain access recovers after a failed load");

  const unexpected = failedResponses.filter((r) => !r.expected);
  assert.equal(unexpected.length, 0, JSON.stringify(unexpected));
  check(
    "Only the deliberate invalid-token 401 failed on the network",
    failedResponses,
  );
  report.success = true;
} catch (error) {
  report.success = false;
  report.error = redact(error.message);
  process.exitCode = 1;
  console.error(`FAIL ${redact(error.stack || error.message)}`);
} finally {
  if (page && priorIonToken !== undefined)
    await restoreToken(page, priorIonToken).catch(() => {});
  report.failedRequests = failedResponses;
  await fs.writeFile(
    path.join(artifacts, "browser-verification.json"),
    JSON.stringify(report, null, 2) + "\n",
  );
  await browser.close().catch(() => {});
}
