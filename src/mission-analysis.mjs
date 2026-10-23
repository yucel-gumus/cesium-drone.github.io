import { STRIDE, PHASE_NAMES, inducedVelocity } from "./quadrotor.mjs";
import { capturePhotos, coverageSummary } from "./payload.mjs";
import { GNSS_MODES } from "./navigation.mjs";
import { FAULTS } from "./failsafe.mjs";
import { obstaclesNear, obstacleAt, obstacleText } from "./obstacles.mjs";

// ENU point of the mission frame to WGS84 longitude/latitude (degrees) and
// ellipsoidal height (m).
export function toGeodetic(C, mission, p) {
  const cartesian = C.Matrix4.multiplyByPoint(
    mission.frame,
    new C.Cartesian3(p[0], p[1], p[2]),
    new C.Cartesian3(),
  );
  const g = C.Cartographic.fromCartesian(cartesian);
  return {
    longitude: C.Math.toDegrees(g.longitude),
    latitude: C.Math.toDegrees(g.latitude),
    height: g.height,
  };
}

// Every flown position inside the route box (the mission geofence).
function geofence(C, mission, sim = mission.simulation) {
  const b = mission.bounds;
  let inside = true,
    worst = null;
  for (let i = 0; i < sim.rows; i += 15) {
    const r = i * STRIDE,
      g = toGeodetic(C, mission, [
        sim.data[r + 1],
        sim.data[r + 2],
        sim.data[r + 3],
      ]);
    if (
      g.longitude < b.west ||
      g.longitude > b.east ||
      g.latitude < b.south ||
      g.latitude > b.north
    ) {
      inside = false;
      worst = g;
      break;
    }
  }
  return { inside, worst };
}

const percent = (x) => `%${Math.round(x * 100)}`;
const metres = (x, digits = 1) => `${x.toFixed(digits)} m`;
// Obstacle clearance of a solve (measureFlight().obstacleClearance) as a
// checklist item: the lowest margin over an obstacle top within the
// lateral buffer, against the required clearance less the tolerance.
function obstacleCheck(mission, flown) {
  const s = mission.settings,
    data = mission.obstacleData,
    count = mission.obstacles?.items.length ?? 0,
    source = data?.snapshot
      ? `OSM ${data.timestamp ? data.timestamp.slice(0, 10) : ""}`.trim()
      : "rota dosyası";
  if (!count || (!data?.covers && !data?.route))
    return {
      detail: "engel verisi yok · binalar ve ağaçlar denetlenmedi",
      ok: false,
      blocking: false,
    };
  if (!flown || !Number.isFinite(flown.clearance))
    return {
      detail: `${s.obstacleLateral} m tamponda engel yok · ${count} engel (${source})`,
      ok: true,
      blocking: true,
    };
  return {
    detail: `${metres(flown.clearance)} · ${obstacleText(flown.obstacle)} · hedef ≥ ${s.obstacleClearance} m, tampon ${s.obstacleLateral} m · ${count} engel (${source})`,
    ok: flown.clearance >= s.obstacleClearance - s.clearanceTolerance,
    blocking: true,
  };
}
// Nearest obstacle to the take-off pad and each rally point: a vertical
// climb or descent needs the lateral buffer free.
function siteCheck(mission) {
  const s = mission.settings,
    sites = [
      { name: "kalkış", xy: [0, 0] },
      ...(mission.rally ?? []).map((r) => ({ name: r.name, xy: r.xy })),
    ].map((site) => {
      const hits = obstaclesNear(mission.obstacles, site.xy, 60).sort(
        (a, b) => a.distance - b.distance,
      );
      return { ...site, nearest: hits[0] ?? null };
    }),
    worst = sites.reduce(
      (w, x) => ((x.nearest?.distance ?? Infinity) < (w.nearest?.distance ?? Infinity) ? x : w),
      sites[0],
    );
  return {
    id: "sites",
    label: "Kalkış ve iniş alanları",
    detail: worst.nearest
      ? `${sites.length} alan · en yakın engel ${worst.name}: ${obstacleText(worst.nearest.obstacle)}, ${metres(worst.nearest.distance)} · tampon ${s.obstacleLateral} m`
      : `${sites.length} alan · 60 m içinde engel yok`,
    ok:
      sites.every((x) => !x.nearest || x.nearest.distance >= s.obstacleLateral) &&
      !sites.some((x) => obstacleAt(mission.obstacles, x.xy)),
    blocking: false,
  };
}

const clock = (seconds) => {
  const t = Math.max(0, Math.round(seconds));
  return `${String(Math.floor(t / 60)).padStart(2, "0")}:${String(t % 60).padStart(2, "0")}`;
};
const distance = (x) =>
  x >= 1000 ? `${(x / 1000).toFixed(2)} km` : `${Math.round(x)} m`;

// Preflight checklist in the order an operator walks it. Blocking items
// keep the mission from being flown; the others are advisories. The plan
// is judged on its fault-free solve (mission.nominal); an injected fault
// is a rehearsal, reported by contingencyChecks().
export function preflightChecks(C, mission, capture, coverage) {
  const s = mission.settings,
    plan = mission.nominal ?? mission,
    sim = plan.simulation,
    m = sim.metrics,
    battery = sim.battery,
    fence = geofence(C, mission, sim),
    minVoltage = battery.cells * battery.minCellVoltage,
    gnss = GNSS_MODES[sim.gnss];
  const checks = [
    {
      id: "terrain",
      label: "Arazi modeli",
      detail: `${mission.samples.length} örnek · ±${s.corridorHalfWidth} m koridor + ${mission.settings.rasterSpacing} m ızgara`,
      ok: mission.samples.every((p) => Number.isFinite(p.height)),
      blocking: true,
    },
    {
      id: "clearance",
      label: "Asgari arazi açıklığı",
      detail: `${metres(plan.minClearance)} · hedef ${s.agl} m, tolerans ${s.clearanceTolerance} m`,
      ok: plan.minClearance >= s.agl - s.clearanceTolerance,
      blocking: true,
    },
    {
      id: "obstacles",
      label: "Engel açıklığı",
      ...obstacleCheck(mission, plan.obstacleClearance),
    },
    {
      id: "ceiling",
      label: "Yasal irtifa tavanı",
      detail: `en yüksek ${metres(plan.maxAgl)} AGL · sınır ${s.maxAgl} m`,
      ok: plan.maxAgl <= s.maxAgl,
      blocking: true,
    },
    {
      id: "geofence",
      label: "Coğrafi sınır",
      detail: fence.inside
        ? "uçulan iz rota kutusunun içinde"
        : `sınır dışı: ${fence.worst.latitude.toFixed(5)}°, ${fence.worst.longitude.toFixed(5)}°`,
      ok: fence.inside,
      blocking: true,
    },
    {
      id: "corridor",
      label: "Koridor sapması",
      detail: `${metres(plan.maxCorridorDeviation)} · sınır ${s.corridorHalfWidth - 2} m`,
      ok: plan.maxCorridorDeviation <= s.corridorHalfWidth - 2,
      blocking: true,
    },
    {
      id: "reserve",
      label: "İniş batarya rezervi",
      detail: `inişte ${percent(m.finalSoc)} · asgari ${percent(battery.reserve)}`,
      ok: m.finalSoc >= battery.reserve,
      blocking: true,
    },
    {
      id: "voltage",
      label: "Pil gerilimi",
      detail: `en düşük ${m.minVoltage.toFixed(2)} V · sınır ${minVoltage.toFixed(1)} V`,
      ok: m.minVoltage >= minVoltage && m.batteryLimitedSeconds === 0,
      blocking: true,
    },
    {
      id: "authority",
      label: "Motor yetkisi",
      detail: `doygunluk ${m.saturationSeconds.toFixed(1)} s · en yüksek ${Math.round((m.maxRotorSpeed * 60) / (2 * Math.PI))} rpm`,
      ok: m.saturationSeconds <= 0.5,
      blocking: true,
    },
    {
      id: "tracking",
      label: "Rota takibi",
      detail: `en büyük sapma ${metres(m.maxTrackingError, 2)}`,
      ok: m.maxTrackingError <= 3,
      blocking: false,
    },
    {
      id: "navigation",
      label: "Navigasyon",
      detail: gnss
        ? `${gnss.label} · ${gnss.satellites} uydu · kestirim hatası ≤ ${metres(m.maxNavigationError, 2)} · yönelim ≤ ${((m.maxTiltError * 180) / Math.PI).toFixed(1)}° eğim, ${((m.maxHeadingError * 180) / Math.PI).toFixed(1)}° yön`
        : "ideal durum bilgisi",
      ok: m.maxNavigationError <= 3,
      blocking: false,
    },
    {
      id: "coverage",
      label: "Fotoğraf bindirmesi",
      detail: `${coverage.count} kare · asgari ${percent(coverage.minOverlap)} · hedef ${percent(capture.overlap)}`,
      ok: coverage.count > 0 && coverage.minOverlap >= capture.overlap - 0.05,
      blocking: false,
    },
    {
      id: "vrs",
      label: "Dikey iniş · VRS payı",
      detail: `düşük ileri hızda en hızlı alçalma ${m.maxSlowDescent.toFixed(2)} m/s · VRS başlangıcı ${(0.3 * inducedVelocity(sim.airframe)).toFixed(2)} m/s (0,3 v_h) · itki kaybı ${percent(m.maxVrsLoss)}`,
      ok:
        m.maxSlowDescent <= 0.3 * inducedVelocity(sim.airframe) &&
        m.maxVrsLoss < 0.01,
      blocking: false,
    },
    {
      id: "landing",
      label: "İniş doğruluğu",
      detail:
        m.landingCompleted === false
          ? `iniş tamamlanamadı: motor yetkisi ya da paket yetmedi · hedeften ${metres(m.landingError, 1)}`
          : `${metres(m.landingError, 2)} · temas ${m.contactSpeed.toFixed(2)} m/s`,
      ok: m.landingCompleted !== false && m.landingError <= (sim.gnss === "standard" ? 3 : 0.5),
      // An unfinished landing makes the plan unflyable.
      blocking: m.landingCompleted === false,
    },
  ];
  if (mission.obstacles?.items.length)
    checks.splice(checks.findIndex((c) => c.id === "geofence") + 1, 0, siteCheck(mission));
  if (mission.survey) {
    const sv = mission.survey;
    checks.splice(checks.findIndex((c) => c.id === "coverage") + 1, 0, {
      id: "area",
      label: "Alan kapsaması",
      detail: `${percent(coverage.areaCovered)} / ${(coverage.areaHectares).toFixed(1)} ha · ${sv.count} hat${sv.sortie.count > 1 ? ` · ${sv.sortie.count} uçuş` : ""} · yan bindirme asgari ${percent(coverage.minSideOverlap)}, hedef ${percent(sv.sideOverlap)}`,
      ok:
        coverage.areaCovered >= 0.98 &&
        coverage.minSideOverlap >= sv.sideOverlap - 0.05,
      blocking: false,
    });
  }
  return checks;
}

// Outcome of an injected fault, from the flown (faulted) solve: what the
// autopilot did, where and how it landed, the clearance and the charge it
// kept. Advisory items of their own group: a rehearsal never blocks.
export function contingencyChecks(mission) {
  const sim = mission.simulation,
    f = sim.failsafe,
    faults = f?.faults ?? [],
    commands = f?.commands ?? [];
  if (!faults.length && !commands.length) return [];
  const s = mission.settings,
    m = sim.metrics,
    battery = sim.battery,
    // Faults, failsafe actions and operator commands in the order they
    // happened.
    chain = f.events
      .filter(
        (e) =>
          e.kind === "fault" ||
          e.kind === "operator" ||
          e.label.startsWith("FAILSAFE"),
      )
      .map(
        (e) =>
          `${e.label.replace(/^(FAILSAFE|ARIZA|OPERATÖR) · /, (x) => (x.startsWith("OPERATÖR") ? "operatör: " : ""))} ${clock(e.time)}`,
      ),
    injected = f.events.some((e) => e.kind === "fault"),
    acted = f.events.some(
      (e) => e.kind === "operator" || e.label.startsWith("FAILSAFE"),
    ),
    site = m.landingSite,
    // An obstacle under a landing away from home: trees or a roof.
    landedOn =
      site !== "home" && mission.obstacles?.items.length
        ? obstacleAt(mission.obstacles, m.landingPosition ?? [Infinity, Infinity])
        : null,
    obstacle = mission.obstacleClearance;
  const checks = [
    {
      id: "fault-response",
      label: commands.length && !faults.length ? "Operatör komutları" : "Arıza ve failsafe",
      detail:
        faults.length && !injected && !commands.length
          ? `arıza ${clock(faults[0].time)}'de tetiklenmedi: uçuş daha önce bitti`
          : acted
            ? chain.join(" → ")
            : `${chain.join(" + ")} → failsafe eylemi gerekmedi`,
      ok: true,
    },
    {
      id: "fault-landing",
      label: "Arıza sonrası iniş",
      detail:
        site === "field"
          ? `yerinde${m.landingMoved ? ` · engelden ${metres(m.landingMoved, 0)} uzağa alındı` : ""} · evden ${distance(m.homeDistance)} · hedeften ${metres(m.landingError)} · temas ${m.contactSpeed.toFixed(2)} m/s`
          : site === "rally"
            ? `rally noktası (${m.landingName}) · evden ${distance(m.homeDistance)} · hedeften ${metres(m.landingError, 2)} · temas ${m.contactSpeed.toFixed(2)} m/s`
            : `evde · ${metres(m.landingError, 2)} · temas ${m.contactSpeed.toFixed(2)} m/s`,
      ...(landedOn && {
        detail: `${site === "field" ? `yerinde${m.landingMoved ? ` (engelden ${metres(m.landingMoved, 0)} uzağa alındı, ölü hesap hatasıyla yine engele indi)` : ""}` : `rally noktası (${m.landingName})`} · ⚠ engel üzerine: ${obstacleText(landedOn)} · evden ${distance(m.homeDistance)}`,
      }),
      ok: m.disarmed && m.contactSpeed < 1 && !landedOn,
    },
    {
      id: "fault-clearance",
      label: "Arıza sonrası arazi açıklığı",
      detail: Number.isFinite(mission.returnClearance)
        ? `${Number.isFinite(mission.minClearance) ? `rota ${metres(mission.minClearance)}` : "rota uçulmadı"} · dönüş bacağı ${metres(mission.returnClearance)} (sapma ${metres(mission.returnDeviation)})`
        : Number.isFinite(mission.minClearance)
          ? `${metres(mission.minClearance)} · hedef ${s.agl} m`
          : "rota uçulmadı: kalkış iptal edildi",
      ok:
        Math.min(mission.minClearance, mission.returnClearance) >=
        s.agl - s.clearanceTolerance,
    },
    ...(mission.obstacles?.items.length
      ? [
          {
            id: "fault-obstacles",
            label: "Arıza sonrası engel açıklığı",
            ...obstacleCheck(mission, obstacle),
          },
        ]
      : []),
    {
      id: "fault-battery",
      label: "Arıza sonrası batarya",
      detail: `inişte ${percent(m.finalSoc)} · asgari ${percent(battery.reserve)}`,
      ok: m.finalSoc >= battery.reserve,
    },
  ];
  if (faults.some((x) => x.type.startsWith("gnss")))
    checks.push({
      id: "fault-navigation",
      label: "Ölü hesap kestirimi",
      detail: `azami hata ${metres(m.maxNavigationError)} · EPH ${metres(m.maxEph)}`,
      ok: m.maxNavigationError <= s.corridorHalfWidth - 2,
    });
  return checks.map((c) => ({ ...c, blocking: false, group: "fault" }));
}

// Mission timeline reconstructed from the flight record, with the fault
// and failsafe events logged by the solve.
export function missionEvents(mission, capture) {
  const sim = mission.simulation,
    m = sim.metrics,
    survey = mission.survey,
    field = m.landingSite === "field",
    events = [];
  const add = (time, label, detail = "", kind = "info") =>
    events.push({ time, label, detail, kind });
  add(0, "Motorlar hazırlanıyor", "ön kontroller tamam, rotorlar rölantide");
  let lastPhase = -1,
    halfBattery = false,
    climbed = false;
  for (let i = 0; i < sim.rows; i++) {
    const r = i * STRIDE,
      t = sim.data[r],
      phase = sim.data[r + 30],
      name = PHASE_NAMES[phase];
    if (phase !== lastPhase) {
      if (name === "takeoff") add(t, "Kalkış", "dikey tırmanış");
      if (!climbed && ["transit", "survey"].includes(name)) {
        climbed = true;
        add(
          t,
          "Görev irtifası",
          `${metres(sim.data[r + 3])} · ${name === "transit" ? "alana geçiş" : "keşif başladı"}`,
        );
      }
      if (survey && name === "survey")
        add(t, "Izgara başladı", `${survey.count} hat · ${metres(survey.spacing)} aralık`);
      if (name === "return")
        add(
          t,
          survey ? "Izgara tamamlandı" : "Dönüş bacağı",
          "eve dönüş rotası",
        );
      if (name === "settle")
        add(t, field ? "İniş noktası üzerinde" : "Ev noktası üzerinde", "konum tutma");
      if (name === "landing") add(t, "İniş alçalması", "3 m sürünme irtifasına");
      if (name === "touchdown" && lastPhase !== phase)
        add(t, "Son yaklaşma", `${mission.settings.landingSpeed} m/s sabit iniş hızı`);
      lastPhase = phase;
    }
    if (!halfBattery && sim.data[r + 36] <= 0.5) {
      halfBattery = true;
      add(t, "Batarya %50", `${sim.data[r + 37].toFixed(1)} V`);
    }
  }
  for (const e of sim.failsafe?.events ?? [])
    add(e.time, e.label, e.detail, e.kind);
  if (m.contactTime !== null)
    add(
      m.contactTime,
      "Zemin teması",
      `${m.contactSpeed.toFixed(2)} m/s${field ? " · yerinde iniş" : ""}`,
    );
  if (capture.photos.length) {
    add(capture.photos[0].time, "İlk fotoğraf", `GSD ${(capture.photos[0].gsd * 100).toFixed(2)} cm`);
    add(
      capture.photos.at(-1).time,
      "Son fotoğraf",
      `${capture.photos.length} kare`,
    );
  }
  if (m.disarmTime !== null)
    add(m.disarmTime, "Motorlar durdu", "iniş dedektörü");
  add(sim.duration, "Görev tamamlandı", `inişte batarya ${percent(m.finalSoc)}`);
  return events.sort((a, b) => a.time - b.time);
}

// The fault-free photos of a mission (what the plan images).
export function planPhotos(mission, payloadSettings = {}) {
  return capturePhotos(
    { ...mission, simulation: mission.nominal?.simulation ?? mission.simulation },
    payloadSettings,
  ).photos;
}

// Photos, coverage, checklist and timeline of a planned mission. With an
// injected fault the plan's checklist and coverage come from the
// fault-free solve and the photos and timeline from the flown one. For a
// sortie of a multi-flight survey, `options.surveyPhotos` are the planned
// photos of every sortie: the area coverage and side overlap are judged on
// them, against all the grid's lines.
export function analyzeMission(C, mission, payloadSettings = {}, options = {}) {
  const capture = capturePhotos(mission, payloadSettings),
    coverage = coverageSummary(capture, undefined, mission.survey),
    nominal = mission.nominal?.simulation;
  let planCapture = capture,
    planCoverage = coverage;
  if (nominal && nominal !== mission.simulation) {
    planCapture = capturePhotos(
      { ...mission, simulation: nominal },
      payloadSettings,
    );
    planCoverage = coverageSummary(planCapture, undefined, mission.survey);
  }
  if (options.surveyPhotos && mission.survey) {
    const whole = coverageSummary(
      { ...planCapture, photos: options.surveyPhotos },
      undefined,
      { ...mission.survey, lines: mission.survey.allLines },
    );
    planCoverage = {
      ...planCoverage,
      areaCovered: whole.areaCovered,
      minSideOverlap: whole.minSideOverlap,
      meanSideOverlap: whole.meanSideOverlap,
      linesImaged: whole.linesImaged,
    };
  }
  const checks = preflightChecks(C, mission, planCapture, planCoverage),
    contingency = contingencyChecks(mission),
    events = missionEvents(mission, capture);
  return {
    capture,
    coverage,
    planCoverage,
    checks: [...checks, ...contingency],
    events,
    blocked: checks.some((c) => c.blocking && !c.ok),
  };
}
