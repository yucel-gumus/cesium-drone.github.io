import { FAULTS, OPERATOR_COMMANDS } from "./failsafe.mjs";
import { AIRFRAME, windLimit } from "./quadrotor.mjs";
import { obstacleText } from "./obstacles.mjs";

const PHASES = {
  ready: "Kalkışa hazır",
  blocked: "Görev onaylanmadı",
  arming: "Motorlar hazırlanıyor",
  takeoff: "Dikey kalkış",
  survey: "Keşif uçuşu · araziyi izliyor",
  transit: "Görev alanına geçiş",
  hold: "Konum tutma",
  rtl: "Eve dönüş · RTL",
  return: "Kalkış noktasına dönüş",
  landing: "Kontrollü alçalma",
  settle: "Ev noktası üzerinde konum tutma",
  touchdown: "Son yaklaşma · iniş",
  disarm: "Temas · motorlar duruyor",
  complete: "Görev tamamlandı",
};
export const formatTime = (seconds) => {
  const t = Math.max(0, Math.round(seconds));
  return `${String(Math.floor(t / 60)).padStart(2, "0")}:${String(t % 60).padStart(2, "0")}`;
};
export const formatDistance = (distance) =>
  distance >= 1000
    ? `${(distance / 1000).toFixed(2)} km`
    : `${Math.round(distance)} m`;
// Mission card texts per mission type.
const MISSION_TEXT = {
  recon: {
    title: "Keşif görevi",
    stage: "Keşif",
    description:
      "Kıyıdan kalkış, sabit AGL'de araziyi izleyerek göl çevresinde fotoğraflı keşif, aynı noktaya otonom iniş.",
    start: "Keşif uçuşunu başlat",
  },
  survey: {
    title: "Alan haritalama",
    stage: "Haritalama",
    description:
      "Göl ve kıyısını kaplayan poligon üzerinde ızgara (lawnmower) deseni: hatlar en uzun kenara paralel, aralık kameranın yan izi ve yan bindirmeden.",
    start: "Haritalama uçuşunu başlat",
  },
};
const escapeHtml = (s) =>
  String(s).replace(
    /[&<>"]/g,
    (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c],
  );

export default class DroneUIController {
  constructor(handlers) {
    this.elements = Object.fromEntries(
      [...document.querySelectorAll("[id]")].map((el) => [el.id, el]),
    );
    this.handlers = handlers;
    this.events = [];
    const e = this.elements;
    e.startButton.addEventListener("click", handlers.start);
    e.pauseButton.addEventListener("click", handlers.pause);
    e.resetButton.addEventListener("click", handlers.reset);
    for (const [id, kind] of [
      ["exportCsvButton", "csv"],
      ["exportKmlButton", "kml"],
      ["exportPlanButton", "plan"],
      ["exportGeojsonButton", "geojson"],
      ["exportUlogButton", "ulog"],
    ])
      e[id].addEventListener("click", () => handlers.export(kind));
    e.calibrationFile.addEventListener("change", () => {
      const file = e.calibrationFile.files?.[0];
      if (file) handlers.calibrate(file);
    });
    e.calibrationApply.addEventListener("click", () => handlers.applyCalibration(true));
    e.calibrationReset.addEventListener("click", () => handlers.applyCalibration(false));
    e.planButton.addEventListener("click", () => {
      if (this.planInputs().every((input) => input.reportValidity()))
        handlers.plan(Number(e.aglInput.value));
    });
    e.tokenButton.addEventListener("click", () =>
      handlers.token(e.ionToken.value.trim()),
    );
    e.flightProgress.addEventListener("input", () =>
      handlers.seek(Number(e.flightProgress.value) / 1000),
    );
    e.playbackSpeed.addEventListener("change", () =>
      handlers.rate(Number(e.playbackSpeed.value)),
    );
    ["overview", "follow", "gimbal"].forEach((mode) =>
      e[`${mode}Button`].addEventListener("click", () => handlers.camera(mode)),
    );
    document.querySelectorAll("[data-follow-view]").forEach((button) =>
      button.addEventListener("click", () =>
        handlers.followView(button.dataset.followView),
      ),
    );
    e.followDistance.addEventListener("input", () =>
      handlers.followDistance(Number(e.followDistance.value)),
    );
    e.zoomInButton.addEventListener("click", () =>
      handlers.followDistance(Number(e.followDistance.value) / 1.3),
    );
    e.zoomOutButton.addEventListener("click", () =>
      handlers.followDistance(Number(e.followDistance.value) * 1.3),
    );
    e.eventLog.addEventListener("click", (event) => {
      const item = event.target.closest("li[data-time]");
      if (item) handlers.seekTime(Number(item.dataset.time));
    });
    e.missionTypeInput.addEventListener("change", () => {
      this.setMissionType(e.missionTypeInput.value);
      this.syncInputs();
    });
    e.faultInput.addEventListener("change", () => this.syncInputs());
    e.secondFaultInput.addEventListener("change", () => this.syncInputs());
    e.sortieControls.addEventListener("click", (event) => {
      const button = event.target.closest("button[data-sortie]");
      if (button) handlers.sortie(Number(button.dataset.sortie));
    });
    e.operatorControls.addEventListener("click", (event) => {
      const button = event.target.closest("button[data-command]");
      if (button && !button.disabled) handlers.operator(button.dataset.command);
    });
    e.massInput.addEventListener("input", () => this.syncWindLimit());
    this.setMissionType(e.missionTypeInput.value);
    this.syncWindLimit();
  }

  // The wind field's upper bound follows the take-off mass (windLimit()).
  syncWindLimit() {
    const e = this.elements,
      mass = Number(e.massInput.value);
    if (!Number.isFinite(mass) || mass < 1 || mass > 3) return;
    const limit = windLimit(this.airframeModel?.(mass) ?? { ...AIRFRAME, mass });
    e.windInput.max = String(limit);
    e.windLimitValue.textContent = limit.toLocaleString("tr", {
      minimumFractionDigits: 1,
    });
  }

  // Calibration panel: { busy } while identifying, { error }, or the
  // identification result with its source and whether it is applied.
  setCalibration(state) {
    const e = this.elements,
      box = e.calibrationResult;
    this.calibrationState = state;
    if (state.busy) {
      box.textContent = `${state.source} tanılanıyor…`;
      this.syncInputs();
      return;
    }
    if (state.error) {
      box.innerHTML = `<span class="error">${escapeHtml(state.error)}</span>`;
      this.syncInputs();
      return;
    }
    if (!state.result) {
      box.textContent = "Kayıt yüklenmedi · nominal model.";
      this.syncInputs();
      return;
    }
    const { report, wind, aero } = state.result,
      number = (x) =>
        x === null || !Number.isFinite(x)
          ? "—"
          : Math.abs(x) >= 0.01 && Math.abs(x) < 1000
            ? x.toLocaleString("tr", { maximumFractionDigits: 3 })
            : x.toExponential(2).replace(".", ",");
    box.innerHTML = `<p>${escapeHtml(state.source)} · ${aero.samples} uçuş örneği · ortalama rüzgâr ${Math.hypot(...wind).toFixed(1)} m/s · ${state.applied ? "<strong>uygulandı</strong>" : "uygulanmadı"}</p>
      <table><thead><tr><th>Parametre</th><th>Nominal</th><th>Kestirim</th><th>Yarı farkı</th></tr></thead><tbody>${report
        .map(
          (r) =>
            `<tr class="${r.applied ? "" : "skipped"}"><td>${escapeHtml(r.label)}</td><td>${number(r.nominal)}</td><td>${number(r.estimate)}${r.unit ? ` ${r.unit}` : ""}</td><td>${Number.isFinite(r.spread) ? `%${(r.spread * 100).toLocaleString("tr", { maximumFractionDigits: 1 })}` : "—"}${r.applied ? "" : " · tutulmadı"}</td></tr>`,
        )
        .join("")}</tbody></table>`;
    this.syncInputs();
  }

  planInputs() {
    const e = this.elements;
    return [
      e.aglInput,
      e.massInput,
      e.windInput,
      e.seedInput,
      e.overlapInput,
      e.sideOverlapInput,
      e.faultTimeInput,
      e.secondFaultTimeInput,
    ];
  }

  // Inputs locked while a plan is solved or flown; side overlap only
  // applies to area surveys and the fault time only with a fault.
  syncInputs() {
    const e = this.elements,
      locked = this.state === "loading" || ["running", "paused"].includes(this.state);
    for (const input of [
      ...this.planInputs(),
      e.missionTypeInput,
      e.turbulenceInput,
      e.gimbalInput,
      e.gnssInput,
      e.faultInput,
      e.secondFaultInput,
      e.rtlTargetInput,
      e.planButton,
      e.tokenButton,
      e.ionToken,
    ])
      input.disabled = locked;
    if (e.missionTypeInput.value !== "survey") e.sideOverlapInput.disabled = true;
    if (e.faultInput.value === "none") e.faultTimeInput.disabled = true;
    if (e.secondFaultInput.value === "none")
      e.secondFaultTimeInput.disabled = true;
    // Applying or dropping a calibration re-plans: not while planning or
    // flying.
    const c = this.calibrationState ?? {};
    e.calibrationApply.disabled = locked || c.busy || !c.result || c.applied;
    e.calibrationReset.disabled = locked || c.busy || !c.applied;
    e.calibrationFile.disabled = locked;
    // Operator commands act on a flight being played.
    const flying = ["running", "paused"].includes(this.state);
    for (const button of e.operatorControls.querySelectorAll("button"))
      button.disabled = !flying;
  }

  // Sortie selector of a multi-flight survey (hidden for one flight).
  setSorties(count, index) {
    const e = this.elements;
    e.sortieControls.hidden = count < 2;
    e.sortieControls.innerHTML =
      count < 2
        ? ""
        : `<span>Batarya değişimli uçuşlar</span>${Array.from(
            { length: count },
            (_, k) =>
              `<button type="button" data-sortie="${k}" class="${k === index ? "active" : ""}" aria-pressed="${k === index}">Uçuş ${k + 1}</button>`,
          ).join("")}`;
  }

  // Mission card texts for a mission type (before and after planning).
  setMissionType(type) {
    const e = this.elements,
      text = MISSION_TEXT[type] ?? MISSION_TEXT.recon;
    e.missionTitle.textContent = text.title;
    e.missionDescription.textContent = text.description;
    document.querySelector('[data-phase="survey"]').lastChild.textContent =
      text.stage;
    this.missionType = type;
    if (this.state) this.setState(this.state);
  }

  message(text, error = false) {
    this.elements.message.textContent = text;
    this.elements.message.classList.toggle("error", error);
  }

  setState(state) {
    const e = this.elements;
    const busy = state === "loading";
    const planned = [
      "ready",
      "running",
      "paused",
      "complete",
      "blocked",
    ].includes(state);
    const flyable = planned && state !== "blocked";
    const active = state === "running" || state === "paused";
    e.startButton.disabled = !flyable || active;
    e.startButton.innerHTML = `<span aria-hidden="true">↗</span> ${state === "complete" ? "Yeniden uçur" : state === "blocked" ? "Kontroller geçilmedi" : (MISSION_TEXT[this.missionType] ?? MISSION_TEXT.recon).start}`;
    e.pauseButton.disabled = !active;
    e.pauseButton.textContent = state === "paused" ? "Devam et" : "Duraklat";
    e.resetButton.disabled = !flyable;
    e.flightProgress.disabled = !planned;
    for (const id of [
      "exportCsvButton",
      "exportKmlButton",
      "exportPlanButton",
      "exportGeojsonButton",
      "exportUlogButton",
    ])
      e[id].disabled = !planned;
    this.state = state;
    this.syncInputs();
    e.connectionStatus.textContent = {
      loading: "Hazırlanıyor",
      ready: "Hazır",
      blocked: "Onay yok",
      running: "Uçuşta",
      paused: "Duraklatıldı",
      complete: "İniş tamam",
      error: "Veri gerekli",
    }[state];
    if (!planned) {
      e.flightPhase.textContent = busy
        ? "Görev hazırlanıyor"
        : "Uçuş hazırlanamadı";
      e.terrainStatus.textContent = busy
        ? "ARAZİ ÖRNEKLENİYOR"
        : "ARAZİ DOĞRULANMADI";
      e.checklistStatus.textContent = busy ? "Hesaplanıyor" : "—";
      e.checklistStatus.className = "status";
    }
    this.state = state;
  }

  setMission(mission, analysis) {
    const e = this.elements;
    this.mission = mission;
    this.analysis = analysis;
    this.setMissionType(mission.type);
    const s = mission.simulation,
      m = s.metrics,
      c = s.airframe,
      b = s.battery,
      cov = analysis.coverage,
      sv = mission.survey,
      faults = s.failsafe?.faults ?? [],
      commands = s.failsafe?.commands ?? [];
    e.legendArea.hidden = !sv;
    e.legendFailsafe.hidden = !s.failsafe?.events.length;
    e.legendRally.hidden = !mission.rally?.length;
    e.legendObstacle.hidden = !mission.obstacles?.items.length;
    e.routeDistance.textContent = formatDistance(mission.horizontalDistance);
    e.routeDuration.textContent = formatTime(mission.duration);
    e.routePhotos.textContent = cov.count
      ? `${cov.count} · ${(cov.gsdMean * 100).toFixed(1)} cm`
      : "—";
    e.routeBattery.textContent = `%${Math.round(m.finalSoc * 100)}`;
    e.routeBattery.classList.toggle("fail", m.finalSoc < b.reserve);
    e.terrainStatus.textContent = `${mission.samples.length} ARAZİ ÖRNEĞİ · WGS84`;
    e.batteryReserve.style.left = `${b.reserve * 100}%`;
    this.setChecklist(analysis.checks);
    this.setEvents(analysis.events);
    const rpm = (w) => Math.round((w * 60) / (2 * Math.PI));
    const row = (k, v) => `<dt>${k}</dt><dd>${v}</dd>`;
    const navigation =
      s.gnss === "standard"
        ? "17 durumlu ESKF · standart GNSS + IMU + baro + manyetometre + sürükleme"
        : s.gnss === "ideal"
          ? "ideal durum"
          : "17 durumlu ESKF · RTK GNSS + IMU + baro + manyetometre + sürükleme";
    e.physicsSummary.innerHTML = `<dl>${[
      row("Çözüm", `RK4 · ${c.physicsHz} Hz / kontrol ${c.controlHz} Hz`),
      row(
        "Kütle / kol",
        `${c.mass.toFixed(1)} kg / ${(c.armLength * 100).toFixed(0)} cm`,
      ),
      row(
        "Atalet Ixx / Iyy / Izz",
        `${c.inertia.map((x) => x.toFixed(3)).join(" / ")} kg·m²`,
      ),
      row("Motor gecikmesi", `${(c.motorTimeConstant * 1000).toFixed(0)} ms`),
      row(
        "Azami rotor / yatış",
        `${rpm(c.maxRotorSpeed)} rpm / ${((c.maxTilt * 180) / Math.PI).toFixed(0)}°`,
      ),
      row(
        "Batarya",
        `${b.cells}S ${b.capacityAh * 1000} mAh · ${Math.round(b.internalResistance * 1000)} mΩ`,
      ),
      row(
        "Kullanılan enerji",
        `${m.energyWh.toFixed(1)} Wh · tepe ${Math.round(m.peakPower)} W`,
      ),
      row("Navigasyon", navigation),
      ...(s.gnss === "ideal"
        ? []
        : [
            row(
              "Yönelim kestirim hatası",
              `eğim ≤ ${((m.maxTiltError * 180) / Math.PI).toFixed(2)}° · yön ≤ ${((m.maxHeadingError * 180) / Math.PI).toFixed(1)}°`,
            ),
          ]),
      row("Kamera", `1″ 20 MP · 8,8 mm · ${analysis.capture.pitchDegrees}°`),
      row(
        "Tetik aralığı",
        `${analysis.capture.spacing.toFixed(1)} m · %${Math.round(analysis.capture.overlap * 100)}`,
      ),
      row(
        "Hava aracı modeli",
        escapeHtml(mission.model ?? "nominal"),
      ),
      row(
        "Görev tipi",
        sv
          ? `alan haritalama · ${(Math.abs(sv.areaM2) / 1e4).toFixed(1)} ha${sv.sortie.count > 1 ? ` · uçuş ${sv.sortie.index + 1}/${sv.sortie.count} (${sv.lines.length} hat)` : ""}`
          : "çevre keşfi · nokta rotası",
      ),
      ...(sv
        ? [
            row(
              "Izgara",
              `${sv.count} hat · ${sv.spacing.toFixed(1)} m aralık · yan iz ${sv.swath.toFixed(0)} m`,
            ),
            row(
              "Yan bindirme",
              `hedef %${Math.round(sv.sideOverlap * 100)} · asgari %${Math.round(cov.minSideOverlap * 100)} · alanın %${(cov.areaCovered * 100).toFixed(1)}'i`,
            ),
          ]
        : []),
      row(
        "Arıza senaryosu",
        faults.length
          ? faults
              .map((f) => `${escapeHtml(FAULTS[f.type]?.label ?? f.type)} · ${formatTime(f.time)}`)
              .join(" + ")
          : "yok · failsafe'ler hazır",
      ),
      ...(commands.length
        ? [
            row(
              "Operatör komutları",
              commands
                .map((c) => `${escapeHtml(OPERATOR_COMMANDS[c.action])} · ${formatTime(c.time)}`)
                .join(" · "),
            ),
          ]
        : []),
      row(
        "Engeller",
        mission.obstacles?.items.length
          ? `${mission.obstacles.items.length} engel (${mission.obstacleData?.snapshot ? `OSM ${mission.obstacleData.timestamp?.slice(0, 10) ?? ""}`.trim() : "rota dosyası"}) · en düşük açıklık ${Number.isFinite(mission.obstacleClearance?.clearance) ? `${Math.round(mission.obstacleClearance.clearance)} m — ${escapeHtml(obstacleText(mission.obstacleClearance.obstacle))}` : "tamponda engel yok"}`
          : "engel verisi yok",
      ),
      row(
        "Güvenli noktalar",
        `kalkış${mission.rally?.length ? ` + ${mission.rally.length} rally (${mission.rally.map((r) => escapeHtml(r.name)).join(", ")})` : ""}`,
      ),
      row(
        "Ortalama rüzgâr",
        `${s.wind.toFixed(1)} m/s · yetki sınırı ${windLimit(c).toFixed(1)} m/s`,
      ),
      row(
        "Dryden türbülansı",
        s.turbulence ? `açık · tohum ${s.seed}` : "kapalı",
      ),
      row("En büyük takip hatası", `${m.maxTrackingError.toFixed(2)} m`),
      row("İniş konum hatası", `${(m.landingError * 100).toFixed(1)} cm`),
      row("Temas hızı", `${m.contactSpeed.toFixed(2)} m/s`),
      row("Hareket bulanıklığı", `${cov.maxBlur.toFixed(2)} px`),
    ].join(
      "",
    )}</dl><p>MIL-F-8785C Dryden türbülansı, yer etkisi, rotor sürüklemesi ve pala çırpması, VRS, sabit hava yoğunluğu, genel LiPo eğrisi ve noktasal zemin teması. Gerçek araç için tanımlanmış katsayılar veya PX4 SITL değildir.</p>`;
  }

  // Plan checks first, then the injected fault's outcome under its own
  // heading; the status counts the plan, with the scenario after it.
  setChecklist(checks) {
    const e = this.elements,
      plan = checks.filter((c) => c.group !== "fault"),
      scenario = checks.filter((c) => c.group === "fault"),
      failed = plan.filter((c) => !c.ok && c.blocking).length,
      warned = plan.filter((c) => !c.ok && !c.blocking).length,
      faultWarned = scenario.filter((c) => !c.ok).length;
    const item = (c) =>
      `<li class="${c.ok ? "ok" : c.blocking ? "fail" : "warn"}" data-check="${c.id}"><span>${escapeHtml(c.label)}</span><small>${escapeHtml(c.detail)}</small></li>`;
    e.checklist.innerHTML =
      plan.map(item).join("") +
      (scenario.length
        ? `<li class="group" data-group="fault">Arıza senaryosu · arızalı çözüm</li>${scenario.map(item).join("")}`
        : "");
    e.checklistStatus.textContent =
      (failed ? `${failed} engel` : warned ? `${warned} uyarı` : "Tümü geçti") +
      (scenario.length
        ? ` · senaryo ${faultWarned ? `${faultWarned} uyarı` : "temiz"}`
        : "");
    e.checklistStatus.className = `status${failed ? " fail" : warned || faultWarned ? " warn" : ""}`;
  }

  setEvents(events) {
    this.events = events;
    this.elements.eventLog.innerHTML = events
      .map(
        (ev) =>
          `<li data-time="${ev.time}"${ev.kind && ev.kind !== "info" ? ` class="${ev.kind}"` : ""}><time>${formatTime(ev.time)}</time><span>${escapeHtml(ev.label)}</span><small>${escapeHtml(ev.detail)}</small></li>`,
      )
      .join("");
    this.lastEventIndex = -2;
  }

  setCamera(mode) {
    for (const name of ["overview", "follow", "gimbal"]) {
      const button = this.elements[`${name}Button`];
      button.classList.toggle("active", mode === name);
      button.setAttribute("aria-pressed", String(mode === name));
    }
    this.elements.followZoomControls.hidden = mode !== "follow";
    this.elements.gimbalHud.hidden = mode !== "gimbal";
    const area = this.elements.cesiumContainer.closest(".map-area");
    area.classList.toggle("follow-camera", mode === "follow");
    area.classList.toggle("gimbal-camera", mode === "gimbal");
  }

  setFollowDistance(distance) {
    this.elements.followDistance.value = distance.toFixed(1);
    this.elements.followDistanceValue.textContent = `${distance.toFixed(1)} m`;
  }

  setFollowView(view) {
    document.querySelectorAll("[data-follow-view]").forEach((button) => {
      const selected = button.dataset.followView === view;
      button.classList.toggle("active", selected);
      button.setAttribute("aria-pressed", String(selected));
    });
  }

  // `payload` is { taken, total, last, gsd, pitchDegrees } at this time.
  render(sample, C, payload) {
    const e = this.elements;
    const phase = ["ready", "blocked"].includes(this.state)
      ? this.state
      : sample.phase;
    const deg = (x) => C.Math.toDegrees(x);
    e.altitudeStatus.textContent = `${sample.agl.toFixed(1)} m`;
    e.speedStatus.textContent = `${(sample.groundSpeed * 3.6).toFixed(1)} km/h`;
    e.absoluteAltitude.textContent = `${sample.position.height.toFixed(1)} m`;
    e.mslAltitude.textContent = Number.isFinite(sample.mslAltitude)
      ? `${sample.mslAltitude.toFixed(1)} m`
      : "—";
    e.relativeAltitude.textContent = `${sample.relativeAltitude.toFixed(1)} m`;
    e.verticalSpeed.textContent = `${sample.verticalSpeed.toFixed(1)} m/s`;
    e.rollStatus.textContent = `${deg(sample.roll).toFixed(1)}°`;
    e.pitchStatus.textContent = `${deg(sample.pitch).toFixed(1)}°`;
    e.rateStatus.textContent = `${deg(Math.hypot(...sample.omega)).toFixed(1)}°/s`;
    e.thrustStatus.textContent = `${sample.thrust.toFixed(1)} N`;
    e.trackingStatus.textContent = `${sample.trackingError.toFixed(2)} m`;
    e.windStatus.textContent = `${Math.hypot(...sample.wind).toFixed(1)} m/s`;
    // Meteorological convention: the direction the wind blows from.
    const [we, wn] = sample.windEstimate,
      from = (((Math.atan2(-we, -wn) * 180) / Math.PI + 360) % 360).toFixed(0);
    e.windEstimateStatus.textContent =
      Math.hypot(we, wn) < 0.05 ? "sakin" : `${Math.hypot(we, wn).toFixed(1)} m/s · ${from}°`;
    e.motorStatus.textContent = sample.motors
      .map((w, i) => `M${i + 1} ${Math.round((w * 60) / (2 * Math.PI))}`)
      .join("  ·  ");
    const heading = Math.round(deg(sample.heading)) % 360;
    e.headingStatus.textContent = `${heading}°`;
    e.horizonGroup.setAttribute(
      "transform",
      `rotate(${-deg(sample.roll)} 60 57) translate(0 ${deg(sample.pitch) * 1.3})`,
    );
    e.horizonHeading.textContent = `${heading}° · ${deg(sample.roll).toFixed(1)}° roll`;
    // Hemisphere suffix follows the sign so southern/western routes read right.
    const lat = deg(sample.position.latitude),
      lon = deg(sample.position.longitude);
    e.coordinates.textContent = `${Math.abs(lat).toFixed(6)}° ${lat >= 0 ? "K" : "G"} / ${Math.abs(lon).toFixed(6)}° ${lon >= 0 ? "D" : "B"}`;
    // Payload.
    e.photoStatus.textContent = `${payload.taken} / ${payload.total}`;
    e.gsdStatus.textContent =
      sample.agl > 1 ? `${(payload.gsd * 100).toFixed(2)} cm` : "—";
    // Navigation; an injected GNSS fault shows as dead reckoning.
    const gnss = this.mission.simulation.gnss,
      denied = (this.mission.simulation.failsafe?.faults ?? []).some(
        (f) =>
          f.type.startsWith("gnss") &&
          sample.elapsed >= f.time &&
          sample.elapsed < f.time + (f.duration ?? Infinity),
      );
    e.gnssStatus.textContent = denied
      ? "FIX YOK · ölü hesap"
      : gnss === "rtk"
        ? "RTK FIX · 24 uydu"
        : gnss === "standard"
          ? "3D FIX · 16 uydu"
          : "ideal";
    e.gnssStatus.classList.toggle("alert", Boolean(denied));
    e.navStatus.textContent = `±${(sample.eph * 100).toFixed(0)} cm · hata ${(sample.navigationError * 100).toFixed(0)} cm`;
    // Battery.
    const battery = sample.battery,
      reserve = this.mission.simulation.battery.reserve,
      soc = Math.max(0, battery.soc);
    e.batteryPercent.textContent = `%${(soc * 100).toFixed(0)}`;
    e.batteryFill.style.width = `${soc * 100}%`;
    e.batteryFill.className =
      soc < reserve ? "critical" : soc < reserve + 0.15 ? "low" : "";
    e.batteryStatus.textContent = `${battery.voltage.toFixed(2)} V · ${battery.current.toFixed(1)} A · ${Math.round(battery.power)} W · rezerve kadar ${formatTime(sample.remainingTime)}`;
    // Flight bar and phases.
    const failsafe = ["ready", "blocked"].includes(this.state)
      ? null
      : sample.failsafe;
    // An area survey's imaging phase is a grid line, not reconnaissance.
    // A failsafe leg flies to home, a rally point or a clear landing site.
    const site = this.mission.simulation.metrics.landingSite,
      name =
        phase === "survey" && this.mission.survey
          ? "Alan haritalama · ızgara hattı"
          : phase === "rtl" && site === "rally"
            ? "Rally noktasına dönüş · RTL"
            : phase === "rtl" && site === "field"
              ? "Engelsiz iniş yerine geçiş"
              : PHASES[phase];
    const label = failsafe ? `${name} — ⚠ ${failsafe}` : name;
    e.flightPhase.textContent =
      this.state === "paused" ? `Duraklatıldı · ${label}` : label;
    e.flightPhase.classList.toggle("failsafe", Boolean(failsafe));
    e.flightTime.textContent = `${formatTime(sample.elapsed)} / ${formatTime(this.mission.duration)}`;
    e.flightProgress.value = String(Math.round(sample.progress * 1000));
    e.flightDistance.textContent = `${formatDistance(sample.traveled)} kat edildi`;
    const activePhase =
      phase === "arming"
        ? "takeoff"
        : ["settle", "touchdown", "disarm", "complete"].includes(phase)
          ? "landing"
          : ["transit", "hold"].includes(phase)
            ? "survey"
            : phase === "rtl"
              ? "return"
              : phase;
    document
      .querySelectorAll("[data-phase]")
      .forEach((el) =>
        el.classList.toggle("active", el.dataset.phase === activePhase),
      );
    // Flight log: events up to now, the latest marked current.
    let current = -1;
    for (let i = 0; i < this.events.length; i++)
      if (this.events[i].time <= sample.elapsed + 1e-6) current = i;
    if (current !== this.lastEventIndex) {
      [...e.eventLog.children].forEach((li, i) => {
        li.classList.toggle("done", i <= current);
        li.classList.toggle("current", i === current);
      });
      this.lastEventIndex = current;
    }
    // Gimbal HUD.
    if (!e.gimbalHud.hidden) {
      e.hudPhoto.textContent = `IMG ${String(payload.taken).padStart(4, "0")} / ${String(payload.total).padStart(4, "0")}`;
      e.hudRec.classList.toggle(
        "flash",
        payload.last !== null && sample.elapsed - payload.last < 0.35,
      );
      e.hudInfo.textContent = `AGL ${sample.agl.toFixed(1)} m · GSD ${(payload.gsd * 100).toFixed(2)} cm · ψ ${heading}° · θ ${payload.pitchDegrees}° · ${(sample.groundSpeed * 3.6).toFixed(0)} km/h`;
    }
  }
}
