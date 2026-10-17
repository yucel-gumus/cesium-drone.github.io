const PHASES = {
  ready: "Kalkışa hazır",
  blocked: "Görev onaylanmadı",
  arming: "Motorlar hazırlanıyor",
  takeoff: "Dikey kalkış",
  survey: "Keşif uçuşu · araziyi izliyor",
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
    ])
      e[id].addEventListener("click", () => handlers.export(kind));
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
  }

  planInputs() {
    const e = this.elements;
    return [e.aglInput, e.massInput, e.windInput, e.seedInput, e.overlapInput];
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
    e.startButton.innerHTML = `<span aria-hidden="true">↗</span> ${state === "complete" ? "Yeniden uçur" : state === "blocked" ? "Kontroller geçilmedi" : "Keşif uçuşunu başlat"}`;
    e.pauseButton.disabled = !active;
    e.pauseButton.textContent = state === "paused" ? "Devam et" : "Duraklat";
    e.resetButton.disabled = !flyable;
    e.flightProgress.disabled = !planned;
    for (const id of [
      "exportCsvButton",
      "exportKmlButton",
      "exportPlanButton",
      "exportGeojsonButton",
    ])
      e[id].disabled = !planned;
    for (const input of [
      ...this.planInputs(),
      e.turbulenceInput,
      e.gimbalInput,
      e.gnssInput,
      e.planButton,
      e.tokenButton,
      e.ionToken,
    ])
      input.disabled = busy || active;
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
    const s = mission.simulation,
      m = s.metrics,
      c = s.airframe,
      b = s.battery,
      cov = analysis.coverage;
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
        ? "Standart GNSS + IMU + baro"
        : s.gnss === "ideal"
          ? "ideal durum"
          : "RTK GNSS + IMU + baro";
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
      row("Kamera", `1″ 20 MP · 8,8 mm · ${analysis.capture.pitchDegrees}°`),
      row(
        "Tetik aralığı",
        `${analysis.capture.spacing.toFixed(1)} m · %${Math.round(analysis.capture.overlap * 100)}`,
      ),
      row("Ortalama rüzgâr", `${s.wind.toFixed(1)} m/s`),
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
    )}</dl><p>MIL-F-8785C Dryden türbülansı, sabit hava yoğunluğu, ideal AHRS, genel LiPo eğrisi ve noktasal zemin teması. Gerçek araç için tanımlanmış katsayılar veya PX4 SITL değildir.</p>`;
  }

  setChecklist(checks) {
    const e = this.elements,
      failed = checks.filter((c) => !c.ok && c.blocking).length,
      warned = checks.filter((c) => !c.ok && !c.blocking).length;
    e.checklist.innerHTML = checks
      .map(
        (c) =>
          `<li class="${c.ok ? "ok" : c.blocking ? "fail" : "warn"}" data-check="${c.id}"><span>${escapeHtml(c.label)}</span><small>${escapeHtml(c.detail)}</small></li>`,
      )
      .join("");
    e.checklistStatus.textContent = failed
      ? `${failed} engel`
      : warned
        ? `${warned} uyarı`
        : "Tümü geçti";
    e.checklistStatus.className = `status${failed ? " fail" : warned ? " warn" : ""}`;
  }

  setEvents(events) {
    this.events = events;
    this.elements.eventLog.innerHTML = events
      .map(
        (ev) =>
          `<li data-time="${ev.time}"><time>${formatTime(ev.time)}</time><span>${escapeHtml(ev.label)}</span><small>${escapeHtml(ev.detail)}</small></li>`,
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
    // Navigation.
    const gnss = this.mission.simulation.gnss;
    e.gnssStatus.textContent =
      gnss === "rtk"
        ? "RTK FIX · 24 uydu"
        : gnss === "standard"
          ? "3D FIX · 16 uydu"
          : "ideal";
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
    e.flightPhase.textContent =
      this.state === "paused"
        ? `Duraklatıldı · ${PHASES[phase]}`
        : PHASES[phase];
    e.flightTime.textContent = `${formatTime(sample.elapsed)} / ${formatTime(this.mission.duration)}`;
    e.flightProgress.value = String(Math.round(sample.progress * 1000));
    e.flightDistance.textContent = `${formatDistance(sample.traveled)} kat edildi`;
    const activePhase =
      phase === "arming"
        ? "takeoff"
        : ["settle", "touchdown", "disarm", "complete"].includes(phase)
          ? "landing"
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
