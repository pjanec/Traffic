// Sim.Viz front-end -- committed static template (VIZ_SPEC.md "Architecture"). Vanilla JS,
// Canvas 2D, no external libraries. Inlined verbatim into the replay HTML by Sim.Viz/Program.cs
// (the `/*TEMPLATE_JS*/` marker in template.html), right after a `const REPLAY_DATA = {...}` block.
//
// REPLAY_DATA is a UNIFIED MULTI-SCENE payload: `{ scenes: [ SCENE, ... ] }`. Each SCENE carries a
// camera view box, an OPTIONAL network (null for pure open-space crowd scenes), a single shared
// vehicle box dim, a timestep, and per-frame lists of vehicles (oriented boxes) and discs
// (crowd/pedestrian agents). A scene selector switches between them. The single-scenario export
// wraps its one scenario as a one-element scenes array, so this same code path serves both.
(function () {
  "use strict";

  // On-screen error surface: a black replay on a device we can't debug remotely is useless, so
  // any uncaught error paints its message over the canvas instead of failing silently.
  window.addEventListener("error", function (e) {
    var d = document.getElementById("vizError");
    if (!d) {
      d = document.createElement("div");
      d.id = "vizError";
      d.style.cssText =
        "position:fixed;left:8px;right:8px;top:8px;z-index:9999;background:rgba(120,20,20,0.95);" +
        "color:#fff;font:12px/1.4 monospace;padding:8px 10px;border-radius:8px;white-space:pre-wrap;";
      document.body.appendChild(d);
    }
    d.textContent =
      "viz error: " + (e.message || e.type || "unknown") +
      (e.filename ? "\n@ " + e.filename + ":" + e.lineno + ":" + e.colno : "");
  });

  var data = REPLAY_DATA;
  var scenes = (data && data.scenes) || [];

  // ---------------------------------------------------------------------
  // DOM
  // ---------------------------------------------------------------------
  var canvas = document.getElementById("cv");
  var ctx = canvas.getContext("2d");
  var wrap = document.getElementById("canvasWrap");
  var legendEl = document.getElementById("legend");
  var vehCountEl = document.getElementById("vehCount");
  var captionEl = document.getElementById("caption");
  var sceneSel = document.getElementById("sceneSel");
  var btnPlay = document.getElementById("btnPlay");
  var btnRestart = document.getElementById("btnRestart");
  var speedSel = document.getElementById("speedSel");
  var speedColorToggle = document.getElementById("speedColorToggle");
  var timeReadout = document.getElementById("timeReadout");
  var timeSlider = document.getElementById("timeSlider");

  // ---------------------------------------------------------------------
  // Palettes
  // ---------------------------------------------------------------------
  // Vehicles in the unified payload share one box dim per scene and render in a single colour
  // (the classic passenger blue). Speed colouring (derived from motion) is available via the HUD.
  var VEHICLE_COLOR = "#4f8ef7";
  // Discs (crowd/pedestrian agents) coloured by kind.
  var DISC_COLORS = {
    0: "#38bdf8", 1: "#fb7185", 2: "#c084fc", 3: "#f59e0b",
    4: "#38bdf8", // fleeing pedestrian (panic evac)
    5: "#34d399", // escaped pedestrian (panic evac)
    6: "#b91c1c", // abandoned car (panic evac)
    8: "#fb923c", // car pushing onto the shoulder / abandoning the lane (panic evac, phase 3)
  };
  var DISC_LABELS = { 0: "stream / agent A", 1: "stream / agent B", 2: "pedestrian" };

  // Static POI "places" layer (scene.pois, optional -- emitted by sim_viz --pois).
  // One glyph + colour per POI kind (deduce_pois.py kinds), drawn UNDER the moving
  // agents so cars/peds always sit on top. Colours are chosen distinct from the
  // vehicle blue (#4f8ef7) and pedestrian purple (#c084fc).
  //   venue             -> filled square    amber
  //   building_entrance -> house/triangle   green
  //   dwell_spot        -> circle           pink
  //   transit_stop      -> diamond          cyan
  //   parking_access    -> "P" glyph        light-blue
  var POI_STYLE = {
    venue:             { color: "#f59e0b", glyph: "square",   label: "venue" },
    building_entrance: { color: "#34d399", glyph: "triangle", label: "building entrance" },
    dwell_spot:        { color: "#f472b6", glyph: "circle",   label: "dwell spot" },
    transit_stop:      { color: "#22d3ee", glyph: "diamond",  label: "transit stop" },
    parking_access:    { color: "#93c5fd", glyph: "P",        label: "parking access" },
  };
  var POI_DEFAULT_STYLE = { color: "#9ca3af", glyph: "circle", label: "place" };
  // Kinds whose text labels are shown when zoomed in (parking_access omitted --
  // there are many and the "P" glyph already reads clearly).
  var POI_LABEL_KINDS = { venue: true, building_entrance: true, dwell_spot: true, transit_stop: true };

  // Stage 4 (docs/SUBAREA-DEMO-CITY-DESIGN.md sec 5) static polygon layers -- district
  // zone tints (scene.zones, optional -- sim_viz --zones), building footprints
  // (scene.buildings, optional -- sim_viz --buildings), and the parking-lot / park /
  // meet-area polygons already carried in scene.pois' derived scene.parkingLots /
  // scene.parks (sim_viz --pois pois/v2 records). All drawn UNDER the crossings/POI
  // glyphs/agents so the moving city always reads on top of its static backdrop.
  var ZONE_FILL = {
    downtown:    "rgba(148,163,184,0.14)",
    retail:      "rgba(245,158,11,0.12)",
    dining:      "rgba(244,114,182,0.12)",
    residential: "rgba(96,165,250,0.12)",
    park:        "rgba(34,197,94,0.12)",
    arterial:    "rgba(148,163,184,0.05)",
  };
  var ZONE_FILL_DEFAULT = "rgba(156,163,175,0.10)";
  var ZONE_LABELS = {
    downtown: "downtown", retail: "retail/mall", dining: "dining quarter",
    residential: "residential", park: "park zone", arterial: "arterial ring",
  };

  var PARK_FILL = "rgba(34,197,94,0.30)";
  var PARK_STROKE = "rgba(34,197,94,0.65)";
  var PARKING_LOT_FILL = "rgba(148,163,184,0.35)";
  var PARKING_LOT_STROKE = "rgba(203,213,225,0.7)";
  var MEET_AREA_COLOR = "#facc15";

  // Building footprint fill by buildings/v1 `type`.
  var BUILDING_FILL = {
    mall:        "rgba(245,158,11,0.65)",
    office:      "rgba(59,130,246,0.6)",
    residential: "rgba(45,212,191,0.55)",
    restaurant:  "rgba(248,113,113,0.65)",
    garage:      "rgba(107,114,128,0.7)",
  };
  var BUILDING_FILL_DEFAULT = "rgba(156,163,175,0.55)";
  var BUILDING_LABELS = {
    mall: "mall", office: "office", residential: "house",
    restaurant: "restaurant", garage: "garage",
  };

  // Speed heatmap: cold (slow) -> hot (fast), 0..cap m/s.
  var SPEED_COLOR_CAP = 20.0;
  function colorForSpeed(speed) {
    var t = Math.max(0, Math.min(1, speed / SPEED_COLOR_CAP));
    var r, g, b;
    if (t < 0.5) {
      var u = t / 0.5;
      r = Math.round(59 + u * (250 - 59));
      g = Math.round(130 + u * (204 - 130));
      b = Math.round(246 + u * (21 - 246));
    } else {
      var v = (t - 0.5) / 0.5;
      r = Math.round(250 + v * (220 - 250));
      g = Math.round(204 + v * (38 - 204));
      b = Math.round(21 + v * (38 - 21));
    }
    return "rgb(" + r + "," + g + "," + b + ")";
  }

  // Fear ramp (panic-evac): calm blue -> panic red, drives the default vehicle-box fill whenever a
  // frame entry carries a 4th (fear) element -- see interpolatedVehicles / drawVehicle.
  function fearColor(f) {
    var t = Math.max(0, Math.min(1, f));
    // calm blue (#4f8ef7 = 79,142,247) -> panic red (#ef4444 = 239,68,68)
    var r = Math.round(79 + t * (239 - 79));
    var g = Math.round(142 + t * (68 - 142));
    var b = Math.round(247 + t * (68 - 247));
    return "rgb(" + r + "," + g + "," + b + ")";
  }

  // ---------------------------------------------------------------------
  // Camera (VIZ_SPEC.md "Coordinate & camera model")
  // screenX = worldX * scale + offsetX
  // screenY = -worldY * scale + offsetY   (Y flipped: canvas-down vs. SUMO-up)
  // ---------------------------------------------------------------------
  var camera = { scale: 1, offsetX: 0, offsetY: 0 };

  function worldToScreen(wx, wy) {
    return [wx * camera.scale + camera.offsetX, -wy * camera.scale + camera.offsetY];
  }

  function screenToWorld(sx, sy) {
    return [(sx - camera.offsetX) / camera.scale, -(sy - camera.offsetY) / camera.scale];
  }

  // ---------------------------------------------------------------------
  // Per-scene state (set by loadScene)
  // ---------------------------------------------------------------------
  var scene = null;        // the active SCENE object
  var frames = [];         // scene.frames
  var network = null;      // scene.network (may be null)
  var vdim = [4.3, 1.8];   // [length, width]
  var stepSize = 1.0;      // scene.dt
  var simStart = 0, simEnd = 0, simTime = 0;
  var lanesById = {}, lanesByEdge = {}, laneMarkings = [], tlsById = {};

  // Set once the user manually zooms/pans, so auto-fit stops fighting them.
  var userAdjusted = false;

  // Offset a flat [x0,y0,...] polyline perpendicular to travel by `dist` (positive = LEFT).
  function offsetPolyline(flatShape, dist) {
    var out = [];
    var n = flatShape.length / 2;
    for (var i = 0; i < n - 1; i++) {
      var x1 = flatShape[i * 2], y1 = flatShape[i * 2 + 1];
      var x2 = flatShape[(i + 1) * 2], y2 = flatShape[(i + 1) * 2 + 1];
      var dx = x2 - x1, dy = y2 - y1;
      var len = Math.sqrt(dx * dx + dy * dy) || 1;
      var nx = (-dy / len) * dist;
      var ny = (dx / len) * dist;
      out.push([x1 + nx, y1 + ny, x2 + nx, y2 + ny]);
    }
    return out;
  }

  function cycleLength(tl) {
    var sum = 0;
    for (var i = 0; i < tl.phases.length; i++) sum += tl.phases[i].duration;
    return sum;
  }

  // Mirrors Sim.Core/TrafficLightState.cs GetLinkState exactly.
  function tlLinkState(tl, linkIndex, simT) {
    var cl = cycleLength(tl);
    if (cl <= 0) return "o";
    var position = (simT - tl.offset) % cl;
    if (position < 0) position += cl;
    var cumulative = 0;
    for (var i = 0; i < tl.phases.length; i++) {
      cumulative += tl.phases[i].duration;
      if (position < cumulative) return tl.phases[i].state[linkIndex];
    }
    return tl.phases[tl.phases.length - 1].state[linkIndex];
  }

  function colorForSignalState(ch) {
    switch (ch) {
      case "G": case "g": return "#2ecc71";
      case "y": case "Y": return "#f1c40f";
      case "r": return "#e74c3c";
      case "u": return "#e08a2f";
      case "o": case "O": return "#555555";
      default: return "#888888";
    }
  }

  // Recompute the per-scene network derivations (lane grouping, lane markings, TL index). Cleanly
  // no-ops when the scene has no network (pure open-space crowd scenes).
  function precomputeNetwork() {
    lanesById = {};
    lanesByEdge = {};
    laneMarkings = [];
    tlsById = {};
    crossingLanes = [];
    if (!network) return;

    network.lanes.forEach(function (lane) {
      lanesById[lane.id] = lane;
      (lanesByEdge[lane.edgeId] = lanesByEdge[lane.edgeId] || []).push(lane);
      if (CROSSING_RE.test(lane.edgeId || "")) crossingLanes.push(lane);
    });
    Object.keys(lanesByEdge).forEach(function (edgeId) {
      lanesByEdge[edgeId].sort(function (a, b) { return a.index - b.index; });
    });

    // Lane markings: dashed line on the LEFT edge of any lane that has a left neighbour.
    Object.keys(lanesByEdge).forEach(function (edgeId) {
      var lanes = lanesByEdge[edgeId];
      for (var i = 0; i < lanes.length; i++) {
        var lane = lanes[i];
        var hasLeftNeighbor = lanes.some(function (l) { return l.index === lane.index + 1; });
        if (!hasLeftNeighbor) continue;
        offsetPolyline(lane.shape, lane.width / 2).forEach(function (s) { laneMarkings.push(s); });
      }
    });

    (network.tls || []).forEach(function (tl) { tlsById[tl.id] = tl; });
  }

  function fitToView() {
    // Prefer the scene's declared view box; fall back to the network extent if absent.
    var view = scene && scene.view;
    var minX, minY, maxX, maxY;
    if (view && view.length === 4) {
      minX = view[0]; minY = view[1]; maxX = view[2]; maxY = view[3];
    } else {
      minX = -1; minY = -1; maxX = 1; maxY = 1;
    }
    var bw = Math.max(1e-6, maxX - minX);
    var bh = Math.max(1e-6, maxY - minY);
    var cw = canvas.clientWidth || 300;
    var ch = canvas.clientHeight || 300;
    var margin = 0.9;
    var scale = Math.min(cw / bw, ch / bh) * margin;
    var wcx = (minX + maxX) / 2;
    var wcy = (minY + maxY) / 2;
    camera.scale = scale;
    camera.offsetX = cw / 2 - wcx * scale;
    camera.offsetY = ch / 2 + wcy * scale;
  }

  function zoomAt(screenX, screenY, factor) {
    userAdjusted = true;
    var before = screenToWorld(screenX, screenY);
    camera.scale = Math.max(0.02, Math.min(4000, camera.scale * factor));
    camera.offsetX = screenX - before[0] * camera.scale;
    camera.offsetY = screenY + before[1] * camera.scale;
  }

  function panBy(dxScreen, dyScreen) {
    userAdjusted = true;
    camera.offsetX += dxScreen;
    camera.offsetY += dyScreen;
  }

  // ---------------------------------------------------------------------
  // Canvas sizing (devicePixelRatio-aware) -- mobile hardening preserved verbatim.
  // ---------------------------------------------------------------------
  var dpr = Math.max(1, window.devicePixelRatio || 1);
  function resizeCanvas() {
    var cw = wrap.clientWidth;
    var ch = wrap.clientHeight;
    // Skip while the layout has no measurable size yet (iOS Safari can report 0 height). Called
    // every frame until the user takes control, so it self-corrects the moment a real size appears.
    if (cw <= 0 || ch <= 0) return;
    var needW = Math.max(1, Math.round(cw * dpr));
    var needH = Math.max(1, Math.round(ch * dpr));
    if (canvas.width !== needW || canvas.height !== needH) {
      canvas.width = needW;
      canvas.height = needH;
      canvas.style.width = cw + "px";
      canvas.style.height = ch + "px";
    }
    if (!userAdjusted) fitToView();
  }
  window.addEventListener("resize", resizeCanvas);
  if (window.visualViewport) window.visualViewport.addEventListener("resize", resizeCanvas);
  if (typeof ResizeObserver !== "undefined") new ResizeObserver(resizeCanvas).observe(wrap);

  // ---------------------------------------------------------------------
  // Pointer input: wheel zoom, drag pan, touch pinch/pan (CSS-pixel coords).
  // ---------------------------------------------------------------------
  canvas.addEventListener("wheel", function (ev) {
    ev.preventDefault();
    var rect = canvas.getBoundingClientRect();
    zoomAt(ev.clientX - rect.left, ev.clientY - rect.top, Math.exp(-ev.deltaY * 0.0015));
  }, { passive: false });

  var dragging = false, lastX = 0, lastY = 0;
  canvas.addEventListener("mousedown", function (ev) {
    if (ev.button !== 0) return;
    dragging = true; lastX = ev.clientX; lastY = ev.clientY;
  });
  window.addEventListener("mousemove", function (ev) {
    if (!dragging) return;
    panBy(ev.clientX - lastX, ev.clientY - lastY);
    lastX = ev.clientX; lastY = ev.clientY;
  });
  window.addEventListener("mouseup", function () { dragging = false; });

  var touchState = null;
  canvas.addEventListener("touchstart", function (ev) {
    ev.preventDefault();
    var rect = canvas.getBoundingClientRect();
    if (ev.touches.length === 1) {
      touchState = { mode: "pan", x: ev.touches[0].clientX, y: ev.touches[0].clientY };
    } else if (ev.touches.length >= 2) {
      var t0 = ev.touches[0], t1 = ev.touches[1];
      var dx = t1.clientX - t0.clientX, dy = t1.clientY - t0.clientY;
      touchState = {
        mode: "pinch",
        dist: Math.sqrt(dx * dx + dy * dy),
        midX: (t0.clientX + t1.clientX) / 2 - rect.left,
        midY: (t0.clientY + t1.clientY) / 2 - rect.top,
      };
    }
  }, { passive: false });
  canvas.addEventListener("touchmove", function (ev) {
    ev.preventDefault();
    if (!touchState) return;
    var rect = canvas.getBoundingClientRect();
    if (touchState.mode === "pan" && ev.touches.length === 1) {
      panBy(ev.touches[0].clientX - touchState.x, ev.touches[0].clientY - touchState.y);
      touchState.x = ev.touches[0].clientX; touchState.y = ev.touches[0].clientY;
    } else if (touchState.mode === "pinch" && ev.touches.length >= 2) {
      var t0 = ev.touches[0], t1 = ev.touches[1];
      var ddx = t1.clientX - t0.clientX, ddy = t1.clientY - t0.clientY;
      var dist = Math.sqrt(ddx * ddx + ddy * ddy);
      var midX = (t0.clientX + t1.clientX) / 2 - rect.left;
      var midY = (t0.clientY + t1.clientY) / 2 - rect.top;
      zoomAt(midX, midY, dist / (touchState.dist || dist));
      touchState.dist = dist; touchState.midX = midX; touchState.midY = midY;
    }
  }, { passive: false });
  canvas.addEventListener("touchend", function (ev) {
    if (ev.touches.length === 0) touchState = null;
    else if (ev.touches.length === 1) touchState = { mode: "pan", x: ev.touches[0].clientX, y: ev.touches[0].clientY };
  });

  // ---------------------------------------------------------------------
  // Playback clock (VIZ_SPEC.md "Real-time clock")
  // ---------------------------------------------------------------------
  var prefersReduced = window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  var playing = !prefersReduced;   // respect prefers-reduced-motion: start paused
  var speedMultiplier = 1;
  var sliderDragging = false;
  var wasPlayingBeforeDrag = playing;

  function frameIndexAt(t) {
    if (frames.length <= 1) return 0;
    var idx = Math.floor(t / stepSize);
    return Math.max(0, Math.min(frames.length - 2, idx));
  }

  // ---------------------------------------------------------------------
  // Kinematic single-track (bicycle) vehicle reconstruction -- ported VERBATIM from SumoSharp
  // src/Sim.Viewer.Motion/KinematicHeading.cs (main commit c628224, "kinematic vehicle-motion
  // smoothing"; params are the fine-tuned + approved defaults there -- do NOT retune here).
  //
  // Two render-side stages, replacing the previous per-vehicle interpolation wholesale:
  //   Stage A -- linear interpolation between the two bracketing frames gives the RAW front pose
  //              (position + reported-heading + speed). [done inline in interpolatedVehicles]
  //   Stage B (khUpdate) -- the bicycle model: (0) absorb SUMO's instantaneous lane-change lateral
  //              snap into a bounded, decaying lateral error so the drawn front never jumps; (1)
  //              predict the front advancing along a low-passed lane DIRECTION and correct toward the
  //              (error-removed) input with a critically-damped position gain (de-facets the lane
  //              staircase, follows the arc through a turn); tow a NO-SLIP rear axle a fixed wheelbase
  //              behind the front in 8 sub-steps -- the rear->front vector IS the body heading, so the
  //              rear off-tracks inside the corner like a real car; hold heading below HoldSpeed (no
  //              idle spin at a stop); reseed on a teleport. Anticipatory turn-in and the output
  //              heading low-pass are disabled in the shipped config (kept for fidelity).
  // Stateful per vehicle slot (slots are stable per scene). Advanced once per render by the sim-time
  // delta; on a scrub / backward / first-frame jump the whole state is cleared so it snaps crisply.
  var KHP = {
    wheelbaseFactor: 0.6, holdSpeed: 0.5, reseedJump: 7.0,
    positionSmoothTime: 0.60, lanePredictSmoothTime: 0.18, headingSmoothTime: 0.0,
    laneChangeDecayTau: 2.0, laneChangeSnap: 1.5, laneChangeErrorCap: 3.4,
  };
  var khState = {};         // slot index -> per-vehicle bicycle-model state
  var khDt = null;          // raw sim-time delta since last render (set by render())
  var khDtEff = 1 / 60;     // clamped positive dt fed to khUpdate (set by render())
  var khPrevSimT = null;
  function khClear() { khState = {}; }
  function resetVehSmoothing() { khClear(); khPrevSimT = null; }

  // navi-degree (0 = north/+Y, clockwise) -> unit vector; == C# KinematicHeading.Dir().
  function khDir(naviDeg) {
    var m = ((90 - naviDeg) * Math.PI) / 180;
    return [Math.cos(m), Math.sin(m)];
  }
  // world motion vector -> navi-degree; == C# Navi() (same as headingFromDelta).
  function khNavi(dx, dy) {
    var deg = 90 - (Math.atan2(dy, dx) * 180) / Math.PI;
    deg = deg % 360;
    return deg < 0 ? deg + 360 : deg;
  }
  // Unity-style critically-damped smoothing toward `tgt`; C1, no overshoot. `st[velKey]` is the
  // persisted velocity. (Only exercised when headingSmoothTime>0; kept faithful.)
  function khSmoothDamp(cur, tgt, st, velKey, smoothTime, dt) {
    if (smoothTime <= 1e-6) { st[velKey] = 0; return tgt; }
    var omega = 2 / smoothTime, x = omega * dt;
    var e = 1 / (1 + x + 0.48 * x * x + 0.235 * x * x * x);
    var change = cur - tgt;
    var temp = (st[velKey] + omega * change) * dt;
    st[velKey] = (st[velKey] - omega * temp) * e;
    return tgt + (change + temp) * e;
  }

  // Advance one vehicle slot one render step; returns { x, y, deg } = smoothed FRONT + body heading.
  // frontX,frontY = raw (Stage-A) lane front; laneHeadingDeg = raw reported heading; speed m/s;
  // length m; dt sim-seconds (clamped positive). Verbatim transliteration of KinematicHeading.Update.
  function khUpdate(slot, frontX, frontY, laneHeadingDeg, speed, length, dt) {
    var p = KHP, lwb = p.wheelbaseFactor * (length || 4.3);
    var s = khState[slot];
    if (!s) {
      s = { Init: false, Ex: 0, Ey: 0, Fx: 0, Fy: 0, LdirX: 0, LdirY: 0, Rx: 0, Ry: 0,
            PrevFaX: 0, PrevFaY: 0, PrevInX: 0, PrevInY: 0, Deg: 0, DegVel: 0 };
      khState[slot] = s;
    }
    if (!s.Init) { s.PrevInX = frontX; s.PrevInY = frontY; s.Ex = 0; s.Ey = 0; }

    // (0) lane-change: absorb only the CROSS-track part of a single-step snap into a decaying error.
    var stepX = frontX - s.PrevInX, stepY = frontY - s.PrevInY;
    s.PrevInX = frontX; s.PrevInY = frontY;
    var step2 = stepX * stepX + stepY * stepY;
    if (step2 > p.reseedJump * p.reseedJump) { s.Ex = 0; s.Ey = 0; }         // teleport: snap
    else if (s.Init && step2 > p.laneChangeSnap * p.laneChangeSnap) {
      var h = khDir(s.Deg), along = stepX * h[0] + stepY * h[1];
      s.Ex += stepX - along * h[0]; s.Ey += stepY - along * h[1];
    }
    var decay = Math.exp(-dt / p.laneChangeDecayTau);
    s.Ex *= decay; s.Ey *= decay;
    var eMag2 = s.Ex * s.Ex + s.Ey * s.Ey;
    if (eMag2 > p.laneChangeErrorCap * p.laneChangeErrorCap) {
      var kc = p.laneChangeErrorCap / Math.sqrt(eMag2); s.Ex *= kc; s.Ey *= kc;
    }
    var inX = frontX - s.Ex, inY = frontY - s.Ey;

    // (1) predict along the low-passed lane direction, correct with a critically-damped position gain.
    var rawL = khDir(laneHeadingDeg);
    if (!s.Init) { s.Fx = inX; s.Fy = inY; s.LdirX = rawL[0]; s.LdirY = rawL[1]; }
    var aDir = p.lanePredictSmoothTime > 1e-6 ? 1 - Math.exp(-dt / p.lanePredictSmoothTime) : 1;
    s.LdirX += aDir * (rawL[0] - s.LdirX); s.LdirY += aDir * (rawL[1] - s.LdirY);
    var lnrm = Math.sqrt(s.LdirX * s.LdirX + s.LdirY * s.LdirY);
    var lhx = lnrm > 1e-9 ? s.LdirX / lnrm : rawL[0];
    var lhy = lnrm > 1e-9 ? s.LdirY / lnrm : rawL[1];
    var predX = s.Fx + speed * dt * lhx, predY = s.Fy + speed * dt * lhy;
    if ((inX - predX) * (inX - predX) + (inY - predY) * (inY - predY) > p.reseedJump * p.reseedJump) {
      s.Fx = inX; s.Fy = inY;
    } else {
      var g = p.positionSmoothTime > 1e-6 ? 1 - Math.exp(-dt / p.positionSmoothTime) : 1;
      s.Fx = predX + g * (inX - predX); s.Fy = predY + g * (inY - predY);
    }
    var smFrontX = s.Fx, smFrontY = s.Fy;

    // (2) anticipatory turn-in disabled (v2 default) -> front follows the smoothed lane line.
    var faX = smFrontX, faY = smFrontY;

    if (!s.Init) {
      var l0 = khDir(laneHeadingDeg);
      s.Rx = faX - lwb * l0[0]; s.Ry = faY - lwb * l0[1];
      s.Deg = laneHeadingDeg; s.PrevFaX = faX; s.PrevFaY = faY; s.Init = true;
    }
    // teleport / handle reuse: front jumped implausibly far -> reseed the rear from the lane heading.
    if ((faX - s.Rx) * (faX - s.Rx) + (faY - s.Ry) * (faY - s.Ry) >
        (lwb + p.reseedJump) * (lwb + p.reseedJump)) {
      var l1 = khDir(laneHeadingDeg);
      s.Rx = faX - lwb * l1[0]; s.Ry = faY - lwb * l1[1];
      s.Deg = laneHeadingDeg; s.PrevFaX = faX; s.PrevFaY = faY;
    }

    var rawDeg;
    if (speed < p.holdSpeed) {
      rawDeg = s.Deg;                                    // hold heading (no spin at a stop)
      var hh = khDir(rawDeg); s.Rx = faX - lwb * hh[0]; s.Ry = faY - lwb * hh[1];
    } else {
      for (var kk = 1; kk <= 8; kk++) {                  // substepped no-slip rear-axle drag
        var f = kk / 8;
        var ffx = s.PrevFaX + (faX - s.PrevFaX) * f, ffy = s.PrevFaY + (faY - s.PrevFaY) * f;
        var vvx = ffx - s.Rx, vvy = ffy - s.Ry, dd = Math.sqrt(vvx * vvx + vvy * vvy);
        if (dd > 1e-9) { s.Rx = ffx - lwb * (vvx / dd); s.Ry = ffy - lwb * (vvy / dd); }
      }
      rawDeg = khNavi(faX - s.Rx, faY - s.Ry);           // heading = rear->front vector
    }
    s.PrevFaX = faX; s.PrevFaY = faY;

    // (4) output heading low-pass (off by default; SmoothDamp returns target when smoothTime<=0).
    var delta = (((rawDeg - s.Deg + 540) % 360) + 360) % 360 - 180;
    var smoothedDelta = khSmoothDamp(0, delta, s, "DegVel", p.headingSmoothTime, dt);
    var outDeg = (((s.Deg + smoothedDelta) % 360) + 360) % 360;
    s.Deg = outDeg;

    // Front bumper sits on the smoothed lane front; our drawVehicle is front-anchored and extends
    // BACK by length along the heading (== SumoSharp's center = front - 0.5*length*dir, same pixels).
    return { x: smFrontX, y: smFrontY, deg: outDeg };
  }

  // Vehicles interpolated at time t. Fixed-slot arrays: slot i is always the same vehicle, a
  // vehicle absent this frame is null in its slot. Centripetal Catmull-Rom in position; heading
  // from the path tangent (falls back to the stored FCD angle when barely moving). Returns an
  // array of {x,y,angle,speed} (or null for an empty slot).
  function interpolatedVehicles(t) {
    var out = [];
    if (frames.length === 0) return out;
    var k = frameIndexAt(t);
    var k2 = Math.min(k + 1, frames.length - 1);
    var fa = frames[k].v || [];
    var fb = frames[k2].v || [];
    var tA = k * stepSize;
    var span = (k2 - k) * stepSize;
    var frac = span > 1e-9 ? Math.max(0, Math.min(1, (t - tA) / span)) : 0;

    var n = Math.max(fa.length, fb.length);
    for (var i = 0; i < n; i++) {
      var a = fa[i], b = fb[i];
      if (!a) { delete khState[i]; out.push(null); continue; }
      if (!b) {
        // Present now, gone next step: hold only at/after the last step, else drop (and free the
        // slot's kinematic state so a later reuse of the slot reseeds instead of smearing).
        if (k !== k2) { delete khState[i]; out.push(null); }
        else out.push({ x: a[0], y: a[1], angle: a[2], speed: 0, fear: a.length >= 4 ? a[3] : undefined });
        continue;
      }

      // Stage A -- linear interp of the RAW front pose between the two bracketing frames: position,
      // shortest-arc reported heading (fed as the lane heading), and step speed.
      var fx = a[0] + (b[0] - a[0]) * frac;
      var fy = a[1] + (b[1] - a[1]) * frac;
      var laneDeg = a[2] + ((((b[2] - a[2] + 540) % 360) + 360) % 360 - 180) * frac;
      var segDx = b[0] - a[0], segDy = b[1] - a[1];
      var speed = span > 1e-9 ? Math.sqrt(segDx * segDx + segDy * segDy) / span : 0;

      // Stage B -- kinematic bicycle-model reconstruction -> smoothed front + body heading.
      var pose = khUpdate(i, fx, fy, laneDeg, speed, vdim[0] || 4.3, khDtEff);

      // Fear (panic-evac only): 4th element on both endpoints -> linear-interpolate it as position.
      var fear = (a.length >= 4 && b.length >= 4) ? a[3] + (b[3] - a[3]) * frac : undefined;

      out.push({ x: pose.x, y: pose.y, angle: pose.deg, speed: speed, fear: fear });
    }
    return out;
  }

  // Discs interpolated at time t (linear -- crowd frames are dense). Returns [x,y,radius,kind].
  function interpolatedDiscs(t) {
    var out = [];
    if (frames.length === 0) return out;
    var k = frameIndexAt(t);
    var k2 = Math.min(k + 1, frames.length - 1);
    var da = frames[k].d || [];
    var db = frames[k2].d || [];
    var tA = k * stepSize;
    var span = (k2 - k) * stepSize;
    var frac = span > 1e-9 ? Math.max(0, Math.min(1, (t - tA) / span)) : 0;

    // Discs occupy FIXED per-entity slots across frames (SceneGen.AssignStableDiscSlots / the stable
    // crowd builders), so da[i] and db[i] are the SAME entity -- index-matching is safe. An absent slot
    // is null (entity not present that frame); hold the present endpoint, mirroring interpolatedVehicles,
    // so a spawning/vanishing disc never smears from/to another entity's position.
    var n = Math.max(da.length, db.length);
    for (var i = 0; i < n; i++) {
      var a = da[i], b = db[i];
      if (!a && !b) continue;
      if (!a) { out.push(b); continue; }   // appears next frame: hold at its position (no smear-in)
      if (!b) { out.push(a); continue; }   // gone next frame: hold in place (abandoned cars persist)
      // Interpolate position; hold radius/kind, and (for shaped vehicles) heading + shape as-is.
      var e = [a[0] + (b[0] - a[0]) * frac, a[1] + (b[1] - a[1]) * frac, a[2], a[3]];
      if (a.length >= 6 && b.length >= 6) {
        // Interpolate heading along the SHORTEST arc (degrees) so a turning vehicle rotates smoothly
        // between frames instead of snapping its facing at each frame boundary (which reads as jerky
        // jump-rotation, worst on long vehicles). Shape held as-is.
        var dh = (((b[4] - a[4]) % 360) + 540) % 360 - 180;
        e.push(a[4] + dh * frac, a[5]);
      }
      if (a.length >= 8) { e.push(a[6], a[7]); }   // true half-length / half-width for shaped rects
      out.push(e);
    }
    return out;
  }

  // ---------------------------------------------------------------------
  // Rendering (back-to-front)
  // ---------------------------------------------------------------------
  function drawPolygon(flatShape, fillStyle) {
    var n = flatShape.length / 2;
    if (n < 3) return;
    ctx.beginPath();
    for (var i = 0; i < n; i++) {
      var p = worldToScreen(flatShape[i * 2], flatShape[i * 2 + 1]);
      if (i === 0) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]);
    }
    ctx.closePath();
    ctx.fillStyle = fillStyle;
    ctx.fill();
  }

  // Same outline as drawPolygon, plus a stroke -- used for the Stage 4 static layers
  // (park / parking-lot / building-footprint polygons) where a visible edge helps the
  // shape read at city zoom. strokeWidthPx is a fixed CSS-px width (not world-scaled).
  function drawPolygonOutlined(flatShape, fillStyle, strokeStyle, strokeWidthPx) {
    var n = flatShape.length / 2;
    if (n < 3) return;
    ctx.beginPath();
    for (var i = 0; i < n; i++) {
      var p = worldToScreen(flatShape[i * 2], flatShape[i * 2 + 1]);
      if (i === 0) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]);
    }
    ctx.closePath();
    if (fillStyle) { ctx.fillStyle = fillStyle; ctx.fill(); }
    if (strokeStyle) {
      ctx.lineWidth = strokeWidthPx || 1;
      ctx.strokeStyle = strokeStyle;
      ctx.stroke();
    }
  }

  // Stage 4 static polygon layers (docs/SUBAREA-DEMO-CITY-DESIGN.md sec 5), drawn in this
  // bottom-to-top order (called from render(), right after the base network layers and
  // BEFORE drawCrossings()/POI glyphs/agents): zone tints -> park -> parking lots ->
  // buildings -> meet-area markers. All null/empty-guarded -- scenes without these scene
  // keys (e.g. any scene sim_viz produced without --zones/--buildings/pois-v2 polygons)
  // skip them cleanly.
  function drawZones() {
    if (!scene || !scene.zones || !scene.zones.length) return;
    // Bigger polygons first so a large background zone (e.g. the arterial-ring bbox)
    // never paints over a smaller, more specific district drawn afterward.
    var zones = scene.zones.slice().sort(function (a, b) {
      return polygonArea(b.polygon) - polygonArea(a.polygon);
    });
    zones.forEach(function (z) {
      drawPolygon(z.polygon, ZONE_FILL[z.type] || ZONE_FILL_DEFAULT);
    });
  }

  function polygonArea(flat) {
    var n = flat.length / 2, area = 0;
    for (var i = 0; i < n; i++) {
      var j = (i + 1) % n;
      area += flat[i * 2] * flat[j * 2 + 1] - flat[j * 2] * flat[i * 2 + 1];
    }
    return Math.abs(area / 2);
  }

  function drawParks() {
    if (!scene || !scene.parks || !scene.parks.length) return;
    scene.parks.forEach(function (park) {
      drawPolygonOutlined(park.polygon, PARK_FILL, PARK_STROKE, 1.5);
    });
  }

  function drawParkingLots() {
    if (!scene || !scene.parkingLots || !scene.parkingLots.length) return;
    scene.parkingLots.forEach(function (lot) {
      drawPolygonOutlined(lot.polygon, PARKING_LOT_FILL, PARKING_LOT_STROKE, 1.5);
    });
  }

  function drawFootprint(building) {
    drawPolygonOutlined(
      building.polygon,
      BUILDING_FILL[building.type] || BUILDING_FILL_DEFAULT,
      "rgba(15,17,22,0.6)",
      1
    );
  }

  function drawBuildings() {
    if (!scene || !scene.buildings || !scene.buildings.length) return;
    scene.buildings.forEach(drawFootprint);
  }

  // Meet-area markers (park/plaza `meet_areas`, carried on scene.parks[].meetAreas):
  // a small dashed ring + dot, sized loosely by group_size so a bigger gathering spot
  // reads as such without needing its own label.
  function drawMeetAreas() {
    if (!scene || !scene.parks || !scene.parks.length) return;
    ctx.save();
    scene.parks.forEach(function (park) {
      (park.meetAreas || []).forEach(function (m) {
        var p = worldToScreen(m.x, m.y);
        var worldR = 6 + 2 * Math.sqrt(m.groupSize || 1);
        var r = Math.max(4, Math.min(24, worldR * camera.scale));
        ctx.beginPath();
        ctx.setLineDash([4, 3]);
        ctx.lineWidth = 1.5;
        ctx.strokeStyle = MEET_AREA_COLOR;
        ctx.arc(p[0], p[1], r, 0, Math.PI * 2);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.beginPath();
        ctx.arc(p[0], p[1], Math.max(2, r * 0.18), 0, Math.PI * 2);
        ctx.fillStyle = MEET_AREA_COLOR;
        ctx.fill();
      });
    });
    ctx.restore();
  }

  // Panic-evac overlay (S6): the known-world hard edge -- a dashed closed loop, flat [x0,y0,...].
  function drawBoundary(flat) {
    var n = flat.length / 2;
    if (n < 2) return;
    ctx.save();
    ctx.setLineDash([8, 6]);
    ctx.lineWidth = 2;
    ctx.strokeStyle = "rgba(148,163,184,0.85)";
    ctx.beginPath();
    for (var i = 0; i < n; i++) {
      var p = worldToScreen(flat[i * 2], flat[i * 2 + 1]);
      if (i === 0) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]);
    }
    ctx.closePath();
    ctx.stroke();
    ctx.restore();
  }

  // Panic-evac overlay (S6): the incident -- a filled danger disc (radius), a dashed safe-radius ring,
  // and an epicentre dot. inc = [x, y, radius, startTime, safeRadius]. Hidden until it "goes off".
  function drawIncident(inc, simT) {
    if (simT < inc[3]) return;   // not detonated yet
    var c = worldToScreen(inc[0], inc[1]);
    var rDanger = inc[2] * camera.scale, rSafe = inc[4] * camera.scale;
    ctx.beginPath();
    ctx.arc(c[0], c[1], rDanger, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(239,68,68,0.10)";
    ctx.fill();
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = "rgba(239,68,68,0.55)";
    ctx.stroke();
    ctx.save();
    ctx.setLineDash([6, 6]);
    ctx.beginPath();
    ctx.arc(c[0], c[1], rSafe, 0, Math.PI * 2);
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = "rgba(52,211,153,0.7)";
    ctx.stroke();
    ctx.restore();
    ctx.beginPath();
    ctx.arc(c[0], c[1], 5, 0, Math.PI * 2);
    ctx.fillStyle = "#ef4444";
    ctx.fill();
    ctx.lineWidth = 1;
    ctx.strokeStyle = "#fff";
    ctx.stroke();
  }

  function drawLaneBand(lane) {
    var n = lane.shape.length / 2;
    if (n < 2) return;
    ctx.beginPath();
    for (var i = 0; i < n; i++) {
      var p = worldToScreen(lane.shape[i * 2], lane.shape[i * 2 + 1]);
      if (i === 0) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]);
    }
    // Pedestrian-only lanes (sidewalks, `lane.ped`) render as a lighter "concrete"
    // band so a footpath is visually distinct from dark car asphalt -- otherwise a
    // sidewalk looks like a car lane and peds walking it read as sharing the road.
    ctx.strokeStyle = lane.ped ? "#8a8f99" : "#4a4d55";
    // True width, floored to ~2.5 px so roads read as roads when zoomed out.
    ctx.lineWidth = Math.max(2.5, lane.width * camera.scale);
    ctx.lineCap = "butt";
    ctx.lineJoin = "round";
    ctx.stroke();
  }

  function drawLaneMarkings() {
    if (laneMarkings.length === 0) return;
    ctx.save();
    ctx.strokeStyle = "rgba(255,255,255,0.85)";
    ctx.lineWidth = Math.max(1, 0.15 * camera.scale);
    ctx.setLineDash([Math.max(2, 1.5 * camera.scale), Math.max(2, 1.5 * camera.scale)]);
    laneMarkings.forEach(function (seg) {
      var p1 = worldToScreen(seg[0], seg[1]);
      var p2 = worldToScreen(seg[2], seg[3]);
      ctx.beginPath();
      ctx.moveTo(p1[0], p1[1]);
      ctx.lineTo(p2[0], p2[1]);
      ctx.stroke();
    });
    ctx.restore();
  }

  function drawSignals(simT) {
    if (!network || !network.signals) return;
    network.signals.forEach(function (sig) {
      var tl = tlsById[sig.tl];
      if (!tl) return;
      var state = tlLinkState(tl, sig.linkIndex, simT);
      var p = worldToScreen(sig.x, sig.y);
      ctx.beginPath();
      ctx.arc(p[0], p[1], Math.max(3, 0.9 * camera.scale), 0, Math.PI * 2);
      ctx.fillStyle = colorForSignalState(state);
      ctx.fill();
      ctx.lineWidth = 1;
      ctx.strokeStyle = "rgba(0,0,0,0.6)";
      ctx.stroke();
    });
  }

  // Oriented vehicle box (same geometry as the original single-scenario drawVehicle): (x,y) is the
  // FRONT-CENTRE reference point; the box extends BACK by `length` along the heading. naviDegree
  // 0 = up-screen (+Y world), increasing clockwise -> ctx.rotate(angleRad - PI/2).
  function drawVehicle(v) {
    var length = vdim[0] || 4.3;
    var width = vdim[1] || 1.8;
    var p = worldToScreen(v.x, v.y);
    ctx.save();
    ctx.translate(p[0], p[1]);
    ctx.rotate((v.angle * Math.PI) / 180 - Math.PI / 2);
    var lengthPx = Math.max(length * camera.scale, 5);
    var widthPx = Math.max(width * camera.scale, 3);
    ctx.fillStyle = speedColorToggle.checked
      ? colorForSpeed(v.speed)
      : (v.fear !== undefined ? fearColor(v.fear) : VEHICLE_COLOR);
    ctx.strokeStyle = "rgba(0,0,0,0.55)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.rect(-lengthPx, -widthPx / 2, lengthPx, widthPx);
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }

  // A crowd/vehicle agent. Round agents (pedestrians) are drawn as filled circles from
  // [x,y,radius,kind]; VEHICLE agents carry [x,y,radius,kind,headingDeg,shape] and are drawn as an
  // ORIENTED shape aligned to travel direction: shape 0 = rectangle (car / tuk-tuk), 1 = hexagon
  // (motorcycle). Coloured by kind. headingDeg is the world-space travel angle (CCW from +x); the
  // shape's corners are built in world coords and mapped through worldToScreen (so the camera's y-flip
  // is handled uniformly, same as the lane/vehicle-box draws).
  function drawDisc(d) {
    var color = DISC_COLORS[d[3]] || "#9ca3af";
    if (d.length >= 6) { drawShaped(d[0], d[1], d[2], color, d[4], d[5], d[6], d[7]); return; }
    var p = worldToScreen(d[0], d[1]);
    var r = Math.max(d[2] * camera.scale, 3);
    ctx.beginPath();
    ctx.arc(p[0], p[1], r, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.lineWidth = 1;
    ctx.strokeStyle = "rgba(0,0,0,0.55)";
    ctx.stroke();
  }

  function drawShaped(x, y, radius, color, headingDeg, shape, halfLen, halfWid) {
    var r = Math.max(radius, 5.5 / camera.scale);   // floor the on-screen size when zoomed out
    var hr = (headingDeg * Math.PI) / 180;
    var hx = Math.cos(hr), hy = Math.sin(hr);        // world heading unit
    // When true footprint half-dimensions are supplied (mixed-traffic scenes) use them directly so a
    // bus renders long and a car short (true aspect); otherwise fall back to a radius-derived box.
    var floor = 3.0 / camera.scale;
    var hasDims = typeof halfLen === "number" && typeof halfWid === "number";
    var hl = hasDims ? Math.max(halfLen, floor) : 0.95 * r;
    var hw = hasDims ? Math.max(halfWid, floor) : 0.5 * r;
    var pts;
    if (shape === 1) {
      // motorcycle: slim pointed-front hexagon, elongated along heading
      var mh = hasDims ? hl : 0.95 * r, mw = hasDims ? hw : 0.42 * r;
      pts = [[mh, 0], [0.4 * mh, mw], [-0.6 * mh, mw], [-mh, 0], [-0.6 * mh, -mw], [0.4 * mh, -mw]];
    } else {
      // car / tuk-tuk / bus: rectangle at true aspect
      pts = [[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]];
    }
    ctx.beginPath();
    for (var i = 0; i < pts.length; i++) {
      var a = pts[i][0], pp = pts[i][1];
      var sp = worldToScreen(x + a * hx + pp * (-hy), y + a * hy + pp * hx);
      if (i === 0) ctx.moveTo(sp[0], sp[1]); else ctx.lineTo(sp[0], sp[1]);
    }
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.fill();
    ctx.lineWidth = 1;
    ctx.strokeStyle = "rgba(0,0,0,0.6)";
    ctx.stroke();
  }

  // Static POI "places" (scene.pois). Drawn in WORLD metres so glyphs scale with
  // zoom, with a px floor/ceiling so they stay legible without swamping the map.
  // (x,y) is the POI position in the net XY frame (no transform needed). Called
  // BEFORE the disc/vehicle passes so moving agents always draw on top.
  var POI_WORLD_SIZE = 3.0;   // nominal glyph half-size in metres
  function drawPoi(poi) {
    var style = POI_STYLE[poi.kind] || POI_DEFAULT_STYLE;
    var p = worldToScreen(poi.x, poi.y);
    var s = Math.max(4, Math.min(14, POI_WORLD_SIZE * camera.scale));
    ctx.save();
    ctx.fillStyle = style.color;
    ctx.strokeStyle = "rgba(0,0,0,0.55)";
    ctx.lineWidth = 1;
    ctx.globalAlpha = 0.9;
    var g = style.glyph;
    if (g === "square") {
      ctx.beginPath();
      ctx.rect(p[0] - s, p[1] - s, 2 * s, 2 * s);
      ctx.fill(); ctx.stroke();
    } else if (g === "triangle") {
      // house/triangle: apex up
      ctx.beginPath();
      ctx.moveTo(p[0], p[1] - s);
      ctx.lineTo(p[0] + s, p[1] + s);
      ctx.lineTo(p[0] - s, p[1] + s);
      ctx.closePath();
      ctx.fill(); ctx.stroke();
    } else if (g === "diamond") {
      ctx.beginPath();
      ctx.moveTo(p[0], p[1] - s);
      ctx.lineTo(p[0] + s, p[1]);
      ctx.lineTo(p[0], p[1] + s);
      ctx.lineTo(p[0] - s, p[1]);
      ctx.closePath();
      ctx.fill(); ctx.stroke();
    } else if (g === "P") {
      // parking: rounded blue tile with a white "P"
      ctx.beginPath();
      ctx.rect(p[0] - s, p[1] - s, 2 * s, 2 * s);
      ctx.fill(); ctx.stroke();
      ctx.fillStyle = "#0b1e3a";
      ctx.font = "bold " + Math.max(7, Math.round(1.6 * s)) + "px -apple-system, Arial, sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText("P", p[0], p[1] + 0.5);
    } else {
      // circle
      ctx.beginPath();
      ctx.arc(p[0], p[1], s, 0, Math.PI * 2);
      ctx.fill(); ctx.stroke();
    }
    ctx.restore();

    // Small text label when zoomed in (world glyph >= ~9 px) for the notable
    // kinds -- keeps the map clean at overview scale, readable up close.
    if (poi.label && POI_LABEL_KINDS[poi.kind] && POI_WORLD_SIZE * camera.scale >= 9) {
      ctx.save();
      ctx.font = "10px -apple-system, Arial, sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "top";
      ctx.lineWidth = 3;
      ctx.strokeStyle = "rgba(15,17,22,0.85)";
      ctx.fillStyle = "rgba(232,232,234,0.95)";
      ctx.strokeText(poi.label, p[0], p[1] + s + 2);
      ctx.fillText(poi.label, p[0], p[1] + s + 2);
      ctx.restore();
    }
  }

  // Crossing (crosswalk) overlay. Crossings are SUMO internal lanes whose edgeId
  // matches ":<node>_c<index>"; sim_viz emits them as ordinary lane bands (they
  // are already drawn grey), so we detect them here by id and repaint a subtle
  // striped zebra on top. Detection lives in the renderer (no payload change) so
  // sim_viz's vehicle/ped output stays byte-identical. crossingLanes is filled by
  // precomputeNetwork().
  var CROSSING_RE = /^:.+_c\d+$/;
  var crossingLanes = [];
  function drawCrossings() {
    if (crossingLanes.length === 0) return;
    ctx.save();
    for (var i = 0; i < crossingLanes.length; i++) {
      var lane = crossingLanes[i];
      var shp = lane.shape;
      if (shp.length < 4) continue;
      // Band base: a lighter fill along the crossing centreline.
      var a = worldToScreen(shp[0], shp[1]);
      var b = worldToScreen(shp[shp.length - 2], shp[shp.length - 1]);
      var dx = b[0] - a[0], dy = b[1] - a[1];
      var len = Math.sqrt(dx * dx + dy * dy) || 1;
      var ux = dx / len, uy = dy / len;          // along-crossing unit (screen)
      var px = -uy, py = ux;                       // perpendicular unit (screen)
      var halfW = Math.max(2.5, (lane.width * camera.scale) / 2);
      // subtle base band
      ctx.strokeStyle = "rgba(226,232,240,0.14)";
      ctx.lineWidth = halfW * 2;
      ctx.lineCap = "butt";
      ctx.beginPath();
      ctx.moveTo(a[0], a[1]);
      ctx.lineTo(b[0], b[1]);
      ctx.stroke();
      // zebra stripes: short perpendicular ticks spaced along the crossing
      var stripeGap = Math.max(4, 0.9 * camera.scale);
      ctx.strokeStyle = "rgba(255,255,255,0.6)";
      ctx.lineWidth = Math.max(1.5, 0.35 * camera.scale);
      for (var d = stripeGap * 0.5; d < len; d += stripeGap) {
        var cx = a[0] + ux * d, cy = a[1] + uy * d;
        ctx.beginPath();
        ctx.moveTo(cx - px * halfW, cy - py * halfW);
        ctx.lineTo(cx + px * halfW, cy + py * halfW);
        ctx.stroke();
      }
    }
    ctx.restore();
  }

  function render(simT) {
    // Sim-time delta since the last render, driving the kinematic reconstruction. On a
    // discontinuity (first frame / scrub / backward step / long stall) clear the per-vehicle state
    // so boxes snap crisply to the scrubbed pose; otherwise advance with a clamped-positive dt.
    khDt = (khPrevSimT === null) ? null : simT - khPrevSimT;
    khPrevSimT = simT;
    if (khDt === null || khDt < 0 || khDt > 0.5) { khClear(); khDtEff = 1 / 60; }
    else khDtEff = Math.max(khDt, 1e-6);

    // devicePixelRatio-correct draw path: clear the whole backing store at identity, then scale the
    // context by dpr so every CSS-pixel coordinate worldToScreen emits lands on the right device px.
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "#1b1e26";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    // 1..4 Network layers (skipped cleanly for pure-crowd scenes).
    if (network) {
      (network.junctions || []).forEach(function (j) { drawPolygon(j.shape, "#33363f"); });
      network.lanes.forEach(function (lane) { drawLaneBand(lane); });
      drawLaneMarkings();
    }

    // Stage 4 static overlay (docs/SUBAREA-DEMO-CITY-DESIGN.md sec 5), bottom-to-top:
    // district zone tints -> park polygon -> parking-lot polygons -> building footprints
    // -> meet-area markers. Drawn UNDER crossings/POI glyphs/agents so those always read
    // on top. All null/empty-guarded -- a no-arg scene draws none of this.
    drawZones();
    drawParks();
    drawParkingLots();
    drawBuildings();
    drawMeetAreas();

    if (network) {
      drawCrossings();
      drawSignals(simT);
    }

    // Panic-evac overlays (S6): the known-world hard edge and the incident danger/safe rings. Additive
    // and null-guarded -- absent on every scene except "Panic evacuation".
    if (scene.boundary) drawBoundary(scene.boundary);
    if (scene.incident) drawIncident(scene.incident, simT);

    // Static POI "places" layer (scene.pois, optional). Drawn UNDER the moving
    // agents so cars/peds render on top. Absent -> clean no-op.
    if (scene.pois && scene.pois.length) {
      scene.pois.forEach(drawPoi);
    }

    // 5. Discs (crowd/pedestrian agents).
    var discs = interpolatedDiscs(simT);
    discs.forEach(drawDisc);

    // 6. Vehicles (oriented boxes).
    var vehicles = interpolatedVehicles(simT);
    var vehCount = 0;
    for (var i = 0; i < vehicles.length; i++) {
      if (vehicles[i]) { drawVehicle(vehicles[i]); vehCount++; }
    }

    var parts = [];
    if (vdim[0] > 0) parts.push("vehicles: " + vehCount);
    if (discs.length > 0) parts.push("discs: " + discs.length);
    vehCountEl.textContent = parts.join("  ·  ") || "";
    timeReadout.textContent = "t = " + simT.toFixed(1) + " s / " + simEnd.toFixed(1) + " s";
    if (!sliderDragging) timeSlider.value = String(simT);
  }

  // ---------------------------------------------------------------------
  // Legend (per scene): the vehicle swatch (if any) + the disc kinds present.
  // ---------------------------------------------------------------------
  function buildLegend() {
    legendEl.innerHTML = "";
    function addRow(color, label, circle) {
      var row = document.createElement("div");
      row.className = "row";
      var sw = document.createElement("span");
      sw.className = "swatch";
      sw.style.background = color;
      if (circle) sw.style.borderRadius = "50%";
      row.appendChild(sw);
      var lab = document.createElement("span");
      lab.textContent = label;
      row.appendChild(lab);
      legendEl.appendChild(row);
    }

    if (vdim[0] > 0) addRow(VEHICLE_COLOR, "vehicle", false);
    if (network && network.lanes && network.lanes.some(function (l) { return l.ped; })) {
      addRow("#8a8f99", "sidewalk (footpath)", false);
    }

    // Collect disc kinds present anywhere in the scene.
    var kinds = {};
    frames.forEach(function (f) {
      (f.d || []).forEach(function (d) { if (d) kinds[d[3]] = true; });
    });
    var labels = (scene && scene.labels) || null;   // per-scene override (e.g. vehicle classes)
    Object.keys(kinds).sort().forEach(function (k) {
      var label = (labels && labels[k]) || DISC_LABELS[k] || ("kind " + k);
      addRow(DISC_COLORS[k] || "#9ca3af", label, true);
    });

    // Static POI "places" kinds present in this scene (optional layer).
    if (scene && scene.pois && scene.pois.length) {
      var poiKinds = {};
      scene.pois.forEach(function (p) { poiKinds[p.kind] = true; });
      Object.keys(poiKinds).sort().forEach(function (kind) {
        var style = POI_STYLE[kind] || POI_DEFAULT_STYLE;
        addRow(style.color, "POI: " + (style.label || kind), style.glyph !== "square" && style.glyph !== "P");
      });
    }

    // Stage 4 static polygon layers (optional): zone tints, park, parking lots, building
    // footprints, meet areas -- one legend row per distinct type actually present.
    if (scene && scene.zones && scene.zones.length) {
      var zoneTypes = {};
      scene.zones.forEach(function (z) { zoneTypes[z.type] = true; });
      Object.keys(zoneTypes).sort().forEach(function (t) {
        addRow(ZONE_FILL[t] || ZONE_FILL_DEFAULT, "zone: " + (ZONE_LABELS[t] || t), false);
      });
    }
    if (scene && scene.parks && scene.parks.length) {
      addRow(PARK_FILL, "park", false);
      if (scene.parks.some(function (p) { return (p.meetAreas || []).length; })) {
        addRow(MEET_AREA_COLOR, "meet area", true);
      }
    }
    if (scene && scene.parkingLots && scene.parkingLots.length) {
      addRow(PARKING_LOT_FILL, "parking lot", false);
    }
    if (scene && scene.buildings && scene.buildings.length) {
      var bldgTypes = {};
      scene.buildings.forEach(function (b) { bldgTypes[b.type] = true; });
      Object.keys(bldgTypes).sort().forEach(function (t) {
        addRow(BUILDING_FILL[t] || BUILDING_FILL_DEFAULT, "building: " + (BUILDING_LABELS[t] || t), false);
      });
    }
  }

  // ---------------------------------------------------------------------
  // Scene loading / switching
  // ---------------------------------------------------------------------
  function loadScene(idx) {
    if (idx < 0 || idx >= scenes.length) return;
    scene = scenes[idx];
    frames = scene.frames || [];
    network = scene.network || null;
    vdim = scene.vdim || [4.3, 1.8];
    stepSize = scene.dt > 0 ? scene.dt : 1.0;
    simStart = 0;
    simEnd = frames.length > 1 ? (frames.length - 1) * stepSize : 0;
    simTime = simStart;

    resetVehSmoothing();   // per-vehicle kinematic-heading state is per-scene; start fresh
    precomputeNetwork();
    buildLegend();

    captionEl.innerHTML = "";
    var title = document.createElement("span");
    title.className = "title";
    title.textContent = scene.name || "";
    captionEl.appendChild(title);
    if (scene.desc) {
      captionEl.appendChild(document.createTextNode(scene.desc));
    }

    timeSlider.min = String(simStart);
    timeSlider.max = String(simEnd);
    timeSlider.step = String(Math.max(stepSize / 10, 0.001));
    timeSlider.value = String(simStart);

    // Re-fit the camera to the new scene and re-enable auto-fit (until the user pans/zooms).
    userAdjusted = false;
    resizeCanvas();
    fitToView();

    if (!prefersReduced) setPlaying(true);
    render(simTime);
  }

  // ---------------------------------------------------------------------
  // HUD wiring
  // ---------------------------------------------------------------------
  function setPlaying(p) {
    playing = p;
    btnPlay.textContent = playing ? "Pause" : "Play";
  }

  // Populate the scene selector.
  (function buildSceneSelector() {
    scenes.forEach(function (s, i) {
      var opt = document.createElement("option");
      opt.value = String(i);
      opt.textContent = (i + 1) + ". " + (s.name || ("scene " + (i + 1)));
      sceneSel.appendChild(opt);
    });
    // Hide the selector chrome entirely for a single-scene (single-scenario) export.
    if (scenes.length <= 1) {
      var wrapEl = document.getElementById("sceneSelWrap");
      if (wrapEl) wrapEl.style.display = "none";
    }
  })();

  sceneSel.addEventListener("change", function () {
    var idx = parseInt(sceneSel.value, 10) || 0;
    loadScene(idx);
  });

  btnPlay.addEventListener("click", function () { setPlaying(!playing); });
  btnRestart.addEventListener("click", function () { simTime = simStart; setPlaying(true); });
  speedSel.addEventListener("change", function () { speedMultiplier = parseFloat(speedSel.value) || 1; });

  timeSlider.addEventListener("pointerdown", function () {
    sliderDragging = true;
    wasPlayingBeforeDrag = playing;
    playing = false;
  });
  timeSlider.addEventListener("input", function () {
    simTime = parseFloat(timeSlider.value);
    render(simTime);
  });
  timeSlider.addEventListener("pointerup", function () {
    sliderDragging = false;
    playing = wasPlayingBeforeDrag;
  });

  window.addEventListener("keydown", function (ev) {
    if (ev.code === "Space") { ev.preventDefault(); setPlaying(!playing); }
    else if (ev.key === "r" || ev.key === "R") { simTime = simStart; setPlaying(true); }
    else if (ev.key === "ArrowRight") { simTime = Math.min(simEnd, simTime + stepSize); render(simTime); }
    else if (ev.key === "ArrowLeft") { simTime = Math.max(simStart, simTime - stepSize); render(simTime); }
  });

  // ---------------------------------------------------------------------
  // Main loop
  // ---------------------------------------------------------------------
  var lastFrameMs = null;
  function frame(nowMs) {
    // Re-measure/size/fit every frame until the user manually zooms or pans (iOS Safari robustness).
    if (!userAdjusted) resizeCanvas();
    if (lastFrameMs === null) lastFrameMs = nowMs;
    var realDelta = (nowMs - lastFrameMs) / 1000;
    lastFrameMs = nowMs;

    if (playing && !sliderDragging) {
      simTime += realDelta * speedMultiplier;
      if (simTime >= simEnd) { simTime = simEnd; setPlaying(false); }
      if (simTime < simStart) simTime = simStart;
    }

    render(simTime);
    requestAnimationFrame(frame);
  }

  if (scenes.length === 0) {
    // Nothing to show -- surface it rather than a silent black canvas.
    throw new Error("REPLAY_DATA has no scenes");
  }

  loadScene(0);
  resizeCanvas();
  requestAnimationFrame(frame);
})();
