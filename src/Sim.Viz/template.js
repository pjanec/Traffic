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
    9: "#94a3b8",  // low-power (PathArc) pedestrian, sim-LOD demo
    10: "#f97316", // promoted (full-ORCA) pedestrian, sim-LOD demo
    11: "#facc15", // sim-LOD interest source marker
    12: "#78716c", // static/dynamic box obstacle
    13: "#4f8ef7", // the one maneuvering car in the parking demo
    14: "#eab308", // paused/idle liveliness pedestrian (ActivityTimeline Pause / idle clamp)
    15: "#22d3ee", // dwelling/seated liveliness pedestrian (ActivityTimeline Dwell, visible)
    16: "#f472b6", // pre-scheduled two-ped interaction (ActivityTimeline Interact, "talk")
    17: "#fbbf24", // LIVE-POC-3 waiter micro-scenario actor
    18: "#22c55e", // evac-district safe-zone (corner) marker
  };
  var DISC_LABELS = {
    0: "stream / agent A", 1: "stream / agent B", 2: "pedestrian", 3: "pedestrian (rerouting)",
    9: "pedestrian (low-power / PathArc)", 10: "pedestrian (promoted / full ORCA)",
    11: "interest source", 12: "obstacle", 13: "maneuvering car",
    14: "pedestrian (paused / idle)", 15: "pedestrian (dwelling / seated)",
    16: "pedestrian (talking)", 17: "pedestrian (waiter)", 18: "safe zone (corner)",
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
  var crossings = [], pedSignals = [];

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
    crossings = [];
    pedSignals = [];
    if (!network) return;

    // Additive: older/vehicle-only scenes (and BuildNetwork's own default) carry no crossing
    // geometry, so these fall back to empty arrays cleanly rather than throwing.
    crossings = network.crossings || [];
    pedSignals = network.pedSignals || [];

    network.lanes.forEach(function (lane) {
      lanesById[lane.id] = lane;
      (lanesByEdge[lane.edgeId] = lanesByEdge[lane.edgeId] || []).push(lane);
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

  function shortestArcDeg(fromDeg, toDeg) {
    var d = ((toDeg - fromDeg + 540) % 360) - 180;
    return fromDeg + d;
  }

  // Centripetal Catmull-Rom through p0..p3 (segment p1->p2, local u in [0,1]). Centripetal
  // (alpha=0.5) avoids the overshoot/loop that uniform Catmull-Rom shows on unevenly-spaced FCD
  // samples (a vehicle slowing through a junction). Barry-Goldman pyramid form.
  function catmullRom(p0, p1, p2, p3, u) {
    function knot(ti, pa, pb) {
      var dx = pb[0] - pa[0], dy = pb[1] - pa[1];
      return ti + Math.max(Math.pow(dx * dx + dy * dy, 0.25), 1e-6);
    }
    function lerp(pa, pb, s) {
      return [pa[0] + (pb[0] - pa[0]) * s, pa[1] + (pb[1] - pa[1]) * s];
    }
    var t0 = 0, t1 = knot(t0, p0, p1), t2 = knot(t1, p1, p2), t3 = knot(t2, p2, p3);
    var tt = t1 + (t2 - t1) * u;
    var a1 = lerp(p0, p1, (tt - t0) / (t1 - t0));
    var a2 = lerp(p1, p2, (tt - t1) / (t2 - t1));
    var a3 = lerp(p2, p3, (tt - t2) / (t3 - t2));
    var b1 = lerp(a1, a2, (tt - t0) / (t2 - t0));
    var b2 = lerp(a2, a3, (tt - t1) / (t3 - t1));
    return lerp(b1, b2, (tt - t1) / (t2 - t1));
  }

  // naviDegree (0 = north/+Y, clockwise) from a world-space motion vector.
  function headingFromDelta(dx, dy) {
    var deg = 90 - (Math.atan2(dy, dx) * 180) / Math.PI;
    deg = deg % 360;
    return deg < 0 ? deg + 360 : deg;
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
    var fp = (frames[k - 1] && frames[k - 1].v) || [];
    var fn = (frames[k + 2] && frames[k + 2].v) || [];
    var tA = k * stepSize;
    var span = (k2 - k) * stepSize;
    var frac = span > 1e-9 ? Math.max(0, Math.min(1, (t - tA) / span)) : 0;

    var n = Math.max(fa.length, fb.length);
    for (var i = 0; i < n; i++) {
      var a = fa[i], b = fb[i];
      if (!a) { out.push(null); continue; }
      if (!b) {
        // Present now, gone next step: hold only at/after the last step, else drop.
        var heldFear = a.length >= 4 ? a[3] : undefined;
        out.push(k === k2 ? { x: a[0], y: a[1], angle: a[2], speed: 0, fear: heldFear } : null);
        continue;
      }
      var p1 = [a[0], a[1]], p2 = [b[0], b[1]];
      var pv = fp[i] || a, pn = fn[i] || b;
      var p0 = [pv[0], pv[1]], p3 = [pn[0], pn[1]];
      var pos = catmullRom(p0, p1, p2, p3, frac);

      var df = 0.06;
      var forward = frac + df <= 1;
      var probe = forward ? catmullRom(p0, p1, p2, p3, frac + df) : catmullRom(p0, p1, p2, p3, frac - df);
      var dx = forward ? probe[0] - pos[0] : pos[0] - probe[0];
      var dy = forward ? probe[1] - pos[1] : pos[1] - probe[1];
      var moving = dx * dx + dy * dy > 1e-4;
      var angle = moving ? headingFromDelta(dx, dy) : shortestArcDeg(a[2], b[2]);

      // Speed (m/s) derived from step displacement -- drives the optional speed heatmap.
      var segDx = p2[0] - p1[0], segDy = p2[1] - p1[1];
      var speed = span > 1e-9 ? Math.sqrt(segDx * segDx + segDy * segDy) / span : 0;

      // Fear (panic-evac only): 4th element on both endpoints -> linear-interpolate it the same way
      // as position. Other scenes' entries stay 3-long, so fear is left undefined for them.
      var fear = (a.length >= 4 && b.length >= 4) ? a[3] + (b[3] - a[3]) * frac : undefined;

      out.push({ x: pos[0], y: pos[1], angle: angle, speed: speed, fear: fear });
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
    ctx.strokeStyle = "#4a4d55";
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

  // Crosswalk zebra (fix "crossings aren't rendered at all"): a slightly-lighter-than-junction fill
  // for the crossing's own polygon footprint, then a run of white bars perpendicular to the
  // centreline's local travel direction, spanning the crossing's real width, evenly spaced along the
  // (possibly multi-segment) centreline. World-unit geometry mapped through worldToScreen per corner
  // (like drawShaped) so the camera y-flip/pan/zoom is handled uniformly.
  var ZEBRA_BAR_LEN = 0.5;     // metres, along the direction of travel
  var ZEBRA_PERIOD = 1.0;      // metres, bar-start to next bar-start (~0.5 m gap between bars)
  function drawCrossings() {
    if (!crossings.length) return;
    crossings.forEach(function (c) {
      if (c.outline && c.outline.length >= 6) {
        drawPolygon(c.outline, "#3d414c");
      }

      var center = c.center;
      if (!center || center.length < 4) return;
      var halfWidth = (c.width || 2.0) / 2.0;

      ctx.save();
      ctx.fillStyle = "rgba(255,255,255,0.55)";
      var travelled = 0;
      var nSeg = center.length / 2 - 1;
      for (var s = 0; s < nSeg; s++) {
        var x1 = center[s * 2], y1 = center[s * 2 + 1];
        var x2 = center[(s + 1) * 2], y2 = center[(s + 1) * 2 + 1];
        var dx = x2 - x1, dy = y2 - y1;
        var segLen = Math.sqrt(dx * dx + dy * dy) || 1e-6;
        var ux = dx / segLen, uy = dy / segLen;   // unit vector along travel
        var px = -uy, py = ux;                    // unit vector perpendicular (crossing width)

        // First bar CENTRE offset within this segment, continuing the period across segment joins,
        // but never less than half a bar length in (else the bar's leading half would poke out
        // before the segment/crossing outline even starts).
        var firstOffset = ZEBRA_PERIOD - (travelled % ZEBRA_PERIOD);
        if (firstOffset >= ZEBRA_PERIOD - 1e-9) firstOffset -= ZEBRA_PERIOD;
        if (firstOffset < ZEBRA_BAR_LEN / 2) firstOffset += ZEBRA_PERIOD;

        for (var d = firstOffset; d + ZEBRA_BAR_LEN / 2 <= segLen; d += ZEBRA_PERIOD) {
          var bx = x1 + ux * d, by = y1 + uy * d;
          var half = ZEBRA_BAR_LEN / 2;
          var corners = [
            [bx - ux * half + px * halfWidth, by - uy * half + py * halfWidth],
            [bx + ux * half + px * halfWidth, by + uy * half + py * halfWidth],
            [bx + ux * half - px * halfWidth, by + uy * half - py * halfWidth],
            [bx - ux * half - px * halfWidth, by - uy * half - py * halfWidth],
          ];
          ctx.beginPath();
          for (var i = 0; i < corners.length; i++) {
            var sp = worldToScreen(corners[i][0], corners[i][1]);
            if (i === 0) ctx.moveTo(sp[0], sp[1]); else ctx.lineTo(sp[0], sp[1]);
          }
          ctx.closePath();
          ctx.fill();
        }

        travelled += segLen;
      }
      ctx.restore();
    });
  }

  // Pedestrian (crossing) signal heads (fix "can't tell crosswalk TLs from vehicle TLs"): drawn as
  // SMALL SQUARES, deliberately smaller than drawSignals' round vehicle-signal dots (side ~=
  // 1.1*scale px vs. the vehicle dot's 0.9*scale RADIUS -- half a square side is ~0.55*scale, well
  // under the circle's radius) with a thin white outline so it reads as a distinct pedestrian head.
  function drawPedSignals(simT) {
    if (!pedSignals.length) return;
    var side = Math.max(4, 1.1 * camera.scale);
    pedSignals.forEach(function (sig) {
      var tl = tlsById[sig.tl];
      if (!tl) return;
      var state = tlLinkState(tl, sig.linkIndex, simT);
      var p = worldToScreen(sig.x, sig.y);
      ctx.beginPath();
      ctx.rect(p[0] - side / 2, p[1] - side / 2, side, side);
      ctx.fillStyle = colorForSignalState(state);
      ctx.fill();
      ctx.lineWidth = 1;
      ctx.strokeStyle = "rgba(255,255,255,0.9)";
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

  function render(simT) {
    // devicePixelRatio-correct draw path: clear the whole backing store at identity, then scale the
    // context by dpr so every CSS-pixel coordinate worldToScreen emits lands on the right device px.
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "#1b1e26";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    // 1..6 Network layers (skipped cleanly for pure-crowd scenes). Draw order: junctions -> lane
    // bands -> lane markings -> crossing zebra -> round vehicle signals -> square ped signals.
    // NOTE: crossings are deliberately drawn AFTER the lane bands/markings, not before them --
    // a crossing's footprint geometrically coincides with the vehicle roadway it crosses (that's
    // the whole point of a crosswalk), so drawing the zebra UNDER the lane band made it fully
    // painted over and invisible (verified empirically: a canvas-fill instrumentation trace showed
    // the zebra fillStyle firing every frame, yet the rendered pixels never showed it, because
    // drawLaneBand's opaque stroke -- true lane width, floored to ~2.5px -- was drawn on top and
    // covered the exact same world-space footprint). Signals stay last so they're never occluded.
    if (network) {
      (network.junctions || []).forEach(function (j) { drawPolygon(j.shape, "#33363f"); });
      network.lanes.forEach(function (lane) { drawLaneBand(lane); });
      drawLaneMarkings();
      drawCrossings();
      drawSignals(simT);
      drawPedSignals(simT);
    }

    // Panic-evac overlays (S6): the known-world hard edge and the incident danger/safe rings. Additive
    // and null-guarded -- absent on every scene except "Panic evacuation".
    if (scene.boundary) drawBoundary(scene.boundary);
    if (scene.incident) drawIncident(scene.incident, simT);

    // 7. Discs (crowd/pedestrian agents).
    var discs = interpolatedDiscs(simT);
    discs.forEach(drawDisc);

    // 8. Vehicles (oriented boxes).
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
