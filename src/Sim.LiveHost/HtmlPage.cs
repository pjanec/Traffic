namespace Sim.LiveHost;

// The self-contained front-end served at "/". A canvas renderer driven by the WebSocket: it receives the
// network geometry once, then per-frame vehicle state, and on click converts the pixel to WORLD
// coordinates (the inverse camera transform) and sends an obstacle request back. Kept inline so the demo
// is a single runnable project with no static-file plumbing.
internal static class HtmlPage
{
    public const string Html = """
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SumoSharp — live</title>
<style>
  html,body{margin:0;height:100%;background:#0e1116;color:#e6edf3;font:13px/1.4 system-ui,sans-serif;overflow:hidden}
  #hud{position:fixed;top:10px;left:12px;z-index:10;background:rgba(20,24,31,.82);padding:10px 12px;border-radius:8px;border:1px solid #2b323c}
  #hud b{color:#7ee787}
  #hud .row{margin-top:6px;color:#9da7b3}
  button{background:#21262d;color:#e6edf3;border:1px solid #3b424c;border-radius:6px;padding:5px 9px;cursor:pointer;margin-top:8px}
  button:hover{background:#2d333b}
  #mode{color:#58a6ff;text-transform:uppercase;font-size:11px;letter-spacing:.5px}
  #diag{position:fixed;top:10px;right:12px;z-index:10;background:rgba(20,24,31,.88);padding:8px 11px;border-radius:8px;border:1px solid #2b323c;font:11px/1.55 ui-monospace,Consolas,monospace;color:#9da7b3;white-space:pre;display:none}
  #diag b{color:#7ee787;font-weight:600}
  #diag .warn{color:#f0883e}
  .tog{margin-left:8px;color:#9da7b3;user-select:none;cursor:pointer}
  canvas{display:block;cursor:crosshair}
</style>
</head>
<body>
<div id="hud">
  <div><b>SumoSharp</b> live &middot; <span id="mode"></span> &middot; <span id="stat">connecting…</span></div>
  <div class="row"><b>click</b> the road to drop an obstacle &middot; wheel = zoom &middot; drag = pan</div>
  <div class="row">
    <button id="restart">restart</button>
    <button id="clear">clear obstacles</button>
    <label class="tog"><input type="checkbox" id="random"> inject random traffic</label>
  </div>
  <div class="row">
    <label class="tog">DR delay <input type="range" id="delay" min="0" max="1.5" step="0.05" value="0" style="vertical-align:middle;width:130px"> <span id="delayval">0.00s</span></label>
    <label class="tog"><input type="checkbox" id="smooth"> smooth (extrap only)</label>
  </div>
  <div class="row" style="color:#6e7681">delay <b style="color:#9da7b3">0</b> = extrapolate (predict ahead, may snap) &middot; raise = interpolate between packets (smooth, delayed)</div>
</div>
<div id="diag">diag</div>
<canvas id="c"></canvas>
<script>
(function(){
  const cv = document.getElementById('c'), ctx = cv.getContext('2d');
  const stat = document.getElementById('stat');
  const modeEl = document.getElementById('mode');
  const randomChk = document.getElementById('random');
  let net = null;
  const cam = { scale: 1, ox: 0, oy: 0 };

  // Lane-relative DEAD RECKONING (SUMOSHARP-DEADRECKONING.md §5.1/§6). The server sends only sparse
  // (~2 Hz) lane-relative state per vehicle {lw,p,pl,s,a}; the client reconstructs world pose by walking
  // the once-sent lane geometry. No position/heading SMOOTHING is applied -- the pose is drawn exactly
  // where the DR math puts it -- so the true behaviour is visible, not hidden behind a filter.
  //
  // Two mechanisms selected by the interactive DELAY knob (`delay`, seconds), because they trade off
  // differently and the demo lets you SEE both:
  //   * delay = 0  -> EXTRAPOLATION. Render at "now"; each vehicle is predicted forward from its newest
  //     packet: pos = p + s·dt + ½·a·dt². Zero latency, but a reactive vehicle (braking at a leader,
  //     slowing for a turn) is mispredicted and snaps when the next, truer packet arrives -- the classic
  //     DR artefact, on purpose.
  //   * delay > 0  -> INTERPOLATION. Render `delay` seconds in the PAST, BETWEEN the two buffered packets
  //     that bracket that time. That is exact real data (just time-shifted), so motion is smooth with no
  //     overshoot; the cost is `delay` seconds of latency. When the render time runs past the newest
  //     packet (a long-deferred vehicle) it falls back to extrapolation.
  // The render time comes from a jitter-free wall->sim map (serverSim ≈ simRate·wall + bEst, bEst an EMA),
  // NOT a chasing PLL, so playback advances at constant velocity -- no "caterpillar" speed pulsing.
  const tracked = new Map();     // id -> { hist: [{t,p,s,a,pl,lw}...] newest last, l, w }
  const HIST = 8;                // packets kept per vehicle (enough to bracket any reasonable delay)
  let frameObstacles = [];
  let frameTl = [];         // [{ln, st}] traffic-light state per controlled lane
  let npub = 0, nalive = 0; // last step's published-count / alive-count (HUD bandwidth stat)
  const CAR_LEN = 5.0, CAR_W = 1.8;  // demo vehicles are the default passenger vType
  let lastStep = -1;        // dedupe: the server re-sends the latest snapshot faster than the sim ticks
  let simRate = 2;          // sim-seconds per wall-second, from a LONG-BASELINE fit (see ingestFrame): total
                            // sim advanced / total wall elapsed. Immune to burst jitter -- a per-gap ratio
                            // (dT/dW) spikes to huge values when two steps arrive ms apart under load, which
                            // made the clock race then snap back (the jerky ~1 s stepping); this cannot.
  let firstWall = null, firstSim = 0;  // anchor (wall/sim of the first step this run) for the rate fit + clock
  let latestSim = 0;        // newest received sim time (for the EXTRAP/INTERP mode readout)
  let renderSim = 0;        // continuous render time in sim seconds (recomputed each frame)
  let delay = 0.0;          // INTERACTIVE playout delay (s): 0 = extrapolate, higher = interpolate
  let smooth = false;       // INTERACTIVE: optional low-pass on the rendered pose (off = raw DR, to diagnose)
  let frameDt = 0.016;      // wall seconds since previous frame (only used to weight optional smoothing)
  let wsCount = 0;          // total frame messages received (for the diagnostics overlay's ws/s readout)

  function ingestFrame(m){
    if(m.step === lastStep) return; // same sim step re-sent -> ignore (keep extrapolating)
    lastStep = m.step;
    const nowWall = performance.now() / 1000;
    // Long-baseline playback rate: total sim advanced / total wall elapsed since the first step. The growing
    // denominator makes it immune to per-packet burst jitter (a clump of steps can't spike it), so the clock
    // runs at a steady velocity instead of racing-and-snapping.
    if(firstWall === null){ firstWall = nowWall; firstSim = m.time; }
    else {
      const base = nowWall - firstWall;
      if(base > 1.0){ const r = (m.time - firstSim) / base; if(r > 0.2 && r < 20) simRate = r; }
    }
    latestSim = m.time;

    // Append this step's published packets to each vehicle's short history (stamped with the SIM time they
    // hold). The history is what interpolation brackets against; extrapolation uses only the newest entry.
    for(const v of (m.vehicles || [])){
      let r = tracked.get(v.id);
      if(!r){ r = { hist: [] }; tracked.set(v.id, r); }
      r.l = v.l; r.w = v.w;
      r.hist.push({ t: m.time, p: v.p, s: v.s, a: v.a, pl: v.pl, lw: v.lw });
      if(r.hist.length > HIST) r.hist.shift();
    }
    // Despawn: drop any tracked vehicle no longer alive. Absence from `vehicles` alone is NOT a despawn --
    // it just means "keep dead-reckoning it".
    const aliveSet = new Set(m.alive || []);
    for(const id of tracked.keys()){ if(!aliveSet.has(id)) tracked.delete(id); }
    frameObstacles = m.obstacles || [];
    frameTl = m.tl || [];
    npub = m.npub || 0; nalive = (m.nalive != null) ? m.nalive : aliveSet.size;
  }

  // SUMO signal char -> colour.
  function tlColor(st){
    switch(st){
      case 'G': case 'g': return '#3fb950';   // green (protected / permissive)
      case 'y': case 'Y': return '#e3b341';   // yellow
      case 'r': return '#f85149';             // red
      case 'o': case 'O': case 'u': return '#e3b341'; // off/blink-ish -> amber
      default: return '#8b949e';
    }
  }

  // Port of Sim.Ingest.LaneGeometry.PositionAtOffset: point + navi-degree tangent at arc `offset` along a
  // flat [x0,y0,x1,y1,...] polyline, shifted by `latOffset` (+ = left of travel).
  function positionAtOffset(pts, offset, latOffset){
    const n = pts.length / 2;
    if(n < 2) return { x: pts[0]||0, y: pts[1]||0, deg: 0 };
    let remaining = offset < 0 ? 0 : offset;
    for(let i = 0; i < n - 1; i++){
      const x1 = pts[2*i], y1 = pts[2*i+1], x2 = pts[2*i+2], y2 = pts[2*i+3];
      const dx = x2 - x1, dy = y2 - y1, segLen = Math.hypot(dx, dy), last = i === n - 2;
      if(remaining <= segLen || last){
        const t = segLen > 0 ? Math.max(0, Math.min(1, remaining / segLen)) : 0;
        let x = x1 + dx*t, y = y1 + dy*t;
        if(latOffset && segLen > 0){ x += latOffset * (-dy/segLen); y += latOffset * (dx/segLen); }
        let deg = 90 - Math.atan2(dy, dx) * 180 / Math.PI; deg %= 360; if(deg < 0) deg += 360;
        return { x, y, deg };
      }
      remaining -= segLen;
    }
    return { x: pts[pts.length-2], y: pts[pts.length-1], deg: 0 };
  }

  const WIN_CUR = 2;  // lw layout: [prev2, prev1, CURRENT, next1, next2, next3]

  // navi-deg (0=N, cw) -> unit world direction (matches PoseResolver.VectorFromNavi).
  function naviVec(deg){ const r = deg*Math.PI/180; return [Math.sin(r), Math.cos(r)]; }

  // Contiguous valid run of a lane window [lo..hi] around CURRENT, with cumulative arc starts (shared by
  // poseAtArc and arcInWindow). Returns null if the window is unusable.
  function windowRun(lw){
    if(!lw) return null;
    let lo = WIN_CUR, hi = WIN_CUR;
    while(lo-1 >= 0 && lw[lo-1] >= 0 && net.lanes[lw[lo-1]]) lo--;
    while(hi+1 < lw.length && lw[hi+1] >= 0 && net.lanes[lw[hi+1]]) hi++;
    const start = []; let cum = 0;
    for(let i = lo; i <= hi; i++){ start[i] = cum; cum += net.lanes[lw[i]].len; }
    return { lo, hi, start, total: cum, curStart: start[WIN_CUR] };
  }

  // Resolve a render pose at an ABSOLUTE along-current-lane arc (0 = current-lane start; may be negative to
  // reach a previous window lane, or exceed the current lane to reach the next). Walks the lane WINDOW so it
  // follows real curves (no corner-cutting). Heading = SUMO back->front CHORD; position = the front, bowed
  // toward the OUTSIDE of the turn by the swept-path off-tracking amount so long vehicles swing wide.
  function poseAtArc(pk, arc, pl, bodyLen){
    const lw = pk.lw; const w = windowRun(lw); if(!w) return null;
    const { lo, hi, start, total, curStart } = w;
    let frontG = curStart + arc;
    if(frontG > total - 1e-4) frontG = total - 1e-4;           // clamp at the end of the known window
    if(frontG < 0) frontG = 0;
    const backG = Math.max(0, frontG - bodyLen);

    const sample = (g) => {
      for(let i = lo; i <= hi; i++){
        const lane = net.lanes[lw[i]];
        if(g <= start[i] + lane.len || i === hi){ return positionAtOffset(lane.pts, g - start[i], pl); }
      }
      const l = net.lanes[lw[hi]]; return positionAtOffset(l.pts, l.len, pl);
    };

    const front = sample(frontG), back = sample(backG);
    const dx = front.x - back.x, dy = front.y - back.y;
    let deg = (dx*dx + dy*dy) > 1e-9 ? (90 - Math.atan2(dy, dx) * 180/Math.PI) : front.deg;
    deg %= 360; if(deg < 0) deg += 360;

    // Off-tracking bow: shift the front toward the outside of the turn by ~ bodyLen*|dpsi|/2.
    const ft = naviVec(front.deg), bt = naviVec(back.deg);
    const cross = bt[0]*ft[1] - bt[1]*ft[0];                   // >0 => left/CCW turn (outside is right)
    let off = bodyLen * Math.abs(Math.asin(Math.max(-1, Math.min(1, cross)))) * 0.5;
    if(off > bodyLen) off = bodyLen;
    const sign = cross >= 0 ? -1 : 1;
    const bx = front.x + off * (-ft[1]*sign), by = front.y + off * (ft[0]*sign);
    return { x: bx, y: by, deg };
  }

  // Express packet `other`'s along-lane position in `ref`'s current-lane arc coordinate, so two packets can
  // be interpolated even when the lane window shifted between them (the vehicle advanced onto the next lane).
  // Returns null if `other`'s current lane isn't present in `ref`'s window (too large a jump -> caller
  // falls back to extrapolation rather than interpolating across an unknown gap).
  function arcInWindow(ref, other){
    const w = windowRun(ref.lw); if(!w) return null;
    const otherLane = other.lw[WIN_CUR];
    for(let i = w.lo; i <= w.hi; i++){
      if(ref.lw[i] === otherLane){ return (w.start[i] - w.curStart) + other.p; }
    }
    return null;
  }

  function resize(){ cv.width = innerWidth; cv.height = innerHeight; if(net) draw(); }
  addEventListener('resize', resize);

  function w2s(x,y){ return [x*cam.scale + cam.ox, -y*cam.scale + cam.oy]; }
  function s2w(x,y){ return [(x-cam.ox)/cam.scale, -(y-cam.oy)/cam.scale]; }

  function fit(b){
    const bw = Math.max(b.maxX-b.minX, 1), bh = Math.max(b.maxY-b.minY, 1);
    const s = Math.min(cv.width/bw, cv.height/bh) * 0.9;
    const cx = (b.minX+b.maxX)/2, cy = (b.minY+b.maxY)/2;
    cam.scale = s; cam.ox = cv.width/2 - cx*s; cam.oy = cv.height/2 + cy*s;
  }

  // A small direction chevron at a lane's midpoint, pointing along travel.
  function drawLaneArrow(lane){
    const p = lane.pts, n = p.length/2; if(n < 2) return;
    const mi = Math.max(0, Math.floor(n/2) - 1);
    const x1=p[2*mi], y1=p[2*mi+1], x2=p[2*mi+2], y2=p[2*mi+3];
    const dx=x2-x1, dy=y2-y1, len=Math.hypot(dx,dy)||1;
    const s = w2s((x1+x2)/2, (y1+y2)/2);
    const a = Math.atan2(-(dy/len), dx/len);  // world dir -> screen angle (y flipped)
    const sz = Math.max(3, 1.3*cam.scale);
    ctx.save(); ctx.translate(s[0], s[1]); ctx.rotate(a);
    ctx.fillStyle = 'rgba(150,170,190,0.30)';
    ctx.beginPath(); ctx.moveTo(sz,0); ctx.lineTo(-sz*0.6,-sz*0.7); ctx.lineTo(-sz*0.6,sz*0.7); ctx.closePath(); ctx.fill();
    ctx.restore();
  }

  function speedColor(s){
    const t = Math.max(0, Math.min(1, s/13.9)); // 0 = stopped (red) .. 1 = free-flow (green)
    const r = Math.round(230*(1-t) + 40*t), g = Math.round(70*(1-t) + 200*t);
    return 'rgb('+r+','+g+',80)';
  }

  function draw(){
    ctx.fillStyle = '#0e1116'; ctx.fillRect(0,0,cv.width,cv.height);
    if(!net){ return; }

    // roads: a dark casing under a lighter lane fill, each drawn as a polyline stroked to the lane width.
    ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    for(let pass = 0; pass < 2; pass++){
      for(const lane of net.lanes){
        const p = lane.pts; if(p.length < 4) continue;
        ctx.beginPath();
        let a = w2s(p[0],p[1]); ctx.moveTo(a[0],a[1]);
        for(let i=2;i<p.length;i+=2){ const q = w2s(p[i],p[i+1]); ctx.lineTo(q[0],q[1]); }
        const wpx = Math.max(1.5, lane.w*cam.scale);
        if(pass === 0){ ctx.strokeStyle = '#0a0c10'; ctx.lineWidth = wpx + 2.5; }        // casing
        else { ctx.strokeStyle = lane.internalLane ? '#2a3038' : '#454e5a'; ctx.lineWidth = wpx; } // surface
        ctx.stroke();
      }
    }

    // lane markings: a subtle dashed centre line + a travel-direction chevron per drivable lane.
    ctx.setLineDash([6, 7]);
    ctx.strokeStyle = 'rgba(200,210,225,0.14)';
    ctx.lineWidth = 1;
    for(const lane of net.lanes){
      if(lane.internalLane){ continue; }
      const p = lane.pts; if(p.length < 4) continue;
      ctx.beginPath();
      let a = w2s(p[0],p[1]); ctx.moveTo(a[0],a[1]);
      for(let i=2;i<p.length;i+=2){ const q = w2s(p[i],p[i+1]); ctx.lineTo(q[0],q[1]); }
      ctx.stroke();
    }
    ctx.setLineDash([]);
    if(cam.scale > 1.2){ for(const lane of net.lanes){ if(!lane.internalLane) drawLaneArrow(lane); } }

    // traffic-light signals: a coloured dot at the end (stop line) of each controlled approach lane.
    for(const t of frameTl){
      const lane = net.lanes[t.ln]; if(!lane || lane.pts.length < 2) continue;
      const px = lane.pts[lane.pts.length-2], py = lane.pts[lane.pts.length-1];
      const s = w2s(px, py), rad = Math.max(2.5, 0.9*cam.scale);
      ctx.beginPath(); ctx.arc(s[0], s[1], rad, 0, 6.2832);
      ctx.fillStyle = tlColor(t.st); ctx.fill();
      ctx.strokeStyle = '#0a0c10'; ctx.lineWidth = 1; ctx.stroke();
    }

    // vehicles: sample each at the render time `sampleT = renderSim - delay`. If that time is at/after the
    // vehicle's newest packet -> EXTRAPOLATE (predict forward); otherwise -> INTERPOLATE between the two
    // buffered packets that bracket it (exact real data, just time-shifted). Optional low-pass smoothing is
    // applied ONLY to extrapolated poses -- interpolation is already smooth and exact, so it's drawn raw.
    // Render clock: advance MONOTONICALLY toward the smooth wall->sim estimate (floored at the newest step
    // so delay=0 stays pure extrapolation). Forward-only: if the estimate dips (jitter) we HOLD rather than
    // step back -- a momentary hold is invisible, a backward step is a visible back-jump (even in interp).
    const clockNow = (firstWall === null) ? latestSim : firstSim + simRate * (performance.now() / 1000 - firstWall);
    const est = Math.max(clockNow, latestSim);
    if(renderSim === 0 || est > renderSim + 3){ renderSim = est; }                 // init / restart -> jump fwd
    else if(est > renderSim){ renderSim += Math.min(est - renderSim, frameDt * simRate * 3); } // capped catch-up
    // else est <= renderSim: hold (never reverse); the estimate climbs back within a frame or two
    if(typeof window !== 'undefined') window.__renderSim = renderSim;               // exposed for diagnostics
    const sampleT = renderSim - delay;
    const aPos = 1 - Math.exp(-frameDt / 0.07), aDeg = 1 - Math.exp(-frameDt / 0.06);
    let drawn = 0;
    for(const rec of tracked.values()){
      const H = rec.hist; if(!H.length) continue;
      const bodyLen = rec.l || CAR_LEN;
      const newest = H[H.length - 1];
      let pk, arc, pl, extrap;
      if(sampleT >= newest.t){                               // EXTRAPOLATE forward from the newest packet
        let dt = sampleT - newest.t; if(dt > 3.0) dt = 3.0;
        pk = newest; pl = newest.pl; arc = newest.p + newest.s*dt + 0.5*newest.a*dt*dt; extrap = true;
      } else if(sampleT <= H[0].t){                          // older than the buffer -> extrapolate from oldest
        let dt = sampleT - H[0].t; if(dt < -3.0) dt = -3.0;
        pk = H[0]; pl = H[0].pl; arc = H[0].p + H[0].s*dt + 0.5*H[0].a*dt*dt; extrap = true;
      } else {                                               // INTERPOLATE between bracketing packets a,b
        let ai = 0; for(let i = H.length-1; i >= 0; i--){ if(H[i].t <= sampleT){ ai = i; break; } }
        const a = H[ai], b = H[Math.min(ai+1, H.length-1)];
        const span = b.t - a.t, f = span > 1e-6 ? (sampleT - a.t) / span : 0;
        const arcA = arcInWindow(b, a);
        if(arcA === null){                                   // window shifted too far -> extrapolate from a
          const dt = sampleT - a.t; pk = a; pl = a.pl; arc = a.p + a.s*dt + 0.5*a.a*dt*dt; extrap = true;
        } else {
          pk = b; pl = a.pl + (b.pl - a.pl)*f; arc = arcA + (b.p - arcA)*f; extrap = false;
        }
      }
      const pose = poseAtArc(pk, arc, pl, bodyLen);
      if(!pose) continue;

      let px = pose.x, py = pose.y, pdeg = pose.deg;
      if(smooth && extrap && rec.vx !== undefined){          // low-pass, extrapolation only
        const ex = pose.x - rec.vx, ey = pose.y - rec.vy;
        if(ex*ex + ey*ey > 49){ rec.vx = pose.x; rec.vy = pose.y; rec.vdeg = pose.deg; } // >7 m: snap
        else {
          rec.vx += ex*aPos; rec.vy += ey*aPos;
          let dd = ((pose.deg - rec.vdeg + 540) % 360) - 180;                             // shortest turn
          if(Math.abs(dd) > 50) rec.vdeg = pose.deg; else rec.vdeg = (rec.vdeg + dd*aDeg + 360) % 360;
        }
        px = rec.vx; py = rec.vy; pdeg = rec.vdeg;
      } else {
        rec.vx = pose.x; rec.vy = pose.y; rec.vdeg = pose.deg; // keep vis synced so enabling smoothing is clean
      }

      const s = w2s(px, py);
      // navi deg (0=N, cw) -> world dir (sin,cos) -> screen dir (x, -y flip) -> screen angle.
      const nr = pdeg * Math.PI/180;
      const sa = Math.atan2(-Math.cos(nr), Math.sin(nr));
      const L = Math.max(3, bodyLen*cam.scale), W = Math.max(2, (rec.w || CAR_W)*cam.scale);
      ctx.save();
      ctx.translate(s[0], s[1]); ctx.rotate(sa);
      ctx.fillStyle = speedColor(pk.s);
      ctx.strokeStyle = 'rgba(0,0,0,0.55)'; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.rect(-L, -W/2, L, W); ctx.fill(); ctx.stroke();
      ctx.restore();
      drawn++;
    }

    // obstacles
    for(const o of frameObstacles){
      const s = w2s(o.x, o.y), k = Math.max(4, 3*cam.scale);
      ctx.strokeStyle = '#ff5c5c'; ctx.lineWidth = 2.5;
      ctx.beginPath(); ctx.moveTo(s[0]-k,s[1]-k); ctx.lineTo(s[0]+k,s[1]+k);
      ctx.moveTo(s[0]+k,s[1]-k); ctx.lineTo(s[0]-k,s[1]+k); ctx.stroke();
    }

    const pct = nalive > 0 ? Math.round(100*npub/nalive) : 0;
    // Bandwidth stat: state records SENT this step vs vehicles ALIVE. The adaptive policy re-sends only
    // uncertain movers at full rate; predictable ones ride on the client's dead reckoning. The mode tag
    // reflects the current delay: INTERP once the render time sits behind the newest step, else EXTRAP.
    const mode = sampleT < latestSim - 1e-3 ? 'INTERP' : 'EXTRAP';
    stat.textContent = drawn + ' vehicles · sent ' + npub + '/' + nalive + ' (' + pct + '%) · t=' +
      renderSim.toFixed(1) + 's · sim ' + simRate.toFixed(1) + '/s · delay ' + delay.toFixed(2) + 's [' +
      mode + (smooth ? ' · smoothed' : '') + ']';
  }

  // --- interaction ---
  let drag = null;
  cv.addEventListener('mousedown', e => { drag = { x:e.clientX, y:e.clientY, ox:cam.ox, oy:cam.oy, moved:false }; });
  addEventListener('mousemove', e => {
    if(!drag) return;
    if(Math.abs(e.clientX-drag.x)+Math.abs(e.clientY-drag.y) > 3) drag.moved = true;
    cam.ox = drag.ox + (e.clientX-drag.x); cam.oy = drag.oy + (e.clientY-drag.y); draw();
  });
  addEventListener('mouseup', e => {
    if(drag && !drag.moved){ // a click, not a pan -> inject obstacle at the world point
      const rect = cv.getBoundingClientRect();
      const wp = s2w(e.clientX-rect.left, e.clientY-rect.top);
      send({ type:'obstacle', x:wp[0], y:wp[1] });
    }
    drag = null;
  });
  cv.addEventListener('wheel', e => {
    e.preventDefault();
    const f = e.deltaY < 0 ? 1.1 : 1/1.1;
    const before = s2w(e.clientX, e.clientY);
    cam.scale *= f;
    cam.ox = e.clientX - before[0]*cam.scale; cam.oy = e.clientY + before[1]*cam.scale; draw();
  }, { passive:false });
  document.getElementById('clear').addEventListener('click', () => send({ type:'clear' }));
  document.getElementById('restart').addEventListener('click', () => {
    tracked.clear();           // drop the old run's vehicles immediately (server rewinds to t=0)
    lastStep = -1; firstWall = null; latestSim = 0; renderSim = 0; // reset the render clock for the new run
    send({ type:'restart' });
  });
  randomChk.addEventListener('change', () => send({ type:'random', on: randomChk.checked }));
  // DR controls: `delay` slides extrapolation (0) -> interpolation (>0); `smooth` toggles the optional
  // low-pass (extrapolation only). Both are read live by draw() every frame.
  const delayEl = document.getElementById('delay'), delayVal = document.getElementById('delayval');
  const smoothEl = document.getElementById('smooth');
  delayEl.addEventListener('input', () => { delay = parseFloat(delayEl.value); delayVal.textContent = delay.toFixed(2) + 's'; });
  smoothEl.addEventListener('change', () => { smooth = smoothEl.checked; });

  // --- socket ---
  let ws;
  function send(o){ if(ws && ws.readyState === 1) ws.send(JSON.stringify(o)); }
  function connect(){
    ws = new WebSocket((location.protocol==='https:'?'wss':'ws')+'://'+location.host+'/ws');
    ws.onmessage = ev => {
      const m = JSON.parse(ev.data);
      if(m.type === 'network'){
        net = m; fit(m.bounds); resize();
        modeEl.textContent = (m.mode || '') + (m.mode === 'scenario' ? ' demand' : '');
        randomChk.checked = !!m.randomTraffic;
      }
      else if(m.type === 'frame'){ wsCount++; ingestFrame(m); }
    };
    ws.onclose = () => { stat.textContent = 'disconnected — retrying…'; setTimeout(connect, 1000); };
  }
  connect();

  // Diagnostics overlay (press 'd'): the render-smoothness truth for THIS browser. fps = actual rAF rate
  // (drops to ~1 when the tab is backgrounded -> Chrome throttles rAF; that alone causes the "HUD freezes,
  // then jumps ~1 s" symptom). "worst" = longest frame gap in the last second (main-thread stalls). ws/s =
  // frames received (server sends ~20/s). vis = document.visibilityState.
  const diagEl = document.getElementById('diag');
  let diagShow = false, dFrames = 0, dMax = 0, dLast = 0, dWs0 = 0;
  addEventListener('keydown', e => {
    if(e.key === 'd'){
      diagShow = !diagShow; diagEl.style.display = diagShow ? 'block' : 'none';
      dLast = performance.now()/1000; dFrames = 0; dMax = 0; dWs0 = wsCount;
    }
  });

  // Render loop: renderSim (the wall->sim clock) is recomputed inside draw(); the loop tracks the inter-frame
  // wall delta (used for optional smoothing + the fps meter) and redraws.
  let lastRaf = 0;
  (function loop(){
    const now = performance.now() / 1000;
    frameDt = lastRaf > 0 ? Math.min(0.1, now - lastRaf) : 0.016; lastRaf = now;
    if(diagShow){
      dFrames++; if(frameDt > dMax) dMax = frameDt;
      const el = now - dLast;
      if(el >= 1){
        const fps = Math.round(dFrames/el), worst = Math.round(dMax*1000), vis = document.visibilityState;
        const bad = fps < 50 || worst > 40 || vis !== 'visible';
        diagEl.innerHTML =
          'fps <b'+(fps<50?' class="warn"':'')+'>'+fps+'</b>   worst <b'+(worst>40?' class="warn"':'')+'>'+worst+'ms</b>\n'+
          'ws/s <b>'+(wsCount-dWs0)+'</b>   vis <b'+(vis!=='visible'?' class="warn"':'')+'>'+vis+'</b>\n'+
          'veh <b>'+tracked.size+'</b>   t <b>'+renderSim.toFixed(1)+'</b>   delay '+delay.toFixed(2)+'s\n'+
          (bad ? 'renderer starved — see console note' : 'render healthy');
        dFrames = 0; dMax = 0; dLast = now; dWs0 = wsCount;
      }
    }
    draw();
    requestAnimationFrame(loop);
  })();
  resize();
})();
</script>
</body>
</html>
""";
}
