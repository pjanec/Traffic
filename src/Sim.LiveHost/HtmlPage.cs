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
  canvas{display:block;cursor:crosshair}
</style>
</head>
<body>
<div id="hud">
  <div><b>SumoSharp</b> live &middot; <span id="stat">connecting…</span></div>
  <div class="row"><b>click</b> the road to drop an obstacle</div>
  <div class="row">wheel = zoom &middot; drag = pan</div>
  <button id="clear">clear obstacles</button>
</div>
<canvas id="c"></canvas>
<script>
(function(){
  const cv = document.getElementById('c'), ctx = cv.getContext('2d');
  const stat = document.getElementById('stat');
  let net = null;
  const cam = { scale: 1, ox: 0, oy: 0 };

  // Lane-relative DEAD RECKONING (SUMOSHARP-DEADRECKONING.md §5.1/§6). The server sends only sparse
  // (~2 Hz) lane-relative state per vehicle {ln,nx,p,pl,s,a}; this client reconstructs world pose by
  // walking the once-sent lane geometry and EXTRAPOLATES forward at 60 fps: pos' = p + s·dt + ½·a·dt².
  // Because it follows the actual lane polyline, prediction tracks real curves (no corner-cutting) --
  // unlike naive world-space (x,y) extrapolation. dt is in SIM seconds; the sim rate is measured from
  // consecutive frames so this works at any server rate.
  let frameVehicles = [];   // latest frame's [{id,ln,nx,p,pl,s,a}]
  let frameObstacles = [];
  let frameTl = [];         // [{ln, st}] traffic-light state per controlled lane
  let frameTime = 0;        // sim time (s) of the latest frame's sample
  const CAR_LEN = 5.0, CAR_W = 1.8;  // demo vehicles are the default passenger vType
  let frameRecvWall = 0;    // performance.now()/1000 when it arrived
  let simRate = 1;          // measured sim-seconds per wall-second (smoothed)
  let lastStep = -1;        // dedupe: the server re-sends the latest snapshot faster than the sim ticks

  function ingestFrame(m){
    if(m.step === lastStep) return; // same sim step re-sent -> ignore (keep extrapolating)
    lastStep = m.step;
    const nowWall = performance.now() / 1000;
    if(frameRecvWall > 0){
      const dW = nowWall - frameRecvWall, dT = m.time - frameTime;
      if(dW > 1e-3 && dT > 0){ simRate = simRate * 0.7 + (dT / dW) * 0.3; }
    }
    frameVehicles = m.vehicles || [];
    frameObstacles = m.obstacles || [];
    frameTl = m.tl || [];
    frameTime = m.time;
    frameRecvWall = nowWall;
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

  // Dead-reckon one vehicle `dt` sim-seconds past its sampled lane-relative state, walking into the next
  // lane if it runs off the end (one boundary, matching the server's single-lookahead `nx`).
  function resolvePose(v, dt){
    let arc = v.p + v.s * dt + 0.5 * v.a * dt * dt;
    if(arc < v.p) arc = v.p;                       // never predict backwards (matches PoseResolver)
    let lane = net.lanes[v.ln];
    if(!lane) return null;
    if(arc > lane.len && v.nx >= 0 && net.lanes[v.nx]){ arc -= lane.len; lane = net.lanes[v.nx]; }
    const a = Math.max(0, Math.min(arc, lane.len));
    return positionAtOffset(lane.pts, a, v.pl);
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

    // vehicles: dead-reckoned from the latest sparse frame to *now* (clamp dt so a server stall can't run
    // the extrapolation away), drawn as oriented rectangles (front at the pose point, extending back).
    let dt = simRate * (performance.now()/1000 - frameRecvWall);
    if(!(dt >= 0)) dt = 0;
    if(dt > 2.0) dt = 2.0;
    let drawn = 0;
    for(const v of frameVehicles){
      const pose = resolvePose(v, dt);
      if(!pose) continue;
      const s = w2s(pose.x, pose.y);
      // navi deg (0=N, cw) -> world dir (sin,cos) -> screen dir (x, -y flip) -> screen angle.
      const nr = pose.deg * Math.PI/180;
      const sa = Math.atan2(-Math.cos(nr), Math.sin(nr));
      const L = Math.max(3, (v.l || CAR_LEN)*cam.scale), W = Math.max(2, (v.w || CAR_W)*cam.scale);
      ctx.save();
      ctx.translate(s[0], s[1]); ctx.rotate(sa);
      ctx.fillStyle = speedColor(v.s);
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

    stat.textContent = drawn + ' vehicles · t=' + (frameTime + dt).toFixed(1) + 's · dead-reckoned ' +
      simRate.toFixed(1) + '× (updates ' + simRate.toFixed(1) + '/s)';
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

  // --- socket ---
  let ws;
  function send(o){ if(ws && ws.readyState === 1) ws.send(JSON.stringify(o)); }
  function connect(){
    ws = new WebSocket((location.protocol==='https:'?'wss':'ws')+'://'+location.host+'/ws');
    ws.onmessage = ev => {
      const m = JSON.parse(ev.data);
      if(m.type === 'network'){ net = m; fit(m.bounds); resize(); }
      else if(m.type === 'frame'){ ingestFrame(m); }
    };
    ws.onclose = () => { stat.textContent = 'disconnected — retrying…'; setTimeout(connect, 1000); };
  }
  connect();
  (function loop(){ draw(); requestAnimationFrame(loop); })();
  resize();
})();
</script>
</body>
</html>
""";
}
