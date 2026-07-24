# IgBridge Integration Guide — embed the live-city engine + reconstruction in IG software

**Read this first, then §4/§5 (the build).** This is the bootstrap for a session that turns the IgBridge
*ideas* (validated as a PoC) into a **concrete, embeddable integration**: a C# class that owns the whole
**live-city** SumoSharp engine (cars **and** pedestrians), is **ticked by a host app at a low update rate
(1/5/10 Hz)**, and is **sampled by the host at its own higher tick rate (20 Hz)** to reconstruct every
entity's current pose (XYZ + orientation), pushing one update sample per entity to an **external 3D image
generator (IG)** — an IG with no predictive dead-reckoning that consumes only plain
`position / orientation / timestamp` samples.

Branch: start from `main` after PR #8 merges (it carries the shared reconstruction). This is **new,
design-first work** (§10) — produce a design → tasks → tracker before coding.

---

## 0. The one-paragraph mental model

Two clocks. The **host** owns both. (1) It **ticks the engine** at the *update rate* (1/5/10 Hz): each tick
spawns/tops-up ambient traffic + peds, steps the sim, and appends one dead-reckoning (DR) sample per entity
to that entity's ring buffer. (2) At its **own 20 Hz tick** it **samples** the buffers: for each entity it
reconstructs a continuous pose at "now" — interpolating/extrapolating between the sparse engine samples and
applying the kinematic smoothing — yielding XYZ + heading (+ pitch/roll if wanted), and emits a
`new/upd/del` sample to the IG. Because the update rate < the sample rate, this is exactly the **coarse-feed**
case IgBridge already tuned and validated (the owner-approved "1 Hz feed still looks smooth" result). The
smoothing math already exists and is shared; the new work is the **embeddable live-city host** and the
**push-to-IG** surface.

---

## 1. What already exists vs what this session builds

**Exists (reuse verbatim — do NOT rewrite the motion math):**
- `src/Sim.IgBridge/IgBridgeRunner.cs` — the closest starting point: the **only** class that owns an
  `Engine` **and** a pedestrian stream in one host-driven `Tick()`, exposing per-entity DR history
  (`VehicleHistories`, `PedHistories`) + `Lanes` + `VehicleDims` + `IsSampleTick`.
- `src/Sim.IgBridge/IgBridgeSession.cs` — the reference wiring `history → DrClock.ResolveAt →
  KinematicReconstructor.Resolve → IgSample new/upd/del`. Generalize this into the sampling API.
- `src/Sim.Viewer.Motion/KinematicReconstructor.cs` — the shared smoothing (no-slip rear-axle + look-ahead +
  lane-change ease + coarse-feed handling). `Resolve(...)` returns `CenterX/CenterY/Z/HeadingDeg` (the IG
  pivot). Set `CoarseFeed = true`.
- `src/Sim.Viewer.Motion/DrClock.cs` — `ResolveAt(history, sampleT, lanes)` (deterministic, host-clock) and
  `Resolve(history, delay, lanes)` (wall-clock Pump). Interpolate/extrapolate/straddle logic.
- `src/Sim.Core/PoseResolver.cs` — `DrState → Pose(x,y,z,heading)`; `Pose.Z` carries lane-surface elevation.
- `src/Sim.IgBridge/PedStream.cs` — a deterministic synthetic ped crowd (PathArc, pure function of time; no
  DrClock needed) + `IgBridgeSession.EmitPeds` (linear-resample + velocity-derived heading).
- `src/Sim.IgBridge/IgSample.cs` — the IG-native `new/upd/del` record schema (`id, t, model, x, y, z,
  headingDeg`).

**New (this session):**
1. An **embeddable, host-driven live-city host** (IgBridgeRunner-shaped) that self-populates the *ambient*
   city (not a fixed parsed route file) with cars + peds and crossing-yield coupling (§9).
2. A **sampling API** the host calls at 20 Hz — `IReadOnlyList<IgSample> Sample(double now)` (or a
   push-callback) — that runs the reconstruction for all entities at one coherent instant and returns/pushes
   the per-entity update stream.
3. The **push-to-IG transport** (the host software "already knows the IG protocol" — it owns the wire; this
   layer just produces the `IgSample` stream).
4. **HPR** (heading/pitch/roll) if the IG wants more than yaw (§7) — pitch exists (City3D's `ComputePitchRad`),
   roll is an open item, and the `IgSample` schema may need extending.

---

## 2. Architecture — the two-clock host

```
 HOST APP (its own 20 Hz loop)
   every UPDATE tick (1/5/10 Hz):   liveCity.Tick()      // step engine+peds, append DR samples
   every 20 Hz tick:                var samples = liveCity.Sample(now);   // reconstruct all entities @ now
                                    foreach (s in samples) ig.Push(s);    // new/upd/del to the IG
```

- **`Tick()`** (update rate): spawn/top-up ambient cars + peds → step peds → `engine.Step()` → append one
  `TimestampedSample`(vehicles)/`PedSample`(peds) per live entity to its ring, keyed by **sim time** → diff
  the active set for spawn/despawn. (This is `IgBridgeRunner.Tick`, `IgBridgeRunner.cs:215`, generalized to
  ambient spawning.)
- **`Sample(now)`** (20 Hz): for each entity, resolve a pose at `tau = now − lookahead` and emit
  `new`/`upd`/`del`. The `lookahead` keeps the query just behind the newest buffered sample so DrClock stays
  in the **interpolate** branch (see §6). `now` is the host's monotonic **sim clock** (drive it yourself for
  determinism, or Pump DrClock off the wall clock).

The engine's own `StepLength` and the host's update rate: the owner's model is **engine step-length = update
rate** (tick the engine at 1/5/10 Hz). That is the validated "coarse feed" regime — believable render, but
the *simulation* is coarser at low rates (car-following at a 1 s step; junction tunneling — see
`LIVE-CITY-STATUS.md:62-74`). If sim fidelity at low update rates matters, the alternative (§6, model B) is to
step the engine at a fixed good rate (e.g. 10 Hz) internally and **decimate** what's fed to the reconstruction
to the update rate (`IgBridgeConfig.FeedSampleEveryN` already does exactly this). Decide with the owner (§11).

---

## 3. The per-sample reconstruction recipe (the exact calls)

**Vehicles** — reuse `IgBridgeSession.EmitVehicles` (`IgBridgeSession.cs:149-243`) verbatim:
```
var resolved = drClock.ResolveAt(history, tau, lanes);            // DrClock.cs:258  (or Resolve(history,delay,lanes) wall-clock)
var realDt   = clampedElapsedSinceLastSampleForThisEntity;        // >= one sample step; IgBridgeSession.cs:193
var r        = recon.Resolve(handle, resolved, lanes, dims, realDt);   // KinematicReconstructor.cs:58 ; recon.CoarseFeed = true
if (!r.Ok) continue;                                              // no resolvable pose this frame -> skip
// emit CENTER (the IG's models pivot on center):
ig.Push(firstSampleForHandle ? IgSample.Created(id, tau, Car, r.CenterX, r.CenterY, r.Z, r.HeadingDeg)
                             : IgSample.Updated(id, tau,      r.CenterX, r.CenterY, r.Z, r.HeadingDeg));
```

**Pedestrians** — reuse `IgBridgeSession.EmitPeds` (`IgBridgeSession.cs:245-292`): peds are smooth by
construction (PathArc), so **no DrClock** — linear-resample the `PedSampleHistory` at `tau`, heading =
`atan2` of the bracketing step (velocity-derived), z = 0 (flat net) or the terrain height. Emit
`IgSample.*(id, tau, Ped, x, y, z, headingDeg)`. (For a DDS-shaped ped path, the LOD reconstructor
`Sim.Pedestrians/Lod/PedRemoteReconstructor.cs` + `demos/City3D/CityLib/PedReconstructor.cs` is the
alternative, but in-process the `PedStream` + `EmitPeds` path is lighter.)

All entities are sampled at **one shared `tau`** per host frame so the IG scene is time-coherent
(`IGBRIDGE-DESIGN.md §8`).

---

## 4. The embeddable host class — start from `IgBridgeRunner`, generalize it

`IgBridgeRunner` is the skeleton (`ctor :143`, `Tick :215`, state API `:186-210`). Two changes make it the
live-city IG host:

1. **Self-populating ambient flow instead of a fixed route file.** Today `IgBridgeRunner` spawns a pre-parsed
   `RouteDemand` (`:181, :222-241`). The live city instead **tops up** to a target concurrency every step
   between random edges, with the crossing-yield coupling. Port `SceneGen.BuildLiveCity`'s spawn loop +
   settings into the runner (§9): ambient cars (`CarTargetConcurrent≈160`, `CarSpawnPerStep=5`,
   `departBestLane:true`), the ped crowd (`PedStream` already does this), and
   `engine.CrowdSource = CompositeFootprintSource(pedFootprints, crossingOccupancy)` so cars yield to crossing
   peds. **Note `BuildLiveCity` is a one-shot recorder, not a tickable host** — you are lifting its *setup*,
   not calling it.
2. **Add the `Sample(now)` API** (generalize `IgBridgeSession.Advance/EmitInstant`): reconstruct all entities
   at one instant and return/push `IgSample`s, instead of draining to a JSONL trace. Keep the per-entity
   `new/upd/del` lifecycle bookkeeping (`_vehNewEmitted`/`_vehDone`, §8).

Shape the public API roughly as:
```
class LiveCityIgHost : IDisposable {
    LiveCityIgHost(LiveCityIgConfig cfg);   // net + settings + update-rate + CoarseFeed=true
    void Tick();                            // host calls at the UPDATE rate (advances the sim + buffers)
    IReadOnlyList<IgSample> Sample(double now);  // host calls at 20 Hz (reconstructs + returns updates)
    double SimTime { get; }
}
```
(Keep it deterministic: no `System.Random`, per-entity state, a host-owned sim clock — the same invariants
IgBridge holds.)

## 5. Wall-clock vs host-owned clock

- **Deterministic (recommended):** the host owns a monotonic sim clock (advance by wall-Δt × rate) and calls
  `Sample(now)`, which uses `DrClock.ResolveAt(history, now − lookahead, lanes)`. This is IgBridge's path
  (`_clock` used ONLY via `ResolveAt`, `IgBridgeSession.cs:36`) — testable, reproducible.
- **Wall-clock:** `DrClock.Pump(newestSampleTime, hold)` then `Resolve(history, delay, lanes)` — the live
  viewer's path (`DrClock.cs:236`). Jitter-immune but non-deterministic. Use only if the host can't own a clock.

## 6. Update-rate ⟷ sample-rate (coarse feed) — the knobs that matter

- **`CoarseFeed = true`** on the reconstructor (the viewer sets it unconditionally, `Reconstructor.cs:52`):
  enables the junction-turn-straddle discriminator so a turn spanning a wide bracket isn't mistaken for a
  lane change (fixes the "rides between lanes for seconds after a turn" artifact at low feed rates).
- **Lookahead ≈ one update interval.** IgBridge's `LookaheadSeconds` (default 0.1 s) keeps the query behind
  the newest buffered sample so `ResolveAt` interpolates rather than extrapolates. At a 1 Hz update that must
  be ~1 s (IgBridge learned this the hard way — too-small a lookahead retired live entities;
  `IGBRIDGE-RESUME.md` feed/emit section). Rule: `lookahead ≈ updateInterval` (`= 1/updateHz`).
- **Two engine-step models** (decide with the owner, §11):
  - **A — engine steps at the update rate** (owner's description). Simple; the reconstruction smooths the
    render; sim fidelity degrades below ~10 Hz (car-following, junction tunneling — `LIVE-CITY-STATUS.md`).
    This is the validated "1 Hz feed looks smooth" result.
  - **B — engine steps fast (e.g. 10 Hz), feed decimated to the update rate** via `FeedSampleEveryN`
    (`IgBridgeRunner.cs:29, :251, :288`). Better dynamics, more compute. Also already implemented.
- **Zero-latency option:** the IG can *extrapolate* from the last sample instead of buffering a delay — see
  the `FakeIg.Extrapolate` mode and the DR-error tradeoff (`IGBRIDGE-RESUME.md`, IG-consumption section).
  Relevant if the IG must be zero-latency.

## 7. 2D → 3D: z, pitch, roll (HPR)

- **z**: already resolved — `KinematicReconResult.Z` / `PoseResolver.Pose.Z` = lane-surface elevation (real
  on a 3-D net, 0 on the flat box). Emit it in the sample.
- **pitch**: reuse `demos/City3D/CityLib/Reconstructor.cs ComputePitchRad` (`:158-169`) — z-gradient 1 m
  ahead along the upcoming lane; 0 on a flat net.
- **roll / banking**: **not built.** The current `IgSample` wire is planar — `x,y,z,headingDeg` (yaw) only.
  Options: leave roll = 0; or derive a lean from lateral acceleration (new). Peds are upright, yaw-from-motion,
  no bank.
- **Schema decision:** if the IG wants full HPR / quaternion (not yaw-only), **extend `IgSample`**
  (`IgSample.cs`) with pitch/roll or a quaternion, and thread pitch through the emit. This is an open owner
  question (`IGBRIDGE-DESIGN.md §12 Q1`): full quaternion vs yaw-only. Confirm what the real IG accepts first.

## 8. The IG output — `new/upd/del` per entity

- `IgSample` (`IgSample.cs`): `new` on an entity's **first** emitted sample (carries `Model` = Car/Ped for
  the IG to pick a model), `upd` thereafter (pose only), `del` once the entity's history is frozen
  (despawned) and the query time passes it. `t` is **sim time** and the sole timing authority; ids are
  stable per entity. A `Finish()`-equivalent should `del` everything still open at shutdown.
- The host **pushes** these to the real IG's transport (the host owns the protocol). This layer's job ends at
  producing the coherent per-frame `IgSample` stream.

## 9. The live city — settings, net, ambient flow (reference values)

> **UPDATE (2026-07): use the real host, not `SceneGen.BuildLiveCity`.** The values below are still the
> right *reference numbers*, but the batch builder `SceneGen.BuildLiveCity` they came from has since
> **diverged** from the shipped demo (it hand-rolls its own Engine/LOD, no `LiveCityConfig`, no cooperative
> LC / per-area LOD / `LaneChangeMinSpeed`). The **real coupled host is now `Sim.LiveCity.LiveCitySim` +
> `LiveCityConfig`** — see **§9b**. Drive the integration from that; treat §9 as the parameter cheat-sheet.

Lift these into the host (historical origin: `src/Sim.Viz/SceneGen.cs`):
- **Net:** `scenarios/_ped/demo_city/box/net.xml` — a real ~4.75 km city with sidewalks, crossings,
  walkingareas, 51 TLs (peds need this infra). Optionally crop to the downtown hero block
  `[2055,2055,2895,2895]` for density. Also ships `scenario.rou.xml/.sumocfg/pois.json/buildings.json/
  zones.json`. (`PedStream` already loads this same net.)
- **Engine settings** (`:2001-2011`): `step-length` (0.5 in the recorder; for the IG host this is your update
  interval, or keep small under model B), `lanechange.duration 2.0`, `default.speeddev 0.0` (sigma 0 →
  deterministic), `time-to-teleport -1` (off), `LaneChangeMinSpeed 1.0` (no dead-stop lateral snaps),
  `VClass "passenger", Sigma 0.0`.
- **Ambient cars** (`:2041-2098`): top up to `CarTargetConcurrent≈160` (`LIVECITY_CARS`),
  `CarSpawnPerStep=5`, random crop edges, `departBestLane:true`.
- **Ped crowd** (`:1934-1960`): `PedDemandConfig` (SpawnRate 8/s, cap 160, MaxSpeed 1.3, weave on,
  crosswalk-signal compliance) via `PedLodManager` + `PedDemand`. (`IgBridgeRunner.EnablePeds` + `PedStream`
  already provide a deterministic version of this.)
- **Crossing-yield** (`:2016-2018`): `engine.CrowdSource = CompositeFootprintSource(pedFootprints,
  CrossingOccupancySource(...))` so cars stop for crossing peds — a **hard requirement** of the live city
  (`SUMOSHARP-LIVE-CITY-DECISIONS.md`, `SUMOSHARP-LIVE-CITY-DATA-REQUEST.md`).
- **Known caveats** (`LIVE-CITY-STATUS.md`): the loop-Δt vs engine-step mismatch + coarse-tick **tunneling**
  through ~4 m crossings at low rates (weigh against the chosen update rate), and the pre-existing dense
  multi-lane car-overlap limit (now fixed on main by the lane-change-overlap work — verify it's in your base).

## 9b. Running the full live-city simulation (the REAL host: `LiveCitySim` + `LiveCityConfig`)

The shipped coupled sim — **vehicles + pedestrians + crossing-yield + the per-area realism LOD /
high-realism zone** — is `Sim.LiveCity.LiveCitySim`, configured by `LiveCityConfig`. This is what the
City3D and raylib demos run, and what the faithful HTML replay renders. Drive the IG integration from this,
NOT from `SceneGen.BuildLiveCity` (§9).

### Construct + step (in-process, deterministic, no wall clock)
```csharp
var cfg = LiveCityConfig.ForRepoRoot(repoRoot);   // demo defaults (see below); env-overridable LIVECITY_*
using var sim = new LiveCitySim(cfg);
for (…) {
    sim.Step();                                   // advances cfg.Dt (0.5 s @ 2 Hz) of coupled sim
    var snap = sim.Sample();                       // authoritative cars + peds (per-tick)
    // OR reconstruct smoothly off the wires: sim.VehicleSource (+ DrClock/KinematicReconstructor)
    //                                        sim.PedSource   (+ PedRemoteReconstructor)
    //   -> IGBRIDGE-HTML-REPLAY-GUIDE.md is the exact recipe (cars §2/§5, peds §5b).
}
```
- **Vehicles:** `sim.VehicleSource` (an `IReplicationSource`, the in-mem replication wire) — history carries
  pose + speed + **UpcomingLanes** (the DR prediction the look-ahead needs). Same wire the viewers/IG consume.
- **Peds:** `sim.PedSource` (an `IPedReplicationSource`) — reconstruct with `PedRemoteReconstructor`.
- **Crossing-yield** is wired automatically when `cfg.YieldEnabled` (`Engine.CrowdSource =
  CompositeFootprintSource(HighPowerFootprints, CrossingOccupancySource)`), so cars brake for peds on zebras.

### The high-realism zone (per-area LOD)
`LiveCitySim.SetLcRealismZone(x, y, radius)` re-centres AND re-sizes the zone that drives BOTH (a) the
**per-car cooperative-LC / no-float** classification and (b) **ped ORCA promotion**. Defaults to a static
crop-centre pocket (`HighRealismPocketX/Y`, promote radius); in the viewers it is **camera-driven** (H cycles
Central / Follow / Locked). Inside the zone = high realism (cooperative LC, no lateral float, peds full-ORCA);
outside = cheap LOD. It is demo-gated and deterministic (pure fn of frozen state). For a headless IG host,
either leave it static or drive it from the IG camera each tick.

### Run it
```bash
# Faithful headless HTML replay (cars+peds+zone, the exact demo config) -- for dev/QA of the reconstruction:
dotnet run --project src/Sim.Viz -c Release -- --live-city-demo out/live-city.html
# Interactive native viewer (raylib), H cycles the camera-driven zone:
dotnet run --project src/Sim.Viewer -c Release -- --mode live-city
# Interactive 3D (Godot): demos/City3D (see demos/City3D/README.md)
```

### `LiveCityConfig` knobs (demo defaults; the parameter cheat-sheet §9 lists the reference values)
`DatasetDir` (the scenario folder), `Dt`/`SimHz`, `CarTargetConcurrent=160`, `CooperativeLaneChange=true`,
`MergeStoppedMinGap=5`, `MergeStoppedStrategicDeferDist=15`, `LaneChangeMinSpeed=1.5`, `YieldEnabled=true`,
crop bounds `X0/Y0/X1/Y1`. All env-overridable (`LIVECITY_*`).

## 9c. REQUIREMENT — the integrated host must accept ANY SUMO scenario from a folder

**The `demo_city/box` live-city scenario is the DEVELOPMENT + TEST fixture only** (it is also a committed
regression fixture — see `scenarios/_ped/demo_city/box/README.md`). The integration is built and validated
against it, but the delivered integrated solution **must be able to play any SUMO road-network scenario**,
not just the demo.

- **Parameterize the dataset directory.** `LiveCityConfig.DatasetDir` is already the single load point
  (`net.xml` + demand + ped infra + optional POI/zone/building JSON). Today `LiveCityConfig.ForRepoRoot`
  **hardcodes** `scenarios/_ped/demo_city/box`; the integrated host must instead take the scenario folder as
  input — e.g. a `--dataset <dir>` / `--scenario <dir>` arg or host-config field — and load `net.xml`
  (+ demand, + ped-walkable infra) from there. A caller points it at ANY SUMO net folder.
- **Graceful capability by dataset.** A bare SUMO net (no sidewalks/crossings/walkingareas) has no ped infra
  → run vehicles-only (peds/crossing-yield auto-off); a ped-equipped net enables the full coupled sim. The
  host should detect and degrade, not require the demo's exact companion files.
- **Keep the demo path as the golden dev/test loop.** Develop against `demo_city/box` (faithful replay +
  metric suite), but never bake its paths/params into the host — they are config, supplied per scenario.
- Related: the deferred "detach the demo data from the locked `box` fixture" task (`docs/TASKS.md`) is the
  same lever — the demo dataset must be a swappable folder, not a hardcoded path.

## 10. How we work (follow these — see `CLAUDE.md`)

- **Design-first.** Produce a design doc (HOW) + a task doc (stages + measurable success conditions) + a
  tracker before implementation. Get owner agreement first.
- **Parity is the iron law.** `dotnet test tests/Sim.ParityTests` stays **654/4 byte-identical**; the
  `Sim.Bench` determinism hash stays put; **never edit `Sim.Core`** for this (it's a consumer/host).
- **Determinism.** No `System.Random`, per-entity state, host-owned clock; a two-run byte-identical sample
  stream is the gate (IgBridge holds this).
- **Measure, don't eyeball** (`IGBRIDGE-METHODOLOGY.md`): the metric suite (yaw-accel reversals, no-slip,
  offset-from-centerline) + the 2D-visualization recipe are the tools for proving the reconstruction is good
  before wiring the real IG.
- **Reuse, don't reinvent:** all curve/heading math stays in `Sim.Viewer.Motion` (fix once, fix all).

## 11. Open questions for the owner (resolve up front)
1. **Engine step model A (step at update rate) vs B (step fast + decimate feed)?** — sets sim fidelity vs
   compute (§6).
2. **Orientation on the wire:** yaw-only (today) or full HPR / quaternion (needs the `IgSample` schema
   extension + pitch/roll, §7)? What does the real IG actually accept?
3. **z / terrain source of truth:** the SUMO lane-surface z (what we have), or does the IG own its terrain
   and want only planar pose + yaw (then z/pitch are moot)?
4. **Update rate default** (1/5/10 Hz) and **IG playout model** (buffered-interpolate with a small delay, or
   zero-latency extrapolate)?
5. **Live-city scope:** the whole ~4.75 km net or the hero crop; target concurrencies; peds on/off.

## 12. Suggested first steps
1. Read this + `IGBRIDGE-DESIGN.md` (§2 architecture, §4 timing, §5 2D→3D, §6 input stream, §8 lifecycle) +
   `IGBRIDGE-METHODOLOGY.md` + `SUMOSHARP-VIEWER-DR-SMOOTHING.md §8` (reimplementing in a new consumer) +
   `LIVE-CITY-STATUS.md` + `SUMOSHARP-LIVE-CITY-DECISIONS.md`.
2. Skim `IgBridgeRunner.cs` + `IgBridgeSession.cs` + `KinematicReconstructor.cs` — the code you generalize.
3. Answer §11 with the owner; write the design → tasks → tracker.
4. Build the `LiveCityIgHost` (Tick + Sample), reusing the reconstruction verbatim; prove it headless with
   the metric suite + a two-run determinism diff, then wire the real IG push.

## 13. Reference docs
- **IgBridge:** `IGBRIDGE-DESIGN.md` (as-built banner), `IGBRIDGE-REQUIREMENTS.md`, `IGBRIDGE-DECISIONS.md`
  (§5 as-built), `IGBRIDGE-METHODOLOGY.md`, `IGBRIDGE-VERSIONS.md`, `IGBRIDGE-RESUME.md`.
- **Reconstruction / DR:** `VIEWER-KINEMATIC-SMOOTHING-DESIGN.md`, `SUMOSHARP-VIEWER-DR-SMOOTHING.md`
  (esp. §8 + §11), `SUMOSHARP-DEADRECKONING.md`.
- **Live city:** `LIVE-CITY-STATUS.md`, `SUMOSHARP-LIVE-CITY-DECISIONS.md`,
  `SUMOSHARP-LIVE-CITY-DATA-REQUEST.md`, `SUBAREA-DEMO-CITY-DESIGN.md`, `LIVE-CITY-2D-BUILDER-DESIGN.md`,
  `LIVE-CITY-CROSSING-YIELD-DESIGN.md`, `LIVE-CITY-CROSSWALK-SIGNAL-DESIGN.md`.
- **Peds:** `PEDESTRIANS.md`, `PEDESTRIAN-OVERVIEW.md`.
- **Key code:** `src/Sim.IgBridge/{IgBridgeRunner,IgBridgeSession,PedStream,IgSample}.cs`,
  `src/Sim.Viewer.Motion/{KinematicReconstructor,DrClock}.cs`, `src/Sim.Core/PoseResolver.cs`,
  `demos/City3D/CityLib/{Reconstructor,PedReconstructor}.cs`.
- **The real live-city host (§9b):** `src/Sim.LiveCity/{LiveCitySim,LiveCityConfig}.cs` (coupled cars+peds +
  crossing-yield + `SetLcRealismZone`), consumed via `sim.VehicleSource` / `sim.PedSource`. The faithful
  HTML replay producer: `src/Sim.Viz/SceneGen.cs → BuildLiveCityDemo` (+ `Program --live-city-demo`).
- **How the smoothed HTML replay is made (cars + peds, the gotchas):** `docs/IGBRIDGE-HTML-REPLAY-GUIDE.md`.
- **The dev/test scenario fixture:** `scenarios/_ped/demo_city/box/README.md` (also a locked regression
  fixture — see §9c on parameterizing the dataset for any SUMO scenario).
