# IgBridge ↔ IG binding — TASK DESCRIPTION

The work broken into stages and tasks. Each task names its **design reference** (a section, not a
copy), the **files it touches**, its **dependencies**, and **success conditions** (specific,
measurable). References: `IGBRIDGE-REQUIREMENTS.md` (WHAT),
`IGBRIDGE-DESIGN.md` (HOW), `IGBRIDGE-DECISIONS.md` (locked decisions +
addendum), `SUMOSHARP-VIEWER-DR-SMOOTHING.md` (the reused stack). Tracker: `IGBRIDGE-TRACKER.md`.

**Code name:** `IgBridge`. No `"IgBridge"` string in any code identifier/namespace/project/file.
**Parity invariant (every task):** `dotnet test` stays green and every golden byte-identical.

---

## Stage 0 — retarget, scaffold, prove .NET 6

### T0.1 — Retarget `Sim.Pedestrians` to `netstandard2.1;net8.0`
- **Design ref:** DECISIONS §1 Q6.
- **Files:** `src/Sim.Pedestrians/Sim.Pedestrians.csproj` (TFM list); any ped source using a
  net8-only API (fix to a netstandard2.1-safe form; `Sim.Core`/`Sim.Replication` deps are already
  `netstandard2.1`).
- **Deps:** none.
- **Success conditions:**
  1. `Sim.Pedestrians` compiles for **both** `netstandard2.1` and `net8.0` (`dotnet build`).
  2. `dotnet test` is **green** and **byte-identical** goldens (diff the parity test output before/after —
     TFM change must not alter any behavior).
  3. No new package dependency added solely to satisfy netstandard2.1 beyond what `Sim.Core`/`Sim.Replication`
     already carry (e.g. `System.Memory`), and none that pulls a native/GPU/UI dep.

### T0.2 — Scaffold `Sim.IgBridge` (producer lib) + `Sim.IgBridge.Host` (console)
- **Design ref:** DECISIONS §2, §6 Q6.
- **Files (new):** `src/Sim.IgBridge/Sim.IgBridge.csproj` (`netstandard2.1;net6.0;net8.0`, refs
  `Sim.Core`, `Sim.Replication`, `Sim.Ingest`, `Sim.Viewer.Motion`, `Sim.Pedestrians`);
  `src/Sim.IgBridge.Host/Sim.IgBridge.Host.csproj` (`net8.0`, Exe, refs `Sim.IgBridge` + `Sim.Viz`);
  add both to `Traffic.sln`.
- **Deps:** T0.1 (so `Sim.IgBridge` can reference `Sim.Pedestrians` under netstandard2.1/net6.0).
- **Success conditions:**
  1. `Sim.IgBridge` **builds for `net6.0`** (this is the literal proof of the .NET-6 embed claim,
     requirement §7 ".NET compatibility") and for `netstandard2.1` + `net8.0`.
  2. `Sim.IgBridge.Host` builds for `net8.0`.
  3. Solution builds; `dotnet test` still green.

### T0.3 — Fixed-10 Hz core loop + per-entity ring buffers + sim clock
- **Design ref:** DECISIONS §2 [1]/[2], §6; DESIGN §2 [1], §4.
- **Files:** `src/Sim.IgBridge/` (e.g. `IgBridgeRunner.cs`, `SampleBuffers.cs`).
- **Deps:** T0.2.
- **Success conditions:**
  1. Loads `scenarios/_ped/demo_city/box` with `StepLength=0.1`; advances `Engine.Step()` N steps;
     `Engine.CurrentTime` increments in exact `0.1` s ticks (assert no wall-dt influence).
  2. Each step appends one `TimestampedSample` per live vehicle handle to that entity's
     `VehicleSampleHistory` (from `VehicleHandles`/`Pos`/`PosLat`/`Speed`/`Acceleration`/`LaneHandles`
     + `GetUpcomingLanes`); ped state buffered via `PedestrianWorld.Step(now,dt)` + `PedPublisher`.
  3. Unit test: two runs of K steps produce **identical** buffered sample sequences (determinism, no
     `System.Random`, order-independent).

---

## Stage 1 — reconstruct + emit + FakeIg + baseline

### T1.1 — `DrClock.ResolveAt(history, sampleT, lanes)` deterministic seam
- **Design ref:** DECISIONS §5.1.
- **Files:** `src/Sim.Viewer.Motion/DrClock.cs`.
- **Deps:** none (can precede/parallel Stage 0).
- **Success conditions:**
  1. New `ResolveAt(IReadOnlyList<TimestampedSample>, double sampleT, ILaneShapeSource)` contains the
     current `Resolve` body; existing `Resolve(history, delay, lanes)` delegates via
     `ResolveAt(history, _renderSim − delay, lanes)`.
  2. A golden-value test asserts `Resolve` and the old inline path return **identical** `Resolved` for a
     fixed history + renderSim + delay (pure refactor, no behavior change).
  3. `dotnet test` green.

### T1.2 — Vehicle reconstruction + emit + trace
- **Design ref:** DECISIONS §3 (schema), §4 (lookahead), §2 [3]; README pipeline.
- **Files:** `src/Sim.IgBridge/` (`Reconstructor.cs`, `IgSample.cs`, `TraceWriter.cs`).
- **Deps:** T0.3, T1.1.
- **Success conditions:**
  1. At the emit cadence (20 Hz), for each live vehicle: `DrClock.ResolveAt(history, tEmit, laneSource)`
     → `PoseResolver.Resolve(..., ChordHeading)` → `DrPoseSmoother.Smooth(...)` → emit an `upd` record
     `{id, t=tEmit, x, y, h}`; `new` on first sample, `del` on `SimEvent` arrival/removal.
  2. Trace is valid JSONL, records ordered by `(t, kind)`; every entity has exactly one `new` before
     its first `upd` and (if it despawns in-run) exactly one `del` after its last `upd`.
  3. Determinism: two runs → **byte-identical** trace files.

### T1.3 — `FakeIg` replay (2-most-recent interp + delay + threshold)
- **Design ref:** DECISIONS §1 Q3, §4; REQUIREMENTS R7.
- **Files:** `src/Sim.IgBridge.Host/` (`FakeIg.cs`).
- **Deps:** T1.2.
- **Success conditions:**
  1. Consumes a trace; per entity keeps the **2 most recent** samples; at `tPlay = clock − igDelay`
     (igDelay swept 0.5–1.0 s) **linear-interpolates x,y** and **shortest-arc interpolates heading**
     between the bracketing pair; extrapolates only when forced (to exercise delay tuning).
  2. `new`/`del` honored: an entity is shown only between its create and remove; a per-pair displacement
     above the **jump threshold** is applied as an **immediate jump**, not a smeared slide.
  3. Reconstructs a known synthetic trace to within floating tolerance of the hand-computed expected
     poses (unit test).

### T1.4 — Side-by-side render (raw vs FakeIg-reconstructed)
- **Design ref:** DESIGN §9 analysis; REQUIREMENTS R8; survey pt.5 (multi-scene payload).
- **Files:** `src/Sim.IgBridge.Host/` (render driver) using `Sim.Viz` `WriteHtml` with two
  `ScenePayload`s ("raw", "reconstructed") in one `ReplayData`.
- **Deps:** T1.3.
- **Success conditions:**
  1. Produces one self-contained HTML with a scene selector toggling **raw SUMO** (direct engine
     `x,y,angle`) vs **FakeIg-reconstructed** from the same run.
  2. Vehicles render as oriented boxes, peds as discs; both scenes share the same view bbox and dt.

### T1.5 — Baseline metrics pass
- **Design ref:** REQUIREMENTS R8/§9.3; DECISIONS §7, C2.
- **Files:** `src/Sim.IgBridge.Host/` (`Metrics.cs`), output table.
- **Deps:** T1.3.
- **Success conditions:**
  1. Computes per-entity **max yaw-rate (°/s), max yaw-jerk (°/s²), max lateral accel (m/s²), max C1
     position gap (m/step), measured lane-change duration (s)** for **raw** and **reconstructed**
     streams; prints a table + writes machine-readable output (JSON/CSV).
  2. Junction turns already show **reconstructed yaw-rate ≪ raw** at baseline (before T2.1), confirming
     the arc-window/ChordHeading reuse works. (Lane-change duration still ~1 tick until T2.1.)

---

## Stage 2 — lane-change ease + lifecycle

### T2.1 — Lane-change ease over ~1.3 s in `Sim.Viewer.Motion`
- **Design ref:** DECISIONS §5.2; DESIGN §7 "Instant lane change"; SMOOTHING §? (lateral straddle).
- **Files:** `src/Sim.Viewer.Motion/` (new `LaneChangeEaser.cs` or resolve-side helper); wired into the
  IgBridge reconstruction; **not** in IgBridge itself.
- **Deps:** T1.5 (baseline to compare against).
- **Success conditions:**
  1. A one-tick displacement **mostly perpendicular to heading** (lane snap, ≳ ~2 m lateral in one tick)
     is spread as a **smoothstep over `W=1.3 s`** (tunable 1.2–1.5); straight cruise and genuine turns
     produce **≤0.05 m** spurious lateral ease (inert when it should be).
  2. Reconstructed **measured lane-change duration ∈ [1.2, 1.5] s** (was ~1 tick at baseline).
  3. Reconstructed **max lateral accel during a lane change is bounded** and **< 40%** of the raw stream's
     one-tick value; no position discontinuity > emit-step·speed introduced.
  4. Because it's in `Sim.Viewer.Motion`, a `dotnet build` of the 2D viewer path picks it up (reuse
     proven structurally); parity `dotnet test` unaffected.

### T2.2 — Entity lifecycle + discontinuity handling
- **Design ref:** DECISIONS §1 Q3, §3; DESIGN §8; REQUIREMENTS R4.
- **Files:** `src/Sim.IgBridge/` (lifecycle emit from `SimEvent`/ped add-remove; discontinuity detector).
- **Deps:** T1.2.
- **Success conditions:**
  1. `new` emitted on first appearance (with `model`), `del` on route-end/despawn (`SimEvent` Arrived /
     ped `Remove`); FakeIg shows each entity exactly over its live interval (no ghost, no early vanish).
  2. A supra-threshold jump that is **not** a lane change (e.g. a forced LOD nudge in a ped test) is
     **not** smoothed as motion — handled by a `del`+`new` re-key or passed through for the IG threshold,
     asserted by a unit test that the reconstructed path shows a jump, **not** a 60 m/s slide.
  3. All entities in a frame share one `tQuery` (time-coherent scene).

---

## Stage 3 — tune & prove

### T3.1 — Sweep & lock the latency budget
- **Design ref:** DECISIONS §4, §7; REQUIREMENTS §7 latency, §9.
- **Files:** `src/Sim.IgBridge.Host/` (sweep driver), a short results note appended to DECISIONS §7.
- **Deps:** T2.1, T2.2.
- **Success conditions:**
  1. Sweeps `igDelay ∈ {0.5,0.75,1.0}` s, emit-rate ∈ {20,30} Hz, `W ∈ {1.2,1.3,1.5}` s; reports the
     metrics table per combo.
  2. Confirms **no extrapolation** occurs in FakeIg at `igDelay ≥ 0.5 s` with 20 Hz emit (every `tPlay`
     bracketed); documents the chosen defaults and the total latency (§4) as the locked budget.

### T3.2 — Commit the metrics regression check
- **Design ref:** REQUIREMENTS §9.3, C2; DECISIONS §6.
- **Files:** a test in the IgBridge test project asserting reconstructed metrics stay under committed
  bounds on `demo_city/box`; committed expected-metrics fixture.
- **Deps:** T3.1.
- **Success conditions:**
  1. A test fails if reconstructed max yaw-rate/yaw-jerk/lateral-accel exceed committed bounds or if
     lane-change duration leaves [1.2,1.5] s — i.e. a future smoothing regression is caught.
  2. This test runs in the **offline** loop and needs **no SUMO** and no network.

### T3.3 — Determinism & parity gate (final)
- **Design ref:** DECISIONS §6; REQUIREMENTS §7, §9.5.
- **Files:** —
- **Deps:** T3.2.
- **Success conditions:**
  1. Two full PoC runs produce **byte-identical** trace + metrics output.
  2. `dotnet test` green; every committed golden byte-identical to pre-feature (parity untouched).
  3. `Sim.IgBridge` still builds for `net6.0`.
