# 3D city demo — task breakdown & success conditions

Each task is a **self-contained briefing** (repo rule): it names the design reference (a §, not a copy),
the files it touches, its dependencies, and **numeric/observable success conditions**. Design:
`docs/DEMO-CITY3D-DESIGN.md`. Tracker: `docs/DEMO-CITY3D-TRACKER.md`. Read `CLAUDE.md` first.

**Standing gate (every task that touches `src/`).** Before any `src/` change, capture the baseline once:
`dotnet test Traffic.sln -c Release` (record pass/fail/skip counts) and the `Sim.Bench` determinism hash
(single **and** parallel). After the change, both must be **unchanged**. Demo-only tasks (Stages 1, 3,
and the demo parts of 2) must not modify any file under `src/`; the gate is therefore unaffected but is
re-asserted at close-out.

**Verification honesty.** Where a task's real proof is on-screen (GPU), the success condition is split
into a *headless-provable* part (done here) and a *desktop-only* part (user confirms). A task is
Opus-accepted on its headless part; the desktop part is flagged for the user.

---

## Stage 0 — Foundations: the reusable publisher + the local feed

### T0.1 — `SumoSharp.Host` library (`ReplicationPublisher`)
**Design:** "`SumoSharp.Host` — the missing reusable publisher". **Depends on:** nothing.
**Touches (additive — no existing file changes):** new `src/Sim.Host/Sim.Host.csproj`,
`src/Sim.Host/ReplicationPublisher.cs`; add the project to `Traffic.sln`.
**Do:**
- New project: TFMs `net8.0;netstandard2.1`, `IsPackable=true`, `PackageId=SumoSharp.Host`,
  `PackageReadmeFile=README.md`; `ProjectReference` → `Sim.Core`, `Sim.Replication` (in-repo build) but
  it must be *packable* against the package deps (Core, Replication). `IsPackable` default is false
  repo-wide, so set it true here explicitly.
- `ReplicationPublisher` with `PublishGeometryOnce(NetworkModel, IReplicationSink)` and
  `PublishStep(SimulationSnapshot, IReplicationSink)` per the design. Port the snapshot→`VehicleRecord`
  and `NetworkModel`→`LaneGeo` and `Tl*`→`TlEntry` logic from
  `src/Sim.Viewer.Core/DdsPublisher.cs` (read it; do **not** depend on it). `UpcomingLanes` from
  `SimulationSnapshot.LaneWindow`. Reuse the existing `PublishScheduler` for gating.
**Success conditions:**
1. `dotnet build src/Sim.Host` succeeds for **both** TFMs; no reference to any DDS type or to
   `Sim.Viewer.Core`.
2. A new xUnit test (in the existing parity/harness test project or a small `Sim.Host.Tests`): drive a
   real `Engine` on `scenarios/09-traffic-light` for ~30 steps, publish each step to an
   `InMemoryReplicationBus` via `ReplicationPublisher`, pump the source, and assert: geometry lane count
   == network lane count; every stepped vehicle appears with monotonic non-decreasing `Pos` on a given
   lane; at least one `TlEntry` with a non-`'g'`/`'G'` state is observed at some step (lights change).
3. `dotnet pack src/Sim.Host -c Release` produces `SumoSharp.Host.<version>.nupkg`.
4. **Standing gate unchanged** (additive project; nothing else rebuilt behaviourally).

### T0.2 — DRY rewire: DDS sink → `Replication.Dds`, `DdsPublisher` delegates
**Design:** "DRY rewire (parity-neutral)". **Depends on:** T0.1.
**Touches (`src/`, render-side):** new `src/Sim.Replication.Dds/DdsReplicationSink.cs`;
edit `src/Sim.Viewer.Core/DdsPublisher.cs` to compose `ReplicationPublisher` + `DdsReplicationSink`;
`Sim.Viewer.Core.csproj` gains a `ProjectReference` → `Sim.Host`.
**Do:** move the encode+DDS-write `IReplicationSink` implementation into `Replication.Dds` as
`DdsReplicationSink`; reduce `DdsPublisher` to reading `EngineHost.Snapshot` and delegating to
`ReplicationPublisher` + the sink. Public entry points and **wire bytes unchanged**.
**Success conditions:**
1. `LoopbackSelfTest` (native viewer's existing DDS loopback self-test) passes exactly as before.
2. Diff shows `DdsPublisher` no longer contains snapshot→record translation (it delegates); no
   duplicated publisher logic remains in the tree.
3. **Standing gate unchanged** (render-side only).
> If, on reading the code, this rewire proves to entangle `EngineHost` internals in a way that risks the
> gate, STOP and report — fall back to shipping T0.1 alone (demo still works via the packaged publisher)
> and defer the rewire. Clean-but-safe over clean-but-risky.

### T0.3 — Local feed + `nuget.config` + demo skeleton dirs
**Design:** "Local package feed". **Depends on:** T0.1 (needs `SumoSharp.Host` to pack).
**Touches (new, demo-only):** `demos/City3D/build.sh`, `demos/City3D/nuget.config`,
`demos/City3D/.gitignore`, `demos/City3D/README.md` (stub).
**Do:** `build.sh` packs Core, Ingest, Replication, Viewer.Motion, Host (+ Replication.Dds) into
`demos/City3D/local-nuget/`, then restores + builds the demo (projects added in later tasks).
`.gitignore` excludes `local-nuget/`, `.godot/`, `bin/`, `obj/`, export output. `nuget.config` exactly as
in the design (packageSourceMapping).
**Success conditions:**
1. From a clean tree, `bash demos/City3D/build.sh` populates `local-nuget/` with the expected `.nupkg`s.
2. A restore of a throwaway probe project referencing `SumoSharp.Host` resolves it **from the local
   feed**, and **fails** when `local-nuget/` is emptied (proving no `nuget.org` fallback for
   `SumoSharp.*`). Capture both outcomes in the task report.
3. `git status` shows `local-nuget/` untracked/ignored.

---

## Stage 1 — Local single-viewport demo (M1, the public-facing demo)

### T1.1 — Godot project skeleton, consuming packages, headless smoke
**Design:** "The demo … (Godot 4 .NET, C# only)". **Depends on:** T0.3.
**Touches (new):** `demos/City3D/Viewer/project.godot`, a main scene, `Viewer.csproj`
(`<PackageReference>` Core/Ingest/Replication/Viewer.Motion/Host from the local feed; `IsPackable=false`),
a `Main` C# node.
**Success conditions:**
1. `dotnet build demos/City3D/Viewer` succeeds resolving `SumoSharp.*` from the local feed (verified by
   emptying `local-nuget/` → build fails).
2. `godot --headless` runs the scene with a scripted "advance N frames, log a heartbeat line per frame,
   quit 0" — completes without exception in this environment.
3. No file under `src/` modified.

### T1.2 — In-process data path (engine → bus → DR reconstruction)
**Design:** "Data path" + DR-smoothing §5/§8. **Depends on:** T1.1.
**Touches:** `demos/City3D/Viewer/` C# (a `SimSource` that hosts `Engine`+`SimulationRunner`+
`ReplicationPublisher`→`InMemoryReplicationBus`; a `ReplicationLaneShapeSource`; per-frame
`DrClock`/`PoseResolver`/`DrPoseSmoother`).
**Success conditions (headless self-test, logged):**
1. Loading `09-traffic-light`, stepping, and reconstructing yields, for a tracked vehicle, per-frame
   `(x,y)` that advance monotonically along travel with **no back-jump > 0.5 m** between frames on steady
   motion, and stay within the net's bounding box.
2. `ReplicationLaneShapeSource` returns lane polylines matching the published geometry (spot-check ≥3
   lanes: endpoint coords within 1e-3 of the network's).
3. Toggling the source construction between "co-hosted engine" and "consume `bus.Source`" changes **no**
   reconstruction/render code (proves the local↔remote seam).
4. The SUMO→Godot coordinate transform and navi-heading→yaw conversion are exercised and asserted once:
   reconstructed yaw for a tracked vehicle matches the sim heading within 1°, and positions fall inside
   the net's bounding box in Godot space.

### T1.3 — Procedural roads
**Design:** "Procedural city → Roads". **Depends on:** T1.2. **Touches:** `demos/City3D/Viewer/` mesh gen.
**Success conditions:** headless — a road mesh is generated for every lane; total ribbon area is within
±5% of Σ(lane length × lane width) (sanity that widths/lengths are honoured); `LaneShapeZ` is sampled
(city-organic-L2 produces non-constant vertex Z; a flat net produces constant Z). **Desktop (user):**
roads look width-accurate and follow the network.

### T1.4 — Procedural buildings
**Design:** "Procedural city → Buildings". **Depends on:** T1.2. **Touches:** `demos/City3D/Viewer/`.
**Success conditions:** headless — building instances generated deterministically (same scenario+seed →
identical count and transforms across two runs); heights in a believable range (e.g. 6–60 m); none
overlap the road ribbons (set-back respected, spot-checked). **Desktop:** a believable box-city alongside
the roads.

### T1.5 — Cars (MultiMesh, true dimensions, smooth motion)
**Design:** "Procedural city → Cars". **Depends on:** T1.2. **Touches:** `demos/City3D/Viewer/`.
**Success conditions:** headless — per-instance scale equals each vehicle's vType `Length`×`Width`
(±1e-3) × a fixed height; instance count tracks live vehicle count each frame; yaw equals the
reconstructed heading (converted to Godot's convention) within 1°; Z follows `LaneShapeZ`. **Desktop:**
cars are true-size, correctly oriented, and move **smoothly** (no teleport/stutter) between sim updates.

### T1.6 — Simplified traffic lights
**Design:** "Procedural city → Traffic lights". **Depends on:** T1.2, T1.5.
**Touches:** `demos/City3D/Viewer/`.
**Success conditions:** headless — one head per TL-controlled approach is generated; head colour index
each frame equals the replicated signal letter (r/y/g) for that approach (assert the mapping over a run
of `09-traffic-light`: at least one red→green and one green→(amber→)red transition observed). **Desktop:**
poles+heads visible, colours match the sim, queues form/release on red/green.

### T1.7 — Scenario switch + scale/polish pass
**Design:** "Scale / scenario switching". **Depends on:** T1.3–T1.6.
**Touches:** `demos/City3D/Viewer/` arg parsing + a `run-local.sh`.
**Success conditions:** headless — `run-local.sh --scenario <path>` loads any of `09-traffic-light`,
`_bench/city-mixed-1k`, `_bench/city-organic-L2` and completes the headless smoke for each (geometry +
vehicles + TL heads counts logged, all > 0 where expected; L2 shows elevation). Default scenario =
`09-traffic-light`. **Desktop:** believable scale end-to-end (cars ~4.5×1.8 m against true lanes and
tens-of-metres buildings).

---

## Stage 2 — Remote (M2)

### T2.1 — `Sim.Host.App` generic headless DDS host
**Design:** "`src/Sim.Host.App`". **Depends on:** T0.1, T0.2.
**Touches (new, `src/`, additive):** `src/Sim.Host.App/` (`IsPackable=false`); add to `Traffic.sln`.
**Success conditions:**
1. `dotnet run --project src/Sim.Host.App -- --scenario scenarios/09-traffic-light --transport inmem`
   runs headless, steps, and a same-process consumer reconstructs the tracked vehicle (reuses the T1.2
   self-test assertions).
2. `--transport dds` runs and publishes (CycloneDDS native restores + loads on this Linux VM); a headless
   DDS subscriber in the same run (or a second process, if VM multicast permits) receives geometry +
   frames. Report which path (2-process vs same-process) actually ran here.
3. **Standing gate unchanged** (additive).

### T2.2 — Viewer remote mode (DDS source), single channel
**Design:** "Data path → Remote mode". **Depends on:** T2.1, T1.7.
**Touches (demo-only):** `demos/City3D/Viewer/` (a DDS `IReplicationSource`; `--transport dds|local`);
`Viewer.csproj` gains `SumoSharp.Replication.Dds` (local feed); `build.sh` already packs it.
**Success conditions:** headless — with `Sim.Host.App` publishing over DDS, the viewer in
`--transport dds --headless` receives geometry+frames and runs its reconstruction self-test (same
assertions as T1.2), with **no change** to the reconstruction/mesh/render code vs local (diff limited to
source construction + arg parsing). **Desktop (user):** a separate desktop viewer process renders the
`Sim.Host.App` stream live and smoothly.

---

## Stage 3 — Video wall + performance (M3)

### T3.1 — Auto-detect video wall
**Design:** "Multi-channel video wall". **Depends on:** T2.2.
**Touches (demo-only):** `demos/City3D/Viewer/` (screen detection, per-screen `Window`+`Camera3D`,
`--video-wall`).
**Success conditions:** headless — `--video-wall` code path constructs `GetScreenCount()` windows/cameras
from **one** DDS subscription (unit-level: with a stubbed 3-screen `DisplayServer`, 3 cameras built, 1
source); default (no flag) builds exactly 1 window. **Desktop (user):** windows span the detected
monitors, all animate from one host; launching the viewer twice yields two independent live subscribers.

### T3.2 — Performance ladder
**Design:** "Scale / scenario switching". **Depends on:** T2.2. **Touches (demo-only):** run scripts + README.
**Success conditions:** headless — `Sim.Host.App` sustains publishing for `city-30 → city-300 →
city-3000 → city-15000` and `city-mixed-1k`; log host step-rate and published-vehicle counts per rung
(documented in the README as the perf story). **Desktop (user):** the viewer renders `city-mixed-1k`
(~1k) at interactive frame rate via MultiMesh; the ladder shows the scale story.

---

## Stage 4 — Docs & close-out

### T4.1 — Demo README + doc wiring + final gate
**Depends on:** all above. **Touches:** `demos/City3D/README.md`, `docs/PACKAGES.md` (fill the
`Viewer.Motion` / add `SumoSharp.Host` example rows), a one-line pointer in `docs/DEMOS.md` and the repo
`README.md` "Documentation & live demos".
**Success conditions:**
1. README documents: one-command `build.sh`; run local; run remote (start `Sim.Host.App`, start viewer);
   `--video-wall`; `--scenario` switching; the alternative CI-artifact feed; and the explicit
   headless-vs-desktop verification split.
2. A fresh-clone dry run of the documented local path (`build.sh` → build → `--headless` smoke) succeeds.
3. **Final standing gate re-confirmed** green + hash unchanged; `git status` clean of ephemeral dirs.
