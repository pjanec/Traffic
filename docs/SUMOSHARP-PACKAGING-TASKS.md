# SUMOSHARP-PACKAGING-TASKS.md — staged tasks & success conditions

Work breakdown for the packaging rethink. **Design reference:** `SUMOSHARP-PACKAGING-DESIGN.md`
(sections/decisions cited per task — not restated here). **Tracker:** `SUMOSHARP-PACKAGING-TRACKER.md`.

**Global invariants (every task must hold these):**
- **G1 — Parity iron law.** `dotnet test` stays green and native-free (baseline, post-main-rebase:
  **446 passed, 0 failed, 3 skipped**; `Sim.Bench` determinism anchor unchanged). No simulation
  trajectory moves.
- **G2 — No native leak into the portable tier.** Nothing in `Ingest`/`Core`/`Replication`/
  `Viewer.Motion` may `PackageReference` a native dep (CycloneDDS, raylib, rlImgui) or
  `ProjectReference` a project that does.
- **G3 — Hermetic tests.** New guard/round-trip tests read committed csproj/source or run fully
  in-process — no build of native libs, no SUMO, no network (style of `RungB13PackagingTargetsTests`).
- **G4 — Additive metadata.** Packaging changes touch project metadata + code *organisation* only;
  public simulation APIs are unchanged except the transport-contract additions (P1, additive) and
  the `DrClock` decoupling (P2), both transport/viewer-side.

---

## Stage P0 — Reconcile the design docs with reality  ✅ (this session)

### P0.1 — Land the packaging design docs
- **Design ref:** whole `SUMOSHARP-PACKAGING-DESIGN.md`.
- **Success:** the three docs exist, cross-reference each other, and the §6 table has a row per
  current `src/` project. **Done.**

### P0.2 — Point `SUMOSHARP-API.md §1` at the new design
- **Design ref:** §0, §3 (D1, D3).
- **Success:** §1 notes packaging is owned by the new design; the "Core = Sim.Core + Sim.Ingest
  bundled" row is corrected to the shipped two-package reality; `SumoSharp.Runtime` is marked
  retired. **Done.**

---

## Stage P1 — Replication transport contract + neutral sample (D8, D9)

*The data model is the API; DDS is one binding. This stage makes that structural and is the
prerequisite for the motion package (which consumes the sample defined here).*

### P1.1 — Add the transport contract to `Sim.Replication`
- **Design ref:** §1 (principle 4), §3 (D8), §5.
- **Files:** new interfaces in `src/Sim.Replication/` (e.g. `IReplicationSink.cs`,
  `IReplicationSource.cs`), modelling the four logical channels — durable geometry-once, durable
  dims/lifecycle-once, per-frame movers, reverse command/status.
- **Success:**
  1. `IReplicationSink`/`IReplicationSource` are declared **in `src/Sim.Replication/`** and expose
     the four channels; `Sim.Replication` still builds for **both** net8.0 and ns2.1 (**G2**).
  2. The interfaces reference only the existing portable data-model types (`VehicleRecord`,
     `LaneGeo`, `TlEntry`, `FrameHeader`, …) and the codec — no transport/native type.
  3. **G1** holds (Replication is in the parity test graph).

### P1.2 — Add the timestamped sample + per-vehicle history to `Sim.Replication`
- **Design ref:** §3 (D9), §5.
- **Files:** new `TimestampedSample` (a `VehicleRecord` + sim/arrival time) and
  `IVehicleSampleHistory` (newest-last, capacity-bounded) in `src/Sim.Replication/`.
- **Shape constraint:** use the **same field set the publish side already stores** in
  `PublishScheduler`'s per-vehicle reference (`pos, speed, accel, posLat, latSpeed, laneHandle,
  time`) so publisher and subscriber share one sample shape — do not invent a parallel definition.
- **Success:**
  1. Both types live in `Sim.Replication` and build for net8.0 + ns2.1.
  2. A unit test round-trips a small history (append past cap, read newest-last, bracket a query
     time) and asserts ordering + cap eviction.
  3. The sample's fields match the `PublishScheduler` reference-state fields (one shape).
  4. **G1** holds.

### P1.3 — Refactor `Sim.Replication.Dds` to *implement* the contract
- **Design ref:** §3 (D8, D8-note on `DdsF32Col`).
- **Files:** `src/Sim.Replication.Dds/*` (`DdsTopics`, `CommandTopic`, `GeometryTlTopics`,
  `StructuredTopics`) adapted so the DDS side implements `IReplicationSink`/`IReplicationSource`;
  DDS samples adapt to `TimestampedSample`.
- **Success:**
  1. `Sim.Replication.Dds` exposes its publish/subscribe as the `Replication` contract; the
     four-channel model is no longer *defined* in the DDS package (only *bound* there).
  2. The `DdsF32Col`/`DdsI32Col` inline-array columns stay in `.Dds` (encoding, not model).
  3. Solution builds (DDS target compiles); **G1** holds (DDS is outside the parity gate, so it
     must remain green unchanged).

### P1.4 — Prove a second binding with an in-memory transport (hermetic)
- **Design ref:** §7 ("Prove a second binding").
- **Files:** `tests/Sim.ParityTests/ReplicationInMemoryTransportTests.cs` (a test-only
  `IReplicationSink`/`Source` pair wired sink→source in-process).
- **Success:**
  1. The test publishes geometry (durable), a per-frame mover batch, and a reverse command through
     the in-memory transport and reads them back byte-identically via the codec — **no CycloneDDS**.
  2. It runs inside `dotnet test`, native-free; total test count rises; **G1** otherwise unchanged.
  3. This demonstrates structurally that DDS is one option among several (D8).

---

## Stage P2 — `SumoSharp.Viewer.Motion` (the load-bearing refactor)

### P2.1 — Decouple `DrClock` from `DdsSubscriber` onto the P1 sample/history
- **Design ref:** §3 (D9), §5.
- **Files:** `src/Sim.Viewer.Core/DrClock.cs`, `src/Sim.Viewer.Core/DdsSubscriber.cs` (adapter),
  callers in `src/Sim.Viewer/Program.cs`.
- **Change:** `DrClock.Resolve` consumes `Sim.Replication`'s `TimestampedSample` /
  `IVehicleSampleHistory` (from P1.2); `DdsSubscriber` adapts its buffer to `IVehicleSampleHistory`
  instead of `DrClock` naming `DdsSubscriber.VehicleSample`/`DdsSubscriber.HistoryCap`.
- **Success:**
  1. `grep -R "DdsSubscriber" src/Sim.Viewer.Core/DrClock.cs` returns **nothing**.
  2. Solution builds; loopback + remote viewer paths compile against the new signature.
  3. A focused unit test builds a hand-made history and asserts `Resolve` picks the same
     interpolate/extrapolate branch and arc as before for a straight-lane and a junction-straddle
     case (regression pin for §5.2 behaviour).
  4. **G1** holds (viewer is outside the parity gate → stays green unchanged).

### P2.2 — Create the `Sim.Viewer.Motion` project (portable, packable)
- **Design ref:** §4, §5, §6.
- **Files:** new `src/Sim.Viewer.Motion/Sim.Viewer.Motion.csproj` + moved `DrClock.cs` and the
  as-built DR-pipeline scalar helpers from `PumpAndBuildVehicleDraws` in `Program.cs` — capped
  position error-smoothing (guide §10.2), motion-derived heading tilt (§10.3), stable manual delay
  (§10.1), heading low-pass; `Traffic.sln`; `Sim.Viewer.Core`/`Sim.Viewer` references. (The older
  §5.4 auto-delay / §5.5 low-pass are superseded — do not extract those.)
- **Do NOT re-extract arc extrapolation:** `DrExtrapolation.Arc` already lives in `Sim.Replication`
  and `DrClock` already delegates to it (landed with DR-error publishing). `Viewer.Motion` depends on
  `Sim.Replication` for it — keep one source of truth.
- **csproj shape:** `<TargetFrameworks>net8.0;netstandard2.1</TargetFrameworks>`, `IsPackable=true`,
  `PackageId=SumoSharp.Viewer.Motion`, `ProjectReference` → `Sim.Core`, `Sim.Ingest`,
  `Sim.Replication` **only**; link `src/Shared/NetstandardPolyfills.cs`; `System.Memory` on ns2.1.
- **Success:**
  1. `dotnet build src/Sim.Viewer.Motion -f netstandard2.1` and `-f net8.0` both succeed.
  2. **No** `ProjectReference` to `Sim.Replication.Dds`, **no** `Raylib`/`rlImgui` ref (**G2**).
  3. `dotnet pack -c Release` produces a `.nupkg` with `lib/net8.0` **and** `lib/netstandard2.1`.
  4. **G1** holds.

### P2.3 — Ship the DR/smoothing guide as the package README
- **Design ref:** §5 (last paragraph), §8.
- **Success:** the `.nupkg` `PackageReadmeFile` renders the reimplementation guide (guide §8) and
  carries the license/disclaimer.

---

## Stage P3 — Generic `SumoSharp.Viewer.Raylib` + demo-tool separation (D5, D10)

*Split the packaged **generic** viewer from the opinionated **demo tool**. The package must render any
SUMO stream and carry no domain (no evac) or curated content.*

### P3.1 — Add the render-overlay seam to the generic viewer (D10)
- **Design ref:** §3 (D10), §5.
- **Files:** `src/Sim.Viewer.Core` (generic host/render), `src/Sim.Viewer/Renderer.cs`.
- **Change:** the generic render loop exposes a per-frame custom-draw callback carrying the camera /
  world→screen transform + the frame's render data, so a consumer can paint extra layers.
- **Success:**
  1. A trivial test/sample overlay (e.g. draw a marker at a world point) renders through the seam
     without the viewer referencing the overlay's type.
  2. Solution builds; **G1** holds.

### P3.2 — Relocate the demo/evac code out of the packaged viewer into the demo layer
- **Design ref:** §3 (D5), §6.
- **Files:** move `DemoCatalog`, `DemoSession`, `EvacRenderSnapshot`, the evac draw pass, and the evac
  `EngineHost` mode out of `src/Sim.Viewer.Core` into the demo project (`src/Sim.Viewer` exe or a new
  `Sim.Viewer.Demos`); relocate the `Sim.Viewer.Core → Sim.Evac` `ProjectReference` to that layer;
  re-express the evac draw as a D10 overlay registered by the demo tool. `Traffic.sln`.
- **Success:**
  1. `grep -R "Sim.Evac\|DemoCatalog\|EvacRenderSnapshot" src/Sim.Viewer.Core` returns **nothing**.
  2. `src/Sim.Viewer.Core/Sim.Viewer.Core.csproj` has **no** `ProjectReference` to `Sim.Evac`.
  3. The demo tool still runs the `DemoCatalog` picker + live evac (manual smoke per `README.md`),
     drawing evac through the D10 seam.
  4. **G1** holds (Sim.Core's inert `OnAfterStep` seam unchanged; determinism hash + 446 tests green).

### P3.3 — Make the generic viewer packable; demo tool stays a sample
- **Design ref:** §3 (D5), §6.
- **csproj shape:** generic viewer `IsPackable=true`, `PackageId=SumoSharp.Viewer.Raylib`, net8.0,
  `ProjectReference` → `Viewer.Motion` + `Replication.Dds`; native raylib/ImGui + TTF as
  content/runtime assets. The demo exe stays `IsPackable=false`.
- **Success:**
  1. `dotnet pack` yields `SumoSharp.Viewer.Raylib.<v>.nupkg` (native raylib assets + font) with **no**
     evac/`DemoCatalog`/curated-scenario types in it.
  2. `Viewer.Raylib` is one of exactly two packable projects with a native dep (the other:
     `Replication.Dds`) — asserted by the guard (P5.2).
  3. The demo tool still runs local/loopback/remote + the picker; **G1** holds.

---

## Stage P4 — Dev-time & domain packages

### P4.1 — `SumoSharp.Testing` (from `Sim.Harness`)
- **Design ref:** §2 (Tier 3), §3 (D6).
- **Success:** `IsPackable=true`, `PackageId=SumoSharp.Testing`, net8.0; `dotnet pack` yields the
  `.nupkg`; the parity test project still references `Sim.Harness` by project ref; **G1**.

### P4.2 — `SumoSharp.Evac` (from `Sim.Evac`)
- **Design ref:** §2 (Tier 4), §3 (D6).
- **Success:** `IsPackable=true`, `PackageId=SumoSharp.Evac`, net8.0, `ProjectReference` → `Sim.Core`;
  `dotnet pack` yields the `.nupkg`; **G1** (Evac is in the parity test graph — trajectory unchanged).

---

## Stage P5 — Convenience & CI

### P5.1 — `SumoSharp` meta-package
- **Design ref:** §2 (Convenience), §3 (D7).
- **Files:** new `packaging/SumoSharp.Meta/SumoSharp.Meta.csproj` (no code; `PackageId=SumoSharp`,
  dependency refs to Core + Ingest + Replication + Viewer.Motion).
- **Success:** `dotnet pack` yields a code-less `SumoSharp.<v>.nupkg` whose `.nuspec` lists the four
  dependency IDs; installing it into a fresh console project restores all four.

### P5.2 — Extend the packaging guard test
- **Design ref:** §7.
- **Files:** `tests/Sim.ParityTests/RungB13PackagingTargetsTests.cs` (extend) or a sibling
  `PackagingLayoutTests.cs`.
- **Success (all hermetic, source-reading):**
  1. `Sim.Replication` and `Sim.Viewer.Motion` assert `<TargetFrameworks>` net8+ns2.1, `IsPackable=true`,
     and the expected `PackageId`.
  2. `Sim.Viewer.Motion.csproj` has **no** `Sim.Replication.Dds` project ref and **no**
     `Raylib`/`rlImgui` package ref.
  3. Exactly two packable projects reference a native dep: `Sim.Replication.Dds`, `Viewer.Raylib`.
  4. `IReplicationSink`/`IReplicationSource` are declared under `src/Sim.Replication/` (D8), not in
     `src/Sim.Replication.Dds/`.
  4b. The packaged generic viewer (`Sim.Viewer.Core`/`Viewer.Raylib`) has **no** `ProjectReference` to
     `Sim.Evac` and no `DemoCatalog`/`EvacRenderSnapshot` source (D5/D10 — package stays generic).
  5. The new test runs inside `dotnet test` and passes; total count rises; **G1** otherwise unchanged.

### P5.3 — Publish CI covers the new package IDs
- **Design ref:** §2, `SUMOSHARP-API.md §1` STATUS note.
- **Files:** `.github/workflows/publish.yml`.
- **Success:** a `v*` tag packs the full shipped set (Core, Ingest, Replication, Replication.Dds,
  Viewer.Motion, Viewer.Raylib, Testing, Evac, meta) at the tag version, uploads artifacts, pushes
  `.nupkg`+`.snupkg` with `--skip-duplicate`; push is *skipped not failed* when `NUGET_API_KEY` is
  absent (fork dry-run preserved). The offline parity gate remains a required pre-pack step.

---

## Sequencing notes
- **P0 done.** **P1 is now the critical path** (the transport contract + neutral sample) because P2
  consumes the sample it defines. P2 depends on P1; P3 depends on P2. **P4 is independent** of
  P1–P3 and can run in parallel. P5 depends on all prior stages.
- Each stage closes only when its success conditions are **verified first-hand** (build/pack/`dotnet
  test` re-run), per the CLAUDE.md accept gate — a reported "done" is unverified until proven.
