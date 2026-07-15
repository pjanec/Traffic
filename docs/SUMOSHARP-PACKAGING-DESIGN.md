# SUMOSHARP-PACKAGING-DESIGN.md — à-la-carte NuGet packaging (the rethink)

**Status:** design of record for how this repo is sliced into NuGet packages.
**Supersedes** the package table in `SUMOSHARP-API.md §1`, which predates Replication, the
native viewer, `PoseResolver`/`DrClock`, and the evacuation subsystem. `SUMOSHARP-API.md` still
owns the *public-API surface* design (handles, read API, execution model); this doc owns *how the
assemblies are grouped, targeted, and shipped*. **Companion docs:** `SUMOSHARP-API.md` (API),
`SUMOSHARP-VIEWER-DR-SMOOTHING.md` (the render-side motion reconstruction that the viewer/motion
package documents and ships), `DESIGN.md` (simulation architecture).

**WHAT this is for:** the goal the user set — turn this repo into NuGet package(s) so the SUMO
port drops cleanly into a **simulation or game engine**, letting an integrator take **only what
they need** (engine only; engine + streaming; engine + streaming + render-side motion; the full
desktop viewer; dev-time tooling). Pay-for-what-you-use, native/heavy dependencies quarantined at
the leaves, one portable core.

---

## 0. Why a rethink (what changed since `SUMOSHARP-API.md §1`)

The original §1 table listed three intended packages — `SumoSharp.Core` (Sim.Core **+** Sim.Ingest
bundled), an optional `SumoSharp.Runtime`, and a dev-time `SumoSharp.Tools`. Since then:

1. **Two packages shipped, not one.** As-built, `Sim.Core` and `Sim.Ingest` publish as **separate**
   packages (`SumoSharp.Core`, `SumoSharp.Ingest`). The §1 table's "bundle them" intent never
   happened, and the split is actually the *better* shape (see §3). This doc adopts the reality.
2. **Replication landed** (`SumoSharp.Replication`, `SumoSharp.Replication.Dds`) — a whole
   transport-agnostic dead-reckoning wire layer plus a DDS transport. Absent from §1 entirely.
3. **The render-side motion stack landed.** `PoseResolver` + `ILaneShapeSource` (in `Sim.Core`) and
   `DrClock` (in `Sim.Viewer.Core`) turn sparse authoritative/DDS samples into smooth per-frame
   poses. This is the single most reusable thing a *game/3D* integrator wants, and it is currently
   **not packaged** and **entangled with the DDS transport** (see §5).
4. **A native desktop viewer landed** (`Sim.Viewer`, raylib + ImGui) with a headless brain
   (`Sim.Viewer.Core`). Native, desktop-only, not packaged.
5. **A panic-evacuation subsystem landed** (`Sim.Evac`) — an optional domain extension over the
   Core seams. Not packaged.
6. **`SumoSharp.Runtime` never split out.** The async `SimulationRunner` lives in `Sim.Core` today
   and carries no extra dependencies. This doc **retires** the separate-Runtime idea (see §3, D3).
7. **Dead-reckoning-error publishing landed** (`SUMOSHARP-DR-ERROR-PUBLISHING-DESIGN.md`), and it
   already put the **DR math** (`DrExtrapolation`) and the **publish-decision principle**
   (`DrErrorPublishPolicy`, `PublishScheduler` per-vehicle reference state) into `Sim.Replication`,
   operating on `VehicleRecord` fields and **never a DDS type**. Its §8 states the same layering the
   user set for this rethink — *"the data model + messages are the transport-agnostic API;
   `Sim.Replication` owns them + the DR math + the publish-decision principle; the DDS layer is just a
   transport; `Sim.Core` stays network-agnostic."* So D8/D9 below are **extending an already-adopted
   principle to the subscribe/transport side**, not introducing a new one — and part of the motion
   extraction (the shared arc extrapolation) is **already done**.
8. **The native viewer is being repositioned as an interactive demo tool**
   (`SUMOSHARP-VIEWER-DEMO-EVAC-DESIGN.md`): a `local`-mode `DemoCatalog` scenario picker with a live
   panic-evac category. `Sim.Viewer.Core` now `ProjectReference`s **`Sim.Evac`**, and `Sim.Core`
   gained a nullable/inert `SimulationRunner.OnAfterStep` seam (parity + determinism hash unchanged;
   446 tests green). This sharpens the viewer split (D5) — reusable motion math vs the showcase app —
   and is why the raylib-viewer-as-package decision is now open again.

The user's ask names all of these: viewer tools as (maybe standalone) packages, and a
reimplementation guide for dead-reckoning/smoothing across viewers. So the packaging surface has to
grow past the four shipped packages, and the growth has to stay **layered and optional**.

---

## 1. Design principles (the rails)

1. **One portable core, minimal dependencies.** `SumoSharp.Core` + `SumoSharp.Ingest` stay
   pure-managed and multi-target `net8.0;netstandard2.1` so **Unity (Mono/IL2CPP), Godot, and
   console/server** hosts can all consume them. Nothing in the base graph pulls a native binary.
2. **Quarantine native / heavy dependencies at the leaves.** `CycloneDDS.NET` (native DDS) and
   `Raylib-cs`/`rlImgui-cs` (native GPU + ImGui) live **only** in leaf packages an integrator opts
   into. A game engine that has its own renderer must be able to take the motion math **without**
   raylib, and the streaming client **without** committing to DDS.
3. **Separate "reconstruct motion" (pure math) from "render motion" (a windowing/GPU toolkit).**
   This is the load-bearing new split (§5). The reconstruction pipeline (`DrClock`, `PoseResolver`,
   `ILaneShapeSource`, stable delay, capped position error-smoothing, motion-derived heading tilt) is
   portable scalar maths and belongs in
   its own portable package; raylib/ImGui belong in an optional desktop-viewer package.
4. **The replication data model *is* the API; transports are bindings.** `SumoSharp.Replication`
   is the replication **API** — the data model (records + their semantics), the packed codec, the
   publish policy, **and the transport contract** (`IReplicationSink`/`IReplicationSource` over the
   four logical channels: durable geometry-once, durable dims/lifecycle-once, per-frame movers, and
   the reverse command/status channel). It carries **no** transport dependency. `SumoSharp.Replication.Dds`
   is **one binding** of that contract; a WebSocket / named-pipe / shared-memory / ENet transport is a
   sibling leaf that implements the same interface. Consumers — viewers, remote clients, other-language
   reimplementations — program against the **data model**, never against DDS. (Decisions D8, D9.)
5. **Additive and parity-inert.** Packaging changes touch only project metadata and *code
   organisation*; they must never alter a simulation trajectory. The offline parity gate
   (`dotnet test`, no SUMO, no native libs) stays byte-identical, and a guard test pins the
   packaging invariants (§7). This is the CLAUDE.md iron law applied to packaging.
6. **Every package is self-describing and legally honest.** Each carries the dual-license
   (`EPL-2.0 OR GPL-2.0-or-later`) and the "unofficial reimplementation of Eclipse SUMO" disclaimer
   in its own README (§8).

---

## 2. The package graph (target state)

Tiers are strictly layered — an arrow means "depends on". Native/heavy leaves are marked ⚠.

```
Tier 0 — Engine (portable: net8.0 + netstandard2.1)
  SumoSharp.Ingest      parsers + net/rou/sumocfg model                     (leaf)
  SumoSharp.Core        engine, obstacle store, SoA read API, runtime        → Ingest
                        demand, SimulationRunner, PoseResolver, ILaneShapeSource

Tier 1 — Replication API (the data model) / transports (bindings)
  SumoSharp.Replication      DR data model + codec + publish policy +        → Core   (portable)
                             transport contract (IReplicationSink/Source, 4 channels) +
                             timestamped sample + per-vehicle history
  SumoSharp.Replication.Dds  ⚠ CycloneDDS *binding* (native)                 → Replication (net8.0)
  SumoSharp.Replication.*    future sibling bindings (WebSocket / named-pipe / shared-mem / ENet)
                             — each implements the same contract; DDS is just the first

Tier 2 — Render-side motion / viewers  (generic packages; opinionated demo is a sample, below)
  SumoSharp.Viewer.Motion    DrClock + DR pipeline glue                       → Core, Replication
                             consumes the Replication timestamped sample + history;
                             (portable render-side reconstruction; NO raylib, NO DDS)   (portable)
  SumoSharp.Viewer.Raylib ⚠  GENERIC 2D viewer: renders ANY SUMO stream       → Viewer.Motion,
                             (vehicles/roads/lanes/TLs/HUD/camera) + a render-   Replication.Dds (net8.0)
                             overlay seam (D10). NO evac, NO DemoCatalog, NO curated scenarios.

Tier 3 — Dev-time / tooling  (net8.0, optional, never referenced by a shipping game build)
  SumoSharp.Tools            SUMO-binary fetch + netconvert/duarouter wrappers          (net8.0)
  SumoSharp.Testing          Sim.Harness: FCD/tripinfo/summary parse + tolerance compare → Core (net8.0)

Tier 4 — Domain extensions
  SumoSharp.Evac             panic-evacuation subsystem over Core seams       → Core    (net8.0)

Convenience
  SumoSharp (meta)           references Core + Ingest + Replication (+ Viewer.Motion) — "just simulate & stream"
```

**Not packaged — shipped as samples/demos:** `Sim.Run`, `Sim.Viz`, `Sim.Bench`, `Sim.BenchCity`,
`Sim.ExtDemo`, `Sim.EvacProfile`, `Sim.LiveHost`, and **the native-viewer demo tool** — the
opinionated showcase (`DemoCatalog` picker + live evac overlay + curated scenarios; references
`Sim.Evac`). The demo tool is built *on top of* the generic `SumoSharp.Viewer.Raylib` package via the
D10 overlay seam; its reusable rendering code lives in the package, its curated/evac content stays in
the sample. These demonstrate; they are not the product.

### Legend of "who installs what"

| Integrator | Installs |
|---|---|
| Headless sim / training / digital-twin backend | `SumoSharp.Core` (+ `Ingest`) |
| …that streams state to a decoupled process | + `SumoSharp.Replication` (the API) + a transport binding, e.g. `SumoSharp.Replication.Dds` |
| **Game / 3D engine with its own renderer** | `SumoSharp.Core` + `SumoSharp.Viewer.Motion` (+ a transport) |
| Wants the batteries-included **generic** 2D desktop view | + `SumoSharp.Viewer.Raylib` (renders any sim; add own overlays via the D10 seam) |
| Content pipeline (build a `.net.xml` from OSM) | + `SumoSharp.Tools` (dev-time) |
| Validating their own scenarios vs SUMO goldens | + `SumoSharp.Testing` (dev-time) |
| Evacuation / crowd scenarios | + `SumoSharp.Evac` |
| "Just give me the usual bundle" | `SumoSharp` (meta) |

---

## 3. Per-package decisions

- **D1 — Keep `SumoSharp.Ingest` a separate package (do not fold into Core).** Reality already
  ships it separate; the split is clean (Ingest is a true leaf with no internal deps) and lets a
  host that feeds pre-parsed data or its own network format take the engine without the parser.
  `SumoSharp.Core` `PackageReference`s `SumoSharp.Ingest`, so `install Core` still transitively
  pulls it — the split costs a consumer nothing but buys flexibility. **The §1 table's "one bundled
  package" intent is dropped.**
- **D2 — `PoseResolver` + `ILaneShapeSource` stay in `SumoSharp.Core`.** They already live in
  `Sim.Core`, are dependency-light, and are shared by the engine's opt-in production render mode as
  well as by every viewer. Moving them out would churn Core's public API for no gain. The motion
  *pipeline* (`DrClock` + glue) is what moves into `Viewer.Motion` (§5).
- **D3 — Retire `SumoSharp.Runtime`.** The async `SimulationRunner` is in `Sim.Core` and adds no
  dependency. A separate package would be churn with no payoff; a training build that wants only the
  stepped API simply never touches `SimulationRunner`. Documented here so the stale §1 idea is
  formally closed.
- **D4 — `Replication` portable, `Replication.Dds` native-leaf.** Already the case; codified as a
  principle so no transport dependency ever creeps into `Replication` (which is `net8.0;ns2.1`).
- **D8 — The transport contract is an explicit interface in `Replication`, and DDS is one binding.**
  Lift the four logical channels (durable geometry-once, durable dims/lifecycle-once, per-frame
  movers, reverse command/status) — today realized *inside* the DDS package (`DdsTopicNames`,
  `DdsTopics`, `CommandTopic`, `GeometryTlTopics`) — into a portable `IReplicationSink` /
  `IReplicationSource` contract in `Sim.Replication`. `Sim.Replication.Dds` becomes the DDS
  *implementation* of that contract; a WebSocket / named-pipe / shared-memory / ENet transport is a
  sibling that implements the same interface. This makes "DDS is one of multiple transfer options"
  **structural**, not aspirational, and lets a second transport be an implement-the-interface job.
  A DDS-specific *representation* (the `DdsF32Col`/`DdsI32Col` inline-array SoA columns — the
  "structured chunk" topic option) stays in `.Dds`; it is an encoding optimization, not the model.
- **D9 — The timestamped sample + per-vehicle history live in `Replication` (the data model is the
  API).** A `VehicleRecord` + its sim/arrival timestamp (and the newest-last per-vehicle history the
  DR clock reads) are part of the data-model API, not a render-side detail. They live in
  `Sim.Replication`; `DrClock` (in `Viewer.Motion`) consumes the **API** sample type, and each
  transport (DDS today) *fills* it. This is what removes `DrClock`'s coupling to
  `DdsSubscriber.VehicleSample` (§5). **Align the shape with the precedent already in
  `PublishScheduler`:** the publish side already stores a per-vehicle DR reference `{ pos, speed,
  accel, posLat, latSpeed, laneHandle, time }`. The subscribe-side timestamped sample should be the
  same field set (it *is* the DR state the viewer predicts from), so publisher and subscriber share
  one sample shape — not two parallel definitions.
- **D5 — Split a *generic* viewer (packaged) from the *demo tool* (a sample).** Main repositioned the
  native viewer as an interactive demo/showcase tool (`SUMOSHARP-VIEWER-DEMO-EVAC-DESIGN.md`): a
  `local`-mode `DemoCatalog` picker + a live panic-evac category, and `Sim.Viewer.Core` now
  `ProjectReference`s **`Sim.Evac`**. That coupling makes the viewer *opinionated about a domain
  feature* — unacceptable for a NuGet, which must render **any** SUMO simulation generically. So the
  viewer is split into three layers by reuse value:
  - **`SumoSharp.Viewer.Motion`** (portable NuGet) — motion reconstruction (`DrClock` + as-built
    pipeline helpers + `PoseResolver`). No renderer, transport, or domain. (P2.)
  - **`SumoSharp.Viewer.Raylib`** (native NuGet) — a **generic** raylib viewer that renders any SUMO
    stream (vehicles / roads / lanes / TLs / HUD / camera). **No evac, no `DemoCatalog`, no curated
    scenarios.** It exposes a generic **render-overlay seam** (D10) so any domain can paint extra
    layers without the viewer depending on them.
  - **The demo tool** — a **sample exe, not packaged** — `DemoCatalog` + picker + the evac overlay
    (registered through the D10 seam). This is the only place `Sim.Evac` is referenced.
  **The refactor:** move `DemoCatalog`, `DemoSession`, `EvacRenderSnapshot`, the evac draw pass, and
  the evac `EngineHost` mode **out** of the packaged viewer core into the demo layer, and relocate the
  `Sim.Viewer.Core → Sim.Evac` edge to the demo project. The generic viewer keeps only the overlay hook.
- **D10 — Domain visualizations plug into a generic render-overlay seam, they are not viewer
  dependencies.** The generic viewer exposes a per-frame custom-draw callback carrying the camera /
  world→screen transform + the frame's render data; a consumer (the demo tool's evac overlay, a game's
  own HUD, a future 3D-IG layer) registers a draw pass without the viewer knowing the domain. This
  mirrors the engine's "external agents" seam: the package provides the extension point; evac is one
  client of it, living in the sample — so the NuGet stays generic and un-opinionated.
- **D6 — `SumoSharp.Testing` and `SumoSharp.Tools` are dev-time packages.** Never referenced by a
  shipping game build. `Testing` = the existing `Sim.Harness`. `Tools` is still design-only (§2 of
  `SUMOSHARP-API.md`) — implement last, or ship as a `dotnet sumosharp` global tool.
- **D7 — `SumoSharp` meta-package** groups the common bundle (Core + Ingest + Replication, and
  optionally Viewer.Motion) for one-line discoverability; it contains no code.

---

## 4. Framework targeting & native isolation

- **Portable tier (`net8.0;netstandard2.1`):** `Ingest`, `Core`, `Replication`, **`Viewer.Motion`
  (new — must stay ns2.1-clean so Unity/Godot can reconstruct motion)**. The ns2.1 discipline
  (span-based one-API surface, `System.Memory` on ns2.1 only, `NetstandardPolyfills.cs`) already
  established for Core/Ingest/Replication (`SUMOSHARP-API.md §3`) extends verbatim to `Viewer.Motion`.
- **Native / net8-only leaves:** `Replication.Dds` (CycloneDDS), `Viewer.Raylib` (raylib + ImGui).
  net8.0 single-target; they carry the native runtime assets. Because they are leaves, an integrator
  who avoids them never pulls a native binary.
- **Dev-time net8-only:** `Tools`, `Testing`, `Evac`.
- The offline parity gate builds only the portable + managed set (the test project references Core,
  Ingest, Replication, Evac, Harness, GameHostSample — never DDS or raylib), so `dotnet test`
  stays hermetic and native-free. **This must not regress.**

---

## 5. The load-bearing refactor: a portable `SumoSharp.Viewer.Motion`

**Problem.** The reusable render-side reconstruction — the thing a game/3D integrator most wants —
is today split across `Sim.Core` (`PoseResolver`, `ILaneShapeSource`) and `Sim.Viewer.Core`
(`DrClock`). `Sim.Viewer.Core` is **not packable** and, worse, `ProjectReference`s
`Sim.Replication.Dds` **and now `Sim.Evac`** (the demo-tool evac path), so it transitively drags the
**native DDS binary** *and* the evac domain layer — neither of which a motion consumer wants. Pulling
`DrClock` + the pipeline helpers into their own `Viewer.Motion` project is thus doubly motivated: it
escapes both unwanted dependencies. And `DrClock.Resolve`
takes a `DdsSubscriber.VehicleSample` / `DdsSubscriber.HistoryCap` — i.e. the clock is **coupled at
the type level to the DDS subscriber**, even though `SUMOSHARP-VIEWER-DR-SMOOTHING.md §8` already
argues (correctly) that `DrClock` is *conceptually* transport-agnostic.

**Target seam** (matches `SUMOSHARP-VIEWER-DR-SMOOTHING.md §8`'s "clean seam"). Note the sample +
history now live in the **`Replication` API** (D9), not in `Viewer.Motion`:

```
SumoSharp.Replication  (net8.0;netstandard2.1)  — the data-model API
  ├─ VehicleRecord, LaneGeo, TlEntry, UpcomingLanes, FrameHeader   (data model — existing)
  ├─ FrameCodec / GeometryCodec / TlCodec / FrameChunker           (codec — existing)
  ├─ IPublishPolicy / PublishScheduler / DrErrorPublishPolicy      (policy — existing; DR-error landed)
  ├─ DrExtrapolation.Arc(...)                                      (DR math — EXISTING; shared pub+view)
  ├─ IReplicationSink / IReplicationSource                         (NEW — the 4-channel contract, D8)
  ├─ TimestampedSample    (NEW — VehicleRecord + sim/arrival time, D9; same shape as sched. ref)
  └─ IVehicleSampleHistory (NEW — transport-neutral newest-last per-vehicle buffer, D9)

SumoSharp.Viewer.Motion  (net8.0;netstandard2.1, NO raylib, NO DDS)
  ├─ DrClock                 (moved from Sim.Viewer.Core; consumes Replication's sample + history;
  │                           already delegates arc extrapolation to Replication.DrExtrapolation)
  ├─ DrPipeline helpers      (as-built guide §10: capped position error-smoothing §10.2,
  │                           motion-derived heading tilt §10.3, stable manual delay §10.1,
  │                           heading low-pass — plain scalar maths; arc extrapolation is NOT
  │                           re-extracted — it lives in Replication)
  └─ (re-exports) PoseResolver, ILaneShapeSource, Pose, DrState   [these stay defined in Core]
      depends on → SumoSharp.Core (PoseResolver), SumoSharp.Replication (data model + DR math + sample)

SumoSharp.Replication.Dds  (net8.0, native)  — one binding of the contract
  └─ implements IReplicationSink/Source over DDS topics; adapts DDS samples → TimestampedSample
```

The real code changes are two, both parity-inert (viewer/transport-side only; `Sim.Core`'s
`PoseResolver` is untouched):
1. **Add the transport contract + neutral sample/history to `Replication`** (D8, D9) and refactor
   `Replication.Dds` to *implement* the contract rather than owning the channel model.
2. **Decouple `DrClock` from `DdsSubscriber`** so it consumes the `Replication` sample/history
   types; the DDS subscriber becomes an adapter that fills them.

Together these make "reimplement DR over a different transport/viewer" an implement-the-interface +
`PackageReference` job, not a copy-paste. After them, the DR/smoothing guide
(`SUMOSHARP-VIEWER-DR-SMOOTHING.md`) becomes the **README/primary doc** of `SumoSharp.Viewer.Motion`.

`SumoSharp.Viewer.Raylib` (the **generic** viewer, D5) then depends on `Viewer.Motion` +
`Replication.Dds` and holds the raylib/ImGui rendering + the DDS subscriber adapter + the render-overlay
seam (D10) — but **not** `DemoCatalog`, the evac path, or curated scenarios. The **demo tool** (a
sample exe) sits on top: it references the generic viewer + `Sim.Evac`, adds the `DemoCatalog` picker
and the evac overlay through the seam. So the `Sim.Viewer.Core → Sim.Evac` edge main just added is
relocated to the demo layer, keeping the packaged viewer generic.

---

## 6. Mapping every current project to its packaging fate

| Project | Kind | Fate |
|---|---|---|
| `Sim.Ingest` | lib (net8;ns2.1) | **`SumoSharp.Ingest`** (shipped) |
| `Sim.Core` | lib (net8;ns2.1) | **`SumoSharp.Core`** (shipped) |
| `Sim.Replication` | lib (net8;ns2.1) | **`SumoSharp.Replication`** (shipped) |
| `Sim.Replication.Dds` | lib (net8) ⚠native | **`SumoSharp.Replication.Dds`** (shipped) |
| `Sim.Viewer.Core` | lib (net8) → Core, Ingest, Replication, Replication.Dds, **Evac** | **split**: portable reconstruction → new `SumoSharp.Viewer.Motion`; demo-tool wiring (`DemoCatalog`, `DemoSession`, evac path) stays demo-side, not packaged |
| `Sim.Viewer` | exe ⚠native | now an **interactive demo tool** (`DemoCatalog` picker + live evac). Ships as a **demo/sample**; optional `SumoSharp.Viewer.Raylib` (D5, open) would package only reusable raylib primitives |
| `Sim.Harness` | lib (net8) | **`SumoSharp.Testing`** (opt-in dev-time package) |
| `Sim.Evac` | lib (net8) | **`SumoSharp.Evac`** (opt-in package) |
| `Sim.EvacProfile` | exe | sample (unchanged) |
| `Sim.Viz` | exe | sample (reusable `PayloadBuilder`/`SceneGen` could later become `SumoSharp.Viz`; not now) |
| `Sim.ExtDemo` | exe | sample (unchanged) |
| `Sim.Run`, `Sim.Bench`, `Sim.BenchCity` | exe | samples/benches (unchanged) |
| `Sim.LiveHost` | web app | sample (unchanged) |
| (none yet) | — | **`SumoSharp.Tools`** (design-only in `SUMOSHARP-API.md §2`; implement last) |
| (none — metadata only) | — | **`SumoSharp` meta-package** |

---

## 7. Parity & packaging guard (success bar)

- **Offline parity unchanged.** `dotnet test` must stay green and native-free after every packaging
  step (baseline at time of writing, post-main-rebase: **446 passed, 0 failed, 3 skipped**;
  determinism anchor per `Sim.Bench` unchanged).
- **Extend the packaging guard test.** `RungB13PackagingTargetsTests` already pins that Core/Ingest
  multi-target net8+ns2.1 and keep the polyfills. Add assertions that:
  - `Sim.Replication` and the new `Sim.Viewer.Motion` also `<TargetFrameworks>` net8+ns2.1 and are
    `IsPackable=true` with the expected `PackageId`;
  - `Viewer.Motion` has **no** `ProjectReference` to `Sim.Replication.Dds` (no native leak into the
    portable motion package) and no `Raylib`/`rlImgui` `PackageReference`;
  - `Replication.Dds` and `Viewer.Raylib` remain the *only* packable projects carrying native deps;
  - the transport contract (`IReplicationSink`/`IReplicationSource`) is defined in **`Sim.Replication`**
    (the API), not in `Sim.Replication.Dds` (D8) — a hermetic check that a source file under
    `src/Sim.Replication/` declares those interfaces.
- **Prove a second binding is possible.** A hermetic unit test implements the contract with an
  **in-memory transport** (sink → source in the same process, no native DDS) and round-trips a
  frame + geometry + a command, demonstrating structurally that DDS is one option among several
  (D8). Runs inside `dotnet test`, native-free.
  This is a hermetic, source-reading/in-memory test (no build of native libs, no SUMO, no network) —
  same style as B13.

---

## 8. Licensing & disclaimer (per package, unchanged policy)

- Dual license **`EPL-2.0 OR GPL-2.0-or-later`** (set in `Directory.Build.props`) — cannot be
  relicensed; SumoSharp is a derivative of SUMO. State the practical read (EPL-2.0 = weak, file-level
  copyleft; a proprietary game may link and keep its own source closed, but must keep SUMO-derived
  files under EPL and publish changes to *those* files) high in each README. Get counsel for
  commercial use — this is not legal advice.
- Each `PackageReadmeFile` carries the disclaimer: *"Unofficial, independent C# reimplementation of
  Eclipse SUMO's microscopic simulation core. Not affiliated with or endorsed by the Eclipse SUMO
  project."* "SUMO" is an Eclipse trademark.

---

## 9. Rollout order (see `SUMOSHARP-PACKAGING-TASKS.md` for tasks + success conditions)

1. **P0 — Reconcile docs with reality.** Update `SUMOSHARP-API.md §1` to point here; record the
   two-package (not bundled) reality and the retired `Runtime` idea. *(This doc + a pointer edit.)*
2. **P1 — Replication transport contract + neutral sample (D8, D9).** Add `IReplicationSink`/
   `IReplicationSource` + the timestamped sample + per-vehicle history to `Sim.Replication`; refactor
   `Replication.Dds` to *implement* the contract; add the in-memory-transport proof test. *(Prereq
   for the motion package — the sample it consumes lives here.)*
3. **P2 — `SumoSharp.Viewer.Motion`.** Decouple `DrClock` from `DdsSubscriber` onto the P1 sample/
   history; move the portable reconstruction into a new packable, ns2.1-clean project; wire
   `Viewer.Raylib`/exe to adapt onto it. Ship the DR/smoothing guide as its README. *(§5.)*
4. **P3 — `SumoSharp.Viewer.Raylib`.** Make the reusable raylib/ImGui component packable; leave the
   exe a thin sample.
5. **P4 — `SumoSharp.Testing` + `SumoSharp.Evac`.** Flip `Sim.Harness`/`Sim.Evac` to packable with
   IDs + READMEs.
6. **P5 — `SumoSharp` meta-package + `SumoSharp.Tools`.** Convenience bundle; then the SUMO-binary
   fetch tool (or global tool) per `SUMOSHARP-API.md §2`.
6. **Every step:** extend the B13-style guard, re-run the offline gate, publish CI packs the new IDs.
