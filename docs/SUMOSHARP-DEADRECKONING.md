# SUMOSHARP-DEADRECKONING.md — networked dead-reckoning & production-render design

**Status: design for review. Nothing here is implemented yet except the two foundational read-surface
inputs (§4.1, landed).** This is the document to read *before* deciding scope. It is a companion to
`SUMOSHARP-API.md` (§5 read surface, §7 async runner, §9 runtime demand) and coordinates with
`docs/LANELESS-DIRECTION.md` on the sibling branch `claude/sumo-phase-2-planning-p3w7kh` (see §9 below).

---

## 1. The problem

A host wants to render/replicate **10 000+ vehicles**, where:

- **Reading vehicle state must never block the next simulation step.** (Solved already: the async
  `SimulationRunner` publishes an immutable snapshot with one lock-free reference swap; the sim thread
  never waits on a reader — `SUMOSHARP-API.md` §7/§8.)
- **The renderer runs much faster than the sim** — 60–120 Hz render over a ~10 Hz sim — and is **often on
  a different machine** (networked). So per-frame world positions cannot be streamed; the renderer must
  **predict** (dead-reckon) between updates.
- **Network bandwidth is the budget.** Updates should be **rate-limited to ~10 Hz**, and for very
  predictable vehicles pushed **as low as ~1 Hz** — ideally adaptively, based on how predictable each
  vehicle currently is.
- **No strings on the API/wire** (owner constraint). Handles + a one-time id table only.
- **Realism > parity for production rendering.** SUMO parity is how we validate the *internal algorithms*
  when extending the port; it is **not** the bar for production visuals. The renderer may legitimately
  look *nicer than SUMO* (e.g. long vehicles not cutting corners), as long as the parity-validation path
  stays byte-exact and separate.

## 2. Why lane-relative prediction (the core idea)

The SUMO microsimulation is **1-D**: a vehicle is an arc-length `pos` on a known lane centerline, moving at
`speed` with a computed `accel`, plus a lateral offset `posLat`. The lane geometry is **static** and can be
sent to the renderer **once**. Therefore the ideal thing to transmit per update is the small **lane-relative
kinematic state**, not world x/y/z/heading:

- **Bandwidth:** ~20–30 bytes/vehicle instead of full world pose + strings every frame.
- **Curve-following prediction for free:** the renderer extrapolates `pos` forward and maps arc-length →
  (x, y, heading) by walking the *actual* lane polyline — so prediction follows real curves and never cuts
  corners, which naive world-space (x, y, heading) extrapolation cannot do.
- **It is exactly the sim's own model**, so the prediction is faithful, not an approximation on top of one.

The renderer reconstructs world pose with the **same `PositionAtOffset` math the engine uses**, packaged as
a portable pure function (§6) that can be mirrored in C#, JS, C++, etc.

## 3. Two motion regimes (and why prediction must switch model)

Not every mover is lane-predictable. Coordinated with the laneless branch's **two-regime** model
(`LANELESS-DIRECTION.md`):

| DR model | Who | Prediction | State transmitted |
|---|---|---|---|
| **`LaneArc`** | lane-bound SUMO vehicle (incl. lateral-dodging / sublane, `posLat ≠ 0`) | integrate `pos` along the upcoming lane path; interpolate `posLat` by `latSpeed` | `laneHandle, pos, posLat, speed, accel, latSpeed, upcomingLaneHandles[k]` |
| **`FreeKinematic`** | holonomic crowd / navmesh / ORCA agent (the laneless branch's `OrcaCrowd` / `WorldDisc`), or a vehicle mid-RVO-swerve whose lateral is not lane-predictable | integrate world position by velocity (± accel) | `x, y, (z), vx, vy, (radius)` |
| **`Stationary`** | parked / arrived / not-yet-departed | none | position only |

**STATUS: the shared `DrModel` enum has landed** (`src/Sim.Core/DrModel.cs`, `byte`-backed for DDS
`@bit_bound(8)`), as the frozen cross-branch seam — see §9 / `SUMOSHARP-API.md` §16.

A mover **switches DR model at runtime**: a lane vehicle is normally `LaneArc`; when it enters an
RVO/ORCA avoidance manoeuvre (`Engine.LanelessRvo` lateral solve, or the cross-regime bridge swerve) its
short-horizon lateral stops being lane-predictable, so it either (a) stays `LaneArc` but the publisher
raises its update rate (§7), or (b) is published as `FreeKinematic` for the duration. Pure crowd agents are
always `FreeKinematic`. The **DR-model tag travels in the packet** so the renderer picks the matching
extrapolator. (This is the single most important coordination point with the laneless branch — see §9.)

## 4. The prediction packet (handle-based, no strings)

### 4.1 Inputs already on the read surface (LANDED)

- `ReadOnlySpan<double> Engine.Acceleration` — per-vehicle longitudinal acceleration (the
  `getAcceleration()` analog). (`RungB19`.)
- `int Engine.GetUpcomingLanes(VehicleHandle, Span<int> dest)` — the lane-handle path ahead, current lane
  first. (`RungB19`.)
- Already present: `VehicleHandles`, `Pos`, `PosLat`, `Speed`, `LaneHandles` (all `double`/handle),
  render floats `PosX/Y/Z/Angle`. (`latSpeed` is the one missing `LaneArc` field — it lands at the
  laneless merge, which owns `Kinematics.LatSpeed`.)

### 4.2 The packet (proposed)

One published frame = a header + a dense array of per-vehicle records. All ids are **handles**; a
**one-time id table** (`handle → string id`, and the static lane geometry) is sent once on connect.

```
FrameHeader { uint simStep; float simTime; uint count; }
VehicleRecord {                       // ~24–28 bytes, quantized
  uint32  handle;                     // index + generation (the entity handle)
  uint8   drModel;                    // LaneArc | FreeKinematic | Stationary
  // --- LaneArc payload ---
  uint32  laneHandle;                 // dense lane handle (index into the once-sent geometry)
  float   pos;                        // arc-length on the lane
  int16   posLat_q;                   // lateral offset, quantized (cm)
  int16   speed_q;                    // m/s, quantized
  int16   accel_q;                    // m/s^2, quantized
  int16   latSpeed_q;                 // m/s, quantized
  uint16  upcoming[K];                // next K lane handles (K small, e.g. 2–4)
  // --- FreeKinematic payload (union; drModel selects) ---
  // float x,y,(z); int16 vx_q,vy_q; uint16 radius_q;
}
```

- **Bandwidth math:** 10 000 vehicles × ~26 B × 10 Hz ≈ **2.6 MB/s** raw; with delta-encoding
  (only-changed), **area-of-interest culling** (only vehicles near a client's camera), and the adaptive
  rate (§7) this drops by an order of magnitude for typical views. Quantization (cm / cm·s⁻¹) is well
  within render tolerance.
- **Physical params are sent once** (per vType or per handle): `length`, `width`, `type` — the renderer
  needs them for the vehicle shape and the corner-cut correction (§6.3), and they never change.
- The static **lane geometry** (polylines + `z`) is sent once, exactly as `Sim.LiveHost` already does.

### 4.3 Transport & DDS-friendly framing (owner target: CycloneDDS.NET)

The replication types must suit **DDS** (the owner's `CycloneDds.NET` — a code-first C# DSL: `[DdsTopic]`
`partial struct`, `[DdsKey]`, `[DdsId(n)]`, `[DdsStruct]` nesting, C#12 `[InlineArray]` fixed buffers,
`fixed byte[]` blobs, `byte`-backed enums → IDL `@bit_bound(8)`, keyed instances + `LookupInstance`),
while also working over plain **TCP/UDP**. Design principles:

- **One canonical packed wire format (a blob).** A single little-endian byte layout for a frame of records
  is the source of truth. It serves TCP/UDP directly, and rides DDS as an opaque `fixed byte[]` payload —
  so **one (de)serializer covers every transport**. (`byte`-backed `DrModel` packs into one byte.)
- **Topics split by update rate (DDS strength):**
  - **Geometry/registry — low-rate, durable/transient-local.** Lane polylines, the `handle→string-id`
    table, per-vType physical params (`length`/`width`/`type`). Published once / rarely; QoS
    `TRANSIENT_LOCAL` so late-joining readers get it without a resend.
  - **Vehicle lifecycle — low-rate, KEYED (`[DdsKey]` handle), transient-local.** Spawn (handle + vType +
    dims) / despawn. Keyed instances are exactly DDS's "track many objects" sweet spot and are *low-rate*,
    so the per-sample overhead is affordable here. This is what makes object tracking / late-join correct.
  - **Vehicle state — high-rate, BATCHED, best-effort keep-last.** The per-tick kinematic state (§4.2).
- **Batch, do not key, the high-rate state — this is the crux of your 10k concern.** One DDS sample **per
  vehicle** at 10 Hz = 10 000 samples/tick, and DDS per-sample overhead (headers, sequence numbers,
  instance bookkeeping) dwarfs a ~26 B record — untenable. Instead **one sample carries an array/blob of
  many records**, amortizing that overhead ~N×. Two concrete DDS shapes, both supported by `CycloneDds.NET`:
  1. **Chunked fixed blocks** — a `[DdsTopic]` struct = `{ FrameHeader; ushort count; InlineArray<Record,
     CHUNK> }` (e.g. `CHUNK=512`), so 10k vehicles = ~20 samples/tick instead of 10 000. Fixed size →
     zero-alloc marshalling (the binding's `[InlineArray]` path).
  2. **Opaque blob** — a `[DdsTopic]` struct = `{ FrameHeader; uint byteCount; fixed byte payload[MAX] }`
     carrying the packed wire format; chunk across samples if it exceeds the DDS max sample size. This is
     the same bytes TCP/UDP send.
  Recommendation: ship **(2) the blob** as the canonical high-rate path (one codec everywhere, minimal
  overhead) and offer **(1) the structured chunk** for consumers who want DDS-native typed access. Both are
  in the replication package; neither is in `Core`.
- **Rate-limit at the writer** (§7): the high-rate topic writes at ≤10 Hz globally, and the adaptive policy
  decides *which* vehicles are even included in each frame (so a predictable vehicle simply isn't re-sent).

### 4.4 Crowd / laneless entities (FreeKinematic) — first-class, same machinery

Human-crowd and laneless-traffic simulations are **large counts of unpredictable movers** — the
`FreeKinematic` regime. They use the **same transport design** with a tiny uniform record and their own
topic (independent QoS/rate from vehicles):

```
CrowdRecord { uint32 handle; float x, y, (z); int16 vx_q, vy_q; uint16 radius_q; }   // ~16–18 B
```

Kept a **separate topic/blob from the `LaneArc` vehicle state** (uniform record size per topic → clean
`InlineArray`/blob, independent rate) rather than a discriminated union in one frame. The crowd source is
the laneless branch's existing **`ICrowdFootprintSource` / `WorldDisc`** seam (§9) — no new abstraction.
Because these movers are unpredictable, the adaptive policy will keep most of them near full rate; the
saving comes from the tiny record + batching + area-of-interest culling, not from rate reduction.

## 5. Threading & non-blocking (already solved, restated)

The publisher reads from the async `SimulationRunner`'s immutable `SimulationSnapshot` (§7 of the API doc)
— the sim thread publishes it after each step with one reference swap and continues; the network/publish
thread serializes the latest snapshot without ever locking the engine. The **snapshot pool** (opt-in,
landed) keeps this allocation-free in steady state. The DR layer adds the packet columns (`accel`,
`drModel`, `latSpeed`, upcoming-path) to the snapshot; no new blocking is introduced.

## 6. The pose-resolver (portable, shared by local render, network client, and realism mode)

A single pure function turns lane-relative state + `dt` into a world pose. It is `netstandard2.1`-clean and
deliberately dependency-light so it can be **ported verbatim to JS/C++** for a non-.NET renderer.

```
PoseResolver.Resolve(
    laneGeometry,               // the once-sent static polylines (+ z)
    drModel, laneHandle, pos, posLat, speed, accel, latSpeed, upcomingLanes,
    dt,                         // render_time - packet_time
    vehicleLength, vehicleWidth,// physical params (sent once)
    realism)                    // enum: ParityTangent | ChordHeading | CornerCutCorrected
  -> Pose { double x, y, z; float headingDeg; }
```

### 6.1 Extrapolation
`pos' = pos + speed·dt + ½·accel·dt²` (clamped at a published stop-line if present); walk the upcoming lane
polylines, crossing into the next lane when `pos'` exceeds the current lane length; map arc-length → (x, y)
and interpolate `z`. `posLat' = posLat + latSpeed·dt`. `FreeKinematic`: `x' = x + vx·dt`, etc.

### 6.2 Heading (three fidelity levels)
- **`ParityTangent`** — the segment tangent at `pos'`. Matches today's read column exactly (what the
  parity harness compares).
- **`ChordHeading`** — reproduce SUMO's own `computeAngle` (`MSVehicle.cpp:1515`): place the front at `pos'`
  and the back at `pos' − length` along the geometry, heading = the back→front **chord**. This is what
  SUMO actually reports; for long vehicles on curves it differs from the tangent. Client-side, from data
  already present (no extra transmission).
- **`CornerCutCorrected`** — a **renderer-only** realism pass, in **two possible tiers** (this is the
  "scope" question):
  - **Tier A — chord heading only (cheap, recommended first).** Just the `ChordHeading` above: the
    rendered rectangle is *oriented* along the back→front chord. Cost: one extra geometry lookup (back
    point at `pos−length`) + an `atan2`. The vehicle's **reference point stays on the lane centerline** —
    so a long vehicle points correctly through the curve, but its body still rides the centerline (the
    rear can still visually clip the inner edge a little). This already removes most of the "wrong-looking"
    heading on turns.
  - **Tier B — full swept-path off-tracking ("trucks swing wide").** Model that a real long/articulated
    vehicle's **rear axle tracks *inside* the front's path**, so the driver swings the front *outward*
    entering the turn to keep the rear clear. This **moves the rendered pose off the centerline** (front
    bows toward the outside, rear cuts across the inside) using a kinematic bicycle/trailer model from the
    wheelbase/`length`. Cost: a small per-vehicle path integration (or a curvature→offset lookup) over the
    upcoming/preceding lane geometry — more math, and a bigger visual change (bodies may overlap a
    neighbour's shape, which you accepted). This is what makes articulated turning look *right*.
  Both tiers are **renderer-only, derived purely from physical params + lane geometry, and cost zero extra
  network data**. Tier B is a superset of Tier A.

  **STATUS: both tiers landed** in `PoseResolver` (`RenderRealism.ChordHeading` / `CornerCutCorrected`,
  `RungB20`). Tier B approximates the swept-wide distance from the heading change over the vehicle body
  (`≈ length·|Δψ|/2`) and bows the rendered front toward the outside of the turn (clamped; vanishes on a
  straight → collapses to ChordHeading). It is a deliberate **visual approximation** (not a full
  bicycle/trailer integration), documented as such in the code.

### 6.3 Why realism can live in the API layer (the owner's point)
The correction is a **deterministic pure function** of state the renderer already has (lane-relative pose +
upcoming/previous lane geometry + physical dims). So:
1. **It simplifies the network data**, not complicates it: the correction is computed *client-side*, so the
   wire packet stays the minimal lane-relative state — realism costs zero extra bytes.
2. **It can be offered directly on the engine's own read surface** as an **opt-in production mode** (a
   `RenderRealism` setting on the `Step()` projection): when on, the published `PosX/PosY/Angle` render
   floats carry the chord/corner-cut-corrected pose for hosts that render locally and want it to *look
   right* rather than match SUMO. **Crucially, this only touches the derived render floats** — the
   parity-exact lane-relative doubles (`Pos`, `PosLat`, `LaneHandle`) are the sim truth and are never
   altered, and the whole thing is off the `Run()`/golden path. So **parity validation stays byte-exact
   and separate**, exactly as it should: parity is for checking the algorithms, realism is for production.

   **STATUS: landed** — `Engine.RenderMode` (default `ParityTangent`). When set to `ChordHeading` /
   `CornerCutCorrected`, `PublishReadState` overrides only the render floats via `PoseResolver` (dt=0),
   building each vehicle's upcoming+preceding lane path from its lane sequence. Default is byte-identical
   (the override branch is skipped). `RungB21` proves the parity-exact columns are invariant to the mode
   and the default `Angle` equals the lane tangent; the determinism hash is unchanged.

## 7. Adaptive publish rate

Per-vehicle, the publisher decides how often to send an update based on a **predictability / error
budget**: cheaply estimate how far the renderer's dead-reckoned pose would drift from the true pose over
the next N frames, and only send when that predicted error exceeds a threshold (or a max interval elapses).

- **Highly predictable** (steady lane-following, `|accel|` small, no near obstacle, not in RVO) → down to
  **~1 Hz**.
- **Low predictability** (braking/accelerating hard, near an obstacle, in an RVO/ORCA manoeuvre, changing
  lanes) → **full 10 Hz**.
- Signals available cheaply per step: `|accel|`, obstacle/leader proximity, `LanelessRvo`-active,
  lane-change-active, DR-model just switched.

The renderer keeps extrapolating with the last packet until a new one arrives; when a correcting packet
arrives it **reconciles** — snap, or blend over a few frames (standard networked-entity smoothing). Because
the packet describes *future* motion (not just a position), a 1 Hz update still renders smoothly while the
prediction holds. Load-dependent global scaling (drop everyone's cap under network pressure) sits on top.

**The policy is a pluggable delegate, not a fixed threshold** (owner decision). The scheduler calls an
`IPublishPolicy` (or a `Func`) per vehicle per candidate frame with the cheap signals below and gets back a
decision (send now / defer / target interval). A sensible default policy ships, but a host can supply its
own (e.g. weight by camera distance, by scenario importance, by a bandwidth governor):

```csharp
public interface IPublishPolicy {
    // Called with frozen per-vehicle signals; returns whether to include this vehicle in the current frame.
    bool ShouldPublish(in PublishSignals s);   // s: handle, drModel, |accel|, speed, leaderGap,
}                                               //    rvoActive, laneChanging, secondsSinceLastSent, ...
```

## 8. Extrapolation vs interpolation (one packet, two modes)

- **Extrapolation** (zero added latency): render at `now`; predict forward from the last packet. Can
  overshoot on a hard stop the predictor didn't know about → mitigated by transmitting `accel` and an
  optional stop-line, and by reconciliation on the next packet.
- **Interpolation** (≈1 packet of latency, exact): render slightly in the past, between the last two
  packets — no overshoot. (This is what `Sim.LiveHost` already does client-side; the arc-length resolver
  makes it curve-correct.)

Same packet drives both; the choice is a client policy.

## 9. Coordination with the laneless / RVO branch (`claude/sumo-phase-2-planning-p3w7kh`)

**This is the piece to align on now.** The laneless branch already owns the second regime and the neutral
world-space seams the DR layer needs. Concretely, from `LANELESS-DIRECTION.md` / `LANELESS-HANDOFF.md`:

- **`WorldDisc` + `ICrowdFootprintSource`** (`src/Sim.Core/Bridge/`) is the neutral world-space footprint
  seam (pos + velocity + footprint). **This IS the `FreeKinematic` DR primitive.** The DR publisher should
  source crowd/ORCA agents through `ICrowdFootprintSource` and emit them as `FreeKinematic` records — no
  new abstraction needed.
- **`OrcaCrowd`** (`src/Sim.Core/Orca/`) is the holonomic agent store; its agents are always
  `FreeKinematic`.
- **`LaneProjection`** (`Sim.Ingest`, the inverse of `PositionAtOffset`) is already the world↔lane bridge;
  the DR pose-resolver is the *forward* direction and should share the same lane-geometry representation.
- **`Kinematics.LatSpeed`** (laneless-owned) is the missing `LaneArc` field for predicting `posLat` drift —
  the DR `latSpeed` packet field should read it at merge.
- **`avoidanceKind` / `Responsibility`** (reserved `{StaticBlocker, OneSided, Reciprocal}`) already
  distinguishes cooperative agents; the DR-model tag is orthogonal but should not duplicate it.

**Proposed shared contract (the ask to the laneless session):**
1. **A shared `DrModel` enum** (`LaneArc | FreeKinematic | Stationary`) on a neutral seam both layers
   reference — mirroring how `RvoNeighbor` is the frozen seam for the obstacle store. The lane engine tags
   its vehicles; `OrcaCrowd`/the bridge tag crowd agents `FreeKinematic`.
2. **A predicate for "is this lane vehicle currently lane-predictable?"** — i.e. is it mid-RVO/ORCA
   swerve or lane-change this step? The laneless layer knows this (it runs `ComputeRvoLateral`); expose a
   cheap per-vehicle flag the DR publisher reads to choose `LaneArc`-at-high-rate vs `FreeKinematic`.
3. **Merge order:** DR is additive/gated and reads through the neutral seams, so (like the obstacle store)
   it does not block the laneless work and vice-versa. The DR snapshot columns (`accel`, `drModel`,
   `latSpeed`, upcoming-path) are additive to `SimulationSnapshot`; the laneless `LatSpeed`/lateral columns
   they already added are the source. Whoever merges second reconciles additive hunks. Shared acceptance
   gate unchanged: determinism hash `909605E965BFFE59`, goldens byte-identical.

**Nothing here asks the laneless branch to change its solver** — only to (a) agree the `DrModel` enum lives
on a shared seam, and (b) expose a "lane-predictable-right-now" flag. Both are cheap and additive.

## 10. Package layout & proposed API surface

**Network/replication types go in their own package(s)** (owner decision — not everyone needs them):

| Package | Contents | Depends on |
|---|---|---|
| `SumoSharp.Core` | the engine; `DrModel` enum (the shared seam); `PoseResolver` (portable pose math — used by local render too); the opt-in `RenderRealism` read mode | — |
| **`SumoSharp.Replication`** *(new)* | transport-agnostic record structs + the **canonical packed blob (de)serializer**; the `IPublishPolicy` adaptive scheduler; TCP/UDP helpers | `Core` only |
| **`SumoSharp.Replication.Dds`** *(new, optional)* | the `[DdsTopic]` types (geometry/registry, keyed lifecycle, batched state + crowd) wrapping the blob/records | `Replication` + `CycloneDds.NET` |

So a host that renders locally needs only `Core`; a host replicating over TCP/UDP adds `Replication`; a
DDS host adds `Replication.Dds`. `CycloneDds.NET` is a dependency **only** of the optional DDS package.
(`PoseResolver` and `DrModel` live in `Core` because they are simulation-domain / local-render concerns,
not networking — the replication packages build the wire types *around* them.)

```csharp
// Core — opt-in production render mode (parity path untouched, off by default):
public enum RenderRealism { ParityTangent, ChordHeading, CornerCutCorrected }  // Tier A/B per §6.2
public RenderRealism RenderMode { get; set; } = RenderRealism.ParityTangent;

// Core — portable resolver (ns2.1-clean, mirrorable in JS/C++):
public static class PoseResolver { public static Pose Resolve(/* §6 signature */); }

// Core — the shared cross-branch seam (LANDED, src/Sim.Core/DrModel.cs):
public enum DrModel : byte { LaneArc, FreeKinematic, Stationary }

// Replication — transport-agnostic records + one blob codec for DDS-blob / TCP / UDP:
public readonly struct VehicleRecord  { /* §4.2, handle-based */ }
public readonly struct CrowdRecord    { /* §4.4 */ }
public static class FrameCodec { /* Write(Span<byte>, records...) / Read(ReadOnlySpan<byte>) */ }
public interface IPublishPolicy { bool ShouldPublish(in PublishSignals s); }  // §7 pluggable

// Replication.Dds — [DdsTopic] wrappers (geometry/registry, keyed lifecycle, batched state + crowd blob).
```

**STATUS: `SumoSharp.Replication` landed** (transport-agnostic). `Records.cs` (`VehicleRecord`,
`CrowdRecord`, `UpcomingLanes`), `FrameCodec` (the canonical packed little-endian blob — header + records,
allocation-free, ns2.1-clean, round-trips both frame kinds), and `PublishPolicy` (`IPublishPolicy` +
`DefaultPublishPolicy` + `PublishSignals`). Depends only on `Core`; in `Traffic.sln` (no external deps →
hermetic gate safe). `RungB22` round-trips both frames, checks the policy, and proves an end-to-end
encode→decode→`PoseResolver` reconstructs the engine's own pose.

**STATUS: `SumoSharp.Replication.Dds` landed** (CycloneDDS.NET 0.3.2 transport). `DdsTopics.cs`:
`DdsWireFrame` — the high-rate **opaque blob** topic (`[DdsTopic] unsafe partial struct` with a keyed
`(Kind, ChunkIndex)` + `fixed byte Payload[64 KiB]`, `SetPayload`/`ReadPayload` bridging `FrameCodec`) —
and `DdsVehicleLifecycle` — the low-rate **keyed** spawn/despawn+dims registry. **Deliberately NOT in
`Traffic.sln`** (external `CycloneDDS.NET` + native DDS dep), so the hermetic gate never needs it; built
explicitly (`dotnet build src/Sim.Replication.Dds`), **compile-verified here** (the CycloneDDS native
runtime is win-x64, so it can't *run* on this Linux VM — publish/read usage is in the package README).
net8.0 only. **Still to build:** the `Sim.LiveHost` showcase wiring the packet + client-side resolver.

## 11. Phased implementation plan (each additive, gated, hash-stable)

1. **Pose-resolver + `ChordHeading`** (`PoseResolver`, pure function; a test proving chord ≠ tangent for a
   long vehicle on a curve, and resolver-at-`dt=0` == the engine's own `PositionAtOffset`). *No forks; safe
   to start immediately.*
2. **`RenderRealism` opt-in on the read surface** (`ChordHeading`, then `CornerCutCorrected`) — derived
   render floats only; parity columns + `Run()` untouched; hash unchanged.
3. **`latSpeed` + `accel` + `drModel` + upcoming-path columns on `SimulationSnapshot`** (needs the
   laneless `LatSpeed`; `drModel` needs the §9 shared enum).
4. **The prediction packet + a compact (de)serializer**, handle-based, with the one-time id/geometry table.
   Home decision: `Core` vs new `SumoSharp.Runtime`.
5. **Adaptive publish scheduler** (§7) + reconciliation helpers.
6. **Wire it into `Sim.LiveHost`** as the reference networked renderer (replace the per-frame world-pose
   JSON with the packet + client-side resolver) — the end-to-end showcase.

## 12. Decisions (owner) & remaining open items

**Decided:**
- **Packet home:** separate packages — `SumoSharp.Replication` (transport-agnostic blob+records) +
  optional `SumoSharp.Replication.Dds` (CycloneDDS.NET topics). Not in `Core`. (§10)
- **Adaptive-rate policy:** a **pluggable `IPublishPolicy` delegate** (a default ships; hosts can replace).
  (§7)
- **`drModel` shared enum:** coordinate **now** — landed in `Core` as the frozen seam; coordination issue
  filed to the laneless branch (DR1/DR2). (§3, §9)
- **DDS-friendly + blob:** two topics by rate (durable geometry/lifecycle vs high-rate batched state); the
  high-rate path is a **batched blob** (not per-vehicle keyed samples) to amortize DDS overhead for 10k+;
  crowd/laneless is first-class via its own tiny-record topic. (§4.3, §4.4)

**Decided (round 2):**
- **`CornerCutCorrected`:** ship **Tier A (chord heading)** *and* **Tier B (full swept-path off-tracking,
  "trucks swing wide") now**. Both renderer-only, zero extra wire data. (§6.2)
- **Sim-side chord heading:** **no** — the parity FCD `Angle` column stays tangent (zero parity risk); the
  chord/off-tracking correction is render-only.
- **DDS high-rate shape:** ship the **opaque blob** as canonical **and** the structured `InlineArray` chunk
  as an option. (§4.3)

All owner decisions are now made; implementation can proceed. Remaining coordination is DR1/DR2 with the
laneless branch (issue #3), which is merge-order-independent and does not block the NuGet-side build.
```
