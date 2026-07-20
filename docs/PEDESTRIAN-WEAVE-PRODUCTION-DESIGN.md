# Pedestrian weave — production-seam design (PED-REALISM-1 / "(b)")

**Status:** design, pre-implementation. **HOW** for graduating the validated `LateralWeave` from the Sim.Viz
prototypes (A–D2) into the real low-power motion path. The **WHAT** (behaviour, determinism/parity argument,
the density ceiling, the restore mechanism) is `docs/PEDESTRIAN-LOWPOWER-AVOIDANCE-DESIGN.md` (esp. §7, §8,
§10.2, §10.2-bis, §11); this doc does not restate it. Read that first.

This closes the §8 "production requirements the prototype does NOT cover": whole-route continuity, endpoint
anchoring to the actual O/D, smooth junction hand-off, and a per-edge shared width — plus the demote-restore
wiring (§10.2). It deliberately does **not** do the per-ped-demote-timing optimization ("(a)", §10.2-bis wire
cost) — that is a later, non-blocking tuning knob; nothing here is made harder by deferring it.

---

## 1. The load-bearing insight: inject at the SHARED evaluator

Both sides already compute a low-power pose through the **same** pure code:

- **Server:** `PedLodManager.PositionOf(id, now)` → `PathArcMotion.PositionAt(...)` / `ActivityTimeline.PoseAt(...)`.
- **IG / remote:** `PedRemoteReconstructor.TryGetRenderPose` → `HeadlessIg.ReconstructSample(id, t)` (Sim.Replication)
  → the decoded `ActivityTimeline.PoseAt` / `PathArcMotion.PositionAt`.

So if the weave offset is applied **inside `PathArcMotion.Walk` / `ActivityTimeline.Evaluate`** (the shared
evaluator), *both* sides get the identical offset from the identical inputs, and **server==IG holds by
construction** — no new reconstruction logic, no per-ped weave bytes on the wire. The wire's only job is to
carry the *inputs* the evaluator needs (per-ped `seed`, per-leg `halfWidth`, and at a demote seam the `l_r`
scalar); everything else is recomputed. This is the whole reason the prototype used pure functions.

**Consequence for the plan:** the change is "thread three inputs to a function that already exists", not
"teach the reconstructor to weave". The reconstructor (`PedRemoteReconstructor.cs`) and its `Smooth` chase are
**untouched**; because `OffsetWithResume` returns exactly `l_r` at the demote seam, the smoother never sees a
pop to absorb.

### Seam map (from the code, for reference)
| file | seam | role |
|---|---|---|
| `Lod/PathArcMotion.cs:61` `Walk(path, s, out direction)` | **server injection** | already computes arc-length `s` + unit tangent; add `point + direction.PerpCW * off` |
| `Lod/PathArcMotion.cs:17` `PositionAt(path, startTime, speed, now)` | evaluator entry | needs a weave-aware overload carrying `(seed, halfWidth, routeLength, resume?)` |
| `Lod/ActivityTimeline.cs:57` `WalkSegment(Path, Speed)` | data model | gains per-leg `HalfWidth` (+ optional resume `l_r`,`leadIn`); the timeline gains a per-ped `Seed` |
| `Lod/ActivityTimeline.cs:261` `Evaluate` Walk case | evaluator | the `PositionAt` call that layers the offset |
| `Lod/ActivityTimelineWire.cs:49/109` `Encode`/`Decode` | wire | +`ulong seed` (per timeline), +`double halfWidth` (per Walk), +resume fields on a demote leg |
| `Lod/PedLodManager.cs:389-406` demote block | **`l_r` re-injection** | project frozen high-power pose onto the new lane centreline → `l_r`; publish it |
| `Lod/PedLodManager.cs:157/188` `AddPed`/`AddPedLively` | seed + width source | assign per-ped seed; thread lane half-width onto the leg |
| `PedNetwork.cs:21` `PedLane(... Width ...)` | width origin | `Width/2`, with the established `0.5 m` fallback (`WalkablePolygonBaker.cs:63`) |
| `Sim.Replication` `HeadlessIg.ReconstructSample` | IG mirror | **no logic change** — decodes the new fields via the wire, calls the same `PoseAt` |

---

## 2. Data-model changes

1. **Per-ped `Seed` (`ulong`) — derived from the SHARED scenario root, not a new one.** SumoSharp already has a
   single scenario-global seed, `Engine.Seed` (default 42; it maps to SUMO's `<random_number><seed>` via
   `ScenarioConfig.Seed`), and an established per-entity discipline: `VehicleRng.SeedFor(globalSeed, entityIndex,
   salt)` (SplitMix64, thread-order-independent, no `System.Random`) — the exact mechanism that makes **cars
   deterministic**. The weave reuses it: `seed = VehicleRng.SeedFor(Engine.Seed, pedIndex, WeaveSalt).RawState`
   with a distinct `WeaveSalt` so the weave stream never aliases a car/dawdle stream. Consequences we WANT:
   (a) the whole run (cars + peds + weave) reproduces from one number; (b) server==IG — both sides derive the
   identical per-ped seed; (c) forward-compat — future ped↔car deterministic interactions (crossing gap
   acceptance, jaywalk decisions) slot into the same rooted, salted discipline. The seed lives on
   `ActivityTimeline` (per-ped) and the plain-PathArc `PedEntry`, assigned once at `AddPed*`.

2. **Per-vertex `HalfWidth` (`double[]`, one per route vertex) — piecewise from day 1.** On `WalkSegment` (and
   the PathArc leg), parallel to `Path`. `halfWidth(s)` is evaluated piecewise as the leg is walked (the
   evaluator already resolves `s`→segment, so `width[segment]` is free). We take piecewise (not a per-leg min)
   from the start because the real cost is associating route geometry with lane widths (§4) — paid either way —
   and once each vertex carries `Width/2`, keeping the array vs collapsing to a scalar is trivial and it is the
   eventual format. **Deferred (W-polish, cosmetic only):** blending the band width across an edge join so it
   doesn't visibly step; piecewise-constant is correct meanwhile, and the clamp handles any jump safely.

3. **Resume fields on a demote leg only:** `ResumeLateral` (`l_r`, `double`) + `LeadIn` (`double`). Absent /
   sentinel on a normally-spawned leg (which uses `Offset`); present on a leg emitted by a demote (which uses
   `OffsetWithResumeOnRoute`). One flag or a `NaN` sentinel distinguishes the two.

No change to `PauseSegment`/`DwellSegment`/`InteractSegment` — the weave is a Walk-only concern.

---

## 3. The evaluator injection (server == IG)

In the shared Walk evaluation, at arc-length `s` along a leg of length `L` with tangent `t̂`:

```
centre = Walk(path, s)                       // existing polyline point
n̂      = t̂.PerpCW                            // right-normal (already available in Walk)
hw     = halfWidth(s)                         // piecewise per-vertex (§4)
c      = LateralWeave.CenterShift(s, now, L, Engine.Seed, maxShift, P)   // shared moving interface
room   = hw - dirSign * c                     // direction-aware: the moving centre squeezes one side, widens the other
off    = resumeLeg
         ? LateralWeave.OffsetWithResumeOnRoute(sAbs, sSinceDemote, L, seed, room, l_r, leadIn, P)
         : LateralWeave.Offset(s, L, seed, room, P)
pose   = centre + n̂ * (dirSign * c + off)     // offset measured from the shifted interface, on the ped's own side
```

- **`CenterShift` (folded into the core, not deferred).** The shared counterflow interface `c(s, now)` is a pure
  function of the SAME `Engine.Seed` we already introduced for the per-ped seed, so its only dependency is now
  free. It shifts the dividing line between the two travel directions; `dirSign` (+1 / −1 from the ped's travel
  direction, already known from `t̂`) makes the squeeze direction-aware — the stream the interface drifts toward
  loses room, the other gains it. This is the "the crowd's centreline breathes" behaviour; without it opposing
  flows still separate (keep-right sign) but the divide is static per segment. `maxShift` is a fraction of `hw`.

- `s` and `t̂` already exist inside `Walk` (`PathArcMotion.cs:90-91`); only `seed`, `halfWidth`, `L`, and the
  multiply are added.
- **Heading:** keep the *tangent* `t̂` as the render heading (the lateral drift is small and slow; using the
  raw tangent avoids a heading wobble and keeps parity with today). Optionally, a later polish derives heading
  from the offset's ds-derivative; not in scope.
- **Whole-route continuity (§8-1):** emit the **entire remaining route as one Walk leg**; `Offset(s, L)` is
  continuous in `s` across edge boundaries and junction curves (prototype B proved the bend), so no seam at
  edges. Width variation is handled by `halfWidth(s)` (W5), not by splitting the leg.
- **Endpoint anchoring (§8-2):** `Offset`'s endpoint taper is a function of the leg's own `(s, L)`. Because the
  leg IS the whole route from true spawn to true arrival, the taper lands the ped exactly on the real O/D — not
  on every internal edge end. A demote leg uses the **arrival-only** taper (`OffsetWithResumeOnRoute`), so it
  converges to the destination but does **not** re-taper at the demote seam.
- **Junction hand-off (§8-3):** at a walkingArea the centreline bends; `t̂` (hence `n̂`) rotates smoothly with
  it, so the offset rides the curve. No special-casing.

**server==IG test:** encode a weave leg, decode it, and assert `HeadlessIg.ReconstructSample == server PositionOf`
bit-for-bit across the leg (the wire uses full doubles, no quantization — `ActivityTimelineWire` already
guarantees this for the existing fields).

---

## 4. Width threading (PedLane → leg)

Width is first-class on `PedLane.Width` but is dropped before the LOD layer — `IPedNavigation.FindPath` returns
a bare `IReadOnlyList<Vec2>`. The real work is **associating each route vertex with the lane it lies on** and
stamping that lane's `Width/2`; that association is paid whether the result is piecewise or a collapsed min, so
we keep it **piecewise from day 1** (ad-2 decision):

- `FindPath` (or a thin wrapper) returns the centreline polyline **plus a parallel `double[] halfWidth`** (one
  per vertex) — a small `RouteWithWidth` record rather than a bare polyline. The width per vertex is the
  traversed lane's `Width/2`.
- The evaluator resolves `s`→segment already; `halfWidth(s)` is then `halfWidth[segment]` (or a lerp between the
  segment's endpoint widths — decide in W2; piecewise-constant is fine to start).
- Fallback `0.5 m` half-width when `Width` is unset — matching `WalkablePolygonBaker.cs:63` exactly, so the
  motion band and the baked walkable strip agree.
- **Deferred (cosmetic):** smoothing the width transition across an edge join. Not correctness- or format-
  affecting; a piecewise-constant step is safe (the clamp absorbs it).

---

## 5. The demote re-anchor (`l_r`) — production specifics

`PedLodManager` demote (`cs:389-406`) already **re-routes from the frozen high-power position** via
`FindPath(frozenPos, dest)` and emits a fresh PathArc leg — structurally exactly the crosser/bystander model.
Two production specifics the prototype glossed:

1. **`l_r` = perpendicular offset of the frozen pose from the NEW leg's centreline**, not a corridor constant.
   `FindPath` snaps the route to lane centrelines; the frozen ORCA pose is generally off-centre. Compute
   `l_r = signdist(frozenPos, centrelineAt(s≈0))` using the leg's start tangent's right-normal. Feeding this to
   `OffsetWithResumeOnRoute(interiorArc, blendDist, ...)` makes `pose(blendDist=0) == frozenPos` exactly
   (no pop), then blends to the lane weave over `LeadIn`.
   - **Guard:** if `|l_r| > halfWidth` (ORCA left the ped well off the sidewalk), clamp the *blend target* but
     still start at the true `l_r` — continuity beats staying in-band for one lead-in.
2. **Absolute-arc vs restart:** a demote that continues the ped's own route uses `OffsetWithResumeOnRoute` with
   `interiorArc = s_r + blendDist` (bystander form, §10.2-bis) so it rejoins its own seeded track; a demote
   onto a genuinely new route (the crosser reached a new POI/edge) uses `OffsetWithResume` (interiorArc==blend).
   The LOD manager knows which (did the destination/edge change?), so it picks the call.

`LeadIn` default: the design's `~8 m` (prototype value); a tunable on the demote.

---

## 6. Wire delta + determinism

`ActivityTimelineWire` is a fixed positional layout of full LE doubles (bit-exact by design). Additions:

- `+ ulong Seed` once per timeline (after `T0`).
- `+ double HalfWidth` **per route vertex** in a Walk segment (alongside each `X,Y` — piecewise, §2/§4).
- `+ (byte hasResume, double ResumeLateral, double LeadIn)` per Walk — only meaningful on a demote leg.
- The scenario-global `Engine.Seed` that `CenterShift` reads is **broadcast once** on the replication header
  (not per ped) — it is already a single scenario constant, so this is one `ulong` for the whole stream.

This is a **breaking format bump** (no version tag today): `Encode`/`Decode` and `ActivityTimelineWireTests`
change together. The plain-PathArc publish path (`PublishPathArc`, `PedLodManager.cs:176/404`) carries the same
inputs via its own record — mirror the addition there. Determinism unchanged: all inputs are exact
doubles/ulongs; the offset is a pure SplitMix64 function; no `System.Random`; independent of thread order.
Because the per-ped seed and the global seed both derive from `Engine.Seed`, the IG reconstructs every offset
(including `CenterShift`) with zero extra per-ped bytes beyond `l_r` at a demote seam.

---

## 7. Parity & safety (the iron law)

This is **additive to the pedestrian LOD path only**. It does not touch the car-following / lane-change /
junction parity core, so no committed `tolerance.json` can move. The offline gate (`dotnet test`, 649 parity +3
skip) must stay green throughout; the new work adds *ped* unit tests, not parity goldens. The existing ped
LOD/reconstruction tests (`ActivityTimelineWireTests`, promote/demote, P3-3 reconstruction) are the regression
guard — each stage keeps them green. SUMO is not involved (pedestrian weave is a SumoSharp believability layer,
not a SUMO-parity behaviour).

---

## 8. Staged tasks (with success conditions) + tracker

Each task names its design section, the files, and a numeric/observable done-condition.

**W1 — evaluator injection + data model (seed from `Engine.Seed`, `CenterShift`).** §2(1), §3.
Files: `PathArcMotion.cs`, `ActivityTimeline.cs`, `VehicleRng` reuse for the seed, `ActivityTimelineWire.cs`
(seed + global-seed header).
Done: a low-power ped's `PoseAt(now)` equals `centre + n̂·(dirSign·CenterShift + Offset(s,L,seed,room))` (new
unit test); per-ped seed derives from `VehicleRng.SeedFor(Engine.Seed, pedIndex, WeaveSalt)`; `Seed` +
global-seed round-trip through Encode/Decode; **decode→PoseAt == server PoseAt bit-for-bit** over a leg; all
existing ped tests green.

**W2 — piecewise width threaded from `PedLane` (day 1).** §2(2), §4.
Files: navigation route seam (`RouteWithWidth`), `PedLodManager.AddPed*`, `ActivityTimelineWire` (per-vertex
width).
Done: a leg over lanes of known `Width` carries per-vertex `Width/2` (and `0.5` when unset); `halfWidth(s)`
evaluated piecewise; the offset clamp never exceeds the baked walkable half-width at any `s` (assert against
`WalkablePolygonBaker` half along the route).

**W3 — reconstructor server==IG on the weave path.** §1, §3, §6.
Files: `HeadlessIg` (Sim.Replication) decode wiring; `PedRemoteReconstructor` (verify untouched).
Done: a replicated weave ped's `ReconstructSample` matches the server pose to `0` (exact) across a whole leg;
the P3-3 reconstruction demo/test shows the weave with zero DR error on low-power spans.

**W4 — demote re-anchor (`l_r`) with no pop.** §5, §10.2/§10.2-bis of the avoidance doc.
Files: `PedLodManager` demote block, `ActivityTimelineWire` resume fields, evaluator resume branch.
Done: promote→demote of a low-power ped resumes with seam ‖Δ‖ < 1e-9 m; server==IG exact after demote; a
bystander shoved by a promoted neighbour rejoins its own track (extend the promote/demote test with the D2
assertions ported off the Sim.Viz prototype).

**W-polish — cosmetic, deferred, non-blocking.** §2(2), §4.
Edge-join width smoothing; optional heading-from-offset derivative; the per-ped-demote-timing optimization
("(a)", §10.2-bis). Gated on measured need, not required for the core to ship.

### Tracker
- [x] **W1** evaluator injection + seed + `CenterShift` + server==IG bit-exact unit test *(DONE: `PathArcMotion.SampleAt`, `ActivityTimeline` weave in `Evaluate`, `WalkSegment.HalfWidths`, `Seed`/`GlobalSeed` + wire; `WeaveEvaluatorTests` 5/5 — weave-off byte-identical, pose==centre+n̂·(c+off), server==IG bit-for-bit, endpoint taper; full gate 649+3, ped 200/200)*
- [x] **W2** piecewise per-vertex width threaded from `PedLane` (0.5 fallback); clamp ≤ baked half everywhere *(DONE: W2a `BakedPolygon.HalfWidth` + `SumoNavMesh.HalfWidthsAlong` [Sonnet, reviewed]; `IPedNavigation.HalfWidthsAlong` default method; `PedDemandConfig.EnableWeave` (default off) wires per-vertex widths + `WeaveSalt` seed + `GlobalSeed` into `BuildLivelyTimeline`. `PedDemandWeaveTests`: weaves >0.2 m yet stays ≤ baked half on the real POC-0 route, deterministic. Gate 649+3, ped 206/206)*
- [x] **W3** `HeadlessIg` decodes new fields; reconstruction exact on the weave path *(DONE, verification-only — no production change: a weaving lively ped run through the real `PedReplicationPublisher` → byte-loopback bus → `PedReplicationReceiver` → `HeadlessIg.ReconstructSample` matches the server pose bit-for-bit across the leg, with the weave confirmed active (max lateral > 0.2 m). `PedReplicationRoundTripTests.ServerEqualsIg_OverTheWire_ForWeavingActivityTimeline`; ped suite 207/207)*
- [x] **W4** demote resumes the weave, no pop, server==IG exact after *(DONE: `PedLodManager` demote emits a weaving single-Walk `ActivityTimeline` resume leg — exact `ActivityTimelineWire`, not the quantized PathArc record — gated on the ped's stored `WeaveSeed`. `ReanchorAt` forces the leg to start exactly at the frozen pose so the Offset start-taper gives machine-precision no-pop; NO `l_r` needed in production because FindPath re-anchors the centreline at the ped (the `l_r`/`OffsetWithResume` path stays proven for the lane-centre-frame case). `WeavingPed_PromotesThenDemotes_ResumesWeave_NoPop_ServerEqualsIgOverTheWire`: resumes as ActivityTimeline (not PathArc), no teleport across the switch, switch arrives over the wire, server==IG bit-exact after demote, weave active after. Non-weaving demote unchanged (gate). Gate 649+3, ped 208/208)*
- [ ] **W-polish** (deferred) edge-join width smoothing + heading polish + per-ped demote timing

---

## 9. Resolved decisions (sign-off)
1. **Seed source — RESOLVED.** Derive the per-ped weave seed from the SHARED scenario root via
   `VehicleRng.SeedFor(Engine.Seed, pedIndex, WeaveSalt)` — the same rooted, salted, thread-order-independent
   discipline that makes cars deterministic. No new seed infrastructure; gives whole-run reproducibility,
   server==IG, and forward-compat for future deterministic ped↔car interaction.
2. **Width granularity — RESOLVED.** Piecewise per-vertex width from day 1 (the route→width association is paid
   regardless; the array is the eventual format). Only edge-join width *smoothing* is deferred as cosmetic.
3. **Heading — RESOLVED.** Keep the raw tangent as render heading; offset-derivative heading is deferred polish.
4. **CenterShift — RESOLVED.** Folded into the core (W1), not deferred: its only dependency (the scenario-global
   `Engine.Seed`) is already introduced for the per-ped seed, so including it early is nearly free and it is in
   the final solution regardless. It adds a direction-aware clamp (the moving interface squeezes one side).
