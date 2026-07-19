# PEDESTRIAN-LIVELINESS-DESIGN.md — believable "alive" pedestrians (activity timelines)

**Status: design, for review.** Extends the pedestrian subsystem (`PEDESTRIAN-OVERVIEW.md`,
`PEDESTRIAN-DESIGN.md`) with **liveliness**: pedestrians that don't just walk but stop, sip a drink, meet
and talk, sit at restaurant tables, enter and leave buildings — and can do so while staying **low-power**
(cheap on CPU and network, deterministically replayed by the IG). Companion POC plan is §12 below (this
is a focused feature, so design + plan are folded into one doc per `CLAUDE.md`).

This design is written to be **compatible with the SumoData subarea mechanism** (auto-deduced demand +
calibrated density within a user-selected subarea of a large road network) — see §11, now aligned to the
SumoData brief `SUBAREA-FOR-PEDESTRIAN-SESSION.md` (full mapping in `COORDINATION-pedestrian-x-subarea.md`).

---

## 1. The one principle: liveliness = deterministic activity-timeline replay

The existing low-power pedestrian is **`PathArc`**: its pose is a pure function `position = f(path,
startTime, speed, now)`, generated once and replayed identically by the IG (server == IG, ~0 ongoing
bytes — `PEDESTRIAN-DESIGN.md` §7, verified in POC-3). Liveliness is the *same trick with a richer
function*: replace the single path with a **precomputed schedule of segments** whose evaluation yields
**both a pose and an animation state** as a pure function of `(schedule, startTime, now)`.

> **Liveliness does not add a per-step behavior loop; it adds richer one-time data that both the server
> and the IG evaluate by the clock.** That is exactly why it stays low-power.

The `PathArc` DR model (`PEDESTRIAN-DESIGN.md` §7) generalizes to a **`ScheduledActivity`** DR model. The
one-time `PathArcRecord` generalizes to an `ActivityTimelineRecord` (a small segment list). Everything
else in the low-power story — sent once on the durable topic, silent per-step, IG-reconstructed — is
unchanged.

## 2. The activity timeline model

An `ActivityTimeline` is an ordered list of segments, each covering a `[t0, t1)` slice of the ped's life:

| Segment | Meaning | Pose during it | Animation |
|---|---|---|---|
| **`Walk(path, speed)`** | move along a polyline (today's `PathArc`) | arc-length along `path` | `walk` (speed-driven) |
| **`Pause(pos, dur, tag)`** | stop in place and do something | fixed `pos` | `tag` (e.g. `sip`, `phone`, `look`) |
| **`Dwell(poi, dur, tag, visible)`** | occupy a spot (table/bench) or be inside a building | `poi.pose` (or hidden) | `tag` (e.g. `sit`, `idle`); `visible=false` = inside/vanished |
| **`Interact(meetPos, offset, dur, tag, partnerRef)`** | the "step aside and talk/greet" beat | `meetPos + offset` | `tag` (e.g. `talk`, `greet`) |

Evaluation is a cursor walk: find the segment containing `now`, compute pose + anim from it. O(log n) or
O(1) amortized; allocation-free. **This shared evaluator (`ActivityMotion.Evaluate`) is used by BOTH the
server (to advance the ped and to expose it as a footprint) AND the IG (to reconstruct pose + pick the
animation clip)** — the same discipline that makes `PathArcMotion` server==IG (POC-3). A pure `Walk`-only
timeline is byte-identical to today's `PathArc`.

## 3. The animation contract (sim emits timing, IG owns the art)

The sim never plays animations; it emits, per active segment, an **`(animTag, segStartTime, duration,
loopMode)`** triple. The IG owns the actual clips and blending. This needs a small **contract** (like the
navmesh contract, `PEDESTRIAN-NAVMESH-CONTRACT.md`):

- A **tag vocabulary** (an enum: `Walk, Idle, Sip, Phone, Look, Sit, StandUp, Talk, Greet, Serve, …`) —
  extensible, `byte`-backed for the wire.
- **Loop semantics** per tag: `oneShot` (sip), `loopWhileInSegment` (talk/sit), `transition` (standUp).
- The **timing is authoritative and deterministic**; the IG blends toward the tag but must land the ped at
  the pose the evaluator dictates at each segment boundary. Missing-tag fallback = `Idle`/`Walk`.

## 4. The schedule generator (extends `PedDemand`)

`PedDemand` (P2-3) already picks O→D and routes; it becomes the **activity scheduler**. When it spawns a
ped it composes a timeline instead of a bare path, seeded-deterministically (`VehicleRng` SplitMix64):

1. Route O→D over the ped network (`IPedNavigation`), producing the `Walk` backbone.
2. With seeded probabilities, **insert** `Pause`/`Dwell` segments: a `Pause` mid-walk (sip/look); a detour
   to a nearby POI (`Dwell` at a table/bench/shop window); a building visit (route to entrance → `Dwell(visible=false)` → re-emerge or despawn, §6).
3. Optionally enroll the ped in a **pre-scheduled interaction** (§5) or a **micro-scenario** (§7).

The output is the `ActivityTimeline`. Because it is a pure function of `(seed, netData, spawnTime)`, it is
reproducible and IG-consistent.

## 5. Interactions — the pre-scheduled social planner (the subtle part)

"Two peds meet, move aside, and talk" is the one behavior that needs **two agents to agree**. To keep it
low-power (deterministic + IG-reproducible) the agreement is decided **at schedule-generation time, not at
runtime**:

- A **social planner** in the demand layer occasionally pairs two peds whose routes bring them near the
  same place around the same time. It picks a **meet point** just off the walkable flow (a widened spot /
  sidewalk edge / plaza), a small **± offset** for each, and a **duration**, and writes **matching
  `Interact` segments** into *both* timelines: A goes to `meetPos − offset`, B to `meetPos + offset`, both
  play `talk` for `[T, T+D]`, then both resume their onward `Walk`.
- Both schedules are authored together and seeded, so they are self-consistent; the IG replays both
  exactly. **No runtime negotiation → no promotion needed → stays low-power.**
- A **reactive** interaction (a player avatar walks up to a ped) is different: that is a genuine live
  reaction, handled by promoting the ped to high-power (existing LOD machinery). Ambient chatter is
  pre-scheduled; reactive interaction is promoted. The LOD split already draws this line.

**Hard part (open, §12/§13):** the *pairing* rule (which peds, where they meet without blocking a doorway
or the flow) and the *geometry* (meet point + offsets that read as "stepped aside"). A bad rule makes pairs
teleport-converge or clog a portal.

## 6. Buildings & venues — sinks, sources, and the no-cheating synergy

"Go inside a building and come out later" has two clean models; support both:

- **Hidden dwell** (keeps identity): route to the entrance POI → `Dwell(visible=false, dur)` at the door →
  re-emerge and continue. The ped still exists (a slot, a schedule) but renders nothing while inside.
  Best for short visits where identity/continuity matters.
- **Despawn-at-door + respawn** (loses identity): the ped despawns at the entrance; later, an *unrelated*
  ped spawns emerging from that door. Best for large flows (shops, transit) where individual identity
  doesn't matter — and it is cheaper (no dormant slots).

**Synergy with no-visible-cheating (subarea, §11):** a **building entrance is a natural, believable
spawn/despawn point** — "a person comes out of a building" and "a person goes into a building" are exactly
the visible appear/disappear the subarea mechanism *wants* (vs. popping on an open sidewalk). So building
POIs double as (a) liveliness destinations and (b) legitimate demand sources/sinks inside the visible area.
This is a strong reason to make building/venue POIs first-class net-data.

## 7. Micro-scenarios — templated scripted actors

A "waiter serves the outdoor tables" is a **templated looping schedule** anchored to a `(building, table-cluster)` pair: emerge from the service door → go to a table in the cluster → `Dwell(serve)` → return →
loop, with seeded variation. Model it as a **scripted actor**: a low-power ped whose timeline is produced
by a **micro-scenario template** bound to a location, rather than by random demand. Other templates:
street performer (dwell + `perform` loop at a plaza anchor), shop greeter, queue at a food truck. Templates
are data + a small generator; each actor is still a deterministic `ActivityTimeline` → low-power.

## 8. POI net-data schema (extra road-net data)

Ingested by the existing separate `PedNetworkParser` (it already reads `<poly>`/`<poi>` — the plaza/parking
used them). New POI layers:

- **`building_entrance`** — a portal point (+ facing) on a building footprint; a source/sink and a
  `Dwell(inside)` target.
- **`venue` / `table_cluster`** — a restaurant/café anchor + a set of table/seat spots with **capacity**.
- **`dwell_spot`** — benches, viewpoints, shop windows (generic `Pause`/`Dwell` magnets), with capacity.
- **`scenario_anchor`** — binds a micro-scenario template to a location (e.g. `waiter@building7+cluster3`).

These are additive companion data (like `walkable.add.xml`). **For subarea compatibility (§11), the schema
is chosen to be auto-deducible from road-net / OSM data** (building footprints + `amenity`/`shop` tags →
venues; door nodes → entrances; sidewalk-adjacent open areas → dwell spots), so the subarea tool can
generate them the same way it deduces vehicle demand.

## 9. Capacity & occupancy

Tables, benches, and entrances have finite capacity; the scheduler must not seat 50 peds at 8 tables. Since
scheduling is deterministic and (per subarea) done up front, capacity is a **resource reservation at
schedule time**: a POI has N slots on a timeline; the scheduler reserves a `[t0,t1)` slot before writing a
`Dwell` to it. Deterministic, but adds bookkeeping (a per-POI interval calendar). Over-subscription falls
back to a shorter/relocated dwell or none.

## 10. LOD & state-machine integration

- The low-power `Walking` sub-state (`PEDESTRIAN-DESIGN.md` §2) generalizes to **`Scheduled`**: following an
  `ActivityTimeline`. `Pause`/`Dwell`/`Interact` are sub-modes of it, not new regimes.
- **Coherence:** a paused/dwelling/talking low-power ped is exposed to the high-power layer as a **static
  `WorldDisc`** (its evaluated pose), so full-ORCA peds and cars avoid the person standing at the table or
  the pair on the sidewalk. A `Dwell(visible=false)` (inside) ped exposes **no** disc. No special-casing.
- **Promotion mid-activity:** if a `Scheduled` ped is promoted (interest source arrives), it **freezes the
  current segment**, hands its evaluated pose+velocity to ORCA, and runs reactive; on demotion it
  **re-plans** the remainder of its timeline from the current time/pose (a fresh schedule, like today's
  demotion re-route). The animation tag persists across the switch where sensible.

## 11. Subarea compatibility (SumoData)

Aligned to the SumoData brief `SUBAREA-FOR-PEDESTRIAN-SESSION.md` (§3, 7 requirements) and its serve-path
note `SUMOSHARP-SERVE-PATH-DROP-IN.md`. The per-requirement mapping, the gaps, and the plan pointers live
in `COORDINATION-pedestrian-x-subarea.md`; this section states the design consequences for liveliness.

**The load-bearing distinction (from their brief).** *sim-LOD is not appearance-legitimacy.* Two
orthogonal axes:
- **sim-LOD** (`PedLodManager` + `InterestField`) — how much compute/wire detail a ped gets. A low-power
  ped is still fully present and visible. This is what §10 wires liveliness into.
- **appearance-legitimacy** (new) — whether a ped may legitimately *materialize / vanish* at a given place
  right now. The no-cheating rule. **Orthogonal to LOD**, and the piece the earlier draft conflated.

Consequences for liveliness:

- **Buildings/venues are the primary in-view legitimacy anchors.** A building **entrance** (§6, §8) is the
  believable place a ped can appear from / disappear into *inside* the visible box — exactly the on-camera
  appear/disappear the subarea mechanism wants, versus popping on an open sidewalk. So the §6 hidden-dwell
  and despawn/respawn paths are not just liveliness flavour: they are the **no-cheating spawn/despawn
  mechanism** for internal demand. This makes liveliness and no-cheating the *same* feature, not two.
- **Fringe + off-camera are the other two legitimacy anchors.** Peds may also appear/disappear at the box
  **fringe** (walkable edges cut by the crop) or **off-camera** (walkable edge not in the camera visible
  set — the direct analogue of the vehicle side's `RealismMask.MayPop`). A denied on-camera despawn does
  not vanish the ped; it routes to the nearest door/fringe or holds (low-power) until off-camera.
- **One camera signal, two uses.** The host's single frustum yields one visible-edge set. The camera stays
  a sim-LOD interest source (§10, unchanged) **and** additionally drives the appearance-legitimacy gate
  (the visible **walkable**-edge set). Liveliness is still spent where it's seen (on-camera → full timeline
  + promotion-eligible; off-camera → cheap `Walk`/culled), but "spend liveliness here" and "may vanish
  here" are now decided by two separate predicates over the same signal.
- **Auto-deduced demand feeds the timeline.** O→D + liveliness POIs are deduced from the subarea's
  walkable-space + land-use/POI net-data (§8), mirroring their topology-based vehicle deduction (their
  `deduce_weights.py` offered as a template). The schedule generator (§4) is the injection point; spawns
  land at fringe/doors per the legitimacy gate.
- **Density knob + crossing guard.** Expose a pedestrian density knob (peds/area or peds/sidewalk-m)
  analogous to the vehicle density level, and cap crossing occupancy so crowds never deadlock a signalized
  crossing hard enough to gridlock the (separately calibrated) cars.
- **Determinism under the calibrator.** The subarea tool runs many probes; the seeded schedule generator +
  legitimacy gate must be reproducible per (seed, subarea, density) so probes are comparable. The gate is a
  pure function of (seed, ped, edge, visible set), and the visible set is captured once per host tick.

These consequences are realized by **Stage P8 — Subarea integration** in `PEDESTRIAN-TASKS.md` (P8-2 is the
appearance-legitimacy layer; P8-3 the auto-deduced demand; P8-4 the density knob). LIVE-POC-4 (§12) is the
end-to-end demonstration once P8 and a real cropped box are in hand.

## 12. POC plan (de-risk the two non-obvious pieces, then make it watchable)

De-risk exactly the parts that aren't obviously low-power/deterministic, and visualize each as a Sim.Viz
HTML demo (mobile-watchable, gallery-publishable) so liveliness can be *seen*.

- **LIVE-POC-1 — `PathArc` → `ActivityTimeline` (`Walk`+`Pause`+`Dwell`).** A ped walks a route, sips
  (`Pause`), sits at a table (`Dwell`), enters a building (`Dwell(visible=false)`), re-emerges, arrives.
  **Success:** server pose+animTag == IG reconstruction from the one-time timeline at every sampled `now`
  (the POC-3 server==IG invariant, extended to animation state); the dwelling ped is a static `WorldDisc`
  a high-power ped avoids; the `Walk`-only path stays byte-identical to today's `PathArc`; deterministic
  run-to-run. Sim.Viz demo: `liveliness-solo`.
- **LIVE-POC-2 — pre-scheduled two-ped `Interact`.** The social planner pairs two peds; both get matching
  `Interact` segments; they converge to just-off-flow offsets, play `talk`, and part — all from
  one-time schedules. **Success:** both peds reconstruct identically on the IG from their timelines; they
  never overlap (min sep ≥ sum-of-radii); they don't block the portal/flow (a third routed ped still
  passes); deterministic. Sim.Viz demo: `liveliness-talk`.
- **LIVE-POC-3 — micro-scenario actor (waiter).** A templated looping schedule (emerge → serve a table →
  return → loop) bound to a `(building, table-cluster)` anchor. **Success:** the actor loops
  deterministically, respects table capacity, is IG-reconstructable, and reads believably in the demo.
  Sim.Viz demo: `liveliness-waiter`.
- **LIVE-POC-4 (after Stage P8 + a real cropped box) — auto-deduced liveliness demand in a subarea +
  density knob + shared-mask legitimacy.** Deduce POIs + ped demand from subarea net-data (P8-3); scale to a
  density level (P8-4); spend liveliness by the camera interest source; gate appearance on the shared
  visible-edge set (P8-2); verify no-visible-cheating (appear/disappear only at fringe/doors/off-camera).
  **Success:** measured density hits target; **zero on-camera pops** (audited like the vehicle side's
  `audit_nocheat.py`); liveliness concentrated in the high-realism zone.

Only after these converge does liveliness graduate to production tasks (extend `PedDemand`, the wire
`ActivityTimelineRecord`, the animation-tag contract, the POI ingest, capacity calendar).

## 13. Open questions / risks

1. **Interaction pairing rule + geometry** (§5) — which peds pair, where they meet without clogging flow.
2. **Building model choice** per venue (hidden-dwell vs despawn/respawn, §6).
3. **Capacity calendar** cost/complexity at scale (§9).
4. **Animation-tag vocabulary + blend semantics contract** (§3) — needs IG-side buy-in.
5. **Promotion mid-activity** semantics (§10) — freeze/replan edge cases (e.g. promoted while inside a
   building).
6. **Subarea alignment** (§11) — aligned to the SumoData brief; the open sub-question is the exact
   walkable-edge visible-set handoff from the host (Stage P8-2) and sharing their `deduce_weights.py`
   template (P8-3).

---

**Document status:** design + POC plan, for review. The load-bearing claim — liveliness stays low-power
because it is deterministic-schedule replay (the `PathArc` trick generalized) — is what LIVE-POC-1/2 prove
before any production build. §11 is aligned to the SumoData brief; see `COORDINATION-pedestrian-x-subarea.md`
for the full requirement mapping and Stage P8 for the implementation plan.
