# PANIC-EVAC.md — organized traffic → panic evacuation

## 0. Goal

Organized urban traffic that, on a localized **security incident** (bomb, shooting, armed
strike), switches to a **panic evacuation**: cars flee, streets block solid, and once a car is
boxed in its occupants **abandon it and flee on foot**, off the lane onto the sidewalk/grass,
until they hit the edge of the known world (a ditch/fence/field) and pile up. Organized traffic is
a realistic-enough backdrop; the evacuation is the product. **Not** an "Indian traffic" model.

## 1. Consolidated requirements (authoritative)

- **R1 — organized backdrop.** Ordinary lane driving. **Sublane is an optional, separate** layer
  (value: scooters/cyclists filtering to the front at red lights, still stopping on red). Start
  **without** sublane.
- **R2 — external decides, the core drives (vehicles only).** The panic **decision** is external,
  applied **per vehicle**. The core exposes the individual SUMO driver knobs (impatience,
  sublane assertiveness, `tau`/gap, speed-factor, right-of-way relaxation, …) **independently
  settable** at runtime — they are not hidden behind panic. **"Flee mode" is just another
  override/call**: it bulk-sets those knobs to **predefined aggressive values**, overwriting
  whatever was there. It is a preset over the same param surface, not a special state. The vehicle
  modes the core drives:
    - **Organized** — normal lane/sublane driving.
    - **Flee** — the aggressive-preset params applied; still normal on-road lane driving, just
      pushy, on a flee route. (A param preset, not a separate code path.)
    - **Orca** *(later phase)* — a genuinely separate mode: free-space movement in the
      road+vicinity band when organized driving has nowhere to go (cars mounting the shoulder). The
      core owns it. Selected by an explicit `mode` flag, not by the param preset.
- **R3 — panic spread is a separate external layer.** Fear/contagion sits on top of the shared
  data; **local-information only** (direct LoS-gated proximity + contagion from panicking
  neighbours + mild jam-unease). No global broadcast → distant traffic stays organized, unaware,
  just jammed.
- **R4 — the port emits a `blocked` signal.** Per vehicle, when it can no longer make progress —
  **derived from the DR seam**: `DrModel.Stationary` (≈ stopped) held for a short dwell with no
  feasible forward gap. No brand-new signal; reuses issue #3's regime read.
- **R5 — pedestrians are ALWAYS external.** The port does **not** simulate pedestrian movement; it
  only sees pedestrians as **external obstacles** (`ExternalObstacle`/`WorldDisc`) its vehicles
  avoid. The **external system drives them** (fake-navmesh + ORCA).
- **R6 — driver→pedestrian conversion is external, at the `blocked` boundary.** On `blocked` **and**
  the vehicle already in flee/panic (tracked by the external layer), the external layer: (a) creates
  a pedestrian entity (external obstacle) and starts driving it; (b) commands the port to **stop the
  vehicle for good → static obstacle**. The entity *migrates* from port-simulated vehicle to
  externally-simulated pedestrian at this instant.
- **R6b — escape = far enough from the incident (not necessarily the world edge).** A pedestrian's
  goal is simply to increase distance from the incident; it is **Escaped** once beyond a safe
  radius — reached at a **far-enough street point or off-road point**, which may be well short of
  the known-world boundary (that boundary is only a hard blocker, not the required destination).
- **R7 — known world = road + close vicinity.** Lanes + an immediately-adjacent band. Where the
  SUMO net **has sidewalks** (SUMO models them as pedestrian-only lanes), the band follows the
  **sidewalk geometry** — it *defines/extends* the vicinity where present, and usually there is a
  blocker just beyond it. Where absent, a **fixed tunable width** is used. Its **outer edge is a
  hard boundary** (ditch/fence/soft soil) where actors **block** — matching reality (cars/people
  pile at the road edge). Beyond = future real navmesh, out of scope; actors mostly block first.
- **R8 — fake-navmesh from SUMO data only (external).** Navigable region = lane+junction geometry
  (incl. sidewalk lanes where present) buffered outward by the vicinity width; outer edge = wall;
  cars (incl. abandoned) = interior obstacles. Replaceable later by a real 3D-world navmesh behind
  the same interface — out of scope.
- **R9 — parity.** The driving core stays parity-exact; with no panic params set and no external
  obstacles, it is byte-identical to today (hash `909605E965BFFE59` unchanged). The panic layer is
  parity-exempt.
- **R10 — reuse the frozen seams.** Everything rides the issue #4 interfaces: `WorldDisc` /
  `ExternalObstacle` (pedestrians-as-obstacles), `ObstacleStore` (frozen cars), `DrModel`/stuck
  (basis for `blocked`), plus per-vehicle param + stop-vehicle inputs. Reinforces the freeze.

## 2. Two systems, one seam

```
  DRIVING CORE (SUMO port) — VEHICLES ONLY            EXTERNAL EVAC SYSTEM
  ┌───────────────────────────────────┐              ┌──────────────────────────────────┐
  │ per-vehicle mode:                  │  vehicle     │ panic decision (fear + contagion, │
  │   Organized → Flee → (Orca later)  │  state,      │   local-information only)          │
  │ lane car-following (+opt sublane)  │  DrModel,    │──► sets per-vehicle PARAMS ───────►│ (into core)
  │ avoids EXTERNAL OBSTACLES (peds)   │  `blocked`   │                                    │
  │ stop-vehicle → static obstacle     │ ───────────► │ on `blocked` + panic:              │
  └───────────────────────────────────┘              │   • spawn PEDESTRIAN (ext obstacle)│
        ▲ params / stop-cmd / ext-obstacles           │   • stop the vehicle (cmd core)    │
        │                                             │ drives ALL pedestrians:            │
        └───────── pedestrians as ext obstacles ◄──── │   ORCA + fake-navmesh (road+vicin.,│
                   (fed back each step)               │   cars as obstacles, hard edge)    │
                                                      └──────────────────────────────────┘
   KNOWN WORLD = road+vicinity; hard outer edge (ditch/fence) → actors BLOCK.
   BEYOND = real 3D navmesh, OUT OF SCOPE.
```

- **Driving core** — simulates vehicles only. Modes are param-selected (R2); parity-exact with no
  panic. Consumes: per-vehicle params, a stop-vehicle→obstacle command, and external obstacles
  (pedestrians). Emits: vehicle state, `DrModel`/stuck, and the per-vehicle `blocked` signal (R4).
- **External evac system** — owns the panic decision (R3), all pedestrians (R5, driven by
  fake-navmesh + ORCA), and the driver→pedestrian handoff at `blocked` (R6). Feeds pedestrian
  positions back to the core as external obstacles each step. Replaceable by a real-navmesh system
  later — the seam is the contract.

## 3. The fake-navmesh (external; road-network-derived; replaceable)

- **Navigable region** = union of lane+junction polygons, **buffered outward** by a close-vicinity
  width — derived entirely from the SUMO `NetworkModel` geometry, no external world data.
- **Hard outer edge** = boundary of that region → a wall pedestrians cannot cross (ditch/fence).
- **Interior obstacles** = the cars (moving, stuck, abandoned) the core exposes.
- **Movement** = ORCA within the region, bounded by the edge, avoiding cars + each other.
- **Replaceable:** a real navmesh (building footprints, walkable/blocked polygons, elevation) drops
  in behind the same "navigable region + obstacles" interface. Out of scope now.

## 4. Per-entity lifecycle

```
 VEHICLE (driving core)
   Organized ─(external sets panic params)─► Flee ─(core, aggressive organized flee)─►
      │  [core emits `blocked`]   ── (later phase: Orca free-move in vicinity, then `blocked`) ──
      ▼
   external, on `blocked`+panic:  stop vehicle → STATIC OBSTACLE (core)  +  spawn PEDESTRIAN
                                                                                   │
 PEDESTRIAN (external evac system: ORCA + fake-navmesh; a moving external obstacle to the core)
      flee AWAY FROM THE INCIDENT within road+vicinity, avoiding cars + walls + peds; block at edge
      │  distance-from-incident > safe radius (a far-enough street or off-road point)
      ▼
   Escaped (may be well short of the world boundary; the edge is only a hard blocker)
```

## 5. Believability / correctness bar

- **Hard invariants:** no panic params + no external obstacles ⇒ core byte-identical to today
  (hash unchanged); no vehicle interpenetration on-road; no pedestrian crosses the hard edge;
  deterministic runs.
- **Behavioural targets:** panic front propagates outward (never teleports); distant traffic stays
  organized/jammed and unaware; jam → `blocked` → driver→pedestrian → foot-exodus cascade
  **emerges**; cars pile at the road edge; evacuation drains toward the away-edges.
- **Acceptance = the viz** (organized traffic, incident marker, fear overlay, fleeing cars,
  abandoned-car obstacles, externally-driven pedestrians, the known-world edge), backed by the
  invariants.

## 6. Phased roadmap (panic-first; each phase watchable)

1. **Spine** — non-sublane organized traffic; external incident + radius fear; external sets Flee
   params on nearby cars; core drives aggressive on-road flee; core emits `blocked`; external
   converts (stop vehicle → obstacle + spawn pedestrian) and drives pedestrians (ORCA +
   fake-navmesh, cars as obstacles) to the away-edge. Both rendered. *Done:* full transition plays
   once. **(Vehicle Orca-mode not needed yet — Flee → blocked → pedestrian.)**
2. **Panic as local information** — contagion + LoS + jam-unease; distant traffic oblivious.
3. **Vehicle Orca-mode / off-road** — cars push into the vicinity (shaped NH free-space model, in
   the core) before blocking; richer fake-navmesh buffer.
4. **Sublane realism (optional, separate)** — filter-to-front at lights in the organized phase.
   **DEFERRED** — see `PANIC-EVAC-PHASE4-DECISION.md`: needs a cohesive `MSLCM_SL2015` parity port (two
   prior attempts failed) and is parity-BOUND; the 10k-vehicle perf is safe because sublane is a global
   per-scenario switch (the fast non-sublane city is a separate run), so this is low-value/high-risk now.
5. **Scale** — hundreds → thousands; spatial indexing; far side stays jammed and unaware.

## 7. Resolved decisions + remaining tunables

Resolved (folded into R1–R10 above):
- **`blocked`** = `DrModel.Stationary` + dwell, no feasible forward gap (R4). No new signal.
- **Param surface** = individual SUMO knobs stay independently settable; **flee mode is a preset
  override** that bulk-sets predefined aggressive values (R2). `mode` (Orca) is a separate flag.
- **Vicinity band** = follows **sidewalk lanes where the net has them**, else a fixed tunable
  width; hard outer edge just beyond (R7).
- **Escape** = pedestrian gets **far enough from the incident** (safe radius), not necessarily to
  the world boundary (R6b).

Remaining as **tunables** (calibrated against the viz, not fixed here):
- Fixed vicinity width (where no sidewalk); dwell time for `blocked`; the flee-preset param values;
  the safe-escape radius; fear constants (θ_panic, decay, contagion kernel/radius).

To pin only at Phase-1 kickoff (implementation detail, not architecture):
- Which SUMO knobs already exist vs. need a small additive opt-in setter; hard-edge representation
  (buffered-polygon obstacle loop vs. explicit fence geometry).

## 8. Phase-1 build plan (kickoff checklist)

**Base:** branch `claude/sumo-phase-2-planning-p3w7kh-i1gsgu`, rebased on `main` (the merged
mainline — has the `VTypeParams` lateral fold + `SimulationSnapshot.DrModels`/`.Manoeuvring`).
Parity-exempt; the engine with no panic + no external obstacles must stay byte-identical (hash
`909605E965BFFE59`).

**Spine (smallest end-to-end transition on a small hand-built network):**
1. **`Incident`** — data: world point + start time + radius (external input).
2. **Fear (radius-only for Phase 1)** — per-vehicle `fear = f(distance to incident)`; panic when
   `> θ_panic`. Contagion/LoS/jam-unease deferred to Phase 2.
3. **External evac layer** — a **new module (`Sim.Evac`)** that each step:
   - reads engine vehicle state (`VehicleReadBuffer` columns / `TryGetVehicle`) + `DrModels`;
   - computes fear, marks panicked vehicles, applies the **flee preset** params + a flee route to
     them (via the core's param/reroute surface);
   - detects **`blocked`** = `DrModel.Stationary` for a dwell (external counts dwell off `DrModels`);
   - on `blocked`+panic: commands the core to **stop the vehicle → static obstacle**, and **spawns a
     pedestrian** into an `OrcaCrowd`;
   - **drives the pedestrian `OrcaCrowd`** (fake-navmesh region + car obstacles) and feeds pedestrian
     positions back to the engine as **external obstacles** (`SetExternalObstacles` / the bridge).
4. **Fake-navmesh (minimal)** — buffer lane+junction polygons outward by a fixed width → a boundary
   obstacle loop for the crowd; cars (stuck/abandoned) as interior obstacles.
5. **Reroute-to-flee** — set a panicked vehicle's route toward an away-direction exit (road graph).
6. **Viz** — reuse `Sim.Viz`: organized cars, incident marker, fleeing cars, abandoned-car
   obstacles, externally-driven pedestrians, the known-world edge.

**Core integration points — confirm exists vs. add small additive opt-in (do NOT touch the golden path):**
- flee-preset param setter (`aggressiveness` → SUMO knobs) — likely additive;
- reroute a vehicle to a flee goal — SUMO dynamic reroute (confirm surface);
- stop-vehicle → static obstacle command (`ObstacleStore`) — likely additive;
- external-obstacle feed for pedestrians — **exists** (`SetExternalObstacles` / `WorldDisc` bridge);
- `blocked` — no core change; external derives it from the `DrModels` span + a dwell timer.

**Tests (invariants):** panic-absent ⇒ full suite byte-identical (hash unchanged); deterministic
(bit-identical run); no pedestrian crosses the hard edge; cascade emerges (incident → flee → some
`blocked` → pedestrians spawn → reach the safe radius).

**Done-condition:** the full transition plays once, coherent, in the viz on a small network.

## 9. Relationship to other docs

- `INDIA-TRAFFIC.md` — shaped / non-holonomic / ORCA **machinery** (`Sim.Core.Mixed`,
  `ShapedVoSolver`, bicycle model): reusable substrate (Phase 3 vehicle Orca-mode + the external
  pedestrian ORCA), not the headline.
- `LANELESS-DIRECTION.md` — the open-space ORCA regime + cross-regime bridge (`ExternalObstacle` /
  `WorldDisc` / `SetExternalObstacles`) the seam is built on.
- `SUMOSHARP-DEADRECKONING.md` (NuGet branch) — the `DrModel` seam the `blocked` signal rides.
