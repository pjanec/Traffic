# PANIC-EVAC.md — organized traffic → panic evacuation (within the road's known world)

## 0. Goal

Organized urban traffic that, on a localized **security incident** (bomb, shooting, armed
strike), switches to a **panic evacuation**: cars flee, streets block solid, people **abandon
their cars and flee on foot** — off the lane, onto the sidewalk / grass shoulder, until they hit
the edge of the known world (a ditch/fence/field) and pile up. Organized traffic is a
realistic-enough backdrop; the evacuation is the product. **Not** an "Indian traffic" model.

## 1. Consolidated requirements (authoritative — refined across discussion)

- **R1 — organized backdrop.** Ordinary lane driving. **Sublane is an optional, separate** layer
  whose value is small vehicles (scooters/cyclists) **filtering to the front at red lights**
  (still organized — they stop on red). Start **without** sublane.
- **R2 — external *decides*, the core *drives*.** The panic **decision** (who panics, when) is
  external and applied **per vehicle** by setting that vehicle's **parameters** (`aggressiveness`,
  `mode`). The **three modes those parameters select — Organized → Flee → Orca — and the
  Flee→Orca escalation all live INSIDE the driving core.** The external layer only flips
  parameters; it never implements driving. (This is the SUMO + TraCI pattern: external code sets
  per-vehicle params at runtime, the engine drives accordingly.)
    - **Organized** — normal lane/sublane driving.
    - **Flee** — aggressive organized driving to a flee route (raised impatience / assertiveness /
      speed factor, smaller gaps, relaxed right-of-way). Still on roads, still the SUMO model.
    - **Orca** — the core escalates a fleeing vehicle to ORCA free-movement (within road+vicinity)
      **when organized driving has nowhere to go** (boxed in). The core detects this and switches
      the mode; the vehicle is then stepped by the ORCA solver instead of lane car-following.
- **R3 — panic spread is a separate layer.** Fear/contagion modelling sits **on top of** the
  driving data, not wired into it. It is **local-information only**: direct proximity (LoS-gated) +
  **contagion** from panicking neighbours + mild **jam-unease** — no global "flee" broadcast, so
  **distant traffic stays organized and unaware**, just jammed.
- **R4 — the known world = road + close vicinity.** Lanes plus the immediately-adjacent band
  (sidewalks, grass shoulder). Our system operates within this. Its **outer edge is a hard
  boundary** (ditch/fence/soft soil) where actors **get blocked** — which matches reality, so
  vehicles rarely get "properly" off-road; they jam at the edge.
- **R5 — panic movement = ORCA within the known world.** Panicked vehicles (and pedestrians) move
  by free-space ORCA inside the road+vicinity band; **stuck/abandoned cars are obstacles**.
- **R6 — driver→pedestrian conversion is orchestrated (external).** When a car is boxed in, its
  occupants become pedestrians (ORCA agents) that flee toward the **away-edge** of the known
  world. Full *world-scale* pedestrian navigation (into buildings, across the city) is **out of
  scope** — deferred to a future real navmesh.
- **R7 — fake-navmesh from SUMO data only.** The navigable region is derived purely from
  lane+junction geometry (buffered outward by a vicinity width); the outer edge is a wall; cars
  are the interior obstacles. **Replaceable** later by a real 3D-world navmesh behind the same
  boundary — out of scope now.
- **R8 — parity.** The organized driving core stays parity-exact; the panic layer is
  parity-exempt; with panic **absent** the engine is byte-identical to today (hash
  `909605E965BFFE59` unchanged).
- **R9 — reuse the frozen seams.** Everything rides the issue #4 interfaces (`WorldDisc`
  obstacles, `DrModel` regime/stuck, the cross-regime bridge) plus generic per-vehicle control
  inputs (reroute / release). The design reinforces that freeze, doesn't disturb it.

## 2. The one system + its known-world boundary

There is **one** system for the realistic scenario (no concurrent "external navmesh engine"):

```
        KNOWN WORLD  = lane polygons ⊕ close-vicinity buffer (sidewalk + grass)
   ┌──────────────────────────────────────────────────────────────────────┐
   │  DRIVING CORE (SUMO port) — per-vehicle mode: Organized → Flee → Orca  │
   │  · Organized/Flee: lane car-following (+ optional sublane filter-to-front) │
   │  · Orca: free-space movement in road+vicinity (nowhere-to-go escalation)│
   │  · junctions / right-of-way;  stuck/abandoned cars = obstacles;  peds   │
   │                                                                        │
   │   ── hard outer edge (ditch / fence / field): actors BLOCK here ──     │
   └──────────────────────────────────────────────────────────────────────┘
              ▲ per-vehicle params: aggressiveness/mode/route (external)
              ▲ fear/contagion decision layer (external, on top)
   BEYOND: real 3D-world navmesh — OUT OF SCOPE (future); actors mostly block before reaching it
```

- **Driving core** — owns all three per-vehicle modes (Organized / Flee / Orca) and the Flee→Orca
  escalation (R2). Organized+Flee are lane driving (parity-exact when no panic params are set);
  Orca is free-space movement (reusing `OrcaCrowd` / `Sim.Core.Mixed` + the bridge) inside the
  known world, bounded by the hard outer edge. Exposes state + cars-as-obstacles + `DrModel`/stuck;
  accepts per-vehicle **parameter** inputs (`aggressiveness`, `mode`, flee route). It never decides
  *whether* to panic — only executes the params it's given, plus the mechanical Flee→Orca escalation.
- **Panic-spread layer** — separate, external; reads the shared data, decides who panics when, and
  flips the per-vehicle params (R2/R3). Thin: a decision layer, no driving.
- **Boundary** — actors that reach the outer edge **block** (R4). "Escape" = reaching the
  **away-edge** of the known world (a street leading away from the incident, off the simulated
  area). Deep off-road / buildings = future navmesh, out of scope.

## 3. The fake-navmesh (road-network-derived, replaceable)

- **Navigable region** = union of lane + junction polygons, **buffered outward** by a
  close-vicinity width (a couple of metres of sidewalk/shoulder). Derived entirely from the SUMO
  `NetworkModel` geometry — no external world data.
- **Hard outer edge** = the boundary of that buffered region → a wall actors cannot cross (the
  ditch/fence/field). This is what makes fleeing vehicles pile up at the road edge (realistic).
- **Interior obstacles** = stuck/abandoned cars (`ObstacleStore` / `WorldDisc` footprints).
- **Movement** = ORCA within the region, avoiding cars + each other, bounded by the edge.
- **Replaceable:** a real navmesh (building footprints, walkable/blocked polygons, elevation —
  from `.poly.xml` or a 3D world) drops in behind the same "navigable region + obstacles"
  interface. Out of scope now; the fake version proves the *movement* using only road data.

## 4. Per-actor state machine

```
 Organized ─(external: set panic params)─► Flee  (aggressive organized lane/sublane driving, flee route)
 (normal route)                               │  nowhere to go in organized traffic (boxed in)
                                              ▼
                                            Orca   (core escalates: free-space flee within road+vicinity,
                                                    cars as obstacles)
                                              │  fully boxed in / reaches known-world edge
                                              ▼
                                           Abandon (car → static road obstacle; occupants → pedestrians)
                                              │
                                              ▼
                                     PedestrianFlee (ORCA)  toward the away-edge; block at the hard edge
                                              │  reaches away-edge
                                              ▼
                                           Escaped (leaves the simulated area)
```

Switch semantics — **resolved** (R2): the panic switch does **not** jump straight to ORCA. It sets
Flee params (aggressive organized driving); the core only escalates to **Orca** when that vehicle
has **nowhere to go** in organized traffic. All in the driving core.

## 5. Believability / correctness bar

- **Hard invariants:** panic **absent** ⇒ byte-identical to today (hash unchanged); no
  interpenetration (vehicles/pedestrians) beyond ε; no actor crosses the known-world hard edge;
  deterministic runs.
- **Behavioural targets:** panic front propagates outward, never teleports; distant traffic stays
  organized/jammed and unaware; jam → abandonment → foot-exodus cascade **emerges**; vehicles pile
  at the road edge; the evacuation drains toward the away-edges.
- **Acceptance = the viz** (organized traffic, the incident marker, panic front / fear overlay,
  ORCA vehicles, abandoned-car obstacles, fleeing pedestrians, the known-world edge), backed by
  the invariants.

## 6. Phased roadmap (panic-first; each phase watchable)

1. **Spine** — non-sublane organized traffic; external incident + radius fear; per-vehicle switch;
   ORCA flee within road+vicinity; boxed-in → abandon (car obstacle + pedestrians); pedestrians
   flee to the away-edge; the fake-navmesh region + hard edge. *Done:* full transition plays once.
2. **Panic as local information** — contagion + LoS + jam-unease; distant traffic oblivious.
3. **Richer vicinity / shaped off-road** — the shaped NH free-space vehicle model for on-shoulder
   flee; better fake-navmesh buffer; pedestrians using sidewalks/grass distinctly.
4. **Sublane realism (optional, separate)** — filter-to-front at lights in the organized phase.
5. **Scale** — hundreds → thousands; spatial indexing; far side stays jammed and unaware.

## 7. Open decisions (pin at Phase-1 kickoff)

- **Pedestrian ownership.** Following the same core/external split as vehicles: does the driving
  core also own **PedestrianFlee** movement (ORCA in the known world), with the external layer only
  *triggering* the driver→pedestrian conversion (e.g. a per-car "occupants bail out" input)? This
  doc assumes **yes** (pedestrians are just another ORCA agent class in the core's known world;
  external only flips the trigger) — confirm, since earlier notes leaned toward external steering.
- The exact per-vehicle **parameter surface** the core exposes for the external layer to set:
  `aggressiveness` (→ impatience / assertiveness / gap / speed-factor / right-of-way relaxation),
  `mode`, flee route. Which map to existing SUMO-port params vs. small additive opt-in fields.
- Vicinity buffer width; how the hard edge is represented (buffered-polygon boundary as an ORCA
  obstacle loop vs. an explicit fence).
- Away-direction flee-goal from the road graph; what counts as an away-edge "escape".
- Fear constants (θ_panic, decay, contagion kernel) — tuned against the viz.

## 8. Relationship to other docs

- `INDIA-TRAFFIC.md` — shaped / non-holonomic / ORCA **machinery** (`Sim.Core.Mixed`,
  `ShapedVoSolver`, bicycle model): reusable substrate for panic movement, not the headline.
- `LANELESS-DIRECTION.md` — the open-space ORCA regime + cross-regime bridge the seam builds on.
- `SUMOSHARP-DEADRECKONING.md` (NuGet branch) — the `DrModel` seam boundary detection rides.
