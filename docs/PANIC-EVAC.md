# PANIC-EVAC.md — urban panic / evacuation from a security incident

## 0. What this simulates (the product)

A localized **security incident** (bomb, shooting, armed strike) at a point in a city, and the
**panic evacuation** that follows: organized traffic near the incident breaks down, drivers try
to flee, streets **block solid**, and once boxed in, people **abandon their cars and flee on
foot** — off the road, into buildings, down side-streets away from the threat. The dramatic,
chaotic evacuation is the headline; the *organized* traffic before it is just the backdrop the
panic erupts from.

Design principle that shapes everything: **panic is a local, propagating information process.**
Nobody receives a global "flee" signal. An agent reacts only to what it can sense — seeing the
threat, seeing others panic, or hitting the wall of a jam. Distant drivers sit in inexplicable
congestion with no idea why. Panic (and its traffic-jam shadow) spreads outward from the
incident; it is never broadcast.

## 1. Two regimes + a transition (each tool doing what it is good at)

| Phase | Engine (existing) | Why it fits |
|---|---|---|
| Organized traffic (backdrop) | lane + **sublane** (`Engine`, `_sublane`, SUMO-parity) | Proven, coordinated (junction right-of-way), parity-tracked. "Good enough" — no Indian-realism investment needed. |
| **Panic on foot** | open-space **ORCA crowd** (`OrcaCrowd`) | RVO's canonical use: dense free-space evacuation, off-road, any direction. Its "gridlock/jostle" behaviour is *desired* here. |
| **Panic in vehicle, off-road** | shaped non-holonomic free-space vehicle (`Sim.Core.Mixed`, later phase) | Erratic curb-mounting / cutting across open ground is *correct* in panic — the module finally in its right context. |
| The mess between | **abandoned cars = static obstacles** (`ObstacleStore`) + the **cross-regime bridge** (`CrossRegimeCoupling` / `WorldDisc`) | Already built. Abandoned cars block the road further → cascade. |

Everything that was a *bug* in the earlier free-space "Indian junction" experiment is a *feature*
here: gridlock is what blocked streets look like; chaotic jostling is panic; foot crowds in a
dense mess are exactly what ORCA was built for. We stop fighting the free-space failure modes and
start using them.

## 2. Per-agent panic state machine

Each actor (a driver + vehicle, later also standalone pedestrians) moves through:

```
        threat sensed / contagion / jam-wall
 Driving ───────────────────────────────────► PanicDriving
 (sublane, normal route)                       (reroute AWAY from threat, aggressive params,
                                                will leave the road — off-road, later phase)
                                                        │
                                     stuck (v≈0) for T seconds AND high fear
                                                        ▼
                                                    Abandon
                                        (vehicle -> static obstacle in ObstacleStore;
                                         a pedestrian agent spawns at that spot)
                                                        │
                                                        ▼
                                                     OnFoot
                                   (ORCA disc; flee to nearest off-road haven / away-street /
                                    building; threads between abandoned cars + walls + people)
```

- **Driving → PanicDriving:** triggered by the fear model (§3). Reroute: goal becomes a flee
  target *away* from the threat (nearest network exit in the away-direction), not the original
  destination. Panic driving relaxes discipline (higher `maxSpeedLat`, smaller `minGapLat`, runs
  gaps it normally wouldn't) — all via the *existing* sublane vType fields, just aggressive
  values. It stays on roads until Phase 3 grants off-road.
- **PanicDriving → Abandon:** speed below a threshold for `T_abandon` seconds while fear is high.
  The vehicle is removed from the lane engine and registered as a **static obstacle** (it keeps
  blocking the road — the realistic cascade that turns a jam into total gridlock). A pedestrian
  disc is spawned into the `OrcaCrowd` at the vehicle's position with the driver's flee intent.
- **OnFoot:** an ORCA agent. Goal = nearest **haven**: an off-road open polygon, a building
  entrance, or a side-street mouth leading away from the threat. People are not lane-bound, so
  ORCA free-space navigation (around abandoned-car obstacles, walls, and each other) is exactly
  right — and the crush/queue behaviour is realistic.

Distant, unaware actors never leave `Driving`; they simply experience the downstream jam.

## 3. Fear & contagion model (local information only)

A scalar **fear** per agent, in [0, 1], updated each step from purely local signals:

- **Direct** — proximity to the incident, optionally gated by line-of-sight (a wall blocks the
  sightline of a blast/shooter): `fear_direct = g(distance, LoS)`, strong and immediate near the
  source, decaying with distance.
- **Contagion (social)** — you catch panic from neighbours: if enough nearby agents are already
  panicking (a count/kernel over the neighbourhood), fear rises. This is what makes panic spread
  *faster than the physical threat* and ripple outward through the crowd.
- **Jam signal** — being stopped in dense traffic for a while nudges fear up (unease), but on its
  own never reaches panic — so a routine jam far away does *not* trigger flight (matches the
  "distant drivers stay put" requirement).

Panic when `fear > θ_panic`; it decays slowly when local signals subside. Deterministic — every
signal is a function of frozen neighbour state; any per-agent tie-break is hashed (no RNG, no
wall-clock), consistent with the rest of the engine.

## 4. Seams it reuses (and the little it adds)

Reused, unchanged (all already on the branch; issue #4 freeze intact):
- **Sublane engine** — the organized backdrop and on-road panic driving.
- **`OrcaCrowd`** — the on-foot crowd (obstacles, `MaxNeighbours`, spatial hash all apply).
- **`CrossRegimeCoupling` / `WorldDisc` / `ICrowdFootprintSource`** — pedestrians and vehicles
  mutually avoid across the two engines (a pedestrian weaving past a still-rolling car).
- **`ObstacleStore`** — abandoned cars become obstacles for both regimes.
- **`DrModel`** — `LaneArc` (driving), `FreeKinematic` (fleeing crowd), `Stationary` (abandoned /
  jammed). The DR seam (issue #3) already anticipated exactly this split.
- **`Sim.Core.Mixed`** (shaped NH vehicle) — the off-road panic-vehicle model (Phase 3).

New, additive, parity-exempt (never on the golden path; hash `909605E965BFFE59` untouched):
- A panic **orchestrator** (`Sim.Core.Evac`) that owns the per-agent state machine, the fear
  field, the abandonment event, and the pedestrian spawn — coordinating the lane engine and the
  crowd each step. Opt-in; absent ⇒ the engine behaves exactly as today.
- A **vehicle→pedestrian** lifecycle transition (abandonment) and a per-agent **panic state** for
  the read/rendering surface.
- Integration hooks the orchestrator needs from the engine (reroute a vehicle to a flee goal;
  remove a vehicle and hand back its pose): confirm which exist vs. need a small additive,
  opt-in API — to be pinned in Phase 1, not assumed here.

## 5. Correctness / believability bar

Parity-exempt (no SUMO golden for panic), judged like the rest of the behavioural layer:
- **Hard invariants:** no interpenetration beyond ε (vehicles and pedestrians); no agent leaves
  the drivable/walkable surface except where the model explicitly allows (off-road flight);
  deterministic (bit-identical runs); the organized backdrop with panic **off** is byte-identical
  to the current engine (hash unchanged).
- **Behavioural targets (eyeball + stats):** panic front propagates outward from the incident and
  does not teleport; distant traffic stays in `Driving`; jam → abandonment → foot-exodus cascade
  emerges rather than being scripted; evacuation drains over time.
- **Acceptance test = the viz.** The replay artifact (cars, pedestrians, abandoned-car obstacles,
  a marked incident, a fear/heat overlay) is the sign-off, backed by the invariants.

## 6. Phased roadmap (each phase a watchable milestone; panic-first)

**Phase 1 — the spine (minimal, end-to-end).** Small hand-built network; sublane traffic; one
threat; radius fear; panicked cars reroute away (on-road) and jam; stuck+afraid ⇒ abandon (car →
obstacle, pedestrian spawns); pedestrians flee on foot (ORCA) to the map edge, threading between
stuck cars. Both regimes rendered together.
*Done:* the full transition plays once, coherent, in the viz.

**Phase 2 — panic as local information.** Contagion + distance/LoS fear + jam-unease; distant
traffic stays oblivious.
*Done:* panic visibly ripples outward; the far edge does not flee.

**Phase 3 — off-road escape.** Panicked cars leave the roadway (shaped NH free-space model);
pedestrians flee into buildings / off-road polygons / side-streets; free-space navigation across
the abandoned-car obstacle field.
*Done:* the "abandon the road, push through anywhere" mess.

**Phase 4 — organized-traffic realism (lower priority).** Real multi-street network + junction
right-of-way (port from `/sumo/` as needed) so the backdrop reads like a real city. Only invested
in when a phase demands it.

**Phase 5 — scale.** Hundreds → thousands as panic spreads; spatial indexing (obstacle grid /
KD-tree + agent hash). A far-side that never learns of the incident just stays jammed.

## 7. Relationship to the existing docs

- `INDIA-TRAFFIC.md` — the shaped-footprint / non-holonomic / ORCA *machinery* (`Sim.Core.Mixed`,
  `ShapedVoSolver`, the bicycle model). It is the **substrate** this evacuation model reuses, not
  the headline; dense mixed traffic per se is now a low-priority backdrop.
- `LANELESS-DIRECTION.md` — the two-regime lane/open-space foundation and the cross-regime bridge.
- `SUMOSHARP-DEADRECKONING.md` (NuGet branch) — the `DrModel` regime seam this transition rides.

## 8. Open decisions (to pin at Phase 1 kickoff)

- Exact engine reroute / despawn hooks (existing vs. small additive opt-in API).
- Haven representation for pedestrians (off-road polygons / building entrances / away-street
  mouths) and how the flee-goal is chosen from the away-direction.
- Fear model constants (θ_panic, decay, contagion kernel) — tuned against the viz, not fixed here.
