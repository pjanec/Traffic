# PEDESTRIAN-LOWPOWER-AVOIDANCE-DESIGN.md — low-power peds must stop passing through each other (PROBLEM + directions)

**Status: mechanism validated in a single-corridor prototype (§7); production seam requirements captured
(§8); not yet promoted into the real multi-segment low-power motion path.** Priority: **high** (a visible,
pervasive realism failure). §1–6 are the original problem + directions; §7 is the validated `LateralWeave`
prototype; §8 is what the real product must add (route continuity, endpoint anchoring, junction hand-off).
Tracked in `PEDESTRIAN-TRACKER.md` as **PED-REALISM-1**.

## 1. The problem (observed)

In the shared sub-area replay, pedestrians on a sidewalk walking in **opposite directions pass straight
through each other**, and everyone walks exactly on the **sidewalk centerline**. With thousands of
pedestrians this is unacceptable — it reads as obviously fake.

## 2. Why it happens (root cause — confirmed in code)

Peds have two LOD regimes:

- **Low-power (`Sim.Pedestrians/Lod/PathArcMotion.cs`, and the lively `ActivityTimeline`)** — a ped's pose is
  a **pure function of `(path, startTime, speed, now)`**: dead-reckoning along its precomputed route
  centerline, with *"no neighbour state, no System.Random, no hidden mutable state"* (the file says so). A
  low-power ped **cannot see other peds**, so it never avoids and never leaves the centerline.
- **High-power (`OrcaCrowd`, FreeKinematic)** — full ORCA/RVO reciprocal avoidance. This is the **only** regime
  that avoids, and it is expensive; it is meant for the *on-camera* subset, promoted via `InterestField`.

So low-power peds interpenetrate **by design**, and the `--ped-subarea-fcd` recorder runs the whole crowd
low-power (no interest sources registered), so nothing ever avoids. High-power ORCA is not a scalable answer
for "thousands of peds everywhere" — that is exactly the population low-power exists to serve cheaply.

**Therefore the fix must live in the low-power path**: make low-power motion itself avoid pass-throughs to a
*reachable, performant* degree, without turning everyone into an ORCA agent.

## 3. The load-bearing constraint (must not break)

Low-power's whole identity is the **server == IG reconstruction** property: the server broadcasts a ped's
path/timeline **once**, and the headless IG reconstructs its pose by calling the *same* `PathArcMotion`/
`ActivityTimeline` function — no per-frame state, no neighbour data on the wire. Any fix must **preserve this
purity**: a low-power ped's pose may depend on its **own** route + its **own** deterministic per-ped data, but
**not on live neighbour positions** (which the IG does not have). This rules out naive neighbour-reactive
nudging in low-power — that would reintroduce the O(n) neighbour state low-power exists to avoid, and would
break reconstruction (the IG would need every neighbour to reproduce one ped).

Also mandatory (CLAUDE.md): **deterministic, no `System.Random`** (per-ped seeded `VehicleRng` only); additive
/ inert-default so committed goldens stay bit-identical; performant at thousands of peds.

## 4. Candidate directions (to be evaluated in a real design)

Ordered by how well they fit the purity constraint:

1. **Direction-based lateral offset ("keep to one side") — RECOMMENDED starting point.** Offset each ped from
   the centerline by a signed lateral amount chosen from its **travel direction along each edge** (e.g. keep
   right), like SUMO's own sidewalk-side behaviour. Opposing flows separate onto opposite halves of the walk
   → head-on centerline pass-throughs disappear. Pose stays a **pure function of the ped's own route** (offset
   the polyline laterally by a per-direction constant, or evaluate offset at sample time), so server == IG and
   O(1)/ped are both preserved. Does **not** fix same-direction stacking or dense crowding, but kills the most
   jarring failure (head-on interpenetration) cheaply.
2. **Deterministic lateral slotting by width.** Distribute peds across the sidewalk **width** into N lateral
   lanes, slotted by `(direction, per-ped seed)`. Reduces same-direction stacking too (peds fan out across the
   walk instead of all riding the centerline). Still a pure function of the ped's own data. Needs the sidewalk
   width (the bake has it: `PedLane.Width`, buffered strip) and care at junctions/crossings where width/side
   are ill-defined.
3. **Precomputed low-power separation field / phase offsets.** A per-edge flow/density model that spreads peds
   in space and time deterministically. More believable under load; more design.
4. **Selective promotion in hotspots (complement, not a fix).** Promote to ORCA only where density is high
   (not just on-camera) so genuinely congested spots avoid properly, while the bulk stays low-power with (1)/(2).
   Doesn't scale to "thousands everywhere" but is a good top-up for pinch points.
5. **NOT recommended: neighbour-reactive nudging in low-power** — breaks the server==IG purity (§3) and the
   O(1) cost model. Only viable if reconstruction is redesigned, which is a much bigger change.

## 5. Acceptance (draft — refine in the design)

- On a straight sidewalk with two opposing streams, low-power peds **do not overlap** (centre-to-centre
  distance ≥ ~1 ped diameter for the head-on case), rendered from the recorder with no interest sources.
- Cost stays O(1)/ped/step (no neighbour queries in the low-power path); thousands of peds remain performant.
- Deterministic + server==IG preserved (a low-power ped reconstructs identically from its one-time broadcast).
- Additive/inert-default: committed ped goldens bit-identical with the feature off; the recorder gains a way
  to show the improved behaviour.
- The `--ped-subarea-fcd` replay visibly shows opposing sidewalk flows parting instead of interpenetrating.

## 6. Notes / open questions

- Interaction with `PathArc` **wire format** (`FrameCodec` KindPathArc): a lateral offset that is a pure
  function of the route can be applied render-side or baked into the broadcast path — decide which keeps the
  wire minimal and the IG reconstruction exact.
- Crossings/walkingAreas: "keep right" is well-defined on a linear sidewalk but not on an open junction area —
  decide behaviour there (likely: offset only on sidewalk legs, centerline through areas, or a slot that
  survives the crossing).
- This is orthogonal to appearance-legitimacy (P8-2) and density (P8-4); it is a *motion realism* axis.

## 7. Prototype 1 — validated mechanism (single corridor)

Built as `Sim.Pedestrians.Lod.LateralWeave` + the `--ped-weave-csv` trails harness. Validated visually over
two opposing streams on a straight corridor:
- `Offset(s, seed, room)` — a keep-right-biased **lane plan** (seeded target per ~30 m, smooth transition),
  **per-ped phase** (lane changes desynchronised), **micro-wander** (no dead-straight lines), `MinFrac=0` so
  peds fill the width from the centreline outward (no dead middle channel). Pure, O(1)/sample, deterministic.
- `CenterShift(x, now, globalSeed)` — a **shared moving interface** (dividing line) between the two
  counterflows: a spatiotemporal field from ONE global seed, so it meanders along the corridor **and** drifts
  in time; each stream keeps to its own side (squeeze/widen breathing). A shared deterministic field
  (`PLANNING-INTENTS` §3) → both server and IG compute the identical `c(x, now)`; no per-ped/neighbour state.

server==IG holds because every term is a pure function of the ped's own route+seed and the global-seed field,
evaluated at the reconstruction `now`.

## 8. Production requirements the prototype does NOT cover (CARRY FORWARD)

The prototype tapers the offset **and** the interface to 0 at the corridor ends purely so a single straight
corridor's spawn/arrival land cleanly. **That is a prototype convenience and must NOT survive into the real
product** — a route is a chain of segments, and pinning to centreline at every join would funnel everyone to
the middle at each junction (robotic). Required for production:

1. **Weave continuously along the WHOLE ROUTE.** Arc-length `s` runs over the whole O→D route; the endpoint
   taper/anchor applies ONLY at the true origin and destination, never at intermediate segment joins. Across
   a join within a corridor the lateral offset is continuous (same value leaving segment A = entering B).
2. **Anchor endpoints to the ACTUAL spawn/arrival lateral position, not centreline.** The origin/destination
   are real points (a building doorway on the building side, a fringe/POI point) with their own lateral
   position; the lead-in/out blends to THAT, not to y=0.
3. **Smooth junction / walkingArea hand-off.** Where the route crosses an open area (a junction/crossing)
   the "sides"/interface are undefined, so the offset must **blend smoothly through** the area (relax toward
   a neutral, re-establish on the next edge) — a smooth interpolation, never a snap to centreline. A ped's
   lateral "slot" ideally survives the crossing.
4. **Per-edge (per-corridor) shared interface.** `CenterShift` is a property of an edge/corridor of collinear
   edges, not the whole route: `c_edge(x_along_edge, now)`, seeded per edge (derivable from `edge_fields`).
   A ped traversing a corridor shares one interface; at a turn onto a new corridor it transitions smoothly
   between the two interfaces' frames. Still IG-reconstructable — the IG has the route (broadcast), the
   broadcast-once `edge_fields`, and `now`.

These are the seam requirements for promoting `LateralWeave` from the single-corridor prototype into the
real low-power motion path (route-arc-length continuity + junction blend + endpoint anchoring). The next
prototype should be a **multi-segment route with a bend**, showing the offset flow continuously across the
join and anchor only at the true O/D. *(Built: `--ped-weave-bend-csv`; continuity + hand-off confirmed.)*

## 9. Composing the weave with live-behaviors (talk / enter-exit / restaurant / car / cross-the-stream)

The live-behaviors (`ActivityTimeline`: Walk/Pause/Dwell/Interact, `SocialPlanner`, `LotCoupling`) compose
with the weave through **one unifying abstraction — the per-ped lateral profile**. Normally the profile is
the weave's keep-right lane plan; a live-behavior *overrides the profile's lateral target* for the duration
of an activity, then blends back. The split that matters:

### 9a. Deterministic displacements — STAY LOW-POWER (most behaviors)
Anything that moves the ped to the **edge / off-stream** on **its own side** deterministically, without
traversing the counterflow, is a lateral-target override + an activity segment. Pure, O(1), server==IG — no
ORCA:
- **Check phone / rest:** Walk → *drift to the kerb edge* (lateral target = ±MaxFrac·halfWidth) → Pause →
  drift back → Walk. Stepping aside FIRST is what keeps the pauser from being an in-flow obstacle.
- **Enter / exit building:** Walk → *drift to the doorway lateral position* (a POI on the building side,
  §8-2 anchoring) → Dwell(hidden) inside / emerge. `ActivityTimeline` already models hidden dwell.
- **Stop at restaurant / shop on the SAME side:** route detour to the venue POI + Dwell (seated visible or
  inside hidden). A venue-side lateral target.
- **Meet & talk:** `SocialPlanner` ALREADY does this low-power + server==IG — both step aside to a bisector
  spot on their own side, Interact, resume. It is a lateral-target override by another name.
- **Board / alight a car at the kerb on its own side:** walk to the curbside car POI, Dwell/vanish (board)
  or appear (alight). `LotCoupling` (POC-6) already handles board/alight; the curbside case is the same with
  a kerb lateral target.

All of these are **deterministic** (the target is a planned POI/step-aside, not a reaction), so the low-power
pose stays a pure function of (route, seed, activity plan, weave field, now) — server==IG preserved, zero
neighbour state.

### 9b. Reactive stream-crossing — TEMPORARY ORCA, then restore (the minority, your instinct)
**Crossing the counterflow** — a shop/restaurant/car on the FAR side of the sidewalk — cannot be
deterministic: a planned straight crossing path would pass *through* the oncoming stream. This genuinely
needs reciprocal avoidance, so it is exactly where a low-power ped **temporarily promotes to ORCA**:

1. **Promote (planned, not reactive):** as the ped approaches its scheduled crossing point it is pinned
   high-power (`SetForcedHighPower` / an interest source at the crossing). It joins the ORCA crowd and
   reactively threads through the counterflow; on the wire it becomes a DR-gated high-power stream (P3-2/3-3),
   which the IG follows — the reconstruction already absorbs the promote with no pop.
2. **ORCA excursion:** reactive, path-dependent, NOT a pure function -> that is fine *because it is broadcast*
   as high-power for the duration.
3. **Restore the deterministic flow (the load-bearing step):** when the ped reaches the far side, **demote by
   broadcasting a fresh low-power leg** — a new PathArc/route leg + weave seed + startTime, anchored at the
   ped's current position projected back onto its route, with the weave's lateral profile blended
   (smoothstep) from the ped's actual arrival lateral position to the lane plan. Because the fresh leg is
   broadcast, server==IG is **restored** from the demote instant on: the IG stops following samples and
   resumes the pure-function reconstruction from the new leg. This reuses the existing reroute-leg
   re-broadcast (P2-2) + the P3-3 demote handling.

### 9b-bis. Per-ped speed + emergent same-direction passing (cheap realism)
Uniform walking speed is as strong a "fake crowd" tell as the centreline was. Give each ped a **seeded
preferred speed** (~0.9–1.7 m/s) — it is already the `PathArc` leg's `speed` parameter, on the wire, so it
is deterministic, O(1), zero extra bytes, server==IG. Two payoffs: the crowd stops moving in lockstep
(uneven longitudinal spacing), and **overtaking emerges for free** — because the weave already spreads
same-direction peds across different lateral tracks, a faster ped simply passes a slower one on a
neighbouring track, no reactivity. The residual (a faster ped catching a slower one on the *same* track)
is the same accepted "reachable level" as the head-on residual — rare, and handled by ORCA hotspots for the
worst cases. *(Built into Prototype C.)*

### 9c. Synergy with the LOD / density design
The stream-crossings are precisely the **areas of interest / density hotspots** where ORCA is already
promoted (`PLANNING-INTENTS` lever 3). Concentrating cross-stream behaviors (and road crossings) in those
zones means the reactive ORCA and the crossing conflicts share the same small high-power budget, while the
bulk everywhere else stays low-power weave. So the composition is: **deterministic weave everywhere +
deterministic activity displacements (still low-power) + temporary ORCA only for the genuinely-reactive
stream-crossings, ideally inside the already-promoted zones.**

### 9c-bis. Measured density ceiling of the pure weave (Prototype 1b)
The pure deterministic weave was packed to failure to find where it stops looking clean, measured (not
claimed) by counting per-frame pairs whose centres fall within 2·r = 0.5 m on a 60 m × 4 m counterflow
corridor (`--ped-weave-density-csv`, steady-state window):

| spawnEvery | peds (avg) | density | overlaps/frame | cross-stream | same-stream | overlap/ped |
|-----------:|-----------:|--------:|---------------:|-------------:|------------:|------------:|
| 2.0 s | 47 | 0.20/m² | 3.5 | 0.6 | 2.9 | 7.5% |
| 1.2 s | 77 | 0.32/m² | 9.3 | 1.0 | 8.3 | 12% |
| 0.8 s | 120 | 0.50/m² | 23 | 2.9 | 20 | 19% |
| 0.5 s | 196 | 0.82/m² | 73 | 8.6 | 64 | 37% |
| 0.35 s | 280 | 1.16/m² | 152 | 19 | 133 | 54% |

Two findings, both load-bearing for the design:

1. **The weave's actual job — separating opposing flows — holds at ALL densities.** Cross-stream overlaps
   stay a small minority (≈12% of overlaps even at a crushing 1.16/m²), and they are confined to the shared
   interface where opposing peds legitimately brush shoulders (side-by-side, not centre-through-centre — the
   0.5 m metric overcounts these). The keep-right sign guarantees east ≤ c ≤ west by construction, so the two
   streams provably never interpenetrate. The 279-ped snapshot shows this cleanly: red stays below, blue above.
2. **The weave does NOT resolve same-stream conflicts, and that is the ceiling.** Same-stream overlaps
   dominate and climb steeply with density: peds sharing a direction but seeded to different preferred speeds
   overtake and pass through each other, because `Offset` is a pure per-ped function with **no minimum-
   separation enforcement** — by design (that reactivity is ORCA's job, not a pure field's).

**Operating range:** the pure weave reads as visually clean up to ≈0.3/m² (spawnEvery ≈1.2 s, ~77 peds on
this corridor: 12% overlap, most of it the interface brush). Beyond ≈0.5/m² same-stream overtakes become
frequent enough to notice. This is exactly the hand-off the LOD design already anticipates (§9c): the weave
delivers free, deterministic, IG-reconstructable crowd up to moderate density, and the **density hotspots
that exceed it are precisely the zones ORCA is already promoted in.** The number to remember: ~0.3/m² is the
per-corridor budget where "low-power weave, zero ORCA" still looks right.

### 9d. Open decisions
- The lateral-profile override API: how an `ActivityTimeline` segment carries its lateral target + the
  blend-in/out lengths, and how that composes with `LateralWeave.Offset` (override vs add).
- The demote re-anchor: projecting the post-ORCA position onto the route + emitting the fresh leg + the
  resume-lateral blend value (needs a small wire field on the leg).
- Promote trigger timing for a scheduled crossing (how far ahead), and whether the crossing target is a POI
  the demand plan assigned.
- Whether a genuine in-flow stop (no step-aside possible) promotes the LOCAL region so followers avoid it, or
  is simply disallowed by always stepping aside first.

## 10. Design proposals for the §9d open decisions

### 10.1 The lateral-profile abstraction (concrete)
The low-power pose is one composition:

```
pos(s, now) = centreline(s) + rightNormal(s) · lateral(s, now)
lateral(s, now) =
    default (in-flow):        c(a, now) ± weaveMag(s, seed, room)          // the weave, relative to the moving interface
    within an activity window: blend( weaveLateral , activityTarget , smoothstep over lead-in/out )
```

An `ActivityTimeline` segment gains an optional **lateral target**:
`{ Mode: None | AbsoluteOffset | Side(kerb|building) | PoiAnchored, Value, LeadInMeters, LeadOutMeters }`.
The pose evaluator, at arc-length `s`, finds the active segment's target (if any) and smoothsteps between the
weave lateral and that **absolute** corridor-lateral target (absolute, because an edge/doorway/step-aside
position is fixed, not relative to the moving interface). Everything is on the broadcast timeline, so the IG
reconstructs the identical profile. This ONE mechanism expresses the weave, `SocialPlanner`'s step-aside, the
doorway/venue/kerb approaches, and the phone-check drift — they differ only in `Value`/`Mode`.

### 10.2 Promote → ORCA → demote for stream-crossing (concrete)
- **Trigger (deterministic, server-side):** the demand plan assigns a far-side POI; the router marks the
  crossing arc-length `s_x`. At `s_x − leadPromote` the server pins the ped high-power (`SetForcedHighPower`
  / a crossing interest source). Scheduled, not reactive.
- **Excursion:** ORCA steers the ped toward the far-side entry point while reciprocally avoiding the
  counterflow; broadcast as a DR-gated high-power stream (P3-2/3-3), which the IG follows.
- **Demote / restore (the load-bearing step):** on arrival at the far side, the server (a) projects the ped's
  position onto its far-side route continuation → resume arc-length `s_r`; (b) emits a **fresh low-power
  PathArc leg** from `s_r` (weave seed + startTime) carrying a **resume-lateral `l_r`** = the ped's current
  projected lateral offset; (c) clears the high-power pin. The IG gets the demote DR-switch + fresh leg +
  `l_r`; P3-3 absorbs the switch; the weave blends from `l_r` to the lane plan over `LeadIn`. **server==IG is
  exact again from the demote instant.**
- **Wire delta:** the fresh-leg record gains ONE scalar (`l_r`); the promote/demote ride the existing
  lifecycle + DR-switch records; the excursion reuses the existing high-power sample stream. No new topic.

### 10.2-bis The BYSTANDER restore (the hard half §10.2 hid)
§10.2 restores the ped that **chose** to leave the flow (the crosser): its post-excursion leg is *planned*, so
"restore" is just re-anchoring an agent we control onto a fresh route. The harder, distinct case is a
**bystander**: a ped that was walking a perfectly deterministic weave and got **involuntarily deflected** by
someone else's ORCA maneuver, and must return to **its own** deterministic track — the same seeded lane it was
already on, not a new route. This is what "restore the pre-programmed deterministic flow" (the original ask)
actually means for the majority of promoted peds, and it differs in three ways:

1. **It returns to its OWN absolute-arc lane plan, not a restarted one.** The resume primitive therefore splits
   two coordinates that coincide for the crosser: `interiorArc` (the ped's absolute arc along its own route —
   the lane plan is evaluated here, so it slots back onto the exact seeded track) and `blendDist` (distance
   since demote — the `l_r`→weave blend runs over this). `LateralWeave.OffsetWithResumeOnRoute(interiorArc,
   blendDist, …)` expresses both; `OffsetWithResume` is the crosser special case `interiorArc==blendDist`.
2. **The deterministic schedule kept advancing while the ped was shoved**, so at demote the ped is off-track in
   BOTH lateral and arc. The policy (chosen): **re-anchor** — emit a fresh leg from the ped's *actual* (delayed,
   projected) arc `s_r` with `l_r`, on its own route+seed. The excursion's lost progress is **permanently
   absorbed** into a re-based anchor (the ped stays a fixed arc behind where it "would have been"). This is
   `O(1)`, exact-after, and needs no catch-up controller. The rejected alternative (**converge-to-schedule** —
   keep the original schedule and steer the ped to catch up to `pose_det(t)`) has no permanent shift but keeps
   the ped off the pure track (more broadcast) and needs a catch-up controller; only worth it if a downstream
   constraint requires the ped to hit a point on the original schedule, which ambient peds don't have.
3. **It is not free.** Every involuntarily-perturbed bystander must be **broadcast** for the span it is
   off-track (the IG can't predict an ORCA shove), and its demote costs the same one-scalar (`l_r`) fresh-leg
   record as the crosser. So the wire cost of a hotspot ≈ (crossers + shoved bystanders) × excursion-span — a
   direct argument for keeping promoted zones small (§9c) and crossings sparse (the density work).

**Measured (Prototype D2, `--ped-weave-cross2-csv`).** The crosser is aimed straight into the moving stream so
ORCA genuinely deflects a specific bystander B (chosen as the cohort ped shoved *most* off its ghost — the one
that really had to react, not a hand-picked winner). B is deflected **2.14 m** off its unperturbed ghost
(mostly longitudinal — ORCA slowed it — plus ~0.5 m lateral), then re-anchors onto its own weave:

| B span | max ‖server − IG‖ | |
|---|---|---|
| before promote | **0** (exact) | pure weave |
| during the shove | **0.10 m** | broadcast, DR-interpolated (< 0.25 m) |
| after demote | **0** (exact) | fresh leg on B's OWN route + `l_r` |

No-pop demote seam ‖Δ‖ = **3.6×10⁻¹⁵ m**. Honest cost, printed by the command: B rejoins its own lane track
but **permanently 2.1 m / 1.7 s behind its ghost**, and was broadcast for the 10.6 s it was off-track. This is
the concrete evidence that ORCA→deterministic is solved for the *reactive bystander*, not just the volunteer —
and an explicit accounting of what it costs.

### 10.3 The reconstruction-fidelity guarantee
Server==IG is **exact** (pure function) for every low-power ped and every low-power *phase* of a ped —
i.e. the whole 9a set and the before-promote / after-demote spans. It is **within render tolerance**
(≤ ~0.25 m, P3-3) only during an actual ORCA excursion, which is broadcast. Because excursions are the
minority (far-side crossings only) and concentrated in hotspots (10.4 / §9c), the exactly-reconstructed
low-power bulk dominates both the wire and the picture.

### 10.4 In-flow stop policy
Default: a stop ALWAYS steps aside first (9a), so it is never an in-flow obstacle and forces no neighbour
reaction. Fallback for a genuinely unavoidable in-flow stop: register a transient interest source at the
stopped ped so local followers promote, avoid, and demote when it clears — rare by construction.

## 11. Prototype roadmap (well-specified, success-criteria'd)

| # | Prototype | Exercises | Success criteria | De-risks | Deps |
|---|---|---|---|---|---|
| A | straight-corridor weave + moving interface *(BUILT — `--ped-weave-csv`)* | §7 | opposing flows separate; bands not lanes; centre populated; interface meanders in space+time | the core weave look | — |
| B | multi-segment route with a bend *(BUILT — `--ped-weave-bend-csv`)* | §8-1/§8-3 | offset flows continuously through the corner; anchors only at true O/D | route continuity + junction hand-off | — |
| C | **activity lateral-override (low-power)** | §9a, §10.1 | a ped drifts to the kerb → Pause(phone) → rejoins; a ped drifts to a doorway → hidden; neighbours unaffected; **server==IG exact** (bit-identical reconstruction) | the lateral-profile API; that most behaviours need NO ORCA | §10.1 API |
| D | **cross-stream ORCA excursion + restore** *(BUILT — `--ped-weave-cross-csv`)* | §9b, §10.2 | a low-power ped promotes, reactively crosses the counterflow to a far-side POI, demotes; resumes the deterministic weave **with no pop**; server==IG **exact before promote and after demote**, within-tolerance during | the promote/demote/re-anchor loop; the "restore" | §10.2 wire delta (`l_r`), P3-3 |
| E | **composed demo on the synthetic mesh** | §9 all + density | weave + all 9a behaviours + a few 9b crossings at the calibrated density read as a believable crowd; no pass-throughs in the low-power bulk; crossings avoid cleanly | the whole story end-to-end | SumoData synthetic mesh + `edge_fields.json` |

Sequencing: C is next and unblocked (pure low-power, on a clean box) — it proves the lateral-profile API and
that 9a needs no ORCA. D follows (the one genuinely-reactive case, end-to-end). E is the acceptance demo and
waits on the synthetic mesh. Each prototype must show its **server==IG** column literally (a reconstruction
diff), not just look right — the same "necessary, not sufficient" discipline as the navmesh witnesses.

### 11.1 Prototype D result (measured)
`--ped-weave-cross-csv` runs the whole loop on a 50 m corridor: an eastbound low-power ped weaving on the
south half is assigned a café POI on the north kerb; it promotes at arc 24 m, crosses the westbound
counterflow via the **real `OrcaCrowd` solver** (the local stream cohort is promoted with it and reciprocally
parts), reaches the café (≈9.4 s excursion), then demotes onto a fresh eastbound low-power leg that resumes
the weave on the north half via `LateralWeave.OffsetWithResume(l_r=1.85 m, leadIn=8 m)`. The command prints
the §11 server==IG column literally:

| span | max ‖server − IG‖ | why |
|---|---|---|
| before promote | **0** (exact) | pure `Offset` — IG recomputes bit-identically |
| during excursion | **0.10 m** | broadcast high-power samples (0.4 s), DR-interpolated — within the P3-3 ≤0.25 m render tolerance |
| after demote | **0** (exact) | fresh leg + the one `l_r` scalar — IG recomputes `OffsetWithResume` bit-identically |

And the two seams that de-risk the "no pop" claim:
- **promote seam** ‖Δ‖ = 3.6×10⁻¹⁵ m — ORCA is seeded at the exact weave pose.
- **demote seam** ‖Δ‖ = 0 m — `l_r` is *defined* as the ORCA arrival lateral, and `OffsetWithResume(sPrime=0)==l_r`
  by construction, so position is continuous to machine precision; the lane plan then blends in over `leadIn`.

The load-bearing restore math (`OffsetWithResume`, `OffsetInterior`, arrival-only taper) is a pure function in
`LateralWeave`, unit-pinned in `LateralWeaveTests` (no-pop at the seam; convergence to the pure weave after the
lead-in; arrival taper; determinism). This is the concrete evidence for §10.2/§10.3: the exactly-reconstructed
low-power spans dominate, and only the broadcast excursion is (bounded) approximate.
