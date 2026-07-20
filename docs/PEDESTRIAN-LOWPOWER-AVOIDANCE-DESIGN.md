# PEDESTRIAN-LOWPOWER-AVOIDANCE-DESIGN.md — low-power peds must stop passing through each other (PROBLEM + directions)

**Status: OPEN task — problem recorded, needs a design + implementation.** Priority: **high** (a visible,
pervasive realism failure). This is a problem statement and a survey of candidate directions, not an agreed
design yet. Tracked in `PEDESTRIAN-TRACKER.md` as **PED-REALISM-1**.

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
