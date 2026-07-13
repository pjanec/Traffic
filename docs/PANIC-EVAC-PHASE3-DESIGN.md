# PANIC-EVAC-PHASE3-DESIGN.md — vehicle Orca-mode / off-road push: HOW it works

Design (HOW) for **Phase 3**. WHAT = `PANIC-EVAC.md` **§6.3** ("cars push into the vicinity — shaped NH
free-space model — before blocking; richer fake-navmesh buffer") and **R2**'s third vehicle mode (the
Orca mode). Builds on the `Sim.Evac` layer + tick (`PANIC-EVAC-DESIGN.md`) and the fear field
(`PANIC-EVAC-PHASE2-DESIGN.md`). Tasks + tracker follow **after the architecture fork in §3 is signed
off** — that decision shapes everything downstream, so this doc is deliberately fork-first.

Goal in one line: when a panicked, fleeing car can no longer make progress on the lane network, instead
of instantly abandoning it (Phase-1 `blocked → pedestrian`), the car first **mounts the shoulder** — a
shaped, non-holonomic free-space push into the road+vicinity band — and only becomes a pedestrian when
even that wedges. This inserts an **Orca-push** stage into the lifecycle:

```
Organized ─► Flee (params) ─►[no lane progress]─► ORCA-PUSH (shaped free-space in the band)
                                                        │ [wedged in the band too]
                                                        ▼
                                                   Pedestrian (foot)   ← Phase-1 handoff, now downstream
```

---

## 1. What already exists (the substrate)

`Sim.Core.Mixed.MixedTrafficCrowd` (the India-traffic work, `INDIA-TRAFFIC.md`) is a **shaped,
non-holonomic, wall-aware free-space vehicle driver** — the vehicle analogue of the pedestrian
`OrcaCrowd`: `Add(VehicleClass, pos, goal, headingRad, …)`, `AddWall`/`AddBlock` (tiled shaped-VO
walls), `Step(dt)`, `Position/Heading/Velocity(i)`, `Deactivate(i)`, `SafetyMargin`, per-`VehicleClass`
`MinTurnRadius`/`Wheelbase` (kinematic-bicycle, rear-axle pivot), soft priority. It lives in `Sim.Core`
but is **off the parity/golden path** (no committed scenario uses it; determinism hash unaffected).

Phase 3 is largely a matter of *driving an individual fleeing car with this existing model* in the
vicinity band — not writing a new motion model.

---

## 2. The lifecycle change

Phase 1 `PostStep`: `blocked (DrModel.Stationary + dwell) && panicked → Convert to pedestrian`
(Despawn + abandoned-car obstacle + spawn pedestrian).

Phase 3 inserts a stage: `blocked && panicked → enter ORCA-PUSH` (the car leaves the lane engine and
becomes a shaped free-space mover in the band, still trying to flee away from the incident). A car in
Orca-push that itself stops making progress (its shaped-mover speed ≈ 0 for a dwell) is **wedged** →
*then* Convert to pedestrian (the same Phase-1 handoff, now sourced from the Orca-push stage rather than
the lane). A car that finds free-space progress keeps pushing toward an away-goal / the safe distance.

Everything else (fear field, flee params, pedestrian crowd, hard edge, escape) is unchanged.

---

## 3. THE ARCHITECTURE FORK (decide this first)

R2 says the Orca mode is "a genuinely separate mode … **The core owns it.** Selected by an explicit
`mode` flag, not by the param preset." There are two faithful readings, and they lead to very different
implementations:

### Option A — external shaped-mover handoff (RECOMMENDED)
The `Sim.Evac` layer transitions a blocked+panicked car into a **`MixedTrafficCrowd` of shaped NH
vehicles** (the band as walls), exactly mirroring how it already hands blocked cars to the pedestrian
`OrcaCrowd`. Mechanically: `Engine.Despawn(handle)` (car leaves the lane sim), then `mixedCrowd.Add(Car,
pose, awayGoal, heading)`; the evac tick advances the mixed crowd, feeds its car footprints to the
pedestrian crowd (and vice-versa) as it already does, and on wedge does the pedestrian handoff.
- **"Core owns it"** is honoured in the sense that the *motion model* (`Sim.Core.Mixed`, shaped NH ORCA)
  is core code. The *orchestration* (when to switch, region/goals) is external — consistent with Phases
  1–2, where the external layer owns transitions and the core owns motion/parity.
- **Parity: trivially safe.** The lane `Engine` gains nothing; a car in Orca-push has already been
  `Despawn`ed, exactly like today's pedestrian conversion. Hash cannot move by construction. No new
  Engine mode flag, no new lane-engine code path.
- **Reuse:** `MixedTrafficCrowd` is used as-is. Least new code, lowest risk.
- **Cost:** the Orca-push car is no longer a lane `VehicleHandle`; it is an evac-owned shaped mover
  (its own id space), like pedestrians. The viz renders it from the mixed crowd, not the lane read-buffer.

### Option B — true in-Engine vehicle Orca-mode
Add `Engine.SetVehicleMode(handle, VehicleMode.Orca)`; a flagged vehicle stops lane-following and is
integrated in world-space by an in-Engine shaped-NH solver, staying a live `VehicleHandle`.
- **Matches R2 literally** ("the core owns it", a mode flag, the vehicle keeps its identity).
- **Parity risk: real.** It adds a new branch inside the lane engine's per-vehicle update. It must be
  provably inert when no vehicle is flagged (default), but the review burden is much higher than A, and
  it entangles the parity core with the shaped-VO layer.
- **More code:** wiring world-space state, DR classification (an Orca vehicle becomes `FreeKinematic`,
  changing `RegimeOf` — today a vehicle is never FreeKinematic), read-buffer publishing of a non-lane
  vehicle, etc.

### Recommendation
**Option A.** It delivers the same visible behaviour (cars mounting the shoulder, shaped NH push, then
foot exodus), reuses proven core machinery, and keeps the parity guarantee free instead of hard-won. B's
only real advantage is literal fidelity to R2's wording and preserving the lane handle — neither is worth
entangling the parity core for a parity-exempt evacuation feature. If identity-preservation ever matters
(e.g. analytics that must track one entity lane→orca→foot), we can add a thin evac-side id that spans the
stages without touching the Engine.

**This doc's §4–§8 assume Option A. If you prefer B, I'll revise before writing tasks.**

---

## 4. Mechanism (Option A)

- **New evac component `VehicleMover`** wrapping a `Sim.Core.Mixed.MixedTrafficCrowd`, analogous to how
  the director already owns the pedestrian `OrcaCrowd`. Holds the band walls and the shaped cars.
- **The band (richer fake-navmesh buffer, §6.3 / R8).** Phase-1's navmesh was a single bounding-box hard
  edge for pedestrians. For Orca-push we want cars to push *along the road + shoulder*, not wander the
  whole box. `FakeNavMesh` gains a **road-band region**: lane/junction polygons buffered outward by the
  vicinity width, whose *complement* (beyond the shoulder) is walls fed to the `MixedTrafficCrowd`
  (`AddWall`/`AddBlock`). Minimal first cut: reuse the outer bounding wall + the abandoned cars + the
  jammed lane cars as shaped obstacles, so pushing happens into real gaps; the per-lane buffered band is
  the richer follow-up within this phase.
- **Transition in (`EvacDirector.PostStep`).** Replace `blocked && panicked → Convert(pedestrian)` with
  `blocked && panicked → EnterOrcaPush`: `Despawn`, `mixedCrowd.Add(VehicleClass.Car, pose, FleeGoalFor(pose),
  heading)`, record an evac-side OrcaPush entry. (Heading from the last `VehicleState.Angle`.)
- **Driving the push.** Each tick, set each Orca-push car's goal to the away-from-incident direction
  (like pedestrians), feed it the current obstacle set (other orca cars are mutual; abandoned cars +
  remaining lane cars as walls/discs; pedestrians as discs), and `mixedCrowd.Step(dt)` (sub-stepped like
  the pedestrian crowd for smoothness).
- **Wedge → pedestrian.** An Orca-push car whose shaped-mover speed stays below a small threshold for a
  dwell is **wedged**; then run the existing pedestrian conversion from its current pose (spawn
  pedestrian(s), leave the car as a static abandoned obstacle, `Deactivate` it in the mixed crowd).
- **Cross-avoidance.** Orca-push cars must appear as obstacles to (a) pedestrians — extend
  `FeedVehicleDiscsToPeds` to include them; and (b) each other — intrinsic to the `MixedTrafficCrowd`.
  Remaining lane vehicles vs orca cars: the band is off-lane, so first cut treats orca cars as static-ish
  discs to the lane engine only if needed (likely unnecessary for the demo).

---

## 5. Determinism & parity

- Parity: Option A adds no Engine code; a scenario with no Orca-push (every committed golden) is
  byte-identical. Hash `909605E965BFFE59` unmoved — a test.
- Determinism: `MixedTrafficCrowd.Step` is already deterministic (plan/execute, deterministic symmetry
  break, no RNG/wall-clock); the evac orchestration iterates fixed insertion order. A run is bit-identical
  — a test (fold Orca-push car poses + counts into the determinism signature).

---

## 6. Believability targets

- A blocked fleeing car visibly **noses off the lane into the shoulder/gap** (shaped, non-holonomic — no
  sideways teleport, no pivot-in-place; long vehicles turn wide) before giving up.
- Cars pile into the band and **wedge** there, then occupants bail on foot — a richer jam than Phase-1's
  instant abandonment.
- Still deterministic; pedestrians still never cross the hard edge; no shaped-car interpenetration
  (SAT/margin, already proven for `MixedTrafficCrowd`).

---

## 7. Viz

Render Orca-push cars as **oriented shaped boxes** (they carry heading) tinted distinctly (e.g. a
darker/››amber‹‹ "abandoning-the-lane" colour, or continue the fear-red). They come from the mixed crowd,
not the lane read-buffer, so `SceneGen.BuildEvacGrid` reads `mixedCrowd.Position/Heading/Count` and emits
them as shaped-disc entries (the viz already renders shaped movers — `drawShaped`). The band walls can be
drawn faintly. Opus renders to confirm the shoulder-push reads correctly.

---

## 8. Non-goals (later phases)

Sublane filter-to-front (Phase 4); scale to thousands + spatial hashing (Phase 5); real 3-D navmesh /
building-aware off-road (future). Multi-occupant crowd dynamics beyond the current pedestrian model.
In-Engine vehicle mode (Option B) unless the fork decision selects it.

---

## 9. Open decisions (for sign-off)

1. **The §3 fork: Option A (recommended) vs B.** Everything downstream depends on this.
2. Band model for the first cut: **outer wall + car obstacles** (simplest) vs **per-lane buffered band**
   (richer, more work) — I'd start with the former and add the latter as a sub-task if the push doesn't
   read well.
3. Wedge threshold (speed/dwell) and the Orca-push away-goal — tunables, calibrated against the viz.
