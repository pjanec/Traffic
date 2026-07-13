# INDIA-TRAFFIC.md — believable mixed-traffic (Indian-style) as a parity-exempt module

## 0. Status / decision record

This document records a **scoped** strategic decision taken with the project owner, and the
architecture that follows from it.

- **Goal of this module:** simulate *believable* Indian / South-Asian mixed traffic — lane
  discipline as a suggestion, motorcycles filtering through gaps, size/speed-based *soft*
  priority (no hard right-of-way), continuous low-speed jostling, heterogeneous vehicle
  shapes where **small vehicles fit through gaps that long/clumsy ones cannot**.
- **Scope of the parity waiver:** *this module only.* The SUMO-parity lane engine, its
  committed goldens, and the determinism hash `909605E965BFFE59` are **untouched and
  un-weakened**. Nothing here is reachable from the lane `Engine`; it is a new, additive
  behavioural layer under `Sim.Core.Mixed`. The iron law in `CLAUDE.md` still governs the
  lane core — this document does **not** repeal it, it carves out a clearly-labelled area
  where SUMO simply has no counterpart to be a parity reference against (exactly the same
  footing as the existing open-space ORCA crowd: validated *behaviourally*, no golden).
- **Why a waiver is even coherent here:** the target behaviours (asymmetric negotiated
  priority, lane-ignoring lateral packing, anisotropic shaped avoidance) have **no SUMO
  model** to match. There is no golden that could express "did the Indian junction look
  right", so exact-hash parity is not merely inconvenient, it is undefined. See §3 for the
  replacement correctness bar.

## 1. Why not "ORCA obstacles around every road, whole city as open space"

The question that started this: *is wrapping every road segment and junction in ORCA
obstacles, and running 10k vehicles as one open-space crowd, usable?*

**No, on two independent grounds:**

1. **Performance.** `OrcaCrowd`'s obstacle-neighbour query is brute-force
   (`OrcaCrowd.Plan`, "brute-force scan"): `O(agents × all-obstacle-vertices)`. The Q3
   spatial hash accelerates only the *agent-agent* query, never obstacles (the RVO2
   reference uses a dedicated obstacle KD-tree we never ported). Wrapping a whole city's
   segments in obstacles makes `all-obstacle-vertices` ≈ the entire network — hundreds of
   millions of segment tests per step at 10k agents. Even *with* an obstacle index, running
   an LP1/LP2/LP3 per agent per step for 10k agents is 1–2 orders heavier than SUMO's cheap
   scalar car-following, which is *why* SUMO does whole cities and open-crowd solvers do not.
2. **Right-of-way is structurally incompatible with reciprocal ORCA.** Stock ORCA is
   *egalitarian* — the reciprocal `u/2` split means everyone always yields half. Real
   traffic priority is *asymmetric* (priority road 100% / minor road 0%). You cannot express
   "you go, I yield fully" with a fixed 0.5 split; open-space ORCA does not lack right-of-way,
   it *contradicts* it.

**Therefore the scalable, correct substrate is the lane + junction engine** (SUMO-parity,
already benchmarked to city scale in `Sim.BenchCity` — `city-15000` ≈ 25k route entries).
Open-space shaped avoidance is applied **only in localized shared-space zones** (a chaotic
junction, a bazaar) coupled through the existing `CrossRegimeCoupling` bridge — never as the
city-wide substrate.

## 2. The three-tier model

| Regime | Model | Priority mechanism | Correctness bar |
|---|---|---|---|
| Ordinary regulated junction (lights, priority road) | Lane + junction engine | Hard, rule-based 100/0 | SUMO parity (unchanged) |
| **Chaotic mixed-traffic junction (India/Egypt)** | **`Sim.Core.Mixed` shaped VO + asymmetric responsibility** | **Soft, emergent, size/speed-driven** | **Believability rubric (§3)** |
| Free carriageway flow (filtering, laneless overtaking) | Lane engine + 1-D RVO / sublane | Following + gap | Statistical parity |

Indian traffic is **not lawless** — it is *asymmetric negotiation*: bigger/faster/more-
committed vehicles get *de-facto* priority and dominant streams emerge, without signs. That
maps exactly onto ORCA's already-present per-neighbour `Responsibility` (`OrcaSolver.Agent`):
- `α = 0.5` → reciprocal (equal jostle),
- `α → 1` → self yields fully (defer to a bus/committed mover),
- `α → 0` → self holds (others flow around a committed heavy vehicle).

Set `α` per *ordered pair* from an **assertiveness** scalar (mass/size + speed + stream
commitment) and priority streams emerge with no hard rule. This is a config of the existing
solver, not new math.

## 3. The replacement correctness bar (believability rubric)

Dropping exact-hash parity for this module does **not** mean "no bar". It means a measurable
*behavioural* bar, so review is not vibes:

**Hard invariants (must always hold — enforced by tests):**
- No interpenetration beyond ε (shape-aware: SAT overlap of oriented footprints).
- No vehicle leaves the drivable surface (road-confinement obstacles).
- Determinism: identical runs are bit-identical (seeded per-entity, order-independent).

**Distributional targets (tuned for plausibility, checked by eye + summary stats):**
- Speed distribution by vehicle class; junction throughput vs inflow.
- Headway/gap distribution; lateral packing (vehicles-abreast on a carriageway).
- % of time motorcycles spend filtering vs blocked.
- Emergent dominant stream at an uncontrolled crossing (assertiveness → flow share).

**Acceptance test = the viz.** The `Sim.Viz` artifact you watch is the primary sign-off,
backed by the hard invariants so it can never be "believable but physically broken".

## 4. Shape-aware avoidance — the OBB/capsule VO extension

The chosen fidelity is **anisotropic (orientation-dependent) shaped avoidance**, not disc +
radius. A disc has one isotropic parameter; it can express "smaller fits better" (small
radius) but not "long vehicle is awkward" (a long body needs longitudinal room and blocks a
long swath broadside, but is narrow end-on — orientation matters).

**Mechanism (`Sim.Core.Mixed.ShapedVoSolver`):** each vehicle is a *centered convex polygon*
(`ConvexShape`) — rectangle (car/bus/auto), hexagon (motorcycle), or a capsule approximated
by a polygon. For a neighbour pair the collision set in relative-position space is the
Minkowski sum `P = A ⊕ B` (centrally symmetric for centered convex shapes). The velocity
obstacle is the truncated cone of `P` translated to the relative position — the exact
polygonal generalization of RVO2's disc cone (cutoff-circle → cutoff-polygon-cap; disc
tangent legs → polygon tangent legs). The resulting ORCA half-plane feeds the **existing,
already-validated** `HalfPlaneLp` (LinearProgram1/2/3 extracted verbatim from `OrcaSolver`).

**Anchor of correctness:** in the limit where every shape is a many-gon approximation of a
circle, `ShapedVoSolver` must reproduce the disc `OrcaSolver` half-plane within tolerance.
That cross-check is a unit test — it is how we trust the novel geometry.

**What this buys, in the owner's words:**
- "hexagonal motorcycles and smaller cars fit better" — small footprints ⇒ small collision
  polygon ⇒ they thread gaps the LP closes for a bus.
- "long clumsy vehicles" — a long OBB's collision polygon is large along its length, so
  others give it a wide berth broadside and cannot cut across it, and it needs room to
  reorient — the anisotropy a disc cannot represent.

## 5. v1 scope and honest limits

**In v1 (this increment):**
- `ConvexShape` + Minkowski + tangents; `ShapedVoSolver` (agent-agent shaped VO); asymmetric
  responsibility from assertiveness; `MixedTrafficCrowd` SoA driver; road-confinement
  obstacles (disc-obstacle lines with the vehicle's inscribed radius — see limit below);
  heterogeneous fleet; invariant + anisotropy + determinism tests; an Indian-junction viz.

**Non-holonomic (car-like) steering — implemented (`MixedTrafficCrowd.Nonholonomic`).**
The pure holonomic model (velocity chosen freely each step, heading = atan2(velocity)) is
unrealistic in congestion: the solve picks backward/sideways escape velocities, heading snaps
180°, and the motion looks like "bacteria under a microscope". The non-holonomic mode treats
the ORCA velocity as a STEERING TARGET and maps it onto executable car-like motion
(`SteerNonholonomic`):
- **no reverse** — a target pointing behind the heading becomes braking, never reverse;
- **bounded, speed-scaled turn** — heading changes at most `speed·dt / MinTurnRadius` per step,
  so a slow/stopped vehicle cannot pivot in place (a small crawl term keeps enough steering
  authority to ease into a turn);
- **bounded accel/brake**, and a `cos(headingError)` speed falloff so a vehicle brakes for a
  sharp maneuver instead of sliding sideways.
Because bounded steering cannot perfectly track the holonomic avoidance velocity, real
footprints could overlap (the NH-ORCA problem); this is handled by a **tracking-error safety
margin** (`SafetyMargin`) that inflates the footprint *for the solve only* (rendering/overlap
use the true footprint), keeping real bodies apart. Measured on the dense junction scene: the
holonomic model had 16% of steps with >45° heading jumps and 9% backward flips; NH steering
drops both to ~0% with no meaningful interpenetration.

**Kinematic-bicycle body motion (rear-axle pivot).** A vehicle does not rotate about its
centre — it pivots about its **rear axle**, so through a turn the rear wheels track a tighter
arc while the front and body sweep wide (off-tracking, very visible on a bus). The execute step
integrates the bicycle model: the rear axle (a `Wheelbase`-derived offset behind the centre)
advances along the mean heading while the heading rotates, then the body pose follows. Because
the body is rigid, its orientation *is* the back→front chord (SUMO `MSVehicle::computeAngle`),
so the correct long-vehicle heading falls out with no separate approximation — where the NuGet
branch's render-side `PoseResolver` uses a documented Tier-B swept-path *approximation*
(`off ≈ length·|Δψ|/2`) because it is constrained to follow a fixed lane polyline, our
free-space case gets the real geometry. Shared conventions with that branch (issue #3): heading
= back→front chord; a laneless *vehicle* follows a curved path with chord heading, it is not a
holonomic point. Measured: a bus through a 90° turn sweeps ~54 m at the front vs ~49 m at the
rear (the rear tracks up to ~7 m inside the front). Scene inflow is metered by a
spawn-clearance gate (a vehicle enters only when its entry point is clear), which also removes
the spawn-overlap that gentle NH acceleration would otherwise cause.

**Remaining explicit limits (documented, not hidden):**
- **Obstacle avoidance stays disc-based** (inscribed radius) in v1 — road confinement is
  isotropic even though agent-agent avoidance is anisotropic. Shaped obstacle VO is a later
  rung; inscribed radius is conservative (keeps the whole footprint on-road) so it is safe,
  just slightly generous. (The NH safety margin on the vehicle's own solve shape adds wall
  clearance too.)
- **Not SUMO-parity, by design.** Judged only by §3. No golden, never in the hash path.
- **Perf:** the shaped VO reuses the agent-agent spatial hash; per-pair Minkowski is a small
  constant (≤ ~8 vertices). This is for *localized zones* (dozens–hundreds of agents), not a
  city-wide 10k crowd — that remains the lane engine's job (§1).

## 6. File map

```
src/Sim.Core/Orca/HalfPlaneLp.cs      LP1/2/3 extracted from OrcaSolver (shared, behaviour-identical)
src/Sim.Core/Mixed/ConvexShape.cs     centered convex polygon: support, Minkowski, contains-origin, tangents
src/Sim.Core/Mixed/ShapedVoSolver.cs  truncated polygonal VO -> OrcaLine (reuses HalfPlaneLp)
src/Sim.Core/Mixed/VehicleClass.cs    bus/car/auto/moto: dims, assertiveness, maxSpeed
src/Sim.Core/Mixed/MixedTrafficCrowd.cs  SoA driver: heading, asymmetric responsibility, road confinement
tests/Sim.ParityTests/MixedTraffic*Tests.cs  circle-limit vs disc, SAT no-overlap, anisotropy, priority, determinism
```
