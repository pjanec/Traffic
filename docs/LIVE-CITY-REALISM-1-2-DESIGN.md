# Live-city realism #1/#2 — cars driving over crossing pedestrians — DESIGN + tasks

**Status:** implemented, verified, shipped on `claude/livecity-realism-fixes`.
**Requirements (the WHAT):** owner-observed realism violations #1 and #2 (`docs/TASKS.md`):
- **#1** — cars in the high-realism zone drive *through* pedestrians on crosswalks (no yield/dodge).
- **#2** — low-realism crossings are not marked "occupied" when low-power peds cross, so cars don't stop.

The analysis (below) proves **#1 and #2 are one root cause**: the crossing-occupancy *feed* the vehicle
CrowdSource consumes is both **impoverished** (a ped is a 0.3 m point) and **incomplete** (paused peds are
dropped). This doc is the HOW. The running investigation trail is
`docs/LIVE-CITY-REALISM-ATTEMPT-LOG.md`; read it for the raw per-entity numbers.

---

## 1. Diagnosis (from entity-state dumps — the binding rule: verify before designing)

Tooling (all diagnostic-only, parity-inert): `Sim.Viz --live-city-yieldtrace <steps>` runs the **real**
`LiveCitySim`(`LiveCityConfig.ForRepoRoot`) and, for every car approaching a ped on a crossing, reads the
**engine-authoritative binder** (`WitnessAuthoritative().Binder == 13` ⇔ `CrowdLongitudinalConstraint` bound
this car) to catch the exact tick the crowd-brake engages, the ped's regime (`Sample()`), feed membership
(`LiveCitySim.IsOccupancyMarkedAt`), and true on-crossing membership
(`LiveCitySim.IsOnCrossingPolygon` → `CrossingOccupancySource.IsInsideAnyCrossing`, point-in-polygon).

**Findings (400 steps = 200 s, real demo sim):**
- **19 → (tightened) 10 nose-in ticks** — a *moving* car with its front bumper on a ped who is inside a real
  crossing polygon (the visible "drives over them"). *(The looser first pass over-counted 9 "paused" peds
  that were on the sidewalk near junction-sized crossing polygons, not on the crossing; the point-in-polygon
  test removed them. Recorded here so the correction isn't lost.)*
- **The late-trigger / release-lunge hypotheses were REFUTED.** 18/19 nose-in ticks the crowd-brake had
  **never engaged** for that car; 7/19 the car was *accelerating*. Since a nose-in means the ped is already
  inside the car's ~1.2 m wheel-path corridor, the only explanation is **the ped disc is not in the
  CrowdSource** — the car cannot see it.
- Regime of the (true-crossing) nosed-over peds: **8 walking, 2 ORCA, 0 paused**; feed membership **8 fed,
  2 not-fed** (the 2 not-fed are the ORCA pair — they ride `HighPowerFootprints`, not the occupancy source).

**Mechanism (two compounding causes, both FEED-SIDE):**
- **(B) Point-disc vs narrow corridor — the live defect (8/10).** `CrossingOccupancySource.Update`
  marks a **0.3 m point disc at the ped's exact position** (`CrossingOccupancySource.cs`). The engine's
  `CrowdLongitudinalConstraint` gate (`Engine.cs` ~8602) only brakes when a disc is inside
  `|Δlat| < egoHalf + discR` = 0.9 + 0.3 = **1.2 m** laterally. A ped walking across the road enters that
  corridor only when nearly in front → too late for a 5 m body (decel 4.5 m/s²) to stop short → it noses in.
- **(A) Feed gap — real mechanism, 0 live cases *in this run* but demo-relevant.** `LiveCitySim.Step`
  gathered occupancy positions with `ModelOf(id)!=FreeKinematic && AnimTagOf(id)==WalkAnimTag`. A low-power
  ped that **pauses** on a crossing (`PauseProbability=0.15`, `PauseAnimTag="idle"`) fails the `WalkAnimTag`
  test → not fed → invisible. **This is defect #2.** No ped happened to pause *on* a crossing polygon during
  the measured run, but the gap is real and a stopped ped on a zebra is *more* reason to yield, not less.

**The engine is correct.** `CrowdLongitudinalConstraint` faithfully brakes for what it is shown; it is being
shown an impoverished, incomplete picture. So the fix lives entirely on the feed side — **no `Sim.Core`
edit, parity untouched.**

---

## 2. Design (HOW)

Two knobs on `LiveCityConfig` (demo path only). Both are parity-inert: every committed golden / the bench
drive the `Engine` directly with `CrowdSource == null`, so `CrossingOccupancySource` is never constructed or
queried there — these knobs cannot move any golden trajectory or the bench hash.

### Fix B — lane-local gate-disc radius  (`CrossingGateRadius`, default **1.5 m**, env `LIVECITY_GATE_RADIUS`)
`LiveCitySim` passes `cfg.CrossingGateRadius` to the `CrossingOccupancySource` ctor (was a hardcoded 0.3).
A ped on a crossing is fed as a disc of that radius — "the ped occupies this patch of the zebra." Effect:
- Laterally, the car now brakes for a ped up to `egoHalf + r = 0.9 + 1.5 = 2.4 m` off its centerline —
  early enough to stop, yet **still lane-local**: 2.4 m < the demo's 4 m lane spacing, so it does **not**
  bleed onto adjacent lanes (the failure mode that ruled out a naïvely fat disc).
- Longitudinally the gate sees the ped's near edge sooner (`nearestBack = offset − r`), opening the
  stopping gap earlier.

**Why not gate the crossing polygon itself (the first-choice "B-2").** The baked crossing polygons here are
*junction-sized* — half-extent median **8.65 m**, max **12.8 m**. A centroid disc or polygon fill over those
would brake cars across the whole junction and regress the #15 dense-flow gains. The A/B sweep (below)
confirms the lane-local disc reaches the same nose-in floor with **no** throughput cost, whereas radius ≥ 2.5
starts to cost flow. So the geometry, not preference, selected the enlarged-disc form at r = 1.5.

### Fix A — feed paused on-crossing peds  (`GatePausedPedsOnCrossing`, default **true**, env `LIVECITY_GATE_PAUSED`)
`LiveCitySim.Step` now feeds **all** low-power (non-ORCA) peds to the occupancy source, dropping the
`WalkAnimTag` requirement; the source's point-in-polygon test still restricts *discs* to peds actually on a
crossing, so an idle ped on the sidewalk costs one bbox reject and marks nothing. Closes the #2 gap
defensively.

### Determinism / parity argument
- Occupancy is a pure function of the (pure-function-of-time) low-power ped poses; no RNG; order-independent.
- `Engine.CrowdSource` is null on every golden and the bench → both fixes are structurally inert there.
  **Verified:** parity `657/4` byte-identical; bench hash **`D96213B7BB4021A7`**, parallel == single.

---

## 3. Tasks & success conditions (all met)

| ID | Task | Files | Success condition | Status |
|----|------|-------|-------------------|--------|
| R1-T1 | Diagnose from entity state; localize to the feed | `Sim.Viz/Program.cs` (`--live-city-yieldtrace`), `LiveCitySim.IsOccupancyMarkedAt/IsOnCrossingPolygon`, `CrossingOccupancySource.IsInsideAnyCrossing` | Nose-ins attributed to a named mechanism from per-entity data, not a guess | ✅ |
| R1-T2 | Fix B: lane-local gate disc | `LiveCityConfig.CrossingGateRadius`, `LiveCitySim` ctor | Walking nose-ins → 0; throughput flat; lane-local (no adjacent-lane brake) | ✅ |
| R1-T3 | Fix A: feed paused on-crossing peds | `LiveCityConfig.GatePausedPedsOnCrossing`, `LiveCitySim.Step` | Paused low-power ped on a crossing produces an occupancy disc | ✅ |
| R1-T4 | Lock it (parity can't) | `tests/Sim.LiveCity.Tests` `CrossingYield_FixedGate_NosesOverFarFewerCrossingPeds_ThanStockPointDisc` | Fixed nose-in ≤ 3 and `fixed*2 < stock`; stock ≥ 5 (defect reproduced) | ✅ |
| R1-T5 | Parity iron law | — | `657/4`; bench `D96213B7BB4021A7`; par==single | ✅ |
| R1-T6 | Visual verification | `Sim.Viz --live-city-demo` HTML replay | Owner confirms cars stop at occupied crossings in the DR-smoothed replay | ⏳ owner |

**A/B sweep (400 steps, true point-in-polygon, engine-authoritative speed):**

| config | nose-in | breakdown | arrived |
|---|---|---|---|
| STOCK (r=0.3, no A) | 10 | walking 8, ORCA 2 | 95 |
| r=1.5, no A | 2 | walking 1, ORCA 1 | 94 |
| **A + r=1.5 (shipped)** | **1** | ORCA 1 | 94 |
| A + r=2.0 | 1 | ORCA 1 | 94 |
| A + r=2.5 | 1 | ORCA 1 | 91 ← flow starts to cost |

**Residual:** 1 ORCA nose-in — a promotion/footprint-timing edge (a ped around promotion isn't in
`HighPowerFootprints` for a tick). Out of scope here; folded into realism defects **#3/#4** (ped LOD
transitions).

---

## 4. Density robustness (owner stress-test at 10× ped density)

At 10× ped density (`LIVECITY_PEDS=1600`) the r=1.5 fix regressed badly — nose-in **28** (15 *fast* ≥4 m/s,
median 6.5 m/s), i.e. cars driving *through* both low-power and ORCA peds. The trace (`--live-city-yieldtrace`
+ `LiveCitySim.CrowdDiscCountsNear`) localized two density-only bugs:

- **(C) The engine's crowd-disc query cap.** `CrowdLongitudinalConstraint` (and the B6 swerve-synthesis site)
  read at most **16** discs into a `stackalloc WorldDisc[16]`. At 10× a car has a **median 39 / max 131**
  crowd discs in range, so the in-path disc was truncated away **25/28** nose-ins → the car never braked.
  Fix: `Engine.MaxCrowdDiscs = 256` at both query sites. **Parity-inert** — every crowd query is gated on
  `CrowdSource != null`, null on every golden/bench, so the buffer is never even allocated there (verified
  `657/4` + bench `D96213B7BB4021A7` unchanged after the edit). This is the load-bearing density fix.

- **(D) ORCA peds fed only as their 0.3 m physics footprint** (`OrcaCrowd.QueryNear`) → same narrow-corridor
  late-trigger. `LiveCityConfig.GateOrcaPedsOnCrossing` (env `LIVECITY_GATE_ORCA`) would also feed ORCA
  peds on a crossing to the wide occupancy gate. **Defaulted OFF:** the occupancy disc is velocity-0, so it
  makes a car fully STOP for an ORCA ped merely walking across → measured ~15 % car-throughput loss (the
  `DenseFlow` liveness guard drops 490→418) for little gain, because fix (C) alone already cut ORCA nose-ins
  **11→3** at 10×. The *proper* ORCA fix — a velocity-PRESERVING wide vehicle-facing footprint (so the car
  brakes with margin but follows the ped's motion rather than stopping dead) — is a **follow-up** (realism
  #3/#4 neighbourhood); the knob stays for experimentation.

**Result (10×, shipped = C on, D off):** nose-in **28 → 5**, of which **0 fast** (median 1.3 m/s — a slow
tail of "ped steps onto the zebra right in front of a committed car", not drive-throughs). 1× stays at
**0**. Car throughput preserved (`DenseFlow` 490 ≥ 450 guard).

Regression guard: `tests/Sim.LiveCity.Tests` `CrossingYield_HoldsUnderHighPedDensity_NoMassDriveThrough`
(10× density, nose-in ≤ 12; pre-fix was 28). `DenseFlow_...NoGridlock` guards the throughput side.
