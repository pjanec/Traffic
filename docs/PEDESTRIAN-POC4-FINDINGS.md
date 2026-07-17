# PEDESTRIAN-POC4-FINDINGS.md — dense-crowd believability (POC-4)

**Status: DECIDED.** Companion to `docs/PEDESTRIAN-POC-PLAN.md` (POC-4) and `docs/PEDESTRIAN-DESIGN.md`
§3 "Believability" (the "measure first" rule: do not pre-build a density model; add a speed–density term
on `maxSpeed` *only* if the measured flow is visibly too orderly). This POC is pure ped-layer — no Engine,
no navmesh — built entirely from committed pieces (`OrcaCrowd`, `OrcaCrowd.AddObstacle`,
`Navigation/Bake/PedRouteController` + `WaypointFollower` for the bottleneck's hand-placed gate waypoint).
No change to `OrcaCrowd` or any other Core/committed-pedestrian file.

## Decision

**Pure ORCA is believable enough. No density term was added.**

Both scenarios below show real crowd negotiation — partial (not perfect) lane self-organization in the
corridor, non-trivial close-quarters passing, and bursty/granular (not perfectly smooth) outflow at the
bottleneck that nonetheless drains completely with no permanent jam. Per §3's explicit gate ("add a
density term *only if* the measured flow is visibly too orderly"), the flow observed here is the opposite
of "too orderly" — it already exhibits the kind of imperfect, granular self-organization real crowds show.
There is no measured evidence to justify a density/pressure term at this POC's scale and configuration.

## Scenario 1 — bidirectional corridor

`src/Sim.Pedestrians/Density/CorridorScenario.cs`, `tests/Sim.Pedestrians.Tests/Density/CorridorScenarioTests.cs`

**Setup.** A straight channel, `Width=5.0 m`, `Length=36.0 m`, bounded by two thin walls
(`OrcaCrowd.AddObstacle`, 2-vertex thin-wall convention). Two groups of 30 agents each (`Radius=0.3 m`,
`MaxSpeed=1.4 m/s`), spawned on a deterministic grid at opposite ends and walking straight across to a
mirrored goal at the far end (same row) — group A spawns at the left and walks +x, group B spawns at the
right and walks −x, so the two streams must cross and negotiate the whole channel. `MaxNeighbours=10`,
`UseSpatialHash=true`, `SymmetryBreak=0.03`, `RemoveOnArrival=true` (`ArrivalRadius=0.3 m`).

**Measured (one representative run; determinism confirmed below):**

| Metric | Value |
|---|---|
| Drain time | 668 steps = 66.8 s (Dt=0.1) |
| Arrived | A: 30/30, B: 30/30 (100%, no deadlock) |
| Throughput | A: 0.449 ped/s, B: 0.449 ped/s |
| Lane-formation \|r\| (mean over qualifying frames) | 0.106 |
| Lane-formation \|r\| (peak) | 0.332 |
| Close head-on encounters (< 0.9 m, distinct A×B pairs) | 188 / 900 possible pairs (~21%) |

**Lane-formation metric.** At each simulation frame, every still-walking agent inside the channel's
"established flow" interior (`x` in the middle 50% of the channel, away from spawn/goal edge effects) is
sampled as `(lateral offset from mid-channel, ±1 direction sign)`; `CrowdMetrics.PearsonCorrelation`
(`src/Sim.Pedestrians/Density/CrowdMetrics.cs`) gives |r| for that frame. |r|=0 means fully mixed/laneless;
|r| growing toward 1 means the two directions have segregated to opposite sides. The observed mean
(0.106) and peak (0.332) show **some** self-organization emerges (agents partially bias to one side
while passing) but it stays far from the sharp, near-1 lane segregation a "too orderly" crystallized flow
would show. Combined with 188 close (< 0.9 m) A↔B encounters out of 900 possible cross-group pairs, the
picture is a channel with real jostling and negotiation, not robotic, pre-sorted lanes.

**No deadlock.** Both directions fully drain (100% arrived) well inside the run — this rules out the
"perfectly symmetric antipodal jam" failure mode `OrcaCrowd.SymmetryBreak`/`RemoveOnArrival` exist to guard
against.

## Scenario 2 — mall-entrance bottleneck

`src/Sim.Pedestrians/Density/BottleneckScenario.cs`, `tests/Sim.Pedestrians.Tests/Density/BottleneckScenarioTests.cs`

**Setup.** A single wall at `x=GateX=20.0` spanning `y` in `[-10, 10]`, broken by a centered `1.6 m` gap.
64 agents spawn in a deterministic grid **well back** from the gate (9 m before the queue-measurement
band, so the queue genuinely has to form rather than being read off the spawn layout) and each follows a
3-point hand-placed path `[start, gate-waypoint, far-side goal]` via
`Navigation/Bake/PedRouteController` + `WaypointFollower` — the same "steering-target stand-in for a
strategic layer" pattern POC-5's `ObstacleDodgeTests` already uses for its box-detour waypoint. The gate
waypoint sits just past the doorway (not exactly in the door plane) with a small per-agent lane offset
inside the gap (deterministic, index-derived, no RNG) so more than one agent can be in the gap at once.

**Measured (one representative run; determinism confirmed below):**

| Metric | Value |
|---|---|
| Drain time | 1875 steps = 187.5 s |
| Arrived | 64/64 (100%) |
| Queue at t=0.3 s (before the front row can reach the queue band) | 0 |
| Peak queue (in the narrow pre-gate band) | 45/64 (70%) |
| Final queue | 0 (fully cleared — no permanent jam) |
| Mean gate through-rate (whole flowing phase) | 0.368 ped/s |
| Through-rate by time-quarter of the flowing phase | 0.74, 0.37, 0.12, 0.25 ped/s |

**Queue formation.** The queue count in a narrow band immediately before the gate starts at 0, climbs to
a peak of 45 agents (70% of the group simultaneously waiting), then drains to exactly 0 by the end of the
run — a genuine accumulate-then-drain curve, not an artifact of the spawn layout.

**Through-rate stability.** The gap through-rate is reported two ways: a fine 5 s-bucket series (kept for
illustration only — at ~1 crossing per 2–3 s, a 5 s bucket holds so few events that its instantaneous rate
is dominated by small-sample noise, swinging between 0 and 1.8 ped/s even for a stationary process) and a
coarse time-quarter split of the whole flowing phase (the actual stability measurement, each quarter
covering ~16 crossings — a large enough sample that the rate reflects the underlying process). The four
quarters (0.74, 0.37, 0.12, 0.25 ped/s) stay within the same rough order of magnitude of the mean
(0.368 ped/s) — no quarter collapses toward zero while the queue is still populated, and the queue
provably empties by the end. The visible burstiness (a fast early quarter, a slow third quarter) mirrors
the "arching"/intermittent-flow behavior well documented in real bottleneck pedestrian dynamics (crowds
narrowing to a doorway do not produce a metronome-steady stream; they clump, stall briefly, and release in
bursts) — this is additional evidence *against* "too orderly," not a sign of a broken solver.

**No permanent jam.** All 64 agents arrive; the queue band is provably empty at the end of the run.

## Determinism

Both scenarios include a dedicated test (`*_DeterministicAcrossIndependentRuns`) that runs the scenario
twice from a fresh `OrcaCrowd`/`PedRouteController` and asserts every per-frame position is bit-identical
(`precision: 12`) across the two runs, plus that summary counts (arrivals, queue peak, close-encounter
count) match exactly. Both pass — no `System.Random`, fixed agent/route registration order, and
`OrcaCrowd`'s own deterministic `SymmetryBreak` hashing (`(agent index, step counter)`, not wall-clock or
RNG) keep the runs reproducible.

## Why no density term was implemented

Per `docs/PEDESTRIAN-DESIGN.md` §3, a density/pressure term (a speed–density fundamental-diagram relation
on `maxSpeed`) is warranted *only* if the measured flow is visibly too orderly. Neither scenario shows
that: the corridor shows partial, imperfect lane formation with substantial cross-group jostling; the
bottleneck shows a genuinely bursty, arching-like outflow that nonetheless is capacity-limited, stays
within the same order of magnitude across the run, and fully drains with zero permanent jam. This is the
"pure ORCA accepted" branch of POC-4's success condition 3 — both outcomes were an equally valid pass, and
the evidence pointed here.

**Note for later (not built, not needed here):** if a future POC's measurements *do* call for a density
term, `OrcaCrowd` currently exposes no per-agent `SetMaxSpeed` after `Add` (only the constructor-time
`maxSpeed` argument) — see `docs/PEDESTRIAN-DESIGN.md` §3(d)'s existing "crowd-API backlog" note for the
same class of gap. Without touching `OrcaCrowd`, an opt-in density wrapper would have to follow the
already-precedented POC-3 pattern (`docs/PEDESTRIAN-DESIGN.md` §3(d): "POC-3 worked around this by
rebuilding the high-power crowd on any membership-change step, carrying each surviving agent's position +
velocity") — rebuild a fresh `OrcaCrowd` each step with per-agent `maxSpeed` set from a local-density
sample of the previous step's frozen positions, carrying position/velocity forward. That is a real,
if O(n)-per-step, decorator around `OrcaCrowd`, not a modification of it; it was not built because
measurement did not call for it.

## Files

- `src/Sim.Pedestrians/Density/CrowdMetrics.cs` — Pearson-correlation helper (lane-formation metric).
- `src/Sim.Pedestrians/Density/CorridorScenario.cs` — bidirectional corridor scenario + measurements.
- `src/Sim.Pedestrians/Density/BottleneckScenario.cs` — mall-entrance bottleneck scenario + measurements.
- `tests/Sim.Pedestrians.Tests/Density/CorridorScenarioTests.cs`
- `tests/Sim.Pedestrians.Tests/Density/BottleneckScenarioTests.cs`
