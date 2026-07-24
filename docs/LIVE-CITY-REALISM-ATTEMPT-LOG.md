# Live-city REALISM violations — attempt log (findings & progress)

Fresh log for the branch **`claude/livecity-realism-fixes`** (off `main` @ `b70c068`, the merged live-city
milestone). Same workflow as #15: **solid repro → extensive analysis from ENTITY-STATE DUMPS → design →
implementation**, each fix shown working via a **Sim.Viz HTML replay** (Canvas 2D, self-contained, no 3D
needed — 3D already proved these out). Owner reviews the HTML replays visually.

The five defects (from `docs/TASKS.md` "Realism violations in high-realism zones", owner-observed in 3D):
1. **Cars drive THROUGH peds on crosswalks** (high-realism zone) — no yield/dodge.
2. **Low-realism crossings not marked 'occupied'** when low-power peds cross — cars don't stop short.
3. **Low-power peds DISAPPEAR on promotion** into the high-realism zone (gone, or re-appear as ORCA later).
4. **ORCA peds leaving the zone STAY ORCA and wander** (off-sidewalk, no route, never demote).
5. **ORCA peds don't dodge a SUMO car standing on the crosswalk** (walk into it despite room to pass).

---

## HOW TO REPRODUCE / SEE IT (deterministic, headless, no SUMO, no 3D)

### Visual replays (what the owner opens)
`Sim.Viz` renders the coupled live-city sim (cars + peds + crossings, ped colour = LOD: low vs ORCA) to a
single self-contained `replay.html` (Canvas 2D):
```bash
dotnet build src/Sim.Viz -c Release
# Full coupled live-city scene:
dotnet run --project src/Sim.Viz -c Release --no-build -- --live-city <out>/live-city.html
# Focused testbeds (one subsystem each) — the repro scenes for these defects:
dotnet run --project src/Sim.Viz -c Release --no-build -- --ped-crossing-gate  <out>/crossing.html   # car↔crossing yield (defects 1,2)
dotnet run --project src/Sim.Viz -c Release --no-build -- --ped-lod-promotion  <out>/lod.html         # promote/demote (defects 3,4)
dotnet run --project src/Sim.Viz -c Release --no-build -- --ped-dodge          <out>/dodge.html       # ORCA obstacle avoidance (defect 5)
```
`SceneGen.BuildLiveCity(boxDir)` builds the SAME coupled sim as the `LiveCitySim` host; the replay frames
carry `frame.V` (vehicles) and `frame.D` (peds; `d[3]` = kind, `KindPedHighPower` = ORCA vs low-power).

### Entity-state dumps (the analysis substrate — do this BEFORE proposing any fix)
Headless `LiveCitySim` run with per-entity dumps (mirrors the #15 `LIVECITY-*` probe approach): per frame,
dump each ped `{id, x, y, model/regime, promoted?, animTag}`, each car `{id, x, y, speed, yield-binding}`,
and each crossing `{centroid, halfWidth, occupiedBy[]}`. The dump is what proves the defect and localizes
the subsystem — never chase a hypothesis before it shows in the per-entity data (the #15 iron rule).

### Wiring under test (from `src/Sim.LiveCity/LiveCitySim.cs`)
- Car↔ped yield: `Engine.CrowdSource = CompositeFootprintSource(PedLodManager.HighPowerFootprints,
  CrossingOccupancySource)` when `YieldEnabled` — cars react to (a) ORCA-ped footprints + (b) crossing
  discs marked occupied by WALKING low-power peds (`_movingLowPowerPositions`, filtered
  `ModelOf(id) != FreeKinematic && AnimTag == WalkAnimTag`, via `_crossingOccupancy.Update(...)`).
- Ped LOD: `PedLodManager` promotion/demotion at the `InterestField` zone (settable via `SetLcRealismZone`,
  promote/demote radii).

---

## Gates (unchanged iron law)
`dotnet test tests/Sim.ParityTests -c Release` = **657/4** byte-identical; `Sim.Bench` hash
**`D96213B7BB4021A7`**, parallel==single; no `System.Random`; all fixes demo-gated → goldens inert.

---

## PROGRESS LOG
(newest last)

### Setup — toolchain established
- Branch created off the merged milestone; fresh log started.
- Confirmed `Sim.Viz --live-city` + the three focused scenes render self-contained HTML replays that show
  cars, peds, and per-ped LOD kind — the visual-verification channel (delivered to owner as files).
- Next: pick the first defect, build a solid headless repro with per-entity dumps, analyse, THEN design.

### Baseline repro metrics (first-hand, `Sim.Viz` batch scene `BuildLiveCity`, 240 frames)
`--live-city` prints built-in realism diagnostics — these are the headless repro numbers:
- crossings = 66 (all **signalized**, 0 unsignalized); peak occupied = 46.
- **near-collision (car within 2.5 m of a WALKING ped on a signalized crossing) = 80**, of which
  ped-on-RED = 2 → **≈78 are cars closing on peds who have GREEN = defect #1**, quantified headlessly.
- `minCarSpeedNearOccupiedCrossing = 0.00 m/s` — some cars DO stop; the 80 near-collisions are the ones
  that don't (or stop too late).
- ped compliance: low-power-ped-on-signalized samples = 1957, **during RED = 37 (should be 0)** — a ped
  signal-compliance angle adjacent to defect #2.
- maxHighPower (ORCA-promoted peds) = 13 → promotion IS happening, so #3/#4 (LOD transitions) are
  exercisable here too.

**Toolchain note (important):** `Sim.Viz --live-city` builds via `SceneGen.BuildLiveCity` (batch), which
shares the ped/crossing/LOD subsystems with the `LiveCitySim` host but does NOT apply the #15 host-side
lane-change gates (it still shows "46 stopped lane-changes"). That's fine for the PED-realism defects
(1–5, which live in the shared CrossingOccupancy / PedLodManager / ORCA subsystems), but any lane-change
observation must be taken on the `LiveCitySim` path, not this one. Decision: use `--live-city` +
the focused scenes (`--ped-crossing-gate`, `--ped-lod-promotion`, `--ped-dodge`) as the ped-realism
repro/visual channel.

Focused testbeds generated (frames): crossing-gate 170 (1 car, 11 peds) · lod-promotion 350
(everPromoted=True) · dodge 234 (2 reroute events). Each isolates one subsystem.

**NEXT:** start with the **crossing / car-yield cluster (defects #1 + #2)** — it is already quantified
(near-collision=80) and has a clean focused testbed. Build the per-entity dump (car speed + yield-binding
vs each occupied crossing + the peds on it), localize WHY the ~78 green-ped near-collisions happen (is the
crossing disc fed? is the car's yield gate consulting it? is it a timing/lookahead gap?), THEN design.

### Toolchain CORRECTED — faithful `--live-city-demo` (owner: "same as demo, no cheating")
The batch `Sim.Viz --live-city` (`SceneGen.BuildLiveCity`) was found to have DIVERGED from the shipped demo:
it hand-rolls its own Engine + PedLodManager + InterestField (no `LiveCityConfig`, no `LaneChangeMinSpeed`,
no cooperative LC, no per-area LOD gate; e.g. its own `CarTargetConcurrent`). A fix verified there would NOT
be guaranteed to transfer to the demo. **Rejected for the realism work.**

Built a FAITHFUL exporter: **`Sim.Viz --live-city-demo <out>`** → `SceneGen.BuildLiveCityDemo(repoRoot)`
drives the REAL `LiveCitySim` with `LiveCityConfig.ForRepoRoot` (the EXACT demo net + demand + params:
cooperative LC, per-area realism LOD gate, `LaneChangeMinSpeed`, crossing-occupancy yield) and captures
`LiveCitySim.Sample()` each step (cars→boxes, peds coloured by the demo's own LOD regime: grey low / orange
ORCA / yellow paused). The headless run uses the static crop-centre zone (no camera) = the demo's
non-interactive baseline. **A fix verified in this replay transfers directly to the City3D/raylib demo.**
(Sim.Viz now references Sim.LiveCity; parity/engine untouched — Sim.Viz is not on the golden path.)

**FAITHFUL baseline (real LiveCitySim, 240 steps = 120 s):** maxCars=158 (real 160-target config), maxPeds=160,
maxHighPower(ORCA)=13. **near-collision (car within 2.5 m of a ped ON a crossing) = 43** over
pedOnCrossingSamples=1197; minCarSpeedNearOccupiedCrossing=0.00 m/s. This 43 is the **defect-#1 repro number
on the demo-faithful sim** — the target to drive toward 0 (or explained) by the fix.

Henceforth ALL realism repro + fix-verification uses `--live-city-demo` (and, for isolation, the focused
`--ped-*` scenes cross-checked against it). The batch `--live-city` stays only as the public-gallery demo.

### Faithful replay now uses the VIEWERS' DR motion reconstruction (owner: HTML must match 2D/3D viewers)
The `--live-city-demo` replay was writing raw 2 Hz `Sample()` poses → the HTML showed the steppy
authoritative motion (junction facet-snaps, instant lane changes), NOT the DR-predicted smoothing the raylib
2D + City3D viewers apply at render time. Fixed: `BuildLiveCityDemo` now runs the EXACT viewer pipeline —
`DrClock.ResolveAt` (deterministic sibling of the wall-clock `Resolve`, the one IgBridge uses) → the shared
`KinematicReconstructor` facade (`Sim.Viewer.Motion`; continuous junction arcs + gliding lane changes +
rear-axle heading drag), fed from `LiveCitySim.VehicleSource` (the same in-process replication wire the
viewers consume) with a one-step (0.5 s) playout delay + `CoarseFeed`, resampled at render rate. Peds are
sampled at the SAME render instants via new `LiveCitySim.SamplePedsAt(t)` (ped DR is continuous in time).
The replay player's own Catmull-Rom interpolation fills between the reconstructed samples, so 6 Hz emit is
smooth AND mobile-friendly (718 frames, 4.8 MB vs 8 MB at 10 Hz). Sim.Viz now references
`Sim.Viewer.Motion` + `Sim.LiveCity`; engine/goldens untouched (full build green, `Sim.LiveCity.Tests`
23/23). near-collision RATE unchanged (129/3582 = 3.6% vs 43/1197 = 3.6% → defect #1 intact). **The HTML now
shows the SAME DR-smoothed motion as the demos.**
