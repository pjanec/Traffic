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
