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

### DR replay CORRECTED per docs/IGBRIDGE-HTML-REPLAY-GUIDE.md (vendored from the poc branch)
First DR attempt was wrong on the guide's headline gotchas — cars "wildly dancing", peds "caterpillar":
- **§5.1 (the #1 cause):** emitted a 3-tuple without `useDataHeading`, so the player drew the jittery PATH
  TANGENT, not the kinematic heading → "wildly dancing". FIX: added `ScenePayload.UseDataHeading`
  (serialized camelCase → `scene.useDataHeading`), emit a **5-tuple `[x,y,headingDeg,len,wid]`**, set true.
- **§5.4:** emitted the FRONT (`SmoothedFront`), not the **CENTER** (the IG pivot). FIX: emit
  `result.CenterX/CenterY` + `result.HeadingDeg` + true per-vehicle dims.
- **§5.3:** the look-ahead uses the engine's **UpcomingLanes** DR prediction (already carried on
  `VehicleSource` history records) — anticipates junction turns.
- **PEDS (not in the guide — my caterpillar):** was `LiveCitySim.SamplePedsAt(tau)` = querying the manager's
  CURRENT arc at a LAGGED tau → fast-then-slow. FIX: reconstruct off the ped WIRE via
  `PedRemoteReconstructor(sim.PedSource)` (`Pump(tau+pedDelay)` → `TryGetRenderPose`) — the analytic
  PathArc/ActivityTimeline playout the viewers use, no tick kink. (Removed the SamplePedsAt approach.)
Result: 796 frames, 5.7 MB, `useDataHeading` in the JSON, build green. Awaiting owner visual confirm; once
confirmed, add a PED section to IGBRIDGE-HTML-REPLAY-GUIDE.md (owner request).

---

## DEFECT #1 (cars vs peds on crossings) — per-entity analysis (faithful sim)

Tool: `Sim.Viz --live-city-yielddump <steps>` (new; runs the REAL `LiveCitySim`, classifies every
car-within-2.5 m-of-a-crossing-ped from raw `Sample()` states using the MOTION direction as "forward" —
convention-free). Exposed `LiveCitySim.CrossingCentroids` for it. Diagnostic-only, parity untouched.

**Run (200 steps = 100 s):** pedOnCrossingSamples=7044, carsWithin2.5m=**152**, classified:
- **stoppedYield (car <0.5 m/s near ped) = 56** — cars that DID stop/yield.
- **moving BESIDE/behind = 70** — passing a ped crossing the other way; NOT a collision course (benign).
- **moving IN-PATH = 26** — car driving AT a ped ahead in its corridor = **the real defect-#1 failures**.
  Of these only **3 are fast (>=6 m/s)** → tunnelling is a MINOR factor.

**So the 2.5 m proximity metric over-counts ~6x.** The real issue is ~26 in-path samples (≈5–10 distinct
encounters/100 s — the same cars recur across ticks). The "worst" cars DECELERATE across consecutive ticks
(veh45 5.2→0.7, veh81 2.6→1.6 m/s) yet stay in-path, nosing to `along≈0.2 m` (ped at the car's front).

**Refined defect statement:** cars mostly DO react to crossing peds (56 stopped, 26 braking-in), but a few
per ~100 s **brake too late / stop with too tight a margin — the front noses onto the ped** ("go over
them"). The encroachment is SLOW, not fast tunnelling.

**Leading hypothesis (VERIFY before designing):** `Engine.CrowdLongitudinalConstraint` only brakes when the
ped disc is inside the car's NARROW wheel-path corridor (`|latOff - ego.LatOffset| < egoHalf + discR`,
`Engine.cs:~8602`). For a ped walking ACROSS the road, it enters that corridor only when almost in front →
the brake triggers late → the car noses in. A human yields for a ped ANYWHERE on the crossing ahead.
NEXT: instrument WHEN the brake first engages vs the ped's lateral offset (confirm late-trigger), + confirm
these peds are on GREEN (batch metric said ~97% ped-on-green). THEN design (likely: brake for a ped anywhere
on a crossing polygon ahead, not just the wheel corridor — demo-gated, parity-safe).

### DEFECT #1 — brake-TIMING trace (`--live-city-yieldtrace`) — the late-trigger hypothesis is REFUTED; the real cause is a FEED problem
New engine-AUTHORITATIVE diagnostic `Sim.Viz --live-city-yieldtrace <steps>` (real `LiveCitySim`; uses
`WitnessAuthoritative().Binder` — **Binder==13 == `CrowdLongitudinalConstraint` bound this car** — to catch
the exact tick the crowd-brake first engages, + `LiveCitySim.IsOccupancyMarkedAt(x,y)` to test feed
membership). Corrected two things first: the demo cars are the **default passenger vType** (`Engine.cs:265`
`DefineVType{VClass="passenger"}` → width 1.8, len 5, **decel 4.5 / emergencyDecel 9**), NOT the army vTypes
in `scenario.rou.xml` (those are unused by `LiveCitySim`, which spawns its own demand).

**Run (400 steps = 200 s):**
- crowd-brake ONSET events = 17 (13 with an in-path crossing-ped ahead). At onset `|lat|` median 1.79 m vs
  corridor-half 1.20 m. **FORCED nose-in at onset (stop-dist > gap) = 1/13** even at COMFORT braking → once
  the brake engages the car can almost always stop. So the late-trigger does NOT force the nose-in.
- **NOSE-IN ticks (ped within 0.3 m of a MOVING car's front) = 19** (fast≥4 m/s = 4; median 2.7 m/s).
- **brake-state at nose-in: neverBraked = 18/19** (crowd-brake never engaged for that car), whileBraking=1,
  after-release=0; car ACCELERATING during nose-in = 7/19. ⇒ **NOT a late-trigger and NOT a release-lunge —
  for the nosed-over peds the crowd-brake never fires at all.** Since a nose-in means the ped is INSIDE the
  1.2 m corridor at that tick, the only explanation is **the ped disc is not in the CrowdSource.**
- **nosed-over ped REGIME: paused = 9, lowPowerWalking = 8, ORCA = 2.** Feed membership: **fed = 8,
  NOT-fed = 11.**

**ROOT CAUSE (two compounding, both FEED-SIDE → `Sim.LiveCity`/`Sim.Pedestrians`, parity-safe, no engine edit):**
- **(A) FEED GAP — 9/19 (paused peds).** `LiveCitySim.Step` gathers `_movingLowPowerPositions` with
  `ModelOf(id)!=FreeKinematic && AnimTagOf(id)==WalkAnimTag` (`LiveCitySim.cs:479`). A low-power ped that
  PAUSES on a crossing (demand has `PauseProbability=0.15`, `PauseAnimTag="idle"`, 2–5 s) fails the
  `WalkAnimTag` test → not fed to `CrossingOccupancySource` → **invisible to cars** (also not ORCA, so not in
  `HighPowerFootprints` either). This IS defect #2, and it is the biggest single bucket of defect #1. A
  stopped ped on a crosswalk is MORE reason to yield, not less.
- **(B) POINT-DISC vs NARROW CORRIDOR — 8/19 (walking peds).** `CrossingOccupancySource.Update`
  (`CrossingOccupancySource.cs:125`) marks a **0.3 m point disc at the ped's exact position** (vel 0). The
  car's corridor gate then only reacts when that tiny disc enters `|Δlat| < egoHalf+0.3 = 1.2 m` — i.e. when
  the crossing ped is nearly in front → too late for a 5 m body to stop short. A human yields for a ped
  anywhere on the crossing width ahead. Fix direction: gate the OCCUPIED CROSSING (span its width across the
  approaching lane / enlarge the disc), not a moving 0.3 m point.
- **(residual) 2/19 ORCA** — a footprint/promotion-timing edge (ped promoted/around demotion); minor, revisit
  with defects #3/#4.

**Conclusion:** the owner's report ("cars don't care, they go over the peds") is VINDICATED, not over-counted
— ~58 % of nose-ins are peds the cars genuinely cannot perceive. Defects #1 and #2 are ONE root cause: the
crossing-occupancy feed is incomplete (drops paused peds) and impoverished (a 0.3 m point, not the occupied
crossing). Both fixes live entirely on the feed side; the engine's `CrowdLongitudinalConstraint` is correct
and stays untouched (parity intact). NEXT: write the design doc for the two feed fixes (A: include on-crossing
non-walking low-power peds; B: gate the crossing span, demo-gated), get owner agreement, then implement +
verify by re-running `--live-city-yieldtrace` (nose-in → ~0) and an HTML replay.

Diagnostic tooling committed (parity-inert): `Sim.Viz --live-city-yieldtrace`, `LiveCitySim.IsOccupancyMarkedAt`.

### DEFECT #1 — CORRECTION + FIX (design: `docs/LIVE-CITY-REALISM-1-2-DESIGN.md`)
**Correction to the "9 paused" headline above:** the trace's on-crossing test was a loose ~9 m centroid
circle (the crossing polys are junction-sized, half-extent median 8.65 m), so it counted peds standing on the
SIDEWALK near junctions. Tightened it to a real point-in-polygon (`CrossingOccupancySource.IsInsideAnyCrossing`
→ `LiveCitySim.IsOnCrossingPolygon`). With the true test, the nosed-over peds are **8 walking + 2 ORCA, 0
paused** — the "9 paused" were an artifact. So the *live* defect is entirely cause **(B)** (the 0.3 m point
disc), not the feed gap. Cause **(A)** (paused-ped feed gap = defect #2) is a REAL mechanism but had 0
on-crossing cases in this run; fixed defensively anyway.

**A/B sweep (400 steps, true polygon test, engine-authoritative binder/speed; nose-in = moving car's bumper
over a ped on a real crossing; arrived = car trips completed):**

| config | nose-in | breakdown | arrived |
|---|---|---|---|
| STOCK r=0.3, no A | 10 | walking 8, ORCA 2 | 95 |
| r=1.5, no A | 2 | walking 1, ORCA 1 | 94 |
| **A + r=1.5 (SHIPPED)** | **1** | ORCA 1 | 94 |
| A + r=2.5 | 1 | ORCA 1 | 91 ← flow starts to cost |

**Fix (feed-side, parity-inert — engine untouched):** two `LiveCityConfig` knobs —
`CrossingGateRadius=1.5` (env `LIVECITY_GATE_RADIUS`): the occupancy gate disc, enlarged from 0.3 m so a car
brakes for a ped on the crossing ahead in time, yet lane-LOCAL (corridor half 0.9+1.5=2.4 m < 4 m lane
spacing → no adjacent-lane brake); and `GatePausedPedsOnCrossing=true` (env `LIVECITY_GATE_PAUSED`): also feed
low-power peds paused on a crossing (defect #2). Owner picked r=1.5 (minimal lane-local value; r≥2.5 costs
flow). **Literal polygon-gating was rejected** — the junction-sized polys would over-brake and regress #15.

**Gates (all green):** parity `657/4` byte-identical; bench `D96213B7BB4021A7`, par==single;
`Sim.LiveCity.Tests` **24/24** incl. new `CrossingYield_FixedGate_NosesOverFarFewerCrossingPeds_ThanStockPointDisc`
(stock nose-in ≥5, fixed ≤3, fixed*2<stock — a guard the parity gate structurally cannot provide).
**Residual = 1 ORCA nose-in** (promotion/footprint-timing) → folded into defects #3/#4. NEXT (owner): confirm
the HTML replay (cars stop at occupied crossings), then move to #3/#4.

### DEFECT #1 — DENSITY regression (owner 10x stress-test): "cars drive through ORCA AND low-power peds like crazy"
At `LIVECITY_PEDS=1600` (10x) the r=1.5 fix FAILED: `--live-city-yieldtrace 400` → nose-in **28** (15 FAST
≥4 m/s, median 6.5 m/s). Two density-only root causes, both proven from entity state:
- **Engine 16-disc query cap** (`CrowdLongitudinalConstraint`/swerve `stackalloc WorldDisc[16]`): new probe
  `LiveCitySim.CrowdDiscCountsNear` shows **25/28** nose-ins have >16 crowd discs near the car (median 39,
  max 131) → the in-path disc is truncated away, car never brakes. Fix: `Engine.MaxCrowdDiscs=256` (both
  sites). Parity-inert (gated on `CrowdSource != null`, null on all goldens) → verified `657/4` + bench
  `D96213B7BB4021A7` unchanged. **This is the load-bearing fix.**
- **ORCA peds = 0.3 m footprint** (narrow corridor). Knob `GateOrcaPedsOnCrossing` (env `LIVECITY_GATE_ORCA`)
  would feed them to the wide gate, but the velocity-0 gate over-brakes a WALKING ORCA ped → DenseFlow
  throughput 490→418. **DEFAULTED OFF**: fix (buffer) alone already cut ORCA nose-in 11→3 at 10x. Proper ORCA
  fix (velocity-preserving wide footprint) = follow-up.

**A/B (buffer on):** ORCA-feed OFF → 10x nose-in 5 (1 fast), 1x/2000-step throughput **490** (≥450 guard ✓);
ORCA-feed ON → 5 (0 fast) but throughput 418 (✗). Shipped = ORCA OFF.
**Result:** 10x nose-in **28→5, 0 fast** (slow ~1.3 m/s tail); 1x **0**; throughput preserved. New guard
`CrossingYield_HoldsUnderHighPedDensity_NoMassDriveThrough` (10x nose-in ≤12). All 25 LiveCity tests green.
**OPEN (owner):** the ORCA narrow-corridor residual (3 at 10x) — do the velocity-preserving ORCA footprint
next, or accept? Also: merge main's viz-unification + add click-to-mark-a-car to the viewer (owner asks).
