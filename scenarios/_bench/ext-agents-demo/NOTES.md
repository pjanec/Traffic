# ext-agents-demo (Sim.ExtDemo visual test)

First visual test of the EXTERNAL-AGENT layer: pedestrians/cars driven OUTSIDE SUMO, injected
into the engine each run via the existing B1/B5 obstacle API
(`IEngine.AddObstacle`/`AddMovingObstacle`), so committed SUMO vehicles visibly react
(brake/stop/follow). No golden.fcd.xml here (same as `scenarios/14-external-obstacle`): the
external agents are not present in any SUMO run, so there is nothing to diff against.

## Net

2-lane, 400 m straight road (`e0_0` right lane y=-4.8, `e0_1` left lane y=-1.6), generated via
netconvert from `nodes.nod.xml`/`edges.edg.xml` (see `provenance.txt`).

## Demand (`rou.rou.xml`)

Two `passenger` cars (`sigma="0"`), both departing at rest (`depart=0 departPos=0 departSpeed=0`):
  - `car_ped` on `e0_1` -- approaches the crossing pedestrian.
  - `car_follow` on `e0_0` -- comes up behind the slow external car.

## External agents (`external-agents.json`)

Schema documented in `src/Sim.ExtDemo/ExternalAgent.cs`. Two agents:

  - `ped1` (pedestrian): steps into the road at pos 150 m, active `[8, 16)` s, sweeping laterally
    (`latFrom=-1.6` to `latTo=4.8`, i.e. all the way across BOTH lanes) for the viz only. Registered
    as a static obstacle on **both** `e0_1` (its reference lane) and `e0_0` (`blockLaneIds`) --
    without this, a SUMO car simply lane-changes around a footprint the engine has no lateral
    field to see on the other lane (this repo's own multi-lane keep-right/speed-gain logic is
    otherwise perfectly happy to dodge a single-lane obstacle -- confirmed by an earlier iteration
    of this fixture where only `e0_1` was blocked and `car_ped` slipped onto `e0_0` around it
    instead of stopping). Blocking every lane the pedestrian's footprint currently spans is the
    physically correct model of a full-road crossing anyway.
  - `slowcar1` (car): a moving obstacle on `e0_0`, front pos 80 m at t=0, constant 3.0 m/s,
    active for the whole run.

## Behavioral proof: WITH vs WITHOUT external agents

Same command, only `--agents` differs (a missing/empty agents file is exactly "no external
agents" -- `ExternalAgentsReader.Read` returns an empty list rather than erroring):

```
dotnet run --project src/Sim.ExtDemo -- scenarios/_bench/ext-agents-demo --fcd-out /tmp/with.fcd.xml
dotnet run --project src/Sim.ExtDemo -- scenarios/_bench/ext-agents-demo --agents /tmp/nonexistent.json --fcd-out /tmp/without.fcd.xml
```

**`car_ped`** (approaches the pedestrian at pos 150, back = 149.4):

| t (s) | WITHOUT (free flow) pos / speed | WITH (reacts to ped1) pos / speed |
|------:|---------------------------------:|-----------------------------------:|
| 11    | 122.34 / 13.89                   | 122.34 / 13.89                     |
| 12    | 136.23 / 13.89                   | 135.03 / 12.69 (braking begins)    |
| 13    | 150.12 / 13.89 (drives through)  | 143.21 / 8.19                      |
| 14    | 164.01 / 13.89                   | 146.90 / 3.69                      |
| 15    | 177.90 / 13.89                   | 146.90 / **0.00** (stopped, gap=2.5 = minGap) |
| 16    | 191.79 / 13.89                   | 146.90 / 0.00 (still stopped; ped1 deactivates exactly at t=16) |
| 17    | 205.68 / 13.89                   | 149.50 / 2.60 (resumes)            |
| 22    | 275.13 / 13.89                   | 199.79 / 13.89 (back to free flow) |

WITHOUT the pedestrian, `car_ped` never drops below free-flow speed and drives straight through
pos 150 at t=13. WITH it, it decelerates smoothly from t=12, comes to a full stop at pos 146.899
(the Krauss steady gap behind the pedestrian's back, 149.4 - 2.5 minGap - eps &asymp; 146.899,
same steady-gap math `RungB1ExternalObstacleTests` cross-checks against `scenarios/13-stopped-
leader`), holds for two steps while `ped1` is still active, then resumes and is back to free-flow
speed by t=22.

**`car_follow`** (comes up behind `slowcar1`, front pos = 80 + 3&middot;t):

| t (s) | WITHOUT (free flow) pos / speed | WITH (follows slowcar1) pos / speed |
|------:|---------------------------------:|--------------------------------------:|
| 8     | 80.67 / 13.89                    | 80.67 / 13.89                         |
| 9     | 94.56 / 13.89                    | 91.61 / 10.94 (braking begins)        |
| 10    | 108.45 / 13.89                   | 99.56 / 7.94                          |
| 11    | 122.34 / 13.89                   | 105.03 / 5.47                         |
| 12    | 136.23 / 13.89                   | 108.00 / 3.97                         |
| 13    | 150.12 / 13.89                   | 112.00 / **3.00** (settled at slowcar1's speed) |
| 14    | 164.01 / 13.89                   | 115.00 / 3.00                         |

WITHOUT the slow external car, `car_follow` reaches and holds free-flow speed (13.89 m/s) the
entire run. WITH it, it decelerates starting at t=9 and settles into following `slowcar1` at
EXACTLY 3.0 m/s (matching the moving obstacle's own speed, the Krauss constant-speed-leader
equilibrium) for several seconds -- a sustained ~11 m/s divergence from the free-flow baseline.
(After ~t=15 `car_follow` opportunistically lane-changes to the now-clear `e0_1` and accelerates
away -- a legitimate speed-gain lane change once its current-lane leader is no longer the tightest
constraint; it does not affect the following-speed proof above.)

## Viz

`src/Sim.ExtDemo`'s `CombinedFcdObserver` writes ONE `--fcd-output`-schema file containing both
the SUMO vehicles (from `OnVehicleExported`, byte-for-byte the same fields `FcdWriterObserver`
writes) and the external agents active at that frame (`OnFrameEnd`), computed from the lane
geometry (`LaneGeometry.PositionAtOffset`) plus a perpendicular lateral offset for the visual x/y
only. External agent ids are prefixed `ext_pedestrian_`/`ext_car_`; `src/Sim.Viz/Program.cs`
recognizes that prefix (no rou.xml entry needed -- adding one would make the ENGINE itself
simulate the agent as a real car) and assigns it a fixed vClass string (`ext_pedestrian`/
`ext_car`) and dimensions directly. `src/Sim.Viz/template.js`'s `VCLASS_COLORS` palette maps
those two vClass strings to loud, unmistakable colors (magenta, lime) distinct from every real
SUMO vClass, and draws `ext_pedestrian` as a filled circle instead of an oriented box.

To regenerate `replay.html` for a look (not committed -- see `.gitignore`):

```
dotnet run --project src/Sim.ExtDemo -- scenarios/_bench/ext-agents-demo
dotnet run --project src/Sim.Viz -- scenarios/_bench/ext-agents-demo --fcd scenarios/_bench/ext-agents-demo/engine.fcd.xml
```

## Known limitations (first demo; see the briefing)

- The engine's obstacle API is longitudinal-only (lane id + arc-length pos) -- there is no
  lateral field yet, so a car cannot swerve within its own lane to avoid a pedestrian; the
  reaction is always brake/stop/follow, never a lateral dodge. `blockLaneIds` (registering the
  same crossing pedestrian on every lane it currently spans) is this fixture's workaround for a
  MULTI-lane crossing, not a substitute for real lateral modeling.
- `latFrom`/`latTo` are consumed ONLY by this demo's viz layer, never by the engine.
