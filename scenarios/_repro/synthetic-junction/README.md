# synthetic_junction — junction-realistic SumoSharp Issue 2 repro

A **geometry-free synthetic** scenario (no real road-network data) that reproduces the
SumoSharp **"Issue 2" jam-teleport divergence**. It is the junction-realistic
counterpart to `../synthetic_parity`: the uniform 8×8 grid there false-greens Issue 2
(0 jam-teleports under both engines), because it lacks the junction micro-geometry that
actually triggers the bug.

## What it reproduces

| metric (identical scenario + flags) | vanilla SUMO 1.20.0 | SumoSharp 2cc2405 |
|-------------------------------------|--------------------:|------------------:|
| **jam teleports**                   | **0**               | **75**            |
| yield teleports                     | 3                   | 0                 |
| mean meanSpeedRelative              | 0.48 (free-flow)    | 0.27              |

Same direction as the real box (105 vs 1). Vanilla stays near free-flow and fires **zero
jam teleports**; SumoSharp jam-teleports 75×.

## Why this net triggers it (diagnosis in one paragraph)
On the real box both engines congest comparably, but only SumoSharp converts long
UNSIGNALIZED-junction yield-waits into jam teleports (vanilla logs them as *yield* waits
and tolerates them). The vehicles that wedge share a feature profile: unsignalized node
(priority / right_before_left), conflicting foe streams forcing a yield, single-lane
minor approaches, **short approach edges (<30 m, many <5 m)** so a stopped car spills back
and blocks the upstream junction, and 1↔2 lane-count drops. This net embeds that profile
via `netgenerate --rand` with small min/max edge distances, `--random-lanenumber`,
`--random-priority`, and `--default-junction-type priority`. The scenario also carries
multi-occupant parkingArea sinks + park-and-stay residents + departPos=stop origins (so it
stays a valid no-visible-cheating sub-area run and also touches Issue 1), but the point is
Issue 2.

`build.py` is deterministic (seed 42) and regenerates every input.
Pass `--jtype right_before_left` to build the RBL variant (also reproduces: 0 vs 65).

## The two commands (run from this directory)

    export SUMO_HOME=/path/to/sumo
    export PYTHONPATH=$SUMO_HOME/tools

    # 1. vanilla reference (jam teleports = 0)
    sumo -c scenario.sumocfg --end 1000 --no-step-log true \
        --statistic-output van.stat.xml --summary-output van.sum.xml --tripinfo-output van.ti.xml

    # 2. SumoSharp (jam teleports = 75)
    /path/to/sumosharp -c scenario.sumocfg --end 1000 --no-step-log true \
        --statistic-output ss.stat.xml --summary-output ss.sum.xml --tripinfo-output ss.ti.xml

    grep teleports van.stat.xml ss.stat.xml     # jam="0"  vs  jam="75"

## No-cheating audit (must PASS)

    python3 audit_nocheat.py grid.net.xml scenario.rou.xml scenario.add.xml ss.ti.xml ss.fcd.xml
    # -> NO-CHEATING AUDIT: PASS   (add --fcd-output ss.fcd.xml to the ss run for the
    #    authoritative birth check; drop the last arg for the route-intent-only audit)

## Rebuild from scratch

    python3 build.py            # writes grid.net.xml, scenario.{rou,add,sumocfg}.xml, vType*.xml

## Files
- `build.py` — deterministic generator (the net has no real geometry)
- `grid.net.xml` — synthetic irregular net (`netgenerate --rand`)
- `scenario.rou.xml`, `scenario.add.xml`, `scenario.sumocfg` — demand, parking, config
- `vType.config.xml`, `vTypeDist.config.xml`, `vType_pedestrians.xml` — vehicle types
  (copied from the fixture, place-labels scrubbed)
- `audit_nocheat.py` — no-visible-cheating verifier
