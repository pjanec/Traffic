# synthetic_junction — SumoSharp Issue 2 repro: DIFF-SUMMARY

Geometry-free synthetic net (`netgenerate --rand`, no real road data). Both engines
run the IDENTICAL scenario with identical flags:

    -c scenario.sumocfg --end 1000 --no-step-log true
    --summary-output --statistic-output --tripinfo-output [--fcd-output]

Engines: vanilla Eclipse SUMO 1.20.0  vs  SumoSharp build 2cc2405.

## Headline — jam-teleport divergence (Issue 2)

| metric                         | vanilla | SumoSharp |
|--------------------------------|--------:|----------:|
| **jam teleports**              | **0**   | **75**    |
| yield teleports                | 3       | 0         |
| total teleports                | 3       | 75        |
| mean meanSpeedRelative         | 0.478   | 0.265     |
| trips completed (tripinfo)     | 475     | 326       |
| mean trip speed (m/s)          | 7.61    | 6.11      |
| running at sim end             | 60      | 141       |
| vehicles loaded / inserted     | 535 / 535 | 535 / 535 |

Vanilla stays near free-flow (rel-speed ~0.48) and fires **zero jam teleports**.
SumoSharp jam-teleports **75×** — the same direction as the real box (105 vs 1),
whereas the pre-existing uniform 8×8 grid false-greens at 0-vs-0.

## The tell — yield vs jam classification
Vanilla accounts its long unsignalized-junction waits as **yield** teleports (3, and
mostly not at all) — waiting for a gap/right-of-way is expected. SumoSharp accounts
the very same waits as **jam** teleports (75, yield=0). The divergence is the
CLASSIFICATION of a junction-yield wait, not the amount of congestion. This matches
the real-box diagnosis: both engines congest comparably; only SumoSharp converts
junction-yield waits into jam teleports once wait > time-to-teleport (120 s).

## No-cheating audit (SumoSharp run)
    NO-CHEATING AUDIT: PASS   (0 birth, 0 death, 0 FCD-birth violations)
All births/deaths at fringe stubs or off-road parkingAreas.

## Files in this bundle
- scenario inputs: `grid.net.xml scenario.rou.xml scenario.add.xml scenario.sumocfg`,
  `vType.config.xml vType_pedestrians.xml vTypeDist.config.xml`, `build.py`, `README.md`
- `audit_nocheat.py`
- per-engine outputs: `{van,ss}.sum.xml` (summary), `{van,ss}.stat.xml` (statistic),
  `{van,ss}.ti.xml` (tripinfo), `{van,ss}.log`
- FCDs OMITTED (ss ~32 MB + van ~15 MB > 30 MB combined). Regenerate by adding
  `--fcd-output van.fcd.xml` / `--fcd-output ss.fcd.xml` to the two run commands below.

## Reproduce
    export SUMO_HOME=/path/to/sumo ; export PYTHONPATH=$SUMO_HOME/tools
    python3 build.py                      # deterministic, seed 42
    sumo      -c scenario.sumocfg --end 1000 --no-step-log true --statistic-output van.stat.xml --summary-output van.sum.xml --tripinfo-output van.ti.xml
    sumosharp -c scenario.sumocfg --end 1000 --no-step-log true --statistic-output ss.stat.xml  --summary-output ss.sum.xml  --tripinfo-output ss.ti.xml
    # grep teleports {van,ss}.stat.xml -> jam=0 vs jam=75
