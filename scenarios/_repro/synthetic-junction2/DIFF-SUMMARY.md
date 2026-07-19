# synthetic_junction2 — TL-approach gridlock witness

Geometry-free synthetic (10 traffic-light junctions of 140 nodes, heavy demand on short
approaches, `device.rerouting`, park-and-stay sinks) that reproduces the real-box TL gridlock
direction: SumoSharp over-teleports and over-halts at the traffic-light approaches relative to
vanilla SUMO 1.20.0. Deterministic (fixed seed 42).

## Controlled A/B/C measurement (fresh binary, one fix toggled at a time)

All rows are the SAME net/demand; only the engine build differs. Vanilla is the committed
golden reference; the three SumoSharp columns are HEAD with the P2-G fixes progressively disabled:

- **Bug-1** = `<routing>`-section config parsing (device.rerouting honored).
- **Bug-2** = traffic_light junctions excluded from the RBL cycle resolver.
- **Bug-3** = crossing gate no longer yields to a red-light foe.

| metric | vanilla | Bug-1 | Bug-1+2 | Bug-1+2+3 (HEAD) |
|---|---|---|---|---|
| teleports (jam / yield) | 0 (0/0) | 24 (8/16) | 23 (4/19) | **11 (0/11)** |
| peak on-net halting | 45 | 101 | 95 | **83** |

**Arrival curve** (trips cleared — the un-confounded believability signal; raw `halting`
over-counts SumoSharp's parked sinks, which SUMO excludes when parked):

| t | vanilla | Bug-1 | Bug-1+2 | Bug-1+2+3 |
|---|---|---|---|---|
| 199 | 26 | 20 | 26 | 26 |
| 499 | 177 | 150 | 162 | **171** |
| 599 | 241 | 226 | 214 | **231** |
| 699 | 280 | 264 | 264 | **272** |
| 999 | 290 | 285 | 287 | 286 |

## Reading

Both fixes move toward vanilla, Bug-3 the larger share:

- **Bug-2** (RBL TL-exclusion): peak halting 101→95, teleports 24→23, arrival curve mixed
  (better early/mid, slightly worse near t=599). A modest, net-positive improvement; its main
  guarantee is that it keeps all committed goldens byte-identical while removing a class of
  spurious green-link holds at dense TL junctions.
- **Bug-3** (red-foe yield): the bigger lever — teleports 23→11 (jam 4→0), peak halting 95→83,
  and the mid-run arrival gap vs vanilla roughly halved (t=499: −15→−6, t=699: −16→−8).

The whole-box progressive gridlock is materially reduced but **not** fully closed: peak halting is
still 83 vs vanilla's 45, and ~4 trips finish just past the t=1000 cutoff. A residual remains — the
minor-link cautious-approach still slows permissive-green movements more than vanilla (a tempo gap,
not a freeze). The committed `ss.*` outputs are the **HEAD (Bug-1+2+3)** run.

## Measurement note (correction)

An earlier version of this file reported larger deltas (e.g. teleports 42→17, arrivals 277→290).
Those were taken against a **stale self-contained binary** left over from an earlier publish and
are wrong. The table above is the corrected, controlled measurement: each column is the fresh HEAD
build with exactly one fix toggled, run through the framework-dependent shim
(`src/Sim.Sumo/bin/Release/net8.0/sumosharp.dll`), never the stale `linux-x64` publish.

## Reproduce

```
python3 build.py            # regenerates grid.net.xml + scenario.* deterministically
sumo        -c scenario.sumocfg --end 1000 --no-step-log true --summary-output van.sum.xml --statistic-output van.stat.xml
<sumosharp> -c scenario.sumocfg --end 1000 --no-step-log true --summary-output ss.sum.xml  --statistic-output ss.stat.xml --tripinfo-output ss.ti.xml
python3 audit_nocheat.py grid.net.xml scenario.rou.xml scenario.add.xml ss.ti.xml   # PASS
```

All inputs are synthetic (netgenerate `--rand`); vType files are place-scrubbed copies.
