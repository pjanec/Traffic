# Scaled-city benchmark — scaling curve (rungs 1-4, `-L 1`)

`BENCHMARK_SPEC.md` / `VIZ_BENCH_TASKS.md` VB-8. All four rungs share the SAME generator
(`scripts/gen-benchmark.sh <targetConcurrency> [gridNumber] [gridLength] [end]`), the same seed
(42), `sigma=0`, single-lane (`-L 1`) edges (see `city-30/NOTES.md` for why), and Euler
integration. Rungs 2-4 (city-300/3000/15000) additionally share ONE net (`netgenerate --grid
--grid.number=24 --grid.length=500 -L 1 --tls.guess --seed 42`, 1,104 lane-km / 576 junctions),
sized once at the 15,000-concurrent rung per `BENCHMARK_SPEC.md`'s stated preference — see
`city-300/NOTES.md` "Net-sizing choice" for the full reasoning and the empirical capacity probing
that led to this size. `city-30` keeps its original small dedicated 3x3/200m net (unchanged,
already committed).

**Headline finding (read before the table): a genuine `Sim.Core` engine defect, not a benchmark
artifact.** Starting at rung 2 (~300 concurrent), the engine's aggregate comparison against SUMO
fails hard, and the failure is dominated by a specific, root-caused bug —
`Engine.FindFoeVehicle` (`src/Sim.Core/Engine.cs:2279`) treats ANY vehicle whose route passes
through a priority-junction foe lane AT ANY FUTURE POINT as an "approaching foe," with no
proximity or time-window filter. On a small net (city-30, 9 junctions) this rarely fires; on the
576-junction shared net it fires almost everywhere, almost always, producing large stuck-vehicle
counts that are NOT a capacity/gridlock effect (SUMO runs the identical demand at
near-free-flow-to-moderate congestion, never anything like the engine's stuck fraction). Full
write-up, repro, and root cause: `/NEED-priorityjunction-farrouted-foe-falsepositive.md` (repo
root). Per `CLAUDE.md`, this is reported here, not patched — benchmark work does not touch
`Sim.Core`.

## Scaling table

| rung | target N | measured peak concurrent (SUMO) | tuned period (s) | engine RTF | engine steps/s | engine peak RSS | engine stuck (ever / still-at-end) | SUMO teleports | arrived (SUMO / engine) | mean duration s (SUMO / engine) | KS distance | PASS/FAIL |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| city-30   | 30    | 41    | 2.0736  | ~1300-2300x | n/a | ~55 MiB  | 0 / 0     | 0 | 255 / 256 | 67.60 / 63.74  | 0.2175 | **PASS** |
| city-300  | 300   | 485   | 1.2448  | 31.3x | 31.3 | 159.1 MiB | 425 / 404 | 0 | 238 / 46  | 424.65 / 387.98 | 0.1659 | **FAIL** (arrived, meanSpeed) |
| city-3000 | 3000  | 4365  | 0.15725 | DEFERRED | — | — | — | 0 | 3260 / — | 514.49 / — | — | DEFERRED |
| city-15000| 15000 | 17639 | 0.05837 | DEFERRED | — | — | — | 0 | 8010 / — | 616.33 / — | — | DEFERRED |

**city-3000 / city-15000 engine runs are DEFERRED**, not pending. Their SUMO references
(`summary.xml`/`tripinfo.xml`) are committed as ground truth, but running the ENGINE at those rungs
is postponed until the `FindFoeVehicle` defect (headline finding above,
`/NEED-priorityjunction-farrouted-foe-falsepositive.md`) is fixed: with that bug present, a larger
rung only measures a worse false-positive-yield stall, so the aggregate comparison carries no signal
and the (expensive) runs aren't worth the wall-clock yet. Re-run them once the fix lands — the
committed net/rou/config/SUMO-refs make each a one-command engine run. city-300 was run to
completion because it is the smallest rung that already exhibits the defect unambiguously (the
proof-of-bug rung).

## Net-capacity / stability findings (SUMO reference side)

All rungs' SUMO reference reached their target concurrency band via the collapse-aware Little's
law tuning in `scripts/gen-benchmark.sh` (see that script's "COLLAPSE DETECTION" comment, added
after directly reproducing several candidate nets going into unbounded queueing collapse during
this session's net-sizing exploration — not merely theorized). None of the four committed rungs'
FINAL tuned configuration collapsed; city-15000's SUMO reference settles into a busy but
non-collapsing plateau (`meanSpeedRelative` ~0.45-0.5, `halting`/`running` ~45-47% at the end of
the 1500s window, `arrived` still climbing steadily — congested, not gridlocked).

## Engine-side stability findings

See the headline finding above and `/NEED-priorityjunction-farrouted-foe-falsepositive.md`. The
engine's stuck-count is expected to grow sharply from rung to rung because the underlying defect's
severity scales with network size (fixed at 576 junctions for rungs 2-4) and concurrent vehicle
count (which increases every rung) — this is a real correctness gap exposed at scale, not a
benchmark-harness artifact, and no amount of net-size or insertion-period retuning works around it
(confirmed: city-300 fails despite its SUMO reference being near-free-flow).
