# PEDESTRIAN-POC7C-FINDINGS.md — integrated LOD CPU-scale acceptance

**Status: measured.** Final POC-7 sub-part. Companion to `PEDESTRIAN-POC7A-FINDINGS.md` (parallel
`OrcaCrowd.Step`) and `PEDESTRIAN-POC7B-FINDINGS.md` (single-stream bandwidth). Measures the *integrated*
LOD system (`PedLodManager` driving ~10 % high-power `OrcaCrowd` agents + ~90 % low-power `PathArc`
path-followers) at the 100k target, via `src/Sim.BenchPedLod`.

**Hardware caveat (as in POC-7a):** this session's VM has **4 logical processors**. The owner target is a
**16+‑core Windows box**. Numbers in the original sections below are 4-core measurements; treat the
*ratios and shapes* as the finding. **The on-target measurement now exists** (P6-1) — see the
**"ON-TARGET (P6-1)"** section at the end of this doc and `docs/PEDESTRIAN-P6-1-RESULTS.md`; the headline
is stable 100k = **27.9 steps/s** and churn 100k = **18.2 steps/s** parallel on a 24-core Core Ultra 9
275HX, both interactive.

Benchmark command (Release):
`dotnet run -c Release --project src/Sim.BenchPedLod -- --sizes 20000,50000,100000 --high-fraction 0.1 --steps 30 --warmup 8`

---

## Q1 — does ms/step scale with the HIGH-power set, or with the TOTAL?

| config | actual high-power | serial ms/step | parallel ms/step | speedup |
|---|---:|---:|---:|---:|
| 10 000 high-power only (0 low-power) | 6 714 | 142.0 | 64.2 | 2.21× |
| 100 000 total (~10k high + ~90k low) | 10 477 | 238.1 | 75.2 | 3.17× |

**Verdict: cost scales with the high-power set, not the total.** Adding **90 000 low-power** `PathArc`
path-followers on top of the high-power crowd raised the parallel step time only from ~64 ms to ~75 ms
(≈ **+11 ms for 90k agents** — and the 100k run even had *more* high-power agents, 10 477 vs 6 714, which
accounts for much of the difference). The low-power tier is genuinely O(1)/step (no neighbour queries, no
ORCA), so the 100k world costs essentially what its ~10k high-power subset costs. **This is the core LOD
thesis (design §5/§9), confirmed empirically.**

## Main sweep — Scenario A (stable membership) vs Scenario B (churning membership)

| N total | scenario | actual high | switches/step | serial ms/step | parallel ms/step | speedup |
|---:|---|---:|---:|---:|---:|---:|
| 20 000 | A / stable | 2 098 | 70 | 19.1 | 7.7 | 2.50× |
| 20 000 | B / churn | 5 254 | 175 | 53.3 | 17.6 | 3.02× |
| 50 000 | A / stable | 5 235 | 175 | 83.8 | 26.9 | 3.11× |
| 50 000 | B / churn | 12 160 | 614 | 321.9 | 94.1 | 3.42× |
| 100 000 | A / stable | 10 477 | 349 | 258.3 | 80.1 | 3.23× |
| 100 000 | B / churn | 23 515 | 1 379 | 1 048.4 | 285.5 | 3.67× |

## Q2 — is promotion churn a bottleneck? (YES — and it quantifies the Add/Remove backlog)

At 100k, the churning scenario costs **285 ms/step vs 80 ms/step stable — 3.6× worse.** Two effects
combine, and both matter:

1. **More high-power agents.** The moving interest sources sweep through the crowd, so Scenario B holds far
   more agents high-power at once (23 515 vs 10 477 at 100k). Part of B's cost is simply *more real ORCA
   work* — legitimate, not overhead.
2. **Rebuild-on-membership-change overhead.** Every promotion/demotion triggers `PedLodManager.RebuildHighCrowd`,
   which rebuilds the *entire* high-power `OrcaCrowd` from scratch (the POC-3/POC-6 workaround for
   `OrcaCrowd` having no `Add`/`Remove`). At **~1 379 switches/step** that rebuild runs constantly, and its
   cost is O(current high-power count) *per switch*. This is the dominant avoidable cost in B.

**This is the concrete, quantified motivation for the deferred crowd-API work (design §3d).** A genuine
`Add`/`Remove` on `OrcaCrowd` (free-list + generation, or stable-slot deactivate/reactivate) replaces the
O(high)-per-switch full rebuild with O(1)-per-switch, which would remove most of the ~205 ms gap between B
and A at 100k. It is the single highest-value follow-up for the interest-sources-move-constantly case
(which is the *normal* case: a player avatar or IG camera sweeping the city, §5). Stable and lightly-
churning worlds already run without it.

## Q3 — interactive-rate verdict + combined acceptance picture

- **On this 4-core VM (original, pre-P0 code):** stable 100k ≈ **80 ms/step (~12.5 steps/s)** parallel;
  heavy-churn 100k ≈ **285 ms/step (~3.5 steps/s)**.
- **ON-TARGET (P6-1, current post-P0-3/P0-4 code, 24-core Core Ultra 9 275HX):** stable 100k ≈ **35.9
  ms/step (27.9 steps/s)** parallel; heavy-churn 100k ≈ **54.9 ms/step (18.2 steps/s)** parallel — **both
  firmly interactive.** See the "ON-TARGET" section below and `docs/PEDESTRIAN-P6-1-RESULTS.md`. The
  extrapolation the next bullet made turned out conservative: the stable case is comfortably real-time and
  the once-worst churn case is now within the interactive band on-target.
- **Extrapolated to the 16+‑core target (the original projection, now superseded by the measured row above):**
  POC-7a measured near-linear scaling up to the box's core count
  and the lane engine reaches ~3.2–3.6× at 8 threads on a 16c/24t box (`PERF-HANDOVER.md`, memory-bandwidth
  bound). So the stable 100k case should reach comfortably interactive rates on the target hardware; the
  heavy-churn case is where the Add/Remove fix pays off and should be prioritized before churn-heavy
  production loads.
- **Combined with POC-7b (bandwidth):** bandwidth is *not* the constraint — even the worst-case all-100k-
  promoted spike is 182 Mbit/s against the ~500 Mbit/s budget (63 % headroom). **CPU is the acceptance
  constraint, the LOD split makes the stable target tractable, and `OrcaCrowd.Add/Remove` is the priority
  optimization for constant-churn workloads.**

---

## Consolidated POC-7 summary

- **7a (CPU parallel):** `OrcaCrowd.Step` parallelized opt-in, **bit-identical** to serial (2000-agent ×
  300-step exact-equality gate); ~3–4× on a 4-core VM, near-linear to core count. Lane parity untouched.
- **7b (bandwidth):** quantized 18 B `PedFreeKinematicRecord` + `PathArc` record; the single DDS-multicast
  stream measures **36.75 Mbit/s typical / 182 Mbit/s worst-case spike / 294 Mbit/s naive** — all under the
  500 Mbit/s budget. Bandwidth is solved.
- **7c (integrated CPU scale):** the LOD split makes 100k tractable (cost tracks the ~10k high-power set,
  not the 100k total); **membership churn via `RebuildHighCrowd` is the main remaining CPU cost**, which
  concretely justifies building real `Add`/`Remove` on the crowd store as the top follow-up.

**Acceptance:** the design's scale thesis holds — 100k pedestrians + 10k cars is tractable on CPU (LOD) and
comfortable on bandwidth (multicast + quantization). The one identified priority follow-up before heavy-
churn production is efficient crowd `Add`/`Remove` (design §3d), now quantified here.

---

## P0-3 update — incremental Add/Remove vs the rebuild (measured, same-VM A/B)

P0-3 replaced `PedLodManager.RebuildHighCrowd` (rebuild the whole high-power crowd + re-route every
high-power ped on each membership change) and `LotCoupling`'s per-step car-crowd rebuild with incremental
`OrcaCrowd`/`MixedTrafficCrowd` `Add`/`Remove` (P0-1/P0-2). To measure P0-3's effect WITHOUT cross-session
VM variance, the benchmark was run **back-to-back on the same VM**, P0-3 stashed (rebuild) vs applied
(incremental), on the **identical scenario** (same `actualHigh` and switches/step):

| 100k total | rebuild (pre-P0-3) | incremental (P0-3) | Δ |
|---|---:|---:|---:|
| A/stable (523.85 switches/step, 10477 high) | 141.8 ms/step | 144.6 ms/step | ~neutral (within noise) |
| B/churn (1578 switches/step, 27444 high) | 514.8 ms/step | 467.95 ms/step | **~9% faster** |

**Honest reading:** P0-3 is a **net win on churn (~9%) and neutral on stable** — no regression. It is
**smaller than POC-7c projected** (the doc implied churn was dominated by rebuild overhead). The same-VM
data shows otherwise: most of B/churn's extra cost over A/stable is genuinely stepping **~2.6× more ORCA
agents** (27444 vs 10477 — the sweeping interest sources simply hold far more peds high-power), not rebuild
overhead, so removing the rebuild only recovers ~9%. (The earlier apparent "regression" to 174/660 ms was
cross-session VM/host variance — comparing to POC-7c's original 285 ms, measured on a different VM, is
invalid; the valid measure is the same-VM before/after above.)

**Follow-up hypothesis (a P6 perf item, not P0-3):** the persistent crowd's slot high-water (`_count`) never
shrinks after a churn spike, so `Step`/`RebuildGrid` iterate vacated slots (cheaply skipped, but still
O(high-water) not O(live)). A maintained dense ascending live-slot list — or shrinking the high-water when
top slots free — would make cost track LIVE count and could widen P0-3's churn margin. Correctness and the
architectural benefit (O(1) per switch, no re-routing every ped each change) already hold; this is a pure
throughput refinement.

---

## P0-4 — turn on the crowd's spatial hash (root cause: the high-power crowd was brute-force O(n²))

**Root cause.** `PedLodManager` constructed its persistent high-power `OrcaCrowd` bare (`new OrcaCrowd()`),
so `UseSpatialHash` defaulted to `false` and every `Plan()` neighbour gather brute-force-scanned the WHOLE
high-power crowd (O(n) per agent, O(n²) per step) instead of using the proven bit-identical spatial-hash
pre-filter that had existed since the P0-1/Q3 era (`OrcaSpatialHashTests`) but was simply never turned on
here. `LotCoupling`'s two bare crowds (`_peds: OrcaCrowd`, `_carCrowd: MixedTrafficCrowd`) had the same gap.

**Part A change.** `PedLodManager`'s persistent `_highCrowd` is now constructed with `UseSpatialHash = true`
(`src/Sim.Pedestrians/Lod/PedLodManager.cs`). It carries no static obstacles, so `UseObstacleSpatialIndex`
is left off (nothing for it to accelerate). `LotCoupling`'s own bare `_peds` gets both `UseSpatialHash =
true` and `UseObstacleSpatialIndex = true` (P2-1 — it DOES carry static parked-car-box obstacles via
`AddParkedCarBox`); its bare `_carCrowd` gets `UseSpatialHash = true` too. A caller-supplied `peds` crowd
(the `LotCoupling(OrcaCrowd? peds)` constructor parameter) is left untouched — only the crowd instances
this code creates itself are "bare" in the sense the task means. `MaxNeighbours` was deliberately NOT set
anywhere (that caps the neighbour set — a behavioral change) so this stays a pure, bit-identical pre-filter
change; both spatial-hash mechanisms are proven bit-identical by construction (candidates gathered from the
grid are sorted into the exact same order the brute-force scan would visit them in).

**Correctness gate.** Full `dotnet test Traffic.sln` (Release, no-build): **585 passed + 3 skipped (parity)
+ 72 (Sim.Pedestrians.Tests) + 2 (Sim.Pedestrians.Nav.DotRecast.Tests) + 1 (Sim.Host.Tests) — all green,
zero failures**, identical to the pre-change count. Every `PedLodManagerTests` / `LotCoupling*` test
(`LotCouplingDeterminismTests`, `LotCouplingMutualAvoidanceTests`, `LotCouplingCarYieldsTests`) passed
unchanged — these assert exact/deterministic trajectories, so this confirms the crowds behave identically,
only faster. `git diff` touches only the three authorized files (`PedLodManager.cs`, `LotCoupling.cs`,
`Sim.BenchPedLod/Program.cs` — the last for benchmark reporting only, see below).

**Measurement discipline.** All numbers below were taken on this session's 4-core VM, in isolation:
`dotnet build-server shutdown` + `kill -9` on any lingering MSBuild/VBCSCompiler node processes before each
run (leftover node-reuse processes from a prior `dotnet build` were confirmed via `pgrep -a dotnet` and
killed — they are NOT part of a `dotnet test`/build run but were still resident and could contend), then
`dotnet build -c Release` once, `dotnet run -c Release --no-build --project src/Sim.BenchPedLod -- --sizes
20000,50000,100000 --high-fraction 0.1 --steps 30 --warmup 8`. "Before" was measured on the unmodified
pre-P0-4 commit; "after" on this change, both back-to-back on the same idle VM. Each side was run twice to
confirm reproducibility; the numbers below are representative single runs (see honest caveats below for
where repeats disagreed).

### Before / after — Verdict Q1 isolation (does cost scale with the HIGH set or the TOTAL)

| config | actualHigh | serial before → after | parallel before → after | serial×/parallel× |
|---|---:|---|---|---|
| 10 000 high-only (0 low) | 6 714 | 251.3 → 192.6 ms | 69.7 → 55.9 ms | 1.30× / 1.25× |
| 100 000 total (~10k high + ~90k low) | 10 477 | 537.3 → 239.2 ms | 167.3 → 94.4 ms | 2.25× / 1.77× |

### Before / after — main sweep

| N total | scenario | actualHigh | serial before → after | parallel before → after | serial×/parallel× |
|---:|---|---:|---|---|---|
| 20 000 | A/stable | 2 098 | 39.4 → **46.9–56.9** ms | 16.3 → **20.4–21.4** ms | **0.7–0.8× (SLOWER)** |
| 20 000 | B/churn | 5 254 | 107.3 → 91.6–92.1 ms | 33.3 → 30.4–33.8 ms | ~1.1–1.2× (noisy/flat) |
| 50 000 | A/stable | 5 235 | 164.9 → 118.7–124.5 ms | 55.9 → 44.5–48.0 ms | 1.32–1.39× / 1.17–1.26× |
| 50 000 | B/churn | 12 160 | 649.6 → 274.8–304.4 ms | 186.0 → 91.5–98.0 ms | 2.13–2.36× / 1.90–2.03× |
| 100 000 | A/stable | 10 477 | 548.6 → 232.9–254.2 ms | 169.9 → 88.5–102.3 ms | 2.16–2.36× / 1.66–1.92× |
| 100 000 | B/churn | 23 515 | 2537.4 → 537.7–542.9 ms | 680.3 → 182.9–184.5 ms | **4.67–4.72× / 3.68–3.72×** |

**Headline result: the worst-case scenario wins biggest.** 100k B/churn — the scenario the whole
POC-7c → P0-3 chain was fighting (P0-3 only shaved ~9% off it) — drops from **~680 ms/step to ~183
ms/step parallel** (~1.5 steps/s → ~5.5 steps/s on this 4-core VM), a **~3.7× parallel / ~4.7× serial**
win, because it holds the most high-power agents spread across the most simultaneously-active interest-
source neighbourhoods (more separate regions for the grid to let each agent skip entirely). 100k A/stable
gets a solid but smaller ~2.2–2.4× serial / ~1.7–1.9× parallel.

**Honest finding #1 — the win is real but smaller than a naive O(n²)→O(n) framing predicts, and WHY.**
An isolated probe (`OrcaCrowd` alone, no `PedLodManager` overhead, 10 clusters × ~1048 agents at the
bench's own density/promote-radius, matching the 100k scenario's geometry exactly) measured only **~1.3–
1.4×** from the hash — not 10×. Reason: this benchmark's per-source promote radius (~25.6 m, sized to
hold ~1000 peds) is only ~1.7× `NeighbourDist` (15 m default), so a 3×3 grid block (45×45 m) already
spans almost an entire cluster — the grid mainly earns its keep by letting an agent skip the OTHER 9
clusters entirely (not by shrinking its own cluster's candidate list much), and that saving is partly
offset by the per-step grid rebuild plus the per-agent `Array.Sort` over each ~1000-strong candidate list
(needed for the bit-identical ordering guarantee). More separate high-power regions → more savings (this
is exactly why 100k, with 10 well-separated clusters, wins far more than 20k, with only ~2).

**Honest finding #2 — a REAL regression at the smallest scale tested.** 20 000-total A/stable (only 2 098
high-power agents in ~2 clusters) is measurably **slower** with the hash on, reproduced across two
separate runs (39.4 ms before vs. 46.9 and 56.9 ms after, serial; 16.3 ms before vs. 20.4 and 21.4 ms
after, parallel). With only ~2 clusters, brute-force wastes iterating just ~1 other cluster per agent —
too little for the grid-rebuild + per-agent-sort overhead to earn back. This is below the design's ~10k
high-power target scale (docs/PEDESTRIAN-DESIGN.md §9) and does not threaten the acceptance case, but it
is a real, reproducible result and is recorded here rather than glossed over: **`UseSpatialHash` is a net
win at the design's target scale (≥~5k well-separated high-power agents) and a net loss on small/sparse
high-power sets.** A future refinement (not implemented here — out of scope for P0-4, which is
authorized only to flip the existing flag, not add new heuristics) could gate `UseSpatialHash` on crowd
size or cluster count; not worth the complexity until a real deployment shows it matters.

### Part B — live-slot compaction: investigated, NOT implemented (Part A subsumes it)

Added a diagnostic-only `PedLodManager.HighCrowdSlotHighWater` (mirrors `OrcaCrowd.Count`'s existing
high-water semantics) and wired it into `Sim.BenchPedLod`'s churn rows to measure the bloat the P0-3
follow-up hypothesis worried about, WITH the spatial hash now on:

| N total (churn) | live (`HighPowerCount`) | slot high-water | bloat | bloat as % of live |
|---:|---:|---:|---:|---:|
| 50 000 | 12 160 | 15 024 | 2 864 | ~23.6% |
| 100 000 | 23 515 | 28 675 | 5 160 | ~21.9% |

The high-water mark IS bloated ~22–24% above the live count under sustained churn, confirming the
hypothesis's premise. But every place that iterates it (`Step`'s plan/execute loops, `RebuildGrid`) skips
a vacated slot with a single `!_slotAlive[i]` bool-array read + branch — no distance math, no solver work,
no allocation. At ~5 160 extra skipped slots against a ~183 ms/step parallel budget (100k churn, hash
on), that overhead is on the order of microseconds, i.e. **well under 0.1% of the measured step cost** —
nowhere near the dominant O(n²) brute-force cost Part A just removed. **Verdict: Part A subsumes the
churn-bloat concern. Compaction is NOT implemented** — the measured benefit would be negligible and not
worth the added complexity (a dense live-slot list, or high-water-shrink-on-trailing-vacate, both need
their own bit-identical-ordering argument for no real payoff). If a future profiling pass on REAL (not
synthetic) production churn shows otherwise, this diagnostic getter is already in place to re-check.

---

## ON-TARGET (P6-1) — the 4-core estimates replaced by a real 16+‑core run

Measured on the owner target — **Intel Core Ultra 9 275HX, `ProcessorCount = 24`** (8 P + 16 E cores, no
SMT), Windows 11, High-performance power plan, Release, **current code** (post P0-3 incremental
`Add`/`Remove` + P0-4 spatial hash), each config 3× with the **median** reported. Command:
`--sizes 20000,50000,100000 --high-fraction 0.1 --steps 30 --warmup 8`. Full report:
`docs/PEDESTRIAN-P6-1-RESULTS.md`.

**Main sweep (runtime-auto parallelism):**

| N total | scenario | actual high | switches/step | serial ms/step | parallel ms/step | parallel steps/s |
|--------:|----------|------------:|--------------:|---------------:|-----------------:|-----------------:|
| 20,000  | A/stable | 2,098       | 70            | 27.9           | 8.25             | 121              |
| 20,000  | B/churn  | 5,254       | 175           | 55.1           | 13.9             | 71.9             |
| 50,000  | A/stable | 5,235       | 175           | 95.4           | 25.3             | 39.5             |
| 50,000  | B/churn  | 12,160      | 614           | 265.2          | 33.1             | 30.2             |
| 100,000 | A/stable | 10,477      | 349           | 158.7          | **35.9**         | **27.9**         |
| 100,000 | B/churn  | 23,515      | 1,379         | 334.3          | **54.9**         | **18.2**         |

**Same-code core-count scaling (this run vs the 4-core P0-4 numbers above, NOT the pre-P0 80/285 ms):**

| 100k parallel | 4-core (post-P0-4) | 24-core (P6-1) | scaling |
|---|---:|---:|---:|
| A/stable | ~88–102 ms | 35.9 ms | ~2.6× |
| B/churn  | ~183 ms    | 54.9 ms | ~3.3× |

**Q1 confirmed on-target:** 10k-high-only = 16.9 ms parallel; 100k-total (same high set + 90k low) = 33.6
ms parallel — +~17 ms for 90k low-power followers. Cost still tracks the high-power set, not the total.

**Two honest on-target notes:**
- **The PedLod `--max-parallelism` knob is a documented no-op** (`Sim.BenchPedLod/Program.cs` ~L187 —
  `PedLodManager` runs its high crowd at runtime-auto). A `--max-parallelism 8` run returned the same
  stable-100k time as auto (36.9 vs 35.9 ms), confirming it. The pedestrian thread-scaling curve is
  therefore the `Sim.BenchCrowd` sweep (POC7A-FINDINGS "ON-TARGET" section) — that is the same
  `OrcaCrowd.Step` the high-power set runs.
- **Churn wall-clock is variance-dominated on-target:** B/churn serial swings ±40% run-to-run (334 / 501 /
  276 ms at 100k) because `RebuildHighCrowd`/membership churn is GC- and allocation-heavy. The parallel
  churn figure (~54–55 ms) is stable. This variance is itself a signal that churn's residual cost is
  allocation/GC-bound, not compute-bound — consistent with the P0-3 finding that most of churn's extra
  cost over stable is genuinely stepping more ORCA agents, not rebuild overhead.

**Verdict:** all three POC-7c acceptance questions resolve favourably on-target. Stable 100k is comfortably
interactive (27.9 steps/s); the once-worst heavy-churn 100k is now interactive too (18.2 steps/s), which is
the payoff of the P0-1/P0-3 `Add`/`Remove` + P0-4 spatial-hash chain this doc motivated. CPU remains the
acceptance constraint (bandwidth is solved, POC-7b), and the LOD split makes the 100k target tractable.
