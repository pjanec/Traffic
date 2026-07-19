# PEDESTRIAN-P6-2-RESULTS.md — on-target perf validation of OrcaCrowd region decomposition

**Status: MEASURED — region-task decomposition is bit-identical but FALLS SHORT of the perf target.**
The per-core uplift target (≥1.4× at 4–8 cores; heavy churn clearing real-time with ≥1.5× margin at 6 ped
cores) is **not met** — the as-built region-*task* scheduling (no SoA reorder) is neutral-to-slightly-
negative vs the flat parallel path. **Per the P6-2-4 decision rule, this is the signal to add the phase-2
SoA-reorder refinement** (design `PEDESTRIAN-P6-2-REGION-DESIGN.md` §8). Determinism is untouched.

Validates `OrcaCrowd.UseRegionDecomposition` (P6-2-1/2/3/5) on the 24-core target box. Companion to
`PEDESTRIAN-P6-1-RESULTS.md` and `PEDESTRIAN-COMBINED-LOAD-RESULTS.md` (which established the GO).

## Box / method

| | |
|---|---|
| CPU | Intel Core Ultra 9 275HX, `ProcessorCount=24` (8 P-cores + 16 E-cores, no SMT) |
| Power plan | High performance; build `-c Release`, net8.0 |
| Core capping | affinity mask + `DOTNET_PROCESSOR_COUNT=N` (as in the combined-load run); flat and region variants use the **identical mask** at each cap, so the flat-vs-region ratio isolates the algorithm |
| Reps | median of 3 |
| Flags | `Sim.BenchCrowd`/`Sim.BenchPedLod --region-decomp [--region-mult K]` (the parallel path becomes region-decomposed; serial column unchanged) |
| Region path engaged | high-power/agent counts (10k–100k) ≫ the 256 parallel threshold, and the benches print `REGION-DECOMP (mult=K)` — confirmed exercising the region path, not a silent fallback |

## 3. Determinism (checked first — untouched) ✅

- `OrcaRegionDecompositionTests` (7) + `RegionPartitionTests` (7) + `PedLodRegionDecompositionTests` (1):
  **all green** — the region path is asserted **bit-identical to serial** every agent every step across
  region sizes {1,2,3,4,8×NeighbourDist}, thread caps, spatial-hash on/off, and the MaxNeighbours+removal
  combination.
- Full offline gate (`dotnet test Traffic.sln -c Release`): **649 parity (+3 skip) / 143 ped / 2 DotRecast
  / 1 host — all pass.** Region decomp is default-off, so the goldens are unaffected by construction.

## 1. Raw ORCA scaling (`BenchCrowd`, 100k, `--steps 20 --warmup 3`)

Parallel steps/s, flat vs region-decomp; ratio = region ÷ flat (>1 = region faster). Median of 3.

| cap | flat st/s | region m2 | region m4 | region m8 | best ratio |
|----:|----------:|----------:|----------:|----------:|-----------:|
| 4  | 6.6  | 6.5 (0.99×) | 6.3 (0.96×) | 5.9 (0.89×) | **0.99×** |
| 6  | 10.7 | 11.0 (1.03×) | 11.0 (1.03×) | 10.6 (0.99×) | **1.03×** |
| 8  | 13.7 | 13.3 (0.97×) | 13.3 (0.97×) | 12.8 (0.93×) | **0.97×** |
| 16 | 19.0 | 17.3 (0.91×) | 17.1 (0.90×) | 16.1 (0.85×) | **0.91×** |
| 24 | 22.4 | 20.3 (0.91×) | 19.9 (0.89×) | 19.1 (0.85×) | **0.91×** |

Region-decomp is **neutral-to-slower** at every cap (best +3% @6c, within noise; a real ~9% *regression*
at 16–24c). `--region-mult 2` (finest regions) is consistently best, `mult 8` worst.
(Absolute st/s here are lower than P6-1's `--max-parallelism` sweep because these are pinned to a *specific*
mixed-P/E core budget rather than letting the scheduler pick fast P-cores — the flat-vs-region ratio is
unaffected, both share the mask.)

## 2a. Integrated LOD churn (`BenchPedLod`, 100k, `--steps 30 --warmup 8`, isolated) — the target metric

Heavy-churn steps/s, flat vs region-decomp; uplift = region ÷ flat. Real-time bar = 10 st/s. Median of 3.

| ped cap | flat churn | region m2 | region m4 | region m8 | best uplift | churn clears RT? |
|--------:|-----------:|----------:|----------:|----------:|------------:|:----------------:|
| 4 | 7.3  | 7.9 (1.08×) | 7.5 (1.04×) | 7.4 (1.02×) | **1.08×** | no (7.9 < 10) |
| 6 | 12.0 | 12.2 (1.02×) | 11.5 (0.96×) | 10.9 (0.92×) | **1.02×** | yes, ~1.2× margin |
| 8 | 14.8 | 14.5 (0.98×) | 14.2 (0.96×) | 13.3 (0.90×) | **0.98×** | yes |

Stable (reference; clears RT everywhere): best region uplift 4c 1.10×, 6c 1.01×, 8c 0.91×.

Even on the LOD scenario — where the high-power crowd sits in ~10 *separated* interest clusters (a more
favourable geometry for region decomposition than BenchCrowd's single dense counter-crossing blob) — churn
uplift peaks at **1.08× (4 cores)** and vanishes/regresses by 8 cores. The **≥1.4× target is not met**, and
6-core churn (12.2 st/s, ~1.2× margin) is far from the **≥1.5×-margin (≥15 st/s)** goal — essentially the
flat baseline.

## 2b. Concurrent re-run, ped region-decomp (mult 2), 6+6 & 8+4 splits

Ped side region-decomposed, vehicle side unchanged (`--region`). Compared to the flat combined-load result.

| split | engine | flat (prior) st/s | region-decomp st/s | clears RT? |
|---|---|---:|---:|:--:|
| **6+6** | ped churn (6c) | 11.1 | **10.9** | ✅ (~1.1× margin, unchanged) |
| | ped stable (6c) | 18.7 | 18.6 | ✅ |
| | vehicles (6c) | 47.6 | 45.9 | ✅ |
| **8+4** | ped churn (4c) | 5.5 | **6.8** | ❌ still fails (−32% under RT) |
| | ped stable (4c) | 10.4 | 12.1 | ✅ |
| | vehicles (8c) | 52.6 | 59.0 | ✅ |

Region-decomp leaves the combined-load verdict unchanged: **6+6 still only just clears churn (no margin
gain), 8+4 still fails.** The 8+4 churn nudge (5.5→6.8) matches the isolated 4-core +8% — real but far too
small to reach real-time.

## Verdict

**P6-2 (region-*task* decomposition, as built) is bit-identical but does NOT deliver the per-core uplift —
targets missed:**

| target | result |
|---|---|
| ≥1.4× per-core uplift at 4–8 cores | ❌ best **1.08×** (isolated churn 4c); raw ORCA best 1.03× |
| heavy churn ≥1.5× RT margin at 6 ped cores (≥15 st/s) | ❌ **12.2 st/s** isolated / **10.9** concurrent (~1.1× margin) |
| determinism / offline gate | ✅ bit-identical, gate green |

**Why — and the fix.** The as-built P6-2-2/3 deliberately kept the **shared, read-only frozen grid** and
changed only *which agents a worker processes together* (region-task parallelism). But a region's agents
still live at **scattered SoA indices**, so their neighbour position/velocity reads still scatter across the
whole array — **the exact random-access bandwidth bottleneck P6-2 targets is left untouched.** Region-task
grouping alone adds dispatch/partition overhead without a locality payoff, so it is neutral-to-negative
(and worse at high core counts / coarse regions, where overhead dominates). The design already names the
missing piece: **§8 phase-2 — reorder the agent SoA into region-contiguous (grid/Morton) order** (kept as
an index permutation to bound the reorder cost, results permuted back to stable handle order so bit-
identity is preserved) so a region's agents are physically contiguous and their neighbour reads stay
cache-resident. **That reorder — not region-task scheduling — is what delivers the locality, and it is now
required to hit the 1.4× bar.**

**P6-2 is NOT done.** Recommended next step: implement the SoA-reorder refinement (phase-2), keep the
bit-identity gate, and re-run this exact campaign. Until then the combined-load operating point stands:
**peds ≥ 8 cores for churn-heavy loads** (the 4+8 split clears; region decomp does not change that).

### Reproduce

```
dotnet run -c Release --project src/Sim.BenchCrowd  -- --sizes 100000 --steps 20 --warmup 3 [--region-decomp --region-mult K]
dotnet run -c Release --project src/Sim.BenchPedLod -- --sizes 100000 --high-fraction 0.1 --steps 30 --warmup 8 [--region-decomp --region-mult K]
# core cap: process affinity mask + DOTNET_PROCESSOR_COUNT=N (K swept ∈ {2,4,8}; mult 2 best)
```
