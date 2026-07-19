# PEDESTRIAN-COMBINED-LOAD-RESULTS.md — vehicles + pedestrians concurrent, single-station envelope

**Status: COMPLETE.** Measures vehicles (`--region`, ~10k concurrent) and pedestrians (100k, LOD) running
**concurrently** on a capped ~half of the 24-core box, to answer whether both clear real-time at once under
the single-station deployment envelope, and whether pedestrian low-core scaling (peds have no region
decomposition, unlike the vehicle `--region` path) is the limiter. Companion to `PEDESTRIAN-P6-1-RESULTS.md`
(which measured each engine alone on the full machine).

## The question

Single-station deployment: one box runs the sim (vehicles **and** pedestrians) plus the image
generators (IG) + OS. The sim may use at most ~50% of the CPU — on this **24-physical-core** box that's
~12 cores, the rest reserved for IG/OS. Target load: **~10k concurrent vehicles** + **100k pedestrians**
(~10% high-power), running **at the same time**. Both engines are memory-bandwidth-bound (random
neighbour access), so run concurrently they contend for the same DRAM bus and will not add linearly.
**Do vehicles and peds each still clear real-time simultaneously under the ~50% cap?** Real-time: peds =
**10 steps/s** (dt=0.1s); vehicles = **1 step/s** (this scenario's step-length is 1.0s).

## Box / method

| | |
|---|---|
| CPU | Intel Core Ultra 9 275HX, `ProcessorCount=24` (8 P-cores + 16 E-cores, no SMT) |
| P-core logical CPUs | **{0,1,10,11,12,13,22,23}** (detected via `GetSystemCpuSetInformation`; interleaved, not contiguous) |
| Power plan | High performance |
| Build | `-c Release`, net8.0 |
| Vehicle harness | `Sim.BenchCity <scn> --region --no-fcd --max-parallelism N` → whole-run-avg steps/s, peak concurrent |
| Pedestrian harness | `Sim.BenchPedLod --sizes 100000 --high-fraction 0.1 --steps 30 --warmup 8 --no-high-only` (A/stable & B/churn) |
| Vehicle scenario | `scenarios/_bench/city-10000` — generated `scripts/gen-benchmark.sh 10000 24 500 1500`; engine-measured **peak 8,354 concurrent**, step-length 1.0s, 0 stuck (SUMO measured ~11k peak / ~8.8k steady) |
| Core capping | **process CPU affinity** (`ProcessorAffinity` mask) + **`DOTNET_PROCESSOR_COUNT=N`** — the latter caps *both* engines' TPL worker counts (neither pins `ProcessorCount`; confirmed each bench prints `logical processors : N`). `BenchPedLod --max-parallelism` is a no-op (P6-1), so affinity+DPC is the only ped lever. |
| Reps | median of 3 (ped concurrent = median of all fully-overlapped iterations; sample counts noted) |

**P/E-aware core assignment (key methodology point).** The P-cores are interleaved, so naive contiguous
blocks would hand the two engines unequal silicon. Using **sim = cores 0–11 (4P+8E), IG/OS reserve =
cores 12–23 (4P+8E)**, each engine keeps **exactly 2 P-cores in every split** (VEH always holds P{0,1},
PED always P{10,11}); only E-cores shift with the split. Masks:

| split | VEH cores (P/E) | mask | PED cores (P/E) | mask |
|---|---|---|---|---|
| 6+6 | 0–5 (2P+4E) | 63 | 6–11 (2P+4E) | 4032 |
| 4+8 | 0–3 (2P+2E) | 15 | 4–11 (2P+6E) | 4080 |
| 8+4 | 0–7 (2P+6E) | 255 | 8–11 (2P+2E) | 3840 |

**Contention tax** = concurrent steps/s ÷ **isolated steps/s on the *identical* core mask** — using the
same physical cores for both cancels P/E-composition differences, so the ratio isolates pure shared-bus
contention. The concurrent protocol keeps the ped config identical to isolated (`--steps 30`) and **loops
the ped bench** while one vehicle scenario run is the clock; only ped iterations that ran entirely within
the vehicle run's window (fully mutually loaded) are counted.

## 1. Isolated-at-cap baselines (pinned)

Vehicles (`--region`, whole-run avg, peak 8,354, step-len 1.0s, 0 stuck):

| cap | cores | steps/s | × real-time |
|----:|------|--------:|------------:|
| 4 | 0–3 | 34.7 | 34.7× |
| 6 | 0–5 | 52.6 | 52.6× |
| 8 | 0–7 | 59.8 | 59.8× |

Pedestrians (100k, parallel ms/step → steps/s):

| cap | cores | stable ms | stable st/s | churn ms | churn st/s |
|----:|------|----------:|------------:|---------:|-----------:|
| 4 | 8–11 | 76.3 | 13.1 | 142.2 | **7.0** |
| 6 | 6–11 | 49.7 | 20.1 | 85.3 | 11.7 |
| 8 | 4–11 | 40.5 | 24.7 | 65.1 | 15.3 |

Even isolated, **ped churn on 4 cores (7.0 st/s) is already below the 10 st/s real-time line.** Stable
clears RT at every cap; churn clears at ≥6 cores. Ped throughput scales strongly with cores in this range
(churn 4→6→8c: 7.0→11.7→15.3 st/s) — below the P6-1 16–24-thread plateau it is still core-limited, not
yet bus-saturated.

## 2. Concurrent split runs + contention tax

Real-time bar: peds ≥ 10 st/s, vehicles ≥ 1 st/s.

| split | engine (cores) | isolated st/s | concurrent st/s | tax | clears RT? |
|---|---|---:|---:|---:|:--:|
| **6+6** | vehicles (6c) | 52.6 | **47.6** | 0.91 (−9%) | ✅ 48× |
| | peds stable (6c) | 20.1 | **18.7** | 0.93 (−7%) | ✅ |
| | peds churn (6c) | 11.7 | **11.1** | 0.95 (−5%) | ✅ (+11% margin) |
| **4+8** | vehicles (4c) | 34.7 | **28.6** | 0.82 (−18%) | ✅ 29× |
| | peds stable (8c) | 24.7 | **23.3** | 0.94 (−6%) | ✅ |
| | peds churn (8c) | 15.3 | **14.1** | 0.92 (−8%) | ✅ (+41% margin) |
| **8+4** | vehicles (8c) | 59.8 | **52.6** | 0.88 (−12%) | ✅ 53× |
| | peds stable (4c) | 13.1 | **10.4** | 0.79 (−21%) | ✅ (+4%, razor-thin) |
| | peds churn (4c) | 7.0 | **5.5** | 0.79 (−21%) | ❌ **−45% (fails)** |

Ped concurrent samples: 6+6 → 7, 4+8 → 12, 8+4 → 6. Vehicle 0 stuck in every run.

**Two findings:**

1. **The shared-bus contention tax is mild when neither engine is squeezed — ~5–9% at 6–8 cores — not the
   linear blow-up a worst-case bandwidth model predicts.** Two disjoint-core engines mostly stay out of
   each other's way on this box's memory subsystem.
2. **But the tax roughly triples (to ~20%) for whichever engine is starved to 4 cores** (peds in 8+4, and
   vehicles in 4+8 both show ~18–21%). On 4 cores each core carries more of the working set and the
   per-core bandwidth demand is higher, so bus contention bites harder — a squeezed engine is hurt twice
   (fewer cores *and* a worse tax).

## 3. Verdict

**(a) Do both clear real-time concurrently under the ~50% cap? — YES, provided peds get ≥ 6 cores.**

- **6+6 and 4+8 both pass cleanly, churn included.** Vehicles clear by 29–48×; peds clear stable
  (≥18.7 st/s) and even heavy-churn (11.1 st/s @6+6, 14.1 st/s @4+8) above the 10 st/s bar.
- **8+4 FAILS on the binding case:** with peds starved to 4 cores, ped churn = **5.5 st/s (−45% under
  real-time)** and even ped stable is razor-thin at 10.4 st/s. Vehicles, meanwhile, sit at 53× real-time —
  gross over-provisioning of the engine that was never at risk.
- **Best allocation: give peds the larger share (4+8).** Because vehicles clear real-time with 30–50×
  headroom in every split, cores should flow to the binding engine: peds at 8 cores clear churn with +41%
  margin while vehicles at 4 cores still run 29× real-time. The deployment should **never give peds fewer
  than ~6 cores**, and heavy-churn production should prefer 8.

**(b) Is ped low-core scaling the limiting factor (peds have no region decomposition, unlike `--region`)?
— YES, unambiguously.** Every real-time failure or thin margin is ped-side; vehicles never approach the
bar (28–53× throughout). The whole question reduces to **ped churn throughput at a low core budget**,
which is:

- **Core-scaling-limited, not contention-limited.** The concurrency tax is small (~5–9%) at 6–8 cores;
  what puts churn under real-time at 4 cores is low *absolute* per-core throughput (7.0 st/s isolated),
  not interference. Raising per-core ped throughput — not reducing cross-engine interference — is the fix.
- **Doubly penalised when squeezed.** At 4 cores peds also take the worst tax (~21%), because their random
  neighbour-access working set is least cache-resident there.

The vehicle engine already solved exactly this shape with **`--region` domain decomposition** (byte-
identical; it is why vehicles pay only a small tax and scale cleanly). **Porting that proven pattern to
`OrcaCrowd.Step` (P6-2) is the direct lever** to (i) raise ped per-core throughput so churn clears
real-time with margin on a modest ped core budget, and (ii) improve ped memory locality, which should
also shrink the ~21% low-core contention tax. **This is a clear GO signal for P6-2**, now backed by a
concrete deployment failure (8+4 churn) rather than only the P6-1 plateau argument.

### What would P6-2 need to buy?

To make even a **6-core** ped budget clear heavy churn with a comfortable ~1.5× margin (≥15 st/s vs the
11.1 st/s measured at 6+6), P6-2 needs roughly a **1.4×** uplift in ped churn per-core throughput — well
within what region decomposition delivered on the vehicle side (`--region`: ~3× at 4c, ~4.5× at 8c vs
serial). Until P6-2 lands, the safe operating point is **peds ≥ 8 cores for churn-heavy loads** (the 4+8
split), which passes today.
