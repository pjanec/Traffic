# PEDESTRIAN-P6-1-RESULTS.md — on-target pedestrian perf run (16+‑core Windows box)

**Status: measured (P6-1 done).** This is the on-target replacement for every 4-core-VM pedestrian
perf estimate in `PEDESTRIAN-POC7A/7B/7C-FINDINGS.md`. All three scale benchmarks
(`Sim.BenchCrowd`, `Sim.BenchPedLod`, `Sim.BenchPedNet`) were run on the owner target box; this
doc is the "report back" artifact. The findings docs are updated in place with an on-target column
alongside a labeled "prior (4-core VM)" column so the scaling story stays visible.

## Box / run conditions

| | |
|---|---|
| CPU | **Intel Core Ultra 9 275HX** (Arrow Lake-HX; 8 P-cores + 16 E-cores) |
| `Environment.ProcessorCount` | **24** (24 physical cores, no SMT) |
| OS | Windows 11 Pro 26200 |
| Power plan | **High performance** (set before the runs) |
| Toolchain | .NET SDK 10.x building the repo's `net8.0` targets, `-c Release` |
| Build gate | `dotnet build Traffic.sln -c Release` → Build succeeded, 0 errors |
| Code state | **current branch head** — i.e. *post* all P0 optimizations (P0-3 incremental `Add`/`Remove`, P0-4 crowd spatial hash on). This matters for the PedLod comparison below. |
| Methodology | each configuration run **3×**, **median** reported; wall-clock, runs in isolation |

> **Note on the 4-core comparison basis.** `Sim.BenchCrowd` code is unchanged since POC-7a (its crowd
> always had `UseSpatialHash=true`), so its 4-core POC-7a table is a valid *same-code* comparison.
> `Sim.BenchPedLod` is **not** — P0-3/P0-4 changed the code after the original POC-7c 80/285 ms
> numbers were taken. The honest same-code 4-core baseline for PedLod is the **P0-4 section** of
> POC7C-FINDINGS (100k stable ≈ 88–102 ms, churn ≈ 183 ms parallel). Both bases are shown below and
> which is which is called out, so the core-count scaling is not conflated with the algorithmic wins.

---

## 1. `Sim.BenchCrowd` — raw ORCA `Step` scaling (POC-7a)

Main sweep, runtime-auto parallelism (all 24 cores), median of 3 runs:

| N (agents) | serial ms/step | parallel ms/step | speedup | parallel steps/s | prior 4-core parallel steps/s |
|-----------:|---------------:|-----------------:|--------:|-----------------:|------------------------------:|
| 1,000      | 3.54           | 1.53             | 2.31×   | 653              | 373                           |
| 10,000     | 18.49          | 4.34             | 4.26×   | 230              | 131                           |
| 50,000     | 76.15          | 18.67            | 4.08×   | 53.6             | 26.6                          |
| 100,000    | 323.3          | 50.1             | 6.45×   | **20.0**         | 10.1                          |

### Thread-count sweep at N = 100,000 — the plateau the 4-core VM could not reach

Median parallel ms/step per `--max-parallelism` cap (3 runs each):

| MaxParallelism | parallel ms/step | steps/s | speedup vs 1 thread | parallel efficiency |
|---------------:|-----------------:|--------:|--------------------:|--------------------:|
| 1              | 328.8            | 3.0     | 1.00×               | 100%                |
| 2              | 164.5            | 6.1     | 2.00×               | 100%                |
| 4              | 91.1             | 11.0    | 3.61×               | 90%                 |
| 8              | 59.7             | 16.7    | 5.50×               | 69%                 |
| 16             | 48.5             | 20.6    | 6.78×               | 42%                 |
| 24             | 48.5             | 20.6    | **6.78×**           | 28%                 |

**Where it plateaus: ~8→16 threads, and it is FLAT from 16→24.** On a box with 24 *physical* cores
the curve stops improving at 16 threads and gains nothing from the last 8 cores. Two compounding
causes, both pointing the same way:

1. **Memory-bandwidth bound.** ORCA's per-agent work is a scatter of spatial-hash bucket lookups and
   neighbour position/velocity reads across the SoA arrays — the same random-access shape
   `PERF-HANDOVER.md` names as the lane engine's ~8-thread bandwidth ceiling. Efficiency falling from
   90% (4t) → 69% (8t) → 42% (16t) is the classic bandwidth-saturation signature, not scheduling noise.
2. **Heterogeneous cores.** The 275HX is 8 P-cores + 16 E-cores. The sharp taper right at the 8-thread
   mark lines up with running out of P-cores; threads 9–24 land on slower E-cores that each contribute
   less, flattening the tail further.

This is the specific measurement POC-7a flagged as "could not reproduce or falsify past 4 cores." It is
now reproduced: **the raw ORCA `Step` does NOT scale cleanly to 24 cores — it saturates around 8–16
threads.** See the P6-2 note at the bottom.

## 2. `Sim.BenchPedLod` — integrated LOD system at 100k (POC-7c) — the headline

Main sweep, runtime-auto parallelism, `--high-fraction 0.1 --steps 30 --warmup 8`, median of 3 runs:

| N total | scenario | actual high | switches/step | serial ms/step | parallel ms/step | parallel steps/s |
|--------:|----------|------------:|--------------:|---------------:|-----------------:|-----------------:|
| 20,000  | A/stable | 2,098       | 70            | 27.9           | 8.25             | 121              |
| 20,000  | B/churn  | 5,254       | 175           | 55.1           | 13.9             | 71.9             |
| 50,000  | A/stable | 5,235       | 175           | 95.4           | 25.3             | 39.5             |
| 50,000  | B/churn  | 12,160      | 614           | 265.2          | 33.1             | 30.2             |
| 100,000 | A/stable | 10,477      | 349           | 158.7          | **35.9**         | **27.9**         |
| 100,000 | B/churn  | 23,515      | 1,379         | 334.3          | **54.9**         | **18.2**         |

**Headline, 100k parallel: stable = 35.9 ms/step (27.9 steps/s); heavy-churn = 54.9 ms/step (18.2
steps/s).** Speedup at 100k: stable 4.4×, churn 6.1×.

On-target vs the **same-code** 4-core baseline (P0-4 section of POC7C):

| 100k parallel | 4-core (post-P0-4) | 24-core (this run) | scaling |
|---|---:|---:|---:|
| A/stable | ~88–102 ms | 35.9 ms | ~2.6× |
| B/churn  | ~183 ms    | 54.9 ms | ~3.3× |

(Against the *original, pre-optimization* POC-7c baseline of 80 / 285 ms the current on-target code is
~2.2× / ~5.2× faster — but that number conflates the P0-3/P0-4 algorithmic wins with the core-count win
and should not be read as pure hardware scaling.)

**Q1 (does cost track the high set or the total?) — confirmed on-target.** 10k-high-only = 16.9 ms
parallel; 100k-total (same ~10.5k high + 90k low) = 33.6 ms parallel. Adding 90,000 low-power PathArc
followers costs ~17 ms — the low-power tier is genuinely O(1)/step. The LOD thesis holds.

**Churn is noisier on-target, and that itself is a finding.** The B/churn serial times swing ±40% run
to run (334 / 501 / 276 ms at 100k) — the `RebuildHighCrowd`/membership-churn path is GC- and
allocation-heavy, so its wall-clock is variance-dominated in a way the stable path is not. The parallel
churn number is stable (~54–55 ms) because the ORCA step dominates once membership settles within a step.

> **Thread-sweep caveat for PedLod.** `Sim.BenchPedLod`'s `--max-parallelism` flag is a **documented
> no-op** (`Program.cs` ~L187: `PedLodManager` drives its high-power `OrcaCrowd` at runtime-auto and
> exposes no `MaxParallelism` passthrough). A confirmation run at `--max-parallelism 8` returned the
> same stable-100k time as runtime-auto (36.9 vs 35.9 ms), proving the knob is inert. **The pedestrian
> thread-scaling curve is therefore the `Sim.BenchCrowd` sweep above** — that IS the same
> `OrcaCrowd.Step` the high-power set runs — not a separate PedLod sweep, which the harness cannot
> produce.

## 3. `Sim.BenchPedNet` — single-stream DDS bandwidth (POC-7b) — confirmation

Byte-counting through the real `FrameCodec`; **byte-identical across all 3 runs** (deterministic,
VehicleRng-seeded), and identical to the 4-core figures — confirming the codec is machine-independent
as expected:

| scenario | Mbit/s | vs 500 Mbit/s budget |
|---|---:|---|
| typical (LOD split, DR-gated) | **36.75** | FITS, 92.7% headroom |
| worst-case spike (all 100k promoted, 100% sent) | **182.4** | FITS, 63.5% headroom |
| naive (unquantized CrowdRecord, all promoted) | **294.4** | FITS, 41.1% headroom |

Bandwidth is confirmed under budget on the target toolchain. (The *live* CycloneDDS transport binding
does not exist yet — the dev session is building it; this is the single-stream **codec** figure, which
is the one that matters: one multicast stream, every IG reads it. Live-DDS confirmation is a follow-up.)

---

## Verdict against the three P6-1 success conditions

1. **Stable 100k is interactive on this box — PASS.** 27.9 steps/s (35.9 ms/step) parallel, well above
   the ~12.5 steps/s the 4-core VM managed and comfortably real-time.
2. **Churn 100k is within the interactive band — PASS.** 18.2 steps/s (54.9 ms/step) parallel. The
   `OrcaCrowd.Add/Remove` work (P0-1/P0-3) plus the P0-4 spatial hash have brought the previously-worst
   case (originally 285 ms / 3.5 steps/s) firmly into interactive range on-target.
3. **Bandwidth under the 500 Mbit/s budget — PASS (confirmed).** 36.75 / 182.4 / 294.4 Mbit/s
   typical / spike / naive, all under budget, byte-identical to the prior figure.

**All three success conditions pass.**

## P6-2 (region decomposition) re-evaluation — **TRIGGERED / warranted**

P6-2's gate is "only if P6-1 shows flat parallel `Step` plateaus." **It does.** The `Sim.BenchCrowd`
thread sweep shows the raw ORCA `Step` plateauing at ~8–16 threads and **completely flat from 16→24
threads on a 24-physical-core box** (parallel efficiency 69% at 8t → 42% at 16t → 28% at 24t) — the
memory-bandwidth-bound shape `PERF-HANDOVER.md` predicted, amplified by the P/E-core split. The last 8
cores buy nothing. This is exactly the P6-2 trigger condition: throughput past ~8–16 threads is capped
by random-access memory bandwidth, and the standard remedy is **spatial region decomposition** — partition
the crowd so each worker sweeps a cache-resident region and neighbour reads stay local, converting the
scattered global-array access pattern into per-region locality. **Recommendation: P6-2 is worth doing**
if higher single-world pedestrian throughput than ~20 steps/s at 100k is needed; if ~20 steps/s stable
(and the LOD split already keeping 100k tractable) is sufficient for the acceptance case, P6-2 can stay
deferred. It is not a correctness gap — it is a headroom ceiling.

## Reproduce

```
dotnet build Traffic.sln -c Release
dotnet run -c Release --project src/Sim.BenchCrowd  -- --sizes 1000,10000,50000,100000 --steps 20 --warmup 3
dotnet run -c Release --project src/Sim.BenchCrowd  -- --sizes 100000 --max-parallelism {1,2,4,8,16,24}
dotnet run -c Release --project src/Sim.BenchPedLod -- --sizes 20000,50000,100000 --high-fraction 0.1 --steps 30 --warmup 8
dotnet run -c Release --project src/Sim.BenchPedNet
```
