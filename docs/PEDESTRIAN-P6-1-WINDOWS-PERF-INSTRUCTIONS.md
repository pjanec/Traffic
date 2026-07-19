# P6-1 — On-target pedestrian perf run (16+‑core Windows box)

**You are the Windows perf-test session.** Your one job is **P6-1**: run the pedestrian scale benchmarks on
this 16+‑core Windows machine, record the real numbers, replace the 4-core VM estimates in the findings
docs, and **report back**. You are dedicated to pedestrians and share a branch with the developer session —
see *Git discipline* below so the two of you never clobber each other.

Branch: **`claude/pedestrian-simulation-design-d8fbme`**.

## Why this exists

Every pedestrian perf number in the repo (`PEDESTRIAN-POC7A/7C-FINDINGS.md`) was measured on a **4-core**
Linux VM and flagged as needing an on-target run. The owner target is this **16+‑core Windows box**. The
*ratios and shapes* are already trusted; what's missing is the **absolute steps/s at real core counts** and
the **scaling curve past 4 threads** (the 4-core VM literally cannot measure an 8- or 16-thread plateau).

## Setup (once)

1. **.NET 8 SDK** must be on `PATH` (`dotnet --version` → 8.x). No SUMO needed — these benches never touch
   the engine's parity loop or the network.
2. From the repo root, confirm a clean Release build:
   ```
   dotnet build Traffic.sln -c Release
   ```
   (Optional sanity: `dotnet test Traffic.sln -c Release` should be green — 630 parity (+3 skip) / 142 ped /
   2 DotRecast / 1 host. Not required for the benches, but confirms the box is healthy.)
3. **Measurement hygiene** — these are wall-clock benchmarks:
   - Set the Windows power plan to **High performance** (or Ultimate).
   - Close other heavy apps; run each bench **in isolation**.
   - Run each configuration **3×** and report the **median** (note variance if runs disagree > ~10%).
   - Record `Environment.ProcessorCount` (each tool prints it) so the numbers are self-describing.

## The three benchmarks — run all of these

Each is a standalone console app (not part of `dotnet test`). Redirect output to a file so nothing is lost.

### 1. `Sim.BenchCrowd` — raw ORCA `Step` scaling (POC-7a)
Serial vs parallel `OrcaCrowd.Step` across crowd sizes + speedup + where it plateaus.
```
dotnet run -c Release --project src/Sim.BenchCrowd -- --sizes 1000,10000,50000,100000 --steps 20 --warmup 3
```
Then **sweep thread counts** to find the on-target plateau (the whole point of a 16-core box):
```
# one run per cap; capture the 100k row from each
dotnet run -c Release --project src/Sim.BenchCrowd -- --sizes 100000 --max-parallelism 1
dotnet run -c Release --project src/Sim.BenchCrowd -- --sizes 100000 --max-parallelism 2
dotnet run -c Release --project src/Sim.BenchCrowd -- --sizes 100000 --max-parallelism 4
dotnet run -c Release --project src/Sim.BenchCrowd -- --sizes 100000 --max-parallelism 8
dotnet run -c Release --project src/Sim.BenchCrowd -- --sizes 100000 --max-parallelism 16
dotnet run -c Release --project src/Sim.BenchCrowd -- --sizes 100000 --max-parallelism 24   # if the box has ≥24 logical
```
**Record:** serial vs parallel ms/step (or steps/s) per size; the speedup at each `--max-parallelism`; the
thread count where the curve flattens (the plateau POC-7a couldn't reach). 4-core baseline was ~3–4×.

### 2. `Sim.BenchPedLod` — integrated LOD system at the 100k target (POC-7c) — **the headline**
Real `PedLodManager`: ~10% high-power ORCA agents + ~90% low-power PathArc followers. Isolates **Scenario A
(stable membership)** vs **Scenario B (churning membership)**, serial vs parallel.
```
dotnet run -c Release --project src/Sim.BenchPedLod -- --sizes 20000,50000,100000 --high-fraction 0.1 --steps 30 --warmup 8
```
Thread-count sweep at 100k (same idea as above):
```
dotnet run -c Release --project src/Sim.BenchPedLod -- --sizes 100000 --high-fraction 0.1 --steps 30 --warmup 8 --max-parallelism 4
dotnet run -c Release --project src/Sim.BenchPedLod -- --sizes 100000 --high-fraction 0.1 --steps 30 --warmup 8 --max-parallelism 8
dotnet run -c Release --project src/Sim.BenchPedLod -- --sizes 100000 --high-fraction 0.1 --steps 30 --warmup 8 --max-parallelism 16
```
**Record:** stable-100k and churn-100k **ms/step and steps/s**, parallel and serial, at the real core count.
**4-core baseline to replace:** stable 100k ≈ **80 ms/step (~12.5 steps/s)**; heavy-churn 100k ≈ **285
ms/step (~3.5 steps/s)**; ~3.7× parallel.

### 3. `Sim.BenchPedNet` — single-stream DDS bandwidth (POC-7b)
Encodes the target population through the **real `FrameCodec`** and measures aggregate Mbit/s. This is
byte-counting, so it is largely machine-independent — running it here just **confirms** the figure on the
target toolchain.
```
dotnet run -c Release --project src/Sim.BenchPedNet
```
**Record:** typical / worst-case-spike / naive Mbit/s. **4-core baseline:** 36.75 / 182 / 294 Mbit/s (all
under the 500 Mbit/s budget). Confirm it still holds.

> **Note on "real DDS multicast" (P6-1 success condition 3):** the *live* CycloneDDS transport binding for
> the pedestrian wire does **not exist yet** — the developer session is building it (DDS ped transport). So
> `Sim.BenchPedNet` measures the single-stream *codec* bytes/sec (which is the figure that matters — one
> multicast stream, every IG reads it), NOT a live DDS socket. Record the codec number now; the live-DDS
> confirmation is a follow-up once the dev session lands the binding. Don't block on it.

## Success conditions (P6-1)

- Stable 100k steps/s is **interactive** on this box (well above the ~12.5 steps/s the 4-core VM managed).
- The churn-100k case is **within the interactive band** (it's the case `OrcaCrowd.Add/Remove` — already
  landed in P0-1/P0-3 — was meant to help; confirm the on-target churn cost).
- Bandwidth **confirmed under the 500 Mbit/s budget** (single-stream codec figure).

## What to write + where

1. **Update the findings docs in place** — replace the 4-core numbers with the on-target ones, keeping the
   4-core figures as a labeled "prior (4-core VM)" column so the scaling story is visible:
   - `docs/PEDESTRIAN-POC7A-FINDINGS.md` (the `Sim.BenchCrowd` scaling + plateau).
   - `docs/PEDESTRIAN-POC7C-FINDINGS.md` (the `Sim.BenchPedLod` 100k stable/churn headline + the Q3 verdict).
   - `docs/PEDESTRIAN-POC7B-FINDINGS.md` (only if the `BenchPedNet` number changed materially — it shouldn't).
2. **Write a new `docs/PEDESTRIAN-P6-1-RESULTS.md`** — a concise on-target report: box spec
   (`ProcessorCount`, CPU model if known), the raw captured tables, the scaling curve (steps/s vs threads),
   and a one-paragraph verdict against the three success conditions. This IS your "report back" artifact.
3. **Tick P6-1** in `docs/PEDESTRIAN-TRACKER.md` (line ~162, Stage P6) with the headline on-target numbers,
   and mark task P6-1 done. Also re-evaluate **P6-2** (region decomposition) — its whole gate is "only if
   P6-1 shows flat parallel `Step` plateaus." If your thread sweep shows an early plateau (e.g. flat past 8
   threads, memory-bandwidth bound like the lane engine), say so explicitly — that's the P6-2 trigger; if it
   scales cleanly to 16, note P6-2 is not needed.

## Git discipline (shared branch — read this)

You and the developer session are on the **same branch** (`claude/pedestrian-simulation-design-d8fbme`). To
avoid clobbering each other:
- **Only edit the perf docs**: `PEDESTRIAN-POC7A/B/C-FINDINGS.md`, the new `PEDESTRIAN-P6-1-RESULTS.md`, and
  the P6-1/P6-2 lines of `PEDESTRIAN-TRACKER.md`. Do **not** touch source, other docs, or the dev session's
  files — that keeps your commits on a disjoint file set so merges are trivial.
- **Never commit build output or bench logs** — put captured output under a scratch dir, not the repo.
- Before pushing: `git fetch origin && git rebase origin/claude/pedestrian-simulation-design-d8fbme`
  (the dev session pushes here too). Resolve nothing but your own doc files; then
  `git push origin claude/pedestrian-simulation-design-d8fbme`.
- Commit message: describe the on-target run; end with the standard trailers this repo uses.

## Report back

When the run is finished: commit + push the results (above), then post a final summary message with the
headline numbers (stable/churn 100k steps/s at the real core count, the scaling curve, bandwidth confirmed)
and the pass/fail against each success condition. If a push-notification tool is available in your session,
use it so we're pinged; otherwise the committed `PEDESTRIAN-P6-1-RESULTS.md` + your final message are the
hand-back.
