# Dense-flow engine integration — DESIGN (HOW)

**Status: DESIGN / for agreement — not yet implemented.**
**Goal:** fix issue #15 (live-city junction gridlock — cars sit at junctions on green and never clear)
by integrating, *wholesale*, the validated engine work from branch
`claude/dense-lane-overlap-fix-5tr4ha` into the live-city line
(`claude/city3d-live-city-mode-3yf4oc`). This doc is the HOW: the exact seams touched, the merge
method, and the parity/determinism argument. The WHAT — the individual engine fixes and *why* they
work — is already designed and validated on that branch; this doc **references** its docs rather than
restating them (`docs/DENSE-FLOW-THROUGHPUT-DIAGNOSIS.md`, `docs/DISCHARGE-YIELD-RESUME.md`,
`docs/GAP1-RESUME.md`, `docs/HIGH-DENSITY-*` on that branch).

## 1. Why (measured, this branch)
A headless A/B was run through the sanctioned harness (`--mode live-city --smoke`, the #15 gridlock
probe added in `dead369`; see `docs/LIVE-CITY-HARNESS-GUIDE.md`). Only the engine was swapped
(the branch's `Sim.Core`/`Sim.Ingest` files overlaid on our render/harness code); everything else was
identical.

| metric (200 s run) | our engine (baseline) | fix-branch engine |
|---|---|---|
| 160 cars — total arrivals | 38 | **81** (2.1×) |
| 160 cars — end / peak stoppedFrac | 0.75 / 0.94 | **0.38 / 0.73** |
| 70 cars — total arrivals | 16 | **36** (2.25×) |
| 70 cars — end / peak stoppedFrac | 0.82 / 0.98 | **0.50 / 0.68** |
| failure mode | **terminal lock** | **jam-and-recover** (oscillates, never terminal) |

Not a full cure (jams still form to ~0.6–0.7), but it converts a *terminal* deadlock into a
*recovering* flow and roughly doubles throughput — at both densities.

## 2. What "wholesale" brings in (inventory)
The engine delta between our merge-base and the branch is **5 source files**:

| file | what it carries (mechanism, per branch docs) |
|---|---|
| `src/Sim.Core/Engine.cs` (~1170 lines) | permissive/minor-crossing yield parity (`MSLink::blockedByFoe` port — the over-yield "waiting on green" fix, `f69a58d`); arriving-vehicle red-light brake / signalized-discharge redistribution (`ca8d515`); dead-lane merge-brake + gated last-resort reroute (Gap 1); parkingArea lowest-free-lot reuse (Gap 2); `departPos="base"` → SUMO `basePos` (Gap 3); turn-lane keep-right segregation WIP (`9a77d3b`, **partly reverted** — branch-HEAD state is what we take) |
| `src/Sim.Core/StopRuntime.cs` | stop/parking bookkeeping used by the reroute/parking paths above |
| `src/Sim.Core/VehicleRuntime.cs` | per-vehicle fields the above read/write |
| `src/Sim.Ingest/DemandParser.cs` | demand parsing for the new depart/parking semantics |
| `src/Sim.Ingest/DepartValue.cs` | adds `DepartPosSpec.Base` |

These bundle features beyond the junction-discharge fix (departPos-base, parking/reroute). Per the
owner's decision that is **accepted** — wholesale is deliberately chosen over a curated subset because
the commits have prerequisite chains (disentangling risks dropping a load-bearing prerequisite), and
branch-HEAD is the branch's own parity-green, self-consistent state.

## 3. Integration method
- **File overlay, not a branch merge.** Take exactly the 5 files above from
  `origin/claude/dense-lane-overlap-fix-5tr4ha` (`git checkout <branch> -- <files>`). A full merge
  would also drag the branch's *older* live-city code (pre-dating this branch's viewer work) and its
  repro/doc scaffolding — we want only the engine.
- **Our branch never touched `Sim.Core`/`Sim.Ingest`** (verified: `git diff merge-base..HEAD --
  src/Sim.Core src/Sim.Ingest` is empty), so the overlay is a clean replacement with no hand-merge.
- The engine surface `LiveCitySim` calls (`LoadNetwork`, `DefineVType`, `SpawnVehicle(... departPos:
  5.0 ...)`, `Step`, `Events`, columns) is unchanged — verified: the whole solution builds clean with
  the overlay.

## 4. Parity argument (the iron law)
Measured with the overlay in place: `dotnet test tests/Sim.ParityTests` = **653 passed / 1 failed / 4
skipped**. The single failure is **not** a trajectory regression:
- `RungHDp0c1SymbolicDepartTests.UnsupportedDepartPosKeyword_ThrowsInvalidDataException("base")` — a
  *unit-test expectation*. The branch added `departPos="base"` support (SUMO `basePos` semantics), so
  the engine now *parses* "base" instead of throwing. The branch itself updated this test; our copy
  still asserts the old throw.
- **Every trajectory / state golden stays byte-identical** (the other 653), i.e. no scenario's
  committed FCD/state moved. The junction-discharge fixes change behavior only on paths the golden
  scenarios don't exercise (permissive-yield / dense-junction discharge), which is exactly why the
  gridlock improves while goldens hold.

**Resolution:** take the branch's version of `RungHDp0c1SymbolicDepartTests.cs` (expects "base" to be
*accepted*, matching the new engine + SUMO). After that, `Sim.ParityTests` → **654 / 4**, restored.

## 5. Determinism argument
- The branch is itself parity-gated and introduces **no `System.Random`** (verify: `grep -rn
  "System.Random\|new Random(" src/Sim.Core src/Sim.Ingest` on the overlay = 0). Per-entity seeded
  state is preserved.
- `Sim.Bench` (`scenarios/_bench/highway-dense`) asserts **determinism** (`hashA == hashB`) and
  **parallel == single** (`hashPar == hashA`) — both must still hold. The hash *value* is **not** a
  pinned regression gate (the bench prints it, does not compare it to a committed constant); if the
  new engine changes the highway-dense trajectory the value may change — that is recorded, not a
  failure, provided determinism + parallel-equality hold.

## 6. Success conditions (what "done" means)
1. Working tree diff vs pre-integration = exactly the 5 engine files + the one test file (§4).
2. Whole solution builds clean (`dotnet build`).
3. `dotnet test tests/Sim.ParityTests -c Release` = **654 passed / 4 skipped** (parity restored).
4. `Sim.Bench`: `deterministic = True` and `parallel == single = True` (hash value recorded).
5. #15 gridlock A/B via `--mode live-city --smoke --frames 400` meets, at cap 160:
   **total arrivals ≥ 2× baseline (≥ ~75)** and **end stoppedFrac ≤ 0.5** — and jam-and-recover
   (never a monotonic march to ~0.95). Re-confirm at cap 70.
6. A full `dotnet test` sweep of the other projects stays green (no non-parity test regresses).

## 7. Optional follow-on (separable, not required for the fix)
The branch also adds dedicated parity tests + `_repro` scenarios (`DenseFlowDeadLaneDrainTests`,
`RungHDp0c2ParkingLotReuseTests`, `RungHDp0c3BaseDepartPosTests`, `LowDensityTeleportTests` edits) that
lock the new behaviors. They are **not** required for #15 (they all stayed green against our copies
except the one flip in §4) but bringing them — with the scenarios they reference — would harden the
integration. Proposed as a later task, gated on the core integration landing first.

## 8. Risks & rollback
- **Bundled unrelated features** (departPos-base, parking/reroute turnover): accepted per owner; they
  are dormant on the live-city path (numeric departPos, no parkingAreas in the crop demand).
- **WIP turn-lane segregation partly reverted on the branch:** we take branch-HEAD as-is; the A/B
  numbers in §1 already reflect that exact state.
- **Rollback:** the integration lands as one commit touching 6 files; a single `git revert` restores
  the prior engine with no entanglement (our branch owns nothing else in `Sim.Core`).
