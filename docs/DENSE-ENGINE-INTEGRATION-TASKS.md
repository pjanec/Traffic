# Dense-flow engine integration — TASKS

Work breakdown for `docs/DENSE-ENGINE-INTEGRATION-DESIGN.md`. Each task names its design reference,
the files it touches, its dependencies, and **success conditions** the implementor must satisfy
first-hand (not by trusting a report). Design §N references point at that doc.

## Stage 1 — Core integration (delivers the #15 fix)

### T1 — Overlay the branch engine wholesale
- **Design ref:** §2, §3.
- **Files:** `git checkout origin/claude/dense-lane-overlap-fix-5tr4ha --`
  `src/Sim.Core/Engine.cs` `src/Sim.Core/StopRuntime.cs` `src/Sim.Core/VehicleRuntime.cs`
  `src/Sim.Ingest/DemandParser.cs` `src/Sim.Ingest/DepartValue.cs`.
- **Deps:** none.
- **Success conditions:**
  1. `git diff --name-only` shows **exactly** those 5 files.
  2. `dotnet build` (whole solution) succeeds, 0 errors.

### T2 — Restore parity: adopt the `departPos="base"` test expectation
- **Design ref:** §4.
- **Files:** `git checkout origin/claude/dense-lane-overlap-fix-5tr4ha --`
  `tests/Sim.ParityTests/RungHDp0c1SymbolicDepartTests.cs`.
- **Deps:** T1.
- **Success conditions:**
  1. `dotnet test tests/Sim.ParityTests -c Release` = **654 passed / 4 skipped / 0 failed**.
  2. The adopted test asserts "base" is *accepted* (resolves to SUMO `basePos`), not that it throws.

### T3 — Determinism / bench gate
- **Design ref:** §5.
- **Files:** none (run only).
- **Deps:** T1.
- **Success conditions:**
  1. `grep -rn "System.Random\|new Random(" src/Sim.Core src/Sim.Ingest` = 0 matches.
  2. `Sim.Bench` prints `deterministic … : True` and `deterministic (par == single) : True`.
  3. Record the new `hashA` value in the tracker (informational, not a gate).

### T4 — #15 gridlock acceptance (the whole point)
- **Design ref:** §1, §6.5.
- **Files:** none (run the probe from `dead369`).
- **Deps:** T1.
- **Command:** `dotnet run --project src/Sim.Viewer -c Release -- --mode live-city --smoke --frames 400`
  (and again with `LIVECITY_CARS=70`).
- **Success conditions (cap 160):**
  1. `LIVECITY-SMOKE`/`ArrivedTotal` → **total arrivals ≥ 75** (baseline 38).
  2. Final-interval **stoppedFrac ≤ 0.5** (baseline 0.75).
  3. Trace is **jam-and-recover** — stoppedFrac dips back down at least twice, never a monotonic
     climb to ~0.95.
  4. Cap 70 re-confirms arrivals ≥ ~30 (baseline 16) and end stoppedFrac ≤ ~0.55.

### T5 — Full test sweep + parity-review gate
- **Design ref:** §6.
- **Files:** none (run only).
- **Deps:** T1, T2.
- **Success conditions:**
  1. `dotnet test` across all projects = green (no non-parity regression).
  2. Opus parity-review confirms T2/T3/T4 numbers first-hand (re-run, not trust) before accepting.

### T6 — Commit + push
- **Deps:** T1–T5 all green.
- **Success conditions:** one commit (6 files + these docs), pushed to
  `claude/city3d-live-city-mode-3yf4oc` after fetch+rebase. Commit body records the A/B numbers,
  the parity result (654/4), and the bench determinism result.

## Stage 2 — Harden (OPTIONAL, separable — design §7)

### T7 — Bring the branch's dedicated parity tests + scenarios
- **Design ref:** §7.
- **Files:** `DenseFlowDeadLaneDrainTests.cs`, `RungHDp0c2ParkingLotReuseTests.cs`,
  `RungHDp0c3BaseDepartPosTests.cs`, `LowDensityTeleportTests.cs` edits, + the `scenarios/_repro/*`
  they reference.
- **Deps:** Stage 1 landed.
- **Success conditions:** those tests pass; `dotnet test` stays green; no golden regressed.
- **Note:** gated on owner go-ahead; not required for the #15 fix.
