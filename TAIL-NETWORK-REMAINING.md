# TAIL-NETWORK-REMAINING.md — the network tail (F2 SUMO-stream parity + W3) — DONE

**Status: COMPLETE.** Both items below (T1, T2) were landed once a network-enabled session (SUMO
1.20.0 available) ran the golden-regeneration loop. This note is kept as the record of what was done
and why. The findings (esp. T1's measurement subtlety) are worth reading before touching this area.

## Outcome (both on `main`, green)

- **T1 DONE** — ensemble statistical parity of the `<flow probability=>` insertion-count distribution
  vs SUMO. `scenarios/58-flow-probability` (50-seed committed `golden.ensemble`) +
  `RungT1ProbabilisticFlowEnsembleTests`. Engine mean 8.42 / std 2.16 vs SUMO 8.68 / 2.36, within a
  statistical `parityMode` tolerance. **Key finding:** comparing *completed trips* (tripinfo, which
  omits cars that don't finish the edge before `end`) made it look like a 2.6× gap (SUMO 3.22 vs ours
  8.42); comparing *insertions* (distinct FCD ids, = the extended-run tripinfo count) shows they
  already match. Our probabilistic insertion + entry-lane gating are faithful; no engine change.
- **T2 DONE** — `SaveSnapshot` vs SUMO `--save-state` mid-run cross-check. `golden.state.mid.xml` for
  `12-overtake` (t=15, one vehicle mid-lane-change) and `22-idm-carfollow` (t=30) +
  `RungT2SnapshotStateParityTests`. Pos/speed/posLat agree within 1e-2. Calibration: SUMO
  `--save-state.times T` writes the state ENTERING step T = our `Run(T-1)`. No engine change.

The original handoff (kept below for context) framed these as network-only because the *offline* test
loop must never install SUMO; when a network session is available, running SUMO to commit the goldens
is exactly the sanctioned golden-regeneration loop.

## Landed offline (on `main`)

- **F2a** deterministic probabilistic `<flow probability=>` insertion — per-flow seeded `VehicleRng`,
  one Bernoulli draw per active flow per step, runtime `CreateRuntime`. Gated inert. Serves the
  warm-start "deterministically precompute a populated network" ask (`WarmUp` fills, `Run` continues).
- **F2b** probabilistic-flow FILE snapshot — captures the per-flow RNG + counter + the runtime-
  generated vehicles (self-contained, reconstructed at their original EntityIndex). Save→load→continue
  is point-for-point deterministic.

Both are validated **engine-vs-engine** (determinism, arrival-rate band, WarmUp continuity, snapshot
round-trip). Neither claims SUMO-STREAM parity — that is the tail below.

## Remaining — network only (no offline-landable slice)

### T1 — F2 SUMO-stream statistical parity
Our probabilistic insertion uses OUR own per-flow `VehicleRng` (SplitMix64), deliberately NOT SUMO's
`RandHelper`/MT19937 flow RNG — the same decision C1 already made for dawdle (see `VehicleRng.cs`
header). So arrivals are deterministic and reproducible for us, but the *individual* arrival times
will NOT match a specific SUMO run bit-for-bit. The parity bar for a `probability=` flow is therefore
**statistical / ensemble**, exactly like C1 `sigma>0`:
- **Steps.** Author a small `probability=` scenario under `scenarios/`; run a SUMO **ensemble** (N
  seeds) via the network loop; record the arrival-count and headway **distributions** (mean count =
  `probability · duration`, exponential-ish headways). Commit those as the golden + a `tolerance.json`
  with an ensemble/statistical `parityMode` (reuse whatever C1's `sigma>0` track defines).
- **Acceptance.** Our per-flow stream's arrival-count and headway distribution over the same N seeds
  (vary the engine `Seed`) fall within the committed statistical tolerance. NOT a point-for-point FCD
  diff.
- **Note.** If a future requirement needs *exact* SUMO arrival times (not just the distribution), the
  per-flow RNG would have to be reworked to replicate SUMO's flow RNG + draw order — a much larger,
  version-brittle job explicitly out of scope here.

### T2 — W3: SUMO `--save-state` cross-check for the snapshot
Our snapshot format (W2/F2b) is our own XML. SUMO can `--save-state <t> file.xml` and
`--load-state file.xml`. This rung cross-checks that our captured *dynamic* state agrees with SUMO's:
- **Steps.** For an existing committed scenario (e.g. a rail+train one, and a `probability=` one),
  run SUMO to time `t` with `--save-state`, and also run our engine to `t` and `SaveSnapshot`. Diff
  the physically-meaningful fields (per-vehicle pos/speed/lane, TLS phase, the clock) between SUMO's
  saved state and ours, within the scenario's existing trajectory tolerance.
- **Acceptance.** Each vehicle present in both, at the save instant, agrees on lane/pos/speed within
  `tolerance.json`; presence sets match. This is optional hardening (our in-engine WarmUp/round-trip
  determinism is already proven offline) — it guards against a *latent* divergence that only shows at
  the state boundary, not the trajectory.
- **Scope guard.** SUMO's saved state is far richer than ours (full RNG state, device internals,
  routing). Diff only the fields we model; do not attempt to import SUMO's RNG state.

## Why not offline
`scripts/regen-goldens.sh` needs SUMO installed (pip/`apt`, network) and the `builds.dotnet`/SUMO
endpoints are outside the offline egress policy. Running SUMO inside the `dotnet test` loop is
explicitly forbidden (CLAUDE.md). So T1/T2 must be done in a network-enabled session that ends in a
committed golden — not in the offline test loop.
