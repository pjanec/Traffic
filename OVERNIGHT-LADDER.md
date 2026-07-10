# OVERNIGHT-LADDER.md — sorted autonomous work queue (flow insertion, warm-start, overtaking)

Three features, decomposed into independently-landable rungs and **sorted to maximise unattended
runtime**: everything fully unblocked / offline-validatable / low-risk is front-loaded; the
network-dependent and statistical pieces sit at the very end so a stall there banks everything
before it. Each rung auto-promotes on: committed `dotnet test` green + `Sim.Bench` determinism hash
`909605E965BFFE59` unchanged (non-feature path) + parity-reviewer ACCEPT. Rebase onto `origin/main`
before every promote (the rail session may still be active). Same discipline as the ER1–ER5 ladder.

## Order (dependency-respecting, risk-back-loaded)

```
F1 ─► W1 ─► W2            (insertion + warm-start snapshot — the explicit asks; all OFFLINE)
OV1 ─► OV2 ─► OV3 ─► OV4  (opposite-direction overtaking — behavioral; risk isolated to OV2)
F2 , W3                   (network / statistical TAIL — last)
```

If any rung can't land cleanly after reasonable effort: **revert the code** (`git checkout`), keep
only the diagnosis/design in a handoff doc (mirror `C4-VII-REMAINING.md`), move to the next
independent rung. Never leave half-built engine code on `main`.

---

## Chain 1 — flow insertion + warm-start snapshot (offline, low risk, do first)

### F1 · deterministic `<flow>` ingest  (exact-parity-safe)
- **Scope.** `DemandParser` learns `<flow id type route begin end>` with exactly one of
  `number` / `period` / `vehsPerHour`. Expand to concrete `VehicleDef`s at LOAD (cold path):
  `period` → depart = begin + k·period; `vehsPerHour` → period = 3600/rate; `number` over
  `[begin,end)` → even spacing. IDs `<flowId>.<k>` (SUMO convention). Routes: explicit `<route>`
  reused; (`from`/`to` via the B2 `NetworkRouter` is a later add, not F1).
- **No engine change** — the existing depart-gated `InsertDepartingVehicles`/`TryInsertOnLane`
  path inserts the expanded defs. Existing scenarios untouched (no `<flow>` → byte-identical).
- **Validation (OFFLINE, no SUMO).** A `<flow period=T>` scenario must produce a trajectory
  **identical** to the equivalent hand-listed `<vehicle>` scenario (engine-vs-engine). Bench hash
  unchanged. (Optional bonus: a SUMO golden for a flow scenario — not required to promote.)

### W1 · `WarmUp(int steps)` — in-memory warm-start
- **Scope.** Public `Engine.WarmUp(steps)`: run the per-step loop `steps` times with the Export
  phase skipped, leaving the engine in a populated, valid start-of-step state; `Run(n)` then
  continues from there. Deterministic by construction (engine is deterministic; per-vehicle
  `RngState` simply advances). Optional clock re-base flag.
- **Validation (OFFLINE).** Determinism: `WarmUp(N); Run(M)` hash == the tail of `Run(N+M)` for the
  same scenario (uses F1 to fill the net). Bench hash unchanged (WarmUp adds zero alloc vs Run).

### W2 · `SaveSnapshot` / `LoadSnapshot` (file) — **all vehicles incl. trains**
- **Capture set** (miss one → warm-started run diverges):
  - Every ACTIVE vehicle's `VehicleRuntime` dynamic fields — **cars and trains identical** (RailModel
    is stateless pure fns; trains carry NO special per-vehicle field; traction params are on the
    immutable `ResolvedVType`, restored by re-loading the same rou/vType). Fields: id, vType id,
    remaining route + `LaneSeqIndex`, `LaneId`/`LaneHandle`, `Pos`/`Speed`/`LatOffset`/`Acceleration`,
    `Inserted`/`Arrived`, and every accumulator: `RngState`, `SpeedGainProbability`,
    `KeepRightProbability`, `LookAheadSpeed`, `LevelOfService`, `WaitingTime`, `LastActionTime`,
    `AccControlMode`/`CaccControlMode`/`AccLastUpdateTime`, `GiveWaySide`/`GiveWayEvSameLane`, B3
    reroute bookkeeping + `_avoidedByEntity`.
  - **Engine-level stateful machines** (global, reset at every `Run()` start — MUST be captured):
    rail level-crossing phases (`_railCrossingStep`/`NextSwitch`/`State`, R5) and actuated-TLS state
    (`_actuatedLogics`, C6-ii). Static TLS need nothing (pure fn of the captured clock).
  - The sim **clock** (time); command buffer flushed (empty at the step boundary); optionally active
    external obstacles (else caller re-adds).
- **Format.** XML `<snapshot time=...>` with one `<vehicle>` per active entity (self-contained about
  WHICH vehicles exist — flow-generated ones are listed, not re-derived). Binary variant optional.
- **Load contract.** Requires a prior `LoadScenario` (net + vType/route defs); reconstructs runtimes,
  re-resolves lane-seq handles from the remaining route, restores all fields + engine machines, sets
  the clock. Recomputes derived side-tables (`_anyBluelight`, rail conflict sets…).
- **Validation (OFFLINE).** Snapshot-roundtrip equivalence on a **mixed road + rail** fixture (≥1
  train, ≥1 rail crossing, ≥1 actuated light): `WarmUp(N)` in-memory hash == `SaveSnapshot` → fresh
  `LoadScenario`+`LoadSnapshot` → `Run(M)` hash. Bench hash unchanged (save/load are cold path).

---

## Chain 2 — opposite-direction overtaking (behavioral, offline; risk isolated to OV2)

Coordinate choice: keep the overtaker longitudinally on its own edge; represent "using the opposite
lane" as a large `LatOffset` spilling into the opposite lane's footprint (seam 2), so all conflict
detection is footprint overlap in a shared lateral frame — no reverse-`pos` bookkeeping. Reuses
`LatOffset`/`DriftToward`/`GiveWayEdgeTarget`, the ER5 `!FootprintsOverlap` leader-bypass, the
multi-constraint reducer, and `TryGetBidiLaneId` (the antiparallel-lane map). All gated on a new
opt-in `lcOpposite` vType flag (default off → inert / bench-hash-safe).

- **OV1** opposite-lane leader visibility query (read-only; inert). LOW risk.
- **OV2** gap-acceptance: safe overtake window vs oncoming (`overtakeDist / (v_ego−v_leader)` vs
  nearest-oncoming distance at closing speed `v_ego+v_oncoming`, safety-factored). Writes a
  commit-overtake `MoveIntent` flag. **HIGH risk / safety-critical** — revert+handoff if it can't land.
- **OV3** execution: lateral spill into the opposite lane, `!FootprintsOverlap` pass of the slow
  leader, a NEW cross-lane oncoming constraint (brake/abort on footprint overlap), return-to-lane.
- **OV4** cooperative oncoming shift (the requested enhancement): oncoming vehicles detect an
  encroaching overtaker and drift to their outer edge — a mirror of give-way ER3/ER5. LOW risk.

Bar: behavioral property tests (overtaker passes, oncoming shifts, no footprint overlap, both
recenter). No SUMO golden (SUMO's oncoming traffic does not cooperate — this is an enhancement).

---

## Tail — network / statistical (highest friction; LAST)

- **F2a** probabilistic `<flow probability=p>` — the DETERMINISTIC mechanism: **LANDED (offline).**
  Per-second Bernoulli from a per-flow seeded `VehicleRng` (never `System.Random`), one draw per
  active flow per step, arrivals `"<flowId>.<k>"` materialized at runtime through the shared
  `CreateRuntime` path. Gated inert (empty `ProbabilisticFlows` ⇒ byte-identical; bench hash
  unchanged). This is what the warm-start "deterministically precompute a populated network" ask
  needs: `WarmUp(W)` fills the net from the flow, `Run(N)` continues the same stream. Fixture
  `scenarios/56-flow-equiv/prob.rou.xml` + `RungF2ProbabilisticFlowTests` (determinism, rate band,
  WarmUp continuity, snapshot guard). SUMO-STREAM statistical parity (matching SUMO's exact insertion
  stream vs an ensemble → network golden regen) is the DEFERRED cross-check, grouped with C1
  `sigma>0`.
- **F2b** probabilistic-flow FILE snapshot — capture each active flow's per-flow RNG (`RawState`) +
  arrival counter in `SaveSnapshot`/`LoadSnapshot`, and lift the `EngineSnapshot` guard that
  currently throws while a flow is active. Small, offline, tested by a save→load→continue equivalence
  on `prob.rou.xml` (the counter/RNG must survive so ids don't collide and the stream continues).
  NOT yet done.
- **W3** SUMO `--save-state` parity cross-check for the snapshot (load SUMO's saved state, diff ours;
  network). Optional hardening. Network-only.
