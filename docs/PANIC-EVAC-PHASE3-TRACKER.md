# PANIC-EVAC-PHASE3-TRACKER.md — vehicle Orca-push checklist (Option A)

At-a-glance status for Phase 3 (Option A: external shaped-mover handoff). Each item references a task in
`PANIC-EVAC-PHASE3-TASKS.md`; design is `PANIC-EVAC-PHASE3-DESIGN.md`. A box is ticked only when Opus has
verified its success conditions first-hand.

> **Status:** owner signed off Option A (external shaped-mover handoff, reusing
> `Sim.Core.Mixed.MixedTrafficCrowd`; band first-cut = outer wall + obstacles; wedge/away-goal as
> tunables). Implementation follows the orchestration loop (Sonnet implements, Opus reviews hard).

## S1 — VehicleMover + config
- [x] **T1.1** `EvacConfig` Phase-3 tunables — *accepted (B1): EnableOrcaPush/OrcaWedgeSpeed/
      OrcaWedgeDwellSeconds/OrcaPushSafetyMargin/OrcaCrowdSubSteps*
- [x] **T1.2** `VehicleMover` wrapping `MixedTrafficCrowd` — *accepted (B1): Nonholonomic=true, ArmWalls/
      AddBlock/AddCar/SetGoal/Step(sub-stepped)/Deactivate + per-index wedge dwell; 4 unit tests
      (moves-to-goal, confinement, wedge before/after dwell, no-sideways-teleport)*

## S2 — Band walls
- [x] **T2.1** `FakeNavMesh.BandWalls` (outer hard edge) + confinement — *accepted (B1): 4 boundary
      segments; VehicleMover confinement test holds (maxX≈96.9 vs a 100 wall)*

## S3 — Integration
- [ ] **T3.1** Orca-push stage in `EvacDirector` (blocked→push→wedge→pedestrian; composite CrowdSource;
      cross-avoidance; Phase-2 backwards-compat)

## S4 — Behavioural / determinism / parity
- [ ] **T4.1** push precedes foot exodus (push ON vs OFF)
- [ ] **T4.2** shaped confinement (no pusher crosses the band wall)
- [ ] **T4.3** wedge → pedestrian
- [ ] **T4.4** no shaped interpenetration
- [ ] **T4.5** determinism (signature incl. pusher poses)
- [ ] **T4.6** parity / inertness + suite + hash gate

## S5 — Viz: cars mounting the shoulder
- [ ] **T5.1** emit pushing cars as oriented shaped boxes
- [ ] **T5.2** render + Opus confirms the shoulder-push reads

---

### Batches
- **B1 — DONE (Sonnet, Opus-reviewed & accepted).** S1 + S2: config + `VehicleMover` + band walls, 4
  unit tests. 410 pass / 3 skip; hash unmoved.
- **B2 (next):** S3 (T3.1) + S4 (T4.1–T4.6) — integration + tests. **Watch-item (from B1):**
  `MixedTrafficCrowd.SteerNonholonomic` uses `targetSpeed = max(min(CreepSpeed, desired), desired·cos(headingErr))`;
  with the default `CreepSpeed=0`, a pusher whose away-goal is ~90° off its lane heading gets
  `targetSpeed≈0` and **deadlocks** (can't nudge-and-turn onto the shoulder). B2 must add a small
  `EvacConfig.OrcaCreepSpeed` (e.g. 0.5 m/s) and set `_crowd.CreepSpeed` in `VehicleMover`, plus a test
  that a pusher with a lateral goal actually reorients + makes progress.
- **B3 (after B2):** S5 (T5.1, T5.2) — shaped-box viz (Opus renders to confirm the shoulder-push reads).
