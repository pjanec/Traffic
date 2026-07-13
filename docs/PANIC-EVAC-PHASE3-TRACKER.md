# PANIC-EVAC-PHASE3-TRACKER.md — vehicle Orca-push checklist (Option A)

At-a-glance status for Phase 3 (Option A: external shaped-mover handoff). Each item references a task in
`PANIC-EVAC-PHASE3-TASKS.md`; design is `PANIC-EVAC-PHASE3-DESIGN.md`. A box is ticked only when Opus has
verified its success conditions first-hand.

> **Status:** owner signed off Option A (external shaped-mover handoff, reusing
> `Sim.Core.Mixed.MixedTrafficCrowd`; band first-cut = outer wall + obstacles; wedge/away-goal as
> tunables). Implementation follows the orchestration loop (Sonnet implements, Opus reviews hard).

## S1 — VehicleMover + config
- [ ] **T1.1** `EvacConfig` Phase-3 tunables (EnableOrcaPush + wedge/margin/substeps)
- [ ] **T1.2** `VehicleMover` wrapping `MixedTrafficCrowd` (AddCar / Step / wedge tracking)

## S2 — Band walls
- [ ] **T2.1** `FakeNavMesh` mover-band walls (first cut: outer hard edge) + confinement

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

### Proposed batches
- **B1:** S1 (T1.1, T1.2) + S2 (T2.1) — `VehicleMover` + config + band walls, with unit tests.
- **B2:** S3 (T3.1) + S4 (T4.1–T4.6) — integration + behavioural/determinism/parity tests.
- **B3:** S5 (T5.1, T5.2) — shaped-box viz (Opus renders to confirm).
