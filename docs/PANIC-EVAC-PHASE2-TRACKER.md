# PANIC-EVAC-PHASE2-TRACKER.md — panic-as-local-information checklist

At-a-glance status for Phase 2. Each item references a task in `PANIC-EVAC-PHASE2-TASKS.md` (detail +
success conditions); design is `PANIC-EVAC-PHASE2-DESIGN.md`. A box is ticked only when Opus has verified
the task's success conditions first-hand.

> **Status:** owner signed off (seed-only-from-incident; Phase-2 features ON by default). **B1 accepted.**
> Batches follow the orchestration loop (Sonnet implements, Opus reviews hard). Suite 399 pass / 3 skip;
> hash 909605E965BFFE59 unmoved.

## S1 — Fear primitives (pure)
- [x] **T1.1** `LineOfSight.IsVisible` (segment-vs-disc) — *accepted (B1): `LineOfSightTests` (clear /
      midpoint-block / offset / beyond-target / behind-from); only occluders strictly between block*
- [x] **T1.2** contagion kernel `w(d, radius)` — *accepted (B1): `ContagionKernelTests` (1 / 0.5 / 0,
      monotone, non-positive-radius=0)*

## S2 — Fear field
- [x] **T2.1** `FearField` plan/commit update + `EvacConfig` tunables — *accepted (B1): update math matches
      DESIGN §2 (frozen prev, max(accum,directLoS), monotone latch, pin-to-1); `FearFieldTests` prove
      order-independence, seed-only (50-step zero), direct latch+pin, jam-adds-nothing, contagion-on-vs-off*

## S3 — Integration
- [ ] **T3.1** wire `FearField` into `EvacDirector.PreStep` (+ Phase-1 backwards-compat)

## S4 — Behavioural validation
- [ ] **T4.1** contagion causes spread (far car panics with contagion ON, never OFF)
- [ ] **T4.2** line-of-sight occlusion (occluded car gets no direct term)
- [ ] **T4.3** jam-unease amplifies, does not originate
- [ ] **T4.4** distant traffic stays unaware
- [ ] **T4.5** front propagates, never teleports (measured)
- [ ] **T4.6** determinism (fear-evolution signature)
- [ ] **T4.7** inertness + suite green + hash gate

## S5 — Viz: the panic front
- [ ] **T5.1** per-vehicle fear tint in the payload
- [ ] **T5.2** `template.js` fear ramp (front visibly spreads)

---

### Batches
- **B1 — DONE (Sonnet, Opus-reviewed & accepted).** S1 + S2: LineOfSight, contagion kernel, FearField +
  EvacConfig tunables, 16 unit tests. Reviewer read the update math + every test (non-vacuous), re-ran
  suite (399 pass) + hash (unmoved).
- **B2 (next):** S3 (T3.1) + S4 (T4.1–T4.7) — wire FearField into EvacDirector + behavioural/determinism/
  parity tests. **Watch-item:** with default θ=0.05, contagion latches an in-range neighbour in ~1 tick;
  T4.5 (front-not-teleport) must measure honestly and may drive a θ/ContagionRadius calibration.
- **B3 (after B2):** S5 (T5.1, T5.2) — viz fear front (Opus renders to confirm).
