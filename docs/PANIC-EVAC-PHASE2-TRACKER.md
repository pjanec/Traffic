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
- [x] **T3.1** wire `FearField` into `EvacDirector.PreStep` — *accepted (B2): fear field driven every tick;
      newly-latched → flee preset + reroute; `Convert` calls `Forget`. Bug caught & fixed: a tick-0
      `TryGetVehicle` miss (vehicle not yet inserted by `Step()`) was treated as death, halving the fleet;
      fixed with `VehState.Seen` (only a post-first-sight miss is a real despawn). EvacSpineTests unchanged
      and passing (panicked=32, converted 3→10 as contagion recruits more).*

## S4 — Behavioural validation
- [x] **T4.1** contagion recruits beyond direct sight — *accepted (B2): onCount(27) > offCount(17). Finding:
      at the demo default θ=0.05 + radius 140, direct LoS alone saturates the fleet, so the test raises θ
      to 0.3 (test-only) to isolate contagion — see the calibration note below.*
- [x] **T4.2** line-of-sight occlusion — *accepted (B2): occluded peer fear 0, clear peer latches*
- [x] **T4.3** jam-unease amplifies, does not originate — *accepted (B2): stationary latches sooner; lone stationary stays 0*
- [x] **T4.4** distant traffic stays unaware — *accepted (B2): isolated far car fear 0 for 50 steps*
- [x] **T4.5** front starts local, spreads — *accepted (B2): firstPanicMaxDist=114.2 < everMaxDist=215.7 (radius 140)*
- [x] **T4.6** determinism (per-handle fear signature) — *accepted (B2)*
- [x] **T4.7** inertness + suite + hash — *accepted (B2): no-incident → 0 panicked & all fear 0; 406 pass / 3 skip; hash unmoved*

## S5 — Viz: the panic front
- [ ] **T5.1** per-vehicle fear tint in the payload
- [ ] **T5.2** `template.js` fear ramp (front visibly spreads)

---

### Batches
- **B1 — DONE (Sonnet, Opus-reviewed & accepted).** S1 + S2: LineOfSight, contagion kernel, FearField +
  EvacConfig tunables, 16 unit tests. Reviewer read the update math + every test (non-vacuous), re-ran
  suite (399 pass) + hash (unmoved).
- **B2 — DONE (Sonnet, Opus-reviewed & accepted).** S3 + S4: FearField wired into EvacDirector (with the
  `Seen`-flag insertion-race fix) + 7 behavioural/determinism/parity tests. 406 pass / 3 skip; hash unmoved.
- **B3 (next):** S5 (T5.1, T5.2) — viz fear front (Opus renders to confirm).

### Calibration finding (from B2 T4.1) — decide before B3
At the demo's shipped default (θ=0.05, incident **radius 140** on the ~360 m grid) **direct line-of-sight
alone saturates the whole fleet** — so the viz would show near-instant panic everywhere, NOT a contagion
front. Contagion is proven by tests (T4.1) but is invisible at demo defaults. **Recommendation for B3:**
give the *viz scene* its own tuned incident (smaller radius, e.g. ~45 m) so only a central cluster panics
by direct sight and contagion visibly recruits outward — WITHOUT changing `EvacGridScenario.DefaultIncident`
(so all tests stay valid). This is a display-only calibration, not a model change.
