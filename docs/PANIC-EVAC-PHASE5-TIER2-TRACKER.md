# PANIC-EVAC-PHASE5-TIER2-TRACKER.md — 10k-city optimization checklist

Status for Phase-5 Tier 2 (`PANIC-EVAC-PHASE5-TIER2-DESIGN.md` / `-TASKS.md`). Priority is set by the
**measured** Tier-1 profile: the two O(m²) crowd solvers (pusher 40 % + pedestrian 34 %) come first, then
the city-size items (10k scenario, viz payload, auto-track scan). Both new hashes are opt-in + proven
bit-identical; parity hash `909605E965BFFE59` stays unmoved throughout.

> **Status:** **Tier2-B1 DONE and Opus-reviewed** (both crowd hashes bit-identical + measurably faster;
> 432 pass / 3 skip; hash `909605E965BFFE59` unmoved). **Awaiting owner go for Tier2-B2** (10k demo).

## STAGE T2-S1 — spatial-hash the two crowd solvers
- [x] **T2.1** `MixedTrafficCrowd` uniform-grid neighbour query — brute+grid share `GatherVehicleNeighbours` (grid = sorted 3×3-cell candidates) → bit-identical by construction; `MixedTrafficSpatialHashTests` asserts exact Position+Heading equality incl. MaxNeighbours-binding + non-holonomic paths. Opt-in, default off.
- [x] **T2.2** `EvacConfig.UseCrowdSpatialHash` enables the hash on BOTH `_peds` (OrcaCrowd) and `_mover` (MixedTrafficCrowd via VehicleMover pass-through); `EvacCrowdSpatialHashTests` proves the 604-ped/151-pusher organic demo signature is identical off vs on.
- [x] **T2.3** micro-benchmark (`Sim.EvacProfile --microbench`): at N=2000 OrcaCrowd **3.7×**, MixedTrafficCrowd **2.65×**; crossover ~N=1000 (grid slightly slower at N=250 due to rebuild overhead, as expected).

## STAGE T2-S2 — the 10k demo + payload/scan handling
- [ ] **T2.4** 10k host scenario (spike: 2-lane `--rand` @10k → adopt; else fall back to committed `city-15000`; decision logged)
- [ ] **T2.5** `EvacCityScenario` (auto-track; large central incident sized to trap low-thousands; hashes on; deterministic; locality holds)
- [ ] **T2.6** viz payload management (region-crop / decimation / cap, every drop logged; Opus renders to confirm)
- [ ] **T2.7** before/after 10k cost profile (hashes off vs on) + auto-track scan measurement + verdict

## Deferred (measurement-gated)
- [ ] spatial `QueryNear` / disc feeds — only if T2.7 shows disc feeds material at 10k (Tier-1 < 0.5 %)
- [ ] FearField grid — only if fear update material (Tier-1 ~1 %)
- [ ] auto-track scan optimization (→ becomes T2.8) — only if T2.7 shows it material

---

### Proposed batches
- **Tier2-B1:** T2.1 + T2.2 + T2.3 — core algorithmic win (crowd-solver hashes), self-contained.
- **Tier2-B2:** T2.4 + T2.5 + T2.6 + T2.7 — 10k demo + closing before/after profile.
