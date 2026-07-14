# PANIC-EVAC-PHASE5-TRACKER.md — scale checklist

Status for Phase 5 (`PANIC-EVAC-PHASE5-DESIGN.md` / `-TASKS.md`). Principle: **full city on the parity
engine; evac attaches only to a bounded working region around the incident** (cost ∝ local affected
population, not city size). Staged: Tier 1 now, Tier 2 (heavy opt, 10k target) after Tier-1 measurement.

> **Status:** owner approved staged (Tier 1 now, Tier 2 later; final goal a 10k-vehicle city, but the
> evac stays local). Design + Tier-1 tasks written; **awaiting go for Tier-1 B1.**

## TIER 1 — realistic organic town + local auto-attach
### S1 — working-region auto-attach
- [ ] **T1.1** `EvacConfig.WorkingRadius`
- [ ] **T1.2** `EvacDirector` auto-track-by-region (deterministic; in-region only; additive)

### S2 — organic scenario
- [ ] **T2.1** `EvacOrganicScenario` (LoadScenario city-organic-L2 net+demand; incident at a busy junction)
- [ ] **T2.2** demand-under-director confirmation

### S3 — behavioural tests
- [ ] **T3.1** cascade on the organic net
- [ ] **T3.2** locality (a far vehicle is never tracked)
- [ ] **T3.3** containment + determinism
- [ ] **T3.4** suite green + hash gate + existing evac tests unchanged

### S4 — viz + measurement
- [ ] **T4.1** organic viz scene (Opus renders to confirm)
- [ ] **T4.2** cost profile at ~400 vehicles (dominant evac hotspot) — scopes Tier 2

## TIER 2 — 10k city (heavy optimization; outline, detailed later)
- [ ] FearField uniform grid (bit-identical)
- [ ] spatial composite CrowdSource + disc feeds (O(local) per query)
- [ ] enable OrcaCrowd spatial hash; MixedTrafficCrowd hash if needed
- [ ] 10k-city demo + viz payload management (region-crop / decimation, logged)
- [ ] working-region scan optimization (only if measured necessary)

---

### Proposed batches
- **Tier1-B1:** S1 (T1.1, T1.2) + S2 (T2.1, T2.2) + S3 tests — auto-attach + organic scenario + behavioural tests.
- **Tier1-B2:** S4 (T4.1 viz + T4.2 measurement) — render + measured cost profile.
- **Tier2-B*:** written as an addendum against Tier-1's measured profile.
