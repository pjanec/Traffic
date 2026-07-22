# LANE-CHANGE-OVERLAP-TRACKER.md — checklist

See `docs/LANE-CHANGE-OVERLAP-TASKS.md` for success conditions, `-DESIGN.md` for HOW, `-SPEC.md` for WHAT.

- [x] **Stage 0** — reproduce (197 vs 0) + instrument + root-cause (LCA_OVERLAPPING blind spot dominant;
      cooperative LC not required; residual = junction emergence)
- [x] **Stage 1** — overlap block on every LC path + keep-right follower veto. **RESULT:** parity-mode
      overlaps **197 → 3** (both-stopped 186 → 0), stuck **0**, full suite green, every golden
      **byte-identical**. Eliminates 100% of the stopped/near-stopped lane-change-into-occupied-slot
      overlaps (the reported bug). ✅
- [x] **Stage 2** — cross-junction leader over the ARRIVAL span (fixes the arrival≠exit lane miss in
      `CrossJunctionLeaderConstraint`; additive braking, faithful). **RESULT:** overlaps **3 → 2**,
      stuck **0**, every golden **byte-identical** (verified full sweep). ✅
- [~] **Stage 3** — last **2** residual are NOT lane-change overlaps: (a) a junction MERGE (two vehicles
      emerging from different internal lanes onto one lane the same step — each on a `:`-lane at the
      other's planning time, invisible to the frozen snapshot) and (b) a 1-step car-following reaction
      (follower closes ~0.25 m when its leader hard-brakes just after a lane change). Both are ECS
      frozen-snapshot/deferred-placement artifacts SUMO avoids via strict serial per-vehicle processing.
      Faithful car-following fixes cannot fully close them; exact-0 needs an ECS-forced deviation
      (deterministic post-settle overlap resolver, byte-identical on goldens) OR is accepted as the
      structural residual.
      **OWNER DECISION (2026-07-21): (C) ACCEPT 2 for now.** Rationale: the reported bug (unrealistic
      stopped/near-stopped lane-change-into-occupied-slot) is 100% fixed; the 2 residual are transient
      MOVING junction merges, not the stopped-car behaviour; and high-density capacity must NOT be
      degraded (verified: throughput unchanged, see below). The full Stage-3 fix (Option A: port SUMO's
      internal-junction foe/merge logic) is a possible follow-up, not yet decided. See
      `docs/LANE-CHANGE-OVERLAP-STATUS.md` for the full standing + resume plan.
- [ ] **Stage 4** — DEFERRED with Stage 3. `LaneChangeOverlapDiagTests` stays `[Fact(Skip)]` (reads 2,
      not 0) with an honest skip note; do NOT loosen the 5.5 m threshold. Already verified this session:
      determinism byte-identical (two-run identical); all goldens byte-identical; full `dotnet test`
      green; **high-density throughput preserved** (saturated grid drains all 412 veh, stuck 0, active@300
      129→127, fully drained by t=600 both pre- and post-fix).
