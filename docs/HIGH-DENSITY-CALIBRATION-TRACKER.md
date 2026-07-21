# HIGH-DENSITY-CALIBRATION-TRACKER.md — checklist

Goal: SumoSharp auto-calibrates the highest believable traffic density (matches vanilla's knee).
See `-DESIGN.md` (HOW), `-TASKS.md` (success conditions), `DENSE-FLOW-THROUGHPUT-DIAGNOSIS.md` (Gap 1
evidence). NEEDs: `SUMOSHARP-NEED-dense-flow-gridlock-vs-vanilla.md`,
`SUMOSHARP-NEED-serve-calibration-parity-gaps.md`.

- [x] **Stage 0** — reproduce Gap 1 gridlock (2× dense synthetic: vanilla 0 tp/290 arr/drains vs SumoSharp
      10 tp/275 arr/~45 stuck) + root-cause (wrong-lane strand + clamp at `TryReResolveFromActualLane`
      ~9080, no reroute fallback) + rule out overlap fix + confirm Gap 2/Gap 3 sites. Diagnosis committed.
- [x] **Stage 1 — Gap 3** departPos="base" → SUMO `basePos` (faithful, not "0"). DONE: `DepartPosSpec.Base`
      + `Engine.BasePos` (MIN(vType.Length+0.1, laneLength), capped to first-edge stop endPos). Anchor
      `scenarios/75-base-depart` (veh0→7.1, veh1→3.0) matches vanilla golden to 1e-13. Full suite green (655
      pass); all pre-existing goldens byte-identical (Base arm inert). Box `"base"` error gone → now blocks
      only on Gap 2 parking (as predicted).
- [ ] **Stage 2 — Gap 2** parkingArea runtime lowest-free-lot reuse (MSParkingArea::computeLastFreePos).
      Parking goldens byte-identical; oversubscribed capacity-1 anchor loads; full box loads (with Gap 3).
- [ ] **Stage 3 — Gap 1** reroute-on-wrong-lane (replace drop-lane clamp with reroute; approach trigger).
      Dense synthetic: teleports ≈ 0, halting drains, arrivals ≈ vanilla. Suite green + byte-identical +
      determinism. New anchor. (May need multiple passes; cooperative-LC escalation only if needed.)
- [ ] **Stage 4** — end-to-end: full box (and crop if reachable) on SumoSharp ≈ vanilla (teleports ≈ 0,
      knee within tolerance). Hand-off note back to SumoData.

## Standing measurements (baseline, main 8bb8219, 2× dense synthetic)
| | teleports | arrivals | halting steady |
|---|---|---|---|
| vanilla | 0 | 290 | 0 (drains) |
| SumoSharp | 10 (8 yield) | 275 | ~45 (gridlock, meanSpeed 0) |
