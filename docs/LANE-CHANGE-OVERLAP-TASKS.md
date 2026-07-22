# LANE-CHANGE-OVERLAP-TASKS.md — work breakdown + success conditions

References `docs/LANE-CHANGE-OVERLAP-DESIGN.md` (HOW) and `docs/LANE-CHANGE-OVERLAP-SPEC.md` (WHAT).
Baseline (measured, parity/library mode, `willpass-saturation`, 200 steps): **197 overlaps (186
both-stopped), 0 stuck**. Vanilla SUMO 1.20.0 on the same net: **0 overlaps**.

## Stage 0 — reproduce & instrument  ✅ (done this session)
Vendor `/sumo` at `v1_20_0`; confirm 197 (engine) vs 0 (SUMO); attribute every bad commit by path +
category (design §2). **Success:** root cause localised to the `LCA_OVERLAPPING` blind spot + keep-right
null follower + junction emergence.

## Stage 1 — overlap block + keep-right follower veto (design §3 Stage 1)
Files: `src/Sim.Core/Engine.cs`.
- `IsTargetLaneOverlapped(ego, targetLaneHandle, neighbors)`: body-overlap of ego's projected slot vs any
  target-lane occupant (port of `checkChange` `neigh.second < 0`).
- AND `!IsTargetLaneOverlapped(...)` into keep-right, speed-gain, strategic, give-way commit gates.
- Keep-right passes the real target-lane follower (drop the `null`).
**Success conditions:**
1. `willpass-saturation` overlaps **197 → ≤ 3** (both-stopped → 0), 200 steps, library mode.
2. `willpass-saturation` stuck **== 0** over 700 steps (`WillPassSaturationDiagTests` green).
3. Full `dotnet test Traffic.sln` **green**; every prior golden **byte-identical** (byte-diff at least
   scenarios 43/44/45/46/47 + a multi-lane grid `engine.fcd.xml` vs `golden.fcd.xml`).
4. Determinism: two runs of the diagnostic byte-identical; no `System.Random`.

## Stage 2 — cross-junction follower (design §3 Stage 2)
Files: `src/Sim.Core/Engine.cs`.
- Load-time reverse index `normalLaneHandle → feeding internal lane handle(s)` from `Network.Connections`.
- Extend the target-lane veto to scan feeding internal lanes, project emerging vehicles onto the target
  lane, apply `IsTargetLaneSafe`'s secure-gap follower test.
**Success conditions:**
1. `willpass-saturation` overlaps drop toward 0 (Case A eliminated); stuck still **0**.
2. Full suite still green + byte-identical (index/scan inert on every golden).

## Stage 3 — junction-merge residual (design §3 Stage 3)
Files: `src/Sim.Core/Engine.cs` (emergence/transfer path).
**Success conditions:**
1. `willpass-saturation` overlaps **== 0**, stuck **0**.
2. Full suite green + byte-identical; determinism byte-identical.

## Stage 4 — acceptance + verification
- Unskip `LaneChangeOverlapDiagTests`; assert `== 0`.
**Success conditions (the acceptance gate, SPEC §8):**
1. `LaneChangeOverlapDiagTests` (unskipped) → **0** overlaps.
2. `WillPassSaturationDiagTests` → stuck **0**.
3. Full `dotnet test Traffic.sln` green, every golden byte-identical (no golden regeneration unless a
   provable SUMO-1.20.0 provenance bump).
4. Determinism byte-identical across two runs; no `System.Random`.
5. Live-city `--live-city` at `LIVECITY_CARS=110` overlap count drops to ≈ 0.
