# Lane-change smoothing — tasks & tracker

Task breakdown + checkable tracker for `SUMOSHARP-LANE-CHANGE-SMOOTHING-DESIGN.md` (the design is the
source of truth for *how*; this file is *what*, in what order, and *done-when*). Viewer-only; all
verification is via the `--trace-veh` harness on loopback (no `dotnet test` / SUMO involvement).

## Conventions
- Files: `src/Sim.Viewer.Core/DrClock.cs`, `src/Sim.Viewer/Program.cs` only.
- Build: `dotnet build src/Sim.Viewer/Sim.Viewer.csproj -c Release`.
- Verify: `dotnet run -c Release --no-build --project src/Sim.Viewer -- --mode loopback --trace-veh <id> --screenshot <png> --frames <n> scenarios/<dir>`, then the lateral/longitudinal analysis of the DRTRACE (design §7).

---

## Stage 1 — lateral-straddle classification & blend

### T1 — `DrClock.Resolve`: split the straddle into downstream vs lateral (design §4.1)
- **Files:** `DrClock.cs`.
- **Do:** in the `a.LaneHandle != b.LaneHandle` branch, keep the `ArcInWindow` attempt; when it returns
  a value → downstream (unchanged). When it returns **null**, no longer extrapolate — return a
  two-state result (`State=a`, `SecondState=b`, `Blend=f`, `IsLateralStraddle=true`). Restore the
  two-state fields on `Resolved` for this case only.
- **Success conditions:**
  1. Same-lane and downstream-straddle paths are byte-for-byte behaviourally unchanged (a
     `44-multilane-junction-turn` `vN` trace is identical to pre-change: max lateral ≤ 0.10 m).
  2. A lateral straddle sets `IsLateralStraddle` and carries both records (assert via a `--trace-veh`
     run on `12-overtake` `follow`: the change instant now reports the lateral-straddle path, not
     `extrap=True`).

### T2 — `PumpAndBuildVehicleDraws`: Cartesian pose blend + chord heading + guard (design §4.2)
- **Files:** `Program.cs`.
- **Do:** when `resolved.IsLateralStraddle`, resolve `poseA`/`poseB` on their own lanes (each with its
  `PosLat`), then `p = lerp(poseA, poseB, f)`; heading = chord `naviFromVector(poseB−poseA)` with the
  short-chord fallback to `LerpAngleDeg`; snap to `poseB` if `|poseB−poseA|` exceeds the §4.2 sanity
  bound. Non-straddle path unchanged.
- **Success conditions:**
  1. `12-overtake` `follow`: DR lateral coordinate moves **monotonically** old-lane→new-lane across the
     packet interval with **no single-frame lateral step > 0.5 m** (baseline: one ~3.2 m step / stuck).
  2. `07-keep-right-change`: same smooth-slide property on its change(s).
  3. Heading tilts during the change and returns to the straight value after.

---

## Stage 2 — regression & edge coverage

### T3 — regression + sublane + sanity-guard verification (design §7)
- **Files:** none (verification only; fixes fold back into T1/T2 if a check fails).
- **Success conditions:**
  1. **No junction regression:** `44-multilane-junction-turn` `vN` max lateral per-frame ≤ 0.10 m.
  2. **Sublane unaffected:** `61-sublane-sidebyside` `v0` shows its steady `posLat=1.5` offset with no
     new per-frame lateral jitter (> 0.2 m) introduced.
  3. **Guard works:** a despawn/respawn or teleport (handle reuse) snaps rather than drawing a long
     diagonal (construct or find a case; confirm no multi-metre diagonal in the DRTRACE).

---

## Stage 3 (optional polish — only if Stage 1–2 look good on screen)

### T4 — heading-tilt tuning / brief low-pass on the lane-change heading
- **Files:** `Program.cs`.
- **Do:** if the chord-derived tilt reads too abrupt or too subtle, apply a short heading low-pass
  during the lateral straddle (analogous to the local viewer's τ≈0.25 s heading filter) and/or clamp
  the tilt magnitude. Interactive/visual judgement.
- **Success condition:** user confirms the lane change "looks NICE" on the interactive viewer (the same
  bar the junction turn cleared).

---

## Tracker — ✅ COMPLETE (shipped on `main`, user-confirmed live 2026-07-15)

All lane-change smoothing work is done and visually confirmed by the user (smooth slide + correct
two-way lean; no junction/sublane regression). The implementation evolved beyond the original T1–T4
plan above — the boxes below record what actually shipped, and §"Evolution" notes where it diverged.

- [x] **T1** — `DrClock.Resolve` classifies downstream (arc-window) vs **lateral straddle** (returns the
      two-state result). Also fixed the real "stuck on old lane" root cause: the wire's `Upcoming[0]`
      lags the record's own `LaneHandle` after a same-edge tactical change → `NormalizeUpcoming`
      re-anchors index 0 (viewer-side read fix, no `Sim.Core` touch). Commit `103870d`.
- [x] **T2** — `PumpAndBuildVehicleDraws` lateral-straddle **Cartesian pose blend** + sanity guard (gated
      on the bracket's REAL span `PacketSpan`, not the smoothed EMA). Commit `103870d`. *(The chord
      heading from the original plan was later replaced — see T4.)*
- [x] **T3** — regression & coverage (verified from DRTRACE, final state):
    - **44 junction** — max lateral/frame **0.081 m** ≤ 0.10 m ✅; heading still sweeps the full turn,
      no wobble (≤1.7°/frame) after the motion-tilt.
    - **61 sublane** — steady `posLat` offset preserved, no new jitter ✅.
    - **12-overtake** — traverses both lanes, smooth slide, **correct lean both ways**; the earlier
      **entry kick / 1.24 m step was resolved at the ROOT by DR-error publishing** (bounds the
      extrapolation overshoot at the source — straight-cruise rendered/true speed ratio 0.72, zero
      sub-70 % events; pass hiccup gone). See `SUMOSHARP-DR-ERROR-PUBLISHING-DESIGN.md`.
    - **07-keep-right** — determined to be a startup/acceleration artifact (standstill depart, ~65 m
      bracket gap), NOT a clean lane-change signal; dropped as a bar. Not a defect.
- [x] **T4** — **heading**: shipped as a **motion-derived tilt** (not the original straddle chord): the
      heading leans by `atan2` of the vehicle's actual per-frame render motion (SUBTRACTED — navi is
      clockwise, `atan2` is CCW), capped ±25°, then a τ=0.18 s render-heading low-pass. This leans the
      car into **both** the outbound and return changes uniformly (the chord approach left the return —
      rendered via extrapolation, not the straddle — sliding flat). Verified: outbound deg 74→90, return
      90→106 (symmetric), junction/cruise unaffected; user confirmed orientation correct live. Commits
      `e01b2d3` (initial always-on heading low-pass) → `a2af94a` (motion-tilt both ways) → `4e4f9ed`
      (sign fix).

## Evolution (what shipped vs the T1–T4 plan)

The original design (`SUMOSHARP-LANE-CHANGE-SMOOTHING-DESIGN.md`) fixed the lateral SNAP via the
straddle Cartesian blend (T1/T2). Getting the motion fully smooth then required three more pieces that
weren't in the initial plan:
- **Position error-smoothing** (capped, forward-biased correction; gentle ≤50 % slowdown, never
  freeze/reverse) — commits `7bafbd8` (+ `7e366e2` for the floor & auto-delay OFF default).
- **Motion-derived heading tilt** (T4 as-shipped, above) — replaced the straddle-only chord heading so
  the return change leans too.
- **DR-error-based publishing** — the ROOT fix for the reconciliation snap / entry kick / pass hiccup
  (publish-on-prediction-error, publish-side, bounds the viewer's overshoot at the source). Its own
  doc: `SUMOSHARP-DR-ERROR-PUBLISHING-DESIGN.md`. This is what let T3's "entry kick" close.
Full as-built pipeline: `SUMOSHARP-VIEWER-DR-SMOOTHING.md` §10.

## Out of scope / parked

**Stopped car lane-changes into an occupied space (vehicle overlap)** — a real `Sim.Core` (parity)
ENGINE bug, NOT a DR/viewer smoothing issue (the overlap is in the authoritative snapshot). Isolated +
documented separately in `docs/SUMOSHARP-ISSUE-stopped-lane-change-overlap.md`; fix deferred (tolerance-
bound, wants a minimal repro + SUMO diff first).
