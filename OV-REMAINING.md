# OV-REMAINING.md — opposite-direction overtaking: landed + deferred

The opposite-direction overtaking arc (OV1–OV3) is landed on `main` and green. This records what
is done, what was deliberately deferred (with the diagnosis), and the design for the rest — so a
follow-up session can resume without re-deriving it. Same discipline as `C4-VII-REMAINING.md`.

## Landed (on `main`, parity-reviewer ACCEPT each)

- **OV1** — detection: an `lcOpposite` vehicle held up behind a slower same-lane leader forms an
  overtake intent (`VehicleRuntime.OvertakeActive`, exported) when the oncoming (bidi) lane is clear
  ahead. `scenarios/57-overtake-opposite` (hand-written two-way `bidi` road net).
- **OV2** — gap acceptance: replaced the fixed clear-ahead with a closing-speed / time-to-complete
  formula (`requiredClear = (egoFreeSpeed + oncomingSpeed)·overtakeTime + safety`), refusing when the
  head-on closes before the pass could finish. Isolated by identical-geometry / different-speed
  fixtures.
- **OV3** — execution: while `OvertakeActive`, ego spills laterally toward the oncoming lane; the
  ER5 `!FootprintsOverlap` same-lane leader bypass carries it past the slow leader; when the intent
  drops (passed the leader, or gap acceptance refused) ego recenters. Collision-free in the tested
  cases. `ov3-clear.rou.xml`.
- **OV3b (this note)** — adversarial abort-mid-spill SAFETY TEST only (no engine change):
  `ov3b-adversarial.rou.xml` + `RungOV3bAbortSafetyTests`. The leader accelerates (maxSpeed 11) so
  OV2's gap acceptance commits early then ABORTS while ego is already spilled; the test asserts ego
  recenters collision-free (all pairs, exported world X/Y), through the abort window (~13 steps).
- **OV4** — cooperative oncoming shift (the requested enhancement): an oncoming driver that sees a
  spilled overtaker closing head-on down its bidi lane within `CooperativeShiftReactionDist` (200 m)
  pulls to its OWN outer lane edge (`DetectCooperativeShift` → `VehicleRuntime.CooperativeShift`,
  exported; `ComputeLateralEvasion` drifts it to `-(laneHalfWidth - egoHalfWidth)`), then recentres.
  The exact ER3-detection + ER5-drift pattern, reading only the overtaker's already-committed
  LatOffset from the frozen snapshot (parallel-safe). Gated on `_anyLcOpposite` (inert everywhere
  else; bench hash unchanged). `scenarios/57-overtake-opposite/ov4-cooperative.rou.xml` +
  `RungOV4CooperativeShiftTests`. NOTE: because OV2's gap acceptance already guarantees the pass
  finishes before the head-on arrives, the cooperative shift is defence-in-depth margin during the
  approach window (the oncoming widens the corridor while the overtaker is spilled) — it is never
  what prevents a collision, and the two vehicles never reach a true side-by-side pass while spilled
  (the same conservatism that made D1 vacuous). A genuine side-by-side pass would require OV2 to
  commit optimistically on the strength of the oncoming's cooperation — a coupled decision left for a
  future rung (see D1/D3).

## Deferred, with diagnosis (do these next)

### D1 — cross-lane hard-brake backstop: STILL NOT NEEDED (confirmed vacuous, now under D3)
A `OppositeOncomingConstraint` (brake while spilled if a laterally-overlapping oncoming is close) plus
a hard-safety intent drop were prototyped and reverted for OV3 because they never bind. D3 (below)
was expected to make them bind — an overtaker that commits OPTIMISTICALLY (betting on cooperation)
could face a spilled head-on. It was re-investigated with the two-overtaker head-on fixture
(`scenarios/59-overtake-cooperative/headon.rou.xml`) at several spacings. Result: **still no
collision, still no bind.** D3's coupled acceptance only commits when the nearest oncoming is NOT
currently spilled toward ego (`CooperativeSideBySideSafe`'s not-spilled check); the moment an opposing
overtaker spills, both egos' next-step acceptance drops the intent and both ABORT (recenter) and pass
in their own lanes — the abort is the safety response, an explicit brake adds nothing. Confirmed
collision-free across head-on spacings from ~120 m down to the tightest that still commits.
`RungD3CooperativeOvertakeTests.TwoOvertakersHeadOn_...` locks this in. So D1 stays UNBUILT (adding an
untestable brake would repeat the original speculative-code mistake). Re-open only if a *dynamic* new
spilled oncoming can appear INSIDE the committed window faster than the one-step abort can react (none
constructible on the current straight-road bidi model).

### D2 — OV3 RETURN-GAP enforcement — DONE
Past ~t=21 in `ov3b-adversarial`, once the oncoming cleared, ego re-committed and overtook the
now-fast leader a second time; its RETURN cut back in only ~3.6 m ahead of the 11 m/s leader (a body
overlap during the recenter). Cause: the return was triggered implicitly by "no longer held up"
(`DetectOvertake` returns false once `ego.Pos > leader.Pos`), i.e. it recentered the instant it
nudged ahead, without enforcing a safe re-entry gap. FIX (landed): `VehicleRuntime`
`OvertakePassedLeaderIndex` remembers the leader being passed; when the held-up decision drops but ego
has nosed AHEAD of that leader, `OvertakeReturnGapSafe` keeps `OvertakeActive` true (stay spilled)
until the re-entry gap is safe (`IsTargetLaneSafe`'s neighFollow secure-gap). An ABORT (ego still
BEHIND the leader) returns true there, so the OV3b abort-mid-spill behaviour is preserved. Inert for
non-`lcOpposite`. `RungD2ReturnGapTests` runs the FULL `ov3b-adversarial` run: no pair overlaps, and
ego recenters only with a safe gap (it now recenters ~15-20 m ahead, at t=27, not 3.6 m).

### D3 — coupled OV2/OV4 decision (a genuine side-by-side pass) — DONE
On a lane wide enough that a spilled overtaker and a cooperatively-shifted oncoming leave a safe
lateral corridor, the gap acceptance keeps the overtaker committed against an oncoming the
conservative (complete-and-return-before-arrival) rule would abort for, so the two pass SIDE BY SIDE
(overtaker spilled, oncoming shifted to its outer edge), then both recentre. Implemented in
`DetectOvertake`: after the conservative `nearestAhead > requiredClear` test, if the nearest oncoming
will cooperate (`CooperativeSideBySideSafe`: not itself spilled toward ego, AND the corridor
`laneSeparation + oncomingCoopShift - egoSpill - combinedHalfWidths >= CooperativeSideBySideMargin`)
and there is at least `OvertakeMinClearDist` head-on room to reach lateral clearance, ACCEPT. The
corridor predicate is NEGATIVE on scenario 57's 3.2 m lanes (−0.2 m) and positive on the 3.8 m lanes
of `scenarios/59-overtake-cooperative` (+0.7 m), so D3 self-gates: every existing OV fixture is
byte-identical (bench hash unchanged). Only place OV2 bets on another vehicle's cooperation; bounded
by the not-spilled check (see D1). `RungD3CooperativeOvertakeTests`: the overtaker does not abort, the
closest head-on approach is with both off-centre and a safe corridor, no collision, both recentre.
