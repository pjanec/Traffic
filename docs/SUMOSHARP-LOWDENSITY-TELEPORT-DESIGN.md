# Low-density spurious teleports — diagnosis + design

**Status: DIAGNOSIS COMPLETE, fix DESIGNED, awaiting go-ahead to implement.** This follows the
incoming ask (`docs` upload "SumoSharp core ask: spurious teleports at LOW density") and the tracked
residual in `ISSUE2-JUNCTION-TELEPORT-DESIGN.md` / `NEED-junctionyield-impatience-saturation.md`. It is
a **behavioral parity fix** in the parity-critical junction path, so per `CLAUDE.md` it is designed
first and gated hard on the committed goldens; no code lands until the approach is agreed.

## Reproduction (confirmed this session)

- Committed repro `scenarios/_repro/synthetic-junction2` (the ask's named base), SUMO 1.20.0 vs the
  `sumosharp` drop-in, seed 42, `time-to-teleport=120`, peak ~124 concurrent (uncongested).
- **vanilla teleports = 0; SumoSharp teleports = 10 (1 jam + 9 yield).** Reproduced fresh both engines.
- A uniform netgenerate grid does NOT reproduce (SumoSharp fires *fewer* than vanilla there); the
  irregular net **with a handful of TLS junctions on short approaches** does — matching the repro's own
  README ("the single load-bearing difference … a handful of traffic-light junctions").

## Root cause (traced to the exact vehicle/step/constraint)

Two independent mechanisms, each ~half the teleports:

### A. TL-blind junction yielding (the NEW, dominant finding — 5 of 10)

Worked example: **veh 48**, movement `204 → 266` (right turn), link **17** on TL junction **203**.
- veh 48 sits at the `204_0` stop line, speed 0, from ~t=290 to the teleport at t=443 (121 s), then
  jumps forward on teleport. Vanilla clears the same vehicle long before t=300.
- TL 203 is a static 10-phase, 90 s program; link 17's protected-green window is `t mod 90 ∈ [33,54)`
  → green at [303,324) and [393,414) during the freeze. **The TL math is correct** (verified
  `GetLinkState`): veh 48 genuinely has state **`'G'` (protected green)** in those windows.
- Per-constraint trace at a green tick (t=315–322, egoLinkState `'G'`): `red=inf` (green),
  `leader` intermittently binds, and **`JunctionYieldConstraint` returns ~0.15–0.39** — it holds a
  **green-light** vehicle. It yields to **foe 113**, a vehicle moving 10 m/s one junction upstream
  (at junction 197) whose *route* passes through the shared internal lane `:203_12_0` (which is **red**
  in veh 48's green phase, so foe 113 will itself stop there and never actually cross).
- The binding comes from `JunctionYieldConstraint`'s priority-junction arms, which decide
  right-of-way from the **static netconvert `<request>` response matrix and geometric conflicts**
  (`request.Response.Contains('1')` == "minor") — a proxy the code itself documents as standing in for
  SUMO's `!(*link)->havePriority()` **"for a priority junction (this rung's scope)"** (Engine.cs:6227).
  That proxy is **TL-blind**: at a TL junction, right-of-way is set by the *live signal*, not the static
  matrix. SUMO's `MSLink::havePriority()` returns true for a `'G'` link, so a protected-green vehicle
  yields to no one; SumoSharp's proxy still sees the geometric conflict bit and yields.
- Because the vehicle only creeps ~0.3 m per 90 s green window (the yield throttles it to ~0.39 m/s),
  it never clears, and its `WaitingTime` — which resets on each brief creep (the exact "intermittently
  blocked, timer should reset" symptom the ask names) — eventually accumulates a full 90 s+ dwell
  between creeps and crosses 120 s → teleport.
- Confirmed by a validation spike: gating the yield arms on `egoHasTlPriority` (TL link + live state a
  major-green `'G'`) cut the crossing/merge-driven teleports and dropped the total **10 → 5**.

Binding arms that must become havePriority-aware (all currently TL-blind):
1. the minor-link **cautious-approach** brake — `request.Response.Contains('1')` gate (Engine.cs:6230);
2. the **approaching-foe crossing yield** — `takesCrossingYield` (Engine.cs:6432);
3. the **same-target merge** yield — `SameTargetMergeConstraint` call (Engine.cs:6314).

Latent sibling bug (not the trigger here, but real): `RedLightConstraint` resolves the lane's TL link
via `NetworkModel.TryGetTlControlledConnection`, which returns the **first** TL connection on the lane,
**ignoring the vehicle's actual routed movement** (Engine.cs:5891 / NetworkModel.cs:242). Harmless when
all of a lane's movements share a signal state (true here: links 16/17/18 are identical every phase),
but it *will* mis-gate a shared lane whose movements have different signals (e.g. straight green +
protected-left red). Should be fixed alongside, with its own guard test.

### B. Priority-junction on-junction wedge (the pre-existing residual — 5 of 10)

The other five teleports are **jam**, all at priority junction `:2436` (state `'M'`, uncontrolled):
four vehicles (102, 196, 262, 307) wedge on the *same* internal lane `:2436_0_1` at pos 13.0, plus one
red case (veh 101). This is the on-junction minor-turner / cascade residual localized by
`ISSUE2-JUNCTION-TELEPORT-DESIGN.md` §4-CORRECTION (a committed vehicle held at its conflict point by
`AdaptToJunctionLeader` against a crossing stream / a downstream cascade) — **not** TLS-related. Prior
attempts to touch these arms regressed `WillPassSaturationDiagTests`, so this half is higher-risk and is
scoped as a separate, later stage.

## Fix design

**Principle:** `JunctionYieldConstraint`'s priority-based yield arms must consult the ego link's live
right-of-way, exactly as SUMO's `MSLink::havePriority()` does. For a TL-controlled link that means the
**live signal state**, not the static `<request>` matrix.

- Introduce `EgoHasSignalPriority(egoLink, time)`: true iff the ego link is TL-controlled **and** its
  live state char (the existing `LinkStateChar`, already used by `ClassifyTeleportKind`) is a
  major-green **`'G'`** (uppercase == priority == `havePriority()`). Uncontrolled links → false (they
  keep today's static-matrix behaviour, byte-identical). Permissive green **`'g'`** → false (a permissive
  turn still yields to oncoming, matching SUMO). Red/yellow are already handled by `RedLightConstraint`
  upstream, so a green ego is the only case this changes.
- When `EgoHasSignalPriority` is true and ego is not yet on its internal lane, **skip** arms A1/A2/A3
  above (the vehicle has protected right-of-way; it does not yield). Keep the on-junction
  `AdaptToJunctionLeader` rear-end following intact (a leader physically ahead on ego's own path is a
  car-following concern, not a yield).
- Fix `TryGetTlControlledConnection` (or add a routed-movement variant) so `RedLightConstraint` reads
  the link for the vehicle's **actual next movement**, not the lane's first connection.

**Determinism / parity argument.** The change is inert for every non-TL link (`havePriority` proxy
unchanged → byte-identical), and for TL links it only *relaxes* a yield when the ego signal is a
major-green `'G'` — the exact case where SUMO grants absolute priority and yields to nobody. Any
committed golden that exercised a green-`'G'` vehicle currently forced to yield would already be **out
of parity today** (the goldens are SUMO-generated); since all 627 pass, the corrected behaviour can only
match or improve them. This must be *verified*, not assumed: the full `dotnet test` suite is the gate.

## Tasks (success conditions are measurable)

- **T1 — havePriority-aware junction yield. DONE.** Added `EgoLinkHasSignalPriority` (Engine.cs, near
  `TlLinkStateChar`) and wired it into the three priority-junction yield arms: the minor-link
  cautious-approach gate, the approaching-foe `takesCrossingYield`, and `SameTargetMergeConstraint`'s
  PHASE 0 arrival-time yield (PHASE 1 / on-junction `AdaptToJunctionLeader` following are left intact —
  they are car-following safety, applied regardless of priority, exactly as SUMO's checkLinkLeader).
  Sampled at `time+dt` to agree with `RedLightConstraint`.
  **Result (verified this session):** `scenarios/_repro/synthetic-junction2` SumoSharp teleports **10 →
  5** (the TLS half eliminated); **all 627 committed goldens byte-identical** (`dotnet test` green,
  incl. `WillPassSaturationDiagTests` / `RungHDp2g2*`).
- **T2 — RedLightConstraint routed-movement link. INVESTIGATED, DEFERRED (not landed).** A routed-
  movement resolver (`TryGetRoutedTlConnection`) was implemented and tested: it kept all 627 goldens
  byte-identical, BUT it moved the repro the WRONG way (5 → 6) — reading a vehicle's true movement
  signal correctly stops it at a red the old first-connection proxy wrongly treated as green, which just
  exposes a *different* freeze rather than helping the ask. It is a genuine latent bug (a shared
  approach lane whose movements carry different signal states — e.g. straight-green + protected-left-red
  — is mis-gated), but it is (a) not needed for this ask, (b) counterproductive to the target metric,
  and (c) a semantics change that warrants its own purpose-built shared-lane test before landing. Held
  out of this change; revisit with a dedicated test + re-measurement.
- **T3 — the residual cascade (mechanism B). DIAGNOSED, NOT ATTEMPTED (regression-prone).** Traced the
  5 residual teleports this session: they are **one single-root cascade**, not five independent wedges.
  - Four of them (veh 102/196/262/307) teleport at the *exact same spot* — internal lane `:2436_0_1`
    pos ~13.3 — one after another (~every 125 s). Each is held by `CrossJunctionLeaderConstraint`
    (crossJL≈0) car-following a stopped leader that has crossed onto the downstream lane.
  - Following the chain: the `:2436` wedgers follow **veh 99** (stopped on `-2437_1` pos 5.1), which
    follows a queue on `-2437` whose head is **veh 78** (pos 20.1). Edge `-2437` feeds **TL junction
    2336**. So the whole cascade is rooted at ONE head-of-queue vehicle that never clears its TL green.
  - veh 78 at 2336: during red phases `RedLightConstraint` holds it (correct); during other steps it is
    held by `JunctionYieldConstraint=0` while its own `egoState='r'` (redundant with red, harmless) and,
    critically, by a `CrossJunctionLeaderConstraint` to **veh 152 sitting on the junction's OWN internal
    lane `:2336_2_0`** with a **negative gap (−6.53)** — an intra-junction "block-the-box" gridlock:
    152 occupies the junction and cannot exit, so 78 cannot enter behind it, so the queue never drains.
  - **Deeper trace (this session) reclassifies it as a THROUGHPUT cascade, not a localized block.**
    The negative cross-junction-leader gap is NOT a false leader — veh 152 genuinely straddles the
    junction entry, so the hard stop is correct block-the-box behaviour. And veh 152 is not permanently
    stuck: it DOES cross junction 2336 on its green (t≈316), then queues DOWNSTREAM on `-389_0` behind
    the next junction's backlog. No single vehicle is the culprit — queues chain across a block of
    junctions and the region stays saturated.
  - **SUMO-oracle confirmation (the decisive datum).** Same net + demand, vanilla SUMO 1.20.0 vs
    SumoSharp, edge `-2437` into TL junction 2336:
    - vanilla: queue forms transiently (peak 5 ~t=200) then **fully drains** — 0 stopped from t≈500 on;
      **15 distinct vehicles** pass through `-2437` over [0,800].
    - SumoSharp: queue forms and **never drains** — a steady **3 stopped from t≈300 onward**; only
      **10 distinct vehicles** get through.
    SumoSharp's per-junction throughput is lower than SUMO's, so queues SUMO drains become permanent
    standing jams that back up through junction 2436 and teleport — the aggregate junction-throughput
    residual (`NEED-junctionyield-impatience-saturation.md`), surfacing at moderate density.
  - **Faithful fix = the un-ported IMPATIENCE / arrival-time gap acceptance, NOT a patch.** SUMO's
    `MSLink::blockedByFoe` (MSLink.cpp:947-965) blends the foe arrival time toward ego's *braking*
    arrival as ego's `getImpatience()` grows with accumulated waiting (default `--time-to-impatience
    180`, `MSFrame.cpp:481`); the engine hardcodes `impatience==0`, so in a near-tie saturated cycle a
    vehicle waits for a perfect gap that never comes, whereas SUMO's growing impatience accepts a tighter
    gap and breaks the deadlock. Known tension: a prior session ported impatience byte-identically
    (goldens unaffected, zero effect on city-3000) but applying it to the APPROACH arm regressed
    `WillPassSaturationDiagTests` (0→15 stuck). So the work is *where/how* to apply the blend faithfully
    without re-breaking the saturated-grid stress test — a real design cycle, not a trigger-timing tweak.
  - **Success (when taken on):** synthetic-junction2 teleports → ~0 and the pop%↔density curve monotone
    within noise, with the full suite AND `WillPassSaturationDiagTests` green.
  - **Attempt log — E1 (merge-arm impatience), reverted.** Ported `getImpatience` = clamp01(waiting/180)
    and a faithful `computeFoeArrivalTimeBraking` blend into `SameTargetMergeConstraint` (SUMO's own
    arrival-time locus). Result: **full parity suite byte-identical (635 pass) — the impatience
    machinery is golden-safe** — but **zero effect on the repro (5→5)**. So the residual teleports are
    NOT held by the merge arm; they are held by the CROSSING arm (the approaching-foe stop-line yield is
    a *blanket* yield with no arrival-time gap acceptance to blend impatience into) and/or the
    red-light + block-the-box (`CrossJunctionLeaderConstraint`, exit-lane occupied) throughput cascade.
    Reverted per the gate (golden-safe but no value). **Implication for the fix:** impatience must go on
    the CROSSING arm, which first needs arrival-time gap acceptance ported there — precisely the change
    the prior session found regressed `WillPassSaturationDiagTests`. And if the true block is
    block-the-box (a genuinely full exit lane), impatience will NOT help at all — that part needs a
    throughput/`checkRewindLinkLanes` treatment, not impatience. This is the crux the dedicated task must
    resolve.
  - **Deliberately NOT force-landed.** Rushing the crossing-arm change would be either a quick patch
    (forbidden) or a large regression-prone change against the parity gate. It stays its own task with a
    dedicated design + the `WillPassSaturationDiagTests` gate.
- **T4 — regression guard. DONE (partial).** `tests/Sim.ParityTests/LowDensityTeleportTests.cs` runs
  the committed synthetic-junction2 through the in-process `SumoShim` path (engine-only, no SUMO) and
  asserts teleports ≤ 5 — locking the T1 (mechanism-A) fix against regression toward 10. The bound
  tightens toward vanilla's 0 when T3 lands.

## Tracker

- [x] T1 — havePriority-aware junction yield (TLS half): 10 → 5, goldens byte-identical
- [~] T2 — RedLightConstraint routed-movement link: investigated, byte-identical on goldens but worsens
      the repro (5→6) and needs a dedicated shared-lane test — DEFERRED, not landed
- [~] T3 — residual cascade (mechanism B): DIAGNOSED (oracle-confirmed) as a multi-junction THROUGHPUT
      cascade at TL 2336 — vanilla drains the -2437 queue (15 through), SumoSharp never drains it (3
      permanently stuck, 10 through). Faithful fix = the un-ported impatience / arrival-time gap
      acceptance; NOT attempted here — a dedicated feature (prior naive port regressed the stress test),
      not a patch. Blocked on its own design cycle + WillPassSaturationDiagTests gate.
- [x] T4 — committed low-density-teleport regression guard (teleports ≤ 5)

## Notes for the implementor

- Diagnosis was done with temporary env-gated instrumentation (`SS_TELEPORT_LOG`, `SS_TRACE_VEH`) and a
  throwaway spike, all reverted — the working tree is clean. Re-add equivalents locally if needed; do
  not commit them.
- SUMO 1.20.0 (pip `eclipse-sumo==1.20.0`, exact `SUMO_VERSION` match) is the oracle; regenerate the
  vanilla-vs-SumoSharp comparison on `synthetic-junction2` after each change. The offline `dotnet test`
  loop must stay SUMO-free.
