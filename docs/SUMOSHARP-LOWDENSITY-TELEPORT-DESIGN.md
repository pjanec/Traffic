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
- **T3 — priority-junction on-junction wedge (mechanism B).** The 5 residual teleports are all at a
  priority (uncontrolled, state `'M'`) junction — four vehicles wedging on the same internal lane
  `:2436_0_1`, plus one red case — the pre-existing on-junction minor-turner / cascade residual
  (`ISSUE2-JUNCTION-TELEPORT-DESIGN.md` §4-CORRECTION), NOT TLS-related. Separate, later stage; prior
  attempts on these arms regressed `WillPassSaturationDiagTests`, so treat with care.
  **Success:** synthetic-junction2 teleports → ~0 and the pop%↔density curve monotone within noise,
  full suite green.
- **T4 — regression guard. DONE (partial).** `tests/Sim.ParityTests/LowDensityTeleportTests.cs` runs
  the committed synthetic-junction2 through the in-process `SumoShim` path (engine-only, no SUMO) and
  asserts teleports ≤ 5 — locking the T1 (mechanism-A) fix against regression toward 10. The bound
  tightens toward vanilla's 0 when T3 lands.

## Tracker

- [x] T1 — havePriority-aware junction yield (TLS half): 10 → 5, goldens byte-identical
- [~] T2 — RedLightConstraint routed-movement link: investigated, byte-identical on goldens but worsens
      the repro (5→6) and needs a dedicated shared-lane test — DEFERRED, not landed
- [ ] T3 — priority-junction on-junction wedge (mechanism B): the 5 residual teleports
- [x] T4 — committed low-density-teleport regression guard (teleports ≤ 5)

## Notes for the implementor

- Diagnosis was done with temporary env-gated instrumentation (`SS_TELEPORT_LOG`, `SS_TRACE_VEH`) and a
  throwaway spike, all reverted — the working tree is clean. Re-add equivalents locally if needed; do
  not commit them.
- SUMO 1.20.0 (pip `eclipse-sumo==1.20.0`, exact `SUMO_VERSION` match) is the oracle; regenerate the
  vanilla-vs-SumoSharp comparison on `synthetic-junction2` after each change. The offline `dotnet test`
  loop must stay SUMO-free.
