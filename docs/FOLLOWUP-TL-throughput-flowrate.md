# Follow-up: traffic-light approach throughput / flow-rate residual

**Status:** OPEN — the **sole remaining follow-up** from the serve-path drop-in work, deferred to a
later performance/parity pass. Everything else is done and on `main`: the drop-in (GAP-1/2/3) + the
P2-G junction fixes (Bug-1/2/3) are merged and were accepted **GREEN** by the Geneva acceptance seat
(see `docs/SERVE-PATH-PLAN.md` top section); the now-obsolete cooperative `informFollower` was retired
(`docs/HIGH-DENSITY-P2G2-COOPERATIVE-LC-DESIGN.md`). This throughput/flow-rate gap was the one minor
residual noted at acceptance — **not** a merge blocker, tracked here for the anticipated
performance-optimization pass.

## What it is

After the P2-G junction fixes (Bug-1 `<routing>` config parsing, Bug-2 RBL traffic-light exclusion,
Bug-3 red-held-foe `WillPass=false`), SumoSharp no longer gridlocks at traffic-light approaches — the
progressive-halting failure is gone and on-lane halting tracks vanilla. What remains is a **flow-rate /
throughput gap**: SumoSharp clears somewhat fewer through-trips per unit time than vanilla, and its
fair (non-sink) mean relative speed sits a little below vanilla's. The vehicles involved are **moving,
not stalled** — it is a tempo difference, not congestion.

## Measured magnitude

**Real box (Geneva acceptance re-run, 1000 s window):**
- Non-sink through-trips cleared in-window: **vanilla 52 vs SumoSharp 38** (~27% fewer; the ~14
  difference are still in transit at the cutoff, not stuck).
- Fair mean relative speed: **vanilla 0.838 vs SumoSharp 0.725**.
- Non-sink on-lane halting tracks vanilla the whole run (both ≤14, no climb) — confirming this is
  throughput, not gridlock.

**Synthetic witness (`scenarios/_repro/synthetic-junction2`, controlled, fresh binary):**
- Peak on-net halting **85 (SumoSharp) vs 45 (vanilla)** — roughly 1.9×, down from 101 pre-fix.
- Teleports **10 (SumoSharp) vs 0 (vanilla)** — down from 24 pre-fix.
- Mid-run arrivals lag vanilla by ~6–8 trips (down from ~27 pre-fix; final t=1000 arrivals 325 vs 327).
- On a **minimal 2-car TL case**, SumoSharp already reproduces vanilla's **lane-timing exactly**
  (green permissive left crosses `:C_5_0` at t=9, same as vanilla), with only a **~1.7 m/s
  approach-speed transient over 2 steps** that converges to an exact match by t=11 (SumoSharp slightly
  *faster*, not more cautious).

## Why it's hard to localize right now

On the minimal case the behavior is already at vanilla lane-parity, so there is **no single dominant
mechanism** to point at on the synthetic — the real-box gap looks like the **accumulation of many
small per-approach speed transients** across ~10 TL junctions over 1000 s, and/or second-order
effects (lane choice, insertion order, permissive-green approach-speed profile). A clean next step
needs a **real-box halting/flow trajectory** (geometry-free is fine) to see *which* junctions/movements
accumulate the lag, since the synthetic no longer isolates it.

## Candidate mechanisms to examine (in a later pass)

1. **Permissive-green approach-speed profile.** On the minimal case SumoSharp enters the junction
   ~1.7 m/s off vanilla for ~2 steps on a `'g'` permissive left — and it is going slightly *faster*
   than vanilla there (it brakes a touch *less* on approach), not slower. So the residual is a
   speed-*profile* mismatch, not excess caution. Suspect the minor-link cautious-approach arm
   (`couldBrakeForMinor`, `Engine.cs` ~6196, keyed on the static `request.Response` "is-minor" test)
   vs SUMO's live-`havePriority()` treatment at TL links: at a TL junction SUMO's `havePriority()` is
   the live signal state (`'G'` protected-green HAS priority → no cautious brake; `'g'`/`'r'`/`'y'` do
   not), whereas our static-`Response` test can mis-classify a link whose live state disagrees with its
   static row. Small per-approach speed errors in either direction accumulate into the flow-rate gap.
   Compare against SUMO's `MSVehicle::planMoveInternal` couldBrakeForMinor for a TL `'g'`/`'G'` link.
2. **Internal-junction (cont-turn) traversal speed.** Cont turns traverse `:C_*` internal lanes; a
   slightly different internal-lane speed/accel would slow turning movements specifically.
3. **Gap-acceptance headway at permissive greens** — whether SumoSharp accepts merge/cross gaps at the
   same threshold vanilla does.
4. **Insertion-order / lane-choice** second-order effects that shift where queues form.

## Guardrails for any fix here

- **Parity is the iron law.** Any change must keep all committed goldens byte-identical (or be gated
  behind an explicit opt-in fast-mode flag). Several of these arms are documented as byte-identical to
  committed priority-junction scenarios (11-priority-junction, 19-onramp-merge) — do not regress them.
- This is a likely candidate for the **performance-optimization pass** the owner anticipates; treat
  throughput parity and speed parity together (a faster-but-wrong flow is still wrong).

## Pointers

- Acceptance record + halting table: `docs/SERVE-PATH-PLAN.md` (top).
- Synthetic witness + controlled A/B/C numbers: `scenarios/_repro/synthetic-junction2/DIFF-SUMMARY.md`.
- The three landed fixes (on `main`): `689463e` (Bug-1+2), `9cd61b8` → `299b17f` (Bug-3, generalized to
  a red-held-foe `WillPass=false` mirroring SUMO's `mySetRequest`).
- informFollower retirement: `afec614` (removed the dead cooperative layer after these fixes obsoleted
  it; dense-LC `CoordinatedLaneChange` unchanged).
- Geneva re-verify-from-main prompt: `docs/GENEVA-SESSION-RECHECK-PROMPT.md`.
