# ISSUE2-JUNCTION-TELEPORT-DESIGN.md — teleport yield/jam classification + the count gap

**Status:** IN PROGRESS. Issue 2, re-diagnosed by the serve-path session as a **teleport
classification / yield-wait** divergence (parking, the earlier confound, is fixed and accepted). Repro:
`scenarios/_repro/synthetic-junction/` (irregular unsignalized-priority net; the uniform 8×8 grid does
NOT show it). Branch: `claude/sumosharp-junction-row-issue2` (based on `drop-in-binary`).

## 1. The divergence (reproduced)

`--end 1000`, identical flags, only the binary differs:

| | vanilla | SumoSharp (before) | SumoSharp (after classification) |
|---|---|---|---|
| jam | 0 | 75 | **31** |
| yield | 3 | 0 | **44** |
| wrongLane | 0 | 0 | 0 |
| total | 3 | 75 | 75 |

Congestion is *equal* (sustained ≥120 s halts: vanilla 256 vs SumoSharp 205 — SumoSharp is not more
stuck), yet SumoSharp fires 25× more teleports. Two independent gaps, now separated by the classification:

- **Yield 44 vs 3** — minor-link waiters (front vehicle whose next junction link is minor, waiting for a
  right-of-way foe).
- **Jam 31 vs 0** — major-link vehicles blocked by downstream congestion. Vanilla has **zero** jam
  teleports even with impatience disabled.

## 2. SUMO's teleport logic (vendored: MSLane.cpp:2257-2300)

Only the **frontmost non-`<stop>`-stopped vehicle per lane** (`firstNotStopped`) is a candidate. It
teleports when `r1 = ttt>0 && firstNotStopped->getWaitingTime() > ttt` (consecutive wait, resets on any
movement >0.1 m/s). The jam/yield/wrongLane split is a **label** applied after r1 fires:
`wrongLane = !appropriate()`; else next link minor (`!havePriority()`) → **yield**; else → **jam**
(`havePriority() = state ∈ 'A'..'Z'`, uppercase == priority).

## 3. Part 1 — classification (LANDED, byte-identical)

Ported the MSLane.cpp:2272-2294 split into `TeleportVehicle`/`ClassifyTeleportKind` (Engine.cs): find
ego's next junction link (first internal lane on its route sequence), read its current state char (live
TL phase char for a TL link, else the static `<connection state=…>` char, now parsed into
`Connection.State`), and classify minor→yield / major→jam. Surfaced `TeleportCountYield` /
`TeleportCountWrongLane`; the drop-in `--statistic-output` now emits all three. **wrongLane** is not
produced yet (documented simplification: every in-scope scenario reports 0). Verified byte-identical:
full suite 613 green, scenario 47's golden stays `jam=1` (its teleport link is `state="M"`, major).

## 4. Part 2 — the count gap: ROOT CAUSE FOUND

### 4a. The primary cause — the crossing yield is a BLANKET stop, not arrival-time gap acceptance
`JunctionYieldConstraint`'s crossing-foe arm (Engine.cs:6335) is:
```
takesCrossingYield = !(egoOnInternal || foeWillNotPass || foeNotApproaching || foeYieldsThisStep || ignoresFoe)
```
None of the escape conditions is an **arrival-time** check. So ego stops completely whenever *any*
priority foe is within its reservation distance of the conflict. SUMO's `MSLink::blockedByFoe`
(MSLink.cpp:981-1013) instead compares arrival-time windows: ego may **cross as the leader** when it
clears the conflict before the foe arrives (`foeArrivalTime > egoLeave + lookAhead`), and only yields on
a genuine window overlap. The merge arm already does this (`BlockedByMergeFoe`, 6964); the **crossing arm
never got it** — it blanket-yields. On a busy foe stream there is almost always a foe within reservation
distance, so SumoSharp's minor-road vehicle waits for a *completely empty* stream → freezes to the 120 s
cutoff → teleports. SUMO darts through the gaps between foes. **This is the dominant cause of the yield
count gap (44 vs 3).**

### 4b. The secondary refinement — impatience (MSLink.cpp:947-965)
On top of arrival-time RoW, SUMO grows impatience with waiting time (`getImpatience() = base +
waitingTime/gTimeToImpatience`, default `--time-to-impatience 180`) and, when impatient, blends the foe's
arrival time toward its braking arrival (`foeArrivalTime = (1-imp)·foeAT + imp·fatb`) — a long waiter
assumes the foe will brake and forces through. **Confirmed the lever:** vanilla with
`--time-to-impatience 0` rises 3→12 (still all yield, 0 jam); so arrival-time RoW alone (no impatience)
already gets vanilla to 12, and impatience closes 12→3. So 4a is the big lever, 4b the finisher.

### 4c. The jam gap (31 vs 0) is a cascade of 4a
Even impatience-off vanilla has **0 jam**. SumoSharp's 31 jam teleports are major-link vehicles queued
behind the frozen minor-link yielders of 4a: a blanket-yielding minor vehicle freezes at a junction, its
approach backs up, and the major-link vehicle feeding that approach is now "jammed" behind a standing
queue → jam teleport. Fixing 4a (letting the minor vehicles find gaps) should drain those queues and
clear most of the 31 jam teleports without a separate fix. (Residual creep / 2→1 lane-drop throughput to
be re-measured only if jam does not fall after 4a.)

## 4bis. The count-gap fix plan (faithful, not a cap/relabel)
Port SUMO's arrival-time RoW into the crossing-yield arm, mirroring the existing merge arm:
1. Compute ego's arrival/leaving window at the crossing conflict (the `getMinimalArrivalTime` machinery
   `SeenToInternalLaneEntry` + the conflict geometry already feed the merge arm) and the foe's window.
2. Replace the blanket `takesCrossingYield` with `BlockedByMergeFoe`-style arrival-time logic adapted to
   a *crossing* conflict (windows from `MSLink::blockedByFoe`, MSLink.cpp:981-1013): don't yield when ego
   clears before the foe arrives.
3. Add impatience: grow it from `WaitingTime`, blend `foeArrivalTime` toward the braking arrival.
**Parity risk is real and must be gated by verification** — this changes when minor-road vehicles cross
at *every* priority junction, so goldens 08/11/26/27/34/38/39/40 must be re-run and stay byte-identical
(the arrival-time RoW must reduce to the same wait where no early gap exists). If any moves, the port is
wrong and is corrected, never the golden. Impatience stays 0 while `WaitingTime==0`, preserving
uncongested goldens. This is a substantial, delicate RoW change warranting its own focused pass.

## 5. Gates
Every existing golden byte-identical (47-teleport-jam + junctions 08/11/26/27/34/38/39/40 + determinism
D1/D8); `dotnet test` green; no tolerance loosened. On synthetic-junction: jam→vanilla-ish (0–3), yield
matching, audit PASS, mean rel-speed converging. New golden mirroring scenario 47 (unsignalized
minor-link junction + busy foe stream) asserting category+count vs vanilla.
