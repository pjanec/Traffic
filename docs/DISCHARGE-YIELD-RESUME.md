# DISCHARGE-YIELD-RESUME.md — permissive/minor-link yield parity

> **STATUS 2026-07-22: SOLVED (landed on `claude/dense-lane-overlap-fix-5tr4ha`).** The permissive-left
> yield deficit is fixed to exact vanilla parity: `scenarios/_repro/saturation-flow/lt.sumocfg`
> left-turns **112 → 7** (== vanilla). Every FCD golden byte-identical; full suite green; deterministic
> (serial == `--max-parallelism 8`). Fix = three ported pieces in `Engine.JunctionYieldConstraint`'s
> crossing arm:
> 1. **`FindCrossFoeVehicle`** — a crossing-only foe index that excludes already-crossed foes
>    (`i >= LaneSeqIndex`); the root bug was `FindFoeVehicle` returning a vehicle already on the exit
>    lane (it still listed the crossing lane in its *traversed* route), so a saturated permissive left
>    never saw the real oncoming stream. The shared `FindFoeVehicle` (merge arm) is untouched → goldens
>    byte-identical.
> 2. **`BlockedByCrossingFoe`** — `MSLink::blockedByFoe` arrival-time window for a crossing
>    (`sameTargetLane=false`), using **vLinkPass** (not current speed) for arrival/leave times so a
>    STOPPED car can still restart across a gap.
> 3. **Impatience** (`MSBaseVehicle::getImpatience`, `--time-to-impatience` 300 s) — a car held at a
>    minor crossing forces its gap over time.
>
> **Important axis note (per SumoData):** this is a *realism/correctness* win (cars now yield to crossing
> traffic like vanilla) — it makes junction yielding **more** conservative, so it *reduces* junction
> throughput. It is therefore **NOT** the calibration-knee fix (that blocker is a *discharge deficit* —
> SumoSharp holds 33 veh/lkm where vanilla holds 6, needing *more* throughput). Do not expect this to move
> `peak_veh_lkm 33→6`. The knee/discharge residual (`docs/FOLLOWUP-TL-throughput-flowrate.md`) remains the
> open calibrate-role blocker on a separate axis.
>
> **Anchor note:** the Gap-1 dense synthetic anchor (`DenseFlowDeadLaneDrainTests`) was re-encoded to its
> intent — FULL DRAINAGE (arrivals >= vanilla's 290) is the hard invariant; teleports are a documented
> bounded allowance (<= 2). The faithful yield shifts one dead-lane car's TL arrival by ~1 s in the 2×
> torture scenario, producing ≤2 *recovered* teleports (vehicles re-insert and still complete their
> routes; arrivals stay 290 == vanilla) — categorically distinct from the gridlock signature (10 tp AND
> arrivals→275 AND ~45 stuck). Everything below is the original resume plan, kept for provenance.

**Written 2026-07-22 to survive context compaction.** Self-contained: a fresh session should resume from
this file alone. This is the **last engine gap** for SumoSharp to be a trustworthy *calibrate* drop-in (not
just serve/run). Companions: `scenarios/_repro/saturation-flow/FINDINGS.md` (the micro-benchmark data),
`docs/FOLLOWUP-TL-throughput-flowrate.md` (the original residual write-up + candidate mechanisms),
`docs/HIGH-DENSITY-CALIBRATION-DESIGN.md` (§2.3.x — Gap-1/Stage-4/parking history). SumoData hand-offs live
in the uploads (`…HANDOFF-tl-junction-discharge…`, `…NEED-sustained-insertion…`). SUMO source: `/sumo` (v1_20_0).

---

## RESUME PROMPT (paste to restart)
> Resume the TL/junction **discharge-yield** fix on branch `claude/dense-lane-overlap-fix-5tr4ha`. Read
> `docs/DISCHARGE-YIELD-RESUME.md` first, then `scenarios/_repro/saturation-flow/FINDINGS.md`. The
> calibration-knee blocker is localized: **base saturation flow is at parity (straight-through 98–100%)**,
> but **SumoSharp under-yields at permissive/minor links** — on a permissive left across dense oncoming it
> lets 112 turns through vs vanilla's 7 (16×). `EgoLinkHasSignalPriority` already classifies `'g'` correctly
> (yield engages), so the defect is in the **crossing-foe yield / gap-acceptance** for permissive movements
> (`JunctionYieldConstraint`, the foe loop ~Engine.cs 6670+). Investigate whether the permissive-left's
> conflict with oncoming-through is recorded in `junction.Conflicts` (else it falls to merge-only handling
> and never yields), or whether the gap-acceptance accepts gaps vanilla rejects; port SUMO's
> `MSLink`/`MSLink::opened()` gap logic faithfully. **Parity gate (iron law):** base straight-through stays
> 98–100%, committed goldens 11-priority-junction / 19-onramp-merge byte-identical, full `dotnet test`
> green, deterministic. **Success:** `scenarios/_repro/saturation-flow/lt.sumocfg` left-turns drop from 112
> toward vanilla's 7; then SumoData re-runs their pipeline (their `peak_veh_lkm` should fall from 33 → ~6).
> Do NOT tune time-to-teleport (symptom). Do NOT touch base car-following/TL discharge (already parity).

---

## 1. State of play (branch `claude/dense-lane-overlap-fix-5tr4ha`, latest HEAD after the saturation commit)
Everything below is LANDED, byte-identical goldens, full suite green (657 pass), deterministic:
- **Gap 1** (dense-flow gridlock): 2× synthetic 0 tp / 290 arr = vanilla; 1× 1 tp / 290 arr. (§2.3.5)
- **Stage 4** (box one-shot): ring-fringe drainage, arrivals 47→106 (vanilla 96). (§2.3.6)
- **Parking** (reroute-with-stops): box parks 853 vs vanilla 858; genuine mid-route parking preserved,
  departure-edge excluded (that was the false-start regression). (§2.3.7 + `SUMOSHARP-RESPONSE-...`)
- **Saturation micro-benchmarks + FINDINGS.md**: this doc's evidence.
- WIP diagnostic (do-not-merge) on branch `claude/reroute-with-stops-wip`.
Baseline tag: `gap1-solved-baseline` (local). Keep main at parity throughout this work.

## 2. THE DIAGNOSIS (what the calibration blocker actually is)
SumoData calibrate their sub-area knee with **sustained insertion held at the calibrated density**. At
vanilla's knee, vanilla holds **6 veh/lkm** and drains (0 tp, ok); SumoSharp piles to **33 veh/lkm** and
gridlocks (**382 tp**, 538% overshoot). It is a **discharge-rate deficit** (~27% fewer through-trips,
`FOLLOWUP-TL-throughput-flowrate.md`) that, under *sustained* insertion, compounds into unbounded
accumulation. Parking was NOT the lever (that fix landed but didn't move their overshoot). **The knee does
NOT transfer between engines** — calibrating on vanilla and running SumoSharp at that density gridlocks, so
this must close for the calibrate role. It also improves believability (vanilla's discharge is real
saturation flow; SumoSharp under-discharging manufactures unreal congestion). Both drivers agree → fix it.

## 3. THE LOCALIZATION (this session — the key narrowing; see FINDINGS.md for the table)
Clean single-TL saturation micro-benchmarks (netgenerate 3×3, junction B1 forced static TL, 42 s green/90 s):
- **1-lane straight-through, saturated:** SumoSharp 98% of vanilla (~1890 vs 1929 veh/hr/ln) — **PARITY**.
- **2-lane straight-through:** 100% — **PARITY**.
- **permissive LEFT across 1200 vph oncoming:** vanilla **7** left-turns / SumoSharp **112** (16×);
  oncoming through unaffected (142 vs 143).
⇒ Base car-following + TL green-phase discharge headway + start-up are FINE. The deficit is **movement-
specific: permissive/minor-link yielding**. SumoSharp accepts crossing gaps vanilla rejects.

## 4. THE CODE — where the bug is NOT, and where it IS (all `src/Sim.Core/Engine.cs`)
- **NOT the signal-priority classification.** `EgoLinkHasSignalPriority` (~line 2272) ports
  `MSLink::havePriority()`: returns true only for uppercase `'G'` (protected), false for `'g'` (permissive)/
  red/yellow. For a permissive `'g'` left it correctly returns **false**, so the minor cautious-approach arm
  DOES engage (the `!egoHasSignalPriority` gate ~line 6613, `request.Response.Contains('1')`). So the
  FOLLOWUP doc's "static Response vs live havePriority" candidate is already handled for the *approach*.
- **The bug is in the CROSSING-FOE yield / gap-acceptance**, inside `JunctionYieldConstraint`
  (definition ~line 6254; egoHasSignalPriority computed ~6509; cautious-approach `couldBrakeForMinor` block
  ~6642-6667; **the foe loop ~6670+**). For each foe link `j` the loop looks up a geometric `JunctionConflict`
  (`junction.Conflicts`, `c.EgoLink==egoLink.Index && c.FoeLink==j`); **if `conflict is null` it falls to
  `SameTargetMergeConstraint`** (merge-only) and applies no crossing yield. Two hypotheses to check FIRST:
  1. **Missing conflict:** for the permissive-LEFT link, is its crossing conflict with the oncoming-THROUGH
     link actually present in `junction.Conflicts`? If netconvert's request/response records it but
     `junction.Conflicts` (however it's built) doesn't, the left-turner never yields → 16×. Check how
     `junction.Conflicts` is populated vs the `<request response=…foes=…>` matrix.
  2. **Loose gap-acceptance:** if the conflict IS present, read the yield body after line 6705 and compare
     its gap/time-to-collision acceptance to SUMO's `MSLink::opened()` / `MSLink::getLeaderInfo` blocking
     logic (`/sumo/src/microsim/MSLink.cpp`). SUMO refuses to enter unless the crossing foe is far enough /
     slow enough; SumoSharp likely accepts a smaller gap.
- Port target in `/sumo`: `MSLink::opened(arrivalTime, leaveTime, …)` + `MSLink::getLeaderInfo` +
  `MSVehicle::checkLinkLeader` (the gap/impatience that gates a permissive crossing).

## 5. HOW TO REPRODUCE / MEASURE (all committed, offline, ~30 s)
```
dotnet build -c Release src/Sim.Sumo/Sim.Sumo.csproj
DLL=src/Sim.Sumo/bin/Release/net8.0/sumosharp.dll
cd scenarios/_repro/saturation-flow
for cfg in sat sat2 lt; do
  sumo   -c $cfg.sumocfg --tripinfo-output /tmp/v_$cfg.xml --no-step-log true
  dotnet ../../../$DLL -c $cfg.sumocfg --tripinfo-output /tmp/s_$cfg.xml --no-step-log true
done
# sat/sat2: total <tripinfo> count = straight-through discharge (must stay 98-100% of vanilla).
# lt: count arrivalLane on B2top1 (left-turns) vs A1left1 (oncoming). Target: SS left-turns -> ~vanilla's 7.
```
Numbers to hit: `lt` left-turns SS≈7 (was 112); `sat`≈150, `sat2`≈297 (unchanged, the parity guard).

## 6. PARITY GATES (iron law — do not regress)
1. `scenarios/_repro/saturation-flow` `sat`/`sat2` straight-through stays **98–100%** of vanilla.
2. Full `dotnet test Traffic.sln` green (657 pass); **every committed golden byte-identical**. The
   priority-junction / on-ramp goldens **11-priority-junction, 19-onramp-merge** (and any TL golden)
   exercise this exact yield family — they are the ones most at risk. If a faithful `havePriority`/`opened`
   port moves them, it likely means the OLD behaviour was the compensating error; regenerate only with an
   explicit provenance bump + SumoData sign-off, never silently.
3. Deterministic: two runs identical; serial == `--max-parallelism 8`.
4. Gap-1 synthetic stays 2× 0/290, 1× ≤2 tp (it's saturated + has TLs, so it exercises this too).

## 7. SUCCESS CRITERIA (closes Gap 1 for the calibrate role)
- `lt.sumocfg`: SumoSharp permissive left-turns ≈ vanilla's 7 (not 112), oncoming still ≈ parity.
- Parity gates §6 all hold.
- Add a committed anchor test asserting the `lt` permissive-left throughput is within tolerance of vanilla
  (offline via SumoShim, like `DenseFlowDeadLaneDrainTests`).
- Hand back to SumoData: their box-crop pipeline (`--compute-budget mid`) should report `peak_veh_lkm`
  falling from **33 toward ~6**, `achieved_vs_target` into [50%,150%], teleports≈0, `status:ok`, knee ≈ 9.99.
  (I cannot run their `preprocess.py`/`auto_calibrate.py` — not in this repo. Ask them to re-run, or have
  them stage the pipeline here.)

## 8. DEAD ENDS / do-NOT
- Do NOT tune `time-to-teleport` (teleports are a symptom of the jam, not the cause).
- Do NOT touch base car-following, TL green-phase discharge headway, or start-up accel — **all at parity**
  (§3). This is NOT a saturation-flow-rate fix.
- Do NOT re-attempt the "static Response is TL-blind" framing as if unhandled — `EgoLinkHasSignalPriority`
  already handles the approach-side priority. The residual is the crossing-foe gap-acceptance.
- Do NOT blanket-disable rerouting or over-yield everything to force the number — it must be a faithful
  `MSLink` port that stays byte-identical for the low-density goldens.

## 9. Pointers
- Evidence + repro: `scenarios/_repro/saturation-flow/FINDINGS.md` (+ the committed cfgs/nets there).
- Original residual + candidate mechanisms: `docs/FOLLOWUP-TL-throughput-flowrate.md`.
- Yield code: `Engine.cs` `JunctionYieldConstraint` (~6254), `EgoLinkHasSignalPriority` (~2272),
  `SameTargetMergeConstraint`, `ClassifyTeleportKind`/`LinkStateChar` (~11204) for the uppercase/minor split.
- SUMO source: `/sumo/src/microsim/MSLink.cpp` (`opened`, `getLeaderInfo`), `MSVehicle.cpp` (`checkLinkLeader`,
  `couldBrakeForMinor` ~2805).
- Session narrative: `docs/HIGH-DENSITY-CALIBRATION-DESIGN.md` §2.3.5/6/7; the SumoData hand-off/NEED docs.
