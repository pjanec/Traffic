# HIGH-DENSITY-PLAN.md — verified gap findings + proposed plan

**Status:** CHECKPOINT (merged to main @baa0a73, 556 green). P0 (A/B/C1/C2/D) + P1-E + P1-F DONE. Remaining: P2-G multi-lane car-following gap, P2-H max-depart-delay, X1 attention-aware popping. Continuation handoff: `docs/HIGH-DENSITY-HANDOFF.md`.

### Owner steer (received)
- **Standing architectural principle — performance may beat exact SUMO parity in production.**
  Where a "faster but different, equally good" option exists it is allowed, **but it must be gated
  behind a CLI flag**. The parity-faithful behaviour stays the **default** so (a) automated dev
  checks (goldens) pass and (b) SUMO scenarios reproduce with similar output. Fast-but-different
  paths are opt-in only. (Matches CLAUDE.md's "fast-mode flag" escape hatch.)
- **Q1 → (b) RNG-insensitive parity model.** Do not port SUMO's PRNG draw order; make parity
  scenarios deterministic/RNG-insensitive, and validate sampling itself statistically.
- **Q2 (P1-E parity bar) → deferred**; owner wants more info when we reach P1-E. Not blocking P0.
  (Info provided; recommendation = two-tier: exact unit/parity on the router + edge-time
  aggregation machinery, statistical/robust-exact on the end-to-end congestion scenario.)
- **P1-E performance constraint (owner).** Rerouting must be **fast** — it must not materially slow
  the step loop. Production machines have many cores; **parallelize the reroute pass** where
  possible. Design implication: the periodic reroute is a read-mostly A* over a double-buffered
  edge-weight snapshot → fan out per-vehicle A* across cores (`Parallel.For`) reading an immutable
  per-interval weight snapshot, write route changes through the CommandBuffer at step end. This
  also feeds the "faster-but-different, gated" option (cheaper cadence/approx weights) for the
  non-parity production path.
- **P1-E scale (owner).** Assume ~10k concurrent vehicles; because `device.rerouting.period` is
  fixed, many vehicles fall due on the same (or nearby) tick — a reroute *thundering herd*. Design:
  **collect the due-this-tick vehicles into a batch, then run their A* searches in parallel**
  (`Parallel.For` over the batch, each reading the shared immutable edge-weight snapshot; A* is
  read-only over the graph so it is embarrassingly parallel), and apply the resulting route swaps
  through the CommandBuffer at step end. A per-vehicle phase offset (SUMO does this) spreads
  vehicles across ticks to flatten the herd; batching + parallel A* absorbs whatever remains.
  Router state must be per-thread/thread-local (no shared mutable open/closed sets).
- **Q3 → P2 empirical check after P0.** ✓
- **Q4 → start P0-A → P0-C → P0-B → P0-D.** ✓
- **Q5 → OK to `pip install eclipse-sumo==1.20.0` for golden regen.** ✓

---

**Superseded status line:** STEP 3 check-in (verification complete, awaiting owner steer before implementation).
**Branch:** `claude/sumosharp-high-density-0r91xo`.
**Source of truth:** `docs/SUMOSHARP-HIGH-DENSITY-FEATURES.md` (committed to this branch).
**Baseline (this checkout):** `dotnet build` clean (0 errors); `dotnet test` green —
`Sim.ParityTests` 466 passed / 3 skipped / 0 failed. SDK provisioned via
`apt-get install -y dotnet-sdk-8.0` (8.0.129) on the fresh VM. SUMO **not** installed
(only needed for golden regen / empirical checks, never for `dotnet test`).

The feature doc's current-state claims were written at checkout `137a047`, which is the
**direct parent** of this branch — so the tree it analysed is essentially this one. Every
claim below was re-confirmed against the live files at this checkout, with `file:line`.

---

## 1. Verified gap findings

### P0 — plumbing prerequisites

**P0-A — `.sumocfg <input>` + multi-file `route-files`/`additional-files` — CONFIRMED gap.**
- `src/Sim.Ingest/ScenarioConfigParser.cs:6` header: *"Parses the rung-1 subset of .sumocfg:
  `<time>` and `<processing>`."* It reads only `<time>`/`<processing>`/`<random_number>`
  (`ScenarioConfigParser.cs:23-26`); never `<net-file>`/`<route-files>`/`<additional-files>`.
- `src/Sim.Run/Program.cs:66-75,105` requires **exactly one** `*.net.xml`, `*.rou.xml`,
  `*.sumocfg` per scenario dir (`SingleFile` errors unless `matches.Length == 1`). No
  comma-list route-files, no additional-files anywhere.
- **Impact:** the real SumoData config (`vType.config.xml,vType_pedestrians.xml,
  vTypeDist.config.xml,box.rou.xml` + parkingArea additional-file) cannot be loaded at all.

**P0-B — `<vTypeDistribution>` resolution — CONFIRMED gap.**
- Zero matches for `vTypeDistribution`/`TypeDistribution` anywhere in `src/`.
- `src/Sim.Ingest/DemandParser.cs:138,181` resolve `type=` strictly as a direct vType-id
  lookup (`vTypesById`); no distribution / weighted sampling step exists.
- **Design flag (parity):** SUMO samples the member vType per vehicle from its own RNG stream.
  Reproducing *the same type-per-vehicle assignment* in a golden requires matching SUMO's PRNG
  draw order — which `DESIGN.md` ("determinism ladder") explicitly calls "brutal and not worth
  it early." **This is the single biggest parity-design question below (see §4 Q1).**

**P0-C — symbolic `departSpeed="max"` / `departLane="best"` / `departPos="stop"` — CONFIRMED gap
(and a latent crash, slightly worse than the doc states).**
- `src/Sim.Ingest/DemandParser.cs:6-10` header: symbolic values *"are NOT resolved here — that
  placement/defaulting logic is a Task 3+ concern."*
- `DemandParser.cs:151` `DepartSpeed: ParseNullableDouble(vehicleEl, "departSpeed") ?? 0.0`.
  `ParseNullableDouble` (`:311-315`) calls `double.Parse` on **any present** value, so a literal
  `"max"` throws `FormatException` — the `?? 0.0` only covers an *absent* attribute. The doc said
  "silently becomes 0.0"; in fact it crashes. Same shape for `departPos` (`:150`) and
  `departLane` via `ParseNullableInt` (`:152`).
- **Impact:** our pipeline sets `departSpeed="max" departLane="best"` on 100% of trips → ingest
  can't parse them today. `departLane="best"` is the non-trivial one (SUMO's `getBestLanes`
  considers downstream route + occupancy). `departPos="stop"` couples to parkingArea stops
  (needs P0-A additional-files).

**P0-D — engine writers for `--summary-output` / `--statistic-output` — CONFIRMED gap
(and the harness side is weaker than the doc implies).**
- `src/Sim.Run/Program.cs` (108 lines) has **no** summary/statistic writer — only FCD.
- `src/Sim.BenchCity/Program.cs:608-627` `WriteSummary` writes only `time,running,arrived,
  meanSpeed` per step — missing `halting,stopped,meanSpeedRelative`; no teleport tally
  (`:643-649` comment: phase-1 runs teleport-OFF, "no SUMO-style teleport count").
- **Harness nuance:** `Sim.Harness/SummaryOutputParser.cs` + `SummaryStepRecord.cs:10-14` parse
  only `time/running/arrived/meanSpeed` — **not** `halting/stopped/meanSpeedRelative`. There is
  **no** `--statistic-output`/`<teleports>` parser anywhere. So P0-D is not just "add a writer";
  it's **writer + extend the harness parser + add a statistic-output parser**.

### P1 — the two high-density levers

**P1-E — `device.rerouting` (periodic, congestion-reactive) — CONFIRMED gap.**
- Only a **one-shot, obstacle-triggered** reroute exists: `src/Sim.Core/Engine.cs:3049-3080`
  fires only when an active `ObstacleStore` obstacle sits on a vehicle's future edges, after
  `RerouteThresholdSeconds`. Router is **static free-flow** Dijkstra
  (`src/Sim.Ingest/NetworkRouter.cs:69-74,171-181`: `effort = length / max-lane-speed`).
- Zero matches for `astar`/`device.rerouting`/`adaptation-steps`/`routing-algorithm` in `src/`.
- **Missing sub-parts:** (1) live per-edge smoothed travel-time aggregation (`adaptation-steps`
  window); (2) periodic per-vehicle reroute gated by `probability`, phase-offset; (3) A* on the
  weighted graph; (4) thread-safe double-buffered edge-weight table.
- Existing reroute plumbing to reuse: `Engine.UpdateReroutes/RegisterRerouted/RerouteActive`,
  `CommandBuffer.cs:86-91 ReplaceRoute`.

**P1-F — bounded teleport valve (`time-to-teleport` jam) + counter — CONFIRMED gap (parsed but
inert).**
- Field `src/Sim.Ingest/ScenarioConfig.cs:25 double TimeToTeleport`; parsed
  `ScenarioConfigParser.cs:33` (default `-1.0`); defaulted `Engine.cs:956`. `grep TimeToTeleport
  src/ tests/` returns **only** those three sites — never read to drive behaviour. No per-vehicle
  stuck timer, no teleport action in the step loop.
- `src/Sim.Core/SimEvent.cs:17-18` even reserves an unemitted `Teleported = 3` enum:
  *"Reserved… Not emitted yet (teleport is off in phase 1)."*

### P2 — investigate-then-fix (grounded in existing repo evidence)

The doc asks for an empirical dense run vs vanilla SUMO. **Key structural finding: a *faithful*
dense run is blocked by P0** — the real SumoData config (multi-file cfg + `departSpeed="max"` +
`vTypeDistribution`) cannot load until P0-A/B/C land, so "run our dense deduced demand through
SumoSharp" is not possible pre-P0. Instead I gathered the repo's *existing* saturation evidence
(the `NEED-*.md` diagnostic docs and `_bench`/`_diag` scenarios that were built precisely to
probe this), which is more grounded than a fresh toy scenario would be:

**P2-G — junction behaviour at saturation — LIKELY gap (partly confirmed, partly open).**
- `NEED-junctionyield-impatience-saturation.md:1-16`: the city-3000 gridlock analysed there was
  **RESOLVED** (commit `0513bad`, a cont-turn/U-turn distance bug in `SameTargetMergeConstraint`;
  2231→0 stuck). But it leaves a real **faithfulness deviation open**: impatience is hardcoded 0
  vs SUMO's `--time-to-impatience 180`, byte-identical on current goldens only because no
  committed scenario waits long enough to exercise it.
- `NEED-multilane-density-willpass.md` (open, no RESOLVED banner): multi-lane (`-L 2`) density
  still gridlocks ~15% stuck, traced to a start-of-step-speed vs `willPass` ordering bug (engine
  reads foe raw speed, not computed `vNext`). **Confirmed & diagnosed, not fixed.**
- `NEED-multilane-junction-passage.md`: `-L 2` grid leaves ~60% stuck at stop lines (SUMO: 0).
  **Confirmed & reproduced, root cause hypothesized.**
- `NEED-priorityjunction-farrouted-foe-falsepositive.md`: `FindFoeVehicle` (`Engine.cs:2279,2302`)
  matches a foe if its route *ever* touches the contested lane (no proximity window) → false
  yields on large nets (58.8% stuck vs SUMO `meanSpeedRelative=0.98`). **Confirmed & traced, open.**
- No **committed parity-gate** scenario exercises a *saturated/queued* junction — all such
  coverage lives in ephemeral `_bench/`/`_diag/`. The golden suite is entirely free-flow / tiny
  vehicle counts (08=1 veh, 11=2, 26=2, 29=2).

**P2-H — `max-depart-delay` / insertion backlog — CONFIRMED gap.**
- `src/Sim.Core/Engine.cs:2275-2344 InsertDepartingVehicles` + `:2350 TryInsertOnLane`: on no-gap,
  `TryInsertOnLane` returns false (`:2396` "no safe gap yet — do not insert this step") and the
  vehicle is simply retried next step **forever** — no elapsed-wait check, no drop. The scope
  comment (`:2241-2245`) admits MSInsertionControl's retry/eviction bookkeeping was left out.
- `max-depart-delay`/`maxDepartDelay` has **zero** matches under `src/` — unparsed entirely
  (weaker even than teleport, which at least reaches the config record). SUMO
  (`MSInsertionControl.h:66`): vehicles waiting longer than this are **deleted** (`-1` = never).

---

## 2. Priority / sequencing (refined)

The doc's dependency order holds and I recommend keeping it. Refinements in **bold**:

1. **P0-A/B/C/D plumbing** — unblocks loading + measuring our real scenarios, and unblocks the
   P2 empirical check. **Do first.** Within P0, suggested internal order:
   **P0-A (multi-file cfg) → P0-C (symbolic departs) → P0-B (vTypeDistribution) → P0-D (outputs)**,
   because A is the loader spine, C is a latent crash on our inputs, B carries the hardest parity
   question, and D is independent and can land any time.
2. **P1-E device.rerouting** — biggest density lever; highest effort/risk.
3. **P1-F teleport valve + counter** — completes "dense but drains"; unlocks X1; depends on P0-D
   for the teleport tally.
4. **P2-G/H** — now that P0 exists, *run the real dense config vs vanilla* and fix only what the
   run proves is binding. P2-H is already a confirmed gap; several P2-G sub-gaps are confirmed in
   the `NEED-*` docs and could be pulled forward if density targets demand it.
5. **X1 attention-aware popping** — the strategic non-parity extra; only after P1-F.

---

## 3. Effort / risk per item

| Item | Effort | Risk to parity | Notes |
|------|--------|----------------|-------|
| P0-A multi-file cfg/routes/additional | Low–Med | Low | Parser + `Sim.Run` loader wiring; touches `ScenarioConfigParser`, `Program.cs`, ingest. |
| P0-B vTypeDistribution | Low code / **High parity** | **High** | Sampling is easy; matching SUMO's *exact* per-vehicle type draw for a golden is the hard part (Q1). |
| P0-C symbolic departs | Low (`max`) / **Med (`best`)** | Med | `departLane="best"` needs `getBestLanes`-equivalent; `departPos="stop"` needs P0-A additional-files. |
| P0-D summary/statistic writers | Low–Med | Low | Writer + extend harness parser + new statistic parser. Aggregates largely exist. |
| P1-E device.rerouting | **High** | Med–High | 4 sub-parts; edge-weight smoothing + A* + thread-safe double-buffer. Golden must be small & deterministic. |
| P1-F teleport valve | Med–High | Med | Port SUMO jam detection faithfully; exact teleport time/vehicle/tally parity is the bar. |
| P2-G junction saturation | Med–High | — | Verify-then-fix; some sub-gaps already confirmed in `NEED-*`. |
| P2-H max-depart-delay | Low–Med | Low | Add config key + eviction bookkeeping in `InsertDepartingVehicles`. |
| X1 attention-aware popping | Med | N/A (no parity) | Functional/statistical tests only; `RealismMask` gating P1-F + insertion. |

---

## 4. Design questions for the owner (please steer)

**Q1 (biggest). vTypeDistribution + symbolic-depart parity model.** Matching SUMO's *exact*
per-vehicle type assignment (P0-B) and any RNG-driven insertion choice requires reproducing
SUMO's PRNG stream/draw order, which `DESIGN.md` deliberately avoids early. Two options:
  - **(a) PRNG parity** — port SUMO's RNG + draw order so type/lane assignment is bit-identical.
    Highest fidelity, most effort, brittle.
  - **(b) Construct parity scenarios to be RNG-insensitive** — e.g. a `vTypeDistribution` whose
    members are behaviourally distinguishable but assigned by a rule we can match, or a
    single-member/`probability=1` distribution for the *parity* gate, and validate the *sampling
    distribution* statistically (per `parityMode:"statistical"`) in a separate scenario.
  My recommendation: **(b)** — keep exact-trajectory parity for scenarios we can make
  deterministic, and use statistical parity for the sampling itself. Do you agree, or do you want
  true PRNG parity?

**Q2. P1-E parity bar.** `device.rerouting` outcomes are sensitive to edge-time smoothing timing.
Do you want `scenarios/NN-reroute-congestion` held to **exact** trajectory parity (requires a
tiny, carefully-timed net where the route switch is unambiguous), or **statistical** parity on
the flow split? I recommend a **small exact** scenario if achievable, with a statistical fallback.

**Q3. P2 timing.** Do the P2 empirical dense run vs vanilla SUMO **after P0** (so the real config
loads), rather than now on a hand-built toy? I recommend after-P0. (I can still build a
single-file saturated probe earlier if you want P2 signal sooner.)

**Q4. Where to start.** I recommend starting with **P0-A → P0-C → P0-B → P0-D** as the first
batch, each with its own parity scenario, since it's the safe necessary foundation and unblocks
everything else. Confirm, or redirect.

**Q5. SUMO install.** I'll `pip install eclipse-sumo==1.20.0` (per `scripts/install-sumo.sh`)
when I first need to regenerate a golden. OK to do that as part of landing the first P0 scenario?

---

## 5. Running checklist (status per item)

- [x] STEP 1 — orient (CLAUDE.md, DESIGN.md, feature doc, harness, regen script, threading)
- [x] Baseline confirmed green (build + `dotnet test`: 466 parity pass / 3 skip / 0 fail)
- [x] STEP 2 — verify P0-A/B/C/D, P1-E/F at this checkout (all CONFIRMED, file:line above)
- [x] STEP 2 — P2-G/H evidence gathered (P2-H confirmed; P2-G likely; live dense run blocked pre-P0)
- [x] STEP 3 — this plan written
- [ ] **STEP 3 — owner steer received (Q1–Q5)** ← WE ARE HERE
- [x] P0-A multi-file cfg  ·  scenarios/41-multifile-cfg + SUMO 1.20.0 golden  ·  parity green (474)
- [x] P0-C1 symbolic departs (max/best + lane-stop)  ·  scenarios/42-symbolic-depart  ·  parity green (497)
- [x] P0-C2 parkingArea departPos=stop  ·  scenarios/48-parking-depart (parked-origin @198.0)  ·  parity green (556)
- [x] P0-B vTypeDistribution  ·  scenarios/43-vtypedist + statistical test  ·  parity green (507)
- [x] P0-D summary/statistic writers + harness parsers + comparator  ·  scenarios/44-summary-output  ·  parity green (521)
- [ ] P1-E device.rerouting (design: docs/HIGH-DENSITY-P1E-DESIGN.md; owner-approved +jitter/behavioural-accept/route-slot)
  - [x] P1E-1 config keys (+gated jitter flag) ✅
  - [x] P1E-2 edge-weight aggregation (ring-buffer moving average, isDelayed latch) ✅
  - [x] P1E-3 A* router + effort fn (== Dijkstra on fixed weights) ✅ (533 green)
  - [x] P1E-4 periodic reroute trigger + parallel batch + route-slot recycling + jitter + integration
  - [x] P1E-5 scenarios/45-reroute-congestion faithful anchor (EXACT parity, all-single-lane) + jitter/recycling tests (539 green)
  - [x] P1E-6 pre-insertion rerouting -> multi-lane route split exact (scenarios/46, behavioural; 543 green). Residual multi-lane pos/speed divergence = pre-existing P2-G gap (confirmed: identical without rerouting), tracked separately.
- [ ] P1-F teleport valve (design: docs/HIGH-DENSITY-P1F-DESIGN.md)
  - [x] P1F-1 config (time-to-teleport.remove) + WaitingTime !isStopped guard ✅
  - [x] P1F-2 jam detection + transfer queue/virtual-proceed + jam counter ✅
  - [x] P1F-3 scenarios/47-teleport-jam BIT-EXACT parity (follower teleports eA->eB @t=200, teleports=1); 549 green
- [ ] P2-G/H verify-then-fix (after P0, real dense config)
- [ ] X1 attention-aware popping (functional/statistical tests, no parity)
