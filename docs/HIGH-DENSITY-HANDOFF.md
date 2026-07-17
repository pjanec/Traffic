# HIGH-DENSITY-HANDOFF.md — checkpoint handoff for the continuation session

**Checkpoint:** merged to `main` at commit `baa0a73` (feature branch
`claude/sumosharp-high-density-0r91xo` == `main` == this tip). Full suite **green: 556 passed /
3 skipped / 0 failed** (`Sim.ParityTests`) + 1 (`Sim.Host.Tests`). 31 commits of high-density work.

This document is the single entry point for the next session. It explains WHAT is done, HOW the
work is done (rules), and WHAT remains, so a fresh session can continue on the same branch without
re-deriving context.

---

## 1. The mission (unchanged)

Make SumoSharp run **optimal high-density sub-area traffic** the way vanilla SUMO does — with
`device.rerouting` + a bounded `time-to-teleport` valve — at **behavioural parity to SUMO 1.20.0**,
and lay groundwork for engine-only "extras" (attention-aware popping). The product context, the
measured density numbers (strict no-cheating clears ~2.7 veh/lane-km; rerouting + 120s teleport
valve reaches ~7 at <1% pops), and the P0/P1/P2/extras breakdown are in
**`docs/SUMOSHARP-HIGH-DENSITY-FEATURES.md`** (the source-of-truth spec). Verified gap findings +
owner steers are in **`docs/HIGH-DENSITY-PLAN.md`**.

## 2. Operating rules (READ — these govern how to work here)

- **Repo bar is behavioural parity to SUMO 1.20.0.** Offline `dotnet test` runs the engine against
  **committed goldens**; SUMO is NEVER needed for the test loop. SUMO (pip `eclipse-sumo==1.20.0`)
  is installed ONLY to regenerate/author goldens and for investigation. See `CLAUDE.md`, `docs/DESIGN.md`.
- **Design-first.** For each new feature: ground it in the vendored SUMO source (`/sumo/src/...`),
  write a design doc, then implement. No ad-hoc dev. (Docs: `HIGH-DENSITY-P0/P1E/P1F-DESIGN.md`.)
- **Every SUMO-parity feature lands with a dedicated `scenarios/NN-*` case**: inputs + a golden
  regenerated from vanilla SUMO 1.20.0, wired into `tests/Sim.ParityTests`, passing within
  `tolerance.json`. Commit the code + scenario + golden together.
- **Additive / gated / byte-identical.** New behaviour is gated (config default = off / spec-kind =
  Given) so **all prior goldens stay byte-identical**. Verify with the full suite every time.
- **Owner steers (standing):**
  1. **Performance may beat exact SUMO parity in production — but only behind a CLI flag.** The
     parity-faithful path is the default; faster-but-different is opt-in. (e.g. `device.rerouting.jitter`.)
  2. **RNG-insensitive parity** (Q1b): don't clone SUMO's PRNG draw order; make parity scenarios
     deterministic; validate sampling statistically.
  3. Rerouting must be **fast + parallel** (10k vehicles; batch + `Parallel.For` over a frozen
     snapshot). Done for P1-E.
  4. **Behavioural/statistical acceptance is fine at the end-to-end level** where bit-exact trajectory
     would just lock us to SUMO quirks; keep exact for the deterministic machinery + a faithful anchor.
- **Orchestration loop (conserve Opus budget):** Opus plans/decomposes/reviews; **Sonnet subagents do
  the implementation + source-reading volume** (delegate with a precise self-contained spec + the
  exact acceptance gate). **Opus reviews HARD** — read the diff, re-run the suite, verify the
  scenario golden independently; never trust a subagent's "done" report.
- **Git:** work on `claude/sumosharp-high-density-0r91xo`. Commit per feature. `git config
  user.email noreply@anthropic.com && user.name Claude`. **Avoid backticks in `git commit -m`
  bodies** (bash command-substitutes them). Push with `-u origin <branch>` + retry. Merging to main
  requires explicit owner permission (granted for this checkpoint).
- **Golden regen:** `pip install eclipse-sumo==1.20.0`; then per scenario:
  `sumo -c config.sumocfg --fcd-output golden.fcd.xml --fcd-output.acceleration --precision 6
  --save-state.times 1 --save-state.files golden.state.xml [--summary-output ... --statistic-output ...]
  --no-step-log true`. XML comments must NOT contain `--` (SUMO rejects it). Write `provenance.txt`.
- **Build/test:** `dotnet build Traffic.sln -c Release`; `dotnet test Traffic.sln -c Release`. SDK on
  a fresh VM: `apt-get update && apt-get install -y dotnet-sdk-8.0` (the `dotnet-install.sh` endpoint
  is blocked; use apt). `netconvert` (from the SUMO pip install) builds synthetic nets.

## 3. What is DONE (all committed + on main, each SUMO-golden-verified)

| Item | What | Scenario | Parity | Key files |
|------|------|----------|--------|-----------|
| **P0-A** | multi-file `.sumocfg <input>` (net/route/additional-files) + DemandParser merge + `LoadScenario(cfg)` | `41-multifile-cfg` | exact | ScenarioConfig(Parser), DemandParser, Engine.LoadScenario, Sim.Run |
| **P0-C1** | symbolic `departSpeed="max"`/`departLane="best"`/lane `departPos="stop"` | `42-symbolic-depart` | exact | `DepartValue.cs`, DemandParser, Engine insertion, EngineSnapshot |
| **P0-B** | `<vTypeDistribution>` (both syntaxes) + per-entity seeded member draw | `43-vtypedist` | exact + statistical | DemandParser, DemandModel, Engine.ResolveEffectiveTypeId |
| **P0-D** | `--summary-output` (running/halting/stopped/meanSpeed/meanSpeedRelative) + `--statistic-output <teleports>` + harness parsers/comparator | `44-summary-output` | exact | Sim.Harness Summary*/Statistic*, Engine aggregates, Sim.Run |
| **P1-E** | `device.rerouting`: live edge-weight smoothing (ring-buffer + `isDelayed`), A* (≡ Dijkstra), periodic **+ pre-insertion** reroute, parallel batch, route-slot recycling, gated jitter | `45-reroute-congestion` (single-lane **bit-exact**), `46-reroute-multilane` (route-split behavioural) | exact machinery + route-exact | `RerouteEdgeWeights.cs`, NetworkRouter (A*), Engine (UpdatePeriodicReroutes/PreInsertionReroute/UpdateRerouteEdgeWeights) |
| **P1-F** | bounded teleport valve (`time-to-teleport` jam): frontmost-stuck-per-lane, strict `>`, transfer queue + virtual-proceed, `time-to-teleport.remove`, jam counter | `47-teleport-jam` | **bit-exact** | Engine (CheckJamTeleports/TeleportVehicle/ProcessTransferQueue), VehicleRuntime.InTransfer, Sim.Run/StatisticWriter |
| **P0-C2** | parkingArea `departPos="stop"` (no-cheating parked origins) | `48-parking-depart` | exact | `ParkingArea.cs`, DemandParser, Engine.ResolveParkingAreaStops |

Design docs: `HIGH-DENSITY-P0-DESIGN.md` (P0-A/B/C1/C2/D), `HIGH-DENSITY-P1E-DESIGN.md` (rerouting,
incl. §0.5 the 3 owner-approved decisions + §11 pre-insertion), `HIGH-DENSITY-P1F-DESIGN.md` (teleport).

**Net effect:** the SumoData high-density config now loads + runs end-to-end in SumoSharp
(multi-file cfg + vTypeDistribution + symbolic departs + parked origins + rerouting + teleport +
calibration outputs), each validated against a vanilla SUMO 1.20.0 golden.

**Scenario numbering wart:** HD scenarios reuse numbers 41–48, which COLLIDE numerically with
pre-existing scenarios of the same numbers (e.g. `41-forced-turn-lane` vs `41-multifile-cfg`). Dir
names are unique so tests/regen work fine; it's cosmetic. If it bothers you, renumber the HD set to
60+ (churns goldens/tests) — otherwise leave it.

## 4. What REMAINS (the continuation work)

### P2-G — multi-lane car-following / lane-distribution gap (HIGHEST VALUE for real dense runs)
- **Confirmed, pre-existing, independent of anything built here.** On a **2-lane** road under load,
  SumoSharp diverges from SUMO by **~7 m / 2.6 m/s** — proven identical **with rerouting OFF**
  (vehicles routed directly on a 2-lane detour) as with it on, so it is a lane-distribution /
  car-following gap, NOT a rerouting bug. Already documented in `docs/NEED-multilane-density-willpass.md`
  and `docs/NEED-multilane-junction-passage.md` (a start-of-step-speed vs `willPass` ordering bug +
  a junction-passage gridlock). This is what stands between "works" and "faithfully dense" on real
  (multi-lane) SumoData nets.
- **Next step:** run the empirical dense check the plan calls for — a representative dense multi-lane
  config, SumoSharp vs SUMO, localise the divergence to the specific lane-change/`willPass` issue,
  then fix (likely in the LC2013 / junction `willPass` path). Reproduce with a committed diagnostic
  scenario. This is potentially deep (lane-change model) — scope with the owner.

### P2-H — `max-depart-delay` / insertion backlog
- Confirmed gap: a vehicle that can't insert backlogs **indefinitely** (never dropped);
  `max-depart-delay` is unparsed. `Engine.InsertDepartingVehicles`/`TryInsertOnLane`. Smaller than P2-G.
  Add the config key + an eviction-after-delay path; scenario from SUMO with `--max-depart-delay`.

### X1 — attention-aware / camera-based selective popping (the strategic non-parity extra)
- **No vanilla parity** (SUMO can't express it) — validate with functional/statistical tests, not
  goldens. Idea: a `RealismMask` (per-edge `MayPop`/`MayTeleport`, settable per step from a camera
  frustum, double-buffered) that gates the **P1-F teleport action** (done — the hook exists) and the
  spawn/despawn path: strict no-cheating in the visible zone, free cheating off-camera. Acceptance
  sketch in `SUMOSHARP-HIGH-DENSITY-FEATURES.md` §5. Depends on P1-F (done). Owner-driven.

### Deferred sub-items (documented where they live; pick up if a scenario needs them)
- **P1-E**: teleport-classification split is jam-only here — N/A (that's P1-F); the `getRerouteOrigin`
  brake-gap bump (§8 risk 5) not ported (matches existing obstacle-reroute); pre-insertion is a single
  reroute at insertion, not SUMO's full `pre-period` horizon (fine for our configs).
- **P1-F**: yield/wrongLane teleport classification deferred (jam-only; `total==jam`); full off-road/
  parked vehicle state (lateral shift, not-blocking-a-follower) deferred — matters only for a
  *follower past a parked/teleporting car* and the `y` coord (not compared).
- **P0-C2**: off-road/lateral parked state deferred (same reason).

## 5. Orientation quick-reference
- **Plan + verified gaps + owner steers:** `docs/HIGH-DENSITY-PLAN.md` (has the per-item tracker).
- **Designs:** `docs/HIGH-DENSITY-P0-DESIGN.md`, `-P1E-DESIGN.md`, `-P1F-DESIGN.md`.
- **Spec:** `docs/SUMOSHARP-HIGH-DENSITY-FEATURES.md`.
- **Repo rules / architecture:** `CLAUDE.md`, `docs/DESIGN.md`.
- **HD tests:** `tests/Sim.ParityTests/RungHD*.cs` (16 files). **HD scenarios:** `scenarios/4[1-8]-*`
  (the ones with suffixes `-multifile-cfg`/`-symbolic-depart`/`-vtypedist`/`-summary-output`/
  `-reroute-congestion`/`-reroute-multilane`/`-teleport-jam`/`-parking-depart`).
- **Existing multi-lane-gap analyses:** `docs/NEED-multilane-*.md`, `docs/NEED-*junction*.md`.

## 6. Recommended next order (continuation)
1. **P2-G** empirical dense check + fix (highest value for real multi-lane density; scope with owner).
2. **P2-H** max-depart-delay (small).
3. **X1** attention-aware popping (strategic extra; design-first, functional tests).
