# DISTRIBUTED-COUPLING-FEASIBILITY.md — running vehicles and pedestrians on separate machines

**Status: feasibility study (no commitment to build).** This documents *whether* and *how* the simulation
could be split across two (or more) machines that still interact — exchanging state over DDS — and, just as
importantly, **what to avoid in current work so the option stays open**. It is a "return to it later"
artifact: we build this only if a single machine cannot meet the deployment envelope. We are already close
to that edge on pedestrians (see §2), so keeping distribution *possible* is cheap insurance worth taking now
even if we never pull the trigger.

Two split strategies are analysed head-to-head: **(A) by entity type** (vehicles on one machine, peds on the
other) and **(B) by spatial topology** (each machine owns a geographic region containing both types).

---

## 1. Scope: interactive coupling, not view-only replication

Two very different things get called "distributed", and only the second is hard:

- **View-only replication (already shipped).** One machine simulates; others *render* the poses it
  broadcasts over DDS (the IG pattern — `IReplicationSink/Source`, `IPedReplicationSink/Source`, the
  `Sim.Replication.Dds` binding). One-directional; the remote is a spectator and never affects the sim.
  Cross-machine in this sense is done and proven (server == IG over live CycloneDDS).
- **Interactive coupling (this study).** Agents on *different* machines that **affect each other's
  simulation**: a car stops for a ped, a ped waits for a car, both mutually avoid in shared space. This is a
  bidirectional simulation-coupling problem, not a render feed.

Everything below is about the second.

## 2. Why keep this option open — the single-machine edge

The on-target combined-load campaign (`PEDESTRIAN-P6-1-RESULTS.md`, `PEDESTRIAN-COMBINED-LOAD-RESULTS.md`)
found, for the single-station envelope (~50 % of a 24-core box = ~12 cores for the sim, both engines
concurrent):

- Vehicles (`--region`) clear real-time by **30–60×** and pay only a small (~9 %) shared-bus contention tax.
- **Pedestrian *churn* is the sole real-time-marginal quantity** — ~7.0 / 11.7 / 15.3 steps/s at 4 / 6 / 8
  cores (real-time = 10). It clears only at **≥6 ped cores**, with a thin margin (~+11 % at 6 cores), and
  *fails* if peds are starved to 4 cores.
- Ped throughput is **core-scaling-limited, not contention-limited** → the primary fix is **P6-2** (port the
  vehicle engine's byte-identical `--region` domain decomposition to `OrcaCrowd.Step`) to raise ped per-core
  throughput.

So the ladder of levers, cheapest first, is: **(1) P6-2 on one machine** (raise ped per-core throughput) →
**(2) more cores / a bigger box** → **(3) distribute across machines** (this study). Distribution is the
last resort, justified only if P6-2 + hardware still can't hold the envelope (e.g. a much larger crowd, or a
box whose memory bandwidth is the wall regardless of cores). We are close enough to the edge that (3) is a
realistic future, so §8's guardrails matter now.

## 3. The foundation that already exists (what distribution would reuse)

Distribution is unusually tractable here because the vehicle↔ped coupling is **already** built as a
network-shaped exchange:

- **The cross-regime bridge** (`src/Sim.Core/Bridge/CrossRegimeCoupling.cs`, `WorldDisc.cs`). Every mover is
  expressed to the *other* regime as a **moving world-space disc** (a car → a short chain of discs along its
  footprint spine, ≤6; a ped already *is* a disc). The bridge exchanges **frozen world-space snapshots each
  step** and is explicitly a **symmetric one-step-latency coupling**:
  - *Direction A (peds avoid cars):* vehicle discs → `OrcaCrowd.SetExternalObstacles` — a ped's ORCA yields
    for a car.
  - *Direction B (cars avoid peds):* the engine's `CrowdSource` projects nearby crowd agents onto the
    laneless-RVO vehicle's lane and forbids their lateral band (`Engine.ComputeRvoLateral`) — a car swerves
    for a ped.
  - Both sides **fully yield** (a conservative, collision-safe double-yield).
  - **`SubSteps` already dead-reckons each vehicle disc along its world velocity** between engine steps so
    the crowd sees it *sweep* rather than teleport — i.e. the DR-extrapolation you'd use to hide latency is
    **already in the bridge**.
- **The consumers of that seam are proven bilateral:** `LotCoupling` (parking: car discs → peds *and* ped
  discs → cars, both at start-of-step state), `EvacDirector.FeedVehicleDiscsToPeds`, `MixedTrafficCrowd`.
- **The wire already carries what the discs need:** both pose streams carry **position and velocity**
  (`VehicleRecord`, `PedFreeKinematicRecord`) over DDS, so a remote machine has exactly the inputs to build
  velocity-aware ghost discs. `DdsTlState` carries TL state; the **P4-1 seam** (`LiveCrossingSignal` /
  `CrossingSignalFactory.ForCrossingLive`) already lets peds read a live crossing signal through a delegate
  (no Engine reference).
- **Spatial partitioning already exists for vehicles:** the `--region` domain decomposition (`Engine.cs`,
  `docs/SPATIAL-OPT.md`) — a grid of regions each owning a disjoint set of lanes, lock-free, dynamic
  work-stealing, **free boundary hand-off** (a mover crossing a region boundary is regrouped by its current
  lane next step). This is the template for Option B *and* for P6-2.

**The essential reframing:** distributing the coupling is **not inventing a mechanism** — it is *replacing
the in-process frozen-snapshot disc exchange with a DDS exchange of the remote machine's discs*, DR-advanced
one step to cover the network hop. The bridge's own one-step-latency + sub-step DR is the same discipline the
wire needs.

## 4. Determinism and parity framing (which gates stay exact)

- A distributed coupling is **one-step-stale plus network latency** in a feedback loop, so it **cannot be
  bit-identical** to a single-machine golden. It lives in the **real-time / statistical-parity regime**, not
  the exact phase-1 gate.
- **This costs no parity we have today.** The cross-regime coupling is *already* gated and parity-inert:
  with no coupling constructed the crowd is standalone and both regimes are byte-identical to their
  un-bridged behaviour (`CrossRegimeCoupling` doc comment: "the determinism hash is unaffected; nothing here
  is reachable from a committed golden"). Mixed vehicle+ped scenarios (laneless/plaza/parking) are already
  the staggered, statistical regime — not the exact lane-arc gate.
- **The pure gates stay exact.** Pure-vehicle lane-arc parity (hash `909605E965BFFE59`) and the pure-ped
  hermetic tests each run *locally* on one machine, untouched by distribution. Distribution only ever
  perturbs the *cross-regime* interaction, which is already statistical.

So the honest claim is: **distribution is a real-time-deployment capability, and it forfeits nothing the
exact-parity gate currently guarantees.**

---

## 5. Option A — split by entity type (vehicles on machine V, peds on machine P)

**Mechanism.** Machine V runs the lane/laneless vehicle engine and is **authoritative for all vehicles**;
machine P runs the pedestrian subsystem and is **authoritative for all peds**. Each publishes its own
agents' pose+velocity over DDS. Each **ingests the other's stream as read-only `WorldDisc` ghosts**,
DR-extrapolated one step, and feeds them into its local solve exactly where the in-process bridge does today:
- V feeds ped ghosts into `Engine.ComputeRvoLateral` / the car-stops-for-ped path (Direction B).
- P feeds vehicle ghosts into `OrcaCrowd.SetExternalObstacles` (Direction A).
Plus: P reads V's `DdsTlState` for crossing gates (P4-1 pattern, sourced from the wire).

**What's reused:** the entire `WorldDisc`/`SetExternalObstacles` bridge, both DDS pose streams, the P4-1 TL
seam, and the bridge's existing one-step-latency + sub-step DR discipline.

**What's new:**
1. A **ghost-ingest adapter** on each machine that turns the *remote* DDS pose stream into DR-extrapolated
   `WorldDisc`s fed to the local seam (today those discs come from the local other-regime).
2. **Mode-switch ownership hand-off** — the genuinely hard part. When a ped *boards a car* (or alights),
   authority for that agent must transfer from machine P to machine V (and back). This is a distributed,
   cross-*engine* hand-off (harder than the intra-machine `--region` hand-off, which stays within one
   engine).
3. A **DR/latency model** for ghosts (extrapolate + optionally cap-correct on the next update).

**Coupling load: highest of the two options.** Vehicles and peds interact *precisely where they share space*
(crossings, plazas, parking), so with all vehicles on V and all peds on P, **every** vehicle↔ped interaction
is cross-machine. This split maximally stresses the wire.

**Why it is still feasible on a LAN.** DDS on a LAN is **sub-millisecond to a few ms**; a ped step is
**100 ms**. So *one step of staleness dominates*, and the bridge already DR-covers a step. Human/driver
reaction timescales (~200–500 ms) are looser still, so the interaction stays *believable*. Bandwidth is a
non-issue: the single-stream ped figure is ~37 Mbit/s typical (`PEDESTRIAN-COMBINED-LOAD-RESULTS.md` /
POC-7b), and vehicles add ~10–20 Mbit/s — both directions fit a 1 Gbit link with margin.

**Pros:** clean operational story (one box tuned for vehicles, one for peds); each engine keeps its own
threading/tuning; matches how the two subsystems are already code-separated. **Cons:** heaviest coupling
(every interaction crosses the wire); mode-switch hand-off is intricate; dense shared-space mutual avoidance
is a stale feedback loop that can oscillate; does not *reduce* either engine's per-core load beyond removing
the other engine from the box (it's about *fitting two engines on two boxes*, not making either faster).

## 6. Option B — split by spatial topology (each machine owns a region, both types)

**Mechanism.** Partition the world into geographic regions (the `--region` grid, generalised across
machines). Each machine simulates **all agents of both types within its region** and is authoritative for
them. Interaction is **local within a region** (same machine, in-process bridge, unchanged). Only at
**region boundaries** do machines exchange: (a) **ghost discs** for agents near the seam (so an agent about
to cross sees the neighbour region's movers), and (b) **ownership hand-off** when an agent crosses the seam
(regrouped by its new region next step — exactly the `--region` free-hand-off, extended cross-machine).

**What's reused:** the `--region` domain decomposition (ownership, work-stealing, boundary hand-off) as the
conceptual and code template; the same `WorldDisc` ghost mechanism, but only for a thin boundary band.

**What's new:**
1. Cross-machine **region ownership + boundary hand-off** for *both* engines (vehicles already hand off
   between regions in-process; making it cross-machine + doing the same for peds is the work).
2. A **boundary ghost band** exchange (only agents within interaction range of a seam need to be published to
   the neighbour).
3. **Dynamic load balancing** across machines as congestion moves (the `--region` work-stealing is
   intra-process; cross-machine rebalancing is a bigger problem — likely static region assignment first).

**Coupling load: low.** The vast majority of interactions are intra-region (in-process, unchanged); only the
thin boundary band crosses the wire. This is the *scalable* split — it extends to N machines, and each
machine's load is bounded by its region, not the global population.

**Pros:** minimal cross-machine coupling; scales past two machines; both types stay co-located where they
interact, so the hard mode-switch hand-off stays **in-process** (a ped boards a car in the same region on the
same machine) — only *geographic* crossings hand off, which is the already-solved `--region` pattern.
**Cons:** more invasive to build (both engines need cross-machine region ownership); boundary correctness is
fiddly (an interaction straddling a seam is split across machines); load balancing as hotspots move is a real
distributed-systems problem; the world must be cleanly partitionable (fine for a city grid, harder for a
dense single plaza that is itself the hotspot).

## 7. Head-to-head

| Dimension | A — by entity type | B — by spatial topology |
|---|---|---|
| Cross-machine coupling volume | **High** (every veh↔ped interaction) | **Low** (boundary band only) |
| Reuses existing seam | `WorldDisc` bridge + DDS streams (as-is) | `--region` hand-off + `WorldDisc` (boundary) |
| Mode-switch (ped↔car) hand-off | **Cross-machine, cross-engine** (hard) | **In-process** (same region) — easy |
| Scales beyond 2 machines | No (fixed 2-way) | **Yes** (N regions) |
| Load balancing | N/A (fixed split) | Needed as hotspots move (hard) |
| Build effort | Lower (ghost-ingest + hand-off) | Higher (cross-machine region ownership for both engines) |
| Best when | Two subsystems, one box each, LAN | Large map, many machines, geographic spread |
| Worst case | A single dense shared plaza (all coupling crosses) | A single dense plaza that *is* one region (no win) |

**Reading:** Option A is the **smaller build** and matches "vehicles here, peds there" literally, at the cost
of the heaviest coupling and the trickiest (cross-machine) mode-switch. Option B is the **scalable**
architecture and keeps the hard hand-off in-process, at the cost of a larger build and a load-balancing
problem. If distribution is ever needed for *raw scale*, B is the right long-term answer; if it is needed to
*fit two already-built engines onto two boxes on a LAN*, A is the faster path.

## 8. Distribution-preserving guardrails (do these now, even if we never distribute)

The cheap insurance the user asked for: keep current work from foreclosing distribution. None of these cost
anything today; all are already true or nearly so — the point is to **not regress** them.

1. **Route ALL cross-regime interaction through the `WorldDisc` / `SetExternalObstacles` / `CrowdSource`
   seam.** Never let one regime reach directly into the other engine's internal state. (Today this holds —
   `CrossRegimeCoupling` is the only bridge, via frozen snapshots.) A direct cross-regime memory access would
   be un-distributable.
2. **Keep the coupling tolerant of one-step-stale inputs.** Never add an interaction that requires the
   *other* regime's *same-step* (post-solve) state. The bridge's start-of-step frozen-snapshot discipline is
   the invariant; preserve it. (This is also what makes the in-process coupling parallel-safe.)
3. **Single-owner authority per agent.** Each agent is authoritative on exactly one engine; the other side
   only ever consumes it as a read-only ghost. Don't add logic that mutates an agent from the "foreign" side.
4. **Carry velocity (and enough to DR) on every pose wire record.** Already true (`VehicleRecord`,
   `PedFreeKinematicRecord`); keep it. Ghost extrapolation depends on it.
5. **Keep TL / crossing-signal reads behind the P4-1 delegate seam**, never a hard Engine reference from the
   ped side. Already true (`Sim.Pedestrians` does not reference the Engine). This lets the signal come from a
   DDS topic instead of a local engine with no ped-side change.
6. **No global per-step barrier that assumes all agents are local.** Both engines already use a plan/execute
   command-buffer discipline per region/crowd; avoid introducing a step that needs a global all-agents view.
7. **Keep agent handles stable and portable** (index+generation, no pointers/strings on the hot path).
   Already the design. A hand-off across machines needs a handle that means the same thing on both.
8. **Keep the mode-switch (board/alight/park) expressed as an explicit lifecycle event**, not an implicit
   in-place mutation. Already true (it rides the lifecycle topic). A distributed hand-off *is* a lifecycle
   event with an ownership transfer attached.

Maintaining these is exactly the discipline the in-process `--region` and LOD-parallel work already enforce,
so "keep distribution possible" and "keep the single-machine engine clean and parallel-safe" are the **same
constraints**. That is the real conclusion of this study: **we do not need to build distribution now, and if
we keep the current seams honest, we lose nothing by deferring it.**

## 9. Rough effort sketch (if triggered)

- **Option A:** ghost-ingest adapters both directions (~small, reuses the bridge), a DR/latency model
  (~small, `SubSteps` DR is the seed), the mode-switch cross-machine hand-off protocol (**the bulk**), and a
  soak/verification harness (two processes on a LAN → visual + collision-free + believable-interaction
  checks). Weeks, not months, for a first believable prototype; the hand-off correctness is the long pole.
- **Option B:** cross-machine region ownership for both engines (**the bulk** — generalising `--region`
  across a network boundary and adding the same to peds), boundary ghost-band exchange, static region
  assignment first (defer dynamic balancing). Larger; a first two-region prototype is a meaningful project.

## 10. What a prototype must measure (open questions)

- Does dense shared-space mutual avoidance stay stable (no oscillation / grazing) at one-step + LAN latency,
  or does it need cap-corrected DR / a wider safety band?
- Real interaction quality at crossings under latency (does car-stops-for-ped trigger early enough with a
  DR'd ghost + margin?).
- Mode-switch hand-off correctness (no dropped/duplicated agent as it crosses machines).
- Actual LAN latency distribution + jitter under load, vs the 100 ms step budget.
- Bandwidth of the boundary band (Option B) vs the full cross-stream (Option A) at target scale.

## 11. Recommendation

**Do not build distribution now.** Exhaust the cheaper levers first: **P6-2** (ped `--region` decomposition)
then **hardware**. Adopt the **§8 guardrails** immediately as standing design rules (they are free and
coincide with keeping the engine parallel-safe). If distribution is later triggered, prefer **Option A** to
fit two engines on two LAN boxes quickly, and **Option B** if the driver is raw scale across many machines.
Revisit this doc with the P6-2 outcome and the final combined-load numbers in hand.
