# COORDINATION — pedestrian/crowd work × SumoData sub-area mechanism

Companion to `COORDINATION-pedestrian-x-highdensity.md` (that note is the **code-seam** protocol with the
lane-engine session; this note is the **product/data compatibility** protocol with the **SumoData
sub-area** session). It is the pedestrian-side response to `SUBAREA-FOR-PEDESTRIAN-SESSION.md` (their §3
compatibility requirements) and records how `SUMOSHARP-SERVE-PATH-DROP-IN.md` touches us (barely).

Written 2026-07-18 from the pedestrian side, against SumoData docs at `main` @ `bcdd766`.

---

## 0. TL;DR

- The sub-area system crops a box out of a big city net and fills it with **believable vehicle traffic**
  under one hard rule: **no visible cheating** — an entity may appear/disappear only at the box **fringe**
  (roads/sidewalks cut by the crop), at an **off-road internal sink** (parkingArea; for us: building
  entrance / transit stop / parking board-alight), or **off-camera** (gated by the X1 RealismMask).
- Our separate-engine architecture already fits: we read the **same `net.xml`** geometry via
  `PedNetworkParser`, coupling is additive, and we already treat the camera as a sim-LOD interest source.
- **The one real gap their brief surfaces:** *sim-LOD is not appearance-legitimacy.* Promotion/demotion
  governs **compute/network detail**; it says nothing about whether a ped may legitimately *materialize*
  at a given place. No-cheating needs a **separate appearance-legitimacy layer** that gates ped
  spawn/despawn on fringe/sink/off-camera — reading the **same** camera visible-edge set the vehicle side
  gates on (`Engine.SetVisibleEdges` / `RealismMask`). This is now planned as **Stage P8** (see
  `PEDESTRIAN-TASKS.md`) and hardens liveliness §11.
- **Serve-path drop-in (`SUMOSHARP-SERVE-PATH-DROP-IN.md`) is NOT our work** — it is the SumoSharp
  lane-engine serve-path session's (CLI shim, tripinfo/`arrivalLane`, multi-occupant `parkingArea`). Our
  only touchpoint is §4 below.

---

## 1. Their 7 requirements → our design, our gaps, where it lands

Requirement numbers are from `SUBAREA-FOR-PEDESTRIAN-SESSION.md` §3.

| # | Requirement | Our status | Lands in |
|---|---|---|---|
| **1** | Consume the **cropped box `net.xml`** in its coordinate frame; verify the bake against a real crop (fringe = dangling sidewalk/crossing/walkingArea stubs) | Architecturally fine — the SUMO-geometry bake (`SumoNavMesh`, P2-1) reads the right elements and is frame-agnostic (SUMO XY metres). **Not yet run against an actual crop.** | **P8-1** (verification task, low effort) |
| **2** | The **fringe is the pedestrian no-cheating boundary** — spawn/despawn only at fringe, legitimate internal sinks, or off-camera | **The biggest gap.** sim-LOD ≠ appearance legitimacy. Building-entrance sinks already exist as a *concept* (liveliness §6, POI schema §8) but there is no **spawn policy** enforcing legitimacy. | **P8-2** (appearance-legitimacy layer — the core new piece) |
| **3** | **Auto-deduced pedestrian demand** from walkable-space + land-use (sidewalk density, POIs, transit/parking, plazas), analogous to their topology-based vehicle deduction | `PedDemand` (P2-3) exists but demand is hand-seeded / uniform. Their `deduce_weights.py` is offered as a template. Our POI net-data schema (liveliness §8) is the land-use input. | **P8-3** (extends `PedDemand`; consumes P8 POI data) |
| **4** | **Share the RealismMask / camera visibility signal** — one frustum → one visible-edge set → both engines gate on it (appearance legitimacy, not just LOD) | We already feed the camera into `InterestField` as an LOD interest source. We do **not** yet gate *appearance* on the visible-edge set. `RealismMask.MayPop(edgeId)` is exactly the gate; we need the **walkable-edge analogue** fed from the same host signal. | **P8-2** (unifies the camera source: LOD interest **and** appearance gate) |
| **5** | **Density is calibrated & dialable** — expose a ped-density knob + safe range; crowds must not deadlock crossings so hard the (calibrated) cars gridlock | ORCA has an intrinsic believable density; no exposed knob, no crossing-deadlock guard tied to the vehicle knee. | **P8-4** (ped-density knob + crossing throughput guard) |
| **6** | **Slot into the produced `scenario.sumocfg` + `manifest.json`**; ideally one replay renders cars **and** peds from the same trajectory stream | Ped demand is currently its own inputs. Sim.Viz already renders ped discs from an FCD-style stream (P7-0), so shared replay is close. | **P8-5** (manifest/scenario slot-in + shared-replay contract) |
| **7** | **Watch item:** if Stage **P4** adds engine vehicle-yields-at-crossing, it shifts their **calibration knee** for boxes with such crossings — ping them to re-calibrate | Already flagged in `COORDINATION-pedestrian-x-highdensity.md` §5 as the one future Core touchpoint. | Coordination action on P4 landing (below) |

**Reframe of the illusion boundary.** Their brief makes precise a split our design had only implied:
- **sim-LOD** (`PedLodManager` + `InterestField`) = *how much compute/wire detail* a ped gets. Purely a
  performance/fidelity axis. A low-power ped is still fully present and visible.
- **appearance-legitimacy** (new, P8-2) = *whether a ped may come into / go out of existence here-and-now*.
  A visual-honesty axis. Orthogonal to LOD.
Keeping these two axes separate is the key correctness idea for sub-area compatibility.

---

## 2. The appearance-legitimacy layer (P8-2), in one screen

The vehicle side already has the machinery; we mirror it for walkable space.

- **Same input signal.** The host publishes one camera visible set. Today it calls
  `Engine.SetVisibleEdges(visibleLaneEdgeIds)`. For peds it also yields the **visible walkable-edge set**
  (sidewalks / `crossing` / `walkingArea` ids in view) — same frustum, mapped to walkable geometry. The
  camera stays a sim-LOD interest source (unchanged); it *additionally* drives the legitimacy gate.
- **Legitimacy predicate for a would-be spawn/despawn of ped `p` on walkable edge `e`:**
  `MaySpawnOrDespawn(p, e) = isFringe(e) OR hostsLegitimateSink(e, p) OR isOffCamera(e)`
  where `isOffCamera(e)` = `e ∉ visibleWalkableSet` (the direct analogue of `RealismMask.MayPop`), and
  `hostsLegitimateSink` = the edge carries a building entrance / transit stop / parking board-alight POI
  the ped is actually using (liveliness §6/§8).
- **Where it plugs in.** `PedDemand` spawn and `PedLodManager` despawn/end-of-route ask the policy first.
  A denied on-camera despawn does **not** vanish the ped: it either (a) routes the ped to the nearest
  legitimate sink/fringe, or (b) holds it (low-power) until it walks off-camera. A denied on-camera spawn
  defers to the next fringe/sink opportunity. Determinism preserved: the policy is a pure function of
  (seed, ped, edge, visible set), and the visible set is captured once per host tick (same discipline as
  the engine's mask snapshot).
- **Inert by default.** Empty visible set → every edge off-camera → fully permissive → all existing
  pedestrian scenarios/tests/goldens unchanged (mirrors the engine's null-mask default). This keeps P8
  additive and parity-neutral, per the standing invariants.

---

## 3. Shared data contract (what we consume / produce)

- **Consume:** the cropped `net.xml` (their pipeline output), the `manifest.json` (box bounds, fringe edge
  list, coordinate frame, calibrated density level), the POI/land-use net-data (their road-net data, in or
  alongside the liveliness §8 schema), and the per-tick camera visible set (host-provided at runtime).
- **Produce:** pedestrian demand as **additional** route/person input referenced by (or alongside)
  `scenario.sumocfg`; an FCD-style ped trajectory stream that the shared replay can render next to
  vehicles; a documented ped-density knob + safe range in the manifest.
- **We do NOT ask them to change** their cropping, weight-deduction, or calibration. All requirements are
  satisfied on our side by consuming their outputs and honouring the fringe/mask contract.

---

## 4. The serve-path drop-in (`SUMOSHARP-SERVE-PATH-DROP-IN.md`) — not ours, one touchpoint

That doc's three gaps (GAP-1 `sumo`-shaped CLI shim, GAP-2 `--tripinfo-output` with `arrivalLane`, GAP-3
multi-occupant `parkingArea`) are all **lane-engine / `Sim.Core` serve-path** work, to be done in a
**separate SumoSharp-engine session** (the user's stated plan). None of it is on the pedestrian critical
path, and none of it touches `Sim.Pedestrians`.

**The one conceptual adjacency — GAP-3 multi-occupant `parkingArea`.** Their multi-occupant parking (many
vehicles resident, each in a distinct off-lane slot, followers pass parked cars) is the *lane-engine* view
of a parking lot. Our `LotCoupling` / `MixedTrafficCrowd` parking demo is the *maneuvering/crowd* view
(non-holonomic car threading a lot, peds board/alight). These are complementary, not conflicting: their
GAP-3 concerns SUMO-schema `parkingArea` presence/timing parity on a travel lane; ours concerns free-space
maneuvering and car↔ped avoidance. **Coordination ask when that session starts:** agree on the shared
parked-car representation (off-lane lateral pose + slot identity) so a car that parks via their
`parkingArea` semantics and one that parks via our lot maneuvering render and couple identically. No action
needed until that session exists.

---

## 5. Actions

- **Pedestrian side (this session):** design landed (this note + liveliness §11 rewrite + Stage P8 tasks).
  Implementation of P8 is **sequenced after** the current unblocked batch unless the user reprioritizes;
  P8-1 (crop verification) is cheap and can run as soon as a real cropped box `net.xml` is available.
- **When P4 lands (vehicle-yields-at-crossing in the lane engine):** ping the SumoData session to
  re-calibrate any box whose selection contains signalized crossings (their req 7). Tracked as the P4
  coordination gate.
- **When the SumoSharp serve-path session starts:** agree the parked-car representation (§4) before either
  side hardens its parkingArea rendering.
