# Sub-area traffic experiment — findings, recommendation & port gap map

**Status: pipeline proven end-to-end in pure SUMO (synthetic net).** Exploratory session.

Goal (A): a pipeline for "user selects a ~3×3 km sub-area of a bigger network → believable SUMO
traffic in that box, fast (ideally instant, ≤10 min worst case)", with the **hard rule: no visible
cheating** — cars appear/disappear only at the network **FRINGE** (roads cut by the boundary) or at
internal **SINKS** (parkingArea). No popping on visible roads.

Goal (B): map what our C# port (**SumoSharp**) is missing to drive this workflow (recorded for later;
we are **not** implementing port features this session).

All SUMO usage is **authoring/investigation only** (CLAUDE.md allows it); nothing here is a
`dotnet test` dependency. Reproduce with `experiments/subarea/run-experiment.sh` +
`experiments/subarea/auto_parking.py`. Generated nets/routes live in `experiments/subarea/scratch/`
(gitignored — only the scripts and this doc are committed).

---

## TL;DR — the recommendation

**Use L2 "macro-crop" + automated parking sinks. There is essentially no simplicity/speed-vs-
believability tradeoff to make**, because the cost splits cleanly:

- **Once per macro map (slow, reused):** obtain/generate believable demand on the big network and
  run it once with per-edge exit times. ~19 s on the synthetic 30×30 macro (3852 vehicles).
- **Per sub-area selection (instant):** crop the net + cut+re-time the routes + auto-generate the
  parking layer = **~3.3 s total** on the synthetic net (crop 1.1 s, cutRoutes 2.0 s, parking 0.2 s).
  Comfortably "instant", and orders of magnitude under the 10-min worst case even once a real Geneva
  net is 10× larger.

The believability win (local trips that park/emerge instead of popping) costs **0.2 s** and is
**fully automated** from the cut routes — so you do not trade speed for it. The only genuine upstream
cost is the *quality of the macro demand* (synthetic randomTrips here; the user's tuned Geneva
scenarios or an L3 OD model later), and that is a one-time, per-map concern independent of sub-area
selection.

L1 "weighted random on a standalone grid" is a **dead end** for this goal (see below): a standalone
generated grid has no fringe at all, so it cannot honor the fringe-only rule. The fringe is *created
by cropping*, which is L2 — so even the "simple" path is L2. L3 (procedural OD) is not needed to hit
the believability bar on this evidence.

---

## Environment (this VM)

| Tool | Provisioning (ephemeral, VM-volatile) |
|---|---|
| SUMO 1.20.0 — `sumo`, `netgenerate`, `netconvert`, `duarouter`, `od2trips` | `pip install eclipse-sumo==1.20.0` → `$SUMO_HOME/bin` |
| `randomTrips.py`, `route/cutRoutes.py`, `sumolib` | `$SUMO_HOME/tools/…` (set `PYTHONPATH`) |
| .NET 8 SDK (to run the port) | `apt-get install -y dotnet-sdk-8.0` (8.0.129) |

Neither is committed or required by the offline test loop.

---

## The approach ladder — what held up

### L1 "weighted random" — cannot satisfy no-cheating on a raw grid

`netgenerate --grid` is a **closed** network: every node has matched in/out degree (corners 2/2,
edges 3/3, interior 4/4), so it has **zero fringe edges** (`sumolib is_fringe() == 0/360`).
`randomTrips --fringe-factor` is therefore a no-op and **100 %** of trips depart *and* arrive on
internal roads → pure popping (audited: 2185/2185 off-fringe both ends). **A fringe does not exist
until you cut a box out of a bigger net** — which is L2.

### L2 "macro-crop" — the workable pipeline ✅

Exact commands (full driver in `run-experiment.sh`):

```bash
# macro map (done once); the standalone box is only used to demonstrate L1's failure
netgenerate --grid --grid.number=30 --grid.length=300 -o synth_macro.net.xml     # 8700x8700 m

# (1) CROP the box — THIS creates the fringe (cut edges become dangling entry/exit stubs)
netconvert -s synth_macro.net.xml --keep-edges.in-boundary 2850,2850,5850,5850 -o sub.net.xml
#     -> 440 edges, 80 is_fringe()==True   (was 0 before the cut)

# (2) demand on the FULL macro, routed, dumped WITH per-edge exit times (cutRoutes needs them)
python3 randomTrips.py -n synth_macro.net.xml -r macro.rou.xml --period 0.8 --end 3600 --validate
sumo -c macro.sumocfg --vehroute-output macro.vehroutes.xml --vehroute-output.exit-times

# (3) CUT demand into the box; departures re-timed to when each car reaches the boundary
python3 route/cutRoutes.py sub.net.xml macro.vehroutes.xml \
        --routes-output sub.rou.xml --orig-net synth_macro.net.xml
#     -> 3852 macro vehicles -> 1608 kept (those crossing the box)
```

**Demand decomposition (the key structural insight), from the cropped box:**

| | count | share | handled by |
|---|---|---|---|
| through-traffic (fringe→fringe) | 1195 | ~73 % | fringe, for free — clean already |
| internal ORIGIN (pops in) | 433 | ~27 % | **parking sink** (pull-out) |
| internal DESTINATION (pops out) | 413 | ~26 % | **parking sink** (pull-in + stay) |
| internal on BOTH ends | 48 | ~3 % | parking on both ends |

Through-traffic inherits realistic flow phasing for free (re-timed to boundary arrival). The internal
O/D share is **not a bug** — it is the demand that *must* be absorbed by internal sinks, and the audit
quantifies how much (~1/4 of demand here).

### L3 "procedural OD" — not needed

`od2trips` is present if we escalate, but L2 already meets the believability bar. Deferred.

---

## Parking sinks — the no-cheating half, fully automated ✅

`experiments/subarea/auto_parking.py` turns the cut routes into a **no-popping** scenario with zero
manual work (input: cropped net + cut routes; output: a parkingArea additional-file + rewritten
routes). Rules it applies:

- **Internal destination** → append `<stop parkingArea=… duration=100000/>` (outlasts the sim): the
  car pulls off the running lane into a lot and **stays** — a believable sink, never a mid-road vanish.
- **Internal origin** → `departPos="stop"` + a short leading parkingArea stop: the car is inserted
  **already parked** (off-road) and pulls out into traffic — a believable source.
- One `<parkingArea>` per internal-endpoint edge (lane `_0`), `roadsideCapacity` sized to that edge's
  demand. (A `<rerouter>` overflow-to-adjacent variant is a later refinement.)

Run: 331 parkingAreas, 798 vehicles rerouted to parking, **zero SUMO warnings**.

**No-cheating audit of the parking-enabled run:**

| check | result |
|---|---|
| through-trips that arrived on a running lane | 1195, **0 off-fringe** ✅ |
| non-parking vehicles first appearing on an internal lane | **0** ✅ (every internal appearance is a car in a parking area, off-road) |
| all 1608 vehicles accounted for | ✅ (433 pull-out origins, 413 park-and-stay destinations) |

→ **Every appearance/disappearance on a visible road is at the fringe. All internal births/deaths
happen off-road inside a parkingArea.** The hard rule is met.

**Believability refinements noted (not blocking):** dest cars currently park forever (lots fill and
stay — fine as a sink; add turnover / finite dwell + a `<rerouter>` for overflow for extra realism);
`roadsideCapacity` is sized exactly to demand (no overflow modeled yet).

---

## Timing (why "instant" holds)

| stage | when | cost (synthetic net) |
|---|---|---|
| macro sim + exit-times | once per macro map | 18.7 s |
| crop (`netconvert`) | per box | 1.1 s |
| `cutRoutes` | per box | 2.0 s |
| `auto_parking.py` | per box | 0.2 s |
| **per-box total** | | **~3.3 s** |

Density is tuned independently of the macro via box size and macro `--period` (headway); a denser box
than the full map = smaller window / shorter period, without touching the macro run.

---

## Port (SumoSharp) gap map — recorded for later

Fed the cropped box (through-traffic only, no parking) to `Sim.Run`: presence parity is **exact** —
1608 vehicles, 3600 steps, peak 136 concurrent, identical to SUMO — **once the parser blockers below
are worked around**. So the macro-crop pipeline already runs through the port for through-traffic; the
parking-sink half does not, and is the bulk of the gap.

Priority order (blockers first). Layer = Ingest (parser) vs Core (engine).

| # | Gap | Layer | Severity | Evidence / note |
|---|---|---|---|---|
| **G1** | **Symbolic depart attrs rejected.** `departSpeed="max"`, `departLane="best"`, and (parking) `departPos="stop"` throw `FormatException` in `DemandParser.ParseNullableDouble/Int`. | Ingest | **Blocker** | `cutRoutes` emits `departSpeed="max"`/`departLane="best"` on *every* cut vehicle; `departPos="stop"` is exactly what origin-parking needs. SUMO's symbolic vocab: speed `max/desired/speedLimit/last/avg/random`; lane `best/free/random/allowed/first`; pos `random/free/base/last/stop/splitFront`. Port accepts only numerics. Worked around by `sed`-stripping. |
| **G2** | **No additional-file handling at all.** The sumocfg `<additional-files>` is never read; `<parkingArea>`, `<rerouter>`, detectors are invisible to Ingest (`grep`: zero `additional` handling). | Ingest | **Blocker (parking)** | The entire internal-sink half has no ingestion path. Needs: read `<additional-files>` from the cfg, parse `<parkingArea id lane startPos endPos roadsideCapacity>`. |
| **G3** | **`<stop>` support is lane-only.** Parser reads `lane/startPos/endPos/duration`; `parkingArea=`, `busStop=`, `triggered`, `until`, waypoint (`speed>0`) explicitly out (`DemandParser.cs:78-80`). Engine mirror `StopRuntime` models only lane stops. | Ingest + Core | **Blocker (parking)** | Need `<stop parkingArea=… duration=…>` in both parser and engine. |
| **G3b** | **Parked vehicle must be OFF the running lane.** A parkingArea stop pulls the car off the carriageway (lateral offset); it must not act as a lane leader/obstacle while parked, and must re-merge on pull-out. Today's lane `<stop>` keeps the vehicle *on* the lane. | Core (semantics) | **Blocker (parking)** | This is the behavior that makes parking "not cheating" (off-road birth/death). Distinct from just parsing the stop. |
| **G4** | **No `<rerouter>` element.** Rerouting exists only as an internal Dijkstra/`ReplaceRoute` code knob, not the XML `<rerouter>` (parkingAreaReroute / overflow-to-adjacent). | Ingest + Core | Medium | Needed for auto-park + overflow; can be deferred if parking capacity is pre-sized to demand (as the generator does). |
| **G5** | **`departSpeed="max"` is a believability lever, not just a parse issue.** Fringe cars are through-traffic already moving; entering at `max` vs. the port's default 0 = "flows in at the boundary" vs. "materializes stopped at the boundary". | Core (semantics) | Medium | Fold into G1's fix: resolve symbolic `departSpeed`, at least `max`/`speedLimit`. |
| — | **No native crop/cut/park seam.** bbox → crop net → cut+re-time routes → map internal O/D to parking currently *must* shell out to `netconvert` + `cutRoutes` + `auto_parking.py`. The port has no authoring API for this. | design | — | Open question: is a SumoSharp-native cropping/authoring seam worth building, or does it stay a SUMO-tool preprocessing step feeding the port? |

**Bottom line for the port:** through-traffic sub-areas work today modulo G1. Delivering the *insisted-
on parking-sink believability* requires **G2 + G3 + G3b** (parse additional-files + parkingArea, parse
parkingArea stops, and represent a parked vehicle off-lane), with G1 (`departPos="stop"`) and G4
(`<rerouter>`) alongside. G3b is the load-bearing engine behavior; the rest is parsing.

---

## Reproduce

```bash
python3 -m pip install "eclipse-sumo==1.20.0"     # ephemeral
apt-get install -y dotnet-sdk-8.0                  # ephemeral, to run the port
bash experiments/subarea/run-experiment.sh         # L1 demo + full L2 pipeline + port run
# parking-sink layer (pure SUMO):
source experiments/subarea/env.sh
export PYTHONPATH="$SUMO_HOME/tools"
cd experiments/subarea/scratch
python3 ../auto_parking.py sub.net.xml sub.rou.xml sub_parking.add.xml sub_parking.rou.xml
sumo -c sub_parking.sumocfg --fcd-output sub_parking.fcd.xml --tripinfo-output sub_parking.tripinfo.xml
```

---

## Scaling to a real, country-sized terrain (box-anywhere, automated)

Target: the user selects **any** sub-area of a huge real net (e.g. all of Switzerland, low-fidelity,
with detailed city insets), and the pipeline produces a ready-to-run believable scenario in
**seconds-to-minutes**, fully automated. The naïve reading — "microsimulate the whole country to get
demand, then crop" — is computationally impossible (millions of vehicles). It is also unnecessary.

### The reframing: precompute = attach weights to the net; per-box = synthesize locally

The macro does **not** scale with the terrain, it scales with the box (see "How big must the macro be"
below). What a box needs from outside is two *boundary conditions*: (1) inflow rate + timing at each
fringe edge, and (2) the through-turn distribution. Neither requires a country-scale micro sim.

**Precompute ONCE per country net (cheap, automated):** attach per-edge demand *weights* to the net —
a **source weight** (propensity for trips to start on this edge), a **sink weight** (propensity to
end), and a **through weight** (road capacity/hierarchy). No per-vehicle simulation.

**Per box selection (instant, automated):** crop box + thin halo, then synthesize demand *locally* on
that crop from the weights: `randomTrips --weights-prefix <landuse> --fringe-factor N` gives internal
O/D (weighted by land use) plus fringe through-traffic (weighted by capacity), routed with `duarouter`,
then `auto_parking.py` maps internal O/D to parking sinks. Cost is bounded by the halo area, **not the
terrain size**.

### Demand-fidelity ladder (this is the "do we need demographic data?" answer)

| Rung | Demand source | External data needed | Effort | When |
|---|---|---|---|---|
| **D1** | land-use-weighted `randomTrips` (source=residential, sink=commercial/work, through=capacity) + `--fringe-factor` on the box+halo crop | **none** — land-use comes from OSM polygons already in the map | autogenerated, zero manual | **default** |
| **D2** | D1 + calibration: ASTRA traffic counts set motorway/arterial fringe-inflow volumes; municipal population weights residential | real counts + population (Swiss open data; automatable fetch) | light, mostly major roads | when major-corridor volumes must match reality |
| **D3** | full OD matrix (census / activity model) → `od2trips` / `marouter` | an OD matrix / activity-based model | heavy to build + calibrate | last resort; long-range O/D correlation |

**Verdict:** demographic data is **optional**, not required. D1 needs only land-use, which is derivable
from the OSM tags already in a real map (`polyconvert` extracts residential/commercial/retail/
industrial polygons; assign each polygon's weight to nearby edges). Demographic/count data (D2/D3) buys
long-range O/D *correlation* — worth it on major corridors, overkill for a believable 3 km box, where
land-use gradients + road hierarchy already approximate directionality.

### How big must the macro be, relative to the sub-area?

Not country-sized. Micro-simulate **box + a thin halo** (a warm-up buffer, shown only for the inner
box). The halo exists so fringe inflows arrive as realistic platoons with realistic turn ratios, rather
than cold-injected. Cold-injected traffic relaxes into realistic microstructure within a few hundred
metres, so for a multi-km box the halo is a small fraction. Rule of thumb by the largest road crossing
the box: local/collector ⇒ ~300–800 m halo; urban arterial ⇒ ~0.5–1.5 km; a motorway's long-range
*volume* is injected as a boundary inflow (from counts) rather than captured by growing the halo.
(Empirical convergence number for the synthetic net: see `experiments/subarea/RESULTS-halo.md`.)

Everything beyond the halo is replaced by **boundary inflow rates**, from the precomputed through/
capacity weights (D1) or real counts (D2) — a one-time, per-map concern independent of box selection.

### The automated end-to-end pipeline (real net)

```
ONCE per country net (automated, no micro sim):
  OSM  --netconvert-->  switzerland.net.xml   (low-fi country + hi-res city insets via typemaps)
  OSM  --polyconvert-->  land-use polygons  --assign-->  per-edge src/dst weights (D1)
  [optional D2] fetch ASTRA counts + municipal population -> calibrate through/src weights

PER BOX (seconds, automated):
  1. netconvert --keep-edges.in-boundary <box+halo>            -> sub.net.xml     (~1 s)
  2. randomTrips --weights-prefix <landuse> --fringe-factor N  -> trips           (~1 s)
     duarouter (or randomTrips -r)                             -> sub.rou.xml
  3. auto_parking.py  (map internal O/D -> parkingArea sinks)  -> add + routes    (~0.2 s)
  4. (optional) thin-halo warm-up; deliver/show only the inner box
```

Time-from-selection-to-ready is dominated by step 2 (demand generation), which scales with box+halo
size, not the country. On the synthetic net the crop+cut+parking steps are ~3 s total; a real 3 km box
should stay well inside the 10-min worst case.

### Real-world net acquisition — env note

Real OSM data could not be fetched in this session: the egress policy blocks Overpass, Geofabrik, and
the OSM API (403 / connection refused via the proxy). So the real-net leg (OSM → netconvert → land-use
weights → box slice, with timing) is **deferred to when the user brings the real Geneva/Switzerland
assets** (or a session with OSM egress allowed). The synthetic experiments below validate the two
load-bearing claims (halo convergence; land-use-weighted structure) in the meantime.

### Empirical validation (synthetic)

- **Halo convergence** → `experiments/subarea/RESULTS-halo.md` (how small a halo reproduces full-macro
  inner-box traffic; validates "macro scales with box, not terrain").
- **Land-use-weighted demand** → `experiments/subarea/RESULTS-landuse.md` (autogenerated per-edge
  weights produce home→commercial structure without external demographics; validates D1).
  **Result (verified independently):** with a commercial core = 2.87 % of edges, weighted demand puts
  **22.7 %** of trip destinations in the core vs **2.8 %** under uniform (= the area baseline) — an
  **8.1× concentration**, with commercial origins symmetrically suppressed 3.25 % → 0.44 %. Uniform
  demand tracks the area baseline exactly (no hidden bias). Confirms a home→work commute pattern from
  per-edge weights alone, zero external data. Scripts: `experiments/subarea/landuse/`.

### New port-gap implications

The D1 path leans on seams the port must eventually consume: `randomTrips`/`duarouter` output with
symbolic depart attrs (**G1**), `<flow>`-based fringe inflows (README says `<flow>` is supported — good),
and parking sinks (**G2/G3/G3b**). Land-use weighting itself is an *authoring-side* concern (produces a
plain `.rou.xml`), so it adds **no new engine gap** — the port just runs the resulting routes. The only
potential new seam is if we ever want the port to do the bbox-crop + demand-synthesis natively instead
of shelling out to `netconvert`/`randomTrips` (the "native crop/authoring seam" open question).

---

## Next steps

1. (Optional) `<rerouter>`-based overflow + finite parking dwell for extra realism / turnover.
2. Run the full pipeline on the user's real manually-tuned Geneva net + scenarios, and do the real-net
   D1 leg (OSM land-use → weights → box slice + timing) once real assets / OSM egress are available.
3. When we choose to close the port gaps, start with G2/G3/G3b (parking ingestion + off-lane parked
   semantics) — that is what unlocks the parking-sink believability through SumoSharp.
