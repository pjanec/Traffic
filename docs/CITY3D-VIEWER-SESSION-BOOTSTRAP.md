# Bootstrap — run City3D (3D) + Raylib (2D) viewers locally on Windows, then remote over DDS

**Read this first, then jump to §4 (the run recipes).** This doc exists so a fresh session on a Windows
desktop can get the two viewers **on screen** fast — running the SumoSharp engine with cars (and pedestrians)
— **without** re-discovering how SumoSharp runs or what data exists. Those basics are settled and given here.

Branch to check out: **`claude/spectacle-ig-binding-poc-cf3fm4`** (has all the work below). Repo root is
wherever you clone; every path here is repo-relative. `.NET 8 SDK` + `Godot 4.7.1 (.NET/mono)` required (§4.0).

---

## 1. What I want from this session

1. **See the new motion smoothing on a real GPU** (the one thing a headless VM can't prove). Both viewers now
   reconstruct vehicle motion with the kinematic algorithm we tuned; I want to confirm it *looks* right:
   smooth junction turns, lane changes that ease (not snap), cars sitting centered on the road.
2. **Run local (in-process) City3D** for cars, and for pedestrians, and understand exactly how.
3. **Then run remote City3D over DDS** — a standalone SumoSharp producer process publishing DDS, a separate
   City3D subscriber rendering it. This is the professional/decoupled use case.
4. Cars **and** pedestrians. ⚠️ Important reality (see §3): today City3D renders cars **or** peds in a given
   process, never both in one scene, and the DDS path is **cars-only**. Combining them is a candidate task
   for this session, not a solved feature — don't assume a single cars+peds City3D scene exists.

Focus the session on **running/observing the viewers and (optionally) the cars+peds unification / DDS wiring**
— not on how to drive the engine or find scenarios. That is all below.

---

## 2. What has already been done (context — don't redo it)

The render-side vehicle **smoothing** was reworked and both viewers were migrated onto it. Full story:
`docs/VIEWER-KINEMATIC-SMOOTHING-DESIGN.md` / `-TASKS.md` / `-TRACKER.md`, and the tuning methodology in
`docs/IGBRIDGE-METHODOLOGY.md`. In short:

- **One shared reconstruction facade** `src/Sim.Viewer.Motion/KinematicReconstructor.cs` — no-slip rear-axle
  drag + low-passed lane-heading prediction + spatial look-ahead (aims down the upcoming connecting lane) +
  anticipation lead-bound + coarse-feed junction-straddle handling + lane-change ease. It replaced the old
  `DrPoseSmoother` (**deleted**) on the vehicle path.
- **Both viewers now call it**: Raylib 2D (`src/Sim.Viewer.Raylib/RenderHelpers.cs`, all of loopback / remote
  / `--mode local`) and City3D 3D (`demos/City3D/CityLib/Reconstructor.cs`).
- **Measured wins** (Raylib `--trace-veh`, 30 Hz-resampled): junction max yaw-jerk 3315→450 °/s², violent
  heading snaps (>1000 °/s²) 4→0; a lane change now eases (per-frame lateral 8× smaller).
- **Two latent City3D bugs fixed for free**: it had *no* lane-change smoothing (it skipped straddles); and it
  drew each car ~½·length too far forward — now fed the true vehicle **center** (measured L/2 behind front).
- **Gates that hold** (this is the bar for any further change): `dotnet test tests/Sim.ParityTests` = **654
  pass / 4 skip byte-identical** (the parity "iron law"); `Sim.Core` is never edited for render work; the
  IgBridge emit trace stays byte-identical to `artifacts/igbridge/trace_v5_baseline.jsonl`; determinism (no
  `System.Random`, per-entity state). The prior context (IgBridge, an offline IG-feed PoC) is where the
  algorithm was tuned; you don't need it to run the viewers, but `IGBRIDGE-METHODOLOGY.md` §10 documents the
  2D-visualization recipe and the metric suite if you need to diagnose motion.
- **Left for this session**: the on-screen sign-off (T3.3), and optionally the cars+peds / DDS items in §3/§6.

## 3. How we work (project rules — follow these)

Read `CLAUDE.md` at the repo root; the load-bearing rules:
- **Design-first, no ad-hoc dev.** For any real feature (e.g. combining cars+peds, or a DDS ped path), write a
  short design doc + tasks + tracker and get agreement **before** coding. Small run/config tweaks don't need it.
- **Parity is the iron law.** `dotnet test tests/Sim.ParityTests` must stay 654/4 byte-identical. Render work
  lives in `Sim.Viewer.*` / the demo / `Sim.Viewer.Motion` — **never edit `Sim.Core`** for viewer work.
- **Measure, don't eyeball.** When motion looks wrong, reproduce it, instrument the specific vehicle (Raylib
  `--trace-veh <id>`), localize the stage, then fix — see `IGBRIDGE-METHODOLOGY.md` §5.
- **Committed-vs-ephemeral.** Only committed files survive. `demos/City3D/local-nuget/`, `.godot/`,
  `artifacts/`, the Godot binary are all ephemeral/regenerated — never commit them.
- **Determinism**: no `System.Random` without a fixed seed; per-entity state; results independent of thread order.
- **Ask in plain chat**, not question widgets (per CLAUDE.md interaction prefs).

---

## 4. HOW TO RUN — the recipes (the important part)

### 4.0 Prereqs (Windows)
- **.NET 8 SDK** (`dotnet --version` → 8.x).
- **Godot 4.7.1 — the .NET/mono build**, `mono_win64` flavor. The repo's `fetch-godot.sh` is bash/Linux; on
  Windows download manually: `https://downloads.godotengine.org/?version=4.7.1-stable&flavor=stable&slug=mono_win64.zip`
  → you get `Godot_v4.7.1-stable_mono_win64.exe` **plus a sibling `GodotSharp/` folder** (keep them together).
  Call that `.exe` `GODOT` below.
- **The bash scripts under `demos/City3D/*.sh` won't run in PowerShell** — this doc gives the raw
  `dotnet`/`godot` commands they wrap, which are cross-platform. (git-bash/WSL can run the `.sh` directly if
  you prefer; the Godot binary must still be a Windows or WSL build to match.)
- No Xvfb / `--headless` / `LIBGL_ALWAYS_SOFTWARE` needed — you have a GPU; run windowed.

### 4.1 Scenarios & data — what to use (already curated)
A SumoSharp scenario dir holds SUMO inputs. **Two file-name contracts** matter:
- **City3D strict**: needs exactly `net.net.xml` + `rou.rou.xml` + `config.sumocfg` in the scenario dir.
- **Sim.Host.App / Raylib loose**: any `*.net.xml` + sibling `*.rou.xml` + `*.sumocfg`.

Recommended committed **cars** scenarios (all flat, no elevation):
| path | what | good for |
|---|---|---|
| `scenarios/09-traffic-light` | tiny signalized dev net | fast first light-up; `--camera=close` |
| `scenarios/44-multilane-junction-turn` | small multilane priority junction, turning movements | close-up of the smoothing on turns |
| `scenarios/_bench/city-mixed-1k` | **flagship**: ~1064 junctions, 262 traffic lights, 3 roundabouts, ~1069 peak vehicles | the impressive full city |
| `scenarios/_bench/city-organic-L2` | organic town net ("L2" = 2 *lanes*, not levels) | organic geometry |

**Pedestrians**: `scenarios/_ped/poc0-crossing-plaza` (a 4-arm crossing plaza; `net.net.xml` +
`walkable.add.xml`, no rou/config — peds only). This is the **hardcoded** ped net the viewers use.
⚠️ `scenarios/_ped/subarea-box` has sidewalks/crossings but its net is named `net.xml` (not `net.net.xml`), so
the strict loaders won't take it as-is — it's a tooling fixture, not a drop-in.

Pedestrians are a **synthetic crowd generated at runtime** over the net's SUMO walkable geometry
(sidewalks/crossings/walkingAreas) — the navmesh is baked in-process; there is **no scenario-authored
`<person>` demand and no committed `.navmesh`**. So "peds" = point a viewer at a ped net and it spawns/animates
a crowd itself (`docs/PEDESTRIAN-OVERVIEW.md`; `CityLib/PedSimSource.cs`).

### 4.2 Raylib 2D viewer (simplest — start here to sanity-check the smoothing)
No packaging; just `dotnet run`. Opens a real window on Windows.
```
# cars, in-process DR (loopback = engine + publisher + subscriber in one process, real reconstruction path):
dotnet run --project src/Sim.Viewer -- --mode loopback scenarios/09-traffic-light
dotnet run --project src/Sim.Viewer -- --mode loopback scenarios/_bench/city-mixed-1k --fleet 4000

# diagnose one vehicle's motion (logs AUTHTRACE vs reconstructed DRTRACE per frame):
dotnet run --project src/Sim.Viewer -- --mode loopback scenarios/44-multilane-junction-turn --trace-veh <id>

# pedestrians (in-process synthetic crowd; no --scenario, chosen by demo name):
dotnet run --project src/Sim.Viewer -- --mode local --demo "Pedestrian remote (over the wire)"
```
Key args: `--mode local|loopback|publish|remote|ped-publish`; positional `<scenarioDir>` (loopback/local/
publish); `--delay <s>` (DR playout, default 1.0); `--fleet <n>` sandbox fill; `--trace-veh <id>`. There is
**no `--peds`/`--scenario` flag** in Raylib — peds come via `--demo "<name>"`.

### 4.3 City3D 3D viewer — LOCAL (in-process)
**Build once (PowerShell equivalent of `build.sh --pack-only` + Debug build).** Godot loads the **Debug**
Viewer build.
```
# 0) IMPORTANT: clear any stale SumoSharp.* from the global NuGet cache (package version is a fixed 0.1.0, so
#    a prior build's package would otherwise be reused instead of your freshly packed one):
dotnet nuget locals global-packages --clear    # or delete %USERPROFILE%\.nuget\packages\sumosharp.*

# 1) pack the SumoSharp.* packages into the demo's local feed:
$FEED = "demos\City3D\local-nuget"
Remove-Item -Recurse -Force $FEED -ErrorAction SilentlyContinue; New-Item -ItemType Directory $FEED | Out-Null
foreach ($p in "Sim.Core","Sim.Ingest","Sim.Replication","Sim.Viewer.Motion","Sim.Host","Sim.Pedestrians") {
  dotnet pack "src\$p\$p.csproj" -c Release -o $FEED
}

# 2) restore + build the Godot C# app (Debug, what Godot loads):
dotnet restore demos\City3D\Viewer\Viewer.csproj
dotnet build   demos\City3D\Viewer\Viewer.csproj -c Debug
```
**Run (cars):**
```
GODOT --path demos\City3D\Viewer -- --scenario=09-traffic-light --camera=close
GODOT --path demos\City3D\Viewer -- --scenario=_bench/city-mixed-1k --camera=overview
```
**Run (pedestrians — the crossing plaza):**
```
GODOT --path demos\City3D\Viewer -- --peds
```
Viewer args (all AFTER Godot's `--`): `--scenario=<rel under scenarios/>` (default `09-traffic-light`; needs
the strict `net.net.xml`+`rou.rou.xml`+`config.sumocfg`); `--camera=overview|close`; `--transport=local|dds`
(default local); `--peds` (bare flag → the plaza, **skips all vehicles**); `--shot=<path>` `--shot-delay=<s>`
for a screenshot. Everything is verified on `09-traffic-light` (dev) and `_bench/city-mixed-1k` (headline).

⚠️ **Cars OR peds, not both.** `--peds` short-circuits to the ped plaza and never builds the vehicle city;
without it you get cars and no peds. There is no combined scene today (see §6 for the unification task).

### 4.4 City3D 3D viewer — REMOTE (DDS, two processes)
The **producer** (standalone engine) and the **subscriber** (City3D) are separate processes; DDS discovery is
LAN/loopback UDP multicast (make sure it isn't firewalled).

**Build the subscriber with the DDS source compiled in** (adds `Sim.Replication.Dds` to the pack + a define):
```
# pack must ALSO include the DDS binding:
dotnet pack src\Sim.Replication.Dds\Sim.Replication.Dds.csproj -c Release -o demos\City3D\local-nuget
# build the Viewer with the remote define:
dotnet build demos\City3D\Viewer\Viewer.csproj -c Debug -p:City3DRemote=true
```
**Producer (cars over DDS)** — the generic headless host `Sim.Host.App`:
```
dotnet run --project src/Sim.Host.App -- --scenario scenarios/_bench/city-mixed-1k --transport dds --hz 10
```
Its args: `--scenario <dir>` (required), `--transport dds|inmem` (default dds), `--hz <n>` (10), `--spawn <n>`
(extra ambient trips on top of scenario demand, seed 12345), `--seconds`/`--steps` (run length). Use
`--transport inmem` for a same-process self-test with no native DDS.
**Subscriber (City3D remote):**
```
GODOT --path demos\City3D\Viewer -- --transport=dds --camera=overview
```
⚠️ **DDS is cars-only.** `Sim.Host.App` publishes vehicles + geometry + traffic lights, **no pedestrians**
(confirmed in code; the ped DDS path is deferred). For **remote peds** there is a *separate* producer,
`Sim.Viewer --mode ped-publish`, that City3D subscribes to with `--transport=dds --peds`:
```
# producer:  dotnet run --project src/Sim.Viewer -- --mode ped-publish
# subscriber: GODOT --path demos\City3D\Viewer -- --transport=dds --peds
```
(Requires the `-p:City3DRemote=true` build and the same multicast.)

### 4.5 Raylib remote (optional, for parity of understanding)
```
# producer:  dotnet run --project src/Sim.Host.App -- --scenario scenarios/09-traffic-light --transport dds --hz 10
# viewer:    dotnet run --project src/Sim.Viewer -- --mode remote
```

---

## 5. What "good" looks like (the sign-off)
Watch, and report anything off with **vehicle id + time** (that's how we diagnosed everything — we can pull a
`--trace-veh` and localize it):
- **Junction turns**: one continuous rotation, no yaw snaps/steps; the car rides the connecting-lane centerline.
- **Lane changes**: a gentle lateral ease over ~1–1.5 s, not an instant sideways jump.
- **Car placement**: the box sits *centered* on the lane (not shoved forward); long vehicles swing wide but
  their rear tracks inside like a real car.
- **Stops**: no creep/jitter at a red light.
- **Peds** (`--peds`): the crossing-plaza crowd walks the sidewalks/crossings smoothly.

## 6. Known gaps / candidate tasks for this session (design-first if you tackle them)
1. **Cars + pedestrians in ONE City3D scene.** Today `--peds` replaces the vehicle city. A real unification =
   run `SimSource` (vehicles) and `PedSimSource` (peds) together over a net that has *both* road + pedestrian
   infrastructure, and render both `MultiMesh`es. Needs a suitable combined net (e.g. adapt
   `_ped/subarea-box`, which has both but is named `net.xml`), and a small `Main.cs`/`Reconstructor` change.
2. **DDS pedestrians in the generic host.** `Sim.Host.App` is cars-only; a combined cars+peds DDS producer
   (or wiring `--mode ped-publish` alongside) would let remote City3D show both. Deferred by design.
3. **Windows run scripts.** The `demos/City3D/*.sh` are bash-only; a PowerShell `build.ps1`/`run-local.ps1`
   mirroring §4.3–4.4 would make this repeatable (nice-to-have).
4. The `--mode local` GL-teardown segfault seen headless on Linux/Xvfb is **not** a concern on a Windows GPU
   box (it's an exit-time software-GL artifact; rendering completes) — ignore it there.

## 7. First 15 minutes (suggested)
1. `dotnet test tests/Sim.ParityTests` → confirm 654/4 (sanity that the checkout is clean).
2. Raylib cars: `dotnet run --project src/Sim.Viewer -- --mode loopback scenarios/44-multilane-junction-turn`
   — eyeball the junction smoothing. This needs no packaging and is the fastest first light-up.
3. City3D local cars (§4.3 build, then the `09-traffic-light --camera=close` run).
4. City3D local peds (`--peds`).
5. City3D remote DDS cars (§4.4).
Then tell me what looks right / wrong, and whether you want to pick up a §6 task (I'll write the design first).
