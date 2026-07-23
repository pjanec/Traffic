using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using CityLib;
using Godot;
using Sim.Core;
using Sim.Ingest;
using Sim.LiveCity;
using Sim.Replication;
#if CITY3D_REMOTE
using CycloneDDS.Runtime;
using Sim.Replication.Dds;
#endif

namespace Viewer;

// docs/DEMO-CITY3D-DESIGN.md "Code structure" / task T1.1 — the Godot glue skeleton. This node proves the
// packaged-consumer chain works INSIDE Godot: it builds a CityLib.SimSource over a real scenario, ticks
// the sim on a fixed sim-cadence accumulator, reconstructs per-frame vehicle poses via CityLib.Reconstructor,
// and logs a heartbeat. Task T1.3 (design "Procedural scene generation -> Roads") adds the one-time static
// road-ribbon mesh build (CityLib.RoadMeshBuilder does the math; this class only turns the plain
// float/int arrays into Godot ArrayMesh/MeshInstance3D nodes) and a Camera3D + DirectionalLight3D framing
// the network. No SumoSharp.* type is referenced directly here; everything computable comes through
// CityLib (repo rule: the Godot layer is thin glue only).
public partial class Main : Node3D
{
    // Matches scenarios/09-traffic-light/config.sumocfg's step-length (1s) -- the sim advances one Engine
    // step per simulated second; the Reconstructor still runs every rendered frame, which is the whole
    // point of the DR (dead-reckoning) motion story (design "Data path").
    private const double SimStepSeconds = 1.0;

    // Playout delay per design "Playout delay": a stable, small manual knob (~0.3-0.5s), not auto-driven.
    private const double PlayoutDelaySeconds = 0.4;

    // Quit after this many rendered frames -- enough to observe several sim ticks and a non-zero vehicle
    // count on scenarios/09-traffic-light, while keeping the headless smoke run fast. At a fixed 60 FPS
    // (see run-smoke.sh's `--fixed-fps 60`, needed because headless dummy-renderer wall-clock deltas are
    // far smaller than real-time and would otherwise take hundreds of real frames to accumulate one
    // simulated second) this covers ~3 sim ticks (SimStepSeconds=1).
    private const int QuitAfterFrames = 200;

    // Dark asphalt (design "Roads": "Dark asphalt material") -- one shared material, every road
    // MeshInstance3D reuses it (a single StandardMaterial3D resource, not one per lane).
    private static readonly Color AsphaltColor = new(0.08f, 0.085f, 0.09f);

    // Small seeded grey palette (design "Buildings": "variety without assets = scale + a small palette of
    // seeded materials") -- picked by instance index, which is itself a deterministic function of
    // (edge, step, side) via BuildingPlacer, so the palette assignment is reproducible too.
    private static readonly Color[] BuildingPalette =
    {
        new(0.55f, 0.55f, 0.58f),
        new(0.62f, 0.60f, 0.57f),
        new(0.47f, 0.49f, 0.52f),
        new(0.58f, 0.56f, 0.51f),
    };

    // Small seeded car-body palette (design "Cars" / task T1.5, mirrors BuildingPalette's "variety without
    // assets" pattern) -- picked deterministically by VehicleHandle.Index (stable across frames for a given
    // vehicle, no hidden RNG), not by draw order.
    private static readonly Color[] CarPalette =
    {
        new(0.75f, 0.15f, 0.12f),
        new(0.12f, 0.35f, 0.70f),
        new(0.85f, 0.80f, 0.20f),
        new(0.20f, 0.55f, 0.25f),
        new(0.90f, 0.90f, 0.92f),
        new(0.25f, 0.25f, 0.28f),
    };

    // Design "Cars": "height a believable constant (~1.4 m for cars)" -- CityLib.CarTransform.DefaultHeightMeters.
    private const float CarHeightMeters = CityLib.CarTransform.DefaultHeightMeters;

    // docs/DEMO-CITY3D-DESIGN.md "#### Pedestrians (P7-3)" -- wall-clock seconds consumed per one 0.2s ped-sim
    // Tick(). The ped path reuses the SAME sim-cadence accumulator pattern the vehicle path uses (fixed
    // increments off `delta`), just with a ped-appropriate threshold: 0.025s wall per 0.2s ped-tick runs the
    // ped sim ~8x real time so the swept crowd (spawns every 4s ped-time, 70s sweep period) populates and
    // promotes within a short `--shot-delay` window instead of needing tens of real seconds.
    private const double PedTickWallSeconds = 0.025;

    // docs/DEMO-CITY3D-DESIGN.md "#### Pedestrians (P7-3)": "slate low-power ... cyan high-power ... visibly
    // distinct". The per-instance MultiMesh colour each ped avatar is tinted by, keyed off its regime.
    private static readonly Color PedLowPowerColor = new(0.58f, 0.64f, 0.72f);   // slate  -- low-power PathArc
    private static readonly Color PedHighPowerColor = new(0.22f, 0.74f, 0.97f);  // cyan   -- high-power FreeKinematic

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §6, -TASKS.md D1 -- the `--live-city` ped regime palette (distinct
    // from the plaza `--peds` palette above, since Sim.LiveCity.PedRegime has a THIRD state, Paused, the
    // plaza's two-state PedRegime doesn't): grey = low-power weave, orange = promoted full-ORCA high-power,
    // yellow = paused/dwell.
    private static readonly Color LiveCityPedLowPowerColor = new(0.55f, 0.55f, 0.55f);  // grey
    private static readonly Color LiveCityPedHighPowerColor = new(0.92f, 0.55f, 0.12f); // orange
    private static readonly Color LiveCityPedPausedColor = new(0.92f, 0.85f, 0.20f);    // yellow

    // docs/LIVE-CITY-VIEWERS-TASKS.md D1 -- the coupled sim's own tick length (Sim.LiveCity.LiveCityConfig.Dt
    // default). The live-city accumulator advances LiveCitySource.Tick() in this many whole increments,
    // exactly like AdvanceLocalSim/ProcessPeds' own fixed sim-cadence accumulators, just on ITS OWN fields
    // (design §6: "the shared per-frame accumulator/frame counter are split per-domain").
    private const double LiveCityTickSeconds = 0.5;

    // Half-extent (metres) of the tight box the `--live-city` OVERVIEW camera frames, centred on the crop
    // (see ReadyLiveCity's remark): small enough that cars/peds are legible in a fixed-resolution
    // screenshot, large enough to still show several intersections' worth of the coupled scene.
    private const float LiveCityFrameHalfExtentMeters = 180f;

    // Viewer-only LEGIBILITY render scale for the ped avatars (docs/DEMO-CITY3D-DESIGN.md "#### Pedestrians
    // (P7-3)"): the tested CityLib.PedTransform keeps the true ~0.5x1.8 m avatar, but at plaza camera range a
    // 0.5 m figure is barely a pixel, so the Viewer draws a scaled-up slim avatar purely for on-screen
    // legibility -- the exact same idea src/Sim.Viewer/RemotePedOverlay.cs uses (it renders peds at a 1.2 m
    // disc vs the 0.3 m physical radius). Proportions stay slim/upright so it still reads as a pedestrian.
    private const float PedRenderHeightMeters = 3.2f;
    private const float PedRenderWidthMeters = 1.1f;

    // The ped camera frames a tight box around the crossing plaza (peds only ever cross the central ~40 m;
    // the road arms extend ~240 m, so framing the whole road bbox -- as the vehicle path does -- would leave
    // the crowd sub-pixel). A fixed half-extent around the road bbox centre gives a legible plaza view.
    private const float PedFrameHalfExtentMeters = 26f;

    // How far above/behind the network's centre (as a multiple of its largest XZ extent) the framing
    // camera sits -- an "angled bird's-eye" per design "Buildings"/"Roads" desktop-check framing. Raised
    // a bit from the roads-only T1.3 values (task T1.4: "you may raise the camera a bit so buildings are
    // in frame") since buildings add tens-of-metres of vertical extent the horizontal bbox alone doesn't
    // capture.
    private const float CameraHeightFactor = 1.1f;
    private const float CameraBackFactor = 0.9f;

    // Task T1.6 Part C -- the `--camera=close` framing (design "Traffic lights" desktop check + this
    // task's Part C: "~40-80m back and ~15-25m up, tilted down, so cars/lights/road-width are actually
    // visible at real scale"). A FIXED offset (not extent-scaled like the overview camera above) is the
    // whole point: this is a real-scale close-up, not a whole-network fit.
    private const float CloseCameraBackMeters = 40f;
    private const float CloseCameraUpMeters = 15f;
    // A modest lateral (across-the-road) component blended into the back offset -- purely along the
    // corridor axis looks straight down a narrow (~3.2m) lane with tall buildings only ~6m off each edge,
    // which reads as a needle-thin canyon shot; a small sideways component gives a 3/4-angled "corner
    // camera" framing (closer to a real traffic-cam mount) without reopening the sideways
    // look-through-the-building-wall problem a full 90deg offset has.
    private const float CloseCameraLateralMeters = 12f;

    // Task T3.1 -- video-wall channel tile pixel height (each channel's SubViewport). Width is derived per
    // channel so its aspect matches what that channel's frustum actually needs (screen-corners mode derives
    // aspect from the geometry itself; offset+fov mode uses WallDefaultAspect below), never distorted/
    // stretched to fit a fixed box.
    private const int WallTileHeightPx = 480;
    private const double WallDefaultAspect = 4.0 / 3.0;

    // Task T1.6 -- emissive colours the signal head material is set to each frame, keyed off
    // CityLib.TrafficLightPlacer.ColorFor's colour bucket (design "Traffic lights": "each frame set the
    // head's emissive material from that approach's current signal letter -> red/amber/green").
    private static readonly Color TlRedColor = new(0.95f, 0.12f, 0.10f);
    private static readonly Color TlYellowColor = new(0.95f, 0.85f, 0.12f);
    private static readonly Color TlGreenColor = new(0.15f, 0.90f, 0.20f);
    private static readonly Color TlOffColor = new(0.22f, 0.22f, 0.24f);
    private static readonly Color TlPoleColor = new(0.15f, 0.15f, 0.16f);

    private SimSource? _sim;
    private Reconstructor? _reconstructor;
    private double _accumulator;
    private int _frame;
    private string? _shotPath;
    private string _cameraMode = "overview";
    private (Vector3 Min, Vector3 Max) _sceneBbox;
    private Camera3D? _closeCamera;

    // docs/DEMO-CITY3D-DESIGN.md "Data path -> Remote mode" / task T2.2b -- `--transport=local|dds`.
    // "local" (default) preserves today's behavior byte-for-byte. "dds" only works in a build compiled
    // with -p:City3DRemote=true (see the non-remote-build guard in _Ready).
    private string _transport = "local";

    // The transport-neutral read side RenderFrame() (below) is written against -- design tenet 3: "only
    // the IReplicationSource + the ILaneShapeSource differ" between local and remote. ReadyLocal() points
    // these at `_sim.Source` / `_sim.LocalLanes`; ReadyRemote()/BuildRemoteScene() point them at the
    // DdsSubscriber + a wire-fed ReplicationLaneShapeSource (the latter only once geometry has arrived --
    // `_lanes` stays null until then, which _Process treats as "nothing to reconstruct/render yet").
    private IReplicationSource? _source;
    private ILaneShapeSource? _lanes;

    // The one placement lookup TrafficLightPlacer needs beyond ILaneShapeSource (per-lane Width;
    // ILaneShapeSource doesn't expose it) -- supplied as a closure over whichever per-lane data this
    // transport actually has (NetworkModel locally, the wire's LaneGeo dictionary remotely,
    // CityLib.TrafficLightPlacer's two Place(...) overloads) so RenderFrame's TL-build call never needs to
    // branch on `_transport` itself.
    private Func<IEnumerable<int>, IReadOnlyList<SignalHead>>? _placeSignalHeads;

#if CITY3D_REMOTE
    private DdsParticipant? _ddsParticipant;
    private DdsSubscriber? _ddsSource;
    private bool _remoteSceneBuilt;
    // D3b (docs/PEDESTRIAN-DDS-TRANSPORT-TASKS.md): the live-DDS ped source for `--transport=dds --peds` --
    // fed by a separate `--mode ped-publish` process (Sim.Viewer), reconstructed by _pedReconstructor. The
    // server clock is wire-authoritative (newest crowd-frame time), kept monotonic across frames.
    private DdsPedReplicationSource? _ddsPedSource;
    private double _pedRemoteServerTime;
#endif

    // Task T1.6 Part B -- built LAZILY on the first _Process frame `sim.Source.TlStateByLane` is
    // non-empty (design: geometry+TL state only exist once the bus has been pumped at least once), then
    // reused every frame after -- only each head's material colour is rewritten per frame (design
    // "Traffic lights": "Only the material param updates per frame"), same "build once, mutate per frame"
    // split RoadMeshBuilder (static) / BuildCarMultiMesh+UpdateCars (per frame) already establish.
    private readonly List<(int LaneHandle, MeshInstance3D HeadInstance, StandardMaterial3D HeadMaterial)> _signalHeads = new();
    private bool _signalHeadsBuilt;

    // Task T1.5 -- ONE MultiMeshInstance3D for every car, built once in _Ready and reused every frame:
    // only the per-instance transforms/colors are rewritten each _Process; the underlying buffer
    // (MultiMesh.InstanceCount) is only ever grown, never shrunk, so a transient dip in vehicle count
    // doesn't force a reallocation -- VisibleInstanceCount (cheap to set) is what actually tracks the live
    // count frame to frame.
    private MultiMesh? _carMultiMesh;

    // docs/DEMO-CITY3D-DESIGN.md "#### Pedestrians (P7-3)" -- the `--peds` LOCAL ped path. When set, the
    // viewer skips the vehicle SimSource entirely and instead hosts CityLib.PedSimSource (the ped server sim
    // + byte-loopback wire) reconstructed each frame by CityLib.PedReconstructor into ONE ped
    // MultiMeshInstance3D, built once and updated per frame exactly like the car MultiMesh. All ped types
    // live in CityLib (Godot-free); this class only turns PedInstance structs into Transform3D + regime
    // colour. Off unless `--peds` is passed -- the non-ped path is byte-identical.
    private bool _peds;
    private PedSimSource? _pedSim;
    private PedReconstructor? _pedReconstructor;
    private MultiMesh? _pedMultiMesh;

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §6, -TASKS.md Stage D (D1) -- the `--live-city` LOCAL combined
    // cars+peds path. When set, ReadyLiveCity/ProcessLiveCity render BOTH a car MultiMesh (via the SAME
    // Reconstructor/UpdateCars the --scenario path uses, over LiveCitySource.Source/LocalLanes) AND a ped
    // MultiMesh (UpdateLivePeds, fed by LiveCitySource.Peds) in ONE scene -- no `if(_peds){...return;}`
    // mutual exclusion. Its own accumulator/frame fields (below) are SEPARATE from _accumulator/_frame (the
    // vehicle/ped-plaza paths' own) so both domains can advance in the same tick without a field collision
    // (design §6).
    private bool _liveCity;
    private LiveCitySource? _liveCitySource;
    private double _liveCityAccumulator;
    private int _liveCityFrame;

    public override void _Ready()
    {
        _transport = ParseTransportArg();
        _peds = ParsePedsArg();
        _liveCity = ParseLiveCityArg();

        switch (_transport)
        {
            case "local":
                ReadyLocal();
                return;

            case "dds":
#if CITY3D_REMOTE
                // D3b: `--peds` over live DDS reuses the ped scene setup (local plaza net for the backdrop)
                // but swaps the local byte-loopback ped-sim for a DdsPedReplicationSource fed by a separate
                // `--mode ped-publish` process; the vehicle remote path (ReadyRemote) is unchanged.
                if (_peds)
                {
                    string pedRepoRoot;
                    try
                    {
                        pedRepoRoot = FindRepoRoot();
                    }
                    catch (Exception ex)
                    {
                        GD.PrintErr($"Main: could not locate repo root (searched upward for Traffic.sln): {ex.Message}");
                        GetTree().Quit(1);
                        return;
                    }

                    ReadyPeds(pedRepoRoot);
                }
                else
                {
                    ReadyRemote();
                }
#else
                GD.PrintErr(
                    "Main: --transport=dds requires a build with DDS support -- rebuild with " +
                    "-p:City3DRemote=true (adds SumoSharp.Replication.Dds; see demos/City3D/run-remote.sh).");
                GetTree().Quit(2);
#endif
                return;

            default:
                GD.PrintErr($"Main: unknown --transport '{_transport}' (expected local|dds).");
                GetTree().Quit(2);
                return;
        }
    }

    // docs/DEMO-CITY3D-DESIGN.md "Data path" LOCAL half -- today's pre-T2.2b _Ready body, unchanged logic,
    // just pulled into its own method so _Ready can dispatch on --transport. Builds the in-process
    // SimSource and the whole static scene (roads+buildings+camera; NetworkModel is known upfront locally,
    // unlike dds), then points the transport-neutral _source/_lanes/_placeSignalHeads fields at the local
    // implementations -- everything from here on (RenderFrame, UpdateCars, UpdateTrafficLights) runs
    // through those same fields regardless of transport.
    private void ReadyLocal()
    {
        string repoRoot;
        try
        {
            repoRoot = FindRepoRoot();
        }
        catch (Exception ex)
        {
            GD.PrintErr($"Main: could not locate repo root (searched upward for Traffic.sln): {ex.Message}");
            GetTree().Quit(1);
            return;
        }

        // docs/LIVE-CITY-VIEWERS-DESIGN.md §6, -TASKS.md D1: `--live-city` takes its OWN dedicated LOCAL
        // path (a LiveCitySource over the coupled cars+peds+crossing-yield host, rendering both a car and a
        // ped MultiMesh in one scene) -- checked BEFORE the `--peds` fork below so the two never collide;
        // the vehicle-SimSource setup further down is never touched in live-city mode either.
        _liveCity = ParseLiveCityArg();
        if (_liveCity)
        {
            ReadyLiveCity(repoRoot);
            return;
        }

        // docs/DEMO-CITY3D-DESIGN.md "#### Pedestrians (P7-3)": `--peds` takes a dedicated LOCAL path (its
        // own poc0-crossing-plaza net for roads + ped server-sim/wire/render), skipping the vehicle
        // SimSource/buildings/TLs entirely. Parsed here so the vehicle setup below is never even touched in
        // ped mode.
        _peds = ParsePedsArg();
        if (_peds)
        {
            ReadyPeds(repoRoot);
            return;
        }

        // `--scenario=<rel>` (a path under scenarios/, e.g. "09-traffic-light" or "_bench/city-organic"),
        // default the tiny signalized dev scenario. A minimal precursor to T1.7's full scenario switch --
        // enough to render a real multi-road network for the screenshot.
        var scenarioRel = ParseScenarioArg();
        var scenarioDir = Path.Combine(repoRoot, "scenarios", scenarioRel);
        var netPath = Path.Combine(scenarioDir, "net.net.xml");
        var rouPath = Path.Combine(scenarioDir, "rou.rou.xml");
        var cfgPath = Path.Combine(scenarioDir, "config.sumocfg");

        if (!File.Exists(netPath) || !File.Exists(rouPath) || !File.Exists(cfgPath))
        {
            GD.PrintErr(
                $"Main: scenario 'scenarios/{scenarioRel}' not found under repo root '{repoRoot}' " +
                $"(expected {netPath}, {rouPath}, {cfgPath}).");
            GetTree().Quit(1);
            return;
        }

        try
        {
            _sim = new SimSource(netPath, rouPath, cfgPath);
        }
        catch (Exception ex)
        {
            GD.PrintErr($"Main: failed to construct SimSource: {ex}");
            GetTree().Quit(1);
            return;
        }

        _reconstructor = new Reconstructor();
        _source = _sim.Source;
        _lanes = _sim.LocalLanes;
        var sim = _sim;
        _placeSignalHeads = keys => TrafficLightPlacer.Place(sim.Network, keys);
        GD.Print($"Main: loaded scenario '{scenarioRel}' from '{scenarioDir}' (transport=local).");

        _cameraMode = ParseCameraArg();

        var roadBbox = BuildRoadMeshes(_sim.Network);
        var buildingBbox = BuildBuildings(_sim.Network);
        var bbox = CombineBbox(roadBbox, buildingBbox);
        _sceneBbox = bbox;

        // Task T3.1 ("Multi-channel video wall", user-refined to CLI-only channels, no screen
        // autodetection): ONE shared scene (built above, exactly as before) + ONE shared replication
        // source (SimSource/_sim, also unchanged) + N cameras, each a possibly off-axis
        // OffAxisFrustum.FrustumSpec computed from its own `--channel=...` arg. Parsed here, BEFORE the
        // camera/light setup below, purely to decide whether the default single overview/close camera
        // should be Current -- when in wall mode, NONE of the whole-network framing camera / --camera=close
        // camera become Current (BuildVideoWallChannels below supplies every Current camera instead), so a
        // run with no --channel is byte-identical to the pre-T3.1 behavior (the `!wallMode &&` term below
        // is the ONLY change to this condition).
        var channels = ParseChannelArgs();
        var wallMode = channels.Count > 0;

        // The overview camera is still always built (env + light + the default whole-network framing
        // run-smoke.sh relies on) -- `--camera=close`/wall mode just leave it non-Current in favour of the
        // close camera / video-wall channel cameras built below, rather than forking the
        // environment/light setup.
        BuildCameraAndLight(bbox, makeCurrent: !wallMode && _cameraMode != "close");
        _carMultiMesh = BuildCarMultiMesh();

        if (wallMode)
        {
            BuildVideoWallChannels(channels);
            GD.Print($"Main: video wall — {channels.Count} channel(s).");
        }
        else if (_cameraMode == "close")
        {
            _closeCamera = new Camera3D { Name = "CloseCamera", Current = true, Far = 2000f };
            AddChild(_closeCamera);
            UpdateCloseCameraFraming(_closeCamera, null);
            GD.Print("Main: --camera=close active (low angled close-up framing).");
        }

        _shotPath = ParseShotArg();
        if (_shotPath is not null)
        {
            var shotDelaySeconds = ParseShotDelayArg();
            GD.Print(
                $"Main: --shot requested, will capture to '{_shotPath}' " +
                $"(shot-delay={shotDelaySeconds:F1}s real wall-clock before capture).");
            CaptureScreenshotAsync(_shotPath, shotDelaySeconds);
        }
    }

    // docs/DEMO-CITY3D-DESIGN.md "#### Pedestrians (P7-3)" -- the `--peds` LOCAL setup. Uses the
    // poc0-crossing-plaza net for the ROAD context (built exactly like the vehicle path's BuildRoadMeshes,
    // from a NetworkModel), then hosts the ped server-sim + byte-loopback wire (CityLib.PedSimSource) and its
    // reconstructor (CityLib.PedReconstructor) plus ONE ped MultiMeshInstance3D. No vehicle SimSource,
    // buildings or traffic lights (poc0 has no rou/config, and peds are a separate engine). The camera frames
    // the ped net's bbox.
    private void ReadyPeds(string repoRoot)
    {
        var pedNetPath = Path.Combine(repoRoot, "scenarios", "_ped", "poc0-crossing-plaza", "net.net.xml");
        if (!File.Exists(pedNetPath))
        {
            GD.PrintErr($"Main: --peds requested but ped net not found at '{pedNetPath}'.");
            GetTree().Quit(1);
            return;
        }

        NetworkModel pedNetwork;
        try
        {
            // Roads-only context for the plaza: the SAME NetworkParser + RoadMeshBuilder path the vehicle
            // viewer uses. (The walkable ped nav is parsed independently INSIDE PedSimSource.)
            pedNetwork = NetworkParser.Parse(pedNetPath);
#if CITY3D_REMOTE
            if (_transport == "dds")
            {
                // D3b: no local ped-sim -- subscribe to a separate `--mode ped-publish` process's live DDS
                // ped stream. The plaza net (backdrop) is still loaded LOCALLY above, so only ped poses cross
                // the wire. _pedReconstructor (below) consumes this IPedReplicationSource transport-neutrally.
                _ddsParticipant ??= new DdsParticipant();
                _ddsPedSource = new DdsPedReplicationSource(_ddsParticipant);
            }
            else
#endif
            {
                _pedSim = new PedSimSource(repoRoot);
            }
        }
        catch (Exception ex)
        {
            GD.PrintErr($"Main: failed to build the ped subsystem: {ex}");
            GetTree().Quit(1);
            return;
        }

        _pedReconstructor = new PedReconstructor();
        _cameraMode = ParseCameraArg();

        var roadBbox = BuildRoadMeshes(pedNetwork);
        _sceneBbox = roadBbox;

        // Frame a tight box centred on the road bbox (the crossing plaza centre) so the ~40 m crowd fills the
        // shot rather than being lost in the ~240 m road arms.
        var roadCenter = new Vector3(
            (roadBbox.Min.X + roadBbox.Max.X) / 2f, 0f, (roadBbox.Min.Z + roadBbox.Max.Z) / 2f);
        var h = PedFrameHalfExtentMeters;
        var pedBbox = (
            Min: roadCenter - new Vector3(h, 0f, h),
            Max: roadCenter + new Vector3(h, 8f, h));
        BuildCameraAndLight(pedBbox, makeCurrent: _cameraMode != "close");

        if (_cameraMode == "close")
        {
            _closeCamera = new Camera3D { Name = "CloseCamera", Current = true, Far = 2000f };
            AddChild(_closeCamera);
            UpdateCloseCameraFraming(_closeCamera, null);
        }

        _pedMultiMesh = BuildPedMultiMesh();
        var pedTransportDesc = "transport=local, byte loopback";
#if CITY3D_REMOTE
        if (_ddsPedSource is not null)
        {
            pedTransportDesc = "transport=dds, live CycloneDDS (needs a running `--mode ped-publish` process)";
        }
#endif
        GD.Print($"Main: --peds active; crossing-plaza render ({pedTransportDesc}).");

        _shotPath = ParseShotArg();
        if (_shotPath is not null)
        {
            var shotDelaySeconds = ParseShotDelayArg();
            GD.Print(
                $"Main: --shot requested, will capture to '{_shotPath}' " +
                $"(shot-delay={shotDelaySeconds:F1}s real wall-clock before capture).");
            CaptureScreenshotAsync(_shotPath, shotDelaySeconds);
        }
    }

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §6, -TASKS.md D1/D2 -- the `--live-city` LOCAL setup. Builds a
    // CityLib.LiveCitySource (wraps Sim.LiveCity.LiveCitySim over scenarios/_ped/demo_city/box), the road
    // meshes from ITS NetworkModel (the same RoadMeshBuilder entry point the vehicle path uses, so
    // LiveCitySource.LocalLanes' Lane.ShapeZ threads through identically -- D2), a car MultiMesh (reused
    // unchanged from the --scenario path) AND a ped MultiMesh (BuildPedMultiMesh, same shape as the plaza
    // path's empty-at-load build), and points the generic `_reconstructor`/`_source`/`_lanes` fields at the
    // live-city source so the SAME Reconstructor/UpdateCars render live-city cars -- only ProcessLiveCity
    // (its own accumulator) ticks the sim and calls them, never AdvanceLocalSim/RenderFrame.
    private void ReadyLiveCity(string repoRoot)
    {
        try
        {
            _liveCitySource = new LiveCitySource(repoRoot);
        }
        catch (Exception ex)
        {
            GD.PrintErr($"Main: failed to construct LiveCitySource: {ex}");
            GetTree().Quit(1);
            return;
        }

        _reconstructor = new Reconstructor();
        _source = _liveCitySource.Source;
        _lanes = _liveCitySource.LocalLanes;

        _cameraMode = ParseCameraArg();

        // LiveCitySource.Network is the FULL parsed net.xml (scenarios/_ped/demo_city/box's net.xml spans
        // 4750x4750m) -- only LiveCitySource.Crop (~840x840m) is where cars/peds actually live. Building
        // road meshes / framing the camera over the WHOLE net would put the crop at sub-pixel scale (every
        // lane a hairline, the camera effectively looking at the entire city from orbit) -- so the local
        // live-city path builds/frames ONLY the crop's lanes, unlike the vehicle --scenario path (whose
        // nets ARE the whole playable area already).
        var (x0, y0, x1, y1) = _liveCitySource.Crop;
        var roadBbox = BuildRoadMeshesCropped(_liveCitySource.Network, x0, y0, x1, y1);
        _sceneBbox = roadBbox;

        // Even cropped to the ~840x840m downtown block, the FULL crop bbox still leaves individual
        // ~4.5m cars/~0.5m peds sub-legible in a fixed-resolution screenshot (whole-network framing
        // scales the camera to the LARGEST extent). Mirrors ReadyPeds' own move (a tight frame box
        // distinct from `_sceneBbox`, which stays the real road bbox for the close-camera fallback):
        // frame the overview camera on a smaller box centred on the crop, wide enough to show several
        // intersections' worth of traffic+crowd while keeping entities legible.
        var cropCenter = new Vector3(
            (roadBbox.Min.X + roadBbox.Max.X) / 2f, 0f, (roadBbox.Min.Z + roadBbox.Max.Z) / 2f);
        var frameHalf = LiveCityFrameHalfExtentMeters;
        var frameBbox = (
            Min: cropCenter - new Vector3(frameHalf, 0f, frameHalf),
            Max: cropCenter + new Vector3(frameHalf, 8f, frameHalf));

        BuildCameraAndLight(frameBbox, makeCurrent: _cameraMode != "close");
        _carMultiMesh = BuildCarMultiMesh();
        _pedMultiMesh = BuildPedMultiMesh();

        if (_cameraMode == "close")
        {
            _closeCamera = new Camera3D { Name = "CloseCamera", Current = true, Far = 2000f };
            AddChild(_closeCamera);
            UpdateCloseCameraFraming(_closeCamera, null);
            GD.Print("Main: --camera=close active (low angled close-up framing).");
        }

        GD.Print("Main: --live-city active (coupled cars+peds+crossing-yield, transport=local).");

        _shotPath = ParseShotArg();
        if (_shotPath is not null)
        {
            var shotDelaySeconds = ParseShotDelayArg();
            GD.Print(
                $"Main: --shot requested, will capture to '{_shotPath}' " +
                $"(shot-delay={shotDelaySeconds:F1}s real wall-clock before capture).");
            CaptureScreenshotAsync(_shotPath, shotDelaySeconds);
        }
    }

#if CITY3D_REMOTE
    // docs/DEMO-CITY3D-DESIGN.md "Data path -> Remote mode" / task T2.2b -- the REMOTE half. No
    // SimSource/Engine is ever created here (design: "do NOT create a SimSource/engine"); the only source
    // of truth is a DdsSubscriber reading whatever a separate `Sim.Host.App --transport dds` process
    // publishes. The static scene (roads/camera; buildings are skipped, see BuildRemoteScene) and the
    // wire-fed lane source can't be built yet -- there is no geometry until the wire delivers it -- so
    // `_lanes` is left null here and set once by BuildRemoteScene, called lazily from _Process the first
    // frame `_ddsSource.GeometryComplete` goes true.
    private void ReadyRemote()
    {
        _cameraMode = ParseCameraArg();

        _ddsParticipant = new DdsParticipant();
        _ddsSource = new DdsSubscriber(_ddsParticipant);
        _reconstructor = new Reconstructor();
        _source = _ddsSource;
        var ddsSource = _ddsSource;
        _placeSignalHeads = keys => TrafficLightPlacer.Place(ddsSource.Geometry, keys);

        _carMultiMesh = BuildCarMultiMesh();

        GD.Print("Main: --transport=dds active; waiting for a Sim.Host.App publisher (geometry + frames)...");

        _shotPath = ParseShotArg();
        if (_shotPath is not null)
        {
            var shotDelaySeconds = ParseShotDelayArg();
            GD.Print(
                $"Main: --shot requested, will capture to '{_shotPath}' " +
                $"(shot-delay={shotDelaySeconds:F1}s real wall-clock before capture).");
            CaptureScreenshotAsync(_shotPath, shotDelaySeconds);
        }
    }

    // Lazily builds the static scene from the RECEIVED wire geometry, once complete. A remote viewer never
    // has a NetworkModel, so roads come from RoadMeshBuilder's geometry-dictionary overload
    // (CityLib/RoadMeshBuilder.cs). Buildings are SKIPPED, not silently wrong: BuildingPlacer needs
    // edge-level lane grouping (Sim.Ingest.NetworkModel.Edges) the wire does not carry (design T2.2b:
    // "buildings need widths/edges the wire may not carry ... SKIP buildings if the data isn't available
    // (log it)").
    private void BuildRemoteScene()
    {
        var geometry = _ddsSource!.Geometry;
        _lanes = new ReplicationLaneShapeSource(geometry);

        var roadBbox = BuildRoadMeshesFromGeometry(geometry);
        GD.Print(
            $"Main: --transport=dds received geometry ({geometry.Count} lane(s)); SKIPPING buildings " +
            "(the wire's LaneGeo carries no edge/lane grouping for BuildingPlacer to march along -- " +
            "roads/cars/traffic-lights still render).");

        _sceneBbox = roadBbox;
        BuildCameraAndLight(roadBbox, makeCurrent: _cameraMode != "close");

        if (_cameraMode == "close")
        {
            _closeCamera = new Camera3D { Name = "CloseCamera", Current = true, Far = 2000f };
            AddChild(_closeCamera);
            UpdateCloseCameraFraming(_closeCamera, null);
            GD.Print("Main: --camera=close active (low angled close-up framing).");
        }
    }
#endif

    public override void _Process(double delta)
    {
        if (_liveCity)
        {
            ProcessLiveCity(delta);
            return;
        }

        if (_peds)
        {
            ProcessPeds(delta);
            return;
        }

        if (_reconstructor is null || _source is null)
        {
            // _Ready already reported the error (or the wrong-build --transport=dds guard already
            // requested quit); nothing more to do this frame.
            return;
        }

        if (_transport == "local")
        {
            AdvanceLocalSim(delta);
        }
#if CITY3D_REMOTE
        else if (_transport == "dds")
        {
            AdvanceRemoteSim();
        }
#endif

        if (_lanes is not null)
        {
            // The ONE per-frame reconstruction+render body -- identical whether _source/_lanes are the
            // local in-process pair or the remote DDS pair (design tenet 3 / T2.2b).
            RenderFrame();
        }

        _frame++;

        // Skip the frame-count auto-quit while a screenshot capture is pending: CaptureScreenshotAsync
        // owns quitting in that case (docs/DEMO-CITY3D-DESIGN.md task T1.5 part C -- a real `--shot-delay`
        // lets enough WALL-CLOCK time pass for the DrClock-driven sim/render to actually populate the
        // roads with cars before the shot is taken; without this guard, a delay longer than
        // QuitAfterFrames' own real-time span at an uncapped frame rate would race the engine into
        // quitting before the delayed capture ever fires).
        //
        // LOCAL ONLY (task T2.2b finding): a headless dummy-render loop burns through QuitAfterFrames
        // (200) in well under a real wall-clock second (measured: ~0.6s including engine startup) --
        // that's a fine proxy for "enough sim ticks happened" locally, since the in-process bus has zero
        // IPC latency. Over DDS, a separate Sim.Host.App process paces itself in REAL time (its own
        // --hz step interval, plus ~500ms of post-geometry discovery settling) that this frame counter
        // knows nothing about, so quitting on frame count alone would very likely fire before any packet
        // ever arrives -- a false "no vehicles" failure that has nothing to do with DDS actually working.
        // The dds transport therefore relies entirely on an EXTERNAL wall-clock bound (run-remote.sh runs
        // the process under `timeout`) instead.
        if (_transport == "local" && _frame >= QuitAfterFrames && _shotPath is null)
        {
            GD.Print($"Main: reached {QuitAfterFrames} frames, quitting.");
            DisposeSources();
            GetTree().Quit();
        }
    }

    // Fixed sim-cadence accumulator: advance the sim in whole SimStepSeconds increments regardless of the
    // (headless-dummy-renderer) frame rate, while reconstruction still runs every _Process call.
    private void AdvanceLocalSim(double delta)
    {
        _accumulator += delta;
        while (_accumulator >= SimStepSeconds)
        {
            _sim!.Tick();
            _accumulator -= SimStepSeconds;
        }
    }

    // docs/DEMO-CITY3D-DESIGN.md "#### Pedestrians (P7-3)" -- the per-frame ped body: advance the ped sim on
    // the sim-cadence accumulator (fixed 0.2s ped-Tick increments off `delta`, threshold PedTickWallSeconds),
    // reconstruct the crowd from the wire, and rewrite the ped MultiMesh transforms + regime colours. Mirrors
    // the vehicle AdvanceLocalSim + RenderFrame/UpdateCars split, one entity-type over.
    private void ProcessPeds(double delta)
    {
        if (_pedReconstructor is null)
        {
            return; // _Ready already reported the error.
        }

        Sim.Replication.IPedReplicationSource pedSource;
        double serverTime;
#if CITY3D_REMOTE
        if (_ddsPedSource is not null)
        {
            // Remote: no local sim to tick. Drive the reconstructor off the WIRE clock (newest crowd-frame
            // sim-time), kept monotonic so a quiet low-power spell never rewinds playout. Reconstruct() pumps
            // the DDS source internally.
            _pedRemoteServerTime = Math.Max(_pedRemoteServerTime, _ddsPedSource.LatestCrowdTime);
            pedSource = _ddsPedSource;
            serverTime = _pedRemoteServerTime;
        }
        else
#endif
        {
            if (_pedSim is null)
            {
                return;
            }

            _accumulator += delta;
            while (_accumulator >= PedTickWallSeconds)
            {
                _pedSim.Tick();
                _accumulator -= PedTickWallSeconds;
            }

            pedSource = _pedSim.Source;
            serverTime = _pedSim.Time;
        }

        var peds = _pedReconstructor.Reconstruct(pedSource, serverTime);
        UpdatePeds(peds);

        if (_closeCamera is not null)
        {
            UpdateCloseCameraFraming(_closeCamera, null);
        }

        var highPower = 0;
        foreach (var p in peds)
        {
            if (p.IsHighPower)
            {
                highPower++;
            }
        }

        GD.Print($"Main: frame={_frame} pedTime={serverTime:F2} peds={peds.Count} highPower={highPower}");

        _frame++;

        if (_frame >= QuitAfterFrames && _shotPath is null)
        {
            GD.Print($"Main: reached {QuitAfterFrames} frames, quitting.");
            DisposeSources();
            GetTree().Quit();
        }
    }

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §6, -TASKS.md D1 -- the per-frame `--live-city` body: advance the
    // coupled sim on ITS OWN fixed sim-cadence accumulator (LiveCityTickSeconds=0.5, matching
    // LiveCityConfig.Dt -- a SEPARATE accumulator/frame pair from _accumulator/_frame, per design §6's
    // "split per-domain accumulator" requirement), reconstruct cars through the SAME Reconstructor/UpdateCars
    // the --scenario path uses (LiveCitySource.Source/LocalLanes are the SAME shapes SimSource exposes), and
    // rewrite the ped MultiMesh via UpdateLivePeds off LiveCitySource.Peds. Both MultiMeshes render in ONE
    // scene every frame -- no `if(_peds){...return;}` mutual exclusion.
    private void ProcessLiveCity(double delta)
    {
        if (_liveCitySource is null || _reconstructor is null || _lanes is null)
        {
            return; // _Ready already reported the error.
        }

        _liveCityAccumulator += delta;
        while (_liveCityAccumulator >= LiveCityTickSeconds)
        {
            _liveCitySource.Tick();
            _liveCityAccumulator -= LiveCityTickSeconds;
        }

        var vehicles = _reconstructor.Reconstruct(_liveCitySource.Source, _liveCitySource.LocalLanes, PlayoutDelaySeconds);
        UpdateCars(vehicles);

        var peds = _liveCitySource.Peds;
        UpdateLivePeds(peds);

        if (_closeCamera is not null)
        {
            UpdateCloseCameraFraming(_closeCamera, vehicles);
        }

        GD.Print(
            $"Main: frame={_liveCityFrame} liveCityTime={_liveCitySource.Time:F2} " +
            $"cars={vehicles.Count} peds={peds.Count}");

        _liveCityFrame++;

        if (_liveCityFrame >= QuitAfterFrames && _shotPath is null)
        {
            GD.Print($"Main: reached {QuitAfterFrames} frames, quitting.");
            DisposeSources();
            GetTree().Quit();
        }
    }

#if CITY3D_REMOTE
    // docs/DEMO-CITY3D-DESIGN.md "Data path -> Remote mode": "Each frame: source.Pump(); once
    // source.GeometryComplete, lazily build the road/building meshes...". Reconstructor.Reconstruct
    // (called from RenderFrame) also calls source.Pump() internally, so this extra Pump() only matters for
    // the geometry-completeness check itself running BEFORE reconstruction on the very frame it completes.
    private void AdvanceRemoteSim()
    {
        _ddsSource!.Pump();

        if (!_remoteSceneBuilt && _ddsSource.GeometryComplete)
        {
            BuildRemoteScene();
            _remoteSceneBuilt = true;
        }
    }
#endif

    // docs/DEMO-CITY3D-DESIGN.md tenet 3 / task T2.2b: the ONE per-frame reconstruction+render body, run
    // UNCHANGED whether `_source`/`_lanes` are the local in-process bus + NetworkLaneSource or the remote
    // DdsSubscriber + wire-fed ReplicationLaneShapeSource -- only those fields (plus `_placeSignalHeads`,
    // the one TL-placement lookup ILaneShapeSource itself doesn't carry) differ per transport; this method
    // never branches on `_transport`.
    private void RenderFrame()
    {
        var vehicles = _reconstructor!.Reconstruct(_source!, _lanes!, PlayoutDelaySeconds);
        UpdateCars(vehicles);

        var tlStateByLane = _source!.TlStateByLane;
        if (!_signalHeadsBuilt && tlStateByLane.Count > 0 && _placeSignalHeads is not null)
        {
            BuildTrafficLights(_placeSignalHeads(tlStateByLane.Keys));
            _signalHeadsBuilt = true;
        }

        if (_signalHeadsBuilt)
        {
            UpdateTrafficLights(tlStateByLane);
        }

        if (_closeCamera is not null)
        {
            UpdateCloseCameraFraming(_closeCamera, vehicles);
        }

        var simTimeLabel = SimTimeLabel();
        if (vehicles.Count > 0)
        {
            var v = vehicles[0];
            GD.Print(
                $"Main: frame={_frame} simTime={simTimeLabel} vehicles={vehicles.Count} cars={vehicles.Count} " +
                $"v0=(x={v.X:F2}, z={v.Z:F2}, yaw={v.YawRad:F3})");
        }
        else
        {
            GD.Print($"Main: frame={_frame} simTime={simTimeLabel} vehicles=0 cars=0");
        }
    }

    // "simTime" means the local sim-cadence clock (SimSource.Time) for the local transport, unchanged from
    // before T2.2b. A remote viewer has no local sim clock of its own -- the closest analogue is the
    // wire's own newest sample timestamp (IReplicationSource.LatestVehicleSampleTime) -- logged instead so
    // the dds heartbeat line still carries a meaningful time axis without pretending to be sim time.
    private string SimTimeLabel()
    {
        if (_transport == "local")
        {
            return $"{_sim?.Time ?? 0.0:F2}";
        }

        return _source?.LatestVehicleSampleTime is { } t ? $"{t:F2}(wire)" : "?";
    }

    private void DisposeSources()
    {
        _sim?.Dispose();
        _sim = null;
        _pedSim?.Dispose();
        _pedSim = null;
        _liveCitySource?.Dispose();
        _liveCitySource = null;
#if CITY3D_REMOTE
        _ddsPedSource?.Dispose();
        _ddsPedSource = null;
        _ddsSource?.Dispose();
        _ddsSource = null;
        _ddsParticipant?.Dispose();
        _ddsParticipant = null;
#endif
    }

    // Resolve the repo root by searching upward from this project's own directory for Traffic.sln --
    // never a hardcoded absolute path (CLAUDE.md prime directive: the VM mount path is not stable).
    // ProjectSettings.GlobalizePath("res://") gives the absolute path Godot loaded this project from,
    // which is demos/City3D/Viewer in a normal checkout.
    private static string FindRepoRoot()
    {
        var dir = new DirectoryInfo(ProjectSettings.GlobalizePath("res://"));
        while (dir is not null)
        {
            if (File.Exists(Path.Combine(dir.FullName, "Traffic.sln")))
            {
                return dir.FullName;
            }

            dir = dir.Parent;
        }

        throw new DirectoryNotFoundException(
            "walked up from '" + ProjectSettings.GlobalizePath("res://") + "' without finding Traffic.sln");
    }

    // docs/DEMO-CITY3D-DESIGN.md "Procedural scene generation -> Roads" / task T1.3 -- the LOCAL entry
    // point. CityLib.RoadMeshBuilder does all the math (segment normals, miter offsets, triangulation, the
    // SUMO->Godot transform) from a NetworkModel; BuildRoadMeshesFromRibbons below does the shared
    // ArrayMesh construction.
    private (Vector3 Min, Vector3 Max) BuildRoadMeshes(NetworkModel network)
        => BuildRoadMeshesFromRibbons(
            RoadMeshBuilder.BuildAll(network, includeInternal: true), network.LanesByHandle.Count);

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §6, -TASKS.md D1 -- the live-city LOCAL entry point's crop-filtered
    // counterpart to BuildRoadMeshes: only lanes with at least one shape vertex inside the crop rect
    // (expanded by a small margin so a lane that just straddles the crop edge still renders whole rather
    // than being clipped mid-ribbon) are built, keeping the legible downtown block from being lost in the
    // full net's ~4750m extent (see ReadyLiveCity's own remark). RoadMeshBuilder.Build (the same per-lane
    // ribbon math BuildAll uses internally) is called directly per surviving lane.
    private (Vector3 Min, Vector3 Max) BuildRoadMeshesCropped(
        NetworkModel network, double x0, double y0, double x1, double y1, double marginMeters = 60.0)
    {
        var minX = x0 - marginMeters;
        var minY = y0 - marginMeters;
        var maxX = x1 + marginMeters;
        var maxY = y1 + marginMeters;

        var filtered = new List<(int Handle, RibbonMesh Mesh)>();
        foreach (var lane in network.LanesByHandle)
        {
            var inside = false;
            foreach (var (sx, sy) in lane.Shape)
            {
                if (sx >= minX && sx <= maxX && sy >= minY && sy <= maxY)
                {
                    inside = true;
                    break;
                }
            }

            if (!inside)
            {
                continue;
            }

            filtered.Add((lane.Handle, RoadMeshBuilder.Build(lane.Shape, lane.ShapeZ, lane.Width)));
        }

        GD.Print(
            $"Main: --live-city crop [{x0:F0},{y0:F0}]-[{x1:F0},{y1:F0}] kept {filtered.Count} of " +
            $"{network.LanesByHandle.Count} lane(s) from the full net.");

        return BuildRoadMeshesFromRibbons(filtered, filtered.Count);
    }

#if CITY3D_REMOTE
    // docs/DEMO-CITY3D-DESIGN.md "Data path -> Remote mode" / task T2.2b -- the REMOTE entry point: builds
    // the identical ribbon meshes from the RECEIVED wire geometry (CityLib.RoadMeshBuilder's
    // geometry-dictionary overload) instead of a NetworkModel. BuildRoadMeshesFromRibbons below (the actual
    // ArrayMesh/MeshInstance3D construction) is untouched -- only the RibbonMesh SOURCE differs.
    private (Vector3 Min, Vector3 Max) BuildRoadMeshesFromGeometry(
        IReadOnlyDictionary<int, GeometryCodec.LaneGeo> geometry)
        => BuildRoadMeshesFromRibbons(RoadMeshBuilder.BuildAll(geometry, includeInternal: true), geometry.Count);
#endif

    // docs/DEMO-CITY3D-DESIGN.md "Procedural scene generation -> Roads" / task T1.3 (and T2.2b's remote
    // extension). Static geometry, built ONCE at load (design: "two clocks: static geometry ... built once
    // at load"). SHARED between local and remote: turns whatever per-lane RibbonMesh sequence the caller
    // computed (from a NetworkModel locally, from received wire geometry remotely -- see the two callers
    // above) into Godot ArrayMesh/MeshInstance3D nodes. Returns the Godot-space bounding box of every
    // emitted vertex, so the camera can frame the whole network.
    private (Vector3 Min, Vector3 Max) BuildRoadMeshesFromRibbons(
        IEnumerable<(int Handle, RibbonMesh Mesh)> perLane, int totalLaneCount)
    {
        var material = new StandardMaterial3D
        {
            AlbedoColor = AsphaltColor,
            Roughness = 0.95f,
            CullMode = BaseMaterial3D.CullModeEnum.Disabled,
        };

        var min = new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
        var max = new Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);
        var laneCount = 0;

        foreach (var (handle, mesh) in perLane)
        {
            if (mesh.Vertices.Length == 0)
            {
                // Degenerate (single-point) lane shape -- nothing to ribbon.
                continue;
            }

            var vertexCount = mesh.Vertices.Length / 3;
            var vertices = new Vector3[vertexCount];
            var normals = new Vector3[vertexCount];
            for (var i = 0; i < vertexCount; i++)
            {
                var vx = mesh.Vertices[i * 3 + 0];
                var vy = mesh.Vertices[i * 3 + 1];
                var vz = mesh.Vertices[i * 3 + 2];
                vertices[i] = new Vector3(vx, vy, vz);
                normals[i] = new Vector3(mesh.Normals[i * 3 + 0], mesh.Normals[i * 3 + 1], mesh.Normals[i * 3 + 2]);

                min.X = Mathf.Min(min.X, vx);
                min.Y = Mathf.Min(min.Y, vy);
                min.Z = Mathf.Min(min.Z, vz);
                max.X = Mathf.Max(max.X, vx);
                max.Y = Mathf.Max(max.Y, vy);
                max.Z = Mathf.Max(max.Z, vz);
            }

            var arrays = new Godot.Collections.Array();
            arrays.Resize((int)Mesh.ArrayType.Max);
            arrays[(int)Mesh.ArrayType.Vertex] = vertices;
            arrays[(int)Mesh.ArrayType.Normal] = normals;
            arrays[(int)Mesh.ArrayType.Index] = mesh.Indices;

            var arrayMesh = new ArrayMesh();
            arrayMesh.AddSurfaceFromArrays(Mesh.PrimitiveType.Triangles, arrays);

            var instance = new MeshInstance3D
            {
                Mesh = arrayMesh,
                Name = $"Lane_{handle}",
            };
            instance.SetSurfaceOverrideMaterial(0, material);
            AddChild(instance);
            laneCount++;
        }

        GD.Print($"Main: built {laneCount} road ribbon mesh(es) from {totalLaneCount} lane(s).");

        if (laneCount == 0)
        {
            // No usable geometry -- fall back to a small default box around the origin so the camera setup
            // below has something sane to frame.
            min = new Vector3(-10f, 0f, -10f);
            max = new Vector3(10f, 0f, 10f);
        }

        return (min, max);
    }

    // docs/DEMO-CITY3D-DESIGN.md "Procedural scene generation -> Buildings" / task T1.4. Static geometry,
    // built ONCE at load, same as BuildRoadMeshes. CityLib.BuildingPlacer does all the math (marching
    // each edge's centerline, the deterministic per-(edge,step,side) hash for footprint/height, the
    // SUMO->Godot transform on each box's center); this method only turns its plain BuildingBox structs
    // into ONE MultiMeshInstance3D (one unit BoxMesh, one per-instance Transform3D) -- design "Buildings":
    // "Thousands of buildings go into a MultiMeshInstance3D -- one unit-cube mesh, one per-instance
    // transform, one draw call". Returns the Godot-space bounding box of every building (a conservative,
    // yaw-independent radius per box) so the camera can frame roads AND buildings together.
    private (Vector3 Min, Vector3 Max) BuildBuildings(NetworkModel network)
    {
        var boxes = BuildingPlacer.PlaceAll(network);

        var min = new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
        var max = new Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);

        if (boxes.Count == 0)
        {
            GD.Print("Main: built 0 building(s).");
            return (min, max); // the all-infinite/all-negative-infinite range is the neutral element for CombineBbox.
        }

        var material = new StandardMaterial3D
        {
            VertexColorUseAsAlbedo = true, // per-instance MultiMesh colors (the seeded grey palette) modulate albedo
            Roughness = 0.9f,
        };

        var multiMesh = new MultiMesh
        {
            TransformFormat = MultiMesh.TransformFormatEnum.Transform3D,
            UseColors = true,
            Mesh = new BoxMesh { Size = Vector3.One }, // unit cube; per-instance transform below scales it to each box's footprint/height
            InstanceCount = boxes.Count,
        };

        for (var i = 0; i < boxes.Count; i++)
        {
            var box = boxes[i];

            // Scale the unit cube to (SizeX, SizeY, SizeZ) in its own local axes, THEN rotate the whole
            // scaled box about +Y by YawRad (Basis.Scaled multiplies as R*S, i.e. scale-then-rotate for a
            // transformed vector) so its SizeX face runs along the local road tangent.
            var scale = new Vector3(box.SizeX, box.SizeY, box.SizeZ);
            var basis = new Basis(Vector3.Up, box.YawRad).Scaled(scale);
            var origin = new Vector3(box.CX, box.CY, box.CZ);
            multiMesh.SetInstanceTransform(i, new Transform3D(basis, origin));
            multiMesh.SetInstanceColor(i, BuildingPalette[i % BuildingPalette.Length]);

            var halfX = box.SizeX / 2f;
            var halfY = box.SizeY / 2f;
            var halfZ = box.SizeZ / 2f;
            // Yaw-independent (conservative) horizontal radius -- the box's XZ diagonal half-extent, so
            // the bbox is correct regardless of which way it's rotated.
            var radiusXz = Mathf.Sqrt(halfX * halfX + halfZ * halfZ);

            min.X = Mathf.Min(min.X, box.CX - radiusXz);
            min.Y = Mathf.Min(min.Y, box.CY - halfY);
            min.Z = Mathf.Min(min.Z, box.CZ - radiusXz);
            max.X = Mathf.Max(max.X, box.CX + radiusXz);
            max.Y = Mathf.Max(max.Y, box.CY + halfY);
            max.Z = Mathf.Max(max.Z, box.CZ + radiusXz);
        }

        var instance = new MultiMeshInstance3D
        {
            Multimesh = multiMesh,
            Name = "Buildings",
            MaterialOverride = material,
        };
        AddChild(instance);

        GD.Print($"Main: built {boxes.Count} building(s).");
        return (min, max);
    }

    // docs/DEMO-CITY3D-DESIGN.md "Procedural scene generation -> Cars" / task T1.5. The ONLY per-frame
    // geometry (design: "The only per-frame geometry"). Built once here as an EMPTY MultiMesh (no vehicles
    // exist yet at load); UpdateCars (below, called from _Process every frame) writes the live transforms.
    private MultiMesh BuildCarMultiMesh()
    {
        var material = new StandardMaterial3D
        {
            VertexColorUseAsAlbedo = true, // per-instance MultiMesh colors (the seeded car palette) modulate albedo
            Roughness = 0.6f,
            Metallic = 0.2f,
        };

        var multiMesh = new MultiMesh
        {
            TransformFormat = MultiMesh.TransformFormatEnum.Transform3D,
            UseColors = true,
            Mesh = new BoxMesh { Size = Vector3.One }, // unit cube; per-instance transform below scales it to each car's real dims
            InstanceCount = 0,
        };

        var instance = new MultiMeshInstance3D
        {
            Multimesh = multiMesh,
            Name = "Cars",
            MaterialOverride = material,
        };
        AddChild(instance);

        GD.Print("Main: built car MultiMesh (0 instances at load).");
        return multiMesh;
    }

    // Called once per render frame from _Process, right after Reconstruct. Reuses the SAME MultiMesh
    // across frames (design "Cars" / task T1.5: "Reuse the MultiMesh across frames"): the underlying
    // instance buffer (InstanceCount) is only ever grown, never shrunk (growing reallocates, but this
    // frame is about to rewrite every live transform anyway, so nothing stale is ever visible); the cheap
    // VisibleInstanceCount is what actually tracks the live vehicle count frame to frame, including
    // shrinking back down when vehicles despawn. CityLib.CarTransform does all the math (pure, no Godot
    // type); this method only turns each CarInstance into a Transform3D + a deterministic per-handle color.
    private void UpdateCars(IReadOnlyList<CityLib.ReconstructedVehicle> vehicles)
    {
        if (_carMultiMesh is null)
        {
            return;
        }

        if (vehicles.Count > _carMultiMesh.InstanceCount)
        {
            _carMultiMesh.InstanceCount = vehicles.Count;
        }

        _carMultiMesh.VisibleInstanceCount = vehicles.Count;

        for (var i = 0; i < vehicles.Count; i++)
        {
            var v = vehicles[i];
            var car = CityLib.CarTransform.ForVehicle(v, CarHeightMeters);

            var scale = new Vector3(car.ScaleX, car.ScaleY, car.ScaleZ);
            var basis = new Basis(Vector3.Up, car.YawRad).Scaled(scale);
            var origin = new Vector3(car.PosX, car.PosY, car.PosZ);
            _carMultiMesh.SetInstanceTransform(i, new Transform3D(basis, origin));

            var paletteIndex = (int)(v.Handle.Index % (uint)CarPalette.Length);
            _carMultiMesh.SetInstanceColor(i, CarPalette[paletteIndex]);
        }
    }

    // docs/DEMO-CITY3D-DESIGN.md "#### Pedestrians (P7-3)" -- ONE ped MultiMeshInstance3D, the ped analog of
    // BuildCarMultiMesh: built once as an EMPTY MultiMesh (no peds at load), a unit BoxMesh scaled per
    // instance by CityLib.PedTransform into a slim upright avatar; UpdatePeds (below) writes the live
    // transforms + regime colours each frame.
    private MultiMesh BuildPedMultiMesh()
    {
        var material = new StandardMaterial3D
        {
            VertexColorUseAsAlbedo = true, // per-instance MultiMesh colours (regime slate/cyan) modulate albedo
            Roughness = 0.7f,
        };

        var multiMesh = new MultiMesh
        {
            TransformFormat = MultiMesh.TransformFormatEnum.Transform3D,
            UseColors = true,
            Mesh = new BoxMesh { Size = Vector3.One }, // unit box; per-instance transform scales it to the slim avatar dims
            InstanceCount = 0,
        };

        var instance = new MultiMeshInstance3D
        {
            Multimesh = multiMesh,
            Name = "Pedestrians",
            MaterialOverride = material,
        };
        AddChild(instance);

        GD.Print("Main: built ped MultiMesh (0 instances at load).");
        return multiMesh;
    }

    // Called once per ped frame from ProcessPeds, right after Reconstruct. Same build-once/grow-only/
    // VisibleInstanceCount discipline as UpdateCars. CityLib.PedTransform does the pure avatar math (no Godot
    // type); this method only turns each PedInstance into a Transform3D + its regime colour (slate low-power,
    // cyan high-power -- visibly distinct per the design).
    private void UpdatePeds(IReadOnlyList<CityLib.ReconstructedPed> peds)
    {
        if (_pedMultiMesh is null)
        {
            return;
        }

        if (peds.Count > _pedMultiMesh.InstanceCount)
        {
            _pedMultiMesh.InstanceCount = peds.Count;
        }

        _pedMultiMesh.VisibleInstanceCount = peds.Count;

        for (var i = 0; i < peds.Count; i++)
        {
            var inst = CityLib.PedTransform.ForPed(peds[i], PedRenderHeightMeters, PedRenderWidthMeters);

            // No yaw needed (a slim upright avatar reads fine without heading, per design) -- an axis-aligned
            // scaled box.
            var basis = Basis.Identity.Scaled(new Vector3(inst.ScaleX, inst.ScaleY, inst.ScaleZ));
            var origin = new Vector3(inst.PosX, inst.PosY, inst.PosZ);
            _pedMultiMesh.SetInstanceTransform(i, new Transform3D(basis, origin));
            _pedMultiMesh.SetInstanceColor(i, inst.IsHighPower ? PedHighPowerColor : PedLowPowerColor);
        }
    }

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §6, -TASKS.md D1 -- the ped analog of UpdateCars for the
    // `--live-city` path. Unlike UpdatePeds (which consumes CityLib.ReconstructedPed, already run through
    // CityLib.PedReconstructor's DR smoothing over a wire), a live-city ped's pose comes straight off
    // LiveCitySource.Peds (Sim.LiveCity.LiveCityPed, an in-process sample -- no wire/DR involved), so this
    // method applies CoordinateTransform.SumoToGodot itself and colors each instance by its
    // Sim.LiveCity.PedRegime (grey low-power / orange high-power / yellow paused) rather than reusing
    // CityLib.PedTransform's two-state (low/high) regime enum. Same build-once/grow-only/
    // VisibleInstanceCount discipline as UpdateCars/UpdatePeds.
    private void UpdateLivePeds(IReadOnlyList<LiveCityPed> peds)
    {
        if (_pedMultiMesh is null)
        {
            return;
        }

        if (peds.Count > _pedMultiMesh.InstanceCount)
        {
            _pedMultiMesh.InstanceCount = peds.Count;
        }

        _pedMultiMesh.VisibleInstanceCount = peds.Count;

        for (var i = 0; i < peds.Count; i++)
        {
            var p = peds[i];
            var (gx, gy, gz) = CoordinateTransform.SumoToGodot(p.X, p.Y, p.Z);

            // No yaw (matches the plaza UpdatePeds convention: a slim upright avatar reads fine without
            // heading at demo scale); center raised half the avatar height above the reconstructed ground
            // point so it stands ON the road/sidewalk rather than being bisected by it.
            var scale = new Vector3(PedRenderWidthMeters, PedRenderHeightMeters, PedRenderWidthMeters);
            var basis = Basis.Identity.Scaled(scale);
            var origin = new Vector3(gx, gy + PedRenderHeightMeters / 2f, gz);
            _pedMultiMesh.SetInstanceTransform(i, new Transform3D(basis, origin));

            var color = p.Regime switch
            {
                Sim.LiveCity.PedRegime.HighPower => LiveCityPedHighPowerColor,
                Sim.LiveCity.PedRegime.Paused => LiveCityPedPausedColor,
                _ => LiveCityPedLowPowerColor, // LowPowerWalking
            };
            _pedMultiMesh.SetInstanceColor(i, color);
        }
    }

    // Merges two Godot-space bounding boxes (component-wise min/max). An all-(+inf,+inf,+inf)/
    // (-inf,-inf,-inf) box (BuildBuildings' "no buildings" case) is the neutral element -- combining with
    // it leaves the other box untouched.
    private static (Vector3 Min, Vector3 Max) CombineBbox((Vector3 Min, Vector3 Max) a, (Vector3 Min, Vector3 Max) b)
    {
        var min = new Vector3(Mathf.Min(a.Min.X, b.Min.X), Mathf.Min(a.Min.Y, b.Min.Y), Mathf.Min(a.Min.Z, b.Min.Z));
        var max = new Vector3(Mathf.Max(a.Max.X, b.Max.X), Mathf.Max(a.Max.Y, b.Max.Y), Mathf.Max(a.Max.Z, b.Max.Z));
        return (min, max);
    }

    // docs/DEMO-CITY3D-DESIGN.md "Procedural scene generation -> Roads" desktop check framing: an
    // above-and-back angled bird's-eye view over the whole network's Godot-space XZ bounds, plus a
    // DirectionalLight3D so the asphalt is actually visible. Always built (env + light + this whole-network
    // camera) regardless of `--camera` mode -- task T1.6 Part C's `--camera=close` only leaves this camera
    // non-Current (in favour of the separate close-up camera built in _Ready) rather than forking the
    // environment/light setup, so run-smoke.sh's default (`--camera=overview`, `makeCurrent: true`) is
    // byte-identical to the pre-T1.6 behavior.
    private void BuildCameraAndLight((Vector3 Min, Vector3 Max) bbox, bool makeCurrent)
    {
        // A soft sky-coloured background + a little ambient fill light (docs/DEMO-CITY3D-DESIGN.md's
        // "Roads" only specifies the asphalt material; a bare gl_compatibility clear colour with pure
        // directional-only lighting renders dark asphalt nearly indistinguishable from the background --
        // this keeps the desktop/screenshot check legible without changing the asphalt's own colour).
        var env = new Godot.Environment
        {
            BackgroundMode = Godot.Environment.BGMode.Color,
            BackgroundColor = new Color(0.55f, 0.68f, 0.82f),
            AmbientLightSource = Godot.Environment.AmbientSource.Color,
            AmbientLightColor = new Color(0.6f, 0.62f, 0.68f),
            AmbientLightEnergy = 0.7f,
        };
        AddChild(new WorldEnvironment { Environment = env });

        var (min, max) = bbox;
        var center = new Vector3((min.X + max.X) / 2f, (min.Y + max.Y) / 2f, (min.Z + max.Z) / 2f);
        var extentX = max.X - min.X;
        var extentZ = max.Z - min.Z;
        var extent = Mathf.Max(Mathf.Max(extentX, extentZ), 10f); // floor so a tiny/degenerate net still frames sanely

        var camera = new Camera3D { Name = "FramingCamera" };
        AddChild(camera);

        var camPos = center + new Vector3(0f, extent * CameraHeightFactor, extent * CameraBackFactor);
        camera.Position = camPos;
        camera.LookAt(center, Vector3.Up);
        camera.Current = makeCurrent;
        camera.Far = Mathf.Max(extent * 4f, 500f);

        var light = new DirectionalLight3D
        {
            Name = "SunLight",
            RotationDegrees = new Vector3(-55f, -35f, 0f),
        };
        light.LightEnergy = 1.3f;
        AddChild(light);

        GD.Print($"Main: camera framing bbox min={min} max={max}, positioned at {camPos} looking at {center}.");
    }

    // docs/DEMO-CITY3D-DESIGN.md "Procedural scene generation -> Traffic lights" / task T1.6 Part B (and
    // T2.2b's remote extension). Called ONCE (lazily, from RenderFrame the first frame TlStateByLane is
    // non-empty). SHARED between local and remote: `heads` is already placed by the caller via
    // `_placeSignalHeads` (a closure over either TrafficLightPlacer.Place(NetworkModel, ...) locally or its
    // geometry-dictionary overload remotely -- CityLib/TrafficLightPlacer.cs); this method only turns each
    // plain SignalHead struct into a pole MeshInstance3D + a head MeshInstance3D with its OWN
    // StandardMaterial3D (one per head, not shared -- each head's emissive colour is rewritten
    // independently every frame in UpdateTrafficLights, unlike the shared asphalt/building/car materials
    // which never change per-instance).
    private void BuildTrafficLights(IReadOnlyList<SignalHead> heads)
    {
        var poleMaterial = new StandardMaterial3D { AlbedoColor = TlPoleColor, Roughness = 0.8f };

        foreach (var head in heads)
        {
            var poleHeight = head.HeadY - head.PoleY;
            if (poleHeight <= 0f)
            {
                poleHeight = TrafficLightPlacer.HeadHeightMeters; // defensive floor; never hit in practice
            }

            var pole = new MeshInstance3D
            {
                Mesh = new BoxMesh { Size = new Vector3(0.25f, poleHeight, 0.25f) },
                Position = new Vector3(head.PoleX, head.PoleY + poleHeight / 2f, head.PoleZ),
                Name = $"TlPole_{head.LaneHandle}",
            };
            pole.SetSurfaceOverrideMaterial(0, poleMaterial);
            AddChild(pole);

            // Small box head (design "Traffic lights": "a small box or three stacked spheres") -- its own
            // material instance so this frame's/every future frame's colour write hits only this head.
            var headMaterial = new StandardMaterial3D
            {
                AlbedoColor = TlOffColor,
                EmissionEnabled = true,
                Emission = TlOffColor,
                EmissionEnergyMultiplier = 2.5f,
            };
            var headInstance = new MeshInstance3D
            {
                Mesh = new BoxMesh { Size = new Vector3(0.9f, 0.9f, 0.9f) },
                Position = new Vector3(head.HeadX, head.HeadY, head.HeadZ),
                Name = $"TlHead_{head.LaneHandle}",
            };
            headInstance.SetSurfaceOverrideMaterial(0, headMaterial);
            AddChild(headInstance);

            _signalHeads.Add((head.LaneHandle, headInstance, headMaterial));
        }

        GD.Print($"Main: built {_signalHeads.Count} signal head(s).");
    }

    // Called every _Process frame once the signal heads exist. Design "Traffic lights": "Only the
    // material param updates per frame" -- geometry (pole+head positions) never changes after
    // BuildTrafficLights; only each head's emissive/albedo colour is rewritten from the live replicated
    // signal byte. A lane that (transiently) has no entry in `tlStateByLane` renders Off rather than
    // stale, though this cannot happen in practice -- TlStateByLane's key set is exactly the lane handles
    // BuildTrafficLights was built from.
    private void UpdateTrafficLights(IReadOnlyDictionary<int, byte> tlStateByLane)
    {
        foreach (var (laneHandle, _, material) in _signalHeads)
        {
            var color = tlStateByLane.TryGetValue(laneHandle, out var signalByte)
                ? ColorForSignal(TrafficLightPlacer.ColorFor(signalByte))
                : TlOffColor;
            material.AlbedoColor = color;
            material.Emission = color;
        }
    }

    private static Color ColorForSignal(SignalColor color) => color switch
    {
        SignalColor.Red => TlRedColor,
        SignalColor.Yellow => TlYellowColor,
        SignalColor.Green => TlGreenColor,
        _ => TlOffColor,
    };

    // Task T1.6 Part C -- the `--camera=close` point of interest, recomputed every frame (cheap: at most a
    // handful of signal heads / one vehicle) per this task's priority order: "the centroid of the signal
    // heads if any, else the first vehicle, else network center". Signal heads (once built) are STATIC, so
    // once any exist they win permanently for this scenario; only a TL-free scenario ever falls through to
    // the vehicle/network-center branches.
    private void UpdateCloseCameraFraming(Camera3D camera, IReadOnlyList<CityLib.ReconstructedVehicle>? vehicles)
    {
        Vector3 focus;
        if (_signalHeads.Count > 0)
        {
            var sum = Vector3.Zero;
            foreach (var head in _signalHeads)
            {
                sum += head.HeadInstance.Position;
            }

            focus = sum / _signalHeads.Count;
        }
        else if (vehicles is { Count: > 0 })
        {
            var v = vehicles[0];
            focus = new Vector3(v.X, v.Y, v.Z);
        }
        else
        {
            var (min, max) = _sceneBbox;
            focus = new Vector3((min.X + max.X) / 2f, (min.Y + max.Y) / 2f, (min.Z + max.Z) / 2f);
        }

        // Fixed (not extent-scaled) low-angled offset -- design Part C: "~40-80m back and ~15-25m up,
        // tilted down". Offset ALONG the approach's own back-of-travel direction (not a fixed world axis):
        // BuildingPlacer lines BOTH sides of every road with boxes almost the corridor's whole length, so a
        // sideways (across-the-road) offset looks straight through that building wall; sitting back along
        // the road's own corridor instead keeps the camera IN the street canyon, looking down it at the
        // light/car -- buildings flank the shot instead of blocking it.
        var backDir = ComputeCloseCameraBackDirection(vehicles);
        var lateralDir = new Vector3(-backDir.Z, 0f, backDir.X); // horizontal perpendicular to backDir
        camera.Position = focus
            + backDir * CloseCameraBackMeters
            + lateralDir * CloseCameraLateralMeters
            + new Vector3(0f, CloseCameraUpMeters, 0f);
        camera.LookAt(focus, Vector3.Up);
    }

    // The horizontal unit direction the close camera sits back along, i.e. opposite the approach's own
    // travel direction. Prefers the first (built, static) signal head's own controlled lane -- its final
    // segment tangent IS the approach direction by definition -- falling back to the first live vehicle's
    // reconstructed yaw (same navi-heading -> Godot-forward relationship CoordinateTransform documents:
    // forward = (-sin(yaw), 0, -cos(yaw))) when no TL exists yet, and finally a fixed default so the very
    // first (pre-tick) frame still gets a sane (if provisional) camera placement.
    private Vector3 ComputeCloseCameraBackDirection(IReadOnlyList<CityLib.ReconstructedVehicle>? vehicles)
    {
        if (_signalHeads.Count > 0 && _sim is not null)
        {
            var laneHandle = _signalHeads[0].LaneHandle;
            var lanes = _sim.Network.LanesByHandle;
            if (laneHandle >= 0 && laneHandle < lanes.Count)
            {
                var shape = lanes[laneHandle].Shape;
                if (shape.Count >= 2)
                {
                    var (x1, y1) = shape[^2];
                    var (x2, y2) = shape[^1];
                    var dx = x2 - x1;
                    var dy = y2 - y1;
                    var len = Math.Sqrt(dx * dx + dy * dy);
                    if (len > 1e-9)
                    {
                        // Travel direction in Godot XZ (CoordinateTransform.SumoToGodot: Godot.X = Sumo.X,
                        // Godot.Z = -Sumo.Y) -- "back" is its negation.
                        var forwardX = (float)(dx / len);
                        var forwardZ = (float)(-dy / len);
                        return new Vector3(-forwardX, 0f, -forwardZ);
                    }
                }
            }
        }

        if (vehicles is { Count: > 0 })
        {
            var yaw = vehicles[0].YawRad;
            return new Vector3(MathF.Sin(yaw), 0f, MathF.Cos(yaw));
        }

        return new Vector3(0f, 0f, 1f);
    }

    // `--camera=<mode>` USER cmdline arg (task T1.6 Part C): "overview" (default, the whole-network
    // bird's-eye framing -- unchanged run-smoke.sh behavior) or "close" (a low, angled, real-scale
    // close-up). Same OS.GetCmdlineUserArgs() mechanism as --shot/--scenario; an unrecognized value falls
    // back to the default rather than failing the run.
    private static string ParseCameraArg()
    {
        const string prefix = "--camera=";
        foreach (var arg in OS.GetCmdlineUserArgs())
        {
            if (arg.StartsWith(prefix, StringComparison.Ordinal))
            {
                var v = arg[prefix.Length..].Trim().ToLowerInvariant();
                if (v == "close")
                {
                    return "close";
                }

                if (v == "overview")
                {
                    return "overview";
                }
            }
        }

        return "overview";
    }

    // docs/DEMO-CITY3D-DESIGN.md task T1.3 part C -- Xvfb screenshot pipeline. `--shot=<abs path>` is a
    // USER cmdline arg (everything after Godot's own `--`), read via OS.GetCmdlineUserArgs() so it never
    // collides with Godot's own switches (--headless, --rendering-driver, ...).
    private static string? ParseShotArg()
    {
        const string prefix = "--shot=";
        foreach (var arg in OS.GetCmdlineUserArgs())
        {
            if (arg.StartsWith(prefix, StringComparison.Ordinal))
            {
                return arg[prefix.Length..];
            }
        }

        return null;
    }

    // `--scenario=<rel>` USER cmdline arg (path under scenarios/); defaults to the tiny signalized
    // dev scenario. Same OS.GetCmdlineUserArgs() mechanism as --shot.
    private static string ParseScenarioArg()
    {
        const string prefix = "--scenario=";
        foreach (var arg in OS.GetCmdlineUserArgs())
        {
            if (arg.StartsWith(prefix, StringComparison.Ordinal))
            {
                var v = arg[prefix.Length..].Trim();
                if (v.Length > 0)
                {
                    return v;
                }
            }
        }

        return "09-traffic-light";
    }

    // docs/DEMO-CITY3D-DESIGN.md "#### Pedestrians (P7-3)" -- the `--peds` USER flag (a bare flag, no `=`),
    // parsed via the same OS.GetCmdlineUserArgs() mechanism as --scenario/--camera/--shot. When present the
    // viewer takes the dedicated ped path (poc0-crossing-plaza roads + ped server-sim/wire/render), skipping
    // the vehicle SimSource entirely; absent, behaviour is byte-identical to before.
    private static bool ParsePedsArg()
    {
        foreach (var arg in OS.GetCmdlineUserArgs())
        {
            if (arg == "--peds")
            {
                return true;
            }
        }

        return false;
    }

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §6, -TASKS.md D1 -- the `--live-city` USER flag (a bare flag, no
    // `=`), parsed via the same OS.GetCmdlineUserArgs() mechanism as --peds/--scenario/--camera/--shot.
    // When present the viewer takes the dedicated live-city path (ReadyLiveCity/ProcessLiveCity): the
    // coupled cars+peds+crossing-yield scene over scenarios/_ped/demo_city/box, rendered together in one
    // scene; absent, behaviour is byte-identical to before.
    private static bool ParseLiveCityArg()
    {
        foreach (var arg in OS.GetCmdlineUserArgs())
        {
            if (arg == "--live-city")
            {
                return true;
            }
        }

        return false;
    }

    // `--transport=<local|dds>` USER cmdline arg (task T2.2b): default "local" (today's in-process
    // behavior, byte-for-byte unchanged). "dds" only actually works in a build compiled with
    // -p:City3DRemote=true (see the non-remote-build guard in _Ready) -- same OS.GetCmdlineUserArgs()
    // mechanism as --scenario/--camera/--shot.
    private static string ParseTransportArg()
    {
        const string prefix = "--transport=";
        foreach (var arg in OS.GetCmdlineUserArgs())
        {
            if (arg.StartsWith(prefix, StringComparison.Ordinal))
            {
                var v = arg[prefix.Length..].Trim().ToLowerInvariant();
                if (v is "local" or "dds")
                {
                    return v;
                }
            }
        }

        return "local";
    }

    // `--shot-delay=<seconds>` USER cmdline arg: extra REAL wall-clock seconds to wait, after the scene is
    // built, before capturing (task T1.5 part C caveat: DrClock/the sim-cadence accumulator are both
    // wall-clock-driven, so an instant capture -- the pre-T1.5 default, still what omitting this arg gives
    // you -- barely advances sim time and cars cluster right at their insertion points; letting real time
    // pass first gives the sim ticks + reconstruction smoother room to actually spread vehicles across the
    // network). Defaults to 0 (the original instant-capture behavior screenshot.sh/T1.3 still rely on).
    private static double ParseShotDelayArg()
    {
        const string prefix = "--shot-delay=";
        foreach (var arg in OS.GetCmdlineUserArgs())
        {
            if (arg.StartsWith(prefix, StringComparison.Ordinal)
                && double.TryParse(arg[prefix.Length..], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var seconds)
                && seconds > 0.0)
            {
                return seconds;
            }
        }

        return 0.0;
    }

    // Task T3.1 -- one `--channel=...` arg, parsed. Either `HasScreen` (the explicit off-axis corners form,
    // `screen=ax,ay,az,bx,by,bz,cx,cy,cz`) or the offset+look-angles+FOV form (`look=`/`fov=`) is populated;
    // `Off` (the eye position) and `Near`/`Far` apply to both forms.
    private sealed record ChannelConfig(
        double EyeX, double EyeY, double EyeZ,
        bool HasScreen,
        double PaX, double PaY, double PaZ,
        double PbX, double PbY, double PbZ,
        double PcX, double PcY, double PcZ,
        double YawDeg, double PitchDeg, double FovDeg,
        double Near, double Far);

    // `--channel="off=x,y,z;look=yaw,pitch;fov=60"` and/or
    // `--channel="off=x,y,z;screen=ax,ay,az,bx,by,bz,cx,cy,cz"` (repeatable -- one video-wall channel per
    // occurrence). Same OS.GetCmdlineUserArgs() mechanism as --scenario/--camera/--shot; a malformed or
    // incomplete channel (missing the required `off=`) is REPORTED and skipped rather than failing the
    // whole run, so one typo'd channel doesn't take down an otherwise-good wall.
    private static List<ChannelConfig> ParseChannelArgs()
    {
        var result = new List<ChannelConfig>();
        const string prefix = "--channel=";

        foreach (var arg in OS.GetCmdlineUserArgs())
        {
            if (!arg.StartsWith(prefix, StringComparison.Ordinal))
            {
                continue;
            }

            var spec = arg[prefix.Length..].Trim();
            if (spec.Length >= 2 && spec[0] == '"' && spec[^1] == '"')
            {
                spec = spec[1..^1]; // tolerate a shell/quoting layer leaving the quotes in place
            }

            var fields = new Dictionary<string, string>();
            foreach (var part in spec.Split(';', StringSplitOptions.RemoveEmptyEntries))
            {
                var eq = part.IndexOf('=');
                if (eq <= 0)
                {
                    continue;
                }

                fields[part[..eq].Trim().ToLowerInvariant()] = part[(eq + 1)..].Trim();
            }

            if (!fields.TryGetValue("off", out var offStr) || !TryParseTriple(offStr, out var off))
            {
                GD.PrintErr($"Main: --channel '{arg}' missing/invalid required 'off=x,y,z'; skipping this channel.");
                continue;
            }

            var near = fields.TryGetValue("near", out var nearStr) && TryParseDouble(nearStr, out var nearVal)
                ? nearVal
                : OffAxisFrustum.DefaultNear;
            var far = fields.TryGetValue("far", out var farStr) && TryParseDouble(farStr, out var farVal)
                ? farVal
                : OffAxisFrustum.DefaultFar;

            if (fields.TryGetValue("screen", out var screenStr))
            {
                if (!TryParseNine(screenStr, out var s))
                {
                    GD.PrintErr($"Main: --channel '{arg}' has an invalid 'screen=...' (need 9 comma-separated numbers: ax,ay,az,bx,by,bz,cx,cy,cz); skipping this channel.");
                    continue;
                }

                result.Add(new ChannelConfig(
                    off.X, off.Y, off.Z, HasScreen: true,
                    s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8],
                    YawDeg: 0, PitchDeg: 0, FovDeg: 0, near, far));
                continue;
            }

            var look = fields.TryGetValue("look", out var lookStr) && TryParsePair(lookStr, out var lookPair)
                ? lookPair
                : (Yaw: 0.0, Pitch: 0.0);
            var fov = fields.TryGetValue("fov", out var fovStr) && TryParseDouble(fovStr, out var fovVal)
                ? fovVal
                : 60.0;

            result.Add(new ChannelConfig(
                off.X, off.Y, off.Z, HasScreen: false,
                0, 0, 0, 0, 0, 0, 0, 0, 0,
                look.Yaw, look.Pitch, fov, near, far));
        }

        return result;
    }

    private static bool TryParseDouble(string s, out double value)
        => double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out value);

    private static bool TryParseTriple(string s, out (double X, double Y, double Z) value)
    {
        value = default;
        var parts = s.Split(',');
        if (parts.Length != 3
            || !TryParseDouble(parts[0], out var x)
            || !TryParseDouble(parts[1], out var y)
            || !TryParseDouble(parts[2], out var z))
        {
            return false;
        }

        value = (x, y, z);
        return true;
    }

    private static bool TryParsePair(string s, out (double Yaw, double Pitch) value)
    {
        value = default;
        var parts = s.Split(',');
        if (parts.Length != 2 || !TryParseDouble(parts[0], out var a) || !TryParseDouble(parts[1], out var b))
        {
            return false;
        }

        value = (a, b);
        return true;
    }

    private static bool TryParseNine(string s, out double[] value)
    {
        value = Array.Empty<double>();
        var parts = s.Split(',');
        if (parts.Length != 9)
        {
            return false;
        }

        var result = new double[9];
        for (var i = 0; i < 9; i++)
        {
            if (!TryParseDouble(parts[i], out result[i]))
            {
                return false;
            }
        }

        value = result;
        return true;
    }

    // Task T3.1 Part B -- the video-wall renderer. ONE shared scene (already built by the time this is
    // called) + ONE shared replication source (_sim, untouched) is rendered by N channels: each channel is
    // a `SubViewport` (default `OwnWorld3D = false`, so it inherits the SAME World3D as Main's own
    // (root-window) viewport -- i.e. the SubViewport shows the identical roads/buildings/cars/lights
    // without a second copy of anything) holding its own `Camera3D`, whose position/orientation/asymmetric
    // projection come straight from an `OffAxisFrustum.FrustumSpec`. The N tiles are then composited into
    // ONE window via a `CanvasLayer` of `TextureRect`s reading each SubViewport's texture, laid out in a
    // horizontal strip left-to-right -- so a single screenshot of the root viewport shows the whole wall
    // (this is also, not coincidentally, exactly a single wide-desktop wall). Real per-monitor OS `Window`s
    // are a desktop-only variant of this same per-channel Camera3D/FrustumSpec math; this headless-safe
    // strip is the one actually exercised/screenshotted here.
    private void BuildVideoWallChannels(List<ChannelConfig> channels)
    {
        var canvas = new CanvasLayer { Name = "VideoWall" };
        AddChild(canvas);

        var xOffset = 0;
        for (var i = 0; i < channels.Count; i++)
        {
            var cfg = channels[i];
            OffAxisFrustum.FrustumSpec spec;
            int tileWidth;
            const int tileHeight = WallTileHeightPx;

            if (cfg.HasScreen)
            {
                var pe = (cfg.EyeX, cfg.EyeY, cfg.EyeZ);
                var pa = (cfg.PaX, cfg.PaY, cfg.PaZ);
                var pb = (cfg.PbX, cfg.PbY, cfg.PbZ);
                var pc = (cfg.PcX, cfg.PcY, cfg.PcZ);
                spec = OffAxisFrustum.OffAxis(pe, pa, pb, pc, cfg.Near, cfg.Far);
                var aspect = (spec.Right - spec.Left) / (spec.Top - spec.Bottom);
                tileWidth = Math.Max(1, (int)Math.Round(tileHeight * aspect));
            }
            else
            {
                var (right, up, normal) = BasisFromYawPitch(cfg.YawDeg, cfg.PitchDeg);
                spec = OffAxisFrustum.SymmetricFromFov(
                    cfg.FovDeg, WallDefaultAspect, cfg.Near, cfg.Far,
                    eye: (cfg.EyeX, cfg.EyeY, cfg.EyeZ), right, up, normal);
                tileWidth = Math.Max(1, (int)Math.Round(tileHeight * WallDefaultAspect));
            }

            var subViewport = new SubViewport
            {
                Name = $"WallChannel{i}",
                Size = new Vector2I(tileWidth, tileHeight),
                RenderTargetUpdateMode = SubViewport.UpdateMode.Always,
                HandleInputLocally = false,
                // OwnWorld3D defaults to false -- deliberately left unset: this SubViewport must inherit
                // Main's World3D (walked up via Viewport.FindWorld3D) so it renders the SAME shared scene,
                // not a second private copy of it (design: "ONE shared 3D scene").
            };
            AddChild(subViewport);

            var camera = new Camera3D
            {
                Name = $"WallCamera{i}",
                Current = true,
                Position = new Vector3((float)spec.EyeX, (float)spec.EyeY, (float)spec.EyeZ),
                Basis = new Basis(
                    new Vector3((float)spec.RightX, (float)spec.RightY, (float)spec.RightZ),
                    new Vector3((float)spec.UpX, (float)spec.UpY, (float)spec.UpZ),
                    new Vector3((float)spec.NormalX, (float)spec.NormalY, (float)spec.NormalZ)),
            };
            subViewport.AddChild(camera);

            // Camera3D.SetFrustum(size, offset, zNear, zFar) mapping (verified against Godot's
            // core/math/projection.cpp Projection::set_frustum(size, aspect, offset, near, far,
            // flip_fov=false), which the engine calls with `aspect = viewport_size.aspect()` (this
            // SubViewport's OWN pixel size, hence tileWidth/tileHeight are chosen to already match `spec`'s
            // aspect above) and the default KeepAspect=KEEP_HEIGHT (flip_fov=false):
            //   effective width  = size * aspect, effective height = size
            //   left/right  = +-width/2  + offset.x  =>  size = Top-Bottom, offset.x = (Left+Right)/2
            //   bottom/top  = +-height/2 + offset.y  =>                    offset.y = (Bottom+Top)/2
            // i.e. size = the near-plane's VERTICAL extent, offset = the near-plane rect's CENTER -- an
            // asymmetric (off-axis) rect is exactly "a centered symmetric rect of that size, recentered by
            // offset", which is what SetFrustum's (size, offset) pair encodes once the SubViewport's own
            // aspect ratio matches (Right-Left)/(Top-Bottom).
            var size = (float)(spec.Top - spec.Bottom);
            var offset = new Vector2((float)((spec.Left + spec.Right) / 2.0), (float)((spec.Bottom + spec.Top) / 2.0));
            camera.SetFrustum(size, offset, (float)spec.Near, (float)spec.Far);

            var textureRect = new TextureRect
            {
                Name = $"WallView{i}",
                Texture = subViewport.GetTexture(),
                Position = new Vector2(xOffset, 0),
                Size = new Vector2(tileWidth, tileHeight),
            };
            canvas.AddChild(textureRect);

            GD.Print(
                $"Main: video-wall channel {i}: eye=({spec.EyeX:F1},{spec.EyeY:F1},{spec.EyeZ:F1}) " +
                $"L={spec.Left:F3} R={spec.Right:F3} B={spec.Bottom:F3} T={spec.Top:F3} " +
                $"tile={tileWidth}x{tileHeight} at x={xOffset}.");

            xOffset += tileWidth;
        }
    }

    // `look=yawDeg,pitchDeg` -> the (Right, Up, Normal) axis triple OffAxisFrustum.SymmetricFromFov expects.
    // yaw is a rotation about world +Y (0 = looking down -Z, matching Godot's own default Node3D
    // orientation); pitch tilts up (+) / down (-) from there. Normal = -Forward per FrustumSpec's own
    // convention (the screen's toward-the-eye normal, not the look direction -- see OffAxisFrustum.cs).
    private static (
        (double X, double Y, double Z) Right,
        (double X, double Y, double Z) Up,
        (double X, double Y, double Z) Normal) BasisFromYawPitch(double yawDeg, double pitchDeg)
    {
        var yawRad = yawDeg * Math.PI / 180.0;
        var pitchRad = pitchDeg * Math.PI / 180.0;
        var forward = (
            X: -Math.Sin(yawRad) * Math.Cos(pitchRad),
            Y: Math.Sin(pitchRad),
            Z: -Math.Cos(yawRad) * Math.Cos(pitchRad));

        var worldUp = (X: 0.0, Y: 1.0, Z: 0.0);
        var right = CrossNormalized(forward, worldUp);
        if (right is null)
        {
            // Forward is (near-)parallel to world up (looking straight up/down) -- fall back to a
            // yaw-only reference so Right is still well-defined instead of NaN.
            var fallbackUp = (X: 0.0, Y: 0.0, Z: -1.0);
            right = CrossNormalized(forward, fallbackUp) ?? (1.0, 0.0, 0.0);
        }

        var r = right.Value;
        var up = (
            X: r.Y * forward.Z - r.Z * forward.Y,
            Y: r.Z * forward.X - r.X * forward.Z,
            Z: r.X * forward.Y - r.Y * forward.X);
        var normal = (X: -forward.X, Y: -forward.Y, Z: -forward.Z);

        return (r, up, normal);
    }

    private static (double X, double Y, double Z)? CrossNormalized(
        (double X, double Y, double Z) a, (double X, double Y, double Z) b)
    {
        var cx = a.Y * b.Z - a.Z * b.Y;
        var cy = a.Z * b.X - a.X * b.Z;
        var cz = a.X * b.Y - a.Y * b.X;
        var len = Math.Sqrt(cx * cx + cy * cy + cz * cz);
        return len < 1e-9 ? null : (cx / len, cy / len, cz / len);
    }

    // Waits `delaySeconds` of real wall-clock time (a no-op when 0, the default), then a couple more real
    // rendered frames (so the just-built meshes/camera/light -- and, after the delay, however many cars
    // have accumulated -- are actually on screen), then saves the viewport to PNG and quits. Guarded for
    // headless (no rendering driver, e.g. a plain `--headless` run under run-smoke.sh):
    // GetViewport().GetTexture().GetImage() returns null (or throws) when there is no real framebuffer, in
    // which case this reports the gap instead of crashing.
    private async void CaptureScreenshotAsync(string path, double delaySeconds = 0.0)
    {
        try
        {
            if (delaySeconds > 0.0)
            {
                await ToSignal(GetTree().CreateTimer(delaySeconds), SceneTreeTimer.SignalName.Timeout);
            }

            for (var i = 0; i < 3; i++)
            {
                await ToSignal(RenderingServer.Singleton, RenderingServer.SignalName.FramePostDraw);
            }

            var viewport = GetViewport();
            var texture = viewport?.GetTexture();
            var image = texture?.GetImage();
            if (image is null)
            {
                GD.PrintErr("Main: --shot requested but no viewport image is available (headless/no rendering driver) -- skipping capture.");
                GetTree().Quit(1);
                return;
            }

            var dir = Path.GetDirectoryName(path);
            if (!string.IsNullOrEmpty(dir))
            {
                Directory.CreateDirectory(dir);
            }

            var err = image.SavePng(path);
            if (err != Error.Ok)
            {
                GD.PrintErr($"Main: failed to save screenshot to '{path}': {err}");
                GetTree().Quit(1);
                return;
            }

            GD.Print($"Main: screenshot saved to '{path}' ({image.GetWidth()}x{image.GetHeight()}).");
            GetTree().Quit();
        }
        catch (Exception ex)
        {
            GD.PrintErr($"Main: screenshot capture failed: {ex}");
            GetTree().Quit(1);
        }
    }
}
