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
using Sim.Replication.Recording;
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

    // docs/LIVE-CITY-VISUALS-NOTES.md "Sidewalks" row / DESIGN-live-city-2d-viz.md §2 layer 2b -- a
    // pedestrian-only lane (Sim.Ingest.Lane.AllowsRoadVehicle == false) renders as a lighter "concrete"
    // band, matching the reference renderer's own `#8a8f99` (138,143,153) vs `#4a4d55` car-asphalt hex
    // exactly (normalized to 0..1), so the two viewers read the same footpath-vs-road contrast even though
    // this 3D path's own AsphaltColor above is a much darker absolute tone than the reference's.
    private static readonly Color ConcreteColor = new(138f / 255f, 143f / 255f, 153f / 255f);

    // docs/LIVE-CITY-VISUALS-NOTES.md "Lane markings" / "Crosswalk zebra" rows -- unshaded near-white paint
    // for both the seam dashes (CityLib.LaneMarkingBuilder) and the zebra stripes (CityLib.CrosswalkBuilder)
    // so they read as bright road paint regardless of the scene's DirectionalLight3D angle, matching the
    // reference renderer's own near-opaque white strokes (`rgba(255,255,255,0.85)` / `rgba(255,255,255,0.6)`).
    private static readonly Color RoadPaintColor = new(0.92f, 0.92f, 0.94f);

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

    // docs/LIVE-CITY-VISUALS-NOTES.md deliverable 2 -- district ground-tint palette, matching the reference
    // renderer's ZONE_FILL table (docs/reference/live-city-viz/renderer/templates/template.js:93-105) by
    // hue: downtown neutral grey, retail amber, dining pink, residential blue, park green, arterial faint
    // grey. Alpha is bumped from the reference's raw canvas-overlay values (0.05-0.14) into the ~0.18-0.25
    // band the task calls for -- a flat unlit 3D ground plane at overview-camera range reads much fainter
    // than the same alpha painted directly onto a 2D canvas, so a straight channel copy would be nearly
    // invisible here. Arterial is kept the faintest of the six (matches the reference's own relative
    // ordering: it is described as "faint grey" there too), unknown zone types fall back to ZoneFillDefault.
    private static readonly Dictionary<string, Color> ZoneFillPalette = new(StringComparer.Ordinal)
    {
        ["downtown"] = new Color(148f / 255f, 163f / 255f, 184f / 255f, 0.20f),
        ["retail"] = new Color(245f / 255f, 158f / 255f, 11f / 255f, 0.22f),
        ["dining"] = new Color(244f / 255f, 114f / 255f, 182f / 255f, 0.22f),
        ["residential"] = new Color(96f / 255f, 165f / 255f, 250f / 255f, 0.20f),
        ["park"] = new Color(34f / 255f, 197f / 255f, 94f / 255f, 0.22f),
        ["arterial"] = new Color(148f / 255f, 163f / 255f, 184f / 255f, 0.10f),
    };

    private static readonly Color ZoneFillDefault = new(156f / 255f, 163f / 255f, 175f / 255f, 0.15f);

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

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §5, -TASKS.md D4 -- the click-to-identify highlight ring/label
    // colour (a distinct cyan, unused by any car/ped/TL palette above so a selected vehicle never blends
    // into its own body colour) and the pick radius in SCREEN pixels (mirrors LiveCityOverlay's world-space
    // `DefaultPickRadius`, just in screen units since Main.cs has no physics colliders to ray-pick against
    // -- design §5: "camera ray -> nearest instance").
    private static readonly Color SelectionColor = new(0.20f, 0.85f, 0.95f);
    private const float PickPixelRadius = 24f;

    // docs/LIVE-CITY-VIEWERS-TASKS.md D1 -- the coupled sim's own tick length (Sim.LiveCity.LiveCityConfig.Dt
    // default). The live-city accumulator advances LiveCitySource.Tick() in this many whole increments,
    // exactly like AdvanceLocalSim/ProcessPeds' own fixed sim-cadence accumulators, just on ITS OWN fields
    // (design §6: "the shared per-frame accumulator/frame counter are split per-domain").
    private const double LiveCityTickSeconds = 0.5;

    // Half-extent (metres) of the tight box the `--live-city` OVERVIEW camera frames, centred on the crop
    // (see ReadyLiveCity's remark): small enough that cars/peds are legible in a fixed-resolution
    // screenshot, large enough to still show several intersections' worth of the coupled scene.
    //
    // Visibility polish (docs/LIVE-CITY-VIEWERS-TASKS.md Stage D "visibility polish"): the original 180m
    // half-extent (360m box) put a 4.5m car at a few dozen pixels and a person at sub-pixel scale in a
    // fixed-resolution screenshot. Halved to a ~180m box (a "mid-zoom that frames ~150-200m of the crop",
    // the task's stated alternative to switching the default `--camera` mode itself) -- `--camera=close`/
    // `--camera=overview` both keep working unchanged, this only tightens what "overview" frames.
    private const float LiveCityFrameHalfExtentMeters = 90f;

    // Visibility polish (docs/LIVE-CITY-VIEWERS-TASKS.md Stage D) -- the live-city overview camera's own
    // (heightFactor, backFactor) pair, LOWER/more-oblique than the whole-network CameraHeightFactor/
    // CameraBackFactor above (1.1/0.9, a near-vertical "satellite" angle where a true-scale car/ped is a
    // sub-pixel dot regardless of zoom): a lower angle actually shows their silhouette height, while still
    // framing the WHOLE tightened crop box (LiveCityFrameHalfExtentMeters), unlike `--camera=close` (which
    // tracks a single vehicle/signal up close, not the crowd as a whole).
    private const float LiveCityCameraHeightFactor = 0.55f;
    private const float LiveCityCameraBackFactor = 1.15f;

    // Viewer-only LEGIBILITY render scale for the ped avatars (docs/DEMO-CITY3D-DESIGN.md "#### Pedestrians
    // (P7-3)"): the tested CityLib.PedTransform keeps the true ~0.5x1.8 m avatar, but at plaza camera range a
    // 0.5 m figure is barely a pixel, so the Viewer draws a scaled-up slim avatar purely for on-screen
    // legibility -- the exact same idea src/Sim.Viewer/RemotePedOverlay.cs uses (it renders peds at a 1.2 m
    // disc vs the 0.3 m physical radius). Proportions stay slim/upright so it still reads as a pedestrian.
    //
    // Visibility polish (docs/LIVE-CITY-VIEWERS-TASKS.md Stage D): bumped further (3.2->4.0m tall,
    // 1.1->1.6m wide) and -- see BuildPedMultiMesh -- given a CylinderMesh instead of a BoxMesh, so the
    // `--live-city` crowd (both this and the plaza `--peds` path share one MultiMesh builder) reads as a
    // crowd of upright figures instead of a scatter of near-invisible slivers at the tighter live-city
    // camera range (see LiveCityFrameHalfExtentMeters above) as well as at plaza range.
    private const float PedRenderHeightMeters = 4.0f;
    private const float PedRenderWidthMeters = 1.6f;

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

    // Interactive camera controller (docs/LIVE-CITY-VIEWERS-DESIGN.md camera-controller deliverable) --
    // input sensitivities + the click-vs-drag disambiguation threshold. Orbit/pan are proportional to
    // pixel motion (Godot InputEventMouseMotion.Relative); pan is additionally scaled by the controller's
    // CURRENT Distance (below) so the pan speed feels consistent whether zoomed in close or far out.
    private const float OrbitYawRadPerPixel = 0.006f;
    private const float OrbitPitchRadPerPixel = 0.006f;
    private const float PanMetersPerPixelPerDistance = 0.0016f;
    private const float ZoomStepFactor = 0.9f; // one wheel notch: *0.9 to zoom in, /0.9 to zoom out
    private const float ClickDragThresholdPixels = 5f;

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

    // Interactive camera controller (docs/LIVE-CITY-VIEWERS-DESIGN.md camera-controller deliverable):
    // `--camera=overview|close` (above) picks the INITIAL framing, exactly as before; `_orbitController`
    // (CityLib, Godot-free pure math) then owns the camera's pose every frame from there on, and
    // `_orbitCamera` is whichever Camera3D node it drives -- the overview `FramingCamera` normally, or
    // `_closeCamera` when `--camera=close` was requested. Left null in video-wall mode (N simultaneous
    // Current cameras -- an orbit model over "the" camera doesn't apply there, out of scope for this
    // deliverable) and stays inert with no input wired up in headless `--shot` runs (no InputEvent ever
    // fires under Xvfb without a real user), so `--shot` renders the untouched initial framing either way.
    private OrbitCameraController? _orbitController;
    private Camera3D? _orbitCamera;

    // Far-plane tracking (bug fix: the far plane used to be fixed at setup time -- roughly extent*4/500m
    // for the overview camera or a flat 2000m for the close camera -- so dollying out past it clipped the
    // road grid/cars into nothing). `_orbitBaseFar` is that ORIGINAL computed far distance (never shrunk
    // below -- it's already sized to comfortably frame the initial view), and `_orbitSceneExtent` is the
    // scene bbox's horizontal half-extent the camera was framed against (same quantity BuildCameraAndLight
    // computes internally). Every ApplyOrbitCamera call recomputes Far as
    // `Max(_orbitBaseFar, Distance * 2.5 + _orbitSceneExtent)`: the `distance * 2.5` term keeps the far
    // plane comfortably beyond the camera regardless of orbit angle (2.5x covers the camera-to-focus
    // distance plus enough slack for the focus-to-far-edge-of-scene distance from any yaw/pitch), and
    // `+ sceneExtent` covers geometry that extends past the focus point in the direction the camera is
    // looking. Both fields are set once in SetupOrbitCamera.
    private float _orbitBaseFar;
    private float _orbitSceneExtent;

    // Left-button-down/up drag-vs-click disambiguation (task requirement: "left-click *without drag* must
    // still pick a vehicle... < 5px -> click -> pick, else it was an orbit drag -- consume it, no pick").
    // `_leftButtonDown` doubles as "is the mouse currently orbiting" for InputEventMouseMotion; Shift+left
    // or a middle-button drag pans instead (`_middleButtonDown`).
    private Vector2? _leftButtonDownPos;
    private bool _leftButtonDown;
    private bool _middleButtonDown;

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

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §7, -TASKS.md Stage E (E4) -- true only for the combined
    // `--live-city --transport=dds` path (ReadyLiveCityRemote): distinguishes it from the plain vehicle
    // remote (ReadyRemote, which ALSO sets `_ddsSource`) and the plaza ped remote (ReadyPeds' dds branch,
    // which ALSO sets `_ddsPedSource`) so ProcessLiveCity can dispatch to ProcessLiveCityRemote without
    // guessing from field nullity alone (a bare `_liveCity` flag is not enough since `_transport` also
    // varies, and `_ddsSource is not null` alone is ambiguous with ReadyRemote's own non-live-city path).
    private bool _liveCityRemote;
#endif

    // Task T1.6 Part B -- built LAZILY on the first _Process frame `sim.Source.TlStateByLane` is
    // non-empty (design: geometry+TL state only exist once the bus has been pumped at least once), then
    // reused every frame after -- only each head's material colour is rewritten per frame (design
    // "Traffic lights": "Only the material param updates per frame"), same "build once, mutate per frame"
    // split RoadMeshBuilder (static) / BuildCarMultiMesh+UpdateCars (per frame) already establish.
    private readonly List<(int LaneHandle, MeshInstance3D HeadInstance, StandardMaterial3D HeadMaterial)> _signalHeads = new();
    private bool _signalHeadsBuilt;

    // Buildings visibility toggle (`--hide-buildings` startup flag + runtime `B` key). Only `ReadyLocal`'s
    // `--scenario` path actually calls BuildBuildings (procedural BuildingPlacer needs a NetworkModel's
    // edge/lane grouping; `--live-city`/`--peds`/remote don't build any buildings node at all today -- see
    // BuildBuildings/BuildRemoteScene's own remarks) -- `_buildingsNode` stays null in every other mode, so
    // the toggle is a harmless no-op there.
    private MultiMeshInstance3D? _buildingsNode;

    // docs/LIVE-CITY-VISUALS-NOTES.md deliverable 2: zone-tint visibility toggle, same shape as
    // `_buildingsNode`/`B` above, except zones default HIDDEN (owner decision: opt-in, not the default
    // wash) -- BuildZoneGround starts `_zonesNode` invisible unconditionally; `--show-zones` startup flag
    // + runtime `Z` key are the only ways to make it visible. Built by BuildZoneGround, wired into
    // ReadyLiveCityLive/ReadyLiveCityReplay/BuildRemoteLiveCityScene -- every `--live-city` entry
    // point, since the zone tint is a live-city-only overlay (no `--scenario`/`--peds` dataset carries a
    // zones.json today). Null (harmless no-op toggle) wherever it wasn't built.
    private Node3D? _zonesNode;

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

    // docs/LIVE-CITY-VISUALS-NOTES.md (tick-rate task): the two INDEPENDENT tick-rate knobs, resolved once
    // at the top of _Ready (ValidateSimHz/ValidateRenderHz, mirroring src/Sim.Viewer/Program.cs's own
    // helpers) so every downstream path (local live, remote, replay) sees the same values. `_simHz` is
    // CLI-only/display -- `_liveCityDt` (the value ProcessLiveCity's accumulator actually loops on, in
    // place of the old hardcoded `LiveCityTickSeconds` const) is set from it once in ReadyLiveCityLive,
    // after building the LiveCityConfig the Hz maps through (Dt = 1/Hz). `_renderHz` IS live -- the
    // on-screen slider (BuildRateControlUi) pushes straight to Godot.Engine.MaxFps at runtime.
    private int _simHz = 2;
    private int _renderHz = 60;
    private double _liveCityDt = LiveCityTickSeconds;

    // The on-screen render-hz control (BuildRateControlUi) -- built once from ReadyLiveCityLive/
    // ReadyLiveCityReplay, mirrors _playbackUi/_timelineSlider's own field shape.
    private CanvasLayer? _rateUi;
    private HSlider? _renderHzSlider;
    private Label? _rateLabel;

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §2.3/§4, -TASKS.md D3 -- the `--live-city --replay <file.simrec>`
    // REPLAY path: swaps LiveCitySource for a PlaybackClock-driven ReplicationFileSource (cars) +
    // PedFrameTrack (peds), both from the packaged SumoSharp.Replication (Sim.Replication.Recording).
    // Live vs. replay differ ONLY in the source (design tenet 2) -- ProcessLiveCity dispatches to
    // ProcessLiveCityReplay below, which still drives the SAME _reconstructor/UpdateCars/UpdateLivePeds-
    // shaped rendering, just off these fields instead of `_liveCitySource`.
    private bool _replay;
    private PlaybackClock? _clock;
    private ReplicationFileSource? _replaySource;
    private PedFrameTrack? _pedTrack;

    // docs/LIVE-CITY-VIEWERS-TASKS.md D3 -- the Godot playback UI (Play/Pause, Restart, frame-step, speed,
    // timeline slider + a "t = .../..." label), built ONLY in replay mode (BuildPlaybackUi, called from
    // ReadyLiveCityReplay) and updated every ProcessLiveCityReplay frame (UpdatePlaybackUi).
    private CanvasLayer? _playbackUi;
    private HSlider? _timelineSlider;
    private Label? _timeLabel;
    private Button? _playPauseButton;
    private bool _sliderDragging;
    private bool _wasPlayingBeforeDrag;

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §5, -TASKS.md D4 -- the stable instance-index -> VehicleHandle map
    // (design §5: "Maintain a stable instance-index -> VehicleHandle map in UpdateCars") plus each car's
    // last-known WORLD position (for the click-pick's screen projection and for positioning the selection
    // ring/label every frame without a second Reconstruct pass) -- both rebuilt every UpdateCars call, in
    // the SAME order as the car MultiMesh's own instance indices, so index i here always means "car
    // MultiMesh instance i" for exactly one frame. `_vehicleNames` is the separate Handle->SUMO-id table
    // (design §5's "resolves handle -> human-readable id"): populated from LiveCitySource.Sample().Cars
    // each live-mode frame (the LIVE path has real names on LiveCitySnapshot -- design §3.1 note); left
    // empty in replay (the wire carries no name yet, Stage E) so NameFor's handle-string fallback applies.
    private readonly List<VehicleHandle> _carHandles = new();
    private readonly List<Vector3> _carWorldPositions = new();
    private readonly Dictionary<VehicleHandle, string> _vehicleNames = new();

    // The click-latched selection (design §5: "latch its VehicleHandle") + the highlight ring/label nodes,
    // built once (BuildSelectionUi, called from both ReadyLiveCityLive/ReadyLiveCityReplay) and repositioned
    // every frame by UpdateSelectionHighlight -- works in BOTH live and replay modes (task D4).
    private VehicleHandle? _selectedHandle;
    private MeshInstance3D? _selectionRing;
    private CanvasLayer? _selectionLabelLayer;
    private Label? _selectionLabelNode;

    public override void _Ready()
    {
        // docs/LIVE-CITY-VISUALS-NOTES.md (tick-rate task): resolved FIRST, before any mode dispatch, so
        // every path (local/dds, live/replay/remote) sees the same validated values. `_renderHz` is applied
        // to Godot's global frame-rate cap immediately (render-hz is a general viewer knob, not scoped to
        // `--live-city`, exactly like the Raylib viewer's own render-fps-cap default of 60 applies to every
        // mode); `_simHz` only has an effect inside the live-city path (ReadyLiveCityLive), so it is display-
        // only elsewhere.
        _simHz = ValidateSimHz(ParseSimHzArg());
        _renderHz = ValidateRenderHz(ParseRenderHzArg());
        Godot.Engine.MaxFps = _renderHz;
        GD.Print($"Main: tick rates -- sim-hz={_simHz} (live-city only) render-hz={_renderHz} (Engine.MaxFps).");

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
                // docs/LIVE-CITY-VIEWERS-DESIGN.md §7, -TASKS.md Stage E (E4): `--live-city --transport=dds`
                // takes its OWN dedicated combined-remote path (dual subscriber: DdsSubscriber + a
                // DdsPedReplicationSource on the SAME participant, rendering both) -- checked BEFORE the
                // `--peds` fork below so the two never collide.
                //
                // D3b: `--peds` over live DDS reuses the ped scene setup (local plaza net for the backdrop)
                // but swaps the local byte-loopback ped-sim for a DdsPedReplicationSource fed by a separate
                // `--mode ped-publish` process; the vehicle remote path (ReadyRemote) is unchanged.
                if (_liveCity)
                {
                    ReadyLiveCityRemote();
                }
                else if (_peds)
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
        var overviewCamera = BuildCameraAndLight(bbox, makeCurrent: !wallMode && _cameraMode != "close");
        _carMultiMesh = BuildCarMultiMesh();

        if (wallMode)
        {
            BuildVideoWallChannels(channels);
            GD.Print($"Main: video wall — {channels.Count} channel(s).");
        }
        else
        {
            if (_cameraMode == "close")
            {
                _closeCamera = new Camera3D { Name = "CloseCamera", Current = true, Far = 2000f };
                AddChild(_closeCamera);
                UpdateCloseCameraFraming(_closeCamera, null);
                GD.Print("Main: --camera=close active (low angled close-up framing).");
            }

            SetupOrbitCamera(overviewCamera, bbox);
        }

        if (ParseHideBuildingsArg() && _buildingsNode is not null)
        {
            _buildingsNode.Visible = false;
            GD.Print("Main: --hide-buildings active (buildings start hidden; press B to toggle).");
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
        var pedOverviewCamera = BuildCameraAndLight(pedBbox, makeCurrent: _cameraMode != "close");

        if (_cameraMode == "close")
        {
            _closeCamera = new Camera3D { Name = "CloseCamera", Current = true, Far = 2000f };
            AddChild(_closeCamera);
            UpdateCloseCameraFraming(_closeCamera, null);
        }

        SetupOrbitCamera(pedOverviewCamera, pedBbox);

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

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §6, -TASKS.md D1/D2/D3 -- the `--live-city` setup entry point.
    // `--replay <file.simrec>` (D3) takes the file-backed replay path; otherwise the LOCAL live path (D1).
    // Live vs. replay differ ONLY in the source (design tenet 2) -- see ReadyLiveCityLive/ReadyLiveCityReplay.
    private void ReadyLiveCity(string repoRoot)
    {
        var replayPath = ParseReplayArg();
        _replay = replayPath is not null;

        if (_replay)
        {
            ReadyLiveCityReplay(repoRoot, replayPath!);
        }
        else
        {
            ReadyLiveCityLive(repoRoot);
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
    private void ReadyLiveCityLive(string repoRoot)
    {
        try
        {
            // docs/LIVE-CITY-VISUALS-NOTES.md (tick-rate task): `_simHz` (resolved in _Ready, before this
            // ever runs) sets LiveCityConfig.SimHz => Dt = 1/Hz -- the SAME Dt that flows into BOTH the
            // engine step-length AND the ped demand's stepDt inside LiveCitySim's own ctor (the coupling
            // invariant), so this one assignment is enough; no separate ped-side knob needed here. `_liveCityDt`
            // mirrors it back out for ProcessLiveCity's accumulator (replacing the old hardcoded
            // `LiveCityTickSeconds` const read).
            var liveCfg = LiveCityConfig.ForRepoRoot(repoRoot);
            liveCfg.SimHz = _simHz;
            _liveCityDt = liveCfg.Dt;
            _liveCitySource = new LiveCitySource(liveCfg);
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

        // docs/LIVE-CITY-VISUALS-NOTES.md deliverable 2: zone ground tint, built BEFORE the road meshes so
        // it lands earlier in the scene tree -- Godot's default opaque/blended draw order for coplanar-ish
        // transparent geometry follows a mix of tree order and distance-from-camera sort, but since the
        // roads sit strictly ABOVE the zone tint (BuildRoadMeshesCropped at z=0 vs. ZoneGroundBuilder's
        // z=-0.05 default) the depth test alone is what keeps roads visually on top; building zones first
        // simply keeps this method's own read order matching "ground layer, then roads" (docs/LIVE-CITY-
        // VISUALS-NOTES.md's stated per-layer sequencing).
        BuildZoneGround(_liveCitySource.Scene);
        var roadBbox = BuildRoadMeshesCropped(_liveCitySource.Network, x0, y0, x1, y1);
        BuildCrosswalksAndLaneMarkings(_liveCitySource.Network, (x0, y0, x1, y1));
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

        var liveOverviewCamera = BuildCameraAndLight(
            frameBbox, makeCurrent: _cameraMode != "close",
            heightFactor: LiveCityCameraHeightFactor, backFactor: LiveCityCameraBackFactor);
        _carMultiMesh = BuildCarMultiMesh();
        _pedMultiMesh = BuildPedMultiMesh();
        BuildSelectionUi();
        BuildRateControlUi();

        // Deliverable 2 (docs/LIVE-CITY-VIEWERS-DESIGN.md TL wiring) -- ReadyLocal/ReadyRemote already wire
        // this; `--live-city` (local live) never did, so TrafficLightPlacer/BuildTrafficLights/
        // UpdateTrafficLights were correct but unreachable and no signal head ever appeared. Same closure
        // shape ReadyLocal uses, over LiveCitySource's own (full, uncropped) NetworkModel -- the
        // TL-controlled lane handles ProcessLiveCity's TlStateByLane lookup passes in are dense handles into
        // that same network, cropping doesn't change lane numbering.
        var liveCitySource = _liveCitySource;
        _placeSignalHeads = keys => TrafficLightPlacer.Place(liveCitySource!.Network, keys);

        if (ParseShowZonesArg() && _zonesNode is not null)
        {
            _zonesNode.Visible = true;
            GD.Print("Main: --show-zones active (zone tint starts visible; press Z to toggle).");
        }

        if (_cameraMode == "close")
        {
            _closeCamera = new Camera3D { Name = "CloseCamera", Current = true, Far = 2000f };
            AddChild(_closeCamera);
            UpdateCloseCameraFraming(_closeCamera, null);
            GD.Print("Main: --camera=close active (low angled close-up framing).");
        }

        SetupOrbitCamera(liveOverviewCamera, frameBbox);

        GD.Print(
            $"Main: --live-city active (coupled cars+peds+crossing-yield, transport=local, " +
            $"sim-hz={_simHz} dt={_liveCityDt.ToString("F3", CultureInfo.InvariantCulture)} render-hz={_renderHz}).");

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

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §2.3/§4, -TASKS.md D3 -- the `--live-city --replay <file>` REPLAY
    // setup. Builds a PlaybackClock (the slider/buttons' authority) + a ReplicationFileSource (cars, over
    // its OWN received wire geometry -- exactly the remote path's ReplicationLaneShapeSource shape, design
    // tenet 2) + a PedFrameTrack (peds). Roads are still parsed straight off net.xml (this task's own
    // "simplest" option: "still parse the same net for roads") via the SAME pinned crop
    // LiveCityConfig.ForRepoRoot resolves -- cheap (no Engine/navmesh bake), and the recording's own crop
    // is that exact pinned rect by construction (LiveCitySim publishes the FULL net's geometry, so the
    // recording's wire geometry is NOT pre-cropped; framing/road-mesh-building off the parsed net + the
    // known crop keeps the replay scene identical in extent to the live one without waiting on
    // GeometryComplete or hard-coding a duplicate crop rect).
    private void ReadyLiveCityReplay(string repoRoot, string replayPath)
    {
        if (!File.Exists(replayPath))
        {
            GD.PrintErr($"Main: --replay file not found: '{replayPath}'.");
            GetTree().Quit(1);
            return;
        }

        _clock = new PlaybackClock();

        try
        {
            _replaySource = new ReplicationFileSource(replayPath, _clock);
            _pedTrack = new PedFrameTrack(replayPath);
        }
        catch (Exception ex)
        {
            GD.PrintErr($"Main: failed to open replay '{replayPath}': {ex}");
            GetTree().Quit(1);
            return;
        }

        _clock.Duration = _replaySource.Duration;
        if (_replaySource.Dt > 0.0)
        {
            _clock.Dt = _replaySource.Dt;
        }

        // docs/LIVE-CITY-VISUALS-NOTES.md (tick-rate task): replay has no live LiveCityConfig to read a
        // Hz off of -- the rate readout (BuildRateControlUi below) instead shows the RECORDING's own
        // cadence, straight off the file's Dt, overriding _Ready's CLI-resolved `_simHz`/`_liveCityDt`
        // (which describe a live host that doesn't exist in this path).
        _liveCityDt = _clock.Dt;
        _simHz = _clock.Dt > 0.0 ? (int)Math.Round(1.0 / _clock.Dt) : _simHz;

        _reconstructor = new Reconstructor();
        _lanes = new ReplicationLaneShapeSource(_replaySource.Geometry);
        _source = _replaySource;

        _cameraMode = ParseCameraArg();

        var cfg = LiveCityConfig.ForRepoRoot(repoRoot);
        var netPath = Path.Combine(cfg.DatasetDir, "net.xml");
        NetworkModel network;
        try
        {
            network = NetworkParser.Parse(netPath);
        }
        catch (Exception ex)
        {
            GD.PrintErr($"Main: --replay could not parse '{netPath}' for road geometry: {ex}");
            GetTree().Quit(1);
            return;
        }

        // docs/LIVE-CITY-VISUALS-NOTES.md deliverable 2: replay has no live LiveCitySource to read `.Scene`
        // off (there is no LiveCitySim in this path at all -- see the class remark above); the zone tint is
        // purely STATIC world data, unrelated to the recorded car/ped frames, so it is loaded directly off
        // the SAME dataset dir `cfg`/`netPath` above already resolved. Mirrors ReadyLiveCityLive's own
        // "zones before roads" build order.
        BuildZoneGround(LiveCityScene.Load(cfg.DatasetDir));
        var roadBbox = BuildRoadMeshesCropped(network, cfg.X0, cfg.Y0, cfg.X1, cfg.Y1);
        BuildCrosswalksAndLaneMarkings(network, (cfg.X0, cfg.Y0, cfg.X1, cfg.Y1));
        _sceneBbox = roadBbox;

        var cropCenter = new Vector3(
            (roadBbox.Min.X + roadBbox.Max.X) / 2f, 0f, (roadBbox.Min.Z + roadBbox.Max.Z) / 2f);
        var frameHalf = LiveCityFrameHalfExtentMeters;
        var frameBbox = (
            Min: cropCenter - new Vector3(frameHalf, 0f, frameHalf),
            Max: cropCenter + new Vector3(frameHalf, 8f, frameHalf));

        var replayOverviewCamera = BuildCameraAndLight(
            frameBbox, makeCurrent: _cameraMode != "close",
            heightFactor: LiveCityCameraHeightFactor, backFactor: LiveCityCameraBackFactor);
        _carMultiMesh = BuildCarMultiMesh();
        _pedMultiMesh = BuildPedMultiMesh();
        BuildSelectionUi();
        BuildPlaybackUi();
        BuildRateControlUi();

        // Deliverable 2 -- same TL wiring gap as ReadyLiveCityLive, replay flavor. `network` above is
        // parsed straight from the SAME net.xml the live path uses (this method's own "still parse the
        // same net for roads" design note), so the NetworkModel overload applies unchanged -- no need for
        // the wire-geometry overload here even though this IS a replay path, since the road geometry was
        // never actually replicated (it's read locally, same as the live path).
        _placeSignalHeads = keys => TrafficLightPlacer.Place(network, keys);

        if (ParseShowZonesArg() && _zonesNode is not null)
        {
            _zonesNode.Visible = true;
            GD.Print("Main: --show-zones active (zone tint starts visible; press Z to toggle).");
        }

        if (_cameraMode == "close")
        {
            _closeCamera = new Camera3D { Name = "CloseCamera", Current = true, Far = 2000f };
            AddChild(_closeCamera);
            UpdateCloseCameraFraming(_closeCamera, null);
            GD.Print("Main: --camera=close active (low angled close-up framing).");
        }

        SetupOrbitCamera(replayOverviewCamera, frameBbox);

        GD.Print(
            $"Main: --live-city --replay '{replayPath}' active (duration={_clock.Duration:F1}s, " +
            $"dt={_clock.Dt:F2}s, {_pedTrack.FrameCount} ped frame(s)).");

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
        var remoteOverviewCamera = BuildCameraAndLight(roadBbox, makeCurrent: _cameraMode != "close");

        if (_cameraMode == "close")
        {
            _closeCamera = new Camera3D { Name = "CloseCamera", Current = true, Far = 2000f };
            AddChild(_closeCamera);
            UpdateCloseCameraFraming(_closeCamera, null);
            GD.Print("Main: --camera=close active (low angled close-up framing).");
        }

        SetupOrbitCamera(remoteOverviewCamera, roadBbox);
    }

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §7, -TASKS.md Stage E (E4) -- the combined `--live-city
    // --transport=dds` REMOTE setup: no LiveCitySim/Engine here at all (mirrors ReadyRemote's own "do NOT
    // create a SimSource/engine"), only a DdsSubscriber (vehicles) AND a DdsPedReplicationSource (peds)
    // attached to the SAME DdsParticipant -- the DDS-branch mutual exclusion ReadyRemote()/ReadyPeds()'s
    // own dds branch each embody is dropped for THIS mode: both wire types subscribe together and render
    // together every frame (ProcessLiveCityRemote), fed by a single `Sim.Host.App --live-city --transport
    // dds` producer (E3) publishing both topic sets from one net. Cars reuse the SAME
    // Reconstructor/ReplicationLaneShapeSource path the plain vehicle remote uses (now Z- and name-aware,
    // Stage E1/E2); peds reuse the SAME PedReconstructor/UpdatePeds path the `--peds --transport dds`
    // branch uses (design §7: "removing the DDS-branch mutual exclusion"). Static scene (roads; buildings
    // skipped, same reasoning as BuildRemoteScene) is built lazily once geometry arrives
    // (BuildRemoteLiveCityScene, called from ProcessLiveCityRemote exactly like BuildRemoteScene is from
    // AdvanceRemoteSim) -- `_lanes` stays null here, same "nothing to reconstruct/render yet" convention.
    private void ReadyLiveCityRemote()
    {
        _cameraMode = ParseCameraArg();
        _liveCityRemote = true;

        _ddsParticipant = new DdsParticipant();
        _ddsSource = new DdsSubscriber(_ddsParticipant);
        _ddsPedSource = new DdsPedReplicationSource(_ddsParticipant);
        _reconstructor = new Reconstructor();
        _pedReconstructor = new PedReconstructor();
        _source = _ddsSource;
        var ddsSource = _ddsSource;
        _placeSignalHeads = keys => TrafficLightPlacer.Place(ddsSource.Geometry, keys);

        _carMultiMesh = BuildCarMultiMesh();
        _pedMultiMesh = BuildPedMultiMesh();
        BuildSelectionUi();

        GD.Print(
            "Main: --live-city --transport=dds active; waiting for a Sim.Host.App --live-city publisher " +
            "(vehicle geometry + frames, ped crowd)...");

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

    // Lazily builds the static scene from the RECEIVED wire geometry, once complete -- the combined
    // live-city remote's counterpart to BuildRemoteScene above. LiveCitySim publishes the FULL (uncropped)
    // net.xml geometry over the wire (mirrors what it publishes onto its own local in-mem bus, design §1),
    // so this crops the received geometry to the SAME pinned downtown block every other live-city path
    // frames (Sim.LiveCity.LiveCityConfig's own default X0..Y1 -- a fixed constant, not tied to the
    // producer's dataset dir, so the subscriber needs no repo/dataset access of its own to know it) purely
    // for render legibility/build cost (BuildRoadMeshesFromGeometryCropped below), same idea as the LOCAL
    // live-city path's BuildRoadMeshesCropped and the replay path's own crop-by-config. The camera framing
    // reuses the live-city overview knobs (LiveCityFrameHalfExtentMeters/-CameraHeightFactor/-BackFactor),
    // not BuildRemoteScene's whole-received-geometry framing.
    private void BuildRemoteLiveCityScene()
    {
        var geometry = _ddsSource!.Geometry;
        _lanes = new ReplicationLaneShapeSource(geometry);

        var crop = new LiveCityConfig(); // pinned defaults only -- no dataset dir needed just for X0..Y1.

        // docs/LIVE-CITY-VISUALS-NOTES.md deliverable 2 (zone tint) + "Sidewalks"/"Crosswalk zebra"/"Lane
        // markings" rows: none of these four overlays are carried over the DDS wire at all (GeometryCodec.
        // LaneGeo has no Id/EdgeId/AllowsRoadVehicle -- only Handle/Width/Length/Points/Z), so ALL FOUR are
        // BEST-EFFORT local reads off the SAME net.xml a same-machine/checkout producer publishes from (this
        // method's own remark above notes the subscriber is designed to need "no repo/dataset access of its
        // own" just for the crop rect) -- one try/catch, one parse, so a genuinely remote subscriber with no
        // local scenarios/ tree just skips every one of them rather than failing the whole DDS scene build.
        // The parsed NetworkModel's lane Handles are guaranteed to match the wire's (NetworkParser assigns
        // them in the SAME deterministic parse order for the SAME net.xml file the publisher itself parsed),
        // so `pedByHandle` below correctly keys the wire-built ribbon meshes' own material choice.
        IReadOnlyDictionary<int, bool>? pedByHandle = null;
        try
        {
            var zoneCfg = LiveCityConfig.ForRepoRoot(FindRepoRoot());
            BuildZoneGround(LiveCityScene.Load(zoneCfg.DatasetDir));

            var network = NetworkParser.Parse(Path.Combine(zoneCfg.DatasetDir, "net.xml"));
            pedByHandle = PedByHandle(network);
            BuildCrosswalksAndLaneMarkings(network, (crop.X0, crop.Y0, crop.X1, crop.Y1));
        }
        catch (Exception ex)
        {
            GD.Print(
                "Main: zone-tint/sidewalk/crosswalk/lane-marking layers skipped (no local dataset access " +
                $"for remote subscriber): {ex.Message}");
        }

        if (ParseShowZonesArg() && _zonesNode is not null)
        {
            _zonesNode.Visible = true;
            GD.Print("Main: --show-zones active (zone tint starts visible; press Z to toggle).");
        }

        var roadBbox = BuildRoadMeshesFromGeometryCropped(geometry, crop.X0, crop.Y0, crop.X1, crop.Y1, pedByHandle: pedByHandle);
        _sceneBbox = roadBbox;

        var cropCenter = new Vector3(
            (roadBbox.Min.X + roadBbox.Max.X) / 2f, 0f, (roadBbox.Min.Z + roadBbox.Max.Z) / 2f);
        var frameHalf = LiveCityFrameHalfExtentMeters;
        var frameBbox = (
            Min: cropCenter - new Vector3(frameHalf, 0f, frameHalf),
            Max: cropCenter + new Vector3(frameHalf, 8f, frameHalf));

        var remoteLiveOverviewCamera = BuildCameraAndLight(
            frameBbox, makeCurrent: _cameraMode != "close",
            heightFactor: LiveCityCameraHeightFactor, backFactor: LiveCityCameraBackFactor);

        if (_cameraMode == "close")
        {
            _closeCamera = new Camera3D { Name = "CloseCamera", Current = true, Far = 2000f };
            AddChild(_closeCamera);
            UpdateCloseCameraFraming(_closeCamera, null);
            GD.Print("Main: --camera=close active (low angled close-up framing).");
        }

        SetupOrbitCamera(remoteLiveOverviewCamera, frameBbox);

        GD.Print(
            $"Main: --live-city --transport=dds received geometry ({geometry.Count} lane(s) total on the " +
            "wire); road meshes cropped to the pinned downtown block for legibility.");
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

        // Interactive camera controller (docs/LIVE-CITY-VIEWERS-DESIGN.md camera-controller deliverable):
        // `--camera=close`'s continuous auto-tracking (UpdateCloseCameraFraming, called per-frame with live
        // `vehicles`) is superseded by the free orbit controller once one exists -- SetupOrbitCamera already
        // seeded it from that SAME close-up pose at setup time, and _UnhandledInput's drag/wheel/reset
        // handlers are what move it from there. ApplyOrbitCamera is a no-op (video-wall mode) when no orbit
        // camera was built.
        ApplyOrbitCamera();

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
    // coupled sim on ITS OWN fixed sim-cadence accumulator (_liveCityDt, docs/LIVE-CITY-VISUALS-NOTES.md
    // tick-rate task -- `--sim-hz`-selected, LiveCityTickSeconds=0.5's old hardcoded value is now only the
    // field's pre-ReadyLiveCityLive default -- matching LiveCityConfig.Dt -- a SEPARATE accumulator/frame
    // pair from _accumulator/_frame, per design §6's "split per-domain accumulator" requirement), reconstruct
    // cars through the SAME Reconstructor/UpdateCars
    // the --scenario path uses (LiveCitySource.Source/LocalLanes are the SAME shapes SimSource exposes), and
    // rewrite the ped MultiMesh via UpdateLivePeds off LiveCitySource.Peds. Both MultiMeshes render in ONE
    // scene every frame -- no `if(_peds){...return;}` mutual exclusion.
    private void ProcessLiveCity(double delta)
    {
        if (_replay)
        {
            ProcessLiveCityReplay(delta);
            return;
        }

#if CITY3D_REMOTE
        if (_liveCityRemote)
        {
            ProcessLiveCityRemote(delta);
            return;
        }
#endif

        if (_liveCitySource is null || _reconstructor is null || _lanes is null)
        {
            return; // _Ready already reported the error.
        }

        // docs/LIVE-CITY-VISUALS-NOTES.md (tick-rate task): `_liveCityDt` (set from LiveCityConfig.Dt in
        // ReadyLiveCityLive, per `--sim-hz`) replaces the old hardcoded `LiveCityTickSeconds` const read --
        // the accumulator now advances the coupled sim in whatever increment `--sim-hz` selected instead of
        // always 0.5s.
        _liveCityAccumulator += delta;
        while (_liveCityAccumulator >= _liveCityDt)
        {
            _liveCitySource.Tick();
            _liveCityAccumulator -= _liveCityDt;
        }

        var vehicles = _reconstructor.Reconstruct(_liveCitySource.Source, _liveCitySource.LocalLanes, PlayoutDelaySeconds);
        UpdateCars(vehicles);

        // docs/LIVE-CITY-VIEWERS-DESIGN.md §3.1, -TASKS.md D4 -- ONE Sample() this frame gives both the
        // peds AND the live Handle->Name table (LiveCityCar.Name is the real SUMO id, straight off
        // Engine.VehicleIds -- design §3.1's "local/live viewers can also read the name directly from
        // LiveCitySim ... without the wire"). Entries are never removed, so a car that despawns between
        // this frame and a later click still resolves to its last-known name rather than the raw handle.
        var snap = _liveCitySource.Sample();
        foreach (var car in snap.Cars)
        {
            _vehicleNames[car.Handle] = car.Name;
        }

        UpdateLivePeds(snap.Peds);
        UpdateSelectionHighlight();

        // Deliverable 2 (docs/LIVE-CITY-VIEWERS-DESIGN.md TL wiring) -- mirrors RenderFrame's own
        // build-once/update-every-frame TL block; `_placeSignalHeads` is now set by ReadyLiveCityLive above.
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

        ApplyOrbitCamera();

        GD.Print(
            $"Main: frame={_liveCityFrame} liveCityTime={_liveCitySource.Time:F2} " +
            $"cars={vehicles.Count} peds={snap.Peds.Count}");

        _liveCityFrame++;

        if (_liveCityFrame >= QuitAfterFrames && _shotPath is null)
        {
            GD.Print($"Main: reached {QuitAfterFrames} frames, quitting.");
            DisposeSources();
            GetTree().Quit();
        }
    }

#if CITY3D_REMOTE
    // docs/LIVE-CITY-VIEWERS-DESIGN.md §7, -TASKS.md Stage E (E4) -- the per-frame `--live-city
    // --transport=dds` REMOTE body: pump the vehicle subscriber, lazily build the static scene once its
    // geometry completes (mirrors AdvanceRemoteSim + BuildRemoteScene), then reconstruct+render BOTH
    // domains from the wire every frame -- cars through the SAME Reconstructor/UpdateCars the plain
    // vehicle remote uses, peds through the SAME PedReconstructor/UpdatePeds the `--peds --transport dds`
    // branch uses (design §7's "removing the DDS-branch mutual exclusion" made concrete: both run in one
    // method, no `if(_peds){...return;}`). The vehicle Name table is refreshed from the wire's lifecycle-
    // derived `DdsSubscriber.Names` every frame (Stage E2's "once per spawn" table, design §5's "resolves
    // handle -> human-readable id ... in every mode/transport") so click-select shows the real SUMO id
    // remotely too, not just in local live mode.
    private void ProcessLiveCityRemote(double delta)
    {
        if (_ddsSource is null || _ddsPedSource is null || _reconstructor is null || _pedReconstructor is null)
        {
            return; // _Ready already reported the error.
        }

        _ddsSource.Pump();

        if (_lanes is null)
        {
            if (!_ddsSource.GeometryComplete)
            {
                return; // waiting for the producer's one-time geometry publish to finish arriving.
            }

            BuildRemoteLiveCityScene();
        }

        var vehicles = _reconstructor.Reconstruct(_ddsSource, _lanes!, PlayoutDelaySeconds);
        UpdateCars(vehicles);

        // Stage E2: the wire's Handle->name table, populated once per spawn lifecycle event -- accumulate
        // into `_vehicleNames` (never removed, same "last-known name survives despawn" rule the local live
        // path's own Sample().Cars loop uses) so NameFor/click-select resolve the real SUMO id remotely.
        foreach (var kv in _ddsSource.Names)
        {
            _vehicleNames[kv.Key] = kv.Value;
        }

        // Remote: no local sim to tick, so drive the ped reconstructor off the WIRE clock (newest
        // crowd-frame sim-time), kept monotonic across frames -- identical pattern to ProcessPeds' own dds
        // branch, just inline here since both domains share this one method.
        _pedRemoteServerTime = Math.Max(_pedRemoteServerTime, _ddsPedSource.LatestCrowdTime);
        var peds = _pedReconstructor.Reconstruct(_ddsPedSource, _pedRemoteServerTime);
        UpdatePeds(peds);

        UpdateSelectionHighlight();

        // Deliverable 2 -- same TL wiring as ProcessLiveCity; `_placeSignalHeads` is already set by
        // ReadyLiveCityRemote (it just was never actually CALLED per-frame before this).
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

        ApplyOrbitCamera();

        var highPowerPeds = 0;
        foreach (var p in peds)
        {
            if (p.IsHighPower)
            {
                highPowerPeds++;
            }
        }

        var wireVehTimeLabel = _ddsSource.LatestVehicleSampleTime is { } wireVehTime ? $"{wireVehTime:F2}" : "?";
        GD.Print(
            $"Main: frame={_liveCityFrame} remote-live-city wireVehTime={wireVehTimeLabel} " +
            $"pedTime={_pedRemoteServerTime:F2} cars={vehicles.Count} peds={peds.Count} highPowerPeds={highPowerPeds}");

        _liveCityFrame++;

        if (_liveCityFrame >= QuitAfterFrames && _shotPath is null)
        {
            GD.Print($"Main: reached {QuitAfterFrames} frames, quitting.");
            DisposeSources();
            GetTree().Quit();
        }
    }
#endif

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §2.3/§4, -TASKS.md D3 -- the per-frame `--live-city --replay` body:
    // advance the PlaybackClock by wall delta (a no-op while paused), pump the file source (SeekTo(Now)
    // internally), reconstruct cars through the SAME Reconstructor/UpdateCars the live path uses (over the
    // file source + its wire-fed ReplicationLaneShapeSource -- design tenet 2), and rewrite the ped
    // MultiMesh from PedFrameTrack.PedsAt(clock.Now). No LiveCitySource/Engine here at all -- replay never
    // steps a sim, only reads the recording.
    private void ProcessLiveCityReplay(double delta)
    {
        if (_clock is null || _replaySource is null || _pedTrack is null
            || _reconstructor is null || _lanes is null)
        {
            return; // _Ready already reported the error.
        }

        _clock.Tick(delta);
        _replaySource.Pump();

        var vehicles = _reconstructor.Reconstruct(_replaySource, _lanes, PlayoutDelaySeconds);
        UpdateCars(vehicles);

        var peds = _pedTrack.PedsAt(_clock.Now);
        UpdateReplayPeds(peds);
        UpdateSelectionHighlight();
        UpdatePlaybackUi();

        // Deliverable 2 -- same TL wiring as ProcessLiveCity/ProcessLiveCityRemote; `_placeSignalHeads` is
        // set by ReadyLiveCityReplay above.
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

        ApplyOrbitCamera();

        GD.Print(
            $"Main: replay frame={_liveCityFrame} t={_clock.Now:F2}/{_clock.Duration:F2} " +
            $"playing={_clock.Playing} speed={_clock.Speed:F1} cars={vehicles.Count} peds={peds.Count}");

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

        ApplyOrbitCamera();

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
        _replaySource?.Dispose();
        _replaySource = null;
#if CITY3D_REMOTE
        _ddsPedSource?.Dispose();
        _ddsPedSource = null;
        _ddsSource?.Dispose();
        _ddsSource = null;
        _ddsParticipant?.Dispose();
        _ddsParticipant = null;
#endif
    }

    // Interactive camera controller (docs/LIVE-CITY-VIEWERS-DESIGN.md camera-controller deliverable) --
    // works in EVERY mode (--scenario, --peds, --live-city local/replay/remote), unlike vehicle pick and
    // the replay transport keys below, which stay live-city-only (their pre-existing scope, unchanged).
    // Left-drag orbits, middle-drag (or shift+left-drag) pans, the wheel zooms, R/Home resets to the
    // initial `--camera=overview|close` framing. Click-vs-drag is disambiguated on left-button RELEASE
    // (task requirement): press just records where the button went down; release compares that to the
    // current position and only fires TryPickVehicleAt when the cursor moved less than
    // ClickDragThresholdPixels -- a real drag is fully consumed by the orbit and never falls through to a
    // pick underneath it. `_UnhandledInput` (not `_Input`) so a click/drag that starts on a playback-UI
    // Control (a Button/HSlider) is consumed by that Control first, same as before.
    //
    // Headless-safe (task requirement): `--shot` runs under Xvfb with no real user, so no InputEvent of any
    // kind is ever delivered here -- `_orbitController` only ever moves from its `SetupOrbitCamera`-seeded
    // (optionally `--cam-yaw`/`--cam-pitch`/`--cam-dist`/`--cam-focus`-overridden) initial state, and the
    // screenshot captures exactly that framing.
    public override void _UnhandledInput(InputEvent @event)
    {
        switch (@event)
        {
            case InputEventMouseButton { ButtonIndex: MouseButton.Left } mouseButton:
                if (mouseButton.Pressed)
                {
                    _leftButtonDownPos = mouseButton.Position;
                    _leftButtonDown = true;
                }
                else
                {
                    var downPos = _leftButtonDownPos;
                    _leftButtonDown = false;
                    _leftButtonDownPos = null;
                    if (_liveCity && downPos is { } start && start.DistanceTo(mouseButton.Position) < ClickDragThresholdPixels)
                    {
                        TryPickVehicleAt(mouseButton.Position);
                    }
                }

                GetViewport().SetInputAsHandled();
                break;

            case InputEventMouseButton { ButtonIndex: MouseButton.Middle } middleButton:
                _middleButtonDown = middleButton.Pressed;
                GetViewport().SetInputAsHandled();
                break;

            case InputEventMouseButton { Pressed: true, ButtonIndex: MouseButton.WheelUp }:
                _orbitController?.Zoom(ZoomStepFactor);
                GetViewport().SetInputAsHandled();
                break;

            case InputEventMouseButton { Pressed: true, ButtonIndex: MouseButton.WheelDown }:
                _orbitController?.Zoom(1f / ZoomStepFactor);
                GetViewport().SetInputAsHandled();
                break;

            case InputEventMouseMotion motion:
                if (_orbitController is not null)
                {
                    if (_middleButtonDown || (_leftButtonDown && motion.ShiftPressed))
                    {
                        // Pan: world-space delta scaled by the CURRENT distance so the pan speed feels
                        // consistent whether zoomed in close or far out (a fixed pixel->metre scale would
                        // crawl when zoomed out and overshoot wildly when zoomed in).
                        var scale = PanMetersPerPixelPerDistance * _orbitController.Distance;
                        _orbitController.Pan(-motion.Relative.X * scale, motion.Relative.Y * scale);
                        ApplyOrbitCamera();
                        GetViewport().SetInputAsHandled();
                    }
                    else if (_leftButtonDown)
                    {
                        _orbitController.Orbit(-motion.Relative.X * OrbitYawRadPerPixel, motion.Relative.Y * OrbitPitchRadPerPixel);
                        ApplyOrbitCamera();
                        GetViewport().SetInputAsHandled();
                    }
                }

                break;

            case InputEventKey { Pressed: true } key when key.Keycode == Key.Home:
                _orbitController?.Reset();
                ApplyOrbitCamera();
                GetViewport().SetInputAsHandled();
                break;

            case InputEventKey { Pressed: true } key when key.Keycode == Key.R && !_replay:
                _orbitController?.Reset();
                ApplyOrbitCamera();
                GetViewport().SetInputAsHandled();
                break;

            case InputEventKey { Pressed: true } key when key.Keycode == Key.B:
                if (_buildingsNode is not null)
                {
                    _buildingsNode.Visible = !_buildingsNode.Visible;
                    GD.Print($"Main: buildings visible={_buildingsNode.Visible} (B toggle).");
                }

                GetViewport().SetInputAsHandled();
                break;

            // docs/LIVE-CITY-VISUALS-NOTES.md deliverable 2: runtime zone-tint toggle, mirrors the B/
            // buildings toggle immediately above.
            case InputEventKey { Pressed: true } key when key.Keycode == Key.Z:
                if (_zonesNode is not null)
                {
                    _zonesNode.Visible = !_zonesNode.Visible;
                    GD.Print($"Main: zones visible={_zonesNode.Visible} (Z toggle).");
                }

                GetViewport().SetInputAsHandled();
                break;

            case InputEventKey { Pressed: true } key when _liveCity && _replay && _clock is not null:
                if (key.Keycode == Key.Space)
                {
                    TogglePlayPause();
                    GetViewport().SetInputAsHandled();
                }
                else if (key.Keycode == Key.Left)
                {
                    _clock.StepFrame(-1);
                    GetViewport().SetInputAsHandled();
                }
                else if (key.Keycode == Key.Right)
                {
                    _clock.StepFrame(1);
                    GetViewport().SetInputAsHandled();
                }

                break;
        }
    }

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §5, -TASKS.md D4 -- the click-pick itself: project every currently-
    // rendered car's WORLD origin (`_carWorldPositions`, written by UpdateCars this same frame or the last
    // one) to SCREEN space via the active Camera3D's own UnprojectPosition (design §5's literal
    // suggestion: "camera ray -> nearest instance", done here as "nearest screen-projected instance" since
    // there is no physics collider to actually ray-cast against), skip anything behind the camera
    // (IsPositionBehind -- an UnprojectPosition of a behind-camera point is not a meaningful screen pixel),
    // and hand the filtered list to CityLib.VehiclePicker.PickNearestScreen (the pure, unit-tested part of
    // this path). A miss (nothing within PickPixelRadius) leaves the previous selection untouched --
    // mirrors LiveCityOverlay.OnWorldClick's own "clicking empty road should not blank out an already-
    // identified vehicle" rule.
    private readonly List<(float X, float Y)> _pickScreenScratch = new();
    private readonly List<int> _pickIndexScratch = new();

    private void TryPickVehicleAt(Vector2 mousePos)
    {
        var camera = GetViewport().GetCamera3D();
        if (camera is null || _carWorldPositions.Count == 0)
        {
            return;
        }

        _pickScreenScratch.Clear();
        _pickIndexScratch.Clear();

        for (var i = 0; i < _carWorldPositions.Count; i++)
        {
            var pos = _carWorldPositions[i];
            if (camera.IsPositionBehind(pos))
            {
                continue;
            }

            var screen = camera.UnprojectPosition(pos);
            _pickScreenScratch.Add((screen.X, screen.Y));
            _pickIndexScratch.Add(i);
        }

        var localIdx = CityLib.VehiclePicker.PickNearestScreen(
            _pickScreenScratch, mousePos.X, mousePos.Y, PickPixelRadius);
        if (localIdx < 0)
        {
            return;
        }

        var carIdx = _pickIndexScratch[localIdx];
        var handle = _carHandles[carIdx];
        _selectedHandle = handle;
        GD.Print($"Main: picked vehicle {NameFor(handle)} at screen ({mousePos.X:F0},{mousePos.Y:F0}).");
    }

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §5 -- "A viewer-side Dictionary<VehicleHandle,string> Names ...
    // resolves handle -> human-readable id ... in every mode/transport". `_vehicleNames` is populated every
    // live-mode frame from LiveCitySource.Sample().Cars (real SUMO ids); it stays empty in replay (the
    // wire's LifecycleRecord carries no name yet -- Stage E, out of scope here), so this falls back to
    // VehicleHandle.ToString()'s own "Vehicle#{Index}.{Generation}" -- exactly the task's stated
    // acceptable-for-now replay label.
    private string NameFor(VehicleHandle handle)
        => _vehicleNames.TryGetValue(handle, out var name) ? name : handle.ToString();

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §5, -TASKS.md D4 -- builds the selection ring (a small emissive
    // torus, hidden until a pick happens) + the identity label (a screen-space Godot Label in its own
    // CanvasLayer, so it always faces the viewer and stays legible regardless of camera distance/angle --
    // same "label drawn in screen space" choice LiveCityOverlay.DrawWorldOver makes). Called once from both
    // ReadyLiveCityLive and ReadyLiveCityReplay -- works in BOTH modes (task D4's explicit requirement).
    private void BuildSelectionUi()
    {
        var ringMaterial = new StandardMaterial3D
        {
            AlbedoColor = SelectionColor,
            EmissionEnabled = true,
            Emission = SelectionColor,
            EmissionEnergyMultiplier = 2.5f,
        };
        _selectionRing = new MeshInstance3D
        {
            Mesh = new TorusMesh { InnerRadius = 1.5f, OuterRadius = 2.0f, Rings = 16, RingSegments = 8 },
            Name = "SelectionRing",
            Visible = false,
        };
        _selectionRing.SetSurfaceOverrideMaterial(0, ringMaterial);
        AddChild(_selectionRing);

        _selectionLabelLayer = new CanvasLayer { Name = "SelectionLabelLayer" };
        AddChild(_selectionLabelLayer);

        _selectionLabelNode = new Label { Text = string.Empty, Visible = false };
        _selectionLabelNode.AddThemeColorOverride("font_color", new Color(1f, 1f, 1f));
        _selectionLabelNode.AddThemeColorOverride("font_outline_color", new Color(0f, 0f, 0f));
        _selectionLabelNode.AddThemeConstantOverride("outline_size", 4);
        _selectionLabelNode.AddThemeFontSizeOverride("font_size", 18);
        _selectionLabelLayer.AddChild(_selectionLabelNode);
    }

    // docs/LIVE-CITY-VISUALS-NOTES.md (tick-rate task): the on-screen rate control -- a small top-left
    // Godot Control mirroring src/Sim.Viewer's DrawLiveCityRatePanel one-for-one: a display-only sim-hz
    // label (sim-hz is baked into the engine's step-length at LiveCitySim construction time, so there is
    // no live knob to turn here -- "relaunch --sim-hz=N to change", same hint the Raylib panel gives) and
    // an HSlider that pushes render-hz straight to Godot.Engine.MaxFps INSTANTLY, clamped to the same
    // [15,60] band ValidateRenderHz enforces at startup. Called once from ReadyLiveCityLive (and, for
    // parity, ReadyLiveCityReplay) -- works in both live and replay live-city modes.
    private void BuildRateControlUi()
    {
        _rateUi = new CanvasLayer { Name = "RateUi" };
        AddChild(_rateUi);

        var panel = new PanelContainer { Name = "RatePanel" };
        panel.SetAnchorsPreset(Control.LayoutPreset.TopLeft);
        panel.OffsetLeft = 16f;
        panel.OffsetTop = 16f;
        panel.OffsetRight = 316f;
        panel.OffsetBottom = 100f;
        _rateUi.AddChild(panel);

        var vbox = new VBoxContainer();
        panel.AddChild(vbox);

        _rateLabel = new Label { Text = $"sim: {_simHz} Hz (dt={_liveCityDt:F3}s) -- relaunch --sim-hz=N to change" };
        vbox.AddChild(_rateLabel);

        var renderRow = new HBoxContainer();
        vbox.AddChild(renderRow);

        var renderLabel = new Label { Text = "render (Hz):" };
        renderRow.AddChild(renderLabel);

        _renderHzSlider = new HSlider
        {
            MinValue = 15,
            MaxValue = 60,
            Step = 1,
            Value = _renderHz,
            CustomMinimumSize = new Vector2(180f, 20f),
            SizeFlagsHorizontal = Control.SizeFlags.ExpandFill,
        };
        _renderHzSlider.ValueChanged += OnRenderHzSliderChanged;
        renderRow.AddChild(_renderHzSlider);

        GD.Print("Main: rate control UI built (sim-hz display + render-hz slider).");
    }

    // Runtime render-hz change (task deliverable: "render-hz adjustable at runtime (instant)") -- pushes
    // straight to Godot.Engine.MaxFps, exactly like the Raylib viewer's DrawLiveCityRatePanel slider pushes
    // to Raylib.SetTargetFPS. No rebuild/relaunch needed.
    private void OnRenderHzSliderChanged(double value)
    {
        _renderHz = Math.Clamp((int)value, 15, 60);
        Godot.Engine.MaxFps = _renderHz;
    }

    // Called every live-city render frame (both live and replay). Repositions the ring above the selected
    // vehicle's CURRENT world position (re-resolved by VehicleHandle via `_carHandles`/`_carWorldPositions`
    // every frame, per UpdateCars' own remark on why instance index is never cached across frames) and the
    // label at its screen projection; hides both while nothing is selected or the selected vehicle isn't
    // present in this exact frame's render set (e.g. transiently out of the crop/reconstruction window).
    private void UpdateSelectionHighlight()
    {
        if (_selectionRing is null || _selectionLabelNode is null)
        {
            return;
        }

        if (_selectedHandle is not { } handle)
        {
            _selectionRing.Visible = false;
            _selectionLabelNode.Visible = false;
            return;
        }

        var idx = _carHandles.IndexOf(handle);
        if (idx < 0)
        {
            _selectionRing.Visible = false;
            _selectionLabelNode.Visible = false;
            return;
        }

        var pos = _carWorldPositions[idx];
        _selectionRing.Visible = true;
        _selectionRing.Position = pos + new Vector3(0f, CarHeightMeters + 0.8f, 0f);

        var camera = GetViewport().GetCamera3D();
        if (camera is not null && !camera.IsPositionBehind(pos))
        {
            var screen = camera.UnprojectPosition(pos);
            _selectionLabelNode.Text = NameFor(handle);
            _selectionLabelNode.Position = new Vector2(screen.X + 14f, screen.Y - 28f);
            _selectionLabelNode.Visible = true;
        }
        else
        {
            _selectionLabelNode.Visible = false;
        }
    }

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §4, -TASKS.md D3 -- the Godot playback UI: a CanvasLayer/Control
    // panel built in code (Play/Pause, Restart, frame-step x2, a 0.5x/1x/2x speed row, a "t = .../..." time
    // label, and an HSlider timeline bound to clock.Now over [0, clock.Duration]). Built ONLY from
    // ReadyLiveCityReplay -- this UI shows ONLY in replay mode, per the task spec.
    private void BuildPlaybackUi()
    {
        if (_clock is null)
        {
            return;
        }

        var canvas = new CanvasLayer { Name = "PlaybackUi" };
        AddChild(canvas);
        _playbackUi = canvas;

        var panel = new PanelContainer { Name = "PlaybackPanel" };
        panel.SetAnchorsPreset(Control.LayoutPreset.BottomWide);
        panel.OffsetTop = -76f;
        panel.OffsetBottom = -12f;
        panel.OffsetLeft = 16f;
        panel.OffsetRight = -16f;
        canvas.AddChild(panel);

        var vbox = new VBoxContainer();
        panel.AddChild(vbox);

        var row = new HBoxContainer();
        vbox.AddChild(row);

        _playPauseButton = new Button { Text = "Pause" };
        _playPauseButton.Pressed += TogglePlayPause;
        row.AddChild(_playPauseButton);

        var restartButton = new Button { Text = "Restart" };
        restartButton.Pressed += () => _clock?.Restart();
        row.AddChild(restartButton);

        var stepBack = new Button { Text = "<< Frame" };
        stepBack.Pressed += () => _clock?.StepFrame(-1);
        row.AddChild(stepBack);

        var stepFwd = new Button { Text = "Frame >>" };
        stepFwd.Pressed += () => _clock?.StepFrame(1);
        row.AddChild(stepFwd);

        foreach (var speed in new[] { 0.5, 1.0, 2.0 })
        {
            var speedButton = new Button { Text = $"{speed:0.0}x" };
            speedButton.Pressed += () =>
            {
                if (_clock is not null)
                {
                    _clock.Speed = speed;
                }
            };
            row.AddChild(speedButton);
        }

        _timeLabel = new Label { Text = "t = 0.0s / 0.0s" };
        row.AddChild(_timeLabel);

        _timelineSlider = new HSlider
        {
            MinValue = 0.0,
            MaxValue = Math.Max(_clock.Duration, 0.01),
            Step = 0.01,
            CustomMinimumSize = new Vector2(0f, 24f),
            SizeFlagsHorizontal = Control.SizeFlags.ExpandFill,
        };
        _timelineSlider.DragStarted += OnTimelineDragStarted;
        _timelineSlider.DragEnded += OnTimelineDragEnded;
        _timelineSlider.ValueChanged += OnTimelineValueChanged;
        vbox.AddChild(_timelineSlider);

        GD.Print("Main: --replay playback UI built (Play/Pause, Restart, frame-step, speed, timeline slider).");
    }

    private void TogglePlayPause()
    {
        if (_clock is null)
        {
            return;
        }

        if (_clock.Playing)
        {
            _clock.Pause();
        }
        else
        {
            _clock.Play();
        }
    }

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §4 -- "dragging it calls clock.SeekTo (pause while dragging, restore
    // on release)". DragStarted latches whether the clock was playing so DragEnded can restore it exactly.
    private void OnTimelineDragStarted()
    {
        if (_clock is null)
        {
            return;
        }

        _sliderDragging = true;
        _wasPlayingBeforeDrag = _clock.Playing;
        _clock.Pause();
    }

    // Live-scrub: while dragging, every slider value change seeks the clock immediately so the rendered
    // scene tracks the thumb instead of only updating on release.
    private void OnTimelineValueChanged(double value)
    {
        if (_sliderDragging)
        {
            _clock?.SeekTo(value);
        }
    }

    private void OnTimelineDragEnded(bool valueChanged)
    {
        if (_clock is null || _timelineSlider is null)
        {
            return;
        }

        _sliderDragging = false;
        _clock.SeekTo(_timelineSlider.Value);
        if (_wasPlayingBeforeDrag)
        {
            _clock.Play();
        }
    }

    // Called every ProcessLiveCityReplay frame: refreshes the "t = .../..." label, the Play/Pause button's
    // own text, and the slider thumb position from clock.Now -- via SetValueNoSignal so this write never
    // re-fires ValueChanged (which would otherwise feed back into OnTimelineValueChanged's own SeekTo and
    // fight the clock every frame). Skipped entirely while the user is actively dragging the thumb, so the
    // thumb only ever reflects either the clock (not dragging) or the user's own drag (dragging), never both.
    private void UpdatePlaybackUi()
    {
        if (_clock is null)
        {
            return;
        }

        if (_timeLabel is not null)
        {
            _timeLabel.Text = $"t = {_clock.Now:F1}s / {_clock.Duration:F1}s";
        }

        if (_playPauseButton is not null)
        {
            _playPauseButton.Text = _clock.Playing ? "Pause" : "Play";
        }

        if (_timelineSlider is not null && !_sliderDragging)
        {
            _timelineSlider.SetValueNoSignal(_clock.Now);
        }
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
            RoadMeshBuilder.BuildAll(network, includeInternal: true), network.LanesByHandle.Count,
            PedByHandle(network));

    // docs/LIVE-CITY-VISUALS-NOTES.md "Sidewalks" row -- Handle -> "is this a pedestrian-only lane"
    // (Sim.Ingest.Lane.AllowsRoadVehicle == false), built once off a NetworkModel and threaded into
    // BuildRoadMeshesFromRibbons's material choice. Shared by every NetworkModel-backed road-mesh caller
    // (BuildRoadMeshes, BuildRoadMeshesCropped) so the ped-vs-car material split is computed in exactly one
    // place.
    private static Dictionary<int, bool> PedByHandle(NetworkModel network)
    {
        var dict = new Dictionary<int, bool>(network.LanesByHandle.Count);
        foreach (var lane in network.LanesByHandle)
        {
            dict[lane.Handle] = !lane.AllowsRoadVehicle;
        }

        return dict;
    }

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

        return BuildRoadMeshesFromRibbons(filtered, filtered.Count, PedByHandle(network));
    }

#if CITY3D_REMOTE
    // docs/DEMO-CITY3D-DESIGN.md "Data path -> Remote mode" / task T2.2b -- the REMOTE entry point: builds
    // the identical ribbon meshes from the RECEIVED wire geometry (CityLib.RoadMeshBuilder's
    // geometry-dictionary overload) instead of a NetworkModel. BuildRoadMeshesFromRibbons below (the actual
    // ArrayMesh/MeshInstance3D construction) is untouched -- only the RibbonMesh SOURCE differs.
    private (Vector3 Min, Vector3 Max) BuildRoadMeshesFromGeometry(
        IReadOnlyDictionary<int, GeometryCodec.LaneGeo> geometry)
        => BuildRoadMeshesFromRibbons(RoadMeshBuilder.BuildAll(geometry, includeInternal: true), geometry.Count);

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §7, -TASKS.md Stage E (E4) -- BuildRoadMeshesFromGeometry's
    // crop-filtered counterpart (mirrors BuildRoadMeshesCropped's NetworkModel-side filtering, one input
    // shape over): the combined `--live-city --transport=dds` remote path receives the FULL (uncropped)
    // net's geometry over the wire (LiveCitySim publishes its whole parsed net.xml, same as the local
    // path's `Network` property), so this keeps only lanes with at least one point inside the crop rect
    // (+ margin) before handing them to RoadMeshBuilder.Build directly -- same reasoning as
    // BuildRoadMeshesCropped: build/frame only the legible downtown block, not the whole ~4750m net. Z
    // (GeometryCodec.LaneGeo.Z, Stage E1) threads through to RoadMeshBuilder.Build exactly like
    // RoadMeshBuilder.BuildAll's own geometry-dictionary overload does.
    private (Vector3 Min, Vector3 Max) BuildRoadMeshesFromGeometryCropped(
        IReadOnlyDictionary<int, GeometryCodec.LaneGeo> geometry,
        double x0, double y0, double x1, double y1, double marginMeters = 60.0,
        IReadOnlyDictionary<int, bool>? pedByHandle = null)
    {
        var minX = x0 - marginMeters;
        var minY = y0 - marginMeters;
        var maxX = x1 + marginMeters;
        var maxY = y1 + marginMeters;

        var filtered = new List<(int Handle, RibbonMesh Mesh)>();
        foreach (var lane in geometry.Values)
        {
            var inside = false;
            foreach (var (px, py) in lane.Points)
            {
                if (px >= minX && px <= maxX && py >= minY && py <= maxY)
                {
                    inside = true;
                    break;
                }
            }

            if (!inside)
            {
                continue;
            }

            var shape = new (double X, double Y)[lane.Points.Length];
            for (var i = 0; i < shape.Length; i++)
            {
                shape[i] = (lane.Points[i].X, lane.Points[i].Y);
            }

            double[]? shapeZ = null;
            if (lane.Z is { Length: > 0 } laneZ)
            {
                shapeZ = new double[laneZ.Length];
                for (var i = 0; i < laneZ.Length; i++)
                {
                    shapeZ[i] = laneZ[i];
                }
            }

            filtered.Add((lane.Handle, RoadMeshBuilder.Build(shape, shapeZ, lane.Width)));
        }

        GD.Print(
            $"Main: --live-city --transport=dds crop [{x0:F0},{y0:F0}]-[{x1:F0},{y1:F0}] kept " +
            $"{filtered.Count} of {geometry.Count} lane(s) from the wire.");

        return BuildRoadMeshesFromRibbons(filtered, filtered.Count, pedByHandle);
    }
#endif

    // docs/DEMO-CITY3D-DESIGN.md "Procedural scene generation -> Roads" / task T1.3 (and T2.2b's remote
    // extension). Static geometry, built ONCE at load (design: "two clocks: static geometry ... built once
    // at load"). SHARED between local and remote: turns whatever per-lane RibbonMesh sequence the caller
    // computed (from a NetworkModel locally, from received wire geometry remotely -- see the two callers
    // above) into Godot ArrayMesh/MeshInstance3D nodes. Returns the Godot-space bounding box of every
    // emitted vertex, so the camera can frame the whole network.
    //
    // docs/LIVE-CITY-VISUALS-NOTES.md "Sidewalks" row -- `pedByHandle` (Handle -> Sim.Ingest.
    // Lane.AllowsRoadVehicle == false), when supplied, picks the lighter ConcreteColor material for that
    // lane's MeshInstance3D instead of the shared AsphaltColor one; null (every non-live-city caller) keeps
    // every lane on the single asphalt material, byte-identical to before this layer existed.
    private (Vector3 Min, Vector3 Max) BuildRoadMeshesFromRibbons(
        IEnumerable<(int Handle, RibbonMesh Mesh)> perLane, int totalLaneCount,
        IReadOnlyDictionary<int, bool>? pedByHandle = null)
    {
        var asphaltMaterial = new StandardMaterial3D
        {
            AlbedoColor = AsphaltColor,
            Roughness = 0.95f,
            CullMode = BaseMaterial3D.CullModeEnum.Disabled,
        };
        var concreteMaterial = new StandardMaterial3D
        {
            AlbedoColor = ConcreteColor,
            Roughness = 0.95f,
            CullMode = BaseMaterial3D.CullModeEnum.Disabled,
        };

        var min = new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
        var max = new Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);
        var laneCount = 0;
        var pedLaneCount = 0;

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

            var isPed = pedByHandle is not null && pedByHandle.TryGetValue(handle, out var pedFlag) && pedFlag;
            if (isPed)
            {
                pedLaneCount++;
            }

            var instance = new MeshInstance3D
            {
                Mesh = arrayMesh,
                Name = $"Lane_{handle}",
            };
            instance.SetSurfaceOverrideMaterial(0, isPed ? concreteMaterial : asphaltMaterial);
            AddChild(instance);
            laneCount++;
        }

        GD.Print(
            $"Main: built {laneCount} road ribbon mesh(es) from {totalLaneCount} lane(s) " +
            $"({pedLaneCount} sidewalk/concrete).");

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
        _buildingsNode = instance;

        GD.Print($"Main: built {boxes.Count} building(s).");
        return (min, max);
    }

    // docs/LIVE-CITY-VISUALS-NOTES.md deliverable 2 ("Zones layer"). Static geometry, built ONCE at load,
    // same "build once, no per-frame cost" split as BuildRoadMeshes/BuildBuildings -- a district tint never
    // animates. CityLib.ZoneGroundBuilder does the polygon -> flat-mesh math (fan triangulation + the SUMO
    // -> Godot elevation offset that keeps the tint just below the road ribbons); this method only turns
    // each FlatGroundMesh into a MeshInstance3D, coloured by `ZoneFillPalette[zone.Type]`.
    //
    // Draw order: zones are added LARGEST-AREA-FIRST (sorted by FlatGroundMesh.Area descending) so a big
    // district's tint node -- and therefore its render -- lands before a smaller nested/adjacent zone's,
    // per the task's explicit "so a big zone doesn't cover a small one" requirement. All zone meshes sit at
    // the SAME ground offset (ZoneGroundBuilder's default -0.05m, just under the road surface at 0m) with
    // an unshaded, alpha-blended, double-sided (CullMode.Disabled -- a flat ground wash has no meaningful
    // back face, and the fan triangulation deliberately does not normalize winding, see
    // ZoneGroundBuilder.Build's own remark) material, so overlapping tints blend by DRAW ORDER, not a depth
    // fight.
    //
    // Every zone lands under ONE parent Node3D (`_zonesNode`) so `--show-zones` / the runtime `Z` key
    // toggles the whole layer with a single Visible flip, mirroring `_buildingsNode`/`B` -- except the
    // layer STARTS hidden here unconditionally (owner decision: zones are opt-in, not the default wash).
    // A scene with no zones (a dataset that ships no zones.json) leaves `_zonesNode` null -- a harmless
    // no-op toggle, exactly like `_buildingsNode` in every mode that never builds buildings.
    private void BuildZoneGround(LiveCityScene scene)
    {
        if (scene.Zones.Count == 0)
        {
            GD.Print("Main: 0 zone(s) in scene -- zone-tint layer skipped.");
            return;
        }

        var built = new List<(SceneZone Zone, FlatGroundMesh Mesh)>(scene.Zones.Count);
        foreach (var zone in scene.Zones)
        {
            built.Add((zone, ZoneGroundBuilder.Build(zone.Polygon)));
        }

        // Largest planar area first (descending) -- a plain insertion sort is fine at n=6 (one dataset's
        // worth of districts); no need to pull in System.Linq for this file's first sort.
        built.Sort((a, b) => b.Mesh.Area.CompareTo(a.Mesh.Area));

        var root = new Node3D { Name = "Zones" };
        // Owner decision (docs/LIVE-CITY-VISUALS-NOTES.md): zones are opt-in, not the default wash -- the
        // layer starts hidden here (regardless of caller) so every `--live-city` entry point (local live,
        // replay, and remote) is hidden-by-default without each needing its own default-visibility logic.
        // `--show-zones` (checked by the caller right after this returns) and the runtime `Z` key
        // (_UnhandledInput) are the only ways to make it visible.
        root.Visible = false;
        AddChild(root);
        _zonesNode = root;

        var built3D = 0;
        foreach (var (zone, flat) in built)
        {
            if (flat.Vertices.Length == 0)
            {
                continue; // degenerate (<3-point) polygon -- nothing to draw.
            }

            var vertexCount = flat.Vertices.Length / 3;
            var vertices = new Vector3[vertexCount];
            var normals = new Vector3[vertexCount];
            for (var i = 0; i < vertexCount; i++)
            {
                vertices[i] = new Vector3(flat.Vertices[i * 3 + 0], flat.Vertices[i * 3 + 1], flat.Vertices[i * 3 + 2]);
                normals[i] = new Vector3(flat.Normals[i * 3 + 0], flat.Normals[i * 3 + 1], flat.Normals[i * 3 + 2]);
            }

            var arrays = new Godot.Collections.Array();
            arrays.Resize((int)Mesh.ArrayType.Max);
            arrays[(int)Mesh.ArrayType.Vertex] = vertices;
            arrays[(int)Mesh.ArrayType.Normal] = normals;
            arrays[(int)Mesh.ArrayType.Index] = flat.Indices;

            var arrayMesh = new ArrayMesh();
            arrayMesh.AddSurfaceFromArrays(Mesh.PrimitiveType.Triangles, arrays);

            var color = ZoneFillPalette.TryGetValue(zone.Type, out var c) ? c : ZoneFillDefault;
            var material = new StandardMaterial3D
            {
                AlbedoColor = color,
                Transparency = BaseMaterial3D.TransparencyEnum.Alpha,
                ShadingMode = BaseMaterial3D.ShadingModeEnum.Unshaded,
                CullMode = BaseMaterial3D.CullModeEnum.Disabled,
            };

            var instance = new MeshInstance3D
            {
                Mesh = arrayMesh,
                Name = $"Zone_{zone.Id}",
            };
            instance.SetSurfaceOverrideMaterial(0, material);
            root.AddChild(instance);
            built3D++;
        }

        GD.Print($"Main: built {built3D} zone ground tile(s) from {scene.Zones.Count} zone(s).");
    }

    // docs/LIVE-CITY-VISUALS-NOTES.md "Crosswalk zebra" + "Lane markings" rows / DESIGN-live-city-2d-viz.md
    // §2 layers 9/3. Both are static, ALWAYS-visible overlays (unlike the opt-in zone tint) built once, off
    // a NetworkModel -- crossing lanes (CrosswalkBuilder.IsCrossingLaneId) get zebra stripes; every CAR lane
    // (AllowsRoadVehicle) that has a car left-neighbour on the same edge (Lane.LeftNeighbor, precomputed by
    // NetworkParser -- it already excludes non-vehicular siblings, see NetworkModel.Lane's own remark) gets
    // a dashed seam marking. `crop`, when given, keeps only lanes with at least one shape vertex inside the
    // rect (+margin) -- the SAME filtering BuildRoadMeshesCropped/BuildRoadMeshesFromGeometryCropped apply,
    // so this overlay never outruns the cropped road ribbons it sits on. All stripes collapse into ONE
    // MeshInstance3D and all dashes into another (CrosswalkBuilder.MergeRibbons), matching BuildRoadMeshes'
    // own "one draw call" ethos.
    private void BuildCrosswalksAndLaneMarkings(
        NetworkModel network, (double X0, double Y0, double X1, double Y1)? crop = null, double marginMeters = 60.0)
    {
        double minX = 0, minY = 0, maxX = 0, maxY = 0;
        if (crop is { } c)
        {
            minX = c.X0 - marginMeters;
            minY = c.Y0 - marginMeters;
            maxX = c.X1 + marginMeters;
            maxY = c.Y1 + marginMeters;
        }

        bool Inside(IReadOnlyList<(double X, double Y)> shape)
        {
            if (crop is null)
            {
                return true;
            }

            foreach (var (sx, sy) in shape)
            {
                if (sx >= minX && sx <= maxX && sy >= minY && sy <= maxY)
                {
                    return true;
                }
            }

            return false;
        }

        var crosswalkParts = new List<RibbonMesh>();
        var markingParts = new List<RibbonMesh>();
        var crossingLaneCount = 0;
        var markingLaneCount = 0;

        foreach (var lane in network.LanesByHandle)
        {
            if (!Inside(lane.Shape))
            {
                continue;
            }

            if (CrosswalkBuilder.IsCrossingLaneId(lane.Id))
            {
                var (mesh, stripeCount) = CrosswalkBuilder.Build(lane.Shape, lane.Width);
                if (stripeCount > 0)
                {
                    crosswalkParts.Add(mesh);
                    crossingLaneCount++;
                }

                continue;
            }

            if (lane.AllowsRoadVehicle && lane.LeftNeighbor >= 0)
            {
                var (mesh, dashCount) = LaneMarkingBuilder.Build(lane.Shape, lane.Width);
                if (dashCount > 0)
                {
                    markingParts.Add(mesh);
                    markingLaneCount++;
                }
            }
        }

        if (crosswalkParts.Count > 0)
        {
            var merged = CrosswalkBuilder.MergeRibbons(crosswalkParts);
            AddChild(BuildRoadPaintMeshInstance(merged, "CrosswalkZebras"));
        }

        if (markingParts.Count > 0)
        {
            var merged = CrosswalkBuilder.MergeRibbons(markingParts);
            AddChild(BuildRoadPaintMeshInstance(merged, "LaneMarkings"));
        }

        GD.Print(
            $"Main: built crosswalk zebra(s) on {crossingLaneCount} crossing lane(s) and seam marking(s) " +
            $"on {markingLaneCount} lane(s) (out of {network.LanesByHandle.Count} lane(s) considered).");
    }

    // Shared "unshaded near-white paint" MeshInstance3D construction for BuildCrosswalksAndLaneMarkings'
    // two overlays -- mirrors BuildZoneGround's own per-mesh ArrayMesh/StandardMaterial3D pattern (unshaded,
    // double-sided, since a flat road-paint quad has no meaningful back face at overview-camera range).
    private static MeshInstance3D BuildRoadPaintMeshInstance(RibbonMesh mesh, string name)
    {
        var vertexCount = mesh.Vertices.Length / 3;
        var vertices = new Vector3[vertexCount];
        var normals = new Vector3[vertexCount];
        for (var i = 0; i < vertexCount; i++)
        {
            vertices[i] = new Vector3(mesh.Vertices[i * 3 + 0], mesh.Vertices[i * 3 + 1], mesh.Vertices[i * 3 + 2]);
            normals[i] = new Vector3(mesh.Normals[i * 3 + 0], mesh.Normals[i * 3 + 1], mesh.Normals[i * 3 + 2]);
        }

        var arrays = new Godot.Collections.Array();
        arrays.Resize((int)Mesh.ArrayType.Max);
        arrays[(int)Mesh.ArrayType.Vertex] = vertices;
        arrays[(int)Mesh.ArrayType.Normal] = normals;
        arrays[(int)Mesh.ArrayType.Index] = mesh.Indices;

        var arrayMesh = new ArrayMesh();
        arrayMesh.AddSurfaceFromArrays(Mesh.PrimitiveType.Triangles, arrays);

        var material = new StandardMaterial3D
        {
            AlbedoColor = RoadPaintColor,
            ShadingMode = BaseMaterial3D.ShadingModeEnum.Unshaded,
            CullMode = BaseMaterial3D.CullModeEnum.Disabled,
        };

        var instance = new MeshInstance3D { Mesh = arrayMesh, Name = name };
        instance.SetSurfaceOverrideMaterial(0, material);
        return instance;
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

        // docs/LIVE-CITY-VIEWERS-DESIGN.md §5, -TASKS.md D4 -- rebuilt every frame, in lockstep with the
        // MultiMesh instance indices above, so index i here means "car MultiMesh instance i" for exactly
        // this frame (a car's instance index is NOT stable across frames -- Reconstructor.Reconstruct
        // iterates `source.History`, a dictionary, so insertion/removal can reshuffle order -- the pick
        // path and the selection highlight both re-resolve by VehicleHandle every frame, never by a cached
        // index, which is exactly why this list is thrown away and rebuilt rather than diffed).
        _carHandles.Clear();
        _carWorldPositions.Clear();

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

            _carHandles.Add(v.Handle);
            _carWorldPositions.Add(origin);
        }
    }

    // docs/DEMO-CITY3D-DESIGN.md "#### Pedestrians (P7-3)" -- ONE ped MultiMeshInstance3D, the ped analog of
    // BuildCarMultiMesh: built once as an EMPTY MultiMesh (no peds at load), a unit mesh scaled per instance
    // by CityLib.PedTransform (plaza path) / UpdateLivePeds/UpdateReplayPeds (live-city path) into a slim
    // upright avatar; the per-frame Update* methods write the live transforms + regime colours.
    //
    // Visibility polish (docs/LIVE-CITY-VIEWERS-TASKS.md Stage D): a unit CYLINDER, not a unit BOX -- a
    // round cross-section reads as an upright figure rather than a slab, and (unlike a box) looks the same
    // from every yaw, which matters here since peds are drawn axis-aligned (no per-instance yaw, see
    // UpdatePeds/UpdateLivePeds/UpdateReplayPeds's own remarks on why heading is skipped). A LOW segment
    // count keeps the per-instance triangle cost negligible at the crowd sizes this demo renders (<=160).
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
            // unit cylinder (radius 0.5, height 1); per-instance transform scales it to the avatar dims
            Mesh = new CylinderMesh { TopRadius = 0.5f, BottomRadius = 0.5f, Height = 1f, RadialSegments = 10 },
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

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §2.1, -TASKS.md D3 -- the REPLAY analog of UpdateLivePeds: consumes
    // PedFrameTrack.PedsAt's plain neutral tuple (int Id, float X, float Y, float Z, byte Regime, string
    // AnimTag) -- deliberately Sim.LiveCity-free on the record/replay side (PedFrameTrack.cs's own doc
    // comment) -- rather than Sim.LiveCity.LiveCityPed. The `Regime` byte maps directly onto
    // Sim.LiveCity.PedRegime's own numeric values (LowPowerWalking=0, HighPower=1, Paused=2, guaranteed by
    // PedFrameTrack's doc comment), so a straight cast reproduces UpdateLivePeds' grey/orange/yellow
    // mapping without re-deriving it. Same build-once/grow-only/VisibleInstanceCount discipline.
    private void UpdateReplayPeds(
        IReadOnlyList<(int Id, float X, float Y, float Z, byte Regime, string AnimTag)> peds)
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

            var scale = new Vector3(PedRenderWidthMeters, PedRenderHeightMeters, PedRenderWidthMeters);
            var basis = Basis.Identity.Scaled(scale);
            var origin = new Vector3(gx, gy + PedRenderHeightMeters / 2f, gz);
            _pedMultiMesh.SetInstanceTransform(i, new Transform3D(basis, origin));

            var color = (Sim.LiveCity.PedRegime)p.Regime switch
            {
                Sim.LiveCity.PedRegime.HighPower => LiveCityPedHighPowerColor,
                Sim.LiveCity.PedRegime.Paused => LiveCityPedPausedColor,
                _ => LiveCityPedLowPowerColor,
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
    //
    // `heightFactor`/`backFactor` default to the original whole-network values (CameraHeightFactor/
    // CameraBackFactor) so every pre-existing caller (--scenario, --peds) is BYTE-IDENTICAL. Visibility
    // polish (docs/LIVE-CITY-VIEWERS-TASKS.md Stage D): the live-city callers pass a LOWER, more oblique
    // pair (see LiveCityCameraHeightFactor/BackFactor below) -- the original near-vertical satellite angle
    // reads true-scale cars/peds as a scatter of a few pixels each regardless of how tight the frame box is
    // (a top-down view of a person is a dot no matter the zoom); a lower angle shows their actual silhouette
    // height instead.
    // Returns the built Camera3D (task requirement: callers seed the interactive orbit controller from
    // whichever camera/pose this method just computed) -- every pre-existing call site either ignores the
    // return value or captures it into a local only used for the new SetupOrbitCamera call, so the method's
    // own behavior is otherwise unchanged.
    private Camera3D BuildCameraAndLight(
        (Vector3 Min, Vector3 Max) bbox, bool makeCurrent,
        float heightFactor = CameraHeightFactor, float backFactor = CameraBackFactor)
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

        var camPos = center + new Vector3(0f, extent * heightFactor, extent * backFactor);
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
        return camera;
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
        var focus = ComputeCloseCameraFocus(vehicles);

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

    // Extracted from UpdateCloseCameraFraming (docs/LIVE-CITY-VIEWERS-DESIGN.md camera-controller
    // deliverable) so SetupOrbitCamera can compute the SAME "what is the close camera looking at" point
    // once, at setup time, to seed the interactive orbit controller's initial focus -- task T1.6 Part C's
    // original priority order (signal-head centroid, else first vehicle, else network center) is unchanged.
    private Vector3 ComputeCloseCameraFocus(IReadOnlyList<CityLib.ReconstructedVehicle>? vehicles)
    {
        if (_signalHeads.Count > 0)
        {
            var sum = Vector3.Zero;
            foreach (var head in _signalHeads)
            {
                sum += head.HeadInstance.Position;
            }

            return sum / _signalHeads.Count;
        }

        if (vehicles is { Count: > 0 })
        {
            var v = vehicles[0];
            return new Vector3(v.X, v.Y, v.Z);
        }

        var (min, max) = _sceneBbox;
        return new Vector3((min.X + max.X) / 2f, (min.Y + max.Y) / 2f, (min.Z + max.Z) / 2f);
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

    // docs/LIVE-CITY-VIEWERS-DESIGN.md camera-controller deliverable -- seeds `_orbitController`/
    // `_orbitCamera` from whichever camera is this scene's "current" one at setup time: `_closeCamera` if
    // `--camera=close` built one (call this AFTER that block so `_closeCamera` is already non-null), else
    // the overview `Camera3D` `BuildCameraAndLight` just returned. `frameBbox` is the SAME box the caller
    // already framed the overview camera against (so the fallback "network center" focus, when no signal
    // heads/vehicles exist yet, matches what the eye already sees). `--cam-yaw`/`--cam-pitch`/`--cam-dist`/
    // `--cam-focus` debug overrides (below) replace the computed initial state 1:1 so a headless screenshot
    // from a rotated angle exercises the EXACT SAME transform the interactive controller uses every frame.
    private void SetupOrbitCamera(Camera3D overviewCamera, (Vector3 Min, Vector3 Max) frameBbox)
    {
        Camera3D activeCamera;
        Vector3 focus;

        if (_closeCamera is not null)
        {
            activeCamera = _closeCamera;
            focus = ComputeCloseCameraFocus(null);
        }
        else
        {
            activeCamera = overviewCamera;
            var (min, max) = frameBbox;
            focus = new Vector3((min.X + max.X) / 2f, (min.Y + max.Y) / 2f, (min.Z + max.Z) / 2f);
        }

        _orbitCamera = activeCamera;
        var pos = activeCamera.Position;
        _orbitController = BuildOrbitController((pos.X, pos.Y, pos.Z), (focus.X, focus.Y, focus.Z));

        // Capture the far-plane baseline (see the fields' doc comment): whatever Far the camera already
        // has (BuildCameraAndLight's extent*4/500m floor, or CloseCamera's flat 2000f) is a floor we never
        // shrink below, and the frameBbox's horizontal half-extent is the scene-size term ApplyOrbitCamera
        // adds on top of the live orbit distance every frame.
        _orbitBaseFar = activeCamera.Far;
        var (bboxMin, bboxMax) = frameBbox;
        _orbitSceneExtent = Mathf.Max(Mathf.Max(bboxMax.X - bboxMin.X, bboxMax.Z - bboxMin.Z), 10f);

        ApplyOrbitCamera(); // push the (possibly CLI-overridden) initial pose to the node right away
        GD.Print(
            $"Main: orbit camera ready on '{activeCamera.Name}' -- yaw={_orbitController.YawRad * 180f / Mathf.Pi:F1}deg " +
            $"pitch={_orbitController.PitchRad * 180f / Mathf.Pi:F1}deg dist={_orbitController.Distance:F1}m " +
            $"focus={_orbitController.Focus}. Controls: left-drag orbit, middle-drag/shift+left-drag pan, " +
            "wheel zoom, R/Home reset.");
    }

    // Seeds an OrbitCameraController from the given (cameraPos, focus) -- normally exactly the pose
    // BuildCameraAndLight/UpdateCloseCameraFraming already computed -- then lets any `--cam-yaw=<deg>`/
    // `--cam-pitch=<deg>`/`--cam-dist=<m>`/`--cam-focus=x,z` debug flag REPLACE the corresponding component
    // of that initial state (never additive) before the controller is actually constructed, so an override
    // becomes part of Reset()'s target too, not a one-off nudge that a later `R` press would undo.
    private static OrbitCameraController BuildOrbitController(
        (float X, float Y, float Z) cameraPos, (float X, float Y, float Z) focus)
    {
        var seeded = OrbitCameraController.FromLookAt(cameraPos, focus);

        var yaw = ParseCamYawArg() is { } yawDeg ? yawDeg * Mathf.Pi / 180f : seeded.YawRad;
        var pitch = ParseCamPitchArg() is { } pitchDeg ? pitchDeg * Mathf.Pi / 180f : seeded.PitchRad;
        var distance = ParseCamDistArg() ?? seeded.Distance;
        var focusOverride = ParseCamFocusArg();
        var focusX = focusOverride?.X ?? seeded.Focus.X;
        var focusZ = focusOverride?.Z ?? seeded.Focus.Z;

        return new OrbitCameraController(focusX, seeded.Focus.Y, focusZ, yaw, pitch, distance);
    }

    // Pushes `_orbitController`'s current (focus, yaw, pitch, distance) to `_orbitCamera`'s actual Godot
    // transform -- called once at setup (via SetupOrbitCamera) and then every render frame from each
    // Process*/RenderFrame body, so it picks up whatever the input handlers below just mutated. A no-op
    // when no orbit camera exists (video-wall mode, or before setup finishes) -- safe to call unconditionally.
    private void ApplyOrbitCamera()
    {
        if (_orbitCamera is null || _orbitController is null)
        {
            return;
        }

        var pos = _orbitController.CameraPosition();
        _orbitCamera.Position = new Vector3(pos.X, pos.Y, pos.Z);
        var focus = _orbitController.Focus;
        _orbitCamera.LookAt(new Vector3(focus.X, focus.Y, focus.Z), Vector3.Up);

        // Bug fix: track the far plane to the live orbit distance so dollying out never clips the scene
        // (see the `_orbitBaseFar`/`_orbitSceneExtent` fields' doc comment for the formula's reasoning).
        _orbitCamera.Far = Mathf.Max(_orbitBaseFar, (_orbitController.Distance * 2.5f) + _orbitSceneExtent);
    }

    // `--cam-yaw=<deg>`/`--cam-pitch=<deg>`/`--cam-dist=<m>` -- headless verification hooks (task
    // requirement): set the orbit controller's INITIAL yaw/pitch/distance so a screenshot from a rotated
    // angle proves the orbit transform actually renders, without needing a real mouse. Same
    // OS.GetCmdlineUserArgs() mechanism as every other `--foo=` arg in this file.
    private static float? ParseCamYawArg() => ParseFloatArg("--cam-yaw=");
    private static float? ParseCamPitchArg() => ParseFloatArg("--cam-pitch=");
    private static float? ParseCamDistArg() => ParseFloatArg("--cam-dist=");

    private static float? ParseFloatArg(string prefix)
    {
        foreach (var arg in OS.GetCmdlineUserArgs())
        {
            if (arg.StartsWith(prefix, StringComparison.Ordinal)
                && float.TryParse(
                    arg[prefix.Length..], System.Globalization.NumberStyles.Float,
                    System.Globalization.CultureInfo.InvariantCulture, out var value))
            {
                return value;
            }
        }

        return null;
    }

    // `--cam-focus=x,z` -- the ground-plane (X, Z) coordinate pair the orbit controller's focus starts at
    // (Y is left at whatever the computed default focus' height was); same `--cam-yaw`-style headless
    // verification hook, for re-centering the initial view rather than only rotating around the default
    // focus point.
    private static (float X, float Z)? ParseCamFocusArg()
    {
        const string prefix = "--cam-focus=";
        foreach (var arg in OS.GetCmdlineUserArgs())
        {
            if (!arg.StartsWith(prefix, StringComparison.Ordinal))
            {
                continue;
            }

            var parts = arg[prefix.Length..].Split(',');
            if (parts.Length == 2
                && float.TryParse(parts[0], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var x)
                && float.TryParse(parts[1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var z))
            {
                return (x, z);
            }
        }

        return null;
    }

    // `--hide-buildings` USER cmdline flag (buildings-visibility deliverable): a bare flag, same
    // OS.GetCmdlineUserArgs() mechanism as `--peds`/`--live-city`. Starts the buildings MultiMeshInstance3D
    // (when this mode builds one at all -- see `_buildingsNode`'s own remark) hidden; the runtime `B` key
    // (_UnhandledInput) toggles it from there regardless of how it started.
    private static bool ParseHideBuildingsArg()
    {
        foreach (var arg in OS.GetCmdlineUserArgs())
        {
            if (arg == "--hide-buildings")
            {
                return true;
            }
        }

        return false;
    }

    // Owner decision (docs/LIVE-CITY-VISUALS-NOTES.md): zones default OFF (hidden), opt-in via
    // `--show-zones` USER cmdline flag -- replaces the earlier `--hide-zones` flag now that hidden is the
    // default (a "hide" flag would be redundant). BuildZoneGround already starts the zone-tint layer
    // (`_zonesNode`, when built at all) hidden unconditionally; this flag is what a caller checks right
    // after to flip it visible at startup. The runtime `Z` key (_UnhandledInput) toggles it from there,
    // regardless of how it started.
    private static bool ParseShowZonesArg()
    {
        foreach (var arg in OS.GetCmdlineUserArgs())
        {
            if (arg == "--show-zones")
            {
                return true;
            }
        }

        return false;
    }

    // docs/LIVE-CITY-VISUALS-NOTES.md (tick-rate task): `--sim-hz=<N>` USER cmdline arg -- the live-city
    // coupled sim's tick rate (Hz), which LiveCityConfig.Dt = 1/Hz derives from (ReadyLiveCityLive). Same
    // `--foo=` OS.GetCmdlineUserArgs() mechanism as --scenario/--camera/--shot. Absent -> null
    // (ValidateSimHz's default, 2 Hz, applies).
    private static int? ParseSimHzArg()
    {
        const string prefix = "--sim-hz=";
        foreach (var arg in OS.GetCmdlineUserArgs())
        {
            if (arg.StartsWith(prefix, StringComparison.Ordinal)
                && int.TryParse(arg[prefix.Length..], NumberStyles.Integer, CultureInfo.InvariantCulture, out var hz))
            {
                return hz;
            }
        }

        return null;
    }

    // docs/LIVE-CITY-VISUALS-NOTES.md (tick-rate task): `--render-hz=<N>` USER cmdline arg -- Godot's
    // Engine.MaxFps cap, applied in _Ready (before mode dispatch) and INDEPENDENT of --sim-hz (render rate
    // is decoupled from sim rate by design -- the kinematic reconstructor interpolates between sparse sim
    // samples). Same mechanism as --sim-hz above. Absent -> null (ValidateRenderHz's default, 60).
    private static int? ParseRenderHzArg()
    {
        const string prefix = "--render-hz=";
        foreach (var arg in OS.GetCmdlineUserArgs())
        {
            if (arg.StartsWith(prefix, StringComparison.Ordinal)
                && int.TryParse(arg[prefix.Length..], NumberStyles.Integer, CultureInfo.InvariantCulture, out var hz))
            {
                return hz;
            }
        }

        return null;
    }

    // Mirrors src/Sim.Viewer/Program.cs's ValidateSimHz one-for-one (same allowed set {1,2,5,10,20}, same
    // clamp-to-nearest-with-a-reported-message behaviour rather than a hard failure over a typo'd flag) --
    // just logged through GD.Print/GD.PrintErr instead of Console, since this is the Godot side.
    private static int ValidateSimHz(int? requested)
    {
        const int defaultHz = 2;
        if (requested is not { } hz)
        {
            return defaultHz;
        }

        var allowed = new[] { 1, 2, 5, 10, 20 };
        if (Array.IndexOf(allowed, hz) >= 0)
        {
            return hz;
        }

        var nearest = allowed[0];
        var bestDiff = Math.Abs(hz - nearest);
        foreach (var candidate in allowed)
        {
            var diff = Math.Abs(hz - candidate);
            if (diff < bestDiff)
            {
                nearest = candidate;
                bestDiff = diff;
            }
        }

        GD.PrintErr($"Main: --sim-hz={hz} is not one of {{1,2,5,10,20}} -- clamped to {nearest} Hz.");
        return nearest;
    }

    // Mirrors src/Sim.Viewer/Program.cs's ValidateRenderHz -- clamped (not a discrete set) to [15,60]; 60
    // is the hard cap, 15 a usability floor. Absent -> 60.
    private static int ValidateRenderHz(int? requested)
    {
        var hz = requested ?? 60;
        var clamped = Math.Clamp(hz, 15, 60);
        if (clamped != hz)
        {
            GD.PrintErr($"Main: --render-hz={hz} clamped to {clamped} (allowed 15..60).");
        }

        return clamped;
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

    // docs/LIVE-CITY-VIEWERS-TASKS.md D3 -- `--live-city --replay <file.simrec>` (a TWO-TOKEN arg, per the
    // task spec, unlike --shot=/--scenario=/--camera='s `=`-joined form) selects the file-backed replay
    // path instead of a live LiveCitySource. `--replay=<file>` (the `=`-joined form, for consistency with
    // this file's other args/shell habits that quote a single token) is also accepted. Returns null when
    // absent -- ReadyLiveCity's `_replay = replayPath is not null` is the single source of truth for which
    // branch runs.
    private static string? ParseReplayArg()
    {
        const string eqPrefix = "--replay=";
        var args = OS.GetCmdlineUserArgs();
        for (var i = 0; i < args.Length; i++)
        {
            var arg = args[i];
            if (arg.StartsWith(eqPrefix, StringComparison.Ordinal))
            {
                var v = arg[eqPrefix.Length..].Trim();
                return v.Length > 0 ? v : null;
            }

            if (arg == "--replay" && i + 1 < args.Length)
            {
                return args[i + 1];
            }
        }

        return null;
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
