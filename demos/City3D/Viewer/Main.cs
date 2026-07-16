using System;
using System.Collections.Generic;
using System.IO;
using CityLib;
using Godot;
using Sim.Ingest;

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

    public override void _Ready()
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
        GD.Print($"Main: loaded scenario '{scenarioRel}' from '{scenarioDir}'.");

        _cameraMode = ParseCameraArg();

        var roadBbox = BuildRoadMeshes(_sim.Network);
        var buildingBbox = BuildBuildings(_sim.Network);
        var bbox = CombineBbox(roadBbox, buildingBbox);
        _sceneBbox = bbox;
        // The overview camera is still always built (env + light + the default whole-network framing
        // run-smoke.sh relies on) -- `--camera=close` just leaves it non-Current in favour of the close
        // camera built below, rather than forking the environment/light setup.
        BuildCameraAndLight(bbox, makeCurrent: _cameraMode != "close");
        _carMultiMesh = BuildCarMultiMesh();

        if (_cameraMode == "close")
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

    public override void _Process(double delta)
    {
        if (_sim is null || _reconstructor is null)
        {
            // _Ready already reported the error and requested quit; nothing more to do this frame.
            return;
        }

        // Fixed sim-cadence accumulator: advance the sim in whole SimStepSeconds increments regardless of
        // the (headless-dummy-renderer) frame rate, while reconstruction still runs every _Process call.
        _accumulator += delta;
        while (_accumulator >= SimStepSeconds)
        {
            _sim.Tick();
            _accumulator -= SimStepSeconds;
        }

        var vehicles = _reconstructor.Reconstruct(_sim.Source, _sim.LocalLanes, PlayoutDelaySeconds);
        UpdateCars(vehicles);

        var tlStateByLane = _sim.Source.TlStateByLane;
        if (!_signalHeadsBuilt && tlStateByLane.Count > 0)
        {
            BuildTrafficLights(_sim.Network, tlStateByLane);
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

        if (vehicles.Count > 0)
        {
            var v = vehicles[0];
            GD.Print(
                $"Main: frame={_frame} simTime={_sim.Time:F2} vehicles={vehicles.Count} cars={vehicles.Count} " +
                $"v0=(x={v.X:F2}, z={v.Z:F2}, yaw={v.YawRad:F3})");
        }
        else
        {
            GD.Print($"Main: frame={_frame} simTime={_sim.Time:F2} vehicles=0 cars=0");
        }

        _frame++;

        // Skip the frame-count auto-quit while a screenshot capture is pending: CaptureScreenshotAsync
        // owns quitting in that case (docs/DEMO-CITY3D-DESIGN.md task T1.5 part C -- a real `--shot-delay`
        // lets enough WALL-CLOCK time pass for the DrClock-driven sim/render to actually populate the
        // roads with cars before the shot is taken; without this guard, a delay longer than
        // QuitAfterFrames' own real-time span at an uncapped frame rate would race the engine into
        // quitting before the delayed capture ever fires).
        if (_frame >= QuitAfterFrames && _shotPath is null)
        {
            GD.Print($"Main: reached {QuitAfterFrames} frames, quitting.");
            _sim.Dispose();
            _sim = null;
            GetTree().Quit();
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

    // docs/DEMO-CITY3D-DESIGN.md "Procedural scene generation -> Roads" / task T1.3. Static geometry, built
    // ONCE at load (design: "two clocks: static geometry ... built once at load"). CityLib.RoadMeshBuilder
    // does all the math (segment normals, miter offsets, triangulation, the SUMO->Godot transform); this
    // method only converts its plain float[]/int[] arrays into a Godot ArrayMesh per lane. Returns the
    // Godot-space bounding box of every emitted vertex, so the camera can frame the whole network.
    private (Vector3 Min, Vector3 Max) BuildRoadMeshes(NetworkModel network)
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

        foreach (var (handle, mesh) in RoadMeshBuilder.BuildAll(network, includeInternal: true))
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

        GD.Print($"Main: built {laneCount} road ribbon mesh(es) from {network.LanesByHandle.Count} lane(s).");

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

    // docs/DEMO-CITY3D-DESIGN.md "Procedural scene generation -> Traffic lights" / task T1.6 Part B.
    // Called ONCE (lazily, the first _Process frame `sim.Source.TlStateByLane` is non-empty --
    // geometry/TL state only exist once the in-process bus has been pumped at least once, see
    // CityLib.SimSource.Tick). CityLib.TrafficLightPlacer does all the placement math (downstream-end +
    // right-edge nudge + the SUMO->Godot transform); this method only turns each plain SignalHead struct
    // into a pole MeshInstance3D + a head MeshInstance3D with its OWN StandardMaterial3D (one per head,
    // not shared -- each head's emissive colour is rewritten independently every frame in
    // UpdateTrafficLights, unlike the shared asphalt/building/car materials which never change per-instance).
    private void BuildTrafficLights(Sim.Ingest.NetworkModel network, IReadOnlyDictionary<int, byte> tlStateByLane)
    {
        var heads = TrafficLightPlacer.Place(network, tlStateByLane.Keys);
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
