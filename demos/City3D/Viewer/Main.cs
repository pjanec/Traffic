using System;
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

    // How far above/behind the network's centre (as a multiple of its largest XZ extent) the framing
    // camera sits -- an "angled bird's-eye" per design "Buildings"/"Roads" desktop-check framing.
    private const float CameraHeightFactor = 0.75f;
    private const float CameraBackFactor = 0.75f;

    private SimSource? _sim;
    private Reconstructor? _reconstructor;
    private double _accumulator;
    private int _frame;
    private string? _shotPath;

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

        var scenarioDir = Path.Combine(repoRoot, "scenarios", "09-traffic-light");
        var netPath = Path.Combine(scenarioDir, "net.net.xml");
        var rouPath = Path.Combine(scenarioDir, "rou.rou.xml");
        var cfgPath = Path.Combine(scenarioDir, "config.sumocfg");

        if (!File.Exists(netPath) || !File.Exists(rouPath) || !File.Exists(cfgPath))
        {
            GD.PrintErr(
                $"Main: scenario 'scenarios/09-traffic-light' not found under repo root '{repoRoot}' " +
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
        GD.Print($"Main: loaded scenario '09-traffic-light' from '{scenarioDir}'.");

        var bbox = BuildRoadMeshes(_sim.Network);
        BuildCameraAndLight(bbox);

        _shotPath = ParseShotArg();
        if (_shotPath is not null)
        {
            GD.Print($"Main: --shot requested, will capture to '{_shotPath}'.");
            CaptureScreenshotAsync(_shotPath);
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

        if (vehicles.Count > 0)
        {
            var v = vehicles[0];
            GD.Print(
                $"Main: frame={_frame} simTime={_sim.Time:F2} vehicles={vehicles.Count} " +
                $"v0=(x={v.X:F2}, z={v.Z:F2}, yaw={v.YawRad:F3})");
        }
        else
        {
            GD.Print($"Main: frame={_frame} simTime={_sim.Time:F2} vehicles=0");
        }

        _frame++;
        if (_frame >= QuitAfterFrames)
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

    // docs/DEMO-CITY3D-DESIGN.md "Procedural scene generation -> Roads" desktop check framing: an
    // above-and-back angled bird's-eye view over the whole network's Godot-space XZ bounds, plus a
    // DirectionalLight3D so the asphalt is actually visible.
    private void BuildCameraAndLight((Vector3 Min, Vector3 Max) bbox)
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
        camera.Current = true;
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

    // Waits a couple of real rendered frames (so the just-built meshes/camera/light are actually on
    // screen), then saves the viewport to PNG and quits. Guarded for headless (no rendering driver, e.g.
    // a plain `--headless` run under run-smoke.sh): GetViewport().GetTexture().GetImage() returns null (or
    // throws) when there is no real framebuffer, in which case this reports the gap instead of crashing.
    private async void CaptureScreenshotAsync(string path)
    {
        try
        {
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
