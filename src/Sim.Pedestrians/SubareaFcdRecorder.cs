using System;
using System.Collections.Generic;
using System.IO;
using Sim.Core;
using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians.Demand;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation.Bake;

namespace Sim.Pedestrians;

// P8-3xP8-4 end-to-end recorder (docs/COORDINATION-pedestrian-x-subarea.md §3; PEDESTRIAN-P8-3/-P8-4 designs):
// drives a committed sub-area box with the auto-deduced weighted demand (P8-3) sized by the density knob
// (P8-4a) and records the live crowd's trajectory as a SUMO `<person>` FCD stream (PersonFcdWriter). The
// output drops into the shared car+ped replay beside the box's vehicle FCD (P8-5, sub-area session).
//
// Deterministic and hermetic: consumes only the committed box (net.xml + manifest.json + pois.json), no SUMO,
// no engine vehicles. Every ped's O/D and timing come from seeded VehicleRng streams, so the same box + same
// options produce a byte-identical FCD. Appearance-legitimate by construction: every spawn/despawn is a
// fringe/POI endpoint (the P8-3xP8-2 synergy), so the recorded stream never pops a ped on an open sidewalk.
public static class SubareaFcdRecorder
{
    public sealed record Options
    {
        // Density dial in [0,1] fed to PedDensityKnob (0 = empty, 1 = the LoS-C safe maximum). A modest
        // default keeps the demo a watchable crowd rather than the box's full safe capacity.
        public double Dial { get; init; } = 0.03;

        public double Seconds { get; init; } = 120.0;

        // Internal sim step -- fine for smooth PathArc/ORCA motion.
        public double Dt { get; init; } = 0.1;

        // Emit cadence: one `<timestep>` per FrameDt of sim time. MUST match the vehicle FCD's step so the
        // shared replay's timesteps align and cars don't blink (SUBAREA-SHARED-REPLAY-CONTRACT.md: "the step
        // must match the vehicle FCD's step ... Merge is keyed on the exact time value"). The box's
        // scenario.sumocfg uses step-length=1.0, so the default is 1.0. Should be an integer multiple of Dt.
        public double FrameDt { get; init; } = 1.0;

        public ulong Seed { get; init; } = 20240719UL;

        public double SafePedsPerWalkableKm { get; init; } = PedDensityKnob.SafePedsPerWalkableKmDefault;
        public double MeanTripSeconds { get; init; } = PedDensityKnob.MeanTripSecondsDefault;
        public double FringeWeight { get; init; } = 1.0;

        public double MaxSpeed { get; init; } = 1.4;
        public double Radius { get; init; } = 0.3;
        public double ArrivalRadius { get; init; } = 0.5;
    }

    public sealed record Result(
        int Frames,
        int PeakLive,
        int Spawns,
        int Arrivals,
        int PopulationCap,
        double SpawnRatePerSecond,
        double WalkableLengthKm,
        int Endpoints,
        // P8-1b diagnostics (docs/PEDESTRIAN-P8-1B-NAVMESH-CONNECTIVITY-DESIGN.md): navmesh connectivity health.
        // A well-connected crop is `ConnectedComponents` == 1 (or a few); ~1000 means the bake fragmented and
        // O/D routing will starve the crowd. `UnreachableSkips` is how many spawn draws were rejected as
        // unroutable (cross-component) -- near 0 on a healthy crop, near the spawn count on a fragmented one.
        int WalkablePolygons,
        int ConnectedComponents,
        int UnreachableSkips);

    // Records the person FCD for `boxDir` into `fcdOut`. `boxDir` must contain net.xml, manifest.json,
    // pois.json (the committed handoff layout). The writer is NOT disposed here -- the caller owns it.
    public static Result Record(string boxDir, PersonFcdWriter fcdOut, Options? options = null)
    {
        if (boxDir is null)
        {
            throw new ArgumentNullException(nameof(boxDir));
        }

        if (fcdOut is null)
        {
            throw new ArgumentNullException(nameof(fcdOut));
        }

        var opt = options ?? new Options();

        var network = PedNetworkParser.Load(Path.Combine(boxDir, "net.xml"));
        var pois = PedPoiReader.LoadJson(Path.Combine(boxDir, "pois.json"));
        var manifest = SubareaManifest.Load(Path.Combine(boxDir, "manifest.json"));

        var polygons = WalkablePolygonBaker.Bake(network);
        var nav = new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));
        var manager = new PedLodManager(nav, new PedPublisher(), arriveRadius: opt.Radius, dwellSeconds: 1.0);

        var fringe = SubareaDemand.FringeEndpointsFromNetwork(network, manifest.WalkableFringeEdges);
        var demandSet = SubareaDemand.Build(pois, fringe, opt.FringeWeight);
        var knob = PedDensityKnob.ForNetwork(network, opt.Dial, opt.SafePedsPerWalkableKm, opt.MeanTripSeconds);

        var config = new PedDemandConfig
        {
            Origins = Array.Empty<Vec2>(),
            Destinations = Array.Empty<Vec2>(),
            SpawnRatePerSecond = knob.SpawnRatePerSecond,
            PopulationCap = knob.PopulationCap,
            Seed = opt.Seed,
            MaxSpeed = opt.MaxSpeed,
            Radius = opt.Radius,
            ArrivalRadius = opt.ArrivalRadius,
            WeightedEndpoints = demandSet,
        };

        var demand = new PedDemand(config, nav, manager);
        var field = new InterestField();
        var noEntities = Array.Empty<WorldDisc>();

        // Previous EMITTED position + heading per live ped, so speed/angle are finite-differenced across the
        // emit cadence (FrameDt), matching the sampled step. Deterministic (PathArc pose is a pure function
        // of time). A ped's first emitted frame reports speed 0 and its prior/0 angle (no prior sample yet).
        var lastPos = new Dictionary<int, Vec2>();
        var lastAngle = new Dictionary<int, double>();

        // Internal steps per emitted frame (FrameDt/Dt). Keeps the sim smooth (Dt) while emitting on the
        // vehicle FCD's coarser grid (FrameDt) so the shared replay's frames align.
        var ratio = Math.Max(1, (int)Math.Round(opt.FrameDt / opt.Dt, MidpointRounding.AwayFromZero));
        var steps = (int)Math.Round(opt.Seconds / opt.Dt, MidpointRounding.AwayFromZero);
        var peakLive = 0;
        var emittedFrames = 0;

        void EmitFrame(double t)
        {
            fcdOut.BeginTimestep(t);
            foreach (var id in demand.LiveIds)
            {
                var pos = manager.PositionOf(id, t);
                double speed = 0.0;
                var angle = lastAngle.TryGetValue(id, out var prevA) ? prevA : 0.0;
                if (lastPos.TryGetValue(id, out var prev))
                {
                    var delta = pos - prev;
                    speed = opt.FrameDt > 0.0 ? delta.Abs / opt.FrameDt : 0.0;
                    angle = PersonFcdWriter.BearingDegrees(pos.X - prev.X, pos.Y - prev.Y, fallback: angle);
                }

                fcdOut.WritePerson($"p{id}", pos.X, pos.Y, angle, speed);
                lastPos[id] = pos;
                lastAngle[id] = angle;
            }

            fcdOut.EndTimestep();
            emittedFrames++;
            if (demand.LiveCount > peakLive)
            {
                peakLive = demand.LiveCount;
            }
        }

        // Initial frame at t=0 (aligns with SUMO begin=0; typically empty before the first spawn).
        EmitFrame(0.0);

        for (var i = 0; i < steps; i++)
        {
            // Non-accumulating, cleanly-rounded times so labels are "1", "2", ... (no float drift over a long
            // run). Step advances [t0, t0+Dt); a frame is emitted only when the boundary lands on the FrameDt
            // grid (every `ratio`-th internal step).
            var t0 = Math.Round(i * opt.Dt, 6);
            demand.Step(t0, opt.Dt, field, noEntities);

            if ((i + 1) % ratio == 0)
            {
                EmitFrame(Math.Round((i + 1) * opt.Dt, 6));
            }
        }

        return new Result(
            WalkablePolygons: polygons.Count,
            ConnectedComponents: nav.ConnectedComponentCount(),
            UnreachableSkips: demand.UnreachableSkipCount,
            Frames: emittedFrames,
            PeakLive: peakLive,
            Spawns: demand.SpawnCount,
            Arrivals: demand.ArrivalCount,
            PopulationCap: knob.PopulationCap,
            SpawnRatePerSecond: knob.SpawnRatePerSecond,
            WalkableLengthKm: knob.WalkableLengthKm,
            Endpoints: demandSet.Count);
    }
}
