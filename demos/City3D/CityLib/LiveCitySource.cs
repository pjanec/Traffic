using System;
using System.Collections.Generic;
using Sim.Core;
using Sim.Ingest;
using Sim.LiveCity;
using Sim.Replication;

namespace CityLib;

// docs/LIVE-CITY-VIEWERS-DESIGN.md §6, -TASKS.md Stage D (D1) -- the LOCAL, in-process LIVE-CITY data
// path: wraps a Sim.LiveCity.LiveCitySim (the shared coupled cars+peds+crossing-yield host,
// docs/LIVE-CITY-VIEWERS-DESIGN.md §1) and exposes exactly the read-back surface the Viewer glue needs.
// Mirrors SimSource.cs's shape one-for-one (Tick/Source/LocalLanes/Network/Time) plus the ped read-back
// SimSource has no analog for (Peds). Per the design's "key reuse insight": the CAR render path is
// UNCHANGED from SimSource's -- Source/LocalLanes feed the SAME CityLib.Reconstructor the --scenario path
// uses, so cars get identical DR/kinematic smoothing in live-city mode, no new car-rendering code at all.
public sealed class LiveCitySource : IDisposable
{
    private readonly LiveCitySim _sim;

    // repoRoot-relative convenience constructor -- LiveCityConfig.ForRepoRoot resolves the pinned
    // scenarios/_ped/demo_city/box dataset dir + the LIVECITY_CARS/LCMIN/YIELD env-var overrides, exactly
    // as SumoSharp.LiveCity's own reference caller does.
    public LiveCitySource(string repoRoot)
        : this(LiveCityConfig.ForRepoRoot(repoRoot))
    {
    }

    public LiveCitySource(LiveCityConfig cfg)
    {
        Crop = (cfg.X0, cfg.Y0, cfg.X1, cfg.Y1);
        _sim = new LiveCitySim(cfg);
    }

    // The X0/Y0/X1/Y1 crop rectangle LiveCityConfig steps cars/peds within (SUMO metres). `Network`
    // (below) is the FULL parsed net.xml -- e.g. scenarios/_ped/demo_city/box's net.xml spans
    // 4750x4750m, of which the crop is only ~840x840m -- so a caller that wants a legible scene (road
    // meshes + camera framing) must filter/frame to THIS rect, not Network's own (whole-net) bounding box.
    public (double X0, double Y0, double X1, double Y1) Crop { get; }

    public NetworkModel Network => _sim.Network;

    // The LOCAL, Z-aware lane source (Lane.ShapeZ-carrying) -- same type SimSource.LocalLanes exposes, so
    // RoadMeshBuilder/Reconstructor honor the net's elevation on the live-city path exactly as they do on
    // the --scenario path (docs/LIVE-CITY-VIEWERS-TASKS.md D2).
    public NetworkLaneSource LocalLanes => _sim.LocalLanes;

    // The transport-neutral car read side -- IReplicationSource, exactly what SimSource.Source exposes, so
    // the SAME CityLib.Reconstructor + KinematicReconstructor render live-city cars unchanged.
    public IReplicationSource Source => _sim.VehicleSource;

    public double Time => _sim.Time;

    // The ped read-back for one render frame -- cars+peds are sampled together off LiveCitySim's last
    // stepped frame (LiveCitySim "does not render, only steps and samples", design §1); the Viewer maps
    // each ped's (X,Y,Z) through CoordinateTransform.SumoToGodot itself (no CityLib ped-transform reuse
    // needed -- LiveCityPed's PedRegime differs from CityLib.PedRegime by design, see PedRegime's doc
    // comment in Sim.LiveCity/LiveCitySnapshot.cs).
    public IReadOnlyList<LiveCityPed> Peds => _sim.Sample().Peds;

    // Advances the coupled sim one Dt=0.5s tick (LiveCityConfig.Dt) and publishes the resulting frame onto
    // the car wire (LiveCitySim.Step()'s own responsibility) -- mirrors SimSource.Tick()'s one-line shape.
    public void Tick() => _sim.Step();

    public void Dispose() => _sim.Dispose();
}
