using Sim.Core.Orca;
using Sim.Evac;
using Sim.Pedestrians;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation.Bake;
using Xunit;
using Xunit.Abstractions;

namespace Sim.ParityTests;

// P5-1(B) (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §6 "evac = specialization"):
// behavioural validation of EvacDistrictDirector -- the pedestrian panic-evacuation built on the
// Sim.Pedestrians PedestrianWorld facade over the REAL walkable P5-PRE evac-district net (a 3x3
// sidewalk grid, incident at the centre (100,100), four corner safe zones -- see
// scenarios/_ped/evac-district/NOTES.md and EvacDistrictNetTests, which prove the net itself bakes
// and routes correctly). This is a wholly separate, additive suite: the legacy car-centric
// EvacDirector and its own Evac*DemoTests are untouched by anything here.
public class EvacDistrictDemoTests
{
    private readonly ITestOutputHelper _out;
    public EvacDistrictDemoTests(ITestOutputHelper output) => _out = output;

    private static readonly Vec2[] SafeZones = { new(0, 0), new(200, 0), new(0, 200), new(200, 200) };
    private static readonly Incident DefaultIncident = new(X: 100.0, Y: 100.0, StartTime: 5.0, Radius: 60.0);

    private static SumoNavMesh BuildNav()
    {
        var netPath = Path.Combine(RepoRoot(), "scenarios", "_ped", "evac-district", "net.net.xml");
        var net = PedNetworkParser.Load(netPath);
        var polygons = WalkablePolygonBaker.Bake(net);
        return new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));
    }

    private static EvacDistrictDirector BuildDirector(EvacDistrictConfig? config = null, Incident? incident = null) =>
        new(BuildNav(), incident ?? DefaultIncident, SafeZones, config);

    private static string RepoRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "Traffic.sln")))
        {
            dir = dir.Parent;
        }

        return dir?.FullName
            ?? throw new InvalidOperationException("Could not locate repo root (Traffic.sln not found above test assembly).");
    }

    // ----- routed, not radial -----

    [Fact]
    public void PanickedPeds_RouteAlongSidewalks_NotRadially()
    {
        var director = BuildDirector(new EvacDistrictConfig { PedCount = 40, PanicRadius = 200.0 });

        for (var step = 0; step < 400; step++)
        {
            director.Step(1.0);
        }

        Assert.True(director.PanickedCount > 0, "expected the whole ambient population to panic");

        var measured = 0;
        for (var id = 1; id <= director.PedestrianCount; id++)
        {
            if (!director.IsPanicked(id))
            {
                continue;
            }

            var panicPos = director.PanicPositionOf(id);
            var safeZone = director.SafeZoneOf(id);
            var straight = (safeZone - panicPos).Abs;

            // Only assert the routed/radial ratio for peds that panicked well inside the district,
            // away from EVERY perimeter street (the district is the [0,200]x[0,200] box, NOTES.md) --
            // a ped that panics close to one of the four boundary streets sits right next to a
            // corner's own junction/walking-area apron, where the shortest legal sidewalk route can
            // legitimately be nearly straight (no block to bend around at all -- confirmed against
            // this net via SumoNavMesh.FindPath directly). A ped panicking deep inside the district,
            // by contrast, MUST thread along the sidewalk grid to reach any corner, so its ratio is
            // the informative case (mirrors EvacDistrictNetTests' own centre-to-corner measurement,
            // generalized over many panic points instead of one fixed centre point).
            var cornerMargin = Math.Min(Math.Min(panicPos.X, 200.0 - panicPos.X), Math.Min(panicPos.Y, 200.0 - panicPos.Y));
            if (cornerMargin < 45.0)
            {
                continue;
            }

            var trace = director.TraceOf(id);
            var arc = 0.0;
            for (var i = 0; i + 1 < trace.Count; i++)
            {
                arc += (trace[i + 1] - trace[i]).Abs;
            }

            _out.WriteLine($"[routed-vs-radial] id={id} straight={straight:F1} arc={arc:F1} ratio={arc / straight:F2}");
            Assert.True(arc > 1.2 * straight,
                $"ped {id}: routed arc {arc:F1} m should exceed 1.2x the straight-line {straight:F1} m " +
                "(i.e. route AROUND blocks, not through them)");
            measured++;
        }

        Assert.True(measured > 0, "expected at least one panicked ped to have panicked far enough from its safe zone to measure");
    }

    // ----- forced promotion -----

    [Fact]
    public void PanickedPed_IsForcedHighPower_WhileFleeing()
    {
        var director = BuildDirector(new EvacDistrictConfig { PedCount = 20, PanicRadius = 200.0 });

        for (var step = 0; step < 15; step++)
        {
            director.Step(1.0);
        }

        Assert.True(director.PanickedCount > 0, "expected some peds to have panicked by now");

        var checkedAny = false;
        for (var id = 1; id <= director.PedestrianCount; id++)
        {
            if (director.IsPanicked(id) && !director.IsEscaped(id))
            {
                Assert.Equal(PedDrModel.FreeKinematic, director.ModelOf(id));
                checkedAny = true;
            }
        }

        Assert.True(checkedAny, "expected at least one panicked-but-not-yet-escaped ped to check ModelOf on");
    }

    // ----- everyone escapes -----

    [Fact]
    public void EveryPanickedPed_EventuallyEscapes()
    {
        var director = BuildDirector(new EvacDistrictConfig { PedCount = 40, PanicRadius = 200.0 });

        for (var step = 0; step < 600; step++)
        {
            director.Step(1.0);
        }

        _out.WriteLine(
            $"panicked={director.PanickedCount} escaped={director.EscapedCount} of population={director.PedestrianCount}");
        Assert.True(director.PanickedCount > 0, "expected the whole ambient population to panic");
        Assert.Equal(director.PanickedCount, director.EscapedCount);
    }

    // ----- nearest safe zone -----

    [Fact]
    public void NearestSafeZone_PicksTheClosestCorner_NotAFarOne()
    {
        // Direct, isolated check of the rule itself, at points clearly closer to one corner than to
        // any other -- independent of any particular ped's spawn/panic geometry.
        Assert.Equal(new Vec2(0, 0), EvacDistrictDirector.NearestSafeZone(new Vec2(20, 20), SafeZones));
        Assert.Equal(new Vec2(200, 0), EvacDistrictDirector.NearestSafeZone(new Vec2(180, 20), SafeZones));
        Assert.Equal(new Vec2(0, 200), EvacDistrictDirector.NearestSafeZone(new Vec2(20, 180), SafeZones));
        Assert.Equal(new Vec2(200, 200), EvacDistrictDirector.NearestSafeZone(new Vec2(180, 180), SafeZones));

        // And confirm the director actually ASSIGNS panicked peds to that same nearest zone -- for
        // every ped whose panic position has a clear (unambiguous) nearest corner, its assigned
        // SafeZoneOf must match the brute-force nearest, not some other (farther) corner.
        var director = BuildDirector(new EvacDistrictConfig { PedCount = 40, PanicRadius = 200.0 });
        for (var step = 0; step < 400; step++)
        {
            director.Step(1.0);
        }

        var checkedAny = false;
        for (var id = 1; id <= director.PedestrianCount; id++)
        {
            if (!director.IsPanicked(id))
            {
                continue;
            }

            var panicPos = director.PanicPositionOf(id);
            var distances = SafeZones.Select(z => (z - panicPos).Abs).OrderBy(d => d).ToArray();

            // Skip ambiguous panic points (nearest and second-nearest corner nearly tied).
            if (distances[1] - distances[0] < 10.0)
            {
                continue;
            }

            var expected = EvacDistrictDirector.NearestSafeZone(panicPos, SafeZones);
            Assert.Equal(expected, director.SafeZoneOf(id));
            checkedAny = true;
        }

        Assert.True(checkedAny, "expected at least one panicked ped with an unambiguous nearest corner");
    }

    // ----- determinism -----

    [Fact]
    public void SameConfig_ProducesIdenticalEscapedAndPositionStream()
    {
        string RunAndSign()
        {
            var director = BuildDirector(new EvacDistrictConfig { PedCount = 30, PanicRadius = 200.0 });
            var sb = new System.Text.StringBuilder();

            for (var step = 0; step < 500; step++)
            {
                director.Step(1.0);

                sb.Append(director.PanickedCount).Append('|').Append(director.EscapedCount).Append(';');
                for (var id = 1; id <= director.PedestrianCount; id++)
                {
                    if (director.IsEscaped(id))
                    {
                        sb.Append('E');
                    }
                    else if (director.IsPanicked(id))
                    {
                        var p = director.PositionOf(id);
                        sb.Append(p.X.ToString("R")).Append(',').Append(p.Y.ToString("R"));
                    }

                    sb.Append(';');
                }
            }

            return sb.ToString();
        }

        var sigA = RunAndSign();
        var sigB = RunAndSign();
        Assert.Equal(sigA, sigB);
    }
}
