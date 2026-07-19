using Sim.Core.Orca;
using Sim.Evac;
using Sim.Pedestrians;
using Sim.Pedestrians.Navigation.Bake;
using Xunit;
using Xunit.Abstractions;

namespace Sim.ParityTests;

// P6-3 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-OVERVIEW.md §2.4 "Evacuation compatibility"): the
// Req4 property test. Lives here (not in Sim.Pedestrians.Tests) because it needs Sim.Evac's
// EvacDistrictDirector, which Sim.Pedestrians.Tests does not reference. Setup mirrors
// EvacDistrictDemoTests.cs (same real walkable P5-PRE evac-district net, same PedNetworkParser /
// WalkablePolygonBaker / SumoNavMesh construction) -- this test only adds the SEEDED-incident-position
// sweep (>= 5 distinct incident epicentres, a fixed config array, no System.Random) that P6-3 asks for,
// reusing EvacDistrictDemoTests' own routed-vs-radial and full-escape assertions verbatim per seed.
public class Req4EvacuationRequirementTests
{
    private readonly ITestOutputHelper _out;
    public Req4EvacuationRequirementTests(ITestOutputHelper output) => _out = output;

    private static readonly Vec2[] SafeZones = { new(0, 0), new(200, 0), new(0, 200), new(200, 200) };

    private static SumoNavMesh BuildNav()
    {
        var netPath = Path.Combine(RepoRoot(), "scenarios", "_ped", "evac-district", "net.net.xml");
        var net = PedNetworkParser.Load(netPath);
        var polygons = WalkablePolygonBaker.Bake(net);
        return new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));
    }

    private static EvacDistrictDirector BuildDirector(Incident incident, EvacDistrictConfig? config = null) =>
        new(BuildNav(), incident, SafeZones, config);

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

    // Seeded incident epicentres -- 5 distinct points scattered around the interior of the district's
    // [0,200]x[0,200] box (NOTES.md), none at the exact centre this suite's own EvacDistrictDemoTests
    // already covers. PanicRadius stays large enough (200, the default) to cover the whole box from any
    // of these epicentres, so every config still panics (and must therefore evacuate) the WHOLE ambient
    // population -- the load-bearing invariant is not weakened by where the incident happens to sit.
    private static readonly Incident[] SeededIncidents =
    {
        new(X: 100.0, Y: 100.0, StartTime: 5.0, Radius: 200.0), // the centre (baseline)
        new(X: 60.0, Y: 140.0, StartTime: 5.0, Radius: 200.0),
        new(X: 150.0, Y: 70.0, StartTime: 5.0, Radius: 200.0),
        new(X: 40.0, Y: 40.0, StartTime: 5.0, Radius: 200.0),
        new(X: 170.0, Y: 160.0, StartTime: 5.0, Radius: 200.0),
    };

    [Fact]
    public void Req4_Evacuation_EveryPanickedPedEscapes_AndRoutesAlongSidewalks_AcrossSeededIncidentPositions()
    {
        const int pedCount = 24;
        const int steps = 600;

        for (var seed = 0; seed < SeededIncidents.Length; seed++)
        {
            var incident = SeededIncidents[seed];
            var director = BuildDirector(incident, new EvacDistrictConfig { PedCount = pedCount, PanicRadius = 200.0 });

            for (var step = 0; step < steps; step++)
            {
                director.Step(1.0);
            }

            Assert.True(director.PanickedCount > 0, $"seed {seed} (incident=({incident.X},{incident.Y})): expected the whole ambient population to panic");

            // Invariant 1 (Req4 "every panicked ped escape"): EscapedCount == PanickedCount.
            Assert.Equal(director.PanickedCount, director.EscapedCount);

            // Invariant 2 (Req4 "routed not radial"): for at least one panicked ped that panicked well
            // inside the district (away from every perimeter street, mirroring EvacDistrictDemoTests'
            // own cornerMargin filter), its travelled arc exceeds 1.2x the straight-line distance to its
            // safe zone -- proof it threaded the sidewalk grid instead of cutting through blocks.
            var measured = 0;
            var worstRatio = 0.0;
            for (var id = 1; id <= director.PedestrianCount; id++)
            {
                if (!director.IsPanicked(id))
                {
                    continue;
                }

                var panicPos = director.PanicPositionOf(id);
                var safeZone = director.SafeZoneOf(id);
                var straight = (safeZone - panicPos).Abs;

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

                var ratio = straight > 1e-6 ? arc / straight : 0.0;
                worstRatio = Math.Max(worstRatio, ratio);

                Assert.True(arc > 1.2 * straight,
                    $"seed {seed}: ped {id} routed arc {arc:F1} m should exceed 1.2x the straight-line {straight:F1} m (route AROUND blocks, not through them)");
                measured++;
            }

            Assert.True(measured > 0, $"seed {seed}: expected at least one panicked ped to have panicked far enough from its safe zone to measure");

            _out.WriteLine($"[Req4 measured] seed {seed} (incident=({incident.X},{incident.Y})): " +
                $"panicked={director.PanickedCount}, escaped={director.EscapedCount} of population={director.PedestrianCount}; " +
                $"{measured} interior peds measured, worst routed/straight ratio={worstRatio:F2} (all > 1.2).");
        }
    }
}
