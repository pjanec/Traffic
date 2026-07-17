using System.Reflection;
using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// P1E-4 (HIGH-DENSITY-P1E-DESIGN.md §1A/§1B/§3/§4, §9) -- the periodic congestion-reactive
// reroute device (device.rerouting / MSDevice_Routing), wired into Engine's step loop on top of
// the already-committed P1E-1/2/3 machinery (ScenarioConfig keys, RerouteEdgeWeights,
// NetworkRouter's injected-cost/A* overloads). These are BEHAVIOURAL/FUNCTIONAL tests against the
// engine's own TrajectorySet and (via reflection, since VehicleRuntime/Engine's side tables are
// intentionally not part of the public API) a few internal invariants -- there is no
// scenarios/NN-reroute-congestion golden here (that is P1E-5's job).
//
// Fixture: reuses the COMMITTED scenarios/_fixtures/routing-diamond/net.net.xml (the same diamond
// SA -> {AB,AC} -> {BD,CD} -> DE net RungB2RouterTests/RungHDp1e2EdgeWeightsTests/
// RungHDp1e3AStarTests already validate against), pairing it with FRESH, per-test rou.xml/
// sumocfg files written to a temp directory (same idiom as RungB10Geometry3dTests/
// RungW2SnapshotTests) -- the top path (SA AB BD DE, 1010m total) is shorter/preferred over the
// bottom detour (SA AC CD DE, 1268m total) at free-flow speed, so congesting AB is exactly the
// "shortcut congests, alternate is cheaper" scenario §1's periodic device targets.
public class RungHDp1e4RerouteDeviceTests
{
    private static readonly string NetPath = Path.Combine(
        RepoRoot(), "scenarios", "_fixtures", "routing-diamond", "net.net.xml");

    private static Engine LoadEngine(string rouXml, string cfgXml)
    {
        var dir = Path.Combine(Path.GetTempPath(), $"hd-p1e4-{Guid.NewGuid():N}");
        Directory.CreateDirectory(dir);
        var rouPath = Path.Combine(dir, "rou.rou.xml");
        var cfgPath = Path.Combine(dir, "config.sumocfg");
        File.WriteAllText(rouPath, rouXml);
        File.WriteAllText(cfgPath, cfgXml);

        var engine = new Engine();
        engine.LoadScenario(NetPath, rouPath, cfgPath);
        return engine;
    }

    // A stopped "blocker" on AB (near its start, just past junction A) plus many staggered
    // "follower" vehicles on the SAME top route -- the followers queue up behind the blocker,
    // driving AB's occupant mean speed down hard, which (once smoothed) makes the router prefer
    // the AC/CD detour for any equipped, still-upstream (on SA) vehicle.
    private static string CongestionRouXml(int followerCount, double followerHeadway)
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("<routes>");
        sb.AppendLine("""    <vType id="car" vClass="passenger" sigma="0"/>""");
        sb.AppendLine("""    <route id="top" edges="SA AB BD DE"/>""");
        sb.AppendLine(
            """    <vehicle id="blocker" type="car" route="top" depart="0" departPos="0" departSpeed="0" departLane="0">""");
        sb.AppendLine("""        <stop lane="AB_0" startPos="50" endPos="55" duration="100000"/>""");
        sb.AppendLine("    </vehicle>");
        for (var i = 0; i < followerCount; i++)
        {
            var depart = (i + 1) * followerHeadway;
            sb.AppendLine(
                $"""    <vehicle id="f{i}" type="car" route="top" depart="{depart.ToString(System.Globalization.CultureInfo.InvariantCulture)}" departPos="0" departSpeed="0" departLane="0"/>""");
        }

        sb.AppendLine("</routes>");
        return sb.ToString();
    }

    private static string CfgXml(
        int runEnd, double probability, double period, int adaptationSteps,
        double adaptationInterval, string algorithm, bool jitter)
    {
        var reroutingSection = period > 0.0
            ? $"""
                    <device.rerouting.probability value="{probability.ToString(System.Globalization.CultureInfo.InvariantCulture)}"/>
                    <device.rerouting.period value="{period.ToString(System.Globalization.CultureInfo.InvariantCulture)}"/>
                    <device.rerouting.adaptation-steps value="{adaptationSteps}"/>
                    <device.rerouting.adaptation-interval value="{adaptationInterval.ToString(System.Globalization.CultureInfo.InvariantCulture)}"/>
                    <routing-algorithm value="{algorithm}"/>
                    <device.rerouting.jitter value="{(jitter ? "true" : "false")}"/>
            """
            : string.Empty;

        return $"""
            <configuration>
                <time><begin value="0"/><end value="{runEnd}"/><step-length value="1"/></time>
                <processing>
                    <time-to-teleport value="-1"/>
                    <default.action-step-length value="1"/>
                    <default.speeddev value="0"/>
            {reroutingSection}        </processing>
                <random_number><seed value="42"/></random_number>
            </configuration>
            """;
    }

    // ----- Test 1: inert-when-off -----

    [Fact]
    public void ReroutePeriodZero_IsInert_NoVehicleEverTakesTheAlternate()
    {
        // Same congestion-inducing demand as the enabled test below, but device.rerouting is
        // entirely absent (ReroutePeriod defaults to 0) -- proves the feature is strictly opt-in:
        // even though the shortcut congests badly, nobody ever diverts.
        var engine = LoadEngine(
            CongestionRouXml(followerCount: 25, followerHeadway: 2.0),
            CfgXml(runEnd: 450, probability: 0.0, period: 0.0, adaptationSteps: 6,
                adaptationInterval: 1.0, algorithm: "dijkstra", jitter: false));

        var trajectory = engine.Run(450);

        Assert.DoesNotContain(trajectory.AllPoints, p => p.Lane == "AC_0" || p.Lane == "CD_0");

        // Every vehicle that ever appears stays on the original top-path lanes only.
        var lanesSeen = trajectory.AllPoints.Select(p => p.Lane).Distinct().ToList();
        Assert.All(lanesSeen, lane => Assert.True(
            lane is "SA_0" or "AB_0" or "BD_0" or "DE_0" || lane.StartsWith(':'),
            $"unexpected lane '{lane}' with rerouting disabled"));
    }

    // ----- Test 2: reroute triggers on congestion -----

    [Fact]
    public void Congestion_OnShortcut_CausesAtLeastOneVehicleToTakeTheAlternate()
    {
        var engine = LoadEngine(
            CongestionRouXml(followerCount: 25, followerHeadway: 2.0),
            CfgXml(runEnd: 450, probability: 1.0, period: 5.0, adaptationSteps: 6,
                adaptationInterval: 1.0, algorithm: "dijkstra", jitter: false));

        var trajectory = engine.Run(450);

        // At least one vehicle's installed route switched to the alternate (AC and/or CD) --
        // the shortcut (AB) congests, the periodic device reads the smoothed weights, and the
        // router prefers the free-flowing detour once AB's effort dominates.
        Assert.Contains(trajectory.AllPoints, p => p.Lane == "AC_0");
    }

    // ----- Test 3: route-slot recycling -----

    [Fact]
    public void ManyReroutesOfOneVehicle_DoNotGrowRoutesByIdUnboundedly()
    {
        // period=1 (reroute is due every single step) + a small adaptation window -- deliberately
        // aggressive so equipped vehicles are considered for reroute on nearly every step of a
        // long run, exercising RegisterPeriodicReroute's overwrite path many times over.
        var engine = LoadEngine(
            CongestionRouXml(followerCount: 25, followerHeadway: 2.0),
            CfgXml(runEnd: 400, probability: 1.0, period: 1.0, adaptationSteps: 4,
                adaptationInterval: 1.0, algorithm: "dijkstra", jitter: false));

        var baselineRouteCount = RoutesByIdCount(engine);
        var equippedCount = EquippedVehicleCount(engine);
        Assert.True(equippedCount > 0, "expected at least one equipped vehicle");

        var trajectory = engine.Run(400);

        // Non-vacuous: confirm the device actually fired repeatedly and actually installed at
        // least one real reroute (otherwise "bounded" would trivially hold with zero activity).
        Assert.Contains(trajectory.AllPoints, p => p.Lane == "AC_0");

        var finalRouteCount = RoutesByIdCount(engine);

        // §0.5.3: at most ONE extra _routesById entry per EQUIPPED vehicle for the whole run --
        // never one per reroute attempt/install.
        Assert.True(
            finalRouteCount <= baselineRouteCount + equippedCount,
            $"_routesById grew unboundedly: baseline={baselineRouteCount}, equipped={equippedCount}, " +
            $"final={finalRouteCount}");
    }

    // ----- Test 4: jitter spreads reroutes -----

    [Fact]
    public void Jitter_On_SpreadsFirstReroutesAcrossThePeriod()
    {
        const int vehicleCount = 40;
        const double period = 20.0;

        var jitterOn = LoadEngine(
            SimultaneousDepartRouXml(vehicleCount),
            CfgXml(runEnd: 10, probability: 1.0, period: period, adaptationSteps: 6,
                adaptationInterval: 1.0, algorithm: "dijkstra", jitter: true));

        var nextRerouteTimes = NextRerouteTimes(jitterOn);
        Assert.Equal(vehicleCount, nextRerouteTimes.Count);

        var distinct = nextRerouteTimes.Distinct().ToList();
        // Spread across [0, period), not a single-tick wave.
        Assert.True(distinct.Count >= vehicleCount / 2,
            $"expected a real spread of NextRerouteTime values, got {distinct.Count} distinct " +
            $"values out of {vehicleCount} vehicles");
        Assert.All(nextRerouteTimes, t => Assert.InRange(t, 0.0, period));
        Assert.True(nextRerouteTimes.Max() - nextRerouteTimes.Min() > period / 2.0,
            "expected the jittered NextRerouteTime values to span a wide range of the period");
    }

    [Fact]
    public void Jitter_Off_AllFirstReroutesLandOnTheSameTick()
    {
        const int vehicleCount = 40;
        const double period = 20.0;

        var jitterOff = LoadEngine(
            SimultaneousDepartRouXml(vehicleCount),
            CfgXml(runEnd: 10, probability: 1.0, period: period, adaptationSteps: 6,
                adaptationInterval: 1.0, algorithm: "dijkstra", jitter: false));

        var nextRerouteTimes = NextRerouteTimes(jitterOff);
        Assert.Equal(vehicleCount, nextRerouteTimes.Count);

        // SUMO-faithful schedule: every vehicle (same depart=0) fires at exactly depart+period.
        var distinct = nextRerouteTimes.Distinct().ToList();
        Assert.Single(distinct);
        Assert.Equal(period, distinct[0], precision: 9);
    }

    private static string SimultaneousDepartRouXml(int vehicleCount)
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("<routes>");
        sb.AppendLine("""    <vType id="car" vClass="passenger" sigma="0"/>""");
        sb.AppendLine("""    <route id="top" edges="SA AB BD DE"/>""");
        for (var i = 0; i < vehicleCount; i++)
        {
            sb.AppendLine(
                $"""    <vehicle id="v{i}" type="car" route="top" depart="0" departPos="0" departSpeed="0" departLane="0"/>""");
        }

        sb.AppendLine("</routes>");
        return sb.ToString();
    }

    // ----- Reflection helpers -----
    //
    // VehicleRuntime/Engine's `_vehicles`/`_routesById` side storage are deliberately NOT part of
    // the public SDK surface (SUMOSHARP-API.md's public API is the columnar Step()/TrajectorySet
    // read surface, not engine-internal bookkeeping) -- these tests need to observe that internal
    // bookkeeping directly (the whole POINT of the route-slot-recycling and jitter-schedule
    // success conditions is an internal-state invariant, not something the FCD/trajectory surface
    // exposes), so they reach in via reflection rather than growing the public API for a test-only
    // concern.
    private static object GetEngineField(Engine engine, string name)
    {
        var field = typeof(Engine).GetField(name, BindingFlags.NonPublic | BindingFlags.Instance)
            ?? throw new InvalidOperationException($"Engine field '{name}' not found (reflection).");
        return field.GetValue(engine)!;
    }

    private static int RoutesByIdCount(Engine engine)
    {
        var routesById = GetEngineField(engine, "_routesById");
        var countProp = routesById.GetType().GetProperty("Count")
            ?? throw new InvalidOperationException("_routesById has no Count property.");
        return (int)countProp.GetValue(routesById)!;
    }

    private static System.Collections.IEnumerable Vehicles(Engine engine) =>
        (System.Collections.IEnumerable)GetEngineField(engine, "_vehicles");

    private static int EquippedVehicleCount(Engine engine)
    {
        var count = 0;
        foreach (var v in Vehicles(engine))
        {
            var field = v.GetType().GetField("RerouteEquipped", BindingFlags.Public | BindingFlags.Instance)
                ?? throw new InvalidOperationException("VehicleRuntime.RerouteEquipped not found (reflection).");
            if ((bool)field.GetValue(v)!)
            {
                count++;
            }
        }

        return count;
    }

    private static List<double> NextRerouteTimes(Engine engine)
    {
        var result = new List<double>();
        foreach (var v in Vehicles(engine))
        {
            var equippedField = v.GetType().GetField("RerouteEquipped", BindingFlags.Public | BindingFlags.Instance)!;
            if (!(bool)equippedField.GetValue(v)!)
            {
                continue;
            }

            var nextField = v.GetType().GetField("NextRerouteTime", BindingFlags.Public | BindingFlags.Instance)
                ?? throw new InvalidOperationException("VehicleRuntime.NextRerouteTime not found (reflection).");
            result.Add((double)nextField.GetValue(v)!);
        }

        return result;
    }

    // Mirrors RungB3RerouteTests.RepoRoot()/RungB2RouterTests.RepoRoot().
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
}
