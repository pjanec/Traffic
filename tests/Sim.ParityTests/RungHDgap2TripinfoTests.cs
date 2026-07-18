using Sim.Core;
using Sim.Harness;
using Sim.Sumo;
using Xunit;

namespace Sim.ParityTests;

// GAP-2 (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §2, docs/SERVE-PATH-PLAN.md): the engine-CLI
// `--tripinfo-output` writer + arrivalLane. Acceptance scenario scenarios/66-tripinfo-arrivallane
// (golden regenerated from real SUMO 1.20.0, sentinel-gated via `.wants-tripinfo` -- see
// scripts/regen-goldens.sh): a leader with a brief scheduled <stop> and a follower that catches up
// and queues behind it, so BOTH vehicles accrue nonzero timeLoss and the follower accrues nonzero
// waitingTime, and both genuinely ARRIVE (reach the edge end) well before the configured sim end.
//
// Mirrors RungHDp0dSummaryOutputParityTests.cs (plain-engine trip-record assertion) and
// RungHDgap1SumoCliTests.cs (drives the same scenario through the CLI shim to prove the wiring),
// combined: this is the first GAP-2 test to assert BOTH the plain-engine path (engine.CompletedTrips)
// AND the CLI path (SumoShim.Run --tripinfo-output) against the SAME golden.
public class RungHDgap2TripinfoTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "66-tripinfo-arrivallane");
    private const int Steps = 60; // matches config.sumocfg's <end value="60"/> at step-length 1s.

    [Fact]
    public void Run60Steps_CompletedTrips_MatchesGoldenTripinfoWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        engine.Run(Steps);

        // Both vehicles must have genuinely arrived (route-end, not a sink/parked-forever vehicle
        // that never appears -- there is none in this scenario, but the point is the SET must be
        // exactly these two, no more, no fewer).
        var arrivedIds = engine.CompletedTrips.Select(t => t.Id).OrderBy(id => id, StringComparer.Ordinal).ToList();
        Assert.Equal(new[] { "follower", "leader" }, arrivedIds);

        var actual = ToTripInfoRecords(engine.CompletedTrips);
        var golden = TripInfoParser.Parse(Path.Combine(ScenarioDir, "golden.tripinfo.xml"));

        AssertGoldenHasArrivalFields(golden);

        var result = TripInfoComparator.Compare(actual, golden);

        Assert.True(result.IsMatch,
            "scenario 66 tripinfo parity FAILED (engine.CompletedTrips path). " +
            $"missingIds=[{string.Join(",", result.MissingIds)}]; extraIds=[{string.Join(",", result.ExtraIds)}]; " +
            $"arrivalLaneMismatches=[{string.Join(";", result.ArrivalLaneMismatches)}]; " +
            string.Join(" ", result.Attributes.Select(a =>
                $"{a.Attribute}(maxAbs={a.MaxAbsError:G6},rmse={a.Rmse:G6},ok={a.WithinTolerance})")));
    }

    // Proves the CLI wiring end to end: SumoShim.Run("--tripinfo-output", ...) must produce a file
    // that parses back to the SAME golden-matching records as the direct engine.CompletedTrips path
    // above -- not a separate/weaker check, the SAME comparator against the SAME golden.
    [Fact]
    public void CliShim_TripinfoOutput_ProducesFileMatchingGoldenWithinTolerance()
    {
        var outDir = Path.Combine(Path.GetTempPath(), "sumosharp-gap2-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(outDir);
        try
        {
            var tripinfoOut = Path.Combine(outDir, "TI.xml");
            var stdout = new StringWriter();
            var stderr = new StringWriter();

            var exit = SumoShim.Run(
                new[]
                {
                    "-c", Path.Combine(ScenarioDir, "config.sumocfg"),
                    "--tripinfo-output", tripinfoOut,
                    "--end", Steps.ToString(),
                    "--no-step-log", "true",
                },
                stdout, stderr);

            Assert.Equal(0, exit);
            Assert.True(File.Exists(tripinfoOut), "--tripinfo-output did not produce a file");

            var actual = TripInfoParser.Parse(tripinfoOut);
            var golden = TripInfoParser.Parse(Path.Combine(ScenarioDir, "golden.tripinfo.xml"));

            var arrivedIds = actual.Select(t => t.Id).OrderBy(id => id, StringComparer.Ordinal).ToList();
            Assert.Equal(new[] { "follower", "leader" }, arrivedIds);

            var result = TripInfoComparator.Compare(actual, golden);

            Assert.True(result.IsMatch,
                "scenario 66 tripinfo parity FAILED (CLI shim path). " +
                $"missingIds=[{string.Join(",", result.MissingIds)}]; extraIds=[{string.Join(",", result.ExtraIds)}]; " +
                $"arrivalLaneMismatches=[{string.Join(";", result.ArrivalLaneMismatches)}]; " +
                string.Join(" ", result.Attributes.Select(a =>
                    $"{a.Attribute}(maxAbs={a.MaxAbsError:G6},rmse={a.Rmse:G6},ok={a.WithinTolerance})")));
        }
        finally
        {
            Directory.Delete(outDir, recursive: true);
        }
    }

    // Guards against a vacuous pass: if the golden itself somehow lost its GAP-2 attributes (a
    // stale/mis-regenerated file), every numeric/lane comparison above would silently degrade to
    // "both sides null" (0 error). This pins the golden to actually carry real values.
    private static void AssertGoldenHasArrivalFields(IReadOnlyList<TripInfoRecord> golden)
    {
        Assert.Equal(2, golden.Count);
        foreach (var t in golden)
        {
            Assert.False(string.IsNullOrEmpty(t.ArrivalLane), $"golden tripinfo for '{t.Id}' has no arrivalLane");
            Assert.NotNull(t.ArrivalPos);
            Assert.NotNull(t.ArrivalTime);
            Assert.NotNull(t.RouteLength);
            Assert.NotNull(t.WaitingTime);
            Assert.NotNull(t.TimeLoss);
        }

        // The whole point of this scenario: BOTH vehicles must show nonzero timeLoss, and the
        // follower must show nonzero waitingTime (see rou.rou.xml's own header comment) -- so this
        // test actually exercises those fields, not an all-zero degenerate case.
        Assert.All(golden, t => Assert.True(t.TimeLoss > 0.0, $"golden timeLoss for '{t.Id}' should be > 0"));
        Assert.Contains(golden, t => t.WaitingTime > 0.0);
    }

    private static List<TripInfoRecord> ToTripInfoRecords(IReadOnlyList<CompletedTripInfo> trips)
    {
        var records = new List<TripInfoRecord>(trips.Count);
        foreach (var trip in trips)
        {
            records.Add(new TripInfoRecord(
                trip.Id, trip.Depart, trip.Duration, trip.ArrivalSpeed,
                ArrivalLane: trip.ArrivalLane,
                ArrivalPos: trip.ArrivalPos,
                ArrivalTime: trip.Arrival,
                RouteLength: trip.RouteLength,
                WaitingTime: trip.WaitingTime,
                TimeLoss: trip.TimeLoss));
        }

        return records;
    }

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
