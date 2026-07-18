using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// VB-6 (VIZ_BENCH_TASKS.md Phase 2): pure/offline unit tests for the aggregate/statistical
// comparator against SYNTHETIC inputs (inline XML strings), not SUMO-generated files -- this
// keeps VB-6 safely inside `dotnet test` (no SUMO, no network) per BENCHMARK_SPEC.md's "not a
// parity test" rule while still proving the comparator's math end to end: parsing, the four
// aggregate metrics, the KS distribution distance, and pass/fail gating against a configurable
// per-rung tolerance.
public class Vb6AggregateComparatorTests
{
    private static readonly AggregateToleranceConfig LooseTolerance = AggregateToleranceConfig.Parse(
        """
        {
          "arrivedRelTol": 0.2,
          "meanDurationRelTol": 0.2,
          "meanSpeedRelTol": 0.2,
          "distributionDistanceTol": 0.25
        }
        """);

    [Fact]
    public void TripInfoParser_ReadsSumoSchema_IncludingGap2Fields_IgnoringOtherExtraAttributes()
    {
        // GAP-2 (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §2): the parser now also reads
        // arrivalLane/arrivalPos/arrival/routeLength/waitingTime/timeLoss (all TOLERATED-ABSENT --
        // see the second <tripinfo>, which carries none of them and still parses). The remaining
        // real-SUMO decorations (departLane/departPos/departSpeed/waitingCount/stopTime/rerouteNo/
        // devices/vType/speedFactor/vaporized) are still simply ignored -- this test's original
        // purpose (VB-6's "one schema, one loader" contract tolerates a real SUMO file's full
        // attribute set) is unchanged, only the read subset grew.
        var xml = """
            <tripinfos>
                <tripinfo id="0" depart="0.00" departLane="a_0" departPos="5.10" departSpeed="0.00"
                          arrival="52.00" arrivalLane="b_0" arrivalPos="179.20" arrivalSpeed="12.62"
                          duration="52.00" routeLength="366.33" waitingTime="0.00" waitingCount="0"
                          stopTime="0.00" timeLoss="6.38" rerouteNo="0" devices="tripinfo_0"
                          vType="DEFAULT_VEHTYPE" speedFactor="1.00" vaporized=""/>
                <tripinfo id="1" depart="3.00" duration="30.00" arrivalSpeed="10.00"/>
            </tripinfos>
            """;

        var records = TripInfoParser.ParseXml(xml);

        Assert.Equal(2, records.Count);
        Assert.Equal(
            new TripInfoRecord("0", 0.0, 52.0, 12.62,
                ArrivalLane: "b_0", ArrivalPos: 179.20, ArrivalTime: 52.0, RouteLength: 366.33,
                WaitingTime: 0.0, TimeLoss: 6.38),
            records[0]);
        Assert.Equal(new TripInfoRecord("1", 3.0, 30.0, 10.0), records[1]);
    }

    [Fact]
    public void TripInfoParser_DerivesDuration_WhenOnlyArrivalPresent()
    {
        var xml = """<tripinfos><tripinfo id="0" depart="10.00" arrival="42.00"/></tripinfos>""";

        var records = TripInfoParser.ParseXml(xml);

        Assert.Single(records);
        Assert.Equal(32.0, records[0].Duration, precision: 6);
    }

    [Fact]
    public void SummaryOutputParser_ReadsSumoSchema_AndTreatsNegativeMeanSpeedAsSentinel()
    {
        var xml = """
            <summary>
                <step time="0.00" loaded="2" inserted="1" running="0" waiting="0" ended="0" arrived="0"
                      collisions="0" teleports="0" halting="0" stopped="0" meanWaitingTime="0.00"
                      meanTravelTime="-1.00" meanSpeed="-1.00" meanSpeedRelative="-1.00" duration="0"/>
                <step time="1.00" running="2" arrived="0" meanSpeed="9.50"/>
                <step time="2.00" running="1" arrived="1" meanSpeed="10.00"/>
            </summary>
            """;

        var records = SummaryOutputParser.ParseXml(xml);

        Assert.Equal(3, records.Count);
        Assert.Null(records[0].MeanSpeed);
        Assert.Equal(9.50, records[1].MeanSpeed);
        Assert.Equal(1, records[2].Arrived);
    }

    [Fact]
    public void Compare_IdenticalInputs_MatchesWithZeroDistributionDistance()
    {
        var trips = TripInfoParser.ParseXml(
            """
            <tripinfos>
                <tripinfo id="0" depart="0.00" duration="40.00" arrivalSpeed="10.00"/>
                <tripinfo id="1" depart="5.00" duration="60.00" arrivalSpeed="12.00"/>
                <tripinfo id="2" depart="10.00" duration="50.00" arrivalSpeed="11.00"/>
            </tripinfos>
            """);
        var summary = SummaryOutputParser.ParseXml(
            """
            <summary>
                <step time="0.00" running="1" arrived="0" meanSpeed="8.00"/>
                <step time="1.00" running="2" arrived="0" meanSpeed="9.00"/>
                <step time="2.00" running="3" arrived="0" meanSpeed="10.00"/>
            </summary>
            """);

        var result = AggregateComparator.Compare(trips, summary, trips, summary, LooseTolerance);

        Assert.True(result.IsMatch, Describe(result));
        Assert.Equal(0.0, result.Arrived.RelError);
        Assert.Equal(0.0, result.MeanDuration.RelError);
        Assert.Equal(0.0, result.MeanSpeed.RelError);
        Assert.Equal(0.0, result.DistributionDistance);
    }

    [Fact]
    public void Compare_WithinLooseTolerance_Passes()
    {
        // Reference: 10 trips, durations 40..85 step 5 (mean 62.5), arrived=10.
        var referenceTrips = SyntheticTrips(count: 10, startDuration: 40, step: 5);
        var referenceSummary = SyntheticSummary(meanSpeed: 10.0, steps: 20);

        // Candidate: 9 trips (10% fewer), durations shifted +10% (mean ~68.75), speed -10%.
        var candidateTrips = SyntheticTrips(count: 9, startDuration: 44, step: 5);
        var candidateSummary = SyntheticSummary(meanSpeed: 9.2, steps: 20);

        var result = AggregateComparator.Compare(
            referenceTrips, referenceSummary, candidateTrips, candidateSummary, LooseTolerance);

        Assert.True(result.IsMatch, Describe(result));
    }

    [Fact]
    public void Compare_ArrivedFarOffTolerance_Fails()
    {
        var referenceTrips = SyntheticTrips(count: 20, startDuration: 40, step: 2);
        var referenceSummary = SyntheticSummary(meanSpeed: 10.0, steps: 20);

        // Half the vehicles arrive -- a stuck/gridlocked candidate run.
        var candidateTrips = SyntheticTrips(count: 10, startDuration: 40, step: 2);
        var candidateSummary = SyntheticSummary(meanSpeed: 10.0, steps: 20);

        var result = AggregateComparator.Compare(
            referenceTrips, referenceSummary, candidateTrips, candidateSummary, LooseTolerance);

        Assert.False(result.IsMatch);
        Assert.False(result.Arrived.WithinTolerance);
        Assert.Equal(0.5, result.Arrived.RelError, precision: 6);
    }

    [Fact]
    public void Compare_DivergentDurationDistribution_FailsOnDistributionDistance_NotJustMean()
    {
        // Reference: tight cluster around 50s.
        var referenceTrips = SyntheticTrips(count: 50, startDuration: 48, step: 0 /* effectively constant via override below */);
        referenceTrips = Enumerable.Range(0, 50)
            .Select(i => new TripInfoRecord($"r{i}", i, 49.0 + (i % 3), null))
            .ToList();

        // Candidate: same count and same MEAN duration (~50s) but bimodal (half at 20s, half at 80s)
        // -- a materially different distribution the mean alone would hide.
        var candidateTrips = Enumerable.Range(0, 50)
            .Select(i => new TripInfoRecord($"c{i}", i, i % 2 == 0 ? 20.0 : 80.0, null))
            .ToList();

        var summary = SyntheticSummary(meanSpeed: 10.0, steps: 5);

        var result = AggregateComparator.Compare(referenceTrips, summary, candidateTrips, summary, LooseTolerance);

        // Means are within ~2% of each other -- would pass on mean-duration alone.
        Assert.True(result.MeanDuration.WithinTolerance, Describe(result));
        // But the distribution shapes are clearly different -- KS distance must catch it.
        Assert.False(result.DistributionWithinTolerance, Describe(result));
        Assert.False(result.IsMatch);
    }

    [Fact]
    public void Compare_EmptyCandidateTrips_ReportsMaximalDivergence_NotThrow()
    {
        var referenceTrips = SyntheticTrips(count: 5, startDuration: 40, step: 5);
        var referenceSummary = SyntheticSummary(meanSpeed: 10.0, steps: 5);
        var candidateTrips = Array.Empty<TripInfoRecord>();
        var candidateSummary = Array.Empty<SummaryStepRecord>();

        var result = AggregateComparator.Compare(
            referenceTrips, referenceSummary, candidateTrips, candidateSummary, LooseTolerance);

        Assert.False(result.IsMatch);
        Assert.Equal(1.0, result.DistributionDistance);
        Assert.True(double.IsPositiveInfinity(result.Arrived.RelError) || result.Arrived.RelError > LooseTolerance.ArrivedRelTol);
    }

    private static List<TripInfoRecord> SyntheticTrips(int count, double startDuration, double step) =>
        Enumerable.Range(0, count)
            .Select(i => new TripInfoRecord($"veh{i}", Depart: i * 3.0, Duration: startDuration + i * step, ArrivalSpeed: 10.0))
            .ToList();

    private static List<SummaryStepRecord> SyntheticSummary(double meanSpeed, int steps) =>
        Enumerable.Range(0, steps)
            .Select(i => new SummaryStepRecord(Time: i, Running: steps - i, Arrived: i, MeanSpeed: meanSpeed))
            .ToList();

    private static string Describe(AggregateComparisonResult r) =>
        $"arrived(ref={r.Arrived.Reference},cand={r.Arrived.Candidate},rel={r.Arrived.RelError:F3}) " +
        $"meanDuration(ref={r.MeanDuration.Reference:F2},cand={r.MeanDuration.Candidate:F2},rel={r.MeanDuration.RelError:F3}) " +
        $"meanSpeed(ref={r.MeanSpeed.Reference:F2},cand={r.MeanSpeed.Candidate:F2},rel={r.MeanSpeed.RelError:F3}) " +
        $"ksDist={r.DistributionDistance:F3} (tol={r.DistributionDistanceTolerance:F3})";
}
