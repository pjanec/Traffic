using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

public class TrajectoryComparatorTests
{
    private static readonly ToleranceConfig ExactTolerance = new()
    {
        ParityMode = ParityMode.Exact,
        ComparedAttributes = new[] { "lane", "pos", "speed" },
        Pos = 0.001,
        Speed = 0.001,
    };

    private static TrajectoryPoint Point(string vehicleId, double time, string lane, double pos, double speed) =>
        new(vehicleId, time, lane, pos, speed, X: pos, Y: 0, Angle: 0, Acceleration: null);

    [Fact]
    public void IdenticalTrajectories_ReportZeroDiffAndNoDivergence()
    {
        var actual = new TrajectorySet();
        var expected = new TrajectorySet();

        for (var t = 0; t <= 5; t++)
        {
            var point = Point("veh0", t, "edge0_0", pos: t * 10.0, speed: 10.0);
            actual.Add(point);
            expected.Add(point);
        }

        var result = TrajectoryComparator.Compare(actual, expected, ExactTolerance);

        Assert.Empty(result.PresenceMismatches);
        Assert.Null(result.FirstDivergenceStep);
        Assert.True(result.IsMatch);
        Assert.All(result.Attributes, a =>
        {
            Assert.Equal(0.0, a.MaxAbsError);
            Assert.Equal(0.0, a.Rmse);
            Assert.True(a.WithinTolerance);
        });
    }

    [Fact]
    public void KnownOffset_IsDetectedAtTheCorrectFirstDivergenceStep()
    {
        var actual = new TrajectorySet();
        var expected = new TrajectorySet();

        const double offset = 5.0;
        const int divergenceStep = 3;

        for (var t = 0; t <= 5; t++)
        {
            expected.Add(Point("veh0", t, "edge0_0", pos: t * 10.0, speed: 10.0));
            var appliedOffset = t >= divergenceStep ? offset : 0.0;
            actual.Add(Point("veh0", t, "edge0_0", pos: t * 10.0 + appliedOffset, speed: 10.0));
        }

        var result = TrajectoryComparator.Compare(actual, expected, ExactTolerance);

        Assert.Equal((double)divergenceStep, result.FirstDivergenceStep);
        Assert.False(result.IsMatch);

        var posResult = Assert.Single(result.Attributes, a => a.Attribute == "pos");
        Assert.Equal(offset, posResult.MaxAbsError, precision: 6);
        Assert.False(posResult.WithinTolerance);

        var speedResult = Assert.Single(result.Attributes, a => a.Attribute == "speed");
        Assert.Equal(0.0, speedResult.MaxAbsError, precision: 6);
        Assert.True(speedResult.WithinTolerance);
    }

    // Phase 2 (sublane) instrument validation: the comparator must compare posLat when a scenario
    // lists it, localize a known lateral offset, and -- the byte-identity guarantee for phase-1
    // goldens -- IGNORE posLat entirely when it is not in comparedAttributes.
    private static readonly ToleranceConfig LateralTolerance = new()
    {
        ParityMode = ParityMode.Exact,
        ComparedAttributes = new[] { "lane", "pos", "speed", "posLat" },
        Pos = 0.001,
        Speed = 0.001,
        PosLat = 0.001,
    };

    private static TrajectoryPoint PointLat(string vehicleId, double time, string lane, double pos, double speed, double posLat) =>
        new(vehicleId, time, lane, pos, speed, X: pos, Y: 0, Angle: 0, Acceleration: null) { PosLat = posLat };

    [Fact]
    public void IdenticalPosLat_ReportsZeroDiff_WhenPosLatCompared()
    {
        var actual = new TrajectorySet();
        var expected = new TrajectorySet();

        for (var t = 0; t <= 5; t++)
        {
            var point = PointLat("veh0", t, "edge0_0", pos: t * 10.0, speed: 10.0, posLat: -1.6);
            actual.Add(point);
            expected.Add(point);
        }

        var result = TrajectoryComparator.Compare(actual, expected, LateralTolerance);

        Assert.True(result.IsMatch);
        var posLatResult = Assert.Single(result.Attributes, a => a.Attribute == "posLat");
        Assert.Equal(0.0, posLatResult.MaxAbsError);
        Assert.True(posLatResult.WithinTolerance);
    }

    [Fact]
    public void KnownPosLatOffset_IsDetectedAtTheCorrectFirstDivergenceStep()
    {
        var actual = new TrajectorySet();
        var expected = new TrajectorySet();

        const double latOffset = 0.5;
        const int divergenceStep = 2;

        for (var t = 0; t <= 5; t++)
        {
            expected.Add(PointLat("veh0", t, "edge0_0", pos: t * 10.0, speed: 10.0, posLat: 0.0));
            var applied = t >= divergenceStep ? latOffset : 0.0;
            actual.Add(PointLat("veh0", t, "edge0_0", pos: t * 10.0, speed: 10.0, posLat: applied));
        }

        var result = TrajectoryComparator.Compare(actual, expected, LateralTolerance);

        Assert.Equal((double)divergenceStep, result.FirstDivergenceStep);
        Assert.False(result.IsMatch);

        var posLatResult = Assert.Single(result.Attributes, a => a.Attribute == "posLat");
        Assert.Equal(latOffset, posLatResult.MaxAbsError, precision: 6);
        Assert.False(posLatResult.WithinTolerance);

        // pos/speed agree -> the divergence is isolated to the lateral axis.
        Assert.True(Assert.Single(result.Attributes, a => a.Attribute == "pos").WithinTolerance);
    }

    [Fact]
    public void PosLatDifference_IsIgnored_WhenNotCompared()
    {
        // A phase-1 scenario compares only ["lane","pos","speed"] (ExactTolerance). Two trajectories
        // that differ ONLY in posLat must still match -> adding the posLat column can never perturb
        // an existing golden's verdict.
        var actual = new TrajectorySet();
        var expected = new TrajectorySet();

        for (var t = 0; t <= 5; t++)
        {
            expected.Add(PointLat("veh0", t, "edge0_0", pos: t * 10.0, speed: 10.0, posLat: 0.0));
            actual.Add(PointLat("veh0", t, "edge0_0", pos: t * 10.0, speed: 10.0, posLat: 1.234));
        }

        var result = TrajectoryComparator.Compare(actual, expected, ExactTolerance);

        Assert.True(result.IsMatch);
        Assert.Null(result.FirstDivergenceStep);
        Assert.DoesNotContain(result.Attributes, a => a.Attribute == "posLat");
    }

    [Fact]
    public void PresenceMismatches_AreReportedExplicitly()
    {
        var actual = new TrajectorySet();
        var expected = new TrajectorySet();

        expected.Add(Point("veh0", 0, "edge0_0", 0, 0));
        expected.Add(Point("veh1", 0, "edge0_0", 0, 0));
        actual.Add(Point("veh0", 0, "edge0_0", 0, 0));
        actual.Add(Point("veh0", 1, "edge0_0", 10, 10));

        var result = TrajectoryComparator.Compare(actual, expected, ExactTolerance);

        Assert.Contains(result.PresenceMismatches, m => m.Kind == PresenceMismatchKind.MissingVehicle && m.VehicleId == "veh1");
        Assert.Contains(result.PresenceMismatches, m => m.Kind == PresenceMismatchKind.ExtraStep && m.VehicleId == "veh0" && m.Time == 1.0);
        Assert.False(result.IsMatch);
    }
}
