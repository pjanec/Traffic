using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

/// <summary>
/// C1-ii self-test: the ENSEMBLE STATISTICAL comparator (<see cref="TrajectoryComparator.CompareEnsemble"/>)
/// and its <c>parityMode="statistical"</c> tolerance schema, exercised entirely offline against
/// synthetic in-memory <see cref="TrajectorySet"/> ensembles — mirrors the Task-0 exact-comparator
/// self-test idiom in <c>TrajectoryComparatorTests</c>. No SUMO, no scenario files, no golden.
/// </summary>
public class RungC1StatisticalHarnessTests
{
    private static readonly ToleranceConfig StatisticalTolerance = new()
    {
        ParityMode = ParityMode.Statistical,
        ComparedAttributes = new[] { "speed", "pos" },
        Statistical = new Dictionary<string, StatisticalAttributeTolerance>
        {
            ["speed"] = new StatisticalAttributeTolerance(Mean: 0.5, Std: 0.5),
            ["pos"] = new StatisticalAttributeTolerance(Mean: 5.0, Std: 5.0),
        },
    };

    private static TrajectoryPoint Point(string vehicleId, double time, double pos, double speed) =>
        new(vehicleId, time, Lane: "edge0_0", pos, speed, X: pos, Y: 0, Angle: 0, Acceleration: null);

    /// <summary>Deterministic simple LCG so the self-test needs no System.Random (CLAUDE.md rule).</summary>
    private static double NextUnit(ref ulong state)
    {
        // splitmix64-style step, mapped to roughly [-1, 1).
        state += 0x9E3779B97F4A7C15UL;
        var z = state;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        z ^= z >> 31;
        var frac = (z >> 11) * (1.0 / (1UL << 53));
        return frac * 2.0 - 1.0;
    }

    /// <summary>
    /// Builds an ensemble of <paramref name="runCount"/> single-vehicle runs, each with
    /// <paramref name="stepCount"/> steps of (pos, speed), where speed = baseSpeed + per-point
    /// jitter*noiseAmplitude and pos accumulates speed*dt. Deterministic given the seed.
    /// </summary>
    private static List<TrajectorySet> BuildEnsemble(
        int runCount, int stepCount, double baseSpeed, double noiseAmplitude, ulong seed)
    {
        var runs = new List<TrajectorySet>();
        var state = seed;

        for (var run = 0; run < runCount; run++)
        {
            var set = new TrajectorySet();
            double pos = 0.0;
            for (var t = 0; t < stepCount; t++)
            {
                var speed = baseSpeed + noiseAmplitude * NextUnit(ref state);
                pos += speed;
                set.Add(Point($"veh{run}", t, pos, speed));
            }
            runs.Add(set);
        }

        return runs;
    }

    [Fact]
    public void IdenticalEnsembles_Match()
    {
        var expected = BuildEnsemble(runCount: 20, stepCount: 10, baseSpeed: 10.0, noiseAmplitude: 1.0, seed: 1);
        // Same generator, same seed -> byte-identical ensemble content (different TrajectorySet instances).
        var actual = BuildEnsemble(runCount: 20, stepCount: 10, baseSpeed: 10.0, noiseAmplitude: 1.0, seed: 1);

        var result = TrajectoryComparator.CompareEnsemble(actual, expected, StatisticalTolerance);

        Assert.True(result.IsMatch);
        Assert.All(result.Attributes, a =>
        {
            Assert.True(a.MeanWithinTolerance, $"{a.Attribute} mean should be within tolerance");
            Assert.True(a.StdWithinTolerance, $"{a.Attribute} std should be within tolerance");
            Assert.Equal(0.0, a.MeanError, precision: 9);
            Assert.Equal(0.0, a.StdError, precision: 9);
        });
    }

    [Fact]
    public void ShiftedMean_FailsOnMeanNotStd()
    {
        var expected = BuildEnsemble(runCount: 20, stepCount: 10, baseSpeed: 10.0, noiseAmplitude: 1.0, seed: 2);
        // Same spread, but the whole ensemble's speed is shifted well beyond the 0.5 mean tolerance.
        var actual = BuildEnsemble(runCount: 20, stepCount: 10, baseSpeed: 12.0, noiseAmplitude: 1.0, seed: 2);

        var result = TrajectoryComparator.CompareEnsemble(actual, expected, StatisticalTolerance);

        Assert.False(result.IsMatch);

        var speedResult = Assert.Single(result.Attributes, a => a.Attribute == "speed");
        Assert.False(speedResult.MeanWithinTolerance);
        Assert.True(speedResult.StdWithinTolerance, "std should still be within tolerance -- only mean shifted");
        Assert.True(speedResult.MeanError > StatisticalTolerance.MeanToleranceFor("speed"));
    }

    [Fact]
    public void InflatedVariance_FailsOnStdNotMean()
    {
        var expected = BuildEnsemble(runCount: 20, stepCount: 10, baseSpeed: 10.0, noiseAmplitude: 1.0, seed: 3);
        // Same mean speed, but much larger jitter amplitude -> much larger std, beyond the 0.5 std tolerance.
        var actual = BuildEnsemble(runCount: 20, stepCount: 10, baseSpeed: 10.0, noiseAmplitude: 8.0, seed: 3);

        var result = TrajectoryComparator.CompareEnsemble(actual, expected, StatisticalTolerance);

        Assert.False(result.IsMatch);

        var speedResult = Assert.Single(result.Attributes, a => a.Attribute == "speed");
        Assert.False(speedResult.StdWithinTolerance);
        Assert.True(speedResult.MeanError <= StatisticalTolerance.MeanToleranceFor("speed"),
            "mean should stay close -- only spread was inflated");
        Assert.True(speedResult.StdError > StatisticalTolerance.StdToleranceFor("speed"));
    }

    [Fact]
    public void SmallJitter_StaysWithinToleranceBand()
    {
        // Same generator/params, different seed -> different per-point noise draws, but the
        // ensemble mean/std of both should land within the same statistical band (proves the
        // tolerance is a real +/- band, not exact per-point or exact-stat equality).
        var expected = BuildEnsemble(runCount: 30, stepCount: 15, baseSpeed: 10.0, noiseAmplitude: 1.0, seed: 4);
        var actual = BuildEnsemble(runCount: 30, stepCount: 15, baseSpeed: 10.0, noiseAmplitude: 1.0, seed: 5);

        var result = TrajectoryComparator.CompareEnsemble(actual, expected, StatisticalTolerance);

        Assert.True(result.IsMatch);
        Assert.All(result.Attributes, a => Assert.True(a.WithinTolerance));
    }

    [Fact]
    public void LaneAttribute_IsSkippedNotCompared()
    {
        var tolerance = new ToleranceConfig
        {
            ParityMode = ParityMode.Statistical,
            ComparedAttributes = new[] { "lane", "speed" },
            Statistical = new Dictionary<string, StatisticalAttributeTolerance>
            {
                ["speed"] = new StatisticalAttributeTolerance(Mean: 0.5, Std: 0.5),
            },
        };

        var expected = BuildEnsemble(runCount: 5, stepCount: 5, baseSpeed: 10.0, noiseAmplitude: 1.0, seed: 6);
        var actual = BuildEnsemble(runCount: 5, stepCount: 5, baseSpeed: 10.0, noiseAmplitude: 1.0, seed: 6);

        // Must not throw even though "lane" has no statistical tolerance entry and is categorical.
        var result = TrajectoryComparator.CompareEnsemble(actual, expected, tolerance);

        Assert.Single(result.Attributes);
        Assert.Equal("speed", result.Attributes[0].Attribute);
    }

    [Fact]
    public void EmptyEnsembles_YieldZeroStatsByConvention()
    {
        var tolerance = new ToleranceConfig
        {
            ParityMode = ParityMode.Statistical,
            ComparedAttributes = new[] { "speed" },
            Statistical = new Dictionary<string, StatisticalAttributeTolerance>
            {
                ["speed"] = new StatisticalAttributeTolerance(Mean: 0.5, Std: 0.5),
            },
        };

        var result = TrajectoryComparator.CompareEnsemble(new List<TrajectorySet>(), new List<TrajectorySet>(), tolerance);

        Assert.True(result.IsMatch);
        var speedResult = Assert.Single(result.Attributes);
        Assert.Equal(0.0, speedResult.MeanActual);
        Assert.Equal(0.0, speedResult.StdActual);
    }

    [Fact]
    public void StatisticalToleranceJson_RoundTrips()
    {
        const string json = """
        {
          "parityMode": "statistical",
          "comparedAttributes": ["speed", "pos"],
          "statistical": {
            "speed": { "mean": 0.5, "std": 0.75 },
            "pos": { "mean": 5.0, "std": 6.5 }
          }
        }
        """;

        var config = ToleranceConfig.Parse(json);

        Assert.Equal(ParityMode.Statistical, config.ParityMode);
        Assert.Equal(0.5, config.MeanToleranceFor("speed"));
        Assert.Equal(0.75, config.StdToleranceFor("speed"));
        Assert.Equal(5.0, config.MeanToleranceFor("pos"));
        Assert.Equal(6.5, config.StdToleranceFor("pos"));
    }

    [Fact]
    public void ExactToleranceJson_LoadsUnchanged()
    {
        const string json = """
        {
          "parityMode": "exact",
          "comparedAttributes": ["lane", "pos", "speed"],
          "pos": 0.001,
          "speed": 0.001
        }
        """;

        var config = ToleranceConfig.Parse(json);

        Assert.Equal(ParityMode.Exact, config.ParityMode);
        Assert.Null(config.Statistical);
        Assert.Equal(0.001, config.ToleranceFor("pos"));
        Assert.Equal(0.001, config.ToleranceFor("speed"));
        Assert.Equal(0.0, config.ToleranceFor("lane"));
        Assert.Throws<InvalidOperationException>(() => config.MeanToleranceFor("speed"));
    }
}
