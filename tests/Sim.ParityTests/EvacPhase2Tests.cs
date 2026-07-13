using Sim.Core;
using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Evac;
using Xunit;
using Xunit.Abstractions;

namespace Sim.ParityTests;

// PANIC-EVAC-PHASE2-DESIGN.md / PANIC-EVAC-PHASE2-TASKS.md Stage S4: behavioural validation of the
// Phase-2 fear field wired into EvacDirector (T3.1) -- contagion spread, LoS occlusion, jam-unease
// amplification, distant-traffic obliviousness, front propagation, determinism, and inertness.
public class EvacPhase2Tests
{
    private readonly ITestOutputHelper _out;
    public EvacPhase2Tests(ITestOutputHelper output) => _out = output;

    private static readonly string NetPath =
        Path.Combine(RepoRoot(), "scenarios", "evac-grid", "net.net.xml");

    private static readonly WorldDisc[] NoOccluders = Array.Empty<WorldDisc>();

    private static VehicleHandle H(uint index) => new(index, 0);

    // ----- T4.1: contagion recruits beyond direct sight (integration) -----

    // NOTE (tuning finding): the grid demo's DEFAULT ThetaPanic (0.05) is so low that direct line-of-
    // sight ALONE already saturates the whole 32-car fleet within ~15 ticks on this net (every route
    // grazes at least the edge of the 140 m incident radius at some point) -- with EnableContagion
    // false or true the count is identically 32 at every horizon from 15 to 250+ ticks, leaving no
    // room for contagion to show a marginal effect. To actually isolate contagion's contribution (the
    // point of this test) ThetaPanic is raised to 0.3 for BOTH runs (still well below the "instantly
    // panic at the epicentre" ceiling of 1.0) so a grazing, near-radius-edge intensity is no longer
    // enough on its own -- only contagion recruits the extra cars beyond direct sight. This is a
    // config override for this test only; the grid's shipped DefaultConfig()/DefaultIncident are
    // unchanged and EvacSpineTests still runs (and passes) against the real defaults.
    [Fact]
    public void Contagion_RecruitsMoreThanDirectSightAlone()
    {
        var onConfig = EvacGridScenario.DefaultConfig() with { ThetaPanic = 0.3 };
        var (_, directorOn, _) = EvacGridScenario.Build(NetPath, config: onConfig);
        for (var step = 0; step < 250; step++)
        {
            directorOn.Tick();
        }
        var onCount = directorOn.PanickedCount;

        var offConfig = onConfig with { EnableContagion = false };
        var (_, directorOff, _) = EvacGridScenario.Build(NetPath, config: offConfig);
        for (var step = 0; step < 250; step++)
        {
            directorOff.Tick();
        }
        var offCount = directorOff.PanickedCount;

        _out.WriteLine($"onCount={onCount} offCount={offCount}");

        Assert.True(offCount > 0, "direct line-of-sight alone should still panic some cars");
        Assert.True(onCount > offCount, "contagion should recruit MORE panicked cars than direct sight alone");
    }

    // ----- T4.2: line-of-sight occlusion (FearField-level, behavioural) -----

    [Fact]
    public void LineOfSight_OccludedVehicleGetsNoDirectTerm_ClearPeerLatches()
    {
        var incident = new Incident(X: 0.0, Y: 0.0, StartTime: 0.0, Radius: 40.0);
        var occluder = new WorldDisc(10.0, 0.0, 0.0, 0.0, 3.0);
        var occluders = new[] { occluder };

        // Pure precondition: the LoS helper itself blocks A, not B.
        Assert.False(LineOfSight.IsVisible(new Vec2(20.0, 0.0), new Vec2(0.0, 0.0), occluders));
        Assert.True(LineOfSight.IsVisible(new Vec2(-20.0, 0.0), new Vec2(0.0, 0.0), occluders));

        var a = H(0);
        var b = H(1);
        var obs = new List<FearField.VehicleObs>
        {
            new(a, 20.0, 0.0, false),    // same distance as B, but occluded from the incident
            new(b, -20.0, 0.0, false),   // symmetric, clear line of sight
        };

        var cfg = new EvacConfig { EnableLineOfSight = true };
        var field = new FearField();
        field.Update(obs, incident, 0.0, occluders, cfg, 1.0);

        Assert.Equal(0.0, field.Fear(a));
        Assert.False(field.IsPanicked(a));

        Assert.True(field.Fear(b) > 0.0);
        Assert.True(field.IsPanicked(b));
    }

    // ----- T4.3: jam amplifies, does not originate (FearField-level) -----

    [Fact]
    public void JamUnease_AmplifiesExistingFear_ButNeverOriginatesIt()
    {
        // P sits at the epicentre (direct term = 1, latches on tick 0, pinned to 1.0 thereafter --
        // a full contagion source). Q sits OUTSIDE the incident radius (no direct term of its own)
        // but within contagion range of P, so its only fear source is contagion seeded from P.
        var incident = new Incident(X: 0.0, Y: 0.0, StartTime: 0.0, Radius: 10.0);
        var p = H(0);
        var q = H(1);

        // High ThetaPanic (test-only) widens the pre-latch window so the jam-vs-no-jam gap is visible
        // before Q latches either way.
        var cfg = new EvacConfig { ThetaPanic = 0.95, ContagionRadius = 25.0, ContagionRate = 0.6, JamUneaseRate = 0.3, FearDecay = 0.98 };

        int LatchTick(bool stationary)
        {
            var pObs = new FearField.VehicleObs(p, 0.0, 0.0, false);
            var qObs = new FearField.VehicleObs(q, 15.0, 0.0, stationary);
            var obs = new List<FearField.VehicleObs> { pObs, qObs };

            // Precondition: Q genuinely receives no direct term (outside the incident radius).
            Assert.Equal(0.0, incident.FearAt(qObs.X, qObs.Y, 0.0));

            var field = new FearField();
            for (var step = 0; step < 60; step++)
            {
                field.Update(obs, incident, step, NoOccluders, cfg, 1.0);
                if (field.IsPanicked(q))
                {
                    return step;
                }
            }

            return -1;
        }

        var stationaryTick = LatchTick(stationary: true);
        var movingTick = LatchTick(stationary: false);

        _out.WriteLine($"stationaryLatchTick={stationaryTick} movingLatchTick={movingTick}");

        Assert.True(stationaryTick >= 0, "Q (stationary) never latched within the iteration budget");
        Assert.True(movingTick >= 0, "Q (moving) never latched within the iteration budget");
        Assert.True(stationaryTick < movingTick,
            $"jam-unease should make Q latch SOONER while stationary (stationary={stationaryTick}, moving={movingTick})");

        // Jam alone originates nothing: a lone stationary vehicle with no visible incident and no
        // panicked neighbour stays at fear 0 forever (explicit here per T4.3, mirrors the B1 assertion).
        var farIncident = new Incident(X: 10_000.0, Y: 10_000.0, StartTime: 0.0, Radius: 5.0);
        var lone = H(2);
        var loneObs = new List<FearField.VehicleObs> { new(lone, 0.0, 0.0, true) };
        var loneField = new FearField();
        for (var step = 0; step < 10; step++)
        {
            var newlyLatched = loneField.Update(loneObs, farIncident, step, NoOccluders, cfg, 1.0);
            Assert.Empty(newlyLatched);
            Assert.Equal(0.0, loneField.Fear(lone));
        }
    }

    // ----- T4.4: distant traffic stays unaware (FearField-level) -----

    [Fact]
    public void DistantIsolatedVehicle_NeverPanics()
    {
        var incident = new Incident(X: 0.0, Y: 0.0, StartTime: 0.0, Radius: 20.0);
        var cfg = new EvacConfig { EnableContagion = true, EnableJamUnease = true, EnableLineOfSight = true };
        var handle = H(0);
        var obs = new List<FearField.VehicleObs> { new(handle, 500.0, 500.0, false) };

        var field = new FearField();
        for (var step = 0; step < 50; step++)
        {
            var newlyLatched = field.Update(obs, incident, step, NoOccluders, cfg, 1.0);
            Assert.Empty(newlyLatched);
            Assert.Equal(0.0, field.Fear(handle));
        }

        Assert.False(field.IsPanicked(handle));
    }

    // ----- T4.5: front starts local, spreads outward (integration, measured) -----

    [Fact]
    public void PanicFront_StartsNearIncident_ThenSpreadsOutward()
    {
        var (engine, director, handles) = EvacGridScenario.Build(NetPath);
        var incident = EvacGridScenario.DefaultIncident;

        var firstPanicMaxDist = double.NaN;
        var everMaxDist = 0.0;

        for (var step = 0; step < 250; step++)
        {
            director.Tick();

            var stepMax = -1.0;
            foreach (var h in handles)
            {
                if (director.IsPanicked(h) && engine.TryGetVehicle(h, out var v))
                {
                    var d = incident.DistanceTo(v.X, v.Y);
                    if (d > stepMax)
                    {
                        stepMax = d;
                    }
                }
            }

            if (stepMax >= 0.0)
            {
                if (double.IsNaN(firstPanicMaxDist))
                {
                    firstPanicMaxDist = stepMax;
                }
                everMaxDist = Math.Max(everMaxDist, stepMax);
            }
        }

        _out.WriteLine($"firstPanicMaxDist={firstPanicMaxDist:F1} everMaxDist={everMaxDist:F1} radius={incident.Radius:F1}");

        Assert.False(double.IsNaN(firstPanicMaxDist), "no vehicle ever panicked");
        Assert.True(firstPanicMaxDist <= incident.Radius + 80.0,
            $"panic front should start near/within the incident radius ({incident.Radius}), not the far corner; first max was {firstPanicMaxDist:F1}");
        Assert.True(everMaxDist > incident.Radius,
            $"panic should later reach vehicles beyond the direct-sight radius ({incident.Radius}); ever-max was only {everMaxDist:F1}");
    }

    // ----- T4.6: determinism (integration) -----

    [Fact]
    public void EvacRun_FearEvolution_IsDeterministic()
    {
        string Signature()
        {
            var (engine, director, handles) = EvacGridScenario.Build(NetPath);
            for (var step = 0; step < 250; step++)
            {
                director.Tick();
            }

            var sb = new System.Text.StringBuilder();
            sb.Append(director.PanickedCount).Append('|')
              .Append(director.ConvertedCount).Append('|')
              .Append(director.PedestrianCount).Append(';');
            foreach (var h in handles)
            {
                sb.Append(director.Fear(h).ToString("R")).Append(';');
            }

            return sb.ToString();
        }

        Assert.Equal(Signature(), Signature());
    }

    // ----- T4.7: inertness + gate -----

    [Fact]
    public void NoIncident_FearFieldStaysInert()
    {
        var neverFires = new Incident(X: 180.0, Y: 180.0, StartTime: 1e9, Radius: 140.0);
        var (_, director, handles) = EvacGridScenario.Build(NetPath, neverFires);

        for (var step = 0; step < 200; step++)
        {
            director.Tick();
        }

        Assert.Equal(0, director.PanickedCount);
        foreach (var h in handles)
        {
            Assert.Equal(0.0, director.Fear(h));
        }
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
