using Sim.Core;
using Sim.Core.Orca;
using Sim.Evac;
using Xunit;
using Xunit.Abstractions;

namespace Sim.ParityTests;

// PANIC-EVAC-PHASE3-DESIGN.md / PANIC-EVAC-PHASE3-TASKS.md Stage S4: behavioural validation of the
// Orca-push stage wired into EvacDirector (T3.1) -- push precedes foot exodus, shaped confinement,
// wedge->pedestrian handoff, no interpenetration, determinism, and inertness.
public class EvacPhase3Tests
{
    private readonly ITestOutputHelper _out;
    public EvacPhase3Tests(ITestOutputHelper output) => _out = output;

    private static readonly string NetPath =
        Path.Combine(RepoRoot(), "scenarios", "evac-grid", "net.net.xml");

    // ----- T4.1: push precedes foot exodus -----

    [Fact]
    public void OrcaPush_PeaksAboveZero_ThenCascadeCompletes()
    {
        var (_, directorOn, _) = EvacGridScenario.Build(NetPath);   // EnableOrcaPush defaults true
        var peakOn = 0;
        for (var step = 0; step < 300; step++)
        {
            directorOn.Tick();
            peakOn = Math.Max(peakOn, directorOn.OrcaPushCount);
        }

        _out.WriteLine($"peakOn={peakOn} pedestriansOn={directorOn.PedestrianCount}");
        Assert.True(peakOn > 0, "expected some cars to enter the Orca-push stage (peak OrcaPushCount > 0)");

        var offConfig = EvacGridScenario.DefaultConfig() with { EnableOrcaPush = false };
        var (_, directorOff, _) = EvacGridScenario.Build(NetPath, config: offConfig);
        var peakOff = 0;
        for (var step = 0; step < 300; step++)
        {
            directorOff.Tick();
            peakOff = Math.Max(peakOff, directorOff.OrcaPushCount);
        }

        _out.WriteLine($"peakOff={peakOff} pedestriansOff={directorOff.PedestrianCount}");
        Assert.Equal(0, peakOff);
        Assert.True(directorOff.PedestrianCount > 0, "cascade should still complete with push disabled");
    }

    // ----- T4.2: shaped confinement -----

    [Fact]
    public void ActivePushers_StayWithinNavmeshBounds()
    {
        var (_, director, _) = EvacGridScenario.Build(NetPath);

        for (var step = 0; step < 300; step++)
        {
            director.Tick();

            foreach (var (x, y, _) in director.ActivePushers())
            {
                Assert.True(director.NavMesh.Contains(new Vec2(x, y)),
                    $"pusher at ({x:F2},{y:F2}) crossed the band wall at step {step}");
            }
        }
    }

    // ----- T4.3: wedge -> pedestrian -----

    [Fact]
    public void WedgedPushers_BecomePedestrians_AndEventuallyDrainToZero()
    {
        var (_, director, _) = EvacGridScenario.Build(NetPath);

        for (var step = 0; step < 500; step++)
        {
            director.Tick();
        }

        _out.WriteLine($"pedestrians={director.PedestrianCount} remainingPushers={director.OrcaPushCount}");

        Assert.True(director.PedestrianCount > 0, "expected pedestrians produced via the push->wedge path");
        Assert.True(director.OrcaPushCount == 0,
            $"expected every pusher to have wedged->pedestrian or otherwise deactivated by end of run; " +
            $"{director.OrcaPushCount} still active");
    }

    // ----- T4.4: no interpenetration among active pushers -----

    [Fact]
    public void ActivePushers_NeverInterpenetrate()
    {
        var (_, director, _) = EvacGridScenario.Build(NetPath);

        var minSep = double.PositiveInfinity;
        for (var step = 0; step < 300; step++)
        {
            director.Tick();

            var poses = new List<(double X, double Y)>();
            foreach (var (x, y, _) in director.ActivePushers())
            {
                poses.Add((x, y));
            }

            for (var a = 0; a < poses.Count; a++)
            {
                for (var b = a + 1; b < poses.Count; b++)
                {
                    var dx = poses[a].X - poses[b].X;
                    var dy = poses[a].Y - poses[b].Y;
                    var d = Math.Sqrt(dx * dx + dy * dy);
                    if (d < minSep)
                    {
                        minSep = d;
                    }
                }
            }
        }

        _out.WriteLine($"minSep={(double.IsPositiveInfinity(minSep) ? -1.0 : minSep):F3}");

        if (!double.IsPositiveInfinity(minSep))
        {
            Assert.True(minSep >= 1.0, $"expected active pushers to keep >= 1.0 m separation, observed min={minSep:F3}");
        }
    }

    // ----- T4.5: determinism -----

    [Fact]
    public void OrcaPush_RunIsDeterministic()
    {
        string Signature()
        {
            var (_, director, handles) = EvacGridScenario.Build(NetPath);
            for (var step = 0; step < 300; step++)
            {
                director.Tick();
            }

            var sb = new System.Text.StringBuilder();
            sb.Append(director.OrcaPushCount).Append('|')
              .Append(director.PanickedCount).Append('|')
              .Append(director.ConvertedCount).Append('|')
              .Append(director.PedestrianCount).Append(';');

            foreach (var (x, y, headingRad) in director.ActivePushers())
            {
                sb.Append(x.ToString("R")).Append(',')
                  .Append(y.ToString("R")).Append(',')
                  .Append(headingRad.ToString("R")).Append(';');
            }

            foreach (var h in handles)
            {
                sb.Append(director.Fear(h).ToString("R")).Append(';');
            }

            return sb.ToString();
        }

        Assert.Equal(Signature(), Signature());
    }

    // ----- T4.6: parity / inertness + gate -----

    [Fact]
    public void NoIncident_OrcaPushStaysInert()
    {
        var neverFires = new Incident(180.0, 180.0, 1e9, 140.0);
        var (_, director, handles) = EvacGridScenario.Build(NetPath, neverFires);

        for (var step = 0; step < 200; step++)
        {
            director.Tick();
        }

        Assert.Equal(0, director.OrcaPushCount);
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
