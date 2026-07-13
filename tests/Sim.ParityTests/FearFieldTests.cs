using Sim.Core;
using Sim.Core.Bridge;
using Sim.Evac;
using Xunit;

namespace Sim.ParityTests;

// PANIC-EVAC-PHASE2-DESIGN.md §2 / PANIC-EVAC-PHASE2-TASKS.md T2.1: the fear field's plan/commit
// update, exercised standalone (no Engine) via hand-built VehicleObs lists.
public class FearFieldTests
{
    private static readonly WorldDisc[] NoOccluders = Array.Empty<WorldDisc>();

    private static VehicleHandle H(uint index) => new(index, 0);

    [Fact]
    public void Update_IsOrderIndependent()
    {
        var incident = new Incident(X: 0.0, Y: 0.0, StartTime: 0.0, Radius: 50.0);
        var cfg = new EvacConfig();

        var obsInOrder = new List<FearField.VehicleObs>
        {
            new(H(0), 0.0, 0.0, false),
            new(H(1), 10.0, 0.0, false),
            new(H(2), 20.0, 0.0, false),
            new(H(3), 30.0, 0.0, false),
        };
        var obsReversed = new List<FearField.VehicleObs>(obsInOrder);
        obsReversed.Reverse();

        var fieldA = new FearField();
        fieldA.Update(obsInOrder, incident, 0.0, NoOccluders, cfg, 1.0);

        var fieldB = new FearField();
        fieldB.Update(obsReversed, incident, 0.0, NoOccluders, cfg, 1.0);

        foreach (var o in obsInOrder)
        {
            Assert.Equal(fieldA.Fear(o.Handle), fieldB.Fear(o.Handle), 12);
            Assert.Equal(fieldA.IsPanicked(o.Handle), fieldB.IsPanicked(o.Handle));
        }
    }

    [Fact]
    public void SeedOnly_NoVisibleIncident_FearStaysZeroForever()
    {
        // Incident starts far in the future -> never active during this test's timeline.
        var incident = new Incident(X: 0.0, Y: 0.0, StartTime: 1_000_000.0, Radius: 50.0);
        var cfg = new EvacConfig { EnableContagion = true, EnableJamUnease = true };

        var obs = new List<FearField.VehicleObs>
        {
            new(H(0), 0.0, 0.0, true),   // stationary, at the would-be epicentre
            new(H(1), 5.0, 0.0, true),   // stationary, close by
            new(H(2), 10.0, 0.0, false),
            new(H(3), 15.0, 0.0, false),
        };

        var field = new FearField();
        for (var step = 0; step < 50; step++)
        {
            var newlyLatched = field.Update(obs, incident, step, NoOccluders, cfg, 1.0);
            Assert.Empty(newlyLatched);
            foreach (var o in obs)
            {
                Assert.Equal(0.0, field.Fear(o.Handle));
                Assert.False(field.IsPanicked(o.Handle));
            }
        }
    }

    [Fact]
    public void DirectTerm_LatchesAndPinsToOne()
    {
        var incident = new Incident(X: 0.0, Y: 0.0, StartTime: 0.0, Radius: 40.0);
        var cfg = new EvacConfig();
        var handle = H(0);
        var obs = new List<FearField.VehicleObs> { new(handle, 0.0, 0.0, false) };

        var field = new FearField();
        var newlyLatched = field.Update(obs, incident, 0.0, NoOccluders, cfg, 1.0);

        Assert.Equal(1.0, field.Fear(handle));
        Assert.True(field.IsPanicked(handle));
        Assert.Contains(handle, newlyLatched);

        // A second tick keeps it panicked and pinned, and it is not re-reported as newly latched.
        var secondLatched = field.Update(obs, incident, 1.0, NoOccluders, cfg, 1.0);
        Assert.Equal(1.0, field.Fear(handle));
        Assert.True(field.IsPanicked(handle));
        Assert.DoesNotContain(handle, secondLatched);
    }

    [Fact]
    public void JamAlone_WithNoSeedAndNoVisibleIncident_AddsNothing()
    {
        // Incident far away in space AND never visible/active -> no direct term ever.
        var incident = new Incident(X: 10_000.0, Y: 10_000.0, StartTime: 0.0, Radius: 5.0);
        var cfg = new EvacConfig { EnableJamUnease = true, EnableContagion = true };
        var handle = H(0);
        var obs = new List<FearField.VehicleObs> { new(handle, 0.0, 0.0, true) }; // stationary, alone

        var field = new FearField();
        for (var step = 0; step < 10; step++)
        {
            var newlyLatched = field.Update(obs, incident, step, NoOccluders, cfg, 1.0);
            Assert.Empty(newlyLatched);
            Assert.Equal(0.0, field.Fear(handle));
        }
    }

    [Fact]
    public void Contagion_SpreadsFromPanickedNeighbour_AndOnlyWhenEnabled()
    {
        // Small incident radius so Q is OUTSIDE it (no direct term) but within contagion range of P.
        var incident = new Incident(X: 0.0, Y: 0.0, StartTime: 0.0, Radius: 5.0);
        var p = H(0);
        var q = H(1);
        var pObs = new FearField.VehicleObs(p, 0.0, 0.0, false);
        var qObs = new FearField.VehicleObs(q, 0.0, 10.0, false);
        var obs = new List<FearField.VehicleObs> { pObs, qObs };

        // Precondition: Q genuinely receives no direct term.
        Assert.Equal(0.0, incident.FearAt(qObs.X, qObs.Y, 0.0));
        Assert.True(incident.FearAt(pObs.X, pObs.Y, 0.0) > 0.0);

        // High latch threshold so Q's ramp is visibly gradual before it crosses it.
        var cfg = new EvacConfig { ThetaPanic = 0.95, ContagionRadius = 25.0, ContagionRate = 0.6, FearDecay = 0.98 };

        var field = new FearField();
        var qFears = new List<double>();
        var latchStep = -1;
        for (var step = 0; step < 10 && latchStep < 0; step++)
        {
            field.Update(obs, incident, step, NoOccluders, cfg, 1.0);
            qFears.Add(field.Fear(q));
            if (field.IsPanicked(q))
            {
                latchStep = step;
            }
        }

        Assert.True(latchStep >= 0, "Q never latched via contagion within the iteration budget.");
        Assert.True(qFears.Count >= 2, "expected a gradual ramp, not an instant jump.");
        for (var i = 1; i < qFears.Count; i++)
        {
            Assert.True(qFears[i] > qFears[i - 1], $"Q's fear did not strictly increase at step {i}: {qFears[i - 1]} -> {qFears[i]}");
        }
        Assert.Equal(1.0, field.Fear(q));

        // With contagion OFF, Q (no direct term, never stationary) stays at 0 forever.
        var cfgNoContagion = cfg with { EnableContagion = false };
        var fieldNoContagion = new FearField();
        for (var step = 0; step < 20; step++)
        {
            fieldNoContagion.Update(obs, incident, step, NoOccluders, cfgNoContagion, 1.0);
            Assert.Equal(0.0, fieldNoContagion.Fear(q));
        }
    }
}
