using Sim.Core;
using Sim.Core.Bridge;
using Sim.Core.Orca;

namespace Sim.Evac;

// PANIC-EVAC-PHASE2-DESIGN.md §2/§4: the per-vehicle fear field. Replaces Phase 1's stateless
// radius-only instant latch with per-vehicle state that integrates local inputs over time --
// direct (line-of-sight-gated) incident perception, contagion from panicking neighbours, and
// jam-unease -- so panic propagates outward as a front instead of filling the radius at once.
//
// Deterministic by construction: Update is a plan/commit pass over a FROZEN previous-fear array
// (exactly like the engine's own plan/execute discipline), fixed iteration order, no RNG/wall-clock.
public sealed class FearField
{
    private readonly Dictionary<VehicleHandle, double> _fear = new();
    private readonly HashSet<VehicleHandle> _panicked = new();

    public readonly record struct VehicleObs(VehicleHandle Handle, double X, double Y, bool Stationary);

    // Current fear (0..1) for a tracked vehicle; 0 if unknown (never observed).
    public double Fear(VehicleHandle h) => _fear.TryGetValue(h, out var f) ? f : 0.0;

    // Whether panic has latched for this vehicle (permanent once true).
    public bool IsPanicked(VehicleHandle h) => _panicked.Contains(h);

    // Drop all state for a vehicle (on convert to pedestrian / despawn) so it stops being tracked.
    public void Forget(VehicleHandle h)
    {
        _fear.Remove(h);
        _panicked.Remove(h);
    }

    // PANIC-EVAC-PHASE2-DESIGN.md §2: one plan/commit tick. Reads every vehicle's PREVIOUS fear from
    // the frozen snapshot (this._fear as it stood before this call), computes every new fear into a
    // scratch buffer, then commits all of them together -- so contagion always reads last-tick values
    // and the result never depends on iteration/processing order.
    // Returns the handles that newly latched panic during THIS call (empty if none).
    public IReadOnlyList<VehicleHandle> Update(
        IReadOnlyList<VehicleObs> obs,
        Incident incident,
        double time,
        IReadOnlyList<WorldDisc> occluders,
        EvacConfig cfg,
        double dt)
    {
        var n = obs.Count;
        var prev = new double[n];
        for (var i = 0; i < n; i++)
        {
            prev[i] = Fear(obs[i].Handle);
        }

        var incidentActive = incident.IsActive(time);
        var newFear = new double[n];
        var newlyLatched = new List<VehicleHandle>();

        for (var i = 0; i < n; i++)
        {
            var oi = obs[i];

            var intensity = incidentActive ? incident.FearAt(oi.X, oi.Y, time) : 0.0;
            var visible = intensity > 0.0 &&
                (!cfg.EnableLineOfSight || LineOfSight.IsVisible(new Vec2(oi.X, oi.Y), new Vec2(incident.X, incident.Y), occluders));
            var directLoS = visible ? intensity : 0.0;

            var contagion = 0.0;
            if (cfg.EnableContagion)
            {
                for (var j = 0; j < n; j++)
                {
                    if (j == i)
                    {
                        continue;
                    }

                    var oj = obs[j];
                    var dx = oi.X - oj.X;
                    var dy = oi.Y - oj.Y;
                    var dist = Math.Sqrt(dx * dx + dy * dy);
                    contagion += ContagionWeight(dist, cfg.ContagionRadius) * prev[j];
                }
            }

            var jam = (cfg.EnableJamUnease && oi.Stationary) ? 1.0 : 0.0;

            var accum = cfg.FearDecay * prev[i]
                + dt * (cfg.ContagionRate * contagion + cfg.JamUneaseRate * jam * prev[i]);

            var value = Math.Clamp(Math.Max(accum, directLoS), 0.0, 1.0);

            var alreadyPanicked = _panicked.Contains(oi.Handle);
            if (!alreadyPanicked && value >= cfg.ThetaPanic)
            {
                newlyLatched.Add(oi.Handle);
                alreadyPanicked = true;
            }

            if (alreadyPanicked)
            {
                value = 1.0;
            }

            newFear[i] = value;
        }

        for (var i = 0; i < n; i++)
        {
            _fear[obs[i].Handle] = newFear[i];
        }

        foreach (var h in newlyLatched)
        {
            _panicked.Add(h);
        }

        return newlyLatched;
    }

    // PANIC-EVAC-PHASE2-DESIGN.md §4: contagion proximity kernel. 1 at d=0, linear to 0 at d=radius,
    // 0 beyond. Pure, no state.
    public static double ContagionWeight(double distance, double radius)
    {
        if (radius <= 0.0)
        {
            return 0.0;
        }

        return Math.Max(0.0, 1.0 - distance / radius);
    }
}
