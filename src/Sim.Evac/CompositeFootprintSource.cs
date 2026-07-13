using Sim.Core.Bridge;

namespace Sim.Evac;

// PANIC-EVAC-PHASE3-DESIGN.md §4 / PANIC-EVAC-PHASE3-TASKS.md T3.1 §3(c): make the LANE engine avoid
// BOTH pedestrians and pushing (Orca-push) cars through the same single Engine.CrowdSource seam. Fills
// from `a` first, then -- with whatever span capacity remains -- from `b`. Still exactly one seam into
// the engine, and still parity-inert when both sources are empty (every committed golden never has an
// EvacDirector attached), so the determinism hash cannot move.
public sealed class CompositeFootprintSource : ICrowdFootprintSource
{
    private readonly ICrowdFootprintSource _a;
    private readonly ICrowdFootprintSource _b;

    public CompositeFootprintSource(ICrowdFootprintSource a, ICrowdFootprintSource b)
    {
        _a = a;
        _b = b;
    }

    public int QueryNear(double x, double y, double radius, Span<WorldDisc> into)
    {
        var n = _a.QueryNear(x, y, radius, into);
        if (n < into.Length)
        {
            n += _b.QueryNear(x, y, radius, into[n..]);
        }

        return n;
    }
}
