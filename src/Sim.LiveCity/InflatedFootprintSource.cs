using System;
using Sim.Core.Bridge;

namespace Sim.LiveCity;

// Wraps an ICrowdFootprintSource and INFLATES every disc's radius by a fixed amount while preserving its
// position AND velocity. realism #1 (mid-junction ORCA): an ORCA ped is exposed to the vehicle crowd-brake
// only as its 0.3 m physics footprint. The engine's CrowdLongitudinalConstraint gate projects the disc onto
// the car's lane and brakes only while |latOff - egoLat| < egoHalf + discRadius; on a short/curved INTERNAL
// junction lane that lane-projection misjudges an ORCA ped's lateral offset, so the 0.3 m disc slips the
// ~1.2 m corridor and a fast car drives through the ped mid-junction (binder shows junctionYield, not the
// crowd-brake). Inflating the vehicle-facing radius to ~1.5 m widens the corridor to egoHalf+1.5 = 2.4 m, so
// the disc is caught despite the projection error -- and because velocity is PRESERVED (unlike the velocity-0
// crossing-occupancy gate), the car brakes with margin but FOLLOWS the ped's motion rather than stopping dead,
// so it does not over-brake for a walking ORCA ped (that was the throughput cost of the velocity-0 gate).
//
// Only the vehicle-facing QueryNear is inflated; ORCA-ORCA avoidance uses the real physics radius via a
// different path (OrcaCrowd.GatherAgentNeighbours), so ped-ped behaviour is unchanged. Parity-inert: only on
// the CrowdSource (demo) path, never constructed for a golden/bench run.
public sealed class InflatedFootprintSource : ICrowdFootprintSource
{
    private readonly ICrowdFootprintSource _inner;
    private readonly double _extraRadius;

    public InflatedFootprintSource(ICrowdFootprintSource inner, double extraRadius)
    {
        _inner = inner ?? throw new ArgumentNullException(nameof(inner));
        _extraRadius = Math.Max(0.0, extraRadius);
    }

    public int QueryNear(double x, double y, double radius, Span<WorldDisc> into)
    {
        // The inner source writes its discs into `into`; inflate each in place (position + velocity kept).
        var n = _inner.QueryNear(x, y, radius, into);
        if (_extraRadius > 0.0)
        {
            for (var i = 0; i < n; i++)
            {
                var d = into[i];
                into[i] = new WorldDisc(d.X, d.Y, d.Vx, d.Vy, d.Radius + _extraRadius);
            }
        }

        return n;
    }
}
