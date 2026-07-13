using Sim.Core.Bridge;
using Sim.Core.Orca;

namespace Sim.Evac;

// PANIC-EVAC-PHASE2-DESIGN.md §3: the "direct" perception gate. A pure, engine-free segment-vs-disc
// occlusion test: is `target` visible from `from` given a set of disc occluders (Phase 2: the
// abandoned-car WorldDiscs the director already tracks)? A disc only blocks if its CLOSEST approach to
// the segment lies STRICTLY BETWEEN the two endpoints -- an occluder behind `from` or beyond `target`
// does not block, matching "you can see past an obstacle that isn't actually in the way".
public static class LineOfSight
{
    public static bool IsVisible(Vec2 from, Vec2 target, IReadOnlyList<WorldDisc>? occluders)
    {
        if (occluders is null || occluders.Count == 0)
        {
            return true;
        }

        var seg = target - from;
        var segLenSq = seg.AbsSq;

        for (var i = 0; i < occluders.Count; i++)
        {
            var occ = occluders[i];
            var toCentre = new Vec2(occ.X, occ.Y) - from;

            // Projection parameter t of the disc centre onto the (infinite) line through from->target.
            // Only t in the OPEN interval (0,1) counts as "between the endpoints"; t<=0 is behind `from`,
            // t>=1 is beyond `target`.
            var t = segLenSq > 0.0 ? Vec2.Dot(toCentre, seg) / segLenSq : 0.0;

            if (t <= 0.0 || t >= 1.0)
            {
                continue;
            }

            var closest = from + t * seg;
            var offset = new Vec2(occ.X, occ.Y) - closest;
            if (offset.AbsSq < occ.Radius * occ.Radius)
            {
                return false;
            }
        }

        return true;
    }
}
