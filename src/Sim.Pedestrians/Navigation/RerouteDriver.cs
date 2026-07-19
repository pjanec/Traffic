using Sim.Core.Orca;
using Sim.Pedestrians.Navigation.Bake;

namespace Sim.Pedestrians.Navigation;

// P2-2 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §4 "the strategic layer reroutes
// (IPedNavigation re-query)", §6 "full occlusion of a portal triggers a strategic reroute"):
// generalizes POC-5's one-shot "block a polygon, re-query FindPath once" (RerouteTests.cs) into a
// production driver that (a) watches a BlockerRegistry's blocked-polygon set for CHANGES, (b) applies
// production hysteresis so a transient blocker never thrashes anyone's route, and (c) reroutes only
// the pedestrians actually affected, leaving everyone else's path + waypoint cursor untouched.
//
// Ownership split (mirrors PedRouteController, P0-3): a ped is identified by the OrcaHandle OrcaCrowd
// already gave it -- the same stable identity every other piece of this codebase uses for "a live
// crowd agent" -- so this driver composes directly with an existing OrcaCrowd + PedRouteController
// pair with no new identity type for callers to juggle. RerouteDriver itself never calls
// OrcaCrowd.Step or PedRouteController.Update; it only reads positions (OrcaCrowd.Position) and hands
// back a new path via PathOf() for the caller to feed into PedRouteController.AddRoute (which already
// resets the waypoint cursor to 0 -- exactly the "resume from current ORCA-drifted position" behaviour
// docs/PEDESTRIAN-DESIGN.md §5 requires on any reroute/promotion).
//
// ---- Hysteresis (the production requirement) ----
//
// Two independent knobs, deliberately not conflated (docs §5's own note that POC-3 conflated a
// similar pair and flagged splitting them for production):
//
//   - `debounceSeconds` (BLOCKER-side): a polygon must be continuously present in the registry's raw
//     BlockedPolygons() for at least this long before it is treated as "effectively" blocked at all.
//     A blocker that flickers register/unregister faster than this never reaches "effective", so it
//     can never trigger a reroute -- see Update()'s debounce bookkeeping below. Unblocking is NOT
//     debounced (a polygon leaves the effective set the instant it leaves the raw set): the knob
//     exists to gate the RISK of committing a ped to an unnecessary detour, not to slow down clearing
//     one, and an immediate-clear still cannot cause a stale detour to un-reroute by itself (see the
//     "only reroute on newly-blocked" rule below) -- it just stops that polygon from being able to
//     force any FUTURE reroute until it debounces again.
//   - `commitDwellSeconds` (PED-side): once a ped has been rerouted, it will not be rerouted again
//     until this much time has passed, regardless of further blocked-set churn. This is what bounds
//     the reroute count for a ped caught near a flickering boundary even if the blocker-side debounce
//     alone were not enough.
//
// ---- What triggers a reroute (deliberately narrow, per the design's own wording) ----
//
// "when the blocked-polygon set CHANGES, reroute ONLY the peds whose current path traverses a
// NEWLY-blocked polygon" (docs/PEDESTRIAN-TASKS.md P2-2). This driver implements exactly that: only
// polygons that just transitioned from not-effectively-blocked to effectively-blocked can force a
// reroute. A polygon clearing never forces one -- a ped who detoured around a now-cleared obstacle
// simply keeps its (still perfectly valid, if no longer shortest) detour; nothing strands or jams.
// This also sidesteps a real hazard a naive "recompute for every ped on any change" design would hit:
// a ped's stored path is re-derived FROM ITS CURRENT (moving) POSITION, so recomputing for an
// unaffected ped on every tick would spuriously look "different" from its previous path purely
// because the ped walked in the meantime -- not a real reroute, just position drift. Restricting
// recomputation to peds whose EXISTING path geometrically enters a polygon that just became blocked
// (PathTraversesAny below) avoids ever recomputing for an unaffected ped in the first place.
public readonly record struct RerouteEvent(double Time, OrcaHandle Ped, int OldWaypointCount, int NewWaypointCount);

public sealed class RerouteDriver
{
    private sealed class PedEntry
    {
        public required Vec2 Goal;
        public required IReadOnlyList<Vec2> Path;
        public double CommitUntil = double.NegativeInfinity;
    }

    private readonly OrcaCrowd _crowd;
    private readonly SumoNavMesh _nav;
    private readonly IReadOnlyList<BakedPolygon> _polygons;
    private readonly double _debounceSeconds;
    private readonly double _commitDwellSeconds;

    private readonly Dictionary<OrcaHandle, PedEntry> _peds = new();

    // Blocker-side debounce state: polygon index -> the time it FIRST appeared, continuously, in the
    // registry's raw blocked set (reset to "absent" the instant it leaves the raw set -- see class
    // remarks on why unblocking is not itself debounced).
    private readonly Dictionary<int, double> _blockedSince = new();

    // The last COMMITTED (post-debounce) blocked-polygon set -- what SumoNavMesh.FindPath is actually
    // queried against on a reroute, and what Update() diffs the next raw set against to detect a
    // genuine, debounced change.
    private HashSet<int> _effectiveBlocked = new();

    private readonly List<RerouteEvent> _events = new();

    public RerouteDriver(
        OrcaCrowd crowd,
        SumoNavMesh nav,
        IReadOnlyList<BakedPolygon> polygons,
        double debounceSeconds,
        double commitDwellSeconds)
    {
        _crowd = crowd;
        _nav = nav;
        _polygons = polygons;
        _debounceSeconds = debounceSeconds;
        _commitDwellSeconds = commitDwellSeconds;
    }

    // Every reroute this driver has ever committed, in the order committed (Update() always visits
    // peds in ascending OrcaHandle.Index order, so this sequence is itself deterministic -- the
    // Determinism success condition's "identical reroute event sequence" run-to-run).
    public IReadOnlyList<RerouteEvent> Events => _events;

    // The blocked set currently in force for routing purposes (post-debounce) -- for tests/telemetry;
    // FindPath calls this driver makes always use exactly this set.
    public IReadOnlySet<int> EffectiveBlockedPolygons => _effectiveBlocked;

    public int PedCount => _peds.Count;

    // Registers an already-added crowd agent (handle) with its current path + goal. `path` is
    // whatever the caller already computed (typically nav.FindPath(start, goal) unblocked, or
    // pre-blocked if the ped spawns into an already-blocked area) -- this driver only decides WHEN to
    // replace it, never how the first one was made.
    public void RegisterPed(OrcaHandle handle, Vec2 goal, IReadOnlyList<Vec2> path)
    {
        _peds[handle] = new PedEntry { Goal = goal, Path = path };
    }

    public bool RemovePed(OrcaHandle handle) => _peds.Remove(handle);

    public bool IsRegistered(OrcaHandle handle) => _peds.ContainsKey(handle);

    public IReadOnlyList<Vec2> PathOf(OrcaHandle handle) => _peds[handle].Path;

    // Advances the blocker-side debounce filter from `rawBlocked` (typically
    // BlockerRegistry.BlockedPolygons(), sampled at `time`) and, if that changes what is
    // EFFECTIVELY blocked, reroutes exactly the affected peds. Deterministic: no System.Random, fixed
    // ascending-OrcaHandle.Index ped order, and FindPath itself is already deterministic (SumoNavMesh).
    public void Update(double time, IReadOnlySet<int> rawBlocked)
    {
        AdvanceDebounce(time, rawBlocked);

        var newEffective = ComputeEffective(time);
        if (newEffective.SetEquals(_effectiveBlocked))
        {
            return; // nothing crossed the debounce threshold since last Update -- cheapest common case
        }

        // Only NEWLY-blocked polygons (present now, absent from the previous committed set) can force
        // a reroute (see class remarks) -- a polygon leaving the effective set is never itself cause
        // to touch any ped.
        var newlyBlocked = new HashSet<int>(newEffective);
        newlyBlocked.ExceptWith(_effectiveBlocked);
        _effectiveBlocked = newEffective;

        if (newlyBlocked.Count == 0)
        {
            return; // this commit only cleared polygons; nothing forces a reroute
        }

        RerouteAffected(time, newlyBlocked);
    }

    private void AdvanceDebounce(double time, IReadOnlySet<int> rawBlocked)
    {
        foreach (var idx in rawBlocked)
        {
            if (!_blockedSince.ContainsKey(idx))
            {
                _blockedSince[idx] = time;
            }
        }

        if (_blockedSince.Count == 0)
        {
            return;
        }

        List<int>? toRemove = null;
        foreach (var idx in _blockedSince.Keys)
        {
            if (!rawBlocked.Contains(idx))
            {
                (toRemove ??= new List<int>()).Add(idx);
            }
        }

        if (toRemove is not null)
        {
            foreach (var idx in toRemove)
            {
                _blockedSince.Remove(idx); // left the raw set before debouncing: clock resets, no accumulation
            }
        }
    }

    private HashSet<int> ComputeEffective(double time)
    {
        var effective = new HashSet<int>();
        foreach (var (idx, since) in _blockedSince)
        {
            if (time - since >= _debounceSeconds)
            {
                effective.Add(idx);
            }
        }

        return effective;
    }

    private void RerouteAffected(double time, HashSet<int> newlyBlocked)
    {
        var handles = new List<OrcaHandle>(_peds.Keys);
        handles.Sort((a, b) => a.Index.CompareTo(b.Index)); // deterministic, fixed order

        List<OrcaHandle>? stale = null;

        foreach (var handle in handles)
        {
            var entry = _peds[handle];

            if (time < entry.CommitUntil)
            {
                continue; // still committed to its last reroute (per-ped hysteresis)
            }

            if (!PathTraversesAny(entry.Path, newlyBlocked))
            {
                continue; // unaffected: path + waypoint cursor stay exactly as they are
            }

            if (!_crowd.IsAlive(handle))
            {
                (stale ??= new List<OrcaHandle>()).Add(handle); // despawned since last Update; clean up
                continue;
            }

            var position = _crowd.Position(handle);
            var newPath = _nav.FindPath(position, entry.Goal, _effectiveBlocked);
            if (newPath is null)
            {
                continue; // goal unreachable under the current blocks: keep the old path rather than strand the ped with none
            }

            _events.Add(new RerouteEvent(time, handle, entry.Path.Count, newPath.Count));
            entry.Path = newPath;
            entry.CommitUntil = time + _commitDwellSeconds;
        }

        if (stale is not null)
        {
            foreach (var handle in stale)
            {
                _peds.Remove(handle);
            }
        }
    }

    // True when `path`'s polyline meaningfully enters ANY polygon named in `polygonIndices` --
    // densely sampled per segment (never just the endpoints, which for a FindPath-produced path are
    // often exact portal points sitting ON a shared polygon edge, an ambiguous case for a plain
    // point-in-polygon test). Mirrors RerouteTests.cs's own PathEntersPolygon sampling approach
    // (production analogue of that test-only helper).
    private bool PathTraversesAny(IReadOnlyList<Vec2> path, HashSet<int> polygonIndices)
    {
        if (polygonIndices.Count == 0)
        {
            return false;
        }

        foreach (var idx in polygonIndices)
        {
            if (idx < 0 || idx >= _polygons.Count)
            {
                continue;
            }

            if (PathTraversesPolygon(path, _polygons[idx].Vertices))
            {
                return true;
            }
        }

        return false;
    }

    private static bool PathTraversesPolygon(IReadOnlyList<Vec2> path, IReadOnlyList<Vec2> polygonVertices)
    {
        for (var i = 0; i + 1 < path.Count; i++)
        {
            var a = path[i];
            var b = path[i + 1];
            var length = (b - a).Abs;
            var steps = Math.Max(8, (int)(length / 0.25));

            // Interior samples only (s in (0, steps)): the segment's own endpoints are frequently
            // exact portal/start/goal points sitting on a polygon boundary, not a genuine interior
            // traversal.
            for (var s = 1; s < steps; s++)
            {
                var t = (double)s / steps;
                var sample = new Vec2(a.X + ((b.X - a.X) * t), a.Y + ((b.Y - a.Y) * t));
                if (PolygonGeometry.Contains(polygonVertices, sample))
                {
                    return true;
                }
            }
        }

        return false;
    }
}
