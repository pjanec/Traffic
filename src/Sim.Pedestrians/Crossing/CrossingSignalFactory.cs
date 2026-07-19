namespace Sim.Pedestrians.Crossing;

// Binds a PedNetworkParser-parsed PedCrossing to its ICrossingSignal by reading the crossing's
// tl-controlled entry link straight from the net.net.xml (CrossingTlReader).
//
// Investigation note (POC-2, docs/PEDESTRIAN-POC-PLAN.md): Engine.TlLaneHandles/TlStates
// (docs/SUMOSHARP-DEADRECKONING.md §5.2) was checked first as the preferred "live Engine signal"
// source. It does NOT cover pedestrian crossing links: Engine.BuildTlControlledLanes
// (src/Sim.Core/Engine.cs) enumerates only lanes whose id does not start with ':' (i.e. real,
// non-internal edges) and keeps those with a tl-controlled outgoing connection -- but a crossing's
// gating connection runs walkingarea -> crossing (e.g. ":c_w1" -> ":c_c0"), and BOTH endpoints are
// internal edges, so no crossing link is ever a candidate in that scan. Engine.TlStates is therefore
// structurally scoped to vehicle approach lanes only; extending it to cover pedestrian crossings would
// require an Engine.cs change, which is out of bounds for this additive Sim.Pedestrians-only POC.
// CrossingSignalFactory instead builds the "fallback deterministic phase clock" POC-2 calls for
// (TlProgramCrossingSignal) directly from the real <tlLogic> + <connection> data, so the walk/don't-
// walk timing genuinely matches what the live Engine would compute for this junction, evaluated at
// the caller's own Engine.CurrentTime -- a true Engine.TlStates-style binding remains a follow-up
// (extending that projection to include pedestrian links).
//
// Falls back to AlwaysWalkSignal for an unsignalized crossing (PedCrossing.TlLogicId == null) --
// nothing gates it, consistent with SUMO itself never emitting a <connection tl=...> for one.
public static class CrossingSignalFactory
{
    public static ICrossingSignal ForCrossing(string netPath, PedCrossing crossing)
    {
        if (crossing.TlLogicId is null)
        {
            return AlwaysWalkSignal.Instance;
        }

        var link = ResolveLink(netPath, crossing);

        var programs = CrossingTlReader.LoadPrograms(netPath);
        if (!programs.TryGetValue(link.TlId, out var program))
        {
            throw new InvalidOperationException($"tlLogic '{link.TlId}' not found in '{netPath}'.");
        }

        return new TlProgramCrossingSignal(program, link.LinkIndex);
    }

    // P4-1 (docs/PEDESTRIAN-TASKS.md §P4-1): bind the crossing to a LIVE signal reader (e.g.
    // Engine.TryGetTlLinkState) instead of the net-XML phase re-derivation ForCrossing does. Resolves the
    // crossing's (tlId, linkIndex) from the SAME net data, then reads the live char through `readLive` on
    // every query -- so the gate follows the Engine's actual signal, including an actuated program the XML
    // re-derivation cannot represent. `readLive(tlId, linkIndex)` is supplied by the caller (who owns the
    // Engine), keeping this factory + Sim.Pedestrians free of any Engine/Sim.Ingest reference. Unsignalized
    // crossings stay AlwaysWalk, exactly as ForCrossing.
    public static ICrossingSignal ForCrossingLive(string netPath, PedCrossing crossing, Func<string, int, char> readLive)
    {
        if (crossing.TlLogicId is null)
        {
            return AlwaysWalkSignal.Instance;
        }

        var link = ResolveLink(netPath, crossing);
        return new LiveCrossingSignal(_ => readLive(link.TlId, link.LinkIndex));
    }

    // The crossing's tl-controlled entry link (tlId, linkIndex), shared by ForCrossing (XML re-derivation)
    // and ForCrossingLive (live Engine read) so both bind to exactly the same link. Throws for a signalized
    // crossing whose gating <connection> is missing (a malformed net) -- the caller has already ruled out
    // the unsignalized (TlLogicId == null) case.
    private static CrossingLink ResolveLink(string netPath, PedCrossing crossing)
    {
        var crossingEdgeId = CrossingEdgeId(crossing);
        return CrossingTlReader.FindCrossingLink(netPath, crossingEdgeId)
            ?? throw new InvalidOperationException(
                $"crossing '{crossing.Id}' (edge '{crossingEdgeId}') has TlLogicId '{crossing.TlLogicId}' " +
                "but no tl-controlled entry <connection> was found.");
    }

    // PedCrossing.Id is the crossing LANE id (e.g. ":c_c0_0"); SUMO's internal-edge convention for a
    // crossing is single-lane ("<edgeId>_0"), so the owning edge id is the lane id with its trailing
    // "_<index>" suffix stripped.
    private static string CrossingEdgeId(PedCrossing crossing)
    {
        var lastUnderscore = crossing.Id.LastIndexOf('_');
        if (lastUnderscore < 0)
        {
            throw new InvalidOperationException($"crossing lane id '{crossing.Id}' has no '_<index>' suffix.");
        }

        return crossing.Id[..lastUnderscore];
    }
}
