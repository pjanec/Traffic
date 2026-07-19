using Sim.Core;
using Sim.Pedestrians.Crossing;
using Xunit;

namespace Sim.Pedestrians.Tests.Crossing;

// P4-1 (docs/PEDESTRIAN-TASKS.md §P4-1): the crossing gate, fed the LIVE Engine signal projection
// (Engine.TryGetTlLinkState), opens/closes exactly with the Engine's own crossing signal -- rather than
// re-deriving the phase from the net XML (TlProgramCrossingSignal). Proven here against POC-0's real
// net.net.xml + <tlLogic id="c"> by stepping an Engine through a full signal cycle and cross-checking the
// LiveCrossingSignal at every step:
//   (a) the projection reaches the pedestrian crossing links (20-23) the vehicle-facing Engine.TlStates
//       structurally cannot (both endpoints internal -- see Engine.TryGetTlLinkState's remarks), and
//   (b) for POC-0's 'static' program the live read agrees EXACTLY with the XML re-derivation the gate uses
//       today, i.e. the live projection is a faithful drop-in (the actuated case, where the two would
//       DIVERGE, rides Engine.TlLinkStateChar's already-parity-tested actuated branch).
public class EngineLiveCrossingSignalTests
{
    private static string NetPath => Path.Combine(AppContext.BaseDirectory, "Fixtures", "net.net.xml");

    // west crossing ":c_c3_0" -> tlLogic "c" linkIndex 23 (CrossingTlReaderTests' hand-derived table);
    // walks during phase0 [0,37) of the 90s cycle.
    private const string WestCrossingLaneId = ":c_c3_0";

    // A live reader over the Engine projection, exactly as an integration layer would wire it: read the live
    // char, fail closed ('r' don't-walk) if the link can't be resolved (never hit for a valid net).
    private static Func<string, int, char> LiveReader(Engine engine)
        => (tl, li) => engine.TryGetTlLinkState(tl, li, out var c) ? c : 'r';

    [Fact]
    public void LiveCrossingSignal_TracksEngineCrossingSignal_ExactlyOverAFullCycle()
    {
        var engine = new Engine();
        engine.LoadNetwork(NetPath);

        var network = PedNetworkParser.Load(NetPath);
        var westCrossing = network.Crossings.Single(c => c.Id == WestCrossingLaneId);
        Assert.Equal("c", westCrossing.TlLogicId);

        var live = CrossingSignalFactory.ForCrossingLive(NetPath, westCrossing, LiveReader(engine));
        var reDerived = CrossingSignalFactory.ForCrossing(NetPath, westCrossing); // the XML re-derivation (today's gate)

        var sawWalk = false;
        var sawDontWalk = false;

        // POC-0's cycle is 90s; the Engine's default StepLength is 1.0s, so 100 steps covers a full cycle
        // and change. Evaluate BOTH signals at the Engine's own CurrentTime each step so they align.
        for (var step = 0; step < 100; step++)
        {
            engine.Step();
            var now = engine.CurrentTime;

            // (a) the projection reaches the crossing link and returns a valid crossing char.
            Assert.True(engine.TryGetTlLinkState("c", 23, out var liveChar),
                "Engine.TryGetTlLinkState should resolve the crossing link (tl 'c', linkIndex 23)");
            Assert.True(liveChar is 'G' or 'r', $"unexpected crossing char '{liveChar}' at t={now}");

            // (b) the live gate decision equals the XML re-derivation, exactly, at every step.
            Assert.Equal(reDerived.WalkAllowed(now), live.WalkAllowed(now));

            // and the LiveCrossingSignal's own char read matches the projection.
            Assert.Equal(liveChar, ((LiveCrossingSignal)live).StateAt(now));

            if (live.WalkAllowed(now))
            {
                sawWalk = true;
            }
            else
            {
                sawDontWalk = true;
            }
        }

        // Non-vacuous: the signal genuinely flipped both ways over the cycle (not a stuck always-open /
        // always-closed read).
        Assert.True(sawWalk, "expected the crossing to be walk-allowed at some point in the cycle");
        Assert.True(sawDontWalk, "expected the crossing to be don't-walk at some point in the cycle");
    }

    [Fact]
    public void TryGetTlLinkState_UnknownTlOrOutOfRangeLink_ReturnsFalse()
    {
        var engine = new Engine();
        engine.LoadNetwork(NetPath);
        engine.Step();

        Assert.False(engine.TryGetTlLinkState("does_not_exist", 0, out var s1));
        Assert.Equal('\0', s1);

        Assert.False(engine.TryGetTlLinkState("c", 9999, out var s2));
        Assert.Equal('\0', s2);

        Assert.False(engine.TryGetTlLinkState("c", -1, out var s3));
        Assert.Equal('\0', s3);

        // A valid link still resolves (sanity that the negatives above aren't false for the wrong reason).
        Assert.True(engine.TryGetTlLinkState("c", 23, out var s4));
        Assert.True(s4 is 'G' or 'r');
    }

    // An unsignalized crossing (no TlLogicId) binds to AlwaysWalk under the live factory too -- nothing
    // gates it, so the live reader is never even consulted.
    [Fact]
    public void ForCrossingLive_UnsignalizedCrossing_IsAlwaysWalk()
    {
        var engine = new Engine();
        engine.LoadNetwork(NetPath);

        // TlLogicId: null short-circuits ForCrossingLive to AlwaysWalk before any net/link resolution, so
        // the other fields are just a well-formed placeholder.
        var noPoints = System.Array.Empty<Sim.Core.Orca.Vec2>();
        var unsignalized = new PedCrossing(
            Id: ":x_c0_0", JunctionId: "x", Width: 4.0, Shape: noPoints, Outline: noPoints,
            CrossingEdges: System.Array.Empty<string>(), TlLogicId: null);

        var signal = CrossingSignalFactory.ForCrossingLive(NetPath, unsignalized, LiveReader(engine));

        Assert.IsType<AlwaysWalkSignal>(signal);
        Assert.True(signal.WalkAllowed(0.0));
        Assert.True(signal.WalkAllowed(50.0));
    }
}
