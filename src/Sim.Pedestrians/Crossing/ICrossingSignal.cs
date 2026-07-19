namespace Sim.Pedestrians.Crossing;

// POC-2 (docs/PEDESTRIAN-POC-PLAN.md POC-2; docs/PEDESTRIAN-DESIGN.md §6, decision D5): the crosswalk
// gate's walk/don't-walk source. Deliberately the smallest possible seam -- a pure function of the
// caller's own clock -- so CrossingGate never needs to know WHERE the signal comes from (a live
// junction program, a scripted demo clock, or a test double).
public interface ICrossingSignal
{
    // True when pedestrians may walk onto the gated crossing at absolute time `now`.
    bool WalkAllowed(double now);
}

// Trivial always-open / always-closed signals, useful for tests and for an unsignalized crossing
// (PedCrossing.TlLogicId == null) that should never hold pedestrians.
public sealed class AlwaysWalkSignal : ICrossingSignal
{
    public static readonly AlwaysWalkSignal Instance = new();
    public bool WalkAllowed(double now) => true;
}

public sealed class NeverWalkSignal : ICrossingSignal
{
    public static readonly NeverWalkSignal Instance = new();
    public bool WalkAllowed(double now) => false;
}

// P4-1 (docs/PEDESTRIAN-TASKS.md §P4-1): reads the walk/don't-walk char from a LIVE external source each
// query -- typically a bound Engine.TryGetTlLinkState projection -- instead of re-deriving the phase from
// the net XML (TlProgramCrossingSignal). Tracking the Engine's ACTUAL signal is what makes an actuated
// (non-time-formula) crossing program gate correctly, and keeps a single source of truth for the signal.
//
// The `Func` is deliberately the whole seam: it keeps Sim.Pedestrians from structurally depending on the
// vehicle Engine (Sim.Core.Engine / Sim.Ingest -- forbidden by PEDESTRIAN-DESIGN.md §0 Principle 6).
// Whoever owns BOTH the Engine and the ped world supplies the reader (e.g.
// `now => engine.TryGetTlLinkState(tlId, linkIndex, out var c) ? c : 'r'`); this class never names an
// Engine type. SUMO gates a crossing entry link with 'G' (walk) / 'r' (don't walk) only, so WalkAllowed is
// exactly `state == 'G'` -- identical to TlProgramCrossingSignal's own rule, just off a live char.
public sealed class LiveCrossingSignal : ICrossingSignal
{
    private readonly Func<double, char> _stateAt;

    // `stateAt(now)` returns the crossing entry link's live signal char at absolute time `now`. A live
    // Engine reader ignores `now` and reads its own CurrentTime; a time-formula reader may use it.
    public LiveCrossingSignal(Func<double, char> stateAt) => _stateAt = stateAt;

    public bool WalkAllowed(double now) => _stateAt(now) == 'G';

    // The raw live char at `now` (exposed for tests / observability -- distinguishing 'r' don't-walk from a
    // would-be clearance char, and cross-checking against the Engine projection), mirroring
    // TlProgramCrossingSignal.StateAt.
    public char StateAt(double now) => _stateAt(now);
}
