using Sim.Ingest;

namespace Sim.Core;

// Ported from sumo/src/microsim/traffic_lights/MSSimpleTrafficLightLogic.cpp's
// getPhaseIndexAtTime/trySwitch: a 'static' tlLogic program cycles through its phases in order,
// holding each for its `duration`, restarting at the top once the cycle (sum of all phase
// durations) elapses. This is a PURE function of (tlLogic, linkIndex, time) -- no per-vehicle or
// per-tlLogic mutable state is needed (unlike MSSimpleTrafficLightLogic's own myStep/
// myLastSwitch bookkeeping, which exists only to avoid recomputing this from scratch every
// query) -- so it slots into the plan phase without violating CLAUDE.md rule 3 (no shared-state
// writes during planning).
//
// Scoped out (see the rung-10 briefing): actuated/delay_based/rail_signal tlLogic types (only
// 'static'), nextPhases overrides, and myCurrentDurationIncrement (TraCI-only runtime overrides
// -- none exist in this scenario).
public static class TrafficLightState
{
    // Returns the link's state character (e.g. 'r', 'y', 'G', 'g') at `linkIndex` for the given
    // absolute simulation `time`. Mirrors getPhaseIndexAtTime's `(time - offset) mod cycle`
    // followed by a walk of cumulative phase durations to find the phase containing that
    // position -- MSSimpleTrafficLightLogic instead tracks this incrementally via myStep/
    // myLastSwitch and a scheduled SwitchCommand, but for a 'static' program (no runtime
    // overrides) that incremental bookkeeping and this pure recomputation agree exactly.
    public static char GetLinkState(TlLogic tlLogic, int linkIndex, double time)
    {
        var cycleLength = tlLogic.CycleLength;
        var position = (time - tlLogic.Offset) % cycleLength;
        if (position < 0)
        {
            // Defensive only: not exercised by this scenario (offset=0, time>=0 throughout), but
            // C#'s % can return a negative remainder where SUMOTime's integer modulo would not.
            position += cycleLength;
        }

        var cumulative = 0.0;
        foreach (var phase in tlLogic.Phases)
        {
            cumulative += phase.Duration;
            if (position < cumulative)
            {
                return phase.State[linkIndex];
            }
        }

        // Only reachable via floating-point rounding right at the cycle boundary (position
        // computed as ~cycleLength due to FP error); MSSimpleTrafficLightLogic's own integer
        // SUMOTime arithmetic never hits this -- fall back to the last phase rather than throw.
        return tlLogic.Phases[^1].State[linkIndex];
    }

    // 'r' (red) and 'y' (yellow) both close the link (MSLink::haveRed()/haveYellow(), checked
    // together as `yellowOrRed` in MSVehicle.cpp's planMoveInternal, ~line 2630) -- no yellow
    // phase exists in this scenario, but the check is written to match the source's actual
    // condition rather than a narrower "red-only" special case.
    public static bool IsRedOrYellow(char state) => state is 'r' or 'y';
}
