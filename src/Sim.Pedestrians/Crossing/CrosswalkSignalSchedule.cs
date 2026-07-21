using System;
using System.Collections.Generic;
using System.Linq;

namespace Sim.Pedestrians.Crossing;

// Phase 2b (docs/LIVE-CITY-CROSSWALK-SIGNAL-DESIGN.md): the deterministic walk-window schedule for
// signalized crossings, so a low-power pedestrian can WAIT at the kerb on red and step onto the crossing
// only when it legitimately has the walk phase. A static <tlLogic> is a pure periodic function of time,
// so `NextWalkStart` is closed-form -- the inserted wait stays a pure function of time and server==IG is
// preserved (no runtime signal polling, no promotion).
//
// A crossing is "signalized" here iff its entry <connection> carries a `tl`+`linkIndex` AND that TL is a
// static program with a positive cycle. Unsignalized / actuated / unknown crossings are simply omitted --
// `NextWalkStart` returns the arrival time (no wait; the car-side occupancy gate covers those).
public sealed class CrosswalkSignalSchedule
{
    // For each signalized crossing edge id: the (merged, cycle-local) green windows [start,end) at which
    // the crossing's own linkIndex shows walk ('G'/'g'), plus the program's offset and cycle length.
    private readonly struct Signal
    {
        public readonly double Offset;
        public readonly double Cycle;
        public readonly (double Start, double End)[] Green; // sorted, non-overlapping, within [0, Cycle)

        public Signal(double offset, double cycle, (double, double)[] green)
        {
            Offset = offset;
            Cycle = cycle;
            Green = green;
        }
    }

    private readonly Dictionary<string, Signal> _byCrossing;

    private CrosswalkSignalSchedule(Dictionary<string, Signal> byCrossing) => _byCrossing = byCrossing;

    public int SignalizedCount => _byCrossing.Count;

    public bool IsSignalized(string crossingEdgeId) => _byCrossing.ContainsKey(crossingEdgeId);

    // Build from a net.net.xml for the given crossing edge ids. Reuses CrossingTlReader (the ped-side
    // independent read). `actuatedTlIds` (optional) names TLs whose program is detector-driven (not a
    // function of time) -- those crossings are omitted so they fall back to the always-yield gate.
    public static CrosswalkSignalSchedule FromNet(
        string netPath, IEnumerable<string> crossingEdgeIds, ISet<string>? actuatedTlIds = null)
    {
        var programs = CrossingTlReader.LoadPrograms(netPath);
        var map = new Dictionary<string, Signal>(StringComparer.Ordinal);

        foreach (var edgeId in crossingEdgeIds)
        {
            var link = CrossingTlReader.FindCrossingLink(netPath, edgeId);
            if (link is null || (actuatedTlIds?.Contains(link.TlId) ?? false))
            {
                continue; // unsignalized, or a non-deterministic (actuated) TL -> gate handles it
            }

            if (!programs.TryGetValue(link.TlId, out var prog) || prog.CycleLength <= 0.0)
            {
                continue;
            }

            var sig = BuildSignal(prog, link.LinkIndex);
            if (sig is { } s)
            {
                map[edgeId] = s;
            }
        }

        return new CrosswalkSignalSchedule(map);
    }

    // Build a single crossing's schedule straight from a program + linkIndex (used by tests and any
    // caller that already has the parsed program in hand).
    public static CrosswalkSignalSchedule ForCrossing(string crossingEdgeId, TlProgramSpec program, int linkIndex)
    {
        var map = new Dictionary<string, Signal>(StringComparer.Ordinal);
        var sig = BuildSignal(program, linkIndex);
        if (sig is { } s)
        {
            map[crossingEdgeId] = s;
        }

        return new CrosswalkSignalSchedule(map);
    }

    // Earliest absolute time >= `tArrive` at which a pedestrian may STEP ONTO `crossingEdgeId` and clear it
    // (needs `crossTime` seconds inside one continuous green window). Unsignalized/unknown -> `tArrive`.
    public double NextWalkStart(string crossingEdgeId, double tArrive, double crossTime)
    {
        if (!_byCrossing.TryGetValue(crossingEdgeId, out var sig) || sig.Green.Length == 0)
        {
            return tArrive; // not signalized here -> no wait
        }

        var cycle = sig.Cycle;
        // Base = start (absolute) of the cycle containing tArrive; local = tArrive within [0, cycle).
        var cyclesElapsed = Math.Floor((tArrive - sig.Offset) / cycle);
        var baseAbs = sig.Offset + (cyclesElapsed * cycle);
        var local = tArrive - baseAbs; // in [0, cycle)

        // Search this cycle and the next (a green window is at most one cycle ahead), so a window that
        // wraps the cycle boundary is caught by the next cycle's copy. Prefer the earliest window the ped
        // can FULLY clear; only if none fits (crossTime longer than every walk phase -- shouldn't happen)
        // fall back to the earliest green start (best effort).
        var bestFit = double.PositiveInfinity;
        var bestAny = double.PositiveInfinity;
        for (var k = 0; k <= 2; k++)
        {
            foreach (var (gStart, gEnd) in sig.Green)
            {
                var absStart = baseAbs + (k * cycle) + gStart;
                var absEnd = baseAbs + (k * cycle) + gEnd;
                if (absEnd <= tArrive)
                {
                    continue; // window already past
                }

                var stepOn = Math.Max(tArrive, absStart);
                if (stepOn < bestAny)
                {
                    bestAny = stepOn;
                }

                if (stepOn + crossTime <= absEnd + 1e-9 && stepOn < bestFit)
                {
                    bestFit = stepOn;
                }
            }
        }

        if (!double.IsPositiveInfinity(bestFit))
        {
            return bestFit;
        }

        return double.IsPositiveInfinity(bestAny) ? tArrive : bestAny;
    }

    // Merge the program's per-phase green intervals (state char at `linkIndex` in {'G','g'}) into windows.
    private static Signal? BuildSignal(TlProgramSpec program, int linkIndex)
    {
        var cycle = program.CycleLength;
        if (cycle <= 0.0)
        {
            return null;
        }

        var windows = new List<(double Start, double End)>();
        var t = 0.0;
        foreach (var phase in program.Phases)
        {
            var green = linkIndex >= 0 && linkIndex < phase.State.Length
                && (phase.State[linkIndex] == 'G' || phase.State[linkIndex] == 'g');
            if (green)
            {
                // Merge with the previous window if this phase abuts it (consecutive green phases).
                if (windows.Count > 0 && Math.Abs(windows[^1].End - t) < 1e-9)
                {
                    windows[^1] = (windows[^1].Start, t + phase.Duration);
                }
                else
                {
                    windows.Add((t, t + phase.Duration));
                }
            }

            t += phase.Duration;
        }

        if (windows.Count == 0)
        {
            return null; // never green for this link -> treat as no schedule (caller: always-yield gate)
        }

        return new Signal(program.Offset, cycle, windows.ToArray());
    }
}
