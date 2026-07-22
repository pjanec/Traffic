using System.Diagnostics;
using Sim.Core;
using Sim.Replication;

namespace Sim.Viewer.Motion;

// docs/SUMOSHARP-NATIVE-VIEWER.md P2b / SUMOSHARP-DEADRECKONING.md §7/§8 — a direct port of
// Sim.LiveHost/HtmlPage.cs's dead-reckoning pacing: `ingestFrame`'s long-baseline wall<->sim rate fit, the
// render loop's continuous strictly-monotonic `renderSim`, and `resolvePose`'s interpolate/extrapolate
// switch on the `delay` knob. Transport-agnostic: `Pump` is fed whatever "sim-axis" timestamp the newest
// vehicle sample carries (the transport's newest-per-vehicle-sample timestamp, e.g. a DDS reader's
// LatestVehicleSampleTime) and supplies its OWN real wall-clock reading (a Stopwatch) for the other side of
// the fit — mirroring HtmlPage.cs's split between
// `performance.now()` (local wall clock) and `m.time` (the server-sent sample time). No rendering here.
public sealed class DrClock
{
    private readonly Stopwatch _wall = Stopwatch.StartNew();

    private double? _firstWallSec;
    private double _firstSim;
    private double _lastIngestedSim = double.NaN;
    private double _latestSim;

    // HtmlPage.cs: `let simRate = 2;` — a placeholder until the long-baseline fit (baseSec > 1.0) kicks in.
    // Ported value is 2 there because the demo's "sim" axis is genuine sim-seconds at a roughly-2 Hz step
    // rate; here the "sim" axis fed to Pump is whatever the transport's newest-per-vehicle-sample timestamp
    // carries (usually the DDS SourceTimestamp, i.e. real wall-clock seconds), which advances 1:1 with wall time
    // by construction — 1.0 is the correct placeholder for that axis and avoids an initial-guess
    // overshoot (see Pump's pre-baseline guard below) that would otherwise register spurious back-steps.
    private double _simRate = 1.0;
    private double _renderSim;
    private double _lastPumpWallSec = -1.0;
    private int _backSteps;

    // --- clock-health readout (SUMOSHARP-NATIVE-VIEWER.md P2b) ---
    public double RenderSim => _renderSim;
    public double SimRate => _simRate;
    public int BackSteps => _backSteps;
    public double EffectiveDelay { get; private set; }

    // EMA of the wall-clock gap between successive NEW distinct samples (Pump's dedupe below), i.e. the
    // measured DDS packet inter-arrival interval. Feeds the "always interpolate" auto-delay (Renderer.cs /
    // Program.cs): delay = ~1.5x this keeps the render clock reliably behind the newest packet even under
    // jitter, so Resolve always lands in the interpolate branch instead of extrapolating.
    public double AvgSampleInterval { get; private set; }

    // Re-anchor the clock to a fresh timeline (publisher restart at t=0). The long-baseline fit + renderSim
    // are cleared so they re-fit from the new stream instead of staying far ahead of it (which would
    // extrapolate wildly). Cumulative back-step count is left as a running health metric.
    public void Reset()
    {
        _firstWallSec = null;
        _lastIngestedSim = double.NaN;
        _latestSim = 0.0;
        _renderSim = 0.0;
        _simRate = 1.0;
        AvgSampleInterval = 0.0;
    }

    public readonly struct Resolved
    {
        public Resolved(DrState state, UpcomingLanes upcoming, bool extrapolated)
        {
            State = state;
            Upcoming = upcoming;
            Extrapolated = extrapolated;
            SecondState = null;
            SecondUpcoming = default;
            Blend = 0f;
            PacketSpan = 0f;
        }

        // Lateral-straddle result (SUMOSHARP-LANE-CHANGE-SMOOTHING-DESIGN.md §4.1): `state`/`upcoming` is
        // packet `a` (old lane), `secondState`/`secondUpcoming` is packet `b` (sibling lane), `blend` is the
        // interpolation fraction `f` between them. The caller (PumpAndBuildVehicleDraws) resolves each on
        // its own lane and Cartesian-lerps the two world poses (design §4.2) — DrClock itself carries no
        // pose/geometry knowledge beyond ArcInWindow's classification. `packetSpan` (optional, defaults to 0
        // so the pinned 5-arg signature still compiles) is the REAL a->b wall-clock gap for this specific
        // bracket -- see PacketSpan below for why the caller's sanity guard needs it instead of the smoothed
        // average sample interval.
        public Resolved(
            DrState stateA, UpcomingLanes upcomingA, DrState stateB, UpcomingLanes upcomingB, float blend,
            float packetSpan = 0f)
        {
            State = stateA;
            Upcoming = upcomingA;
            Extrapolated = false;
            SecondState = stateB;
            SecondUpcoming = upcomingB;
            Blend = blend;
            PacketSpan = packetSpan;
        }

        // Render-time state: Pos/PosLat are already advanced to render time (dt already applied here), so
        // a caller feeds this to PoseResolver.Resolve with dt=0 (Length/Width are NOT set — the caller
        // fills those in from the lifecycle/dims registry via `State with { Length = ..., Width = ... }`).
        public DrState State { get; }

        // The lane-path (current lane first) of whichever buffered packet the resolution used — the
        // caller's `upcomingLanes` span for PoseResolver.
        public UpcomingLanes Upcoming { get; }

        // True if this frame extrapolated (forward past the newest packet, backward past the oldest, or
        // because the two bracketing packets straddled a lane change too large for the arc-window) rather
        // than interpolated exactly.
        public bool Extrapolated { get; }

        // Lateral straddle only (see two-state ctor above): the second bracketing packet's state, its own
        // upcoming-lane window (resolved on ITS lane, not `a`'s), and the a->b interpolation fraction.
        public DrState? SecondState { get; }
        public UpcomingLanes SecondUpcoming { get; }
        public float Blend { get; }
        public bool IsLateralStraddle => SecondState.HasValue;

        // The REAL wall-clock gap (b.TimestampSeconds - a.TimestampSeconds) between this specific bracketing
        // pair, for the caller's §4.2 sanity guard. NOT the same as DrClock.AvgSampleInterval (a smoothed EMA
        // across ALL packets): a lane-change bracket's actual span can run well above the smoothed average
        // (observed on 12-overtake: avgint~1.2s but this pair's real span ~2.9s), so gating the guard on the
        // average under-estimates a perfectly normal slide's chord length and would wrongly snap it.
        public float PacketSpan { get; }
    }

    // Call once per app frame (regardless of whether new DDS data arrived this frame). `newestSampleTime`
    // is the newest per-vehicle sample timestamp seen so far (the transport's own LatestVehicleSampleTime
    // reading); repeated/unchanged values between DDS arrivals are ignored for the rate fit (mirrors HtmlPage.cs
    // ingestFrame's `if(m.step === lastStep) return` dedupe) but the render clock still advances every call.
    // `hold` (the sim is paused): freeze the render clock -- do NOT advance renderSim past the newest sample,
    // so a paused publisher's vehicles stop immediately instead of coasting on extrapolation for ~3s. The
    // sample-time ingest still runs (so the rate fit + restart detection stay live for when it resumes).
    public void Pump(double? newestSampleTime, bool hold = false)
    {
        var nowWall = _wall.Elapsed.TotalSeconds;

        if (newestSampleTime is { } sim && sim != _lastIngestedSim)
        {
            // Track the packet inter-arrival interval (EMA) for the "always interpolate" auto-delay. Guarded
            // against the restart case below (a huge negative gap) and against absurd outliers -- either
            // would otherwise drag the average somewhere useless for one packet.
            if (!double.IsNaN(_lastIngestedSim))
            {
                var interval = sim - _lastIngestedSim;
                if (interval > 0.001 && interval < 5.0)
                {
                    AvgSampleInterval = AvgSampleInterval <= 0.0
                        ? interval
                        : AvgSampleInterval + (interval - AvgSampleInterval) * 0.2;
                }
            }

            // Restart detection: the sim-axis timestamp jumping BACKWARD (the publisher rebuilt at t=0) breaks
            // the long-baseline fit and would strand renderSim far ahead of the stream -> wild forward
            // extrapolation (vehicles racing off after a restart). Re-anchor the fit + clock to the new
            // timeline. A small backward wobble is normal jitter; only a real reset trips this.
            if (!double.IsNaN(_lastIngestedSim) && sim < _lastIngestedSim - 2.0)
            {
                _firstWallSec = null;
                _renderSim = 0.0;
                _simRate = 1.0;
            }

            _lastIngestedSim = sim;

            // Long-baseline playback rate: total sim advanced / total wall elapsed since the first sample.
            // The growing denominator makes it immune to per-packet burst jitter (HtmlPage.cs's comment on
            // `ingestFrame`), so the clock runs at a steady velocity instead of racing-and-snapping.
            if (_firstWallSec is null)
            {
                _firstWallSec = nowWall;
                _firstSim = sim;
            }
            else
            {
                var baseSec = nowWall - _firstWallSec.Value;
                if (baseSec > 1.0)
                {
                    var r = (sim - _firstSim) / baseSec;
                    if (r > 0.2 && r < 20.0)
                    {
                        _simRate = r;
                    }
                }
            }

            _latestSim = sim;
        }

        if (hold)
        {
            // Paused: hold renderSim where it is (no advance -> no extrapolation coast). Keep the frame-dt
            // anchor current so the first frame after resume doesn't see a huge dt.
            _lastPumpWallSec = nowWall;
            return;
        }

        // Render clock: advance MONOTONICALLY toward the smooth wall->sim estimate (floored at the newest
        // sample so delay=0 stays pure extrapolation). Forward-only: if the estimate dips (jitter) we HOLD
        // rather than step back (HtmlPage.cs draw()'s clock comment) — a momentary hold is invisible, a
        // backward step is a visible back-jump. Each held regression is counted as a back-step "attempt".
        //
        // Pre-baseline guard (not in HtmlPage.cs, needed here): before the long-baseline fit has its first
        // second of data, `_simRate` is only a placeholder guess. Projecting `_firstSim + guess*elapsed`
        // during that window can overshoot the true rate and push the render clock ahead of reality; once
        // the fit corrects, `est` would then sit BEHIND the already-advanced renderSim -- a spurious
        // "back-step" that is really just this bootstrap transient, not real jitter. Holding at `_latestSim`
        // until the fit is live avoids the overshoot entirely (and is exactly what the fit converges to
        // continuously at the baseSec > 1.0 boundary, so there is no visible discontinuity either).
        var haveStableFit = _firstWallSec is not null && nowWall - _firstWallSec.Value > 1.0;
        var clockNow = haveStableFit
            ? _firstSim + _simRate * (nowWall - _firstWallSec!.Value)
            : _latestSim;
        var est = Math.Max(clockNow, _latestSim);

        var frameDt = _lastPumpWallSec >= 0.0 ? Math.Min(0.1, nowWall - _lastPumpWallSec) : 0.016;
        _lastPumpWallSec = nowWall;

        if (_renderSim == 0.0 || est > _renderSim + 3.0)
        {
            _renderSim = est; // init / restart -> jump forward
        }
        else if (est > _renderSim)
        {
            _renderSim += Math.Min(est - _renderSim, frameDt * _simRate * 3.0); // capped catch-up
        }
        else if (est < _renderSim - 1e-9)
        {
            _backSteps++; // would-be backward step -- held instead; _renderSim itself never decreases
        }
    }

    // Resolve one vehicle's render-time DrState from its buffered history (newest last, capped by the
    // history's own capacity -- e.g. Sim.Replication.VehicleSampleHistory) + the interactive playout `delay`
    // seconds (HtmlPage.cs's resolvePose): delay=0 extrapolates forward from the newest packet; delay>0
    // interpolates between the two buffered packets bracketing (renderSim - delay), falling back to
    // extrapolation past the newest/oldest packet or across a lane change between the two bracketing packets
    // (HtmlPage.cs's `arcInWindow` null case).
    public Resolved Resolve(IReadOnlyList<TimestampedSample> history, double delay, ILaneShapeSource lanes)
    {
        if (history.Count == 0)
        {
            throw new ArgumentException("history must have at least one sample.", nameof(history));
        }

        EffectiveDelay = delay;

        // The live viewer's query instant is renderSim (Pump'd from the wall clock) minus the playout
        // delay. ResolveAt carries the actual interpolate/extrapolate logic (see its remarks).
        return ResolveAt(history, _renderSim - delay, lanes);
    }

    // Deterministic sibling of Resolve (docs/IGBRIDGE-DECISIONS.md §5.1). Resolve derives its query
    // instant from `_renderSim` (advanced by Pump from a real Stopwatch — correct for a live 60 Hz
    // viewer, but non-reproducible for an offline trace generator). ResolveAt takes the render-time
    // `sampleT` DIRECTLY, so a caller that owns a deterministic sim clock (IgBridge) can reconstruct
    // without touching Pump/the Stopwatch. This is a PURE EXTRACTION of the former Resolve body: the
    // viewer keeps calling Resolve (which now delegates here) and is byte-identical. EffectiveDelay is
    // intentionally left to the delay-based Resolve entry point; a direct ResolveAt caller drives the
    // query instant itself, so there is no "delay behind newest" to report.
    public Resolved ResolveAt(IReadOnlyList<TimestampedSample> history, double sampleT, ILaneShapeSource lanes)
    {
        if (history.Count == 0)
        {
            throw new ArgumentException("history must have at least one sample.", nameof(history));
        }

        var newest = history[^1];
        var oldest = history[0];

        VehicleRecord pk;
        double posLat, arc;
        bool extrapolated;

        if (sampleT >= newest.TimestampSeconds)
        {
            // EXTRAPOLATE forward from the newest packet.
            var dt = Math.Min(sampleT - newest.TimestampSeconds, 3.0);
            pk = newest.Record;
            posLat = pk.PosLat;
            arc = ExtrapolateArc(pk.Pos, pk.Speed, pk.Accel, dt);
            extrapolated = true;
        }
        else if (sampleT <= oldest.TimestampSeconds)
        {
            // Older than the buffer -> extrapolate from the oldest (HtmlPage.cs's same fallback).
            var dt = Math.Max(sampleT - oldest.TimestampSeconds, -3.0);
            pk = oldest.Record;
            posLat = pk.PosLat;
            arc = ExtrapolateArc(pk.Pos, pk.Speed, pk.Accel, dt);
            extrapolated = true;
        }
        else
        {
            // INTERPOLATE between the bracketing packets a (last sample with t <= sampleT) and b (the next).
            var ai = 0;
            for (var i = history.Count - 1; i >= 0; i--)
            {
                if (history[i].TimestampSeconds <= sampleT)
                {
                    ai = i;
                    break;
                }
            }

            var a = history[ai];
            var b = history[Math.Min(ai + 1, history.Count - 1)];
            var span = b.TimestampSeconds - a.TimestampSeconds;
            var f = span > 1e-6 ? (sampleT - a.TimestampSeconds) / span : 0.0;

            if (a.Record.LaneHandle == b.Record.LaneHandle)
            {
                // Same lane in both packets -> a straight arc-length lerp is exact (no lane-window walk
                // needed, unlike HtmlPage.cs's arcInWindow, because PoseResolver's own forward walk handles
                // crossing into the next lane from a single arc-length value).
                pk = b.Record;
                posLat = a.Record.PosLat + (b.Record.PosLat - a.Record.PosLat) * f;
                arc = a.Record.Pos + (b.Record.Pos - a.Record.Pos) * f;
                extrapolated = false;
            }
            else
            {
                // The vehicle crossed onto a new lane between the two buffered packets, so `b`'s arc-length is
                // on a different lane than `a`'s and the two can't be lerped directly. Extrapolating along
                // `a`'s lane (the old fallback) overshoots the turn then snaps onto `b`'s lane -> the visible
                // lateral jump through a junction. Instead, express `b`'s position as an arc offset in `a`'s
                // FORWARD lane window (arcInWindow) and interpolate the arc there: PoseResolver then walks
                // `a`'s upcoming lanes -- through the curved internal junction lanes -- following the real
                // geometry (no corner-cut, no snap). This is HtmlPage.cs's `arcInWindow` interpolation, just
                // anchored on `a` (whose window is forward-only, matching our UpcomingLanes).
                var arcB = ArcInWindow(a.Record, b.Record.LaneHandle, lanes);
                if (arcB is { } arcBVal)
                {
                    pk = a.Record;
                    posLat = a.Record.PosLat + (b.Record.PosLat - a.Record.PosLat) * f;
                    arc = a.Record.Pos + (arcBVal + b.Record.Pos - a.Record.Pos) * f;
                    extrapolated = false;
                }
                else
                {
                    // `b`'s lane isn't within `a`'s forward window -> not a downstream/junction straddle at
                    // all, but a LATERAL one: `b` is a sibling lane beside `a` (a lane change), not ahead of
                    // it. Extrapolating along `a`'s old lane (the old fallback) never leaves it, then snaps
                    // when `b` becomes current (SUMOSHARP-LANE-CHANGE-SMOOTHING-DESIGN.md §1/§3). Instead,
                    // return both bracketing states + the fraction `f` so the caller can resolve each on its
                    // own lane and Cartesian-lerp the two world poses (design §4.2).
                    var stateA = new DrState
                    {
                        Model = a.Record.Model,
                        LaneHandle = a.Record.LaneHandle,
                        Pos = a.Record.Pos,
                        PosLat = a.Record.PosLat,
                        Speed = a.Record.Speed,
                        Accel = a.Record.Accel,
                        LatSpeed = a.Record.LatSpeed,
                    };
                    var stateB = new DrState
                    {
                        Model = b.Record.Model,
                        LaneHandle = b.Record.LaneHandle,
                        Pos = b.Record.Pos,
                        PosLat = b.Record.PosLat,
                        Speed = b.Record.Speed,
                        Accel = b.Record.Accel,
                        LatSpeed = b.Record.LatSpeed,
                    };
                    return new Resolved(
                        stateA, NormalizeUpcoming(a.Record.LaneHandle, a.Record.Upcoming),
                        stateB, NormalizeUpcoming(b.Record.LaneHandle, b.Record.Upcoming),
                        (float)f, (float)span);
                }
            }
        }

        var state = new DrState
        {
            Model = pk.Model,
            LaneHandle = pk.LaneHandle,
            Pos = arc,
            PosLat = posLat,
            Speed = pk.Speed,
            Accel = pk.Accel,
            LatSpeed = pk.LatSpeed,
        };

        return new Resolved(state, NormalizeUpcoming(pk.LaneHandle, pk.Upcoming), extrapolated);
    }

    // Defensive re-anchor of the wire's lane-path onto the record's OWN authoritative `LaneHandle`.
    // `Resolved.Upcoming`'s contract (see its XML doc) is "current lane first" -- but the wire's
    // `Upcoming`/`LaneWindow` (Engine.GetUpcomingLanes -> the route's per-edge lane-SEQUENCE POOL, refreshed
    // only at edge boundaries/reroutes) can lag a vehicle's actual dynamic `LaneHandle` right after a
    // same-edge tactical lane change (keep-right / speed-gain overtake): the pool's "current" entry still
    // names the OLD lane for the rest of that edge, while `LaneHandle` already flipped. PoseResolver's
    // SampleForward walks `path[0]` directly as the arc-length frame, so a stale head silently samples the
    // WRONG lane's geometry (confirmed against scenario 12-overtake: the record reports LaneHandle=1 while
    // its own Upcoming[0] still reads 0, rendering the vehicle frozen on the old lane's y forever after the
    // change). Re-anchoring here — swap in the record's own LaneHandle as index 0, leaving any downstream
    // entries (unaffected by an intra-edge lane index choice) as-is — is a no-op whenever the wire is already
    // consistent (every branch this feeds, incl. the downstream/junction path where LaneSeqIndex IS kept in
    // sync at the edge boundary), so it does not change behaviour there; it only corrects the exact stale-pool
    // case this feature newly exercises. This does not touch Sim.Core -- purely a defensive read-side fix.
    private static UpcomingLanes NormalizeUpcoming(int currentLane, UpcomingLanes raw)
    {
        if (raw[0] == currentLane)
        {
            return raw;
        }

        Span<int> buf = stackalloc int[UpcomingLanes.Count];
        var n = raw.CopyTo(buf);
        buf[0] = currentLane;
        return new UpcomingLanes(buf[..Math.Max(n, 1)]);
    }

    // HtmlPage.cs's `arcInWindow`: express a later packet's lane position in `aRec`'s FORWARD lane-window arc
    // frame. Walk `aRec.Upcoming` (current lane first, so its start is `aRec.Pos`'s origin) summing lane
    // lengths; return the cumulative distance to the START of `laneB` (caller adds `laneB`'s own pos). Null if
    // `laneB` isn't within the window (the vehicle crossed more than the K upcoming lanes between packets) ->
    // caller extrapolates rather than interpolating across an unknown gap.
    private static double? ArcInWindow(in VehicleRecord aRec, int laneB, ILaneShapeSource lanes)
    {
        var cum = 0.0;
        for (var k = 0; k < UpcomingLanes.Count; k++)
        {
            var lane = aRec.Upcoming[k];
            if (lane < 0)
            {
                break;
            }

            if (lane == laneB)
            {
                return cum;
            }

            cum += lanes.LaneLength(lane);
        }

        return null;
    }

    // Constant-acceleration arc-length extrapolation, guarded so a DECELERATING vehicle never drives
    // backward. The raw kinematic `pos + v*dt + 0.5*a*dt^2` is a downward parabola when a<0: it peaks at the
    // stop time (dt = v/-a) and then DECREASES, i.e. predicts the car reversing past where it stopped. At a
    // red light a slow/stopped vehicle publishes infrequently, so `sampleT` runs past its newest packet and
    // this extrapolation is what's rendered -- producing the visible backward jump / reversing. Clamp
    // forward-time dt to the stop time (freeze at the predicted stop), and hold an already-stopped vehicle in
    // place. Backward-in-time (dt<=0, oldest-packet underrun) is left raw -- reconstructing an earlier
    // position, where the parabola is monotonic the correct way.
    private static double ExtrapolateArc(double pos, double speed, double accel, double dt)
        => Sim.Replication.DrExtrapolation.Arc(pos, speed, accel, dt);
}
