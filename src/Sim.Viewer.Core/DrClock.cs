using System.Diagnostics;
using Sim.Core;
using Sim.Replication;

namespace Sim.Viewer.Core;

// docs/SUMOSHARP-NATIVE-VIEWER.md P2b / SUMOSHARP-DEADRECKONING.md §7/§8 — a direct port of
// Sim.LiveHost/HtmlPage.cs's dead-reckoning pacing: `ingestFrame`'s long-baseline wall<->sim rate fit, the
// render loop's continuous strictly-monotonic `renderSim`, and `resolvePose`'s interpolate/extrapolate
// switch on the `delay` knob. Transport-agnostic: `Pump` is fed whatever "sim-axis" timestamp the newest
// DDS vehicle sample carries (DdsSubscriber.LatestVehicleSampleTime) and supplies its OWN real wall-clock
// reading (a Stopwatch) for the other side of the fit — mirroring HtmlPage.cs's split between
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
    // rate; here the "sim" axis fed to Pump is whatever DdsSubscriber.LatestVehicleSampleTime carries
    // (usually the DDS SourceTimestamp, i.e. real wall-clock seconds), which advances 1:1 with wall time
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
    }

    public readonly struct Resolved
    {
        public Resolved(DrState state, UpcomingLanes upcoming, bool extrapolated)
        {
            State = state;
            Upcoming = upcoming;
            Extrapolated = extrapolated;
        }

        // Render-time state: Pos/PosLat are already advanced to render time (dt already applied here), so
        // a caller feeds this to PoseResolver.Resolve with dt=0 (Length/Width are NOT set — the caller
        // fills those in from the lifecycle/dims registry via `State with { Length = ..., Width = ... }`).
        public DrState State { get; }

        // The lane-path (current lane first) of whichever buffered packet the resolution used — the
        // caller's `upcomingLanes` span for PoseResolver.
        public UpcomingLanes Upcoming { get; }

        // True if this frame extrapolated (forward past the newest packet, backward past the oldest, or
        // because the two bracketing packets straddled a lane change) rather than interpolated exactly.
        public bool Extrapolated { get; }
    }

    // Call once per app frame (regardless of whether new DDS data arrived this frame). `newestSampleTime`
    // is the newest per-vehicle DDS sample timestamp seen so far (DdsSubscriber.LatestVehicleSampleTime);
    // repeated/unchanged values between DDS arrivals are ignored for the rate fit (mirrors HtmlPage.cs
    // ingestFrame's `if(m.step === lastStep) return` dedupe) but the render clock still advances every call.
    // `hold` (the sim is paused): freeze the render clock -- do NOT advance renderSim past the newest sample,
    // so a paused publisher's vehicles stop immediately instead of coasting on extrapolation for ~3s. The
    // sample-time ingest still runs (so the rate fit + restart detection stay live for when it resumes).
    public void Pump(double? newestSampleTime, bool hold = false)
    {
        var nowWall = _wall.Elapsed.TotalSeconds;

        if (newestSampleTime is { } sim && sim != _lastIngestedSim)
        {
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

    // Resolve one vehicle's render-time DrState from its buffered history (newest last, capped by
    // DdsSubscriber.HistoryCap) + the interactive playout `delay` seconds (HtmlPage.cs's resolvePose):
    // delay=0 extrapolates forward from the newest packet; delay>0 interpolates between the two buffered
    // packets bracketing (renderSim - delay), falling back to extrapolation past the newest/oldest packet
    // or across a lane change between the two bracketing packets (HtmlPage.cs's `arcInWindow` null case).
    public Resolved Resolve(IReadOnlyList<DdsSubscriber.VehicleSample> history, double delay)
    {
        if (history.Count == 0)
        {
            throw new ArgumentException("history must have at least one sample.", nameof(history));
        }

        EffectiveDelay = delay;

        var sampleT = _renderSim - delay;
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
                // The vehicle crossed onto a new lane between the two buffered packets -- the two arc-length
                // values aren't directly comparable without walking the lane window (HtmlPage.cs's
                // `arcInWindow` returns null here) -> fall back to extrapolating from `a`.
                var dt = sampleT - a.TimestampSeconds;
                pk = a.Record;
                posLat = pk.PosLat;
                arc = ExtrapolateArc(pk.Pos, pk.Speed, pk.Accel, dt);
                extrapolated = true;
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

        return new Resolved(state, pk.Upcoming, extrapolated);
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
    {
        if (dt > 0.0)
        {
            if (speed <= 0.0)
            {
                return pos; // already stopped -> stay put (no drift, no reverse)
            }

            if (accel < 0.0)
            {
                var timeToStop = speed / -accel;
                if (dt > timeToStop)
                {
                    dt = timeToStop; // freeze at the stopping point instead of reversing past it
                }
            }
        }

        return pos + speed * dt + 0.5 * accel * dt * dt;
    }
}
