namespace Sim.IgBridge;

// Consumer-side configuration of the simulated IG (docs/IGBRIDGE-REQUIREMENTS.md R7, DECISIONS §1 Q2/Q3).
public sealed class FakeIgConfig
{
    // Playout delay: the IG plays at tPlay = clock - delay so it interpolates between two buffered samples
    // instead of extrapolating a turn (Q2: the real IG's 0.5-1.0 s config knob).
    public double DelaySeconds { get; init; } = 0.75;

    // Above this straight-line gap between the two bracketing samples, the IG does an IMMEDIATE JUMP to the
    // newer sample instead of interpolating (Q3: "too long a spatial jump" -> jump, don't smear a teleport).
    public double JumpThresholdMeters { get; init; } = 8.0;

    // Cadence at which the reconstructed (displayed) pose is sampled for rendering/metrics.
    public double RenderHz { get; init; } = 60.0;

    // Low-latency DEAD-RECKONING mode (the alternative to the buffered interpolation above). When true the
    // IG applies NO playout delay and NEVER peeks at a future sample: at render time it takes the newest
    // update it has RECEIVED (t <= now) and extrapolates forward to now from the DR state -- constant speed
    // + constant yaw rate (a coordinated turn), both estimated from the last two received samples. This is
    // zero-latency but pays for it with extrapolation error: between updates the pose leads the truth and
    // then corrects ("snaps") when the next update lands. The error grows with the update interval, so it is
    // negligible at a high emit rate and visible at a low one.
    public bool Extrapolate { get; init; }
}

// One reconstructed (IG-displayed) pose.
public readonly struct ReconPose
{
    public ReconPose(double t, double x, double y, double z, float headingDeg, bool jumped)
    {
        T = t;
        X = x;
        Y = y;
        Z = z;
        HeadingDeg = headingDeg;
        Jumped = jumped;
    }

    public double T { get; }
    public double X { get; }
    public double Y { get; }
    public double Z { get; }
    public float HeadingDeg { get; }
    public bool Jumped { get; }
}

// The FakeIg (docs/IGBRIDGE-REQUIREMENTS.md R7): replays an emitted IG trace exactly as the real IG would --
// per entity it buffers the update samples and, at the delayed play cursor, reconstructs the pose from the
// two samples bracketing that cursor (linear position, SHORTEST-ARC heading -- never a raw angle lerp), or
// extrapolates from the last two when the cursor runs past the newest (exercised only when the delay is set
// too small). A bracketing pair whose endpoints are farther apart than JumpThresholdMeters is treated as a
// teleport: an immediate jump to the newer sample, not a smeared slide. No real IG required.
public sealed class FakeIg
{
    private sealed class Track
    {
        public IgEntityModel Model;
        public readonly List<ReconPose> Updates = new(); // (t,x,y,z,h); Jumped unused on inputs
    }

    private readonly Dictionary<string, Track> _tracks = new();
    private readonly FakeIgConfig _cfg;

    public FakeIg(IEnumerable<IgSample> trace, FakeIgConfig cfg)
    {
        _cfg = cfg;
        foreach (var s in trace)
        {
            switch (s.Kind)
            {
                case IgRecordKind.New:
                    var track = GetOrAdd(s.Id);
                    track.Model = s.Model;
                    track.Updates.Add(new ReconPose(s.T, s.X, s.Y, s.Z, s.HeadingDeg, false));
                    break;
                case IgRecordKind.Upd:
                    GetOrAdd(s.Id).Updates.Add(new ReconPose(s.T, s.X, s.Y, s.Z, s.HeadingDeg, false));
                    break;
                case IgRecordKind.Del:
                    break; // life end -- the update list already bounds the entity's displayed span
            }
        }
    }

    public IReadOnlyCollection<string> Ids => _tracks.Keys;

    public IgEntityModel ModelOf(string id) => _tracks[id].Model;

    // The IG-displayed pose of one entity at play time `t`, or false if `t` is outside the entity's
    // displayable span (before its first / after its last sample) -- used to bin poses into render frames.
    public bool TryDisplayPose(string id, double t, out ReconPose pose)
    {
        pose = default;
        if (!_tracks.TryGetValue(id, out var track) || track.Updates.Count < 2)
        {
            return false;
        }

        var upd = track.Updates;
        if (t < upd[0].T || t > upd[upd.Count - 1].T)
        {
            return false;
        }

        pose = SampleAt(upd, t);
        return true;
    }

    // Reconstruct every entity's DISPLAYED pose timeline at RenderHz over its update span -- the poses the IG
    // would actually show. This is what the metrics pass (T1.5) measures for smoothness. The playout delay
    // is a uniform time shift (it does not change the reconstructed SHAPE while the cursor stays bracketed),
    // so the smoothness metrics are computed over the entity's update span directly; the delay/extrapolation
    // interaction is exercised separately by the sweep (T3.1).
    public IReadOnlyDictionary<string, IReadOnlyList<ReconPose>> Reconstruct()
    {
        var result = new Dictionary<string, IReadOnlyList<ReconPose>>(_tracks.Count);
        var dt = 1.0 / _cfg.RenderHz;

        foreach (var kv in _tracks)
        {
            var upd = kv.Value.Updates;
            if (upd.Count < 2)
            {
                result[kv.Key] = Array.Empty<ReconPose>();
                continue;
            }

            var t0 = upd[0].T;
            var t1 = upd[upd.Count - 1].T;
            var poses = new List<ReconPose>((int)((t1 - t0) * _cfg.RenderHz) + 2);
            for (var t = t0; t <= t1 + 1e-9; t += dt)
            {
                poses.Add(SampleAt(upd, Math.Min(t, t1)));
            }

            result[kv.Key] = poses;
        }

        return result;
    }

    // The IG's 2-sample rule at play time `tPlay`: interpolate the bracketing pair, or (past the newest)
    // extrapolate from the last two; a pair farther apart than the jump threshold snaps to the newer sample.
    private ReconPose SampleAt(List<ReconPose> upd, double tPlay)
    {
        // locate the last update with t <= tPlay
        var j = 0;
        for (var i = upd.Count - 1; i >= 0; i--)
        {
            if (upd[i].T <= tPlay)
            {
                j = i;
                break;
            }
        }

        if (_cfg.Extrapolate)
        {
            return ExtrapolateAt(upd, j, tPlay);
        }

        var a = upd[Math.Max(0, j)];
        var b = upd[Math.Min(j + 1, upd.Count - 1)];
        if (a.T >= b.T)
        {
            return new ReconPose(tPlay, b.X, b.Y, b.Z, b.HeadingDeg, false);
        }

        var dx = b.X - a.X;
        var dy = b.Y - a.Y;
        var dz = b.Z - a.Z;
        if (dx * dx + dy * dy + dz * dz > _cfg.JumpThresholdMeters * _cfg.JumpThresholdMeters)
        {
            // Teleport: immediate jump to the newer sample once the cursor reaches/passes it.
            var snapped = tPlay >= b.T;
            var src = snapped ? b : a;
            return new ReconPose(tPlay, src.X, src.Y, src.Z, src.HeadingDeg, true);
        }

        var f = (tPlay - a.T) / (b.T - a.T); // may be >1 (extrapolate) when the delay is too small
        return new ReconPose(
            tPlay,
            a.X + dx * f,
            a.Y + dy * f,
            a.Z + dz * f,
            LerpHeadingDeg(a.HeadingDeg, b.HeadingDeg, f),
            false);
    }

    // Low-latency dead-reckoning: extrapolate from the newest RECEIVED sample (index j, t <= tPlay) forward
    // to tPlay using a constant-speed constant-yaw-rate (coordinated-turn) model, with speed and yaw rate
    // estimated from the previous received sample. Never touches upd[j+1] (the "future"), so this is what a
    // zero-delay IG would actually show at tPlay. Reduces to straight constant-velocity when the yaw rate ~0.
    private ReconPose ExtrapolateAt(List<ReconPose> upd, int j, double tPlay)
    {
        var cur = upd[j];
        var tau = tPlay - cur.T;
        if (tau <= 1e-9 || j == 0)
        {
            // At/behind the first received sample, or exactly on a sample: show it as received (no DR yet).
            return new ReconPose(tPlay, cur.X, cur.Y, cur.Z, cur.HeadingDeg, false);
        }

        var prev = upd[j - 1];
        var dt = cur.T - prev.T;
        if (dt <= 1e-9)
        {
            return new ReconPose(tPlay, cur.X, cur.Y, cur.Z, cur.HeadingDeg, false);
        }

        // A teleport in the last received step is not a DR-able velocity -> hold the newest sample.
        var jdx = cur.X - prev.X;
        var jdy = cur.Y - prev.Y;
        if (jdx * jdx + jdy * jdy > _cfg.JumpThresholdMeters * _cfg.JumpThresholdMeters)
        {
            return new ReconPose(tPlay, cur.X, cur.Y, cur.Z, cur.HeadingDeg, true);
        }

        // Estimated DR state at the newest sample: planar speed and yaw rate from the last received step.
        var speed = Math.Sqrt(jdx * jdx + jdy * jdy) / dt;                        // m/s
        var omegaDeg = (((cur.HeadingDeg - prev.HeadingDeg + 540f) % 360f) - 180f) / dt; // deg/s (navi, CW+)
        var h0 = cur.HeadingDeg * Math.PI / 180.0;                               // navi radians
        var hT = (cur.HeadingDeg + omegaDeg * tau) * Math.PI / 180.0;
        var omega = omegaDeg * Math.PI / 180.0;                                  // rad/s

        double x, y;
        if (Math.Abs(omega) < 1e-4)
        {
            // Straight: constant velocity along the current heading (navi dir = (sin, cos)).
            x = cur.X + speed * tau * Math.Sin(h0);
            y = cur.Y + speed * tau * Math.Cos(h0);
        }
        else
        {
            // Coordinated turn: integrate speed along a heading turning at omega.
            // dir(θ) = (sinθ, cosθ) => ∫ = ((cosθ0 − cosθτ)/ω, (sinθτ − sinθ0)/ω).
            x = cur.X + speed * (Math.Cos(h0) - Math.Cos(hT)) / omega;
            y = cur.Y + speed * (Math.Sin(hT) - Math.Sin(h0)) / omega;
        }

        var headingT = cur.HeadingDeg + (float)(omegaDeg * tau);
        headingT %= 360f;
        if (headingT < 0f)
        {
            headingT += 360f;
        }

        return new ReconPose(tPlay, x, y, cur.Z, headingT, false);
    }

    private Track GetOrAdd(string id)
    {
        if (!_tracks.TryGetValue(id, out var track))
        {
            track = new Track();
            _tracks[id] = track;
        }

        return track;
    }

    // Shortest-arc heading interpolation (navi-degrees): 350->10 rotates +20, not -340.
    private static float LerpHeadingDeg(float a, float b, double f)
    {
        var d = ((b - a + 540f) % 360f) - 180f;
        var h = a + (float)(d * f);
        h %= 360f;
        if (h < 0f)
        {
            h += 360f;
        }

        return h;
    }
}
