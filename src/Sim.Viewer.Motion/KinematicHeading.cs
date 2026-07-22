using Sim.Core;

namespace Sim.Viewer.Motion;

// Tunables for the kinematic single-track (bicycle) reconstruction (docs/IGBRIDGE-DECISIONS.md §5.3).
// All render-side; none affect parity.
public sealed class KinematicHeadingParams
{
    public double WheelbaseFactor { get; init; } = 0.6;      // wheelbase / vehicle length
    public double HoldSpeed { get; init; } = 0.5;            // below this: hold heading (no spin at stops)
    public double ReseedJumpMeters { get; init; } = 7.0;     // front jump beyond this -> reseed (teleport)

    // Position-gain smoothing time (s) for the (lane-change-corrected) front: low-passes residual jitter.
    public double PositionSmoothTime { get; init; } = 0.60;

    // Smoothing time (s) for the lane-DIRECTION used as the front predictor. De-facets the staircase lane
    // heading so the front follows a smooth arc through a turn (no corner-overshoot into the parallel lane)
    // without the facet jitter that predicting along the raw lane heading would inject.
    public double LanePredictSmoothTime { get; init; } = 0.18;

    // Critically-damped smoothing time (s) for the output HEADING. The drawn body extends a full length
    // from the pivot, so any residual heading micro-step is amplified into a tail wiggle; a light heading
    // low-pass removes the last of it. 0 disables it (the substepped drag already yields a clean heading).
    public double HeadingSmoothTime { get; init; } = 0.0;

    // Lane-change model (docs/IGBRIDGE-DECISIONS.md §5.8). SUMO changes lane instantly — the raw front snaps
    // ~one lane width sideways in a single step. The cross-track part of such a snap is absorbed into a
    // decaying lateral error so the drawn front never jumps; it then decays with this time constant, spreading
    // the slide into a gentle yaw (peak ~25 deg/s instead of the raw ~70-130 deg/s "rotation jump"). Larger =
    // gentler but the body trails the new lane longer.
    public double LaneChangeDecayTau { get; init; } = 2.0;

    // A single-step jump of the RAW front larger than this (metres) is treated as a lane-change snap whose
    // cross-track part is absorbed. A turn advances the front only ~½ m/step however sharp, so it never trips
    // this; a lane-width (~3.2 m) snap always does. Well below a teleport (ReseedJumpMeters), which reseeds.
    public double LaneChangeSnapMeters { get; init; } = 1.5;

    // Hard cap (metres) on the absorbed lateral error, so back-to-back lane changes stay bounded and the error
    // can never approach the teleport threshold (which would make the reseed misfire). ~one lane width.
    public double LaneChangeErrorCapMeters { get; init; } = 3.4;

    // Anticipatory turn-in (docs/IGBRIDGE-DECISIONS.md §5.10). A real car cannot yaw instantly, so on a SHARP
    // corner it takes a wider line — continues a bit farther, then rounds the turn. The drawn front is modelled
    // as a point whose heading chases the smoothed front's travel direction at THIS bounded turn-rate (deg/s);
    // it advances at the vehicle speed and springs gently back to the lane so the wide line reconverges. A
    // gentle turn (needed rate below this) is unaffected. Set ≤ 0 to DISABLE the stage entirely. Default OFF:
    // the lane-heading predictor (§1) already gives a natural, in-lane wide line through corners, so the
    // explicit turn-in double-counts and pushes the front toward the parallel lane. Kept for experimentation.
    public double TurnInSmoothTime { get; init; } = 0.0;

    // Per-step spring gain pulling the wide steered line back toward the lane, so it reconverges after the
    // corner (and a stopped car eases onto the lane point without drifting).
    public double TurnInReconverge { get; init; } = 0.15;

    // Hard cap (metres) on how far the drawn FRONT may sit LATERALLY (perpendicular to travel) from the raw
    // lane front, so the front stays in its lane however sharp the corner — the rear still off-tracks inside.
    // This bounds BOTH the anticipatory wide line and the g-h tracker's natural corner-overshoot (its velocity
    // feed-forward rides wide of a tight arc); without it a sharp junction turn drifts ~1.8 m toward the
    // parallel lane. The along-track part ("go a bit farther before turning") is not capped. ~a fifth of a lane.
    public double TurnInMaxOffsetMeters { get; init; } = 0.6;
}

// One reconstructed rigid-body pose: the vehicle geometric center (what the owner's IG models pivot on),
// plus the front reference and the dragged rear axle (for a renderer that anchors elsewhere), and the
// body heading in navi-degrees.
public readonly struct KinematicPose
{
    public KinematicPose(double centerX, double centerY, double frontX, double frontY,
        double rearX, double rearY, float headingDeg)
    {
        CenterX = centerX;
        CenterY = centerY;
        FrontX = frontX;
        FrontY = frontY;
        RearX = rearX;
        RearY = rearY;
        HeadingDeg = headingDeg;
    }

    public double CenterX { get; }
    public double CenterY { get; }
    public double FrontX { get; }
    public double FrontY { get; }
    public double RearX { get; }
    public double RearY { get; }
    public float HeadingDeg { get; }
}

// Kinematic single-track ("bicycle") reconstruction (docs/IGBRIDGE-DECISIONS.md §5.3). Fixes the
// "vehicle on rails" artifact: instead of pinning the front to the lane polyline and swinging the rear
// around it (the chord-heading model), the reliable front reference TOWS a rear axle that cannot slip
// sideways -- exactly a car's unsteered rear wheels. The rear follows a smooth path and cuts inside the
// corner (real off-tracking); heading = rear->front integrates polyline facets away; on a lane change the
// towed rear lags so the body yaws into the change and back. Stateful per vehicle, integrated at the emit
// cadence; deterministic (seeded from the lane heading, no System.Random, order-independent per handle).
// Shared in Sim.Viewer.Motion so the 2D/3D viewers and the IG feed all inherit it (fix once, fix all).
public sealed class KinematicHeading
{
    private struct State
    {
        public double Ex;   // decaying LATERAL error absorbed from lane-change snaps (input = raw - E)
        public double Ey;
        public double Fx;   // smoothed front position (lane-heading-predicted, facet-jitter low-pass over raw - E)
        public double Fy;
        public double LdirX; // smoothed lane-heading unit vector (predictor direction; de-facets the lane heading)
        public double LdirY;
        public double Rx;   // rear-axle position
        public double Ry;
        public double Sx;   // anticipatory-turn-in "steered" front (wide-line pursuit of the smoothed front)
        public double Sy;
        public float Sphi;  // steered front heading (critically-damped chase of the lane heading)
        public double SphiVel; // angular SmoothDamp velocity for the steered heading
        public double PrevFaX; // previous front-axle position (for substepped drag integration)
        public double PrevFaY;
        public double PrevInX; // previous RAW front input (for single-step lane-change-snap detection)
        public double PrevInY;
        public float Deg;   // last (emitted, smoothed) body heading (navi)
        public double DegVel; // angular SmoothDamp velocity for the heading low-pass
        public bool Init;
    }

    // Unity-style critically-damped smoothing toward `target`; C1, no overshoot.
    private static double SmoothDamp(double current, double target, ref double vel, double smoothTime, double dt)
    {
        if (smoothTime <= 1e-6)
        {
            vel = 0.0;
            return target;
        }

        var omega = 2.0 / smoothTime;
        var x = omega * dt;
        var exp = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x);
        var change = current - target;
        var temp = (vel + omega * change) * dt;
        vel = (vel - omega * temp) * exp;
        return target + (change + temp) * exp;
    }

    private readonly Dictionary<VehicleHandle, State> _state = new();
    private readonly KinematicHeadingParams _p;

    public KinematicHeading(KinematicHeadingParams? p = null) => _p = p ?? new KinematicHeadingParams();

    public void Clear() => _state.Clear();

    // Advance one vehicle by one emit step. `frontX/Y` is the (smooth) lane front reference from
    // PoseResolver, `laneHeadingDeg` its lane/chord heading (used only to seed and to inset the front
    // axle), `speed` the resolved speed, `length` the vehicle length, `dt` the emit interval.
    public KinematicPose Update(
        VehicleHandle handle, double frontX, double frontY, float laneHeadingDeg,
        double speed, double length, float dt, bool lateralEvent = false,
        float? predictHeadingDeg = null)
    {
        var lwb = _p.WheelbaseFactor * length;

        _state.TryGetValue(handle, out var s);

        if (!s.Init)
        {
            s.PrevInX = frontX;
            s.PrevInY = frontY;
            s.Ex = 0.0;
            s.Ey = 0.0;
        }

        // (0) LANE-CHANGE handling via a bounded, decaying LATERAL error.
        //
        // SUMO changes lane instantly: the raw front leaps ~one lane width (~3.2 m) SIDEWAYS in a single step.
        // Trying to low-pass the whole (fast-moving) front to hide that makes it lag so far behind the real
        // motion that it trips the teleport reseed and snaps — the failure mode behind the earlier spikes.
        // Instead we absorb ONLY the cross-track (lateral) part of such a snap into a bounded error E, so the
        // DRAWN front (= raw − E) does not jump; E then decays critically, so the body slides into the new
        // lane over ~LaneChangeDecayTau and yaws gently. ALONG-track motion always passes through untouched —
        // zero lag — so a real turn (which never snaps sideways, only advances ~½ m/step) is unaffected and
        // stays crisp. A genuine teleport (raw step > ReseedJumpMeters) clears E and snaps.
        var stepX = frontX - s.PrevInX;
        var stepY = frontY - s.PrevInY;
        s.PrevInX = frontX;
        s.PrevInY = frontY;
        var step2 = stepX * stepX + stepY * stepY;

        if (step2 > _p.ReseedJumpMeters * _p.ReseedJumpMeters)
        {
            s.Ex = 0.0; // teleport: snap to the new location, no easing
            s.Ey = 0.0;
        }
        else if (s.Init && (lateralEvent || step2 > _p.LaneChangeSnapMeters * _p.LaneChangeSnapMeters))
        {
            // Decompose the snap step relative to the current body heading and absorb its CROSS-track part.
            var (hx, hy) = Dir(s.Deg);
            var along = stepX * hx + stepY * hy;
            s.Ex += stepX - along * hx;
            s.Ey += stepY - along * hy;
        }

        // Critically-damped exponential decay of the lateral error, capped so successive lane changes stay
        // bounded (and E can never reach the teleport threshold, so the reseed cannot misfire on it).
        var decay = Math.Exp(-dt / _p.LaneChangeDecayTau);
        s.Ex *= decay;
        s.Ey *= decay;
        var eMag2 = s.Ex * s.Ex + s.Ey * s.Ey;
        if (eMag2 > _p.LaneChangeErrorCapMeters * _p.LaneChangeErrorCapMeters)
        {
            var k = _p.LaneChangeErrorCapMeters / Math.Sqrt(eMag2);
            s.Ex *= k;
            s.Ey *= k;
        }

        // Input to the front tracker: the raw front with the lane-change lateral error removed. This is
        // CONTINUOUS (E absorbed the snap), so the tracker below never sees the ~3.2 m jump and cannot lag
        // into a reseed.
        var inX = frontX - s.Ex;
        var inY = frontY - s.Ey;

        // Predictor direction: the LOOK-AHEAD heading when supplied (the chord to a point a few metres ahead
        // ON THE UPCOMING LANE CENTERLINE — through a junction it already points down the connecting lane, so
        // the front anticipates the turn and tracks the connecting centerline instead of turning in late).
        // Falls back to the current lane heading (reactive) when no look-ahead is available.
        var (rawLx, rawLy) = Dir(predictHeadingDeg ?? laneHeadingDeg);
        if (!s.Init)
        {
            s.Fx = inX;
            s.Fy = inY;
            s.LdirX = rawLx;
            s.LdirY = rawLy;
        }

        // (1) Track the (continuous) front input by predicting it advancing at the resolved speed ALONG THE
        // LANE HEADING, then correcting toward the input with a critically-damped position gain. Predicting
        // along the lane — which curves through a turn — makes the front follow the ARC, so it does not ride
        // wide of a tight corner into the next lane the way a straight constant-velocity prediction does (its
        // tangent overshoot was the bulk of the "drifts toward the parallel lane"). The predictor is the lane
        // INPUT, not the fed-back body heading, so it cannot resonate into a post-turn overshoot. The lane
        // heading is a facet staircase, though, so we predict along a low-passed lane DIRECTION (EMA of the
        // unit vector) — that de-facets the predictor without lagging the arc, keeping the front smooth. At a
        // standstill speed → 0, so the front just eases onto the lane point (no drift). A teleport snaps.
        var aDir = _p.LanePredictSmoothTime > 1e-6 ? 1.0 - Math.Exp(-dt / _p.LanePredictSmoothTime) : 1.0;
        s.LdirX += aDir * (rawLx - s.LdirX);
        s.LdirY += aDir * (rawLy - s.LdirY);
        var lnrm = Math.Sqrt(s.LdirX * s.LdirX + s.LdirY * s.LdirY);
        var lhx = lnrm > 1e-9 ? s.LdirX / lnrm : rawLx;
        var lhy = lnrm > 1e-9 ? s.LdirY / lnrm : rawLy;
        var predX = s.Fx + speed * dt * lhx;
        var predY = s.Fy + speed * dt * lhy;
        if ((inX - predX) * (inX - predX) + (inY - predY) * (inY - predY)
            > _p.ReseedJumpMeters * _p.ReseedJumpMeters)
        {
            s.Fx = inX;
            s.Fy = inY;
        }
        else
        {
            var g = _p.PositionSmoothTime > 1e-6 ? 1.0 - Math.Exp(-dt / _p.PositionSmoothTime) : 1.0;
            s.Fx = predX + g * (inX - predX);
            s.Fy = predY + g * (inY - predY);
        }

        var smFrontX = s.Fx;
        var smFrontY = s.Fy;

        // (2) ANTICIPATORY TURN-IN. A real car cannot yaw instantly: on a sharp corner it continues a bit
        // farther and rounds the turn on a wider line, rather than pivoting on the lane-centerline apex. Model
        // the drawn front as a point whose HEADING chases the smoothed front's travel direction at a bounded
        // turn-rate; it advances at the vehicle speed and springs gently back to the lane so the wide line
        // reconverges after the corner. On a gentle turn the needed rate is below the limit, so it tracks
        // exactly (no change); on a sharp one the rate limit makes it overshoot the corner (go wide) then
        // curve in. This feeds the no-slip drag a lower-curvature path, so the reconstruction looks like a
        // driver taking the corner, not a body pinned to the centerline.
        double faX, faY;
        if (_p.TurnInSmoothTime <= 0.0)
        {
            // Disabled (v2 default): the front follows the smoothed lane line directly.
            faX = smFrontX;
            faY = smFrontY;
        }
        else
        {
            if (!s.Init)
            {
                s.Sx = smFrontX;
                s.Sy = smFrontY;
                s.Sphi = laneHeadingDeg;
                s.SphiVel = 0.0;
            }

            // Chase the lane heading with a CRITICALLY-DAMPED (C1) smoother rather than a hard rate limit: the
            // steered heading lags the lane heading (that lag is what makes the front overshoot the apex and
            // take a wide line), but with a continuous derivative — a hard rate-clamp's slope discontinuities
            // are what re-introduced the yaw-accel kinks. Larger TurnInSmoothTime → more lag → wider line.
            var dPhi = ((laneHeadingDeg - s.Sphi + 540f) % 360f) - 180f;
            var smPhi = SmoothDamp(0.0, dPhi, ref s.SphiVel, _p.TurnInSmoothTime, dt);
            s.Sphi = (float)(((s.Sphi + smPhi) % 360.0 + 360.0) % 360.0);

            if ((smFrontX - s.Sx) * (smFrontX - s.Sx) + (smFrontY - s.Sy) * (smFrontY - s.Sy)
                > _p.ReseedJumpMeters * _p.ReseedJumpMeters)
            {
                s.Sx = smFrontX;
                s.Sy = smFrontY;
                s.Sphi = laneHeadingDeg;
                s.SphiVel = 0.0;
            }
            else
            {
                var (spx, spy) = Dir(s.Sphi);
                s.Sx += speed * dt * spx + _p.TurnInReconverge * (smFrontX - s.Sx);
                s.Sy += speed * dt * spy + _p.TurnInReconverge * (smFrontY - s.Sy);
            }

            faX = s.Sx;
            faY = s.Sy;
        }

        if (!s.Init)
        {
            var (lx, ly) = Dir(laneHeadingDeg);
            s.Rx = faX - lwb * lx;
            s.Ry = faY - lwb * ly;
            s.Deg = laneHeadingDeg;
            s.PrevFaX = faX;
            s.PrevFaY = faY;
            s.Init = true;
        }

        // Teleport / handle reuse: front jumped implausibly far -> reseed the rear from the lane heading.
        if ((faX - s.Rx) * (faX - s.Rx) + (faY - s.Ry) * (faY - s.Ry)
            > (lwb + _p.ReseedJumpMeters) * (lwb + _p.ReseedJumpMeters))
        {
            var (lx, ly) = Dir(laneHeadingDeg);
            s.Rx = faX - lwb * lx;
            s.Ry = faY - lwb * ly;
            s.Deg = laneHeadingDeg;
            s.PrevFaX = faX;
            s.PrevFaY = faY;
        }

        float rawDeg;
        if (speed < _p.HoldSpeed)
        {
            // Near-stationary: hold heading (no spin at a red light), keep the rear rigid behind.
            rawDeg = s.Deg;
            var (hx, hy) = Dir(rawDeg);
            s.Rx = faX - lwb * hx;
            s.Ry = faY - lwb * hy;
        }
        else
        {
            // SUBSTEPPED trailer drag: integrate the rear axle along the front's motion this frame in N fine
            // steps. The rear velocity stays along the body axis (no lateral slip) BY CONSTRUCTION; a single
            // big step lets the rear cut the corner and appear to "steer"/skid, so we substep to make the
            // discrete no-slip accurate even at high yaw rate.
            const int n = 8;
            for (var k = 1; k <= n; k++)
            {
                var f = (double)k / n;
                var ffx = s.PrevFaX + (faX - s.PrevFaX) * f;
                var ffy = s.PrevFaY + (faY - s.PrevFaY) * f;
                var vvx = ffx - s.Rx;
                var vvy = ffy - s.Ry;
                var dd = Math.Sqrt(vvx * vvx + vvy * vvy);
                if (dd > 1e-9)
                {
                    s.Rx = ffx - lwb * (vvx / dd);
                    s.Ry = ffy - lwb * (vvy / dd);
                }
            }

            rawDeg = Navi(faX - s.Rx, faY - s.Ry);
        }

        s.PrevFaX = faX;
        s.PrevFaY = faY;

        // Heading low-pass (critically-damped, angular): removes any residual micro-step. Off by default.
        var delta = ((rawDeg - s.Deg + 540f) % 360f) - 180f;
        var smoothedDelta = SmoothDamp(0.0, delta, ref s.DegVel, _p.HeadingSmoothTime, dt);
        var outDeg = (float)(((s.Deg + smoothedDelta) % 360.0 + 360.0) % 360.0);
        s.Deg = outDeg;
        _state[handle] = s;

        // Vehicle CENTER placed so the front BUMPER sits exactly on the lane front reference (faX): the body
        // center is half a length behind the front bumper along the body heading. This makes the front
        // "stick" to the lane while the rear follows the drag heading.
        var (odx, ody) = Dir(outDeg);
        var cx = faX - length * 0.5 * odx;
        var cy = faY - length * 0.5 * ody;
        return new KinematicPose(cx, cy, smFrontX, smFrontY, s.Rx, s.Ry, outDeg);
    }

    // Navi-degree (0 = north, clockwise) unit vector -- identical convention to PoseResolver.
    private static (double X, double Y) Dir(float naviDeg)
    {
        var math = (90.0 - naviDeg) * Math.PI / 180.0;
        return (Math.Cos(math), Math.Sin(math));
    }

    private static float Navi(double dx, double dy)
    {
        var deg = 90.0 - Math.Atan2(dy, dx) * 180.0 / Math.PI;
        deg %= 360.0;
        if (deg < 0.0)
        {
            deg += 360.0;
        }

        return (float)deg;
    }
}
