namespace Sim.Pedestrians.Lod;

// PED-REALISM-1 / Prototype 1 (docs/PEDESTRIAN-LOWPOWER-AVOIDANCE-DESIGN.md, docs/PEDESTRIAN-PLANNING-INTENTS.md
// lever 1): the deterministic low-power lateral weave. A low-power ped's realized pose is
//   pos = centerline(s) + rightNormal(s) * LateralWeave.Offset(s, ...)
// where `Offset` is a PURE function of the ped's OWN (arc-length, seed, corridor half-width) -- no neighbour
// state -- so it stays O(1)/sample and server==IG (the headless IG recomputes the identical offset from the
// broadcast route + seed + the broadcast-once per-edge width field; no per-ped weave bytes on the wire).
//
// The offset is a KEEP-RIGHT-biased LANE PLAN, not a constant: each ~Wavelength metres the ped's lateral
// target changes to a new seeded position on its RIGHT half of the walk, with a smooth (smoothstep) drift
// between targets. This (a) separates opposing flows -- each uses its own travel-direction right-normal, so
// they sit on opposite sides -- killing the head-on centreline pass-through, and (b) spreads a same-direction
// flow into a BAND across the width (varying per-ped targets), not a single rigid lane -- the "not car lanes"
// requirement. Tapered to 0 at the route ends so spawn/arrival land on the true endpoint.
//
// Offset is returned as a SIGNED distance where positive == the ped's RIGHT side; the caller multiplies by the
// local right-normal (rotate the travel tangent -90 deg). Deterministic (a SplitMix64 hash of (seed, laneIndex),
// no System.Random).
public readonly record struct WeaveParams
{
    // Metres between lane-target changes -- the believability knob. ~30 m reads as "keeps a lane, occasionally
    // shifts", not jittery (too short) and not rigid (too long / constant).
    public double WavelengthMeters { get; init; }

    // Metres over which the ped drifts (smoothstep) from the old lane target to the new one -- a gradual lane
    // change, not a snap.
    public double TransitionMeters { get; init; }

    // Lane targets live in [MinFrac, MaxFrac] * halfWidth on the ped's right side: MinFrac keeps it off the
    // dead centre (so opposing flows always separate), MaxFrac keeps it off the very kerb edge.
    public double MinFrac { get; init; }
    public double MaxFrac { get; init; }

    // Metres over which the offset ramps 0 -> full at the route start and full -> 0 at the end, so the ped
    // spawns and arrives on the true (centreline) endpoint.
    public double EndpointTaperMeters { get; init; }

    // A gentle continuous undulation added on top of the lane plan, so a ped does not walk a dead-straight
    // lateral line between lane changes (which reads as a rigid lane). Small amplitude (m) + its own
    // wavelength (m); the per-ped phase is seeded. 0 amplitude = off.
    public double MicroAmpMeters { get; init; }
    public double MicroWavelengthMeters { get; init; }

    public static WeaveParams Default => new()
    {
        // Calm norm: a ped mostly holds its lane and walks fairly straight; lateral moves should be the
        // exception, not constant fidgeting (a "drunken ped" look). Rare, gentle lane changes (long
        // wavelength + long transition) and a barely-perceptible micro-sway.
        WavelengthMeters = 55.0,   // ~40 s between lane changes at walking pace -> mostly holds a lane
        TransitionMeters = 9.0,    // a slow, deliberate drift between lanes, not a dart
        // MinFrac = 0 so lane targets fill from the centreline outward -- keep-right is a SOFT bias (each
        // direction stays on its own half, never crosses), not a hard exclusion that leaves a dead empty
        // channel down the middle. Peds brush the centreline; opposing flows meet there but don't interpenetrate.
        MinFrac = 0.0,
        MaxFrac = 0.9,
        EndpointTaperMeters = 4.0,
        MicroAmpMeters = 0.04,     // barely-there sway (was 0.12 -> read as fidgeting in motion)
        MicroWavelengthMeters = 13.0,
    };
}

public static class LateralWeave
{
    // Distinct `k` salts for the two auxiliary seeded quantities (per-ped lane-phase and micro-wander phase),
    // so they don't collide with the lane-target hashes at small k.
    private const ulong PhaseSalt = 0xA5A5A5A5A5A5A5A5UL;
    private const ulong MicroSalt = 0x5A5A5A5A5A5A5A5AUL;
    private const ulong MeanderSalt1 = 0x1234567898765432UL;
    private const ulong MeanderSalt2 = 0x0FEDCBA987654321UL;

    // Signed lateral offset (metres, positive = the ped's RIGHT side) at arc-length `s` along a route of total
    // length `routeLength`, for a ped with `seed` on a corridor of half-width `halfWidth`. Pure + deterministic.
    public static double Offset(double s, double routeLength, ulong seed, double halfWidth, in WeaveParams p)
    {
        if (halfWidth <= 0.0 || routeLength <= 0.0)
        {
            return 0.0;
        }

        var s0 = s < 0.0 ? 0.0 : (s > routeLength ? routeLength : s);
        return OffsetInterior(s0, seed, halfWidth, p) * EndpointTaper(s0, routeLength, p.EndpointTaperMeters);
    }

    // The un-tapered lane value (metres, right side) at arc-length `s` -- the keep-right lane plan + micro-wander,
    // clamped to [0, halfWidth], BEFORE the endpoint taper. Split out so a resume leg (OffsetWithResume) can
    // blend to the interior weave without re-imposing a START taper at a demote seam that is not a true endpoint
    // (docs/PEDESTRIAN-LOWPOWER-AVOIDANCE-DESIGN.md Section 8: anchor tapers to the ACTUAL O/D, not every leg).
    private static double OffsetInterior(double s0, ulong seed, double halfWidth, in WeaveParams p)
    {
        var wl = p.WavelengthMeters > 1e-6 ? p.WavelengthMeters : 1e-6;

        // Per-ped PHASE offset on the lane segmentation so peds do NOT all change lane at the same arc-length
        // (the synchronised-wave artifact the first prototype showed). Each ped's change points are shifted by
        // a seeded fraction of a wavelength.
        var phase = Hash01(seed, PhaseSalt) * wl;
        var sp = s0 + phase;

        var k = (long)Math.Floor(sp / wl);
        var localS = sp - (k * wl);

        var targetPrev = LaneTarget(seed, k - 1, halfWidth, p);
        var targetCur = LaneTarget(seed, k, halfWidth, p);

        double lane;
        if (localS < p.TransitionMeters && p.TransitionMeters > 1e-6)
        {
            var u = SmoothStep(localS / p.TransitionMeters); // 0 -> 1 across the transition zone
            lane = targetPrev + ((targetCur - targetPrev) * u);
        }
        else
        {
            lane = targetCur;
        }

        // Gentle continuous micro-wander so the between-change segments aren't dead-straight lines.
        if (p.MicroAmpMeters > 0.0 && p.MicroWavelengthMeters > 1e-6)
        {
            var microPhase = Hash01(seed, MicroSalt) * (2.0 * Math.PI);
            lane += p.MicroAmpMeters * Math.Sin(((2.0 * Math.PI * s0) / p.MicroWavelengthMeters) + microPhase);
        }

        // Keep it on the ped's own (right) half and inside the kerb: never cross the centreline (so opposing
        // flows always separate) and never past halfWidth.
        return lane < 0.0 ? 0.0 : (lane > halfWidth ? halfWidth : lane);
    }

    // The DEMOTE-RESTORE (docs/PEDESTRIAN-LOWPOWER-AVOIDANCE-DESIGN.md Section 10.2, the load-bearing "restore"
    // step of Prototype D): after a reactive ORCA excursion (e.g. a stream-crossing) the ped is re-anchored on a
    // FRESH low-power leg. `sPrime` is arc-length along that fresh leg (0 at the demote instant). `resumeLateral`
    // (l_r) is the ped's projected lateral offset at demote -- the ONE extra scalar on the wire. The pose blends
    // (smoothstep over `leadInMeters`) from l_r to the ordinary interior weave, so it is CONTINUOUS across the
    // demote (NO POP: at sPrime==0 this returns exactly l_r) and converges to the pure lane plan. Pure +
    // deterministic => server==IG is exact again from the demote instant. The ARRIVAL endpoint still tapers to 0
    // (true O/D anchoring), but there is deliberately no START taper -- the demote is not a true endpoint.
    public static double OffsetWithResume(
        double sPrime, double routeLength, ulong seed, double halfWidth,
        double resumeLateral, double leadInMeters, in WeaveParams p)
        // Crosser case: the fresh leg is a NEW route, so the lane-plan arc and the blend distance are the same
        // coordinate (both start at 0 at the demote). Delegates to the general form.
        => OffsetWithResumeOnRoute(sPrime, sPrime, routeLength, seed, halfWidth, resumeLateral, leadInMeters, p);

    // The GENERAL demote-restore, needed for the BYSTANDER case (Prototype D2 / design §10.2-bis): a ped that was
    // NOT choosing to leave the flow but got INVOLUNTARILY deflected by someone else's ORCA maneuver, then returns
    // to ITS OWN deterministic weave -- the same seeded lane track it was already on, NOT a fresh route. The two
    // coordinates are therefore distinct:
    //   `interiorArc`  -- the ped's ABSOLUTE arc-length along its own route; the lane plan is evaluated here, so
    //                     the ped slots back onto the exact seeded track it would have been on (continuity of the
    //                     lane sequence, not a restarted one). For a bystander that kept walking, this advances as
    //                     s_r + (distance since demote); the excursion's delay is absorbed into a re-based anchor.
    //   `blendDist`    -- distance travelled SINCE the demote; the l_r->weave blend runs over this, so at the
    //                     demote instant (blendDist==0) the pose is EXACTLY l_r (no pop) regardless of interiorArc.
    // Pure + deterministic => server==IG exact again from the demote instant (the IG recomputes from the fresh-leg
    // record: route + seed + s_r + startTime + the one l_r scalar). Arrival end still tapers on the ABSOLUTE arc.
    public static double OffsetWithResumeOnRoute(
        double interiorArc, double blendDist, double routeLength, ulong seed, double halfWidth,
        double resumeLateral, double leadInMeters, in WeaveParams p)
    {
        if (routeLength <= 0.0)
        {
            return 0.0;
        }

        var arc = interiorArc < 0.0 ? 0.0 : (interiorArc > routeLength ? routeLength : interiorArc);
        var lane = halfWidth <= 0.0 ? 0.0 : OffsetInterior(arc, seed, halfWidth, p);

        double blended;
        if (leadInMeters > 1e-6 && blendDist < leadInMeters)
        {
            var bd = blendDist < 0.0 ? 0.0 : blendDist;
            var u = SmoothStep(bd / leadInMeters); // 0 at demote -> 1 after the lead-in
            blended = resumeLateral + ((lane - resumeLateral) * u);
        }
        else
        {
            blended = lane;
        }

        // Only the ARRIVAL end tapers (converge to the true endpoint); the demote seam does not.
        return blended * ArrivalTaper(arc, routeLength, p.EndpointTaperMeters);
    }

    // The SHARED, moving interface between two counterflowing streams (PED-REALISM-1: "the crowd has a moving
    // centreline -- sometimes more left, sometimes more right"). A low-frequency meander of the dividing line,
    // signed lateral (metres) in ~[-maxShift, maxShift], computed from a GLOBAL seed shared by every ped -- so
    // it is a shared DETERMINISTIC FIELD (docs/PEDESTRIAN-PLANNING-INTENTS.md Section 3): both server and IG
    // compute the identical c(s) from the same scenario-global seed, no per-ped or neighbour state. The caller
    // gives each stream its own side of c(s): the stream the interface drifts toward is squeezed, the other
    // widens -- the real emergent-lane breathing. Longer wavelength than the per-ped lane weave so it reads as
    // the whole interface drifting, not individual jitter. Tapered to 0 at the route ends.
    // `now` (seconds) makes the interface a SPATIOTEMPORAL field: the dividing line at a fixed corridor
    // position also drifts over TIME, so it isn't frozen for a segment forever. The two components travel in
    // OPPOSITE temporal directions (+w1, -w2) at slow, non-commensurate periods, giving a never-exactly-
    // repeating slow slosh. server==IG holds because the low-power pose already reconstructs at a specific
    // `now`; the IG evaluates the identical c(x, now) from the same global seed. The spatial endpoint taper
    // (a function of x only) keeps c == 0 at the corridor ends at ALL times, so peds still converge to the
    // true endpoint whenever they arrive.
    public static double CenterShift(double x, double now, double routeLength, ulong globalSeed, double maxShift, in WeaveParams p)
    {
        if (maxShift <= 0.0 || routeLength <= 0.0)
        {
            return 0.0;
        }

        const double wl1 = 55.0, wl2 = 33.0; // spatial wavelengths (m) -- longer than the per-ped lane weave
        const double tp1 = 47.0, tp2 = 71.0; // temporal periods (s) -- slow, non-commensurate slosh
        var x0 = x < 0.0 ? 0.0 : (x > routeLength ? routeLength : x);
        var w1 = (2.0 * Math.PI) / tp1;
        var w2 = (2.0 * Math.PI) / tp2;
        var ph1 = Hash01(globalSeed, MeanderSalt1) * (2.0 * Math.PI);
        var ph2 = Hash01(globalSeed, MeanderSalt2) * (2.0 * Math.PI);
        var raw = (0.6 * Math.Sin((((2.0 * Math.PI) * x0) / wl1) + (w1 * now) + ph1))
                + (0.4 * Math.Sin((((2.0 * Math.PI) * x0) / wl2) - (w2 * now) + ph2)); // in [-1, 1]
        return maxShift * raw * EndpointTaper(x0, routeLength, p.EndpointTaperMeters);
    }

    // The ped's lateral target (metres, right side) for lane segment `k`: a seeded position in
    // [MinFrac, MaxFrac] * halfWidth. hash(seed, k) makes every ped's lane sequence distinct and every segment
    // independent, so a flow fans into a band rather than a line.
    private static double LaneTarget(ulong seed, long k, double halfWidth, in WeaveParams p)
    {
        var u = Hash01(seed, unchecked((ulong)k));
        var frac = p.MinFrac + ((p.MaxFrac - p.MinFrac) * u);
        return frac * halfWidth;
    }

    // 0 at the route ends, 1 in the interior -- ramps over `taper` metres at each end (min of the two ramps).
    private static double EndpointTaper(double s, double routeLength, double taper)
    {
        if (taper <= 1e-6)
        {
            return 1.0;
        }

        var up = s / taper;
        var down = (routeLength - s) / taper;
        var t = Math.Min(up, down);
        return t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
    }

    // Only the ARRIVAL-end ramp (0 at s==routeLength, 1 by `taper` metres before it) -- used by the resume leg,
    // whose START is a demote seam (no taper) but whose END is a true arrival (must converge to the endpoint).
    private static double ArrivalTaper(double s, double routeLength, double taper)
    {
        if (taper <= 1e-6)
        {
            return 1.0;
        }

        var down = (routeLength - s) / taper;
        return down < 0.0 ? 0.0 : (down > 1.0 ? 1.0 : down);
    }

    private static double SmoothStep(double x)
    {
        var t = x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x);
        return t * t * (3.0 - (2.0 * t));
    }

    // Deterministic SplitMix64 hash of (seed, k) -> [0, 1). Same mixing family as VehicleRng; no System.Random.
    private static double Hash01(ulong seed, ulong k)
    {
        var z = unchecked(seed + (k * 0x9E3779B97F4A7C15UL) + 0x9E3779B97F4A7C15UL);
        z = unchecked((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL);
        z = unchecked((z ^ (z >> 27)) * 0x94D049BB133111EBUL);
        z ^= z >> 31;
        return (z >> 11) * (1.0 / 9007199254740992.0); // 53-bit mantissa -> [0,1)
    }
}
