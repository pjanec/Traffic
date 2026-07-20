using Sim.Pedestrians.Lod;
using Xunit;

namespace Sim.Pedestrians.Tests.Lod;

// PED-REALISM-1 Prototype 1 (docs/PEDESTRIAN-LOWPOWER-AVOIDANCE-DESIGN.md): pins the deterministic low-power
// lateral weave -- pure, seeded, clamped, keep-right, endpoint-tapered. server==IG rests on this being a pure
// function of (s, seed, halfWidth), so determinism is the load-bearing property.
public class LateralWeaveTests
{
    private const double RouteLen = 60.0;
    private const double HalfWidth = 2.0;
    private static readonly WeaveParams P = WeaveParams.Default;

    [Fact]
    public void Deterministic_SameArgs_SameOffset()
    {
        for (var s = 0.0; s <= RouteLen; s += 1.0)
        {
            var a = LateralWeave.Offset(s, RouteLen, seed: 42, HalfWidth, P);
            var b = LateralWeave.Offset(s, RouteLen, seed: 42, HalfWidth, P);
            Assert.Equal(a, b, precision: 15);
        }
    }

    [Fact]
    public void EndpointsTaperToZero()
    {
        Assert.Equal(0.0, LateralWeave.Offset(0.0, RouteLen, seed: 7, HalfWidth, P), precision: 12);
        Assert.Equal(0.0, LateralWeave.Offset(RouteLen, RouteLen, seed: 7, HalfWidth, P), precision: 12);
    }

    [Fact]
    public void Offset_NeverCrossesCentreline_NorPastKerb()
    {
        // The load-bearing separation invariant: the offset is ALWAYS on the ped's own (right) half --
        // >= 0 so it never crosses the centreline into the opposing flow -- and never past the kerb
        // (<= halfWidth). With MinFrac = 0 peds may reach the centreline (populating it), but never cross it.
        for (var s = 0.0; s <= RouteLen; s += 0.1)
        {
            var off = LateralWeave.Offset(s, RouteLen, seed: 123, HalfWidth, P);
            Assert.True(off >= 0.0 && off <= HalfWidth + 1e-9, $"offset {off} not in [0, {HalfWidth}] at s={s}");
        }
    }

    [Fact]
    public void ZeroWidth_NoOffset()
    {
        Assert.Equal(0.0, LateralWeave.Offset(30.0, RouteLen, seed: 1, halfWidth: 0.0, P), precision: 15);
    }

    [Fact]
    public void ChangesLaneAlongRoute_NotConstant()
    {
        // The weave must VARY along arc-length (the "not a rigid car lane" property): sampling the interior
        // yields more than one distinct lateral value for a single ped.
        var seen = new System.Collections.Generic.HashSet<double>();
        for (var s = 5.0; s <= 55.0; s += 0.5)
        {
            seen.Add(System.Math.Round(LateralWeave.Offset(s, RouteLen, seed: 99, HalfWidth, P), 3));
        }

        Assert.True(seen.Count > 5, $"expected the offset to vary along the route (lane changes); distinct values = {seen.Count}");
    }

    [Fact]
    public void CenterShift_Deterministic_Bounded_TapersToZero()
    {
        // The shared moving-interface field: deterministic (server==IG rests on it), bounded by maxShift, and
        // 0 at the corridor ends so peds still converge to the true endpoint.
        const double maxShift = 0.7;
        // Ends pinned to 0 at every time (peds still converge to the true endpoint whenever they arrive).
        Assert.Equal(0.0, LateralWeave.CenterShift(0.0, now: 0.0, RouteLen, 777, maxShift, P), precision: 12);
        Assert.Equal(0.0, LateralWeave.CenterShift(RouteLen, now: 33.0, RouteLen, 777, maxShift, P), precision: 12);

        var distinct = new System.Collections.Generic.HashSet<double>();
        for (var x = 0.0; x <= RouteLen; x += 0.5)
        {
            var a = LateralWeave.CenterShift(x, now: 12.0, RouteLen, 777, maxShift, P);
            var b = LateralWeave.CenterShift(x, now: 12.0, RouteLen, 777, maxShift, P);
            Assert.Equal(a, b, precision: 15);                       // deterministic at a fixed time
            Assert.True(System.Math.Abs(a) <= maxShift + 1e-9, $"interface {a} exceeds maxShift at x={x}");
            distinct.Add(System.Math.Round(a, 3));
        }

        Assert.True(distinct.Count > 5, "interface should meander along the corridor");
    }

    [Fact]
    public void CenterShift_FluctuatesInTime()
    {
        // At a FIXED corridor position the interface must change over time (not frozen for the segment forever).
        const double maxShift = 0.7;
        const double x = 25.0;
        var t0 = LateralWeave.CenterShift(x, now: 0.0, RouteLen, 777, maxShift, P);
        var moved = false;
        for (var t = 1.0; t <= 60.0; t += 1.0)
        {
            if (System.Math.Abs(LateralWeave.CenterShift(x, t, RouteLen, 777, maxShift, P) - t0) > 0.05)
            {
                moved = true;
                break;
            }
        }

        Assert.True(moved, "the interface at a fixed position should drift over time");
    }

    [Fact]
    public void OffsetWithResume_NoPop_AtDemoteSeam()
    {
        // Prototype D restore (docs/PEDESTRIAN-LOWPOWER-AVOIDANCE-DESIGN.md 10.2): at sPrime==0 the resume leg
        // must return EXACTLY the resume-lateral l_r -- the pose is continuous across the demote (no pop),
        // whatever l_r the ORCA excursion projected to.
        foreach (var lr in new[] { 0.0, 0.3, 1.1, 1.9, -0.5 })
        {
            var at0 = LateralWeave.OffsetWithResume(0.0, RouteLen, seed: 55, HalfWidth, resumeLateral: lr, leadInMeters: 8.0, P);
            Assert.Equal(lr, at0, precision: 12);
        }
    }

    [Fact]
    public void OffsetWithResume_ConvergesToPureWeave_AfterLeadIn()
    {
        // After the lead-in the resume leg must equal the ordinary interior weave (the l_r memory is gone) --
        // so the ped is back on its deterministic lane plan, server==IG exact. Compared against the plain Offset
        // in the interior (away from either endpoint taper, where Offset == its own interior value).
        const double leadIn = 8.0;
        for (var sp = leadIn + 2.0; sp <= RouteLen - 10.0; sp += 0.5)
        {
            var resume = LateralWeave.OffsetWithResume(sp, RouteLen, seed: 55, HalfWidth, resumeLateral: 1.7, leadInMeters: leadIn, P);
            var pure = LateralWeave.Offset(sp, RouteLen, seed: 55, HalfWidth, P);
            Assert.Equal(pure, resume, precision: 12);
        }
    }

    [Fact]
    public void OffsetWithResume_Deterministic_AndArrivalTapersToZero()
    {
        // Pure/deterministic (server==IG rests on it) and the ARRIVAL end still converges to the true endpoint,
        // even though the demote START does not taper.
        var a = LateralWeave.OffsetWithResume(12.0, RouteLen, seed: 9, HalfWidth, 1.2, 8.0, P);
        var b = LateralWeave.OffsetWithResume(12.0, RouteLen, seed: 9, HalfWidth, 1.2, 8.0, P);
        Assert.Equal(a, b, precision: 15);
        Assert.Equal(0.0, LateralWeave.OffsetWithResume(RouteLen, RouteLen, seed: 9, HalfWidth, 1.2, 8.0, P), precision: 12);
    }

    [Fact]
    public void OffsetWithResumeOnRoute_BystanderReturnsToItsOwnAbsoluteTrack()
    {
        // The BYSTANDER restore (Prototype D2, §10.2-bis): a ped deflected mid-route must re-join the SAME seeded
        // lane track it was on -- so once the blend completes, the resume pose must equal the ORIGINAL pure weave
        // evaluated at the ABSOLUTE arc (not a restarted arc). Demote at absolute arc s_r; blend over leadIn.
        const double sr = 22.0, leadIn = 6.0, lr = 1.4;
        for (var bd = leadIn + 1.0; bd <= 20.0; bd += 0.5)
        {
            var absArc = sr + bd;
            if (absArc > RouteLen - 10.0) break; // stay in the interior (away from the arrival taper)
            var resume = LateralWeave.OffsetWithResumeOnRoute(absArc, bd, RouteLen, seed: 77, HalfWidth, lr, leadIn, P);
            var ownTrack = LateralWeave.Offset(absArc, RouteLen, seed: 77, HalfWidth, P);
            Assert.Equal(ownTrack, resume, precision: 12);
        }
    }

    [Fact]
    public void OffsetWithResumeOnRoute_NoPop_AtSeam_RegardlessOfAbsoluteArc()
    {
        // At the demote instant (blendDist==0) the pose is exactly l_r whatever the absolute arc -- continuity of
        // position no matter where along its route the ped was deflected.
        foreach (var sr in new[] { 5.0, 22.0, 40.0 })
        {
            var at0 = LateralWeave.OffsetWithResumeOnRoute(sr, 0.0, RouteLen, seed: 77, HalfWidth, resumeLateral: -0.8, leadInMeters: 6.0, P);
            Assert.Equal(-0.8, at0, precision: 12);
        }
    }

    [Fact]
    public void DifferentSeeds_DifferentLaneSequences()
    {
        // Two peds fan into a band: their lane sequences differ, so a same-direction flow is not a single line.
        var diff = false;
        for (var s = 5.0; s <= 55.0; s += 1.0)
        {
            if (System.Math.Abs(LateralWeave.Offset(s, RouteLen, 1, HalfWidth, P)
                              - LateralWeave.Offset(s, RouteLen, 2, HalfWidth, P)) > 0.05)
            {
                diff = true;
                break;
            }
        }

        Assert.True(diff, "different seeds should produce visibly different lateral tracks");
    }
}
