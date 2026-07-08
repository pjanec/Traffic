using Sim.Ingest;

namespace Sim.Core;

// Ported from sumo/src/microsim/cfmodels/MSCFModel_IDM.cpp (whole file, plain-IDM ctor arm only
// -- myIDMM=false) plus the shared base sumo/src/microsim/cfmodels/MSCFModel.cpp::finalizeSpeed
// (the accel/decel-bound clamp IDM's own finalizeSpeed override delegates to unmodified --
// MSCFModel_IDM.cpp:67-74 -- already ported once for Krauss as KraussModel.FinalizeSpeed's
// vMin/vMax computation; reused here via the shared KraussModel helpers rather than duplicated).
//
// Plain IDM only (myIDMM=false, C11-i scope -- TASKS.md defers ACC/CACC/IDMM):
//   delta=4.0 (ctor :40, no <param> override in scope), adaptationFactor=1.0 (ctor :41's idmm=false
//   arm), adaptationTime=0.0 (:42, unused since adaptationFactor==1). Because adaptationFactor is
//   hardwired to 1.0 here, every `if (myAdaptationFactor != 1.)` branch in the vendored source
//   (_v's headwayTime-scaling block at MSCFModel_IDM.cpp:204-207, finalizeSpeed's
//   levelOfService update at :69-72, createVehicleVariables' VehicleVariables allocation at
//   MSCFModel_IDM.h:180-185) is PROVABLY DEAD and is omitted below, not "ported as a no-op" --
//   there is no VehicleVariables/levelOfService state anywhere in this port.
//
// Units: same dt-threading convention as KraussModel.cs (ACCEL2SPEED(a)=a*dt, SPEED2DIST(v)=v*dt,
// DIST2SPEED(d)=d/dt) -- TS in the vendored source is the *simulation* step-length in seconds,
// threaded explicitly here as `dt` rather than a global. NUMERICAL_EPS is KraussModel.NumericalEps
// (0.001, sumo/src/utils/common/StdDefs.h via config.h.cmake:211) -- the SAME global epsilon
// KraussModel.cs already cites, not a distinct IDM-only constant.
public static class IdmModel
{
    // MSCFModel_IDM.cpp:40 myDelta -- plain-IDM ctor arg is
    // vtype->getCFParam(SUMO_ATTR_CF_IDM_DELTA, 4.0); no scenario in this rung's scope overrides
    // this via a <param> child, so it is the literal constant rather than a parsed vType field.
    private const double Delta = 4.0;

    // MSCFModel_IDM.cpp:43 myIterations = MAX2(1, int(TS / stepping + .5)) where stepping
    // defaults to getCFParam(SUMO_ATTR_CF_IDM_STEPPING, 0.25) -- no scenario in scope overrides
    // it either. For dt=1s (every phase-1 scenario's step-length) this is
    // MAX2(1, int(1/0.25 + .5)) = MAX2(1, int(4.5)) = 4, matching the briefing.
    private const double Stepping = 0.25;

    public static int Iterations(double dt) => Math.Max(1, (int)((dt / Stepping) + 0.5));

    // MSCFModel_IDM.cpp:44 myTwoSqrtAccelDecel = 2*sqrt(myAccel*myDecel). For the resolved
    // passenger defaults (accel=2.6, decel=4.5): 2*sqrt(2.6*4.5) = 6.8410..., matching
    // provenance.txt's cited value.
    public static double TwoSqrtAccelDecel(ResolvedVType vType) => 2.0 * Math.Sqrt(vType.Accel * vType.Decel);

    // MSCFModel_IDM.cpp:52-62 minNextSpeed OVERRIDE (MSCFModel_IDM.h:168) -- IDM permits
    // exceeding myDecel when approaching stops, using
    // `decel = MAX2(myDecel, MIN2(myEmergencyDecel, 1.5))` instead of the base class's plain
    // myDecel (KraussModel.MinNextSpeed). This is virtual-dispatched in the vendored source from
    // TWO call sites: MSCFModel::finalizeSpeed's own `minNextSpeed(oldV, veh)` (used below in
    // IdmModel.FinalizeSpeed, NOT KraussModel.MinNextSpeed) and MSVehicle.cpp:2191's
    // `vMinComfortable = cfModel.minNextSpeed(getSpeed())` (Engine.cs's StopLineConstraint, via
    // its own CarFollowModel dispatch). Euler branch only (phase 1's only integration mode, per
    // CLAUDE.md/DESIGN.md) -- the ballistic arm (no MAX2 floor at 0) is not reachable here.
    public static double MinNextSpeed(double speed, ResolvedVType vType, double dt)
    {
        var decel = Math.Max(vType.Decel, Math.Min(vType.EmergencyDecel, 1.5));
        return Math.Max(speed - KraussModel.Accel2Speed(decel, dt), 0.0);
    }

    // MSCFModel_IDM.cpp:190-194 getSecureGap (IDM's own override of the base MSCFModel one).
    // headwayTime is plain vType.Tau here -- the myAdaptationFactor!=1 headwayTime-scaling block
    // inside _v (see class header) never applies to this plain-IDM port, and getSecureGap itself
    // never referenced headwayTime scaling in the vendored source anyway (only _v's local did).
    public static double GetSecureGap(double speed, double leaderSpeed, ResolvedVType vType)
    {
        var deltaV = speed - leaderSpeed;
        return Math.Max(0.0, (speed * vType.Tau) + (speed * deltaV / TwoSqrtAccelDecel(vType)));
    }

    // MSCFModel_IDM.cpp:198-242 _v, the iterated core shared by every entry point below.
    // `respectMinGap` mirrors the header's own default (`= true`) at each call site that omits
    // it explicitly (followSpeed does; freeSpeed/stopSpeed pass `false` explicitly) -- callers
    // here always pass it explicitly for clarity.
    //
    // C11-iv: `headwayTimeOverride` ports the `myAdaptationFactor != 1.` headwayTime-adaptation
    // block at MSCFModel_IDM.cpp:203-207 (`headwayTime = myHeadwayTime; if (myAdaptationFactor !=
    // 1.) headwayTime *= myAdaptationFactor + levelOfService*(1.-myAdaptationFactor);`) WITHOUT
    // touching this function's plain-IDM body: every existing (IDM/ACC/CACC) caller passes null,
    // so `headwayTimeOverride ?? vType.Tau` resolves to the exact literal `vType.Tau` this line
    // always used -- byte-identical, no plain-IDM call-site or codepath change. Only Engine.cs's
    // IDMM dispatch arms ever pass a non-null override (the caller-precomputed adapted headway,
    // `vType.Tau * (AdaptationFactor - LevelOfService*(AdaptationFactor-1))`, i.e.
    // `myAdaptationFactor + levelOfService*(1-myAdaptationFactor)` applied to `vType.Tau` up
    // front -- see IdmmModel.AdaptedHeadwayTime).
    private static double V(
        double gap2pred,
        double egoSpeed,
        double predSpeed,
        double desSpeed,
        bool respectMinGap,
        ResolvedVType vType,
        double dt,
        double? headwayTimeOverride = null)
    {
        var headwayTime = headwayTimeOverride ?? vType.Tau;
        var newSpeed = egoSpeed;
        var gap = gap2pred;
        if (respectMinGap)
        {
            // gap2pred comes with minGap already subtracted so we need to add it here again.
            gap += vType.MinGap;
        }

        var iterations = Iterations(dt);
        var twoSqrtAccelDecel = TwoSqrtAccelDecel(vType);

        for (var i = 0; i < iterations; i++)
        {
            var deltaV = newSpeed - predSpeed;
            var s = Math.Max(0.0, (newSpeed * headwayTime) + (newSpeed * deltaV / twoSqrtAccelDecel));
            if (respectMinGap)
            {
                s += vType.MinGap;
            }

            gap = Math.Max(KraussModel.NumericalEps, gap); // avoid singularity
            var acc = vType.Accel * (1.0 - Math.Pow(newSpeed / Math.Max(KraussModel.NumericalEps, desSpeed), Delta) - ((s * s) / (gap * gap)));
            newSpeed = Math.Max(0.0, newSpeed + (KraussModel.Accel2Speed(acc, dt) / iterations));
            // TODO (upstream, MSCFModel_IDM.cpp:238): use a more realistic position update which
            // takes accelerated motion into account -- ported literally, not "improved".
            gap -= Math.Max(0.0, KraussModel.Speed2Dist(newSpeed - predSpeed, dt) / iterations);
        }

        return Math.Max(0.0, newSpeed);
    }

    // MSCFModel_IDM.cpp:77-100 freeSpeed. `maxSpeed < 0` ballistic-only guard (:79-82) is kept
    // literally even though phase 1 is Euler-only (CLAUDE.md/DESIGN.md) -- it can never actually
    // trigger here, exactly like KraussModel's own dead-branch citations elsewhere in this repo.
    // `desSpeed` is `veh->getLane()->getVehicleMaxSpeed(veh)` at the vendored call sites (:87/:92)
    // -- our engine's simplified single-constraint free-flow term (Engine.cs's
    // FreeFlowDesiredSpeedConstraint) has no separate "next lane" vs. "current lane" distinction,
    // so both `maxSpeed` and `desSpeed` are the SAME caller-supplied `laneVehicleMaxSpeed` value
    // -- see that call site's own comment for why this collapses correctly for a normal
    // (no-upcoming-speed-limit-drop) free-flow lane.
    // C11-iv: `headwayTimeOverride` threads straight through to both `V` calls below, unread by
    // `GetSecureGap` above -- MSCFModel_IDM::getSecureGap (:190-193) is IDM's own override, and it
    // ALWAYS uses the plain (unadapted) `myHeadwayTime` member, never `levelOfService` -- ported
    // faithfully as "no adaptation here", not an oversight (see IdmModel.GetSecureGap's own header
    // comment, unchanged by this rung).
    public static double FreeSpeed(double speed, double seen, double maxSpeed, double desSpeed, ResolvedVType vType, double dt, double? headwayTimeOverride = null)
    {
        if (maxSpeed < 0.0)
        {
            return maxSpeed;
        }

        var secGap = GetSecureGap(maxSpeed, 0.0, vType);
        double vSafe;
        if (speed <= maxSpeed)
        {
            // accelerate -- the free-accel branch the briefing's scenario 22 anchor exercises.
            vSafe = V(1e6, speed, maxSpeed, desSpeed, respectMinGap: false, vType, dt, headwayTimeOverride);
        }
        else
        {
            // decelerate; relax gap to avoid emergency braking (leader speed set to 0, as upstream).
            vSafe = V(Math.Max(seen, secGap), speed, 0.0, desSpeed, respectMinGap: false, vType, dt, headwayTimeOverride);
        }

        if (seen < secGap)
        {
            // avoid overshoot when close to a change in speed limit.
            vSafe = Math.Min(vSafe, maxSpeed);
        }

        return vSafe;
    }

    // MSCFModel_IDM.cpp:104-107 followSpeed (applyHeadwayAndSpeedDifferencePerceptionErrors is a
    // driver-state perception model -- no driver state exists in phase 1, exactly like
    // KraussModel.FollowSpeed's own citation, so it is correctly omitted, not ported as a no-op).
    // `desSpeed` is `veh->getLane()->getVehicleMaxSpeed(veh)` -- the caller's laneVehicleMaxSpeed.
    // C11-iv: `headwayTimeOverride` threads through to `V` -- null (every IDM/ACC/CACC call site)
    // resolves to `vType.Tau` inside `V`, byte-identical to the pre-C11-iv call.
    public static double FollowSpeed(double egoSpeed, double gap2pred, double predSpeed, double desSpeed, ResolvedVType vType, double dt, double? headwayTimeOverride = null) =>
        V(gap2pred, egoSpeed, predSpeed, desSpeed, respectMinGap: true, vType, dt, headwayTimeOverride);

    // MSCFModel_IDM.cpp:151-173 stopSpeed (applyHeadwayPerceptionError is likewise a no-op driver-
    // state hook in phase 1). `desSpeed` is again the caller's laneVehicleMaxSpeed.
    // C11-iv: `headwayTimeOverride` threads through to `V` the same way as FollowSpeed/FreeSpeed.
    public static double StopSpeed(double speed, double gap, double desSpeed, ResolvedVType vType, double dt, double actionStepLengthSecs, double? headwayTimeOverride = null)
    {
        if (gap < 0.01)
        {
            return 0.0;
        }

        var result = V(gap, speed, 0.0, desSpeed, respectMinGap: false, vType, dt, headwayTimeOverride);

        if (gap > 0 && speed < KraussModel.NumericalEps && result < KraussModel.NumericalEps)
        {
            // ensure that stops can be reached: fall back to the Krauss maximumSafeStopSpeed
            // (decel=vType.Decel, the 3-arg stopSpeed's own default; relaxEmergency=true, the
            // parameter default at this call's arity -- MSCFModel.h:612) -- the SAME formula
            // KraussModel.StopSpeed itself calls, reused verbatim rather than re-derived.
            result = KraussModel.MaximumSafeStopSpeed(
                gap,
                vType.Decel,
                vType.EmergencyDecel,
                currentSpeed: speed,
                headway: actionStepLengthSecs,
                dt: dt,
                relaxEmergency: true);
        }

        if (gap >= 0)
        {
            // avoid overshooting the stop location.
            result = Math.Min(result, KraussModel.Dist2Speed(gap, dt));
        }

        return result;
    }

    // MSCFModel_IDM.cpp:67-74 finalizeSpeed = MSCFModel::finalizeSpeed(veh, vPos) (the base
    // vMin/vMax accel/decel-bound clamp) with NO dawdle: MSCFModel::patchSpeedBeforeLC's base
    // (non-overridden) default is `return vMax` verbatim (MSCFModel.h:102-105) -- MSCFModel_IDM
    // never overrides patchSpeedBeforeLC, so vNext == vMax exactly, with no RNG draw at all (IDM
    // has no sigma/dawdle concept in this port's scope, unlike KraussModel.FinalizeSpeed's
    // vType.Sigma>0 branch). applyStartupDelay (MSCFModel.cpp:232) defaults to a no-op (0 delay)
    // and the lane-change model's patchSpeed is a no-op in phase 1 -- both already established as
    // unaffected by KraussModel.FinalizeSpeed's own citations, equally true here.
    // The myAdaptationFactor!=1 levelOfService update (MSCFModel_IDM.cpp:69-72) is unreachable
    // for plain IDM (see class header) and is correctly omitted.
    //
    // C11-iv: this function's body stays EXACTLY this -- the base `vNext = MSCFModel::
    // finalizeSpeed(veh, vPos)` computation MSCFModel_IDM.cpp:68 delegates to before its own
    // `if (myAdaptationFactor != 1.)` levelOfService-update block (:69-72). Rather than growing an
    // `ref levelOfService`/`adaptationTime` parameter here (which would touch every IDM/ACC/CACC
    // call site's signature), the IDMM levelOfService update is applied by the CALLER
    // (Engine.ComputeMoveIntent's IDMM dispatch arm, right after this call returns `vNext`) --
    // mirroring the vendored source's own sequencing (base finalizeSpeed first, THEN the memory
    // update) without touching this shared body at all. See IdmmModel.UpdateLevelOfService.
    public static double FinalizeSpeed(
        double oldV,
        double vPos,
        double vStop,
        double laneVehicleMaxSpeed,
        ResolvedVType vType,
        double dt,
        double actionStepLengthSecs)
    {
        var vMinEmergency = KraussModel.MinNextSpeedEmergency(oldV, vType, dt);
        var vMin = Math.Min(MinNextSpeed(oldV, vType, dt), Math.Max(vPos, vMinEmergency));

        // getFriction()==1 in phase 1 (no weather/friction model) -> factor == 1, exactly like
        // KraussModel.FinalizeSpeed.
        const double factor = 1.0;

        var aMax = ((Math.Max(laneVehicleMaxSpeed, vPos) * factor) - oldV) / actionStepLengthSecs;
        var vMax = Math.Min(oldV + KraussModel.Accel2Speed(aMax, dt), Math.Min(KraussModel.MaxNextSpeed(oldV, vType, dt), vStop));
        vMax = Math.Max(vMin, vMax);

        return vMax;
    }
}
