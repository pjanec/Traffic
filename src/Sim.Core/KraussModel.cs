using Sim.Ingest;

namespace Sim.Core;

// Ported from sumo/src/microsim/cfmodels/MSCFModel.cpp, MSCFModel_KraussOrig1.cpp and
// MSCFModel_Krauss.cpp. CLAUDE.md rule 1: this is a straight port of what those files DO, not
// a re-derivation from a paper -- read them again before touching this file.
//
// Unit macros (sumo/src/utils/common/SUMOTime.h) are kept as explicit functions taking dt
// rather than hard-coded constants, so this generalizes past dt=1 (rung 1's step-length):
//   ACCEL2SPEED(a) = a*TS   SPEED2DIST(v) = v*TS   DIST2SPEED(d) = d/TS
// TS there is the *simulation* step-length in seconds; we thread dt through explicitly instead
// of relying on a global.
public static class KraussModel
{
    public static double Accel2Speed(double accel, double dt) => accel * dt;

    public static double Speed2Dist(double speed, double dt) => speed * dt;

    public static double Dist2Speed(double dist, double dt) => dist / dt;

    // MSCFModel.cpp: maxNextSpeed(speed, veh) =
    //   MIN2(speed + ACCEL2SPEED(getMaxAccel()), myType->getMaxSpeed())
    public static double MaxNextSpeed(double speed, ResolvedVType vType, double dt) =>
        Math.Min(speed + Accel2Speed(vType.Accel, dt), vType.MaxSpeed);

    // MSCFModel.cpp: minNextSpeed(speed, veh), Euler branch (gSemiImplicitEulerUpdate) --
    // phase 1 is Euler-only per CLAUDE.md/DESIGN.md (Ballistic support is a later task).
    //   return MAX2(speed - ACCEL2SPEED(myDecel), 0.);
    public static double MinNextSpeed(double speed, ResolvedVType vType, double dt) =>
        Math.Max(speed - Accel2Speed(vType.Decel, dt), 0.0);

    // MSCFModel.cpp: minNextSpeedEmergency(speed, veh), Euler branch --
    //   return MAX2(speed - ACCEL2SPEED(myEmergencyDecel), 0.);
    public static double MinNextSpeedEmergency(double speed, ResolvedVType vType, double dt) =>
        Math.Max(speed - Accel2Speed(vType.EmergencyDecel, dt), 0.0);

    // sumo/src/utils/common/StdDefs.h via config.h.cmake:211 -- "defines the epsilon to use on
    // general floating point comparison". CLAUDE.md rule 1: confirmed by reading the actual
    // vendored macro rather than trusting the briefing's guess of 1e-9 -- config.h.cmake:211
    // reads `#define NUMERICAL_EPS 0.001`. Internal (not private): Engine.cs's stop-line
    // constraint (rung 5) needs the same constant for newStopDist = endPos + NUMERICAL_EPS - pos.
    internal const double NumericalEps = 0.001;

    // sumo/src/utils/common/StdDefs.h:58 -- #define SUMO_const_haltingSpeed 0.1. Used by the
    // rung-5 stop-reached test (MSVehicle.cpp:1805's `currentVelocity <= stop.getSpeed() +
    // SUMO_const_haltingSpeed`) and by keepStopping's speed>=haltingSpeed check (not exercised
    // here since our only stop is non-waypoint). Rung 9b-ii: also the MAX2(leaderSpeed,
    // SUMO_const_haltingSpeed) divisor in MSVehicle::adaptToJunctionLeader
    // (MSVehicle.cpp:3263) -- widened to public rather than re-declared, so both call sites
    // stay the same constant.
    public const double HaltingSpeed = 0.1;

    // MSCFModel.cpp:75-96 brakeGap/brakeGapEuler (Euler branch only -- ballistic is a later
    // task per CLAUDE.md/DESIGN.md). Called from maximumSafeFollowSpeed with headwayTime=0.
    public static double BrakeGap(double speed, double decel, double headwayTime, double dt)
    {
        var speedReduction = Accel2Speed(decel, dt);
        var steps = (int)(speed / speedReduction);
        return Speed2Dist((steps * speed) - (speedReduction * steps * (steps + 1) / 2.0), dt) + (speed * headwayTime);
    }

    // MSCFModel.cpp:827-851 maximumSafeStopSpeedEuler. relaxEmergency is always false at both
    // call sites reachable from followSpeed (MSCFModel_Krauss.cpp:127 and
    // MSCFModel.cpp:952/960's onInsertion=false, relaxEmergency=false), so the emergency-relax
    // block in maximumSafeStopSpeed (lines 782-820) is never entered from here and is folded
    // into this call chain via the emergency-decel correction inlined below in
    // MaximumSafeFollowSpeed instead (source keeps it as two separate call sites: one inside
    // maximumSafeStopSpeed's relaxEmergency branch -- unreachable here since relaxEmergency is
    // always passed false by followSpeed's call chain -- and one inside
    // maximumSafeFollowSpeed itself, which IS reachable and is the one ported).
    public static double MaximumSafeStopSpeedEuler(double gap, double decel, double headway, double dt)
    {
        var g = gap - NumericalEps;
        if (g < 0.0)
        {
            return 0.0;
        }

        var b = Accel2Speed(decel, dt);
        var t = headway >= 0 ? headway : throw new ArgumentException("headway must be >= 0 (myHeadwayTime fallback not modeled)", nameof(headway));
        var s = dt; // TS

        var n = Math.Floor(0.5 - ((t + (Math.Sqrt((s * s) + (4.0 * ((s * ((2.0 * g / b) - t)) + (t * t)))) * -0.5)) / s));
        var h = (0.5 * n * (n - 1) * b * s) + (n * b * t);
        var r = (g - h) / ((n * s) + t);
        var x = (n * b) + r;
        return x;
    }

    // MSCFModel.cpp:1002-1050 calculateEmergencyDeceleration. Not exercised by rung 4 (the
    // follower never needs to brake harder than decel=4.5 to stay behind the capped-at-5m/s
    // leader), ported now so rung 5+ scenarios that DO trigger it are an additive scenario
    // change, not an algorithm change.
    public static double CalculateEmergencyDeceleration(double gap, double egoSpeed, double predSpeed, double predMaxDecel)
    {
        if (gap <= 0.0)
        {
            return double.NaN; // caller must supply myEmergencyDecel in this case -- see MaximumSafeFollowSpeed
        }

        var predBrakeDist = 0.5 * predSpeed * predSpeed / predMaxDecel;
        var b1 = 0.5 * egoSpeed * egoSpeed / (gap + predBrakeDist);
        if (b1 <= predMaxDecel)
        {
            return b1;
        }

        var b2 = 0.5 * ((egoSpeed * egoSpeed) - (predSpeed * predSpeed)) / gap;
        return b2;
    }

    // MSCFModel.cpp:774-823 maximumSafeStopSpeed, Euler branch (gSemiImplicitEulerUpdate=true
    // per phase-1 CLAUDE.md/DESIGN.md), called from maximumSafeFollowSpeed with
    // relaxEmergency=false (MSCFModel.cpp:952) -- so the relax/emergency-correction block at
    // lines 782-820 is dead here and intentionally not ported at this call site (it only runs
    // for stopSpeed's other callers, not followSpeed's).
    public static double MaximumSafeStopSpeed(double gap, double decel, double headway, double dt) =>
        MaximumSafeStopSpeedEuler(gap, decel, headway, dt);

    // MSCFModel.cpp:774-823 maximumSafeStopSpeed, relaxEmergency branch (lines 782-820) -- NOW
    // ported for real (rung 5) since MSCFModel_Krauss.cpp's stopSpeed (the stop-line caller,
    // NOT followSpeed's) always reaches it: stopSpeed's default `usage=CalcReason::CURRENT`
    // gives `relaxEmergency = usage != FUTURE` = true. Euler branch only
    // (gSemiImplicitEulerUpdate, phase 1's only integration mode).
    public static double MaximumSafeStopSpeed(
        double gap,
        double decel,
        double emergencyDecel,
        double currentSpeed,
        double headway,
        double dt,
        bool relaxEmergency)
    {
        var vsafe = MaximumSafeStopSpeedEuler(gap, decel, headway, dt);

        if (relaxEmergency && decel != emergencyDecel)
        {
            var origSafeDecel = (currentSpeed - vsafe) / dt; // SPEED2ACCEL(currentSpeed - vsafe)
            if (origSafeDecel > decel + NumericalEps)
            {
                // MSCFModel.cpp:803: calculateEmergencyDeceleration(gap, currentSpeed, /*predSpeed*/0., /*predMaxDecel*/1)
                // -- calculateEmergencyDeceleration itself special-cases gap<=0 to return
                // myEmergencyDecel (MSCFModel.cpp:1006-1008); our ported CalculateEmergencyDeceleration
                // pushes that special case to the caller (see its own doc comment), so replicate the
                // ternary here exactly as MaximumSafeFollowSpeed already does above.
                var rawEmergency = gap <= 0.0
                    ? emergencyDecel
                    : CalculateEmergencyDeceleration(gap, currentSpeed, 0.0, 1.0);
                var safeDecel = 1.2 * rawEmergency; // EMERGENCY_DECEL_AMPLIFIER
                safeDecel = Math.Max(safeDecel, decel);
                safeDecel = Math.Min(safeDecel, origSafeDecel);
                vsafe = currentSpeed - Accel2Speed(safeDecel, dt);
                vsafe = Math.Max(vsafe, 0.0); // Euler: MAX2(vsafe, 0.)
            }
        }

        return vsafe;
    }

    // MSCFModel.cpp:920-998 maximumSafeFollowSpeed -- the REAL formula MSCFModel_Krauss (our
    // vType's resolved carFollowModel) uses via followSpeed, NOT MSCFModel_KraussOrig1::vsafe
    // (removed -- see git history / CLAUDE.md's rung-4 briefing: that formula is dead code once
    // a real leader exists). onInsertion is always false here (only MSLane insertion logic
    // passes true; the plan-phase per-step call from MSVehicle::planMoveInternal, ported by
    // Engine.ComputeConstrainedSpeed, always passes onInsertion=false), and gComputeLC is always
    // false (no lane-change model in phase 1), so the `gap<0 && !gComputeLC` emergency-brake
    // branch (line 954) and the `onInsertion` guard on the correction block (line 960) are kept
    // literally but always take their "false" arm given phase-1 inputs.
    public static double MaximumSafeFollowSpeed(
        double gap,
        double egoSpeed,
        double predSpeed,
        double predMaxDecel,
        ResolvedVType vType,
        double dt,
        bool onInsertion = false)
    {
        var headway = vType.Tau; // myHeadwayTime

        double x;
        if (gap >= 0)
        {
            var effectiveGap = gap + BrakeGap(predSpeed, Math.Max(vType.Decel, predMaxDecel), 0.0, dt);
            x = MaximumSafeStopSpeed(effectiveGap, vType.Decel, headway, dt);
        }
        else
        {
            // gComputeLC is always false in phase 1 -> this branch IS reachable for gap<0.
            x = Math.Max(egoSpeed - Accel2Speed(vType.EmergencyDecel, dt), 0.0); // Euler: MAX2(x, 0.)
        }

        // Emergency-decel correction (MSCFModel.cpp:960-994): only triggers when
        // myDecel != myEmergencyDecel (true for our resolved vType: 4.5 vs 9.0) AND the safe
        // speed computed above implies braking harder than myDecel. Not exercised by rung 4
        // (see CLAUDE.md briefing) but ported literally for rung 5+ readiness.
        if (vType.Decel != vType.EmergencyDecel && !onInsertion)
        {
            var origSafeDecel = (egoSpeed - x) / dt; // SPEED2ACCEL(egoSpeed - x)
            if (origSafeDecel > vType.Decel + NumericalEps)
            {
                var rawEmergency = gap <= 0.0
                    ? vType.EmergencyDecel
                    : CalculateEmergencyDeceleration(gap, egoSpeed, predSpeed, predMaxDecel);
                var safeDecel = 1.2 * rawEmergency; // EMERGENCY_DECEL_AMPLIFIER
                safeDecel = Math.Max(safeDecel, vType.Decel);
                safeDecel = Math.Min(safeDecel, origSafeDecel);
                x = Math.Max(egoSpeed - Accel2Speed(safeDecel, dt), 0.0); // Euler: MAX2(x, 0.)
            }
        }

        return x;
    }

    // MSCFModel_Krauss.cpp:111-127 followSpeed, Euler branch only: return MIN2(vsafe, vmax);
    // where vsafe = maximumSafeFollowSpeed(...) and vmax = maxNextSpeed(speed, veh).
    // applyHeadwayAndSpeedDifferencePerceptionErrors (line 114) is a driver-state perception
    // model (MSVehicle::hasDriverState()) -- no driver state exists in phase 1
    // (CLAUDE.md/DESIGN.md), so it is unconditionally a no-op there and is correctly omitted
    // here rather than ported as a dead branch.
    public static double FollowSpeed(
        double egoSpeed,
        double gap,
        double predSpeed,
        double predMaxDecel,
        ResolvedVType vType,
        double dt)
    {
        var vsafe = MaximumSafeFollowSpeed(gap, egoSpeed, predSpeed, predMaxDecel, vType, dt);
        var vmax = MaxNextSpeed(egoSpeed, vType, dt);
        return Math.Min(vsafe, vmax);
    }

    // MSCFModel_Krauss.cpp:100-107 stopSpeed, Euler branch: usage defaults to CalcReason::CURRENT
    // at every call site the "process stops" block in planMoveInternal uses (MSVehicle.cpp:2528's
    // `cfModel.stopSpeed(this, getSpeed(), newStopDist)`), so relaxEmergency = usage != FUTURE is
    // always true here -- unlike followSpeed's maximumSafeFollowSpeed, which always passes
    // relaxEmergency=false. headway is veh->getActionStepLengthSecs() (NOT myHeadwayTime/tau --
    // this differs from followSpeed's gap-closing formula, see the briefing).
    // applyHeadwayPerceptionError (no driver state in phase 1) is a no-op, correctly omitted.
    public static double StopSpeed(
        double gap,
        double speed,
        ResolvedVType vType,
        double dt,
        double actionStepLengthSecs)
    {
        var vsafe = MaximumSafeStopSpeed(
            gap,
            vType.Decel,
            vType.EmergencyDecel,
            currentSpeed: speed,
            headway: actionStepLengthSecs,
            dt: dt,
            relaxEmergency: true);

        return Math.Min(vsafe, MaxNextSpeed(speed, vType, dt));
    }

    // MSLane.h getVehicleMaxSpeed (no-restriction branch): MIN2(veh->getMaxSpeed(), laneSpeed *
    // veh->getChosenSpeedFactor()). This is the "desired free-flow speed" constraint fed into
    // the leader/junction/stop-line reducer as the no-obstruction case.
    public static double LaneVehicleMaxSpeed(double laneSpeed, ResolvedVType vType) =>
        Math.Min(laneSpeed * vType.SpeedFactor, vType.MaxSpeed);

    // MSCFModel.cpp: finalizeSpeed(veh, vPos). vPos is the MIN over the (already-reduced)
    // leader/junction/stop-line constraint collection computed by the caller; laneVehicleMaxSpeed
    // is the lane's speed limit adaptation for this vehicle (MSLane::getVehicleMaxSpeed).
    //
    // vStop (MSCFModel.cpp:191: `MIN2(vPos, veh->processNextStop(vPos))`) is now a real caller-
    // supplied input (rung 5: Engine.ProcessNextStop ports processNextStop's reached/duration
    // state machine); the caller has already taken that MIN2 before calling in, so this method
    // only has to consume the result -- when there is no stop it is simply vPos again (a no-op
    // MIN, exactly like phase 1-4's dead +infinity slot).
    public static double FinalizeSpeed(
        double oldV,
        double vPos,
        double vStop,
        double laneVehicleMaxSpeed,
        ResolvedVType vType,
        double dt,
        double actionStepLengthSecs)
    {
        var vMinEmergency = MinNextSpeedEmergency(oldV, vType, dt);
        // vMin = MIN2(minNextSpeed(oldV), MAX2(vPos, vMinEmergency))
        var vMin = Math.Min(MinNextSpeed(oldV, vType, dt), Math.Max(vPos, vMinEmergency));

        // getFriction()==1 in phase 1 (no weather/friction model) -> factor == 1.
        const double factor = 1.0;

        var aMax = ((Math.Max(laneVehicleMaxSpeed, vPos) * factor) - oldV) / actionStepLengthSecs;
        var vMax = Min3(oldV + Accel2Speed(aMax, dt), MaxNextSpeed(oldV, vType, dt), vStop);
        vMax = Math.Max(vMin, vMax);

        // sigma=0 in rung 1 => patchSpeedBeforeLC (dawdle) is a no-op; no lane-change model and
        // startupDelay defaults to 0 => applyStartupDelay is a no-op too. So vNext = vMax.
        var vNext = vMax;
        return vNext;
    }

    private static double Min3(double a, double b, double c) => Math.Min(a, Math.Min(b, c));
}
