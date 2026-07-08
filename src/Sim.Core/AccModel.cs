using Sim.Ingest;

namespace Sim.Core;

// Ported from sumo/src/microsim/cfmodels/MSCFModel_ACC.cpp (whole file) + MSCFModel_ACC.h (the
// ACCVehicleVariables state). CLAUDE.md rule 1: ported from what the vendored file DOES -- the
// [1]/[2]/[3] paper citations in that file's own header are background only, the literal C++
// control law (accelSpeedControl/accelGapControl/_v's mode machine) below is what this file
// matches, not a re-derivation from those papers (CLAUDE.md rule 1's "misplaced gap term"
// warning is specifically about trusting a paper over the vendored source).
//
// Gains/thresholds (MSCFModel_ACC.cpp:57-71, ctor :78-90 -- no <param> override is exercised by
// scenario 23's vTypes, so these are the literal DEFAULT_* macros, not parsed per-vType fields):
// SC_GAIN=-0.4, GCC_GAIN_SPEED=0.8, GCC_GAIN_SPACE=0.04, GC_GAIN_SPEED=0.07, GC_GAIN_SPACE=0.23,
// CA_GAIN_SPACE=0.8, CA_GAIN_SPEED=0.23, EMERGENCY_THRESHOLD=2.0, GAP_THRESHOLD_SPEEDCTRL=120,
// GAP_THRESHOLD_GAPCTRL=100. headwayTime = myHeadwayTime, MSCFModel's OWN base-class field
// (never overridden by MSCFModel_ACC's ctor) -- i.e. the same `vType.Tau` KraussModel/IdmModel
// already read (ResolvedVType.Tau, default 1.0 -- see VTypeDefaults.cs).
//
// STATE (the class header comment on MSCFModel_ACC.h:140-146's private ACCVehicleVariables):
// ACC_ControlMode (0=speed control,1=gap control) and lastUpdateTime, both per-EGO-vehicle,
// both initialized to 0 (MSCFModel_ACC.h:130-135's createVehicleVariables). Ported here as two
// VehicleRuntime fields (AccControlMode/AccLastUpdateTime, both default 0), threaded `ref` into
// V/FollowSpeed below so a call only ever mutates the CALLING ego's own state -- never another
// vehicle's -- exactly like C1's dawdle draw already mutates `v.RngState` in place from the plan
// phase (see Engine.UseParallelPlan's header comment, the established per-entity-write
// precedent this rung reuses rather than reinvents). Byte-identical for every non-ACC vType:
// these two fields are never read or written unless CarFollowModel=="ACC" (Engine.cs's dispatch
// only calls into this class on that check).
public static class AccModel
{
    private const double ScGain = -0.4;
    private const double GccGainSpeed = 0.8;
    private const double GccGainSpace = 0.04;
    private const double GcGainSpeed = 0.07;
    private const double GcGainSpace = 0.23;
    private const double CaGainSpace = 0.8;
    private const double CaGainSpeed = 0.23;
    private const double EmergencyThreshold = 2.0;
    private const double GapThresholdSpeedCtrl = 120.0;
    private const double GapThresholdGapCtrl = 100.0;

    // MSCFModel_ACC.cpp:165-167 accelSpeedControl -- the speed-control law.
    private static double AccelSpeedControl(double vErr) => ScGain * vErr;

    // MSCFModel_ACC.cpp:171-208 accelGapControl -- the gap-control/collision-avoidance/gap-
    // closing law, selected by the |spacingErr|/|vErr| thresholds below.
    private static double AccelGapControl(double gap2pred, double speed, double predSpeed, double vErr, double headwayTime)
    {
        var deltaVel = predSpeed - speed;

        // :177 -- dynamic gap margin (Xiao et al. 2018, eq. 5, reformulated as MAX2/MIN2 to
        // avoid a discontinuity). Ported literally, including the 75./speed division: this
        // branch is only reachable when gap2pred<100 (the GAP_THRESHOLD_GAPCTRL caller guard in
        // V below) or from the hysteresis-band "previous mode==gap" arm, and on scenario 23's
        // golden path the follower's speed is always > 0 by the time gap2pred first drops under
        // 100 (it has been accelerating under speed control since t=0) -- so speed==0 here never
        // actually occurs; not special-cased beyond what the vendored source itself does.
        var d0 = Math.Max(0.0, Math.Min((75.0 / speed) - 5.0, 2.0));
        // :179 -- equation 4: gap2pred is the (already minGap-netto) distance between vehicle
        // positions.
        var spacingErr = gap2pred - (headwayTime * speed) - d0;

        if (Math.Abs(spacingErr) < 0.2 && Math.Abs(vErr) < 0.1)
        {
            // :182-189 gap mode.
            return (GcGainSpeed * deltaVel) + (GcGainSpace * spacingErr);
        }

        if (spacingErr < 0.0)
        {
            // :190-197 collision avoidance mode.
            return (CaGainSpeed * deltaVel) + (CaGainSpace * spacingErr);
        }

        // :198-205 gap closing mode.
        return (GccGainSpeed * deltaVel) + (GccGainSpace * spacingErr);
    }

    // MSCFModel_ACC.cpp:211-282 _v -- the stateful control-mode machine.
    //
    // `time` is this engine's own per-step Plan-phase timestamp (Engine.cs's
    // `time = _config.Begin + step * dt`, the exact analog of
    // `MSNet::getInstance()->getCurrentTimeStep()` at the vendored call site, :232) -- compared
    // against `lastUpdateTime` (the ego's own VehicleRuntime.AccLastUpdateTime, threaded `ref`)
    // to reproduce the "(re-)arm the mode write at most once per timestep" guard at :232-235
    // EXACTLY, including its own quirk: both `lastUpdateTime` and `time` are 0 at a vehicle's
    // very first evaluation (scenario 23's vehicles both depart at t=0, matching
    // ACCVehicleVariables' own lastUpdateTime=0 default), so `setControlMode` is FALSE on that
    // very first call -- a mode write that would otherwise occur is skipped, exactly as the
    // vendored source itself does (observably inert here since AccControlMode's own default, 0,
    // already equals what speed control would have written).
    // C11-iii: widened from `private` to `internal` so CaccModel's speedGapControl port
    // (MSCFModel_CACC.cpp:271-273 `acc_CFM._v(veh, gap2pred, speed, predSpeed, desSpeed, true)`)
    // can call the SAME raw control-mode machine ACC's own followSpeed uses -- CACC's ACC
    // fallback calls MSCFModel_ACC::_v directly, NOT MSCFModel_ACC::followSpeed (so it does
    // NOT go through ACC's own vSafe/EmergencyThreshold check -- that safety check is CACC's
    // own followSpeed's job, applied once, after the whole _v dispatch resolves, exactly like
    // it already is for CACC's own non-fallback branches). Not a distinct formula -- literally
    // the same private mode machine, just called from a second entry point.
    internal static double V(
        double gap2pred,
        double speed,
        double predSpeed,
        double desSpeed,
        double headwayTime,
        double dt,
        double time,
        ref int mode,
        ref double lastUpdateTime)
    {
        double accelAcc;

        // :229 velocity error.
        var vErr = speed - desSpeed;

        // :230-235 update guard.
        var setControlMode = false;
        if (lastUpdateTime != time)
        {
            lastUpdateTime = time;
            setControlMode = true;
        }

        if (gap2pred > GapThresholdSpeedCtrl)
        {
            // :236-248
            accelAcc = AccelSpeedControl(vErr);
            if (setControlMode)
            {
                mode = 0;
            }
        }
        else if (gap2pred < GapThresholdGapCtrl)
        {
            // :249-255
            accelAcc = AccelGapControl(gap2pred, speed, predSpeed, vErr, headwayTime);
            if (setControlMode)
            {
                mode = 1;
            }
        }
        else if (mode == 0)
        {
            // :256-271 hysteresis band, previous mode==speed.
            accelAcc = AccelSpeedControl(vErr);
        }
        else
        {
            // :256-271 hysteresis band, previous mode==gap.
            accelAcc = AccelGapControl(gap2pred, speed, predSpeed, vErr, headwayTime);
        }

        // :273 ACCEL2SPEED(accelACC) = accelACC * dt.
        var newSpeed = speed + KraussModel.Accel2Speed(accelAcc, dt);
        // :281
        return Math.Max(0.0, newSpeed);
    }

    // MSCFModel_ACC.cpp:96-106 followSpeed. `desSpeed` = MIN2(lane speed limit, vType.maxSpeed)
    // is the caller-supplied `laneVehicleMaxSpeed` (Engine.cs's FollowSpeedFor convention, same
    // as KraussModel.FollowSpeed/IdmModel.FollowSpeed's own `laneVehicleMaxSpeed` argument).
    // `vSafe` reuses KraussModel.MaximumSafeFollowSpeed verbatim -- the SAME
    // MSCFModel::maximumSafeFollowSpeed the vendored ACC's followSpeed calls (:99), not a
    // separate ACC-only safety formula.
    public static double FollowSpeed(
        double egoSpeed,
        double gap2pred,
        double predSpeed,
        double predMaxDecel,
        double laneVehicleMaxSpeed,
        ResolvedVType vType,
        double dt,
        double time,
        ref int accControlMode,
        ref double accLastUpdateTime)
    {
        var desSpeed = laneVehicleMaxSpeed;
        var vAcc = V(gap2pred, egoSpeed, predSpeed, desSpeed, vType.Tau, dt, time, ref accControlMode, ref accLastUpdateTime);
        var vSafe = KraussModel.MaximumSafeFollowSpeed(gap2pred, egoSpeed, predSpeed, predMaxDecel, vType, dt);

        if (vSafe + EmergencyThreshold < vAcc)
        {
            // :100-104 emergency override.
            return vSafe + EmergencyThreshold;
        }

        return vAcc;
    }

    // MSCFModel_ACC.cpp:110-115 stopSpeed:
    //   MIN2(maximumSafeStopSpeed(gap, decel, speed, /*onInsertion*/false,
    //        veh->getActionStepLengthSecs() /* relaxEmergency defaults to true, MSCFModel.h:612
    //        -- omitted at this call site, so the base class's own default applies */),
    //        maxNextSpeed(speed, veh))
    // `decel` at every reachable call site is the caller's (possibly-overridden) `myDecel`
    // (MSCFModel.h:168's 3-arg `stopSpeed` inline wrapper passes `myDecel` as the 4-arg
    // virtual's `decel`, and every call site this engine reaches routes through that wrapper) --
    // i.e. `vType.Decel`. This is EXACTLY the same formula MSCFModel_Krauss::stopSpeed uses
    // (verified against sumo/src/microsim/cfmodels/MSCFModel_Krauss.cpp:100-107 -- ACC does not
    // override stopSpeed with anything different from Krauss's own), so this is a thin,
    // byte-identical pass-through to the SAME KraussModel.StopSpeed call Engine.cs's
    // StopSpeedFor already makes for a Krauss-resolved vType, kept as its own named entry point
    // purely so this class stays a self-contained, citable port of the whole vendored file
    // rather than a silent gap in it.
    public static double StopSpeed(double speed, double gap, ResolvedVType vType, double dt, double actionStepLengthSecs) =>
        KraussModel.StopSpeed(gap, speed, vType, dt, actionStepLengthSecs);
}
