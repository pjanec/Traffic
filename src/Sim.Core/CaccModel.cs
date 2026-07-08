using Sim.Ingest;

namespace Sim.Core;

// Ported from sumo/src/microsim/cfmodels/MSCFModel_CACC.cpp (whole file) + MSCFModel_CACC.h (the
// CACCVehicleVariables state, which literally INHERITS MSCFModel_ACC::ACCVehicleVariables --
// see the STATE note below, this is not a re-derivation from the [1]/[2] paper citations in the
// vendored file's own header, CLAUDE.md rule 1). CACC is the last of the ACC/CACC set (C11-iii):
// STATEFUL (its own CACC_ControlMode hysteresis) and COOPERATIVE (its cooperative gap-control law
// uses the EGO's OWN last-step acceleration, veh->getAcceleration(), and falls back to the ACC
// control law whenever the leader is NOT itself CACC).
//
// Constants (MSCFModel_CACC.cpp:57-68, ctor :88-100 -- no <param> override is exercised by
// scenario 24's vTypes, so these are the literal DEFAULT_* macros):
// SC_GAIN=-0.4, GCC_GAIN_GAP=0.005, GCC_GAIN_GAP_DOT=0.05, GC_GAIN_GAP=0.45,
// GC_GAIN_GAP_DOT=0.0125, CA_GAIN_GAP=0.45, CA_GAIN_GAP_DOT=0.05, HEADWAYTIME_ACC=1.0,
// SC_MIN_GAP=1.66, EMERGENCY_THRESHOLD=2.0. CACC's OWN headwayTime (tau, used by its outer _v and
// by the cooperative gap-control law) is `myHeadwayTime`, MSCFModel's base-class field -- NEVER
// overridden by MSCFModel_CACC's ctor (unlike myHeadwayTimeACC, forwarded into the embedded
// `acc_CFM` for the ACC-fallback call) -- i.e. the same vType.Tau (ResolvedVType.Tau, default 1.0)
// AccModel/IdmModel/KraussModel already read. CommunicationsOverrideMode default is
// CACC_NO_OVERRIDE (:193's createVehicleVariables default, unchanged by scenario 24's vTypes) --
// only that path is ported; CACC_MODE_NO_LEADER/CACC_MODE_LEADER_NO_CAV/CACC_MODE_LEADER_CAV
// (:414-449) are the other CACC_CommunicationsOverrideMode branches, unreachable here and
// deferred (no vType/param in this engine's ingest sets CACC_CommunicationsOverrideMode away
// from its ctor default).
//
// STATE (MSCFModel_CACC.h:222-228's private CACCVehicleVariables): `class CACCVehicleVariables :
// public MSCFModel_ACC::ACCVehicleVariables` -- i.e. it INHERITS ACC_ControlMode/lastUpdateTime
// from the ACC variables struct rather than declaring its own copies, and adds exactly ONE new
// field, CACC_ControlMode (the CACC-specific hysteresis mode: 0=speed control,1=gap control).
// Confirmed at both use sites: MSCFModel_CACC.cpp:353-357's own per-step guard reads/writes
// `vars->lastUpdateTime` (the INHERITED field, not a CACC-private one), and MSCFModel_ACC.cpp:231-
// 235's guard -- reached via speedGapControl's `acc_CFM._v(veh, ...)` ACC-fallback call, :273,
// passing the SAME `veh` -- reads/writes THAT SAME `vars->lastUpdateTime` through an
// `ACCVehicleVariables*` cast of the identical underlying object (MSCFModel_CACC.h:189-196's
// createVehicleVariables allocates exactly one CACCVehicleVariables per CACC vehicle, initializing
// BOTH ACC_ControlMode=0 and lastUpdateTime=0 alongside CACC_ControlMode=0). So for a CACC-typed
// vehicle, "ACC_ControlMode"/"lastUpdateTime" are literally the SAME memory the ACC-fallback path
// reads/writes -- NOT a separate, parallel copy. This engine ports that literally: a CACC-typed
// vehicle reuses `VehicleRuntime.AccControlMode`/`AccLastUpdateTime` (the C11-ii fields) for its
// embedded ACC-fallback state -- deliberately NOT a fresh `CaccLastUpdateTime` field, since a
// vehicle's own CarFollowModel is fixed at exactly one string ("ACC" xor "CACC"), so there is no
// collision: an ACC-typed vehicle's dispatch never reaches CaccModel, and a CACC-typed vehicle's
// dispatch never reaches AccModel.FollowSpeed directly -- only CaccModel's fallback call into
// AccModel.V (both paths only ever touch that one vehicle's OWN two fields). This also reproduces
// a real behavioral coupling the vendored source has and a naive "separate CaccLastUpdateTime"
// port would silently break: CACC's own outer _v (MSCFModel_CACC.cpp:354-357) stamps
// `vars->lastUpdateTime = currentTimeStep` (and sets `setControlMode=true`) BEFORE dispatching to
// speedGapControl's ACC-fallback branch within the SAME call -- so by the time
// MSCFModel_ACC::_v's OWN guard (MSCFModel_ACC.cpp:232-235) runs, lastUpdateTime is already
// stamped to the current step, making the ACC-fallback's `setControlMode` FALSE that step (its
// ACC_ControlMode field is read for the hysteresis band's previous-mode branch, but never itself
// rewritten by a call reached this way) -- exactly reproduced here by threading CaccModel's
// fallback call through the very SAME `ref accLastUpdateTime` the outer CACC guard already
// stamped, rather than a fresh independent field.
// A brand-new CACC_ControlMode field IS needed (no ACC analog for it): ported as
// `VehicleRuntime.CaccControlMode` (int, default 0, matching createVehicleVariables' own
// CACC_ControlMode=0 default), read/written ONLY by the owning vehicle from inside
// CaccModel.FollowSpeed -- the SAME per-entity-write pattern C11-ii's AccControlMode and C1's
// RngState already establish (see Engine.UseParallelPlan's header comment).
// `vehMode` (MSCFModel_CACC.h:211-217's VehicleMode enum -- CC_MODE/ACC_MODE/CACC_GAP_MODE/
// CACC_GAP_CLOSING_MODE/CACC_COLLISION_AVOIDANCE_MODE) is NOT ported: it is written only into a
// debug/introspection SUMOVehicleParameter ("caccVehicleMode", MSCFModel_CACC.cpp:451-453) that
// this engine has no analog of and which never feeds back into the control law itself -- omitted
// as a pure debug side channel, not a behavioral gap.
//
// EGO-ACCELERATION (the cooperative dependency, MSCFModel_CACC.cpp:287 `veh->getAcceleration()`):
// the ego's own acceleration from the LAST COMPLETED step, (speed-prevSpeed)/dt. Ported as
// `VehicleRuntime.Acceleration` (double, default 0 -- matching `getAcceleration()`'s own value
// before any step has run), written in Engine.ExecuteMoves right next to the pre-existing
// `oldSpeed` capture (`v.Acceleration = (v.Intent.NewSpeed - oldSpeed) / dt`), i.e. written in the
// EXECUTE phase and read in the FOLLOWING step's PLAN phase -- consistent with the frozen-
// start-of-step-snapshot invariant (CLAUDE.md rule 2): a follower never sees its own
// this-step-in-progress acceleration, only its last COMPLETED step's, and never a leader's/foe's
// acceleration at all (each vehicle's cooperative term only ever reads its OWN `Acceleration`).
public static class CaccModel
{
    private const double ScGain = -0.4;
    private const double GccGainGap = 0.005;
    private const double GccGainGapDot = 0.05;
    private const double GcGainGap = 0.45;
    private const double GcGainGapDot = 0.0125;
    private const double CaGainGap = 0.45;
    private const double CaGainGapDot = 0.05;
    private const double HeadwayTimeAcc = 1.0;
    private const double ScMinGap = 1.66;
    private const double EmergencyThreshold = 2.0;

    // MSCFModel_CACC.cpp:254-261 speedSpeedControl: speed + ACCEL2SPEED(SC_GAIN * vErr).
    private static double SpeedSpeedControl(double speed, double vErr, double dt) =>
        speed + KraussModel.Accel2Speed(ScGain * vErr, dt);

    // MSCFModel_CACC.cpp:264-334 speedGapControl.
    private static double SpeedGapControl(
        double gap2pred,
        double speed,
        double predSpeed,
        double desSpeed,
        double vErr,
        double tau,
        double egoAcceleration,
        bool hasPred,
        bool predIsCacc,
        double dt,
        double time,
        ref int accControlMode,
        ref double accLastUpdateTime)
    {
        // :270 `if (pred != nullptr)`.
        if (!hasPred)
        {
            // :323-330 "no leader" arm.
            return SpeedSpeedControl(speed, vErr, dt);
        }

        if (!predIsCacc)
        {
            // :271-278 leader is NOT CACC -> ACC fallback: MSCFModel_ACC::_v called DIRECTLY
            // (not MSCFModel_ACC::followSpeed -- no vSafe/EmergencyThreshold check here, that is
            // CACC's OWN followSpeed's job, applied once after this whole dispatch resolves).
            // headwayTime = myHeadwayTimeACC = HEADWAYTIME_ACC (the CACC ctor forwards this
            // constant into the embedded acc_CFM via setHeadwayTime -- MSCFModel_CACC.cpp:102 --
            // NOT this vehicle's own tau).
            return AccModel.V(gap2pred, speed, predSpeed, desSpeed, HeadwayTimeAcc, dt, time, ref accControlMode, ref accLastUpdateTime);
        }

        // :279-321 leader IS CACC -> cooperative control law.
        var spacingErr = gap2pred - (tau * speed);
        var speedErr = predSpeed - speed + (tau * egoAcceleration);

        if (spacingErr > 0 && spacingErr < 0.2 && vErr < 0.1)
        {
            // :290-300 gap mode.
            return speed + (GcGainGap * spacingErr) + (GcGainGapDot * speedErr);
        }

        if (spacingErr < 0)
        {
            // :301-310 collision avoidance mode.
            return speed + (CaGainGap * spacingErr) + (CaGainGapDot * speedErr);
        }

        // :311-320 gap closing mode.
        return speed + (GccGainGap * spacingErr) + (GccGainGapDot * speedErr);
    }

    // MSCFModel_CACC.cpp:336-473 _v -- CACC_NO_OVERRIDE path only (see this class's own header
    // comment for why the other CommunicationsOverrideMode branches are unreachable/deferred).
    private static double V(
        double gap2pred,
        double speed,
        double predSpeed,
        double desSpeed,
        double tau,
        double egoAcceleration,
        bool hasPred,
        bool predIsCacc,
        double dt,
        double time,
        ref int caccControlMode,
        ref int accControlMode,
        ref double accLastUpdateTime)
    {
        // :350-357 velocity error + the shared per-step guard (see this class's header comment:
        // this REUSES the ego's own AccLastUpdateTime field -- CACCVehicleVariables' inherited
        // ACCVehicleVariables::lastUpdateTime -- not a separate CaccLastUpdateTime).
        var vErr = speed - desSpeed;
        var setControlMode = false;
        if (accLastUpdateTime != time)
        {
            accLastUpdateTime = time;
            setControlMode = true;
        }

        // :361-364 CACC_NO_OVERRIDE: time-gap-based mode selection.
        var timeGap = gap2pred / Math.Max(KraussModel.NumericalEps, speed);
        var spacingErr = gap2pred - (tau * speed);

        double newSpeed;
        if (timeGap > 2 && spacingErr > ScMinGap)
        {
            // :367-378 speed control.
            newSpeed = SpeedSpeedControl(speed, vErr, dt);
            if (setControlMode)
            {
                caccControlMode = 0;
            }
        }
        else if (timeGap < 1.5)
        {
            // :379-391 gap control.
            newSpeed = SpeedGapControl(
                gap2pred, speed, predSpeed, desSpeed, vErr, tau, egoAcceleration, hasPred, predIsCacc,
                dt, time, ref accControlMode, ref accLastUpdateTime);
            if (setControlMode)
            {
                caccControlMode = 1;
            }
        }
        else if (caccControlMode == 0)
        {
            // :392-413 hysteresis band, previous mode == speed control.
            newSpeed = SpeedSpeedControl(speed, vErr, dt);
        }
        else
        {
            // :392-413 hysteresis band, previous mode == gap control.
            newSpeed = SpeedGapControl(
                gap2pred, speed, predSpeed, desSpeed, vErr, tau, egoAcceleration, hasPred, predIsCacc,
                dt, time, ref accControlMode, ref accLastUpdateTime);
        }

        // :458-464 the DELTA_T<100ms step-length rescale. dt (this engine's step length in
        // SECONDS) < 0.1 is the exact translation of `DELTA_T < 100` (DELTA_T is in ms,
        // sumo/src/utils/common/SUMOTime.h:39/42) -- always false at phase-1's dt=1s, so this is
        // ported literally (readiness for a future sub-100ms scenario) rather than assumed dead
        // and dropped.
        var newSpeedScaled = newSpeed;
        if (dt < 0.1)
        {
            var accel01 = (newSpeed - speed) * 10.0;
            newSpeedScaled = speed + KraussModel.Accel2Speed(accel01, dt);
        }

        // :472 -- CLAUDE.md rule 1: the vendored source DOES clamp `_v`'s own return value to
        // >=0 here (`return MAX2(0., newSpeedScaled);`), unlike a naive reading of ACC's own _v
        // (which returns unclamped `newSpeed`, its MAX2(0,.) is instead the LAST line of AccModel
        // .V above) might suggest applies to CACC too. Ported exactly as the file reads, not as
        // "no MAX2 here" -- followSpeed's own vSafe/EmergencyThreshold check still runs
        // afterward regardless.
        return Math.Max(0.0, newSpeedScaled);
    }

    // MSCFModel_CACC.cpp:117-146 followSpeed. `desSpeed` = the caller-supplied
    // `laneVehicleMaxSpeed` (veh->getLane()->getVehicleMaxSpeed(veh), :122 -- the FOLLOWER's own
    // desired speed on its current lane, independent of which leader/foe/obstacle this call is
    // against; same convention AccModel.FollowSpeed/IdmModel.FollowSpeed already use).
    // `hasPred`/`predIsCacc` -- see SpeedGapControl's own header comment: every call site this
    // engine reaches (LeaderFollowSpeedConstraint/ObstacleConstraint/AdaptToJunctionLeader) always
    // has SOME candidate (leader/obstacle/foe) by the time it calls in, so `hasPred` is `true` at
    // every one of them; `predIsCacc` is `false` for an ExternalObstacle (no CarFollowModel to
    // begin with) and `leader/foe.VType.CarFollowModel == "CACC"` for a real vehicle leader/foe.
    public static double FollowSpeed(
        double egoSpeed,
        double gap2pred,
        double predSpeed,
        double predMaxDecel,
        double laneVehicleMaxSpeed,
        ResolvedVType vType,
        double dt,
        double time,
        double egoAcceleration,
        bool hasPred,
        bool predIsCacc,
        ref int caccControlMode,
        ref int accControlMode,
        ref double accLastUpdateTime)
    {
        var desSpeed = laneVehicleMaxSpeed;
        var vCacc = V(
            gap2pred, egoSpeed, predSpeed, desSpeed, vType.Tau, egoAcceleration, hasPred, predIsCacc,
            dt, time, ref caccControlMode, ref accControlMode, ref accLastUpdateTime);

        // :125 -- onInsertion=true (disables the emergency-decel-relax correction inside
        // maximumSafeFollowSpeed -- see KraussModel.MaximumSafeFollowSpeed's own `onInsertion`
        // parameter/comment), UNLIKE ACC's own followSpeed (MSCFModel_ACC.cpp:99), which omits
        // the argument (defaults to false). This is a real, deliberate difference between the two
        // ports, not a copy/paste of ACC's call.
        var vSafe = KraussModel.MaximumSafeFollowSpeed(gap2pred, egoSpeed, predSpeed, predMaxDecel, vType, dt, onInsertion: true);

        // :136-144 emergency override.
        var speedOverride = Math.Min(EmergencyThreshold, gap2pred);
        if (vSafe + speedOverride < vCacc)
        {
            return Math.Max(0.0, vSafe + speedOverride);
        }

        return vCacc;
    }

    // MSCFModel_CACC.cpp:148-158 stopSpeed: MIN2(maximumSafeStopSpeed(gap, decel, speed, false,
    // actionStepLengthSecs), maxNextSpeed(speed)) -- provably the SAME formula
    // MSCFModel_Krauss::stopSpeed/MSCFModel_ACC::stopSpeed use (see AccModel.StopSpeed's own
    // header comment), so this is a thin, byte-identical pass-through, not a duplicate formula.
    public static double StopSpeed(double speed, double gap, ResolvedVType vType, double dt, double actionStepLengthSecs) =>
        KraussModel.StopSpeed(gap, speed, vType, dt, actionStepLengthSecs);
}
