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

    // MSCFModel.cpp:75-96 brakeGap. Euler branch (`brakeGapEuler`) by default; C8-iii adds the
    // ballistic branch (MSCFModel.cpp:79-86 `speed <= 0 ? 0 : speed * (headwayTime + 0.5*speed/decel)`)
    // taken only when `ballistic` is set. Called from maximumSafeFollowSpeed with headwayTime=0.
    // `ballistic` defaults false so every Euler call site is byte-identical.
    public static double BrakeGap(double speed, double decel, double headwayTime, double dt, bool ballistic = false)
    {
        if (ballistic)
        {
            return speed <= 0.0 ? 0.0 : speed * (headwayTime + (0.5 * speed / decel));
        }

        var speedReduction = Accel2Speed(decel, dt);
        var steps = (int)(speed / speedReduction);
        return Speed2Dist((steps * speed) - (speedReduction * steps * (steps + 1) / 2.0), dt) + (speed * headwayTime);
    }

    // MSCFModel.cpp:855-910 maximumSafeStopSpeedBallistic -- the ballistic counterpart of
    // maximumSafeStopSpeedEuler (C8-iii). Given a gap, returns the maximum speed from which the
    // vehicle can still stop within `gap` under the ballistic update (constant acceleration across
    // the step; a NEGATIVE return means "brake so hard you stop mid-step", clamped by the caller).
    // `onInsertion` uses the constant-insertion-speed form (the vehicle covers no distance until the
    // next step). `emergencyDecel` supplies the hard-brake return for the g==0 case.
    public static double MaximumSafeStopSpeedBallistic(
        double gap, double decel, double currentSpeed, bool onInsertion, double headway, double emergencyDecel, double dt)
    {
        var g = Math.Max(0.0, gap - NumericalEps);
        var h = headway >= 0 ? headway : throw new ArgumentException("headway must be >= 0", nameof(headway));

        if (onInsertion)
        {
            // g = tau*v0 + v0^2/(2b); solve for v0.
            var btauIns = decel * h;
            return -btauIns + Math.Sqrt((btauIns * btauIns) + (2.0 * decel * g));
        }

        var tau = h == 0.0 ? dt : h;
        var v0 = Math.Max(0.0, currentSpeed);

        // Case 1: a stop must take place within time tau.
        if (v0 * tau >= 2.0 * g)
        {
            if (g == 0.0)
            {
                return v0 > 0.0 ? -Accel2Speed(emergencyDecel, dt) : 0.0;
            }

            // g = v0^2/(-2a); return v0 + a*TS.
            var a1 = -v0 * v0 / (2.0 * g);
            return v0 + (a1 * dt);
        }

        // Case 2: the vehicle may still have positive speed v1 after time tau.
        // 0 = v1^2 + b*tau*v1 + b*tau*v0 - 2bg  =>  v1 = -b*tau/2 + sqrt((b*tau)^2/4 + b(2g - tau*v0)).
        var btau2 = decel * tau / 2.0;
        var v1 = -btau2 + Math.Sqrt((btau2 * btau2) + (decel * ((2.0 * g) - (tau * v0))));
        var a = (v1 - v0) / tau;
        return v0 + (a * dt);
    }

    // MSCFModel.cpp:getMinimalArrivalTime -- the minimal time (SECONDS here; the source returns
    // TIME2STEPS) for a vehicle at `currentSpeed` to cover `dist` and arrive at `arrivalSpeed`,
    // either decelerating as late as possible or accelerating then holding. Used by the junction
    // arrival-time right-of-way (MSLink::opened/blockedByFoe) to place each vehicle's arrival window
    // at a conflicting link. We keep it in seconds because the block decision only ever compares
    // ego-vs-foe windows against a lookAhead, so the common `t - DELTA_T` offset and the ms
    // quantization both cancel.
    public static double MinimalArrivalTime(double dist, double currentSpeed, double arrivalSpeed, ResolvedVType vType)
    {
        if (dist <= 0.0)
        {
            return 0.0;
        }

        var accel = arrivalSpeed >= currentSpeed ? vType.Accel : -vType.Decel;
        var accelTime = accel == 0.0 ? 0.0 : (arrivalSpeed - currentSpeed) / accel;
        var accelWay = accelTime * (arrivalSpeed + currentSpeed) * 0.5;
        if (dist >= accelWay)
        {
            var nonAccelWay = dist - accelWay;
            var nonAccelSpeed = Math.Max(currentSpeed, Math.Max(arrivalSpeed, HaltingSpeed));
            return accelTime + nonAccelWay / nonAccelSpeed;
        }

        return -(currentSpeed - Math.Sqrt(currentSpeed * currentSpeed + 2.0 * accel * dist)) / accel;
    }

    // MSCFModel.cpp:105-121 freeSpeed (the BASE class method, semi-implicit Euler arm only --
    // phase 1 is Euler-only per CLAUDE.md/DESIGN.md). This is NOT overridden by MSCFModel_Krauss,
    // so a Krauss vehicle uses exactly this braking-curve formula for the successive-lane speed
    // limit lookahead in MSVehicle::planMoveInternal (MSVehicle.cpp:2896:
    // `cfModel.freeSpeed(this, getSpeed(), seen, laneMaxV)`): the maximum speed from which the
    // vehicle can still slow to `targetSpeed` after covering `dist`, given max deceleration.
    // `onInsertion` is always false at that call site, so its `+1` term in fullSpeedGain is
    // dropped. (IDM overrides freeSpeed -- MSCFModel_IDM.cpp:78 -- so the IDM family routes through
    // IdmModel.FreeSpeed instead; see Engine.SuccessiveLaneSpeedConstraint.) Unit macros:
    // SPEED2DIST(x)=x*TS, ACCEL2DIST(x)=x*TS*TS, DIST2SPEED(x)=x/TS, ACCEL2SPEED(x)=x*TS.
    public static double FreeSpeed(double currentSpeed, double decel, double dist, double targetSpeed, double dt)
    {
        var v = Speed2Dist(targetSpeed, dt);
        if (dist < v)
        {
            return targetSpeed;
        }

        var b = decel * dt * dt;
        var y = Math.Max(0.0, ((Math.Sqrt((b + 2.0 * v) * (b + 2.0 * v) + 8.0 * b * dist) - b) * 0.5 - v) / b);
        var yFull = Math.Floor(y);
        var exactGap = (yFull * yFull + yFull) * 0.5 * b + yFull * v + (y > yFull ? v : 0.0);
        var fullSpeedGain = yFull * Accel2Speed(decel, dt);
        return Dist2Speed(Math.Max(0.0, dist - exactGap) / (yFull + 1), dt) + fullSpeedGain + targetSpeed;
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
        bool onInsertion = false,
        bool ballistic = false)
    {
        var headway = vType.Tau; // myHeadwayTime

        double x;
        if (gap >= 0)
        {
            // C8-iii: both the leader-brakeGap term and the stop-speed take their ballistic branch
            // under ballistic integration (MSCFModel.cpp:936-940 dispatch on gSemiImplicitEulerUpdate).
            var effectiveGap = gap + BrakeGap(predSpeed, Math.Max(vType.Decel, predMaxDecel), 0.0, dt, ballistic);
            x = ballistic
                ? MaximumSafeStopSpeedBallistic(effectiveGap, vType.Decel, egoSpeed, onInsertion, headway, vType.EmergencyDecel, dt)
                : MaximumSafeStopSpeed(effectiveGap, vType.Decel, headway, dt);
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
        double dt,
        bool ballistic = false)
    {
        var vsafe = MaximumSafeFollowSpeed(gap, egoSpeed, predSpeed, predMaxDecel, vType, dt, onInsertion: false, ballistic: ballistic);
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
    //
    // C7-i: `speedFactor` is the CALLER's per-vehicle chosen speedFactor
    // (VehicleRuntime.SpeedFactor -- MSVehicleType::computeChosenSpeedDeviation's result, drawn
    // once at creation), NOT vType.SpeedFactor (that field is now only the DISTRIBUTION MEAN fed
    // into the sampler -- see NormcDistribution.cs / VehicleRuntime.SpeedFactor's own comments).
    // Every existing (speeddev=0) scenario has SpeedFactor==vType.SpeedFactor==1.0 exactly, so
    // this stays byte-identical to the pre-C7 `vType.SpeedFactor`-only formula.
    public static double LaneVehicleMaxSpeed(double laneSpeed, double speedFactor, ResolvedVType vType) =>
        Math.Min(laneSpeed * speedFactor, vType.MaxSpeed);

    // MSCFModel.cpp: finalizeSpeed(veh, vPos). vPos is the MIN over the (already-reduced)
    // leader/junction/stop-line constraint collection computed by the caller; laneVehicleMaxSpeed
    // is the lane's speed limit adaptation for this vehicle (MSLane::getVehicleMaxSpeed).
    //
    // vStop (MSCFModel.cpp:191: `MIN2(vPos, veh->processNextStop(vPos))`) is now a real caller-
    // supplied input (rung 5: Engine.ProcessNextStop ports processNextStop's reached/duration
    // state machine); the caller has already taken that MIN2 before calling in, so this method
    // only has to consume the result -- when there is no stop it is simply vPos again (a no-op
    // MIN, exactly like phase 1-4's dead +infinity slot).
    //
    // C1-i: `rng` is threaded `ref` so patchSpeedBeforeLC's dawdle draw (below) advances the
    // CALLER's (this vehicle's own) VehicleRuntime.RngState in place -- see Engine.
    // ComputeMoveIntent's call site. When vType.Sigma==0 (every pre-C1 rung's scenario), `rng`
    // is never read/advanced and vNext==vMax exactly as before this rung -- byte-identical.
    public static double FinalizeSpeed(
        double oldV,
        double vPos,
        double vStop,
        double laneVehicleMaxSpeed,
        ResolvedVType vType,
        double dt,
        double actionStepLengthSecs,
        ref VehicleRng rng)
    {
        var vMinEmergency = MinNextSpeedEmergency(oldV, vType, dt);
        // vMin = MIN2(minNextSpeed(oldV), MAX2(vPos, vMinEmergency))
        var vMin = Math.Min(MinNextSpeed(oldV, vType, dt), Math.Max(vPos, vMinEmergency));

        // getFriction()==1 in phase 1 (no weather/friction model) -> factor == 1.
        const double factor = 1.0;

        var aMax = ((Math.Max(laneVehicleMaxSpeed, vPos) * factor) - oldV) / actionStepLengthSecs;
        var vMax = Min3(oldV + Accel2Speed(aMax, dt), MaxNextSpeed(oldV, vType, dt), vStop);
        vMax = Math.Max(vMin, vMax);

        // MSCFModel.cpp:220 `vNext = patchSpeedBeforeLC(veh, vMin, vMax)`. Our resolved
        // carFollowModel is always "Krauss" (VTypeDefaults.Resolve), i.e.
        // MSCFModel_Krauss::patchSpeedBeforeLC with the default sigmaStep==DELTA_T (per-step)
        // path -- MSCFModel_Krauss.cpp:73-96's `myDawdleStep > DELTA_T` accelDawdle branch is a
        // DEFERRED feature (sigmaStep>dt sub-stepped dawdling is out of scope for C1-i; every
        // vType here resolves sigmaStep==dt via the default cf-param), so this always takes the
        // `else` arm: `vDawdle = MAX2(vMin, dawdle2(vMax, sigma, rng))`.
        //
        // sigma==0 is the strict no-op case required for byte-identical parity: dawdle2 would
        // still draw a random number even at sigma==0 in SUMO's own C++ (the draw happens
        // unconditionally, only its effect is scaled to zero by sigma==0's multiplication) --
        // but we skip the draw entirely here rather than draw-and-multiply-by-zero, so that (a)
        // no existing sigma=0 scenario's RNG-state timeline is perturbed by a call that used to
        // not exist, and (b) `vNext` is bit-for-bit `vMax`, matching every prior rung exactly
        // (0.0 * anything is still exactly 0.0 in IEEE 754, so this is a genuine no-op, not an
        // approximation -- we simply also avoid the pointless draw+multiply).
        //
        // No lane-change model exists in phase 1 (MSAbstractLaneChangeModel::patchSpeed is a
        // no-op) and applyStartupDelay defaults to 0 (MSCFModel.cpp:232) -- both unaffected by
        // C1-i, exactly as before.
        var vNext = vType.Sigma > 0.0 ? Math.Max(vMin, Dawdle2(vMax, vType.Sigma, vType.Accel, dt, ref rng)) : vMax;
        return vNext;
    }

    // MSCFModel_Krauss.cpp:129-151 dawdle2, Euler branch only (the `!gSemiImplicitEulerUpdate`
    // negative-speed short-circuit at the top is a ballistic-only guard -- out of scope per
    // phase 1/DESIGN.md, Euler is the only integration mode). One draw per call:
    //   random = rand[0,1)
    //   if (speed < myAccel) speed -= ACCEL2SPEED(sigma * speed * random);
    //   else                 speed -= ACCEL2SPEED(sigma * myAccel * random);
    //   return MAX2(0., speed);
    // `speed` here is vMax (the caller always passes vMax, matching
    // MSCFModel_Krauss.cpp:91's `dawdle2(vMax, sigma, veh->getRNG())`).
    private static double Dawdle2(double speed, double sigma, double accel, double dt, ref VehicleRng rng)
    {
        var random = rng.NextDouble();
        speed -= Accel2Speed(sigma * Math.Min(speed, accel) * random, dt);
        return Math.Max(0.0, speed);
    }

    private static double Min3(double a, double b, double c) => Math.Min(a, Math.Min(b, c));
}
