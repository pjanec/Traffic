using Sim.Ingest;

namespace Sim.Core;

// C11-iv: IDMM (IDM with Memory / "Improved IDM") -- ported from
// sumo/src/microsim/cfmodels/MSCFModel_IDM.cpp/.h's `idmm=true` ctor arm. SUMO reuses the SAME
// MSCFModel_IDM class for both IDM and IDMM (a single `myIDMM`/`myAdaptationFactor` ctor flag,
// MSCFModel_IDM.cpp:37-47) -- this file does NOT duplicate IdmModel.cs's shared `_v`/finalizeSpeed
// bodies; it only holds the two small IDMM-specific pieces that plain IDM/ACC/CACC never exercise
// (myAdaptationFactor stays hardwired at 1.0 for all three of those, see IdmModel.cs's own header
// comment), threaded through IdmModel's new optional `headwayTimeOverride` parameter (V/FollowSpeed/
// FreeSpeed/StopSpeed) and applied by Engine.ComputeMoveIntent's own IDMM dispatch arm rather than
// by touching IdmModel.FinalizeSpeed's shared body (see that function's own C11-iv comment).
//
// Constants: MSCFModel_IDM's ctor (:41-42) `idmm ? getCFParam(SUMO_ATTR_CF_IDMM_ADAPT_FACTOR, 1.8)
// : 1.0` / `idmm ? getCFParam(SUMO_ATTR_CF_IDMM_ADAPT_TIME, 600.0) : 0.0` -- scenario 25's vTypes
// carry no <param> override for either attribute, so these are the literal DEFAULT_* constants,
// not parsed per-vType fields (matching IdmModel.Delta's own "no override in scope" citation).
public static class IdmmModel
{
    public const double AdaptationFactor = 1.8;
    public const double AdaptationTime = 600.0;

    // MSCFModel_IDM.cpp:203-207 _v's headwayTime-adaptation block:
    //   headwayTime = myHeadwayTime;
    //   if (myAdaptationFactor != 1.) headwayTime *= myAdaptationFactor + levelOfService*(1.-myAdaptationFactor);
    // For IDMM (AdaptationFactor=1.8 != 1.), this is always taken:
    //   headwayTime = tau * (1.8 + LOS*(1-1.8)) = tau * (1.8 - 0.8*LOS)
    // At LOS=1.0 (the ctor default, and IDM/ACC/CACC's permanent value) this collapses to
    // `tau * 1.0 == tau`, exactly IDM's own unadapted headway.
    public static double AdaptedHeadwayTime(double tau, double levelOfService) =>
        tau * (AdaptationFactor + (levelOfService * (1.0 - AdaptationFactor)));

    // MSCFModel_IDM.cpp:67-74 finalizeSpeed's `if (myAdaptationFactor != 1.)` levelOfService
    // update, applied by the CALLER (Engine.ComputeMoveIntent) immediately after IdmModel.
    // FinalizeSpeed returns `vNext` -- see that function's own C11-iv comment for why the shared
    // body itself stays untouched:
    //   vars->levelOfService += (vNext / veh->getLane()->getVehicleMaxSpeed(veh) - vars->levelOfService) / myAdaptationTime * TS;
    // `TS` is the vendored source's GLOBAL simulation step length (not the per-vehicle
    // actionStepLengthSecs) -- Engine.ComputeMoveIntent's own `dt` local (== `_config.StepLength`)
    // is the exact match.
    public static double UpdateLevelOfService(double levelOfService, double vNext, double laneVehicleMaxSpeed, double dt) =>
        levelOfService + (((vNext / laneVehicleMaxSpeed) - levelOfService) / AdaptationTime * dt);
}
