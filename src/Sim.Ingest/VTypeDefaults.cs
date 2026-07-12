namespace Sim.Ingest;

// Fully-resolved vType parameters used by the car-following model. .rou.xml only captures
// attributes explicitly present (see VType above); everything else must be filled from SUMO's
// vClass-default tables so the engine's init matches what SUMO actually resolves internally.
//
// Values and citations below are cross-checked against BOTH the vendored source AND a
// libsumo/TraCI dump of the resolved passenger defaults (they agree -- see
// scenarios/01-single-free-flow/VTYPE_CROSSCHECK.md / golden.vtype.json). CLAUDE.md rule 6:
// diff resolved defaults against golden.state.xml/golden.vtype.json before chasing trajectory
// drift -- this resolver plus its init cross-check test exists precisely for that.
public sealed record ResolvedVType(
    string Id,
    string VClass,
    string CarFollowModel,
    double Length,
    double MinGap,
    double MaxSpeed,
    double Accel,
    double Decel,
    double EmergencyDecel,
    double ApparentDecel,
    double Sigma,
    double Tau,
    double SpeedFactor,
    double Width,
    double Height,
    // Rung A3: sumo/src/microsim/MSVehicle.cpp:7266 ignoreRed's jmDriveAfterRedTime junction-
    // model param. NOT a per-vClass default-table value (see RawDefaults below) -- it is a
    // per-vType override that defaults to -1 ("never ignore red") for every vClass alike,
    // threaded straight from the raw vType the same way Sigma is.
    double JmDriveAfterRedTime,
    // Rung ER2 (emergency ignore-FOE at junctions). Three junction-model params, all defaulting
    // to 0 ("never ignore a foe") for every vClass alike, so every vType that does not set them is
    // byte-identical to before this rung. Same per-vType override threading as JmDriveAfterRedTime.
    //   JmIgnoreFoeProb / JmIgnoreFoeSpeed: MSLink::blockedAtTime (sumo/src/microsim/MSLink.cpp:
    //     898-902, reached from MSLink::opened) -- an APPROACHING foe is ignored iff
    //     jmIgnoreFoeProb>0 AND jmIgnoreFoeSpeed>=foe.speed AND jmIgnoreFoeProb>=rand(). Gates the
    //     approaching-foe stop-line yield arm of JunctionYieldConstraint.
    //   JmIgnoreJunctionFoeProb: MSVehicle::checkLinkLeaderCurrentAndParallel (sumo/src/microsim/
    //     MSVehicle.cpp:3419/3430) -- an ON-JUNCTION link-leader is ignored iff
    //     jmIgnoreJunctionFoeProb>0 AND jmIgnoreJunctionFoeProb>=rand() (no speed gate). Gates the
    //     adaptToJunctionLeader arm.
    double JmIgnoreFoeProb,
    double JmIgnoreFoeSpeed,
    double JmIgnoreJunctionFoeProb,
    // Rung ER3 (give-way): does this vType carry an active blue-light siren (our opt-in model of
    // SUMO's MSDevice_Bluelight)? Defaults to false, so give-way detection is inert for every
    // vType that does not set hasBluelight="true".
    bool HasBluelight,
    // Rung OV1 (opposite-direction overtaking): may this vType overtake using the oncoming lane?
    // Defaults to false, so the whole opposite-direction subsystem is inert for every other vType.
    bool LcOpposite,
    // Phase 2 (sublane, P2.3): SUMO's sublane vType params (SUMOVTypeParameter.cpp:333-335).
    // MaxSpeedLat = max lateral speed (m/s, default 1.0); LatAlignment = preferred lateral
    // alignment (default "center"). Read only by the sublane lateral driver (lateral-resolution >
    // 0); for every phase-1 vType they carry these inert defaults and are never read.
    double MaxSpeedLat,
    string LatAlignment,
    // Rung P2-core (keepLatGap): SUMOVTypeParameter.cpp:62 minGapLat(0.6) -- the desired lateral
    // gap (m) kept from a same-lane neighbour (MSLCM_SL2015::keepLatGap/updateGaps). Overridable
    // via the vType's minGapLat attribute; inert (never read) unless lateral-resolution > 0.
    double MinGapLat,
    // Rung R6: resolved MSCFModel_Rail traction parameters. Only meaningful when
    // CarFollowModel=="Rail"; for every other model they stay at these inert defaults and are never
    // read. Weight/MassFactor give the rotating mass (rotWeight = Weight*MassFactor);
    // MaxPower/MaxTraction the parametric traction curve; ResCoef* the parametric resistance curve.
    double Weight = 0.0,
    double MassFactor = 1.0,
    double MaxPower = double.NaN,
    double MaxTraction = double.NaN,
    double ResCoefConstant = double.NaN,
    double ResCoefLinear = double.NaN,
    double ResCoefQuadratic = double.NaN);

public static class VTypeDefaults
{
    // Raw (pre-override) per-vClass defaults, ported from:
    //   sumo/src/utils/common/SUMOVehicleClass.cpp:545 getDefaultVehicleLength(vc)
    //   sumo/src/utils/vehicle/SUMOVTypeParameter.cpp:59  VClassDefaultValues::VClassDefaultValues(vc)
    //     (minGap, maxSpeed, width, height overrides per vclass; default minGap=2.5,
    //     maxSpeed=200./3.6, width=1.8, height=1.5 when a case falls through to `default:`)
    //   sumo/src/utils/vehicle/SUMOVTypeParameter.cpp:834 getDefaultAccel(vc)
    //   sumo/src/utils/vehicle/SUMOVTypeParameter.cpp:872 getDefaultDecel(vc)
    //   sumo/src/utils/vehicle/SUMOVTypeParameter.cpp:905 getDefaultEmergencyDecel(vc, decel, ...)
    //     (VcDecel below is the `vcDecel` local from that switch; the actual emergencyDecel
    //     default is MAX2(decel, VcDecel), applied AFTER decel overrides are resolved -- see
    //     Resolve() below, never hardcoded here.)
    // getDefaultImperfection (SUMOVTypeParameter.cpp:948) is vClass-dependent, NOT uniform: the
    // road classes below all fall through to its `default:` branch (sigma=0.5), but the rail-family
    // classes (tram, rail_urban, rail, rail_electric, rail_fast; also ship) return 0.0. So sigma's
    // default is carried per-class in the table (the Sigma field), not hardcoded in Resolve().
    // carFollowModel="Krauss", tau=1.0, speedFactor mean=1.0 ARE uniform across this table
    // (SUMOVTypeParameter.cpp:331, MSCFModel.cpp:63, SUMOVTypeParameter.cpp:317) and are not part
    // of the per-class lookup.
    private readonly record struct RawDefaults(
        double Length,
        double MinGap,
        double MaxSpeed,
        double Accel,
        double Decel,
        double VcDecel,
        double Sigma,
        double Width,
        double Height);

    private static readonly Dictionary<string, RawDefaults> RawDefaultsByVClass = new()
    {
        // length=5 (getDefaultVehicleLength `default:`), minGap=2.5, maxSpeed=200/3.6, width=1.8,
        // height=1.5 (VClassDefaultValues `default:` -- SVC_PASSENGER falls through with only
        // `shape` and `speedFactor` sigma set, neither tracked here), accel=2.6, decel=4.5
        // (both `default:` branches), vcDecel=9.0 (getDefaultEmergencyDecel `default:`).
        ["passenger"] = new RawDefaults(Length: 5.0, MinGap: 2.5, MaxSpeed: 200.0 / 3.6, Accel: 2.6, Decel: 4.5, VcDecel: 9.0, Sigma: 0.5, Width: 1.8, Height: 1.5),

        // SUMOVehicleClass.cpp:559 length=7.1. SUMOVTypeParameter.cpp SVC_TRUCK case: maxSpeed
        // =130/3.6, width=2.4, height=2.4 (minGap stays at the constructor default 2.5).
        // getDefaultAccel SVC_TRUCK=1.3, getDefaultDecel SVC_TRUCK=4.0,
        // getDefaultEmergencyDecel SVC_TRUCK vcDecel=7.0.
        ["truck"] = new RawDefaults(Length: 7.1, MinGap: 2.5, MaxSpeed: 130.0 / 3.6, Accel: 1.3, Decel: 4.0, VcDecel: 7.0, Sigma: 0.5, Width: 2.4, Height: 2.4),

        // SUMOVehicleClass.cpp:563 length=12. SVC_BUS case: maxSpeed=100/3.6, width=2.5,
        // height=3.4. getDefaultAccel SVC_BUS=1.2, getDefaultDecel SVC_BUS=4.0,
        // getDefaultEmergencyDecel SVC_BUS vcDecel=7.0.
        ["bus"] = new RawDefaults(Length: 12.0, MinGap: 2.5, MaxSpeed: 100.0 / 3.6, Accel: 1.2, Decel: 4.0, VcDecel: 7.0, Sigma: 0.5, Width: 2.5, Height: 3.4),

        // SUMOVehicleClass.cpp:565 length=14. SVC_COACH case: maxSpeed=100/3.6, width=2.6,
        // height=4.0. getDefaultAccel SVC_COACH=2.0, getDefaultDecel SVC_COACH=4.0,
        // getDefaultEmergencyDecel SVC_COACH vcDecel=7.0.
        ["coach"] = new RawDefaults(Length: 14.0, MinGap: 2.5, MaxSpeed: 100.0 / 3.6, Accel: 2.0, Decel: 4.0, VcDecel: 7.0, Sigma: 0.5, Width: 2.6, Height: 4.0),

        // SUMOVehicleClass.cpp:577 length=6.5 (shared with SVC_EMERGENCY). SVC_DELIVERY case:
        // width=2.16, height=2.86 (maxSpeed stays at the constructor default 200/3.6).
        // getDefaultAccel/getDefaultDecel/getDefaultEmergencyDecel all fall through to their
        // `default:` branches (2.6 / 4.5 / vcDecel=9.0) -- SVC_DELIVERY has no case in any of
        // those three switches.
        ["delivery"] = new RawDefaults(Length: 6.5, MinGap: 2.5, MaxSpeed: 200.0 / 3.6, Accel: 2.6, Decel: 4.5, VcDecel: 9.0, Sigma: 0.5, Width: 2.16, Height: 2.86),

        // SUMOVehicleClass.cpp:561 length=16.5. SVC_TRAILER case: maxSpeed=130/3.6, width=2.55,
        // height=4.0. getDefaultAccel SVC_TRAILER=1.1, getDefaultDecel SVC_TRAILER=4.0,
        // getDefaultEmergencyDecel SVC_TRAILER vcDecel=7.0.
        ["trailer"] = new RawDefaults(Length: 16.5, MinGap: 2.5, MaxSpeed: 130.0 / 3.6, Accel: 1.1, Decel: 4.0, VcDecel: 7.0, Sigma: 0.5, Width: 2.55, Height: 4.0),

        // SUMOVehicleClass.cpp:552 length=1.6. SVC_BICYCLE case: minGap=0.5, maxSpeed=50/3.6,
        // width=0.65, height=1.7. getDefaultAccel SVC_BICYCLE=1.2, getDefaultDecel
        // SVC_BICYCLE=3.0, getDefaultEmergencyDecel SVC_BICYCLE vcDecel=7.0.
        ["bicycle"] = new RawDefaults(Length: 1.6, MinGap: 0.5, MaxSpeed: 50.0 / 3.6, Accel: 1.2, Decel: 3.0, VcDecel: 7.0, Sigma: 0.5, Width: 0.65, Height: 1.7),

        // SUMOVehicleClass.cpp:557 length=2.2. SVC_MOTORCYCLE case: width=0.9, height=1.5
        // (maxSpeed stays at the constructor default 200/3.6; minGap stays at 2.5).
        // getDefaultAccel SVC_MOTORCYCLE=6.0, getDefaultDecel SVC_MOTORCYCLE=10.0,
        // getDefaultEmergencyDecel SVC_MOTORCYCLE vcDecel=10.0.
        ["motorcycle"] = new RawDefaults(Length: 2.2, MinGap: 2.5, MaxSpeed: 200.0 / 3.6, Accel: 6.0, Decel: 10.0, VcDecel: 10.0, Sigma: 0.5, Width: 0.9, Height: 1.5),

        // SUMOVehicleClass.cpp:555 length=2.1. SVC_MOPED case: maxSpeed=60/3.6, width=0.78,
        // height=1.7 (minGap stays at 2.5). getDefaultAccel SVC_MOPED=1.1, getDefaultDecel
        // SVC_MOPED=7.0, getDefaultEmergencyDecel SVC_MOPED vcDecel=10.0.
        ["moped"] = new RawDefaults(Length: 2.1, MinGap: 2.5, MaxSpeed: 60.0 / 3.6, Accel: 1.1, Decel: 7.0, VcDecel: 10.0, Sigma: 0.5, Width: 0.78, Height: 1.7),

        // SUMOVehicleClass.cpp:578 length=6.5 (shared with SVC_DELIVERY). SVC_EMERGENCY case:
        // width=2.16, height=2.86 (maxSpeed stays at the constructor default 200/3.6).
        // getDefaultAccel/getDefaultDecel/getDefaultEmergencyDecel all fall through to their
        // `default:` branches (2.6 / 4.5 / vcDecel=9.0) -- SVC_EMERGENCY has no case in any of
        // those three switches.
        ["emergency"] = new RawDefaults(Length: 6.5, MinGap: 2.5, MaxSpeed: 200.0 / 3.6, Accel: 2.6, Decel: 4.5, VcDecel: 9.0, Sigma: 0.5, Width: 2.16, Height: 2.86),

        // --- Rail family (rung R1) -------------------------------------------------------------
        // Ported from the SAME three tables as the road classes above, rail cases:
        //   getDefaultVehicleLength (SUMOVehicleClass.cpp:567): tram=22, rail_urban/subway=36.5*3,
        //     rail=67.5*2, rail_electric/rail_fast=25*8.
        //   VClassDefaultValues (SUMOVTypeParameter.cpp:138 SVC_TRAM .. :180 SVC_RAIL_FAST):
        //     minGap/maxSpeed/width/height per case (tram keeps the ctor-default minGap 2.5;
        //     all *rail* classes set minGap=5).
        //   getDefaultAccel (SUMOVTypeParameter.cpp:854): tram=1, rail_urban=1, rail=0.25,
        //     rail_electric/rail_fast=0.5; SVC_SUBWAY has NO case -> `default:` 2.6.
        //   getDefaultDecel (SUMOVTypeParameter.cpp:889): tram/rail_urban=3, rail/rail_electric/
        //     rail_fast=1.3; SVC_SUBWAY -> `default:` 4.5.
        //   getDefaultEmergencyDecel vcDecel (SUMOVTypeParameter.cpp:924): tram/rail_urban=7,
        //     rail/rail_electric/rail_fast=5; SVC_SUBWAY -> `default:` 9.
        //   getDefaultImperfection (SUMOVTypeParameter.cpp:948): tram/rail_urban/rail/rail_electric/
        //     rail_fast=0.0; SVC_SUBWAY -> `default:` 0.5.
        // NOTE subway shares its VClassDefaultValues case with rail_urban (length/minGap/maxSpeed/
        // width/height) but has NO case in the accel/decel/emergencyDecel/imperfection switches, so
        // it falls through to their road `default:` values -- a genuinely mixed profile, faithfully
        // reproduced here.

        // SVC_TRAM: length 22, minGap 2.5 (ctor default), maxSpeed 80/3.6, accel 1, decel 3,
        // vcDecel 7, sigma 0, width 2.4, height 3.2.
        ["tram"] = new RawDefaults(Length: 22.0, MinGap: 2.5, MaxSpeed: 80.0 / 3.6, Accel: 1.0, Decel: 3.0, VcDecel: 7.0, Sigma: 0.0, Width: 2.4, Height: 3.2),

        // SVC_RAIL_URBAN: length 36.5*3=109.5, minGap 5, maxSpeed 100/3.6, accel 1, decel 3,
        // vcDecel 7, sigma 0, width 3.0, height 3.6.
        ["rail_urban"] = new RawDefaults(Length: 36.5 * 3, MinGap: 5.0, MaxSpeed: 100.0 / 3.6, Accel: 1.0, Decel: 3.0, VcDecel: 7.0, Sigma: 0.0, Width: 3.0, Height: 3.6),

        // SVC_SUBWAY: shares VClassDefaultValues with rail_urban (length 36.5*3, minGap 5,
        // maxSpeed 100/3.6, width 3.0, height 3.6) but accel/decel/vcDecel/sigma fall through to
        // the road `default:` branches (2.6 / 4.5 / 9.0 / 0.5).
        ["subway"] = new RawDefaults(Length: 36.5 * 3, MinGap: 5.0, MaxSpeed: 100.0 / 3.6, Accel: 2.6, Decel: 4.5, VcDecel: 9.0, Sigma: 0.5, Width: 3.0, Height: 3.6),

        // SVC_RAIL: length 67.5*2=135, minGap 5, maxSpeed 160/3.6, accel 0.25, decel 1.3,
        // vcDecel 5, sigma 0, width 2.84, height 3.75.
        ["rail"] = new RawDefaults(Length: 67.5 * 2, MinGap: 5.0, MaxSpeed: 160.0 / 3.6, Accel: 0.25, Decel: 1.3, VcDecel: 5.0, Sigma: 0.0, Width: 2.84, Height: 3.75),

        // SVC_RAIL_ELECTRIC: length 25*8=200, minGap 5, maxSpeed 220/3.6, accel 0.5, decel 1.3,
        // vcDecel 5, sigma 0, width 2.95, height 3.89.
        ["rail_electric"] = new RawDefaults(Length: 25.0 * 8, MinGap: 5.0, MaxSpeed: 220.0 / 3.6, Accel: 0.5, Decel: 1.3, VcDecel: 5.0, Sigma: 0.0, Width: 2.95, Height: 3.89),

        // SVC_RAIL_FAST: length 25*8=200, minGap 5, maxSpeed 330/3.6, accel 0.5, decel 1.3,
        // vcDecel 5, sigma 0, width 2.95, height 3.89.
        ["rail_fast"] = new RawDefaults(Length: 25.0 * 8, MinGap: 5.0, MaxSpeed: 330.0 / 3.6, Accel: 0.5, Decel: 1.3, VcDecel: 5.0, Sigma: 0.0, Width: 2.95, Height: 3.89),
    };

    // Generalized (rung A1) so that any attribute the rou.xml <vType> sets EXPLICITLY overrides
    // the vClass-default table above, via `override ?? default` per attribute -- e.g. rung 4's
    // leader sets maxSpeed="5.00" and both its vTypes set sigma="0". ApparentDecel is not itself
    // overridable from rou.xml in our scope; it derives from the (possibly overridden) decel,
    // matching MSCFModel.cpp:61's getCFParam(SUMO_ATTR_APPARENTDECEL, myDecel) default-to-decel
    // fallback.
    // sumo/src/utils/common/SUMOVehicleClass.cpp:497 isRailway(permissions): true iff the class is
    // in SVC_RAIL_CLASSES (rail, rail_electric, rail_fast, rail_urban, tram, subway, cable_car) and
    // not passenger. For a single-vClass vehicle this is just membership in that set. Used by the
    // engine's R3 rail-bidi insertion check (rail vehicles don't insert onto a bidi track whose
    // opposing lane is occupied). cable_car is out of the curated table's scope.
    private static readonly HashSet<string> RailwayVClasses = new(StringComparer.Ordinal)
    {
        "rail", "rail_electric", "rail_fast", "rail_urban", "tram", "subway",
    };

    public static bool IsRailway(ResolvedVType vType) => RailwayVClasses.Contains(vType.VClass);

    public static ResolvedVType Resolve(VType vType)
    {
        // SUMOVehicleParserHelper.cpp:1658 getOpt<std::string>(SUMO_ATTR_VCLASS, ..., "") ->
        // getVehicleClassID("") resolves to SVC_PASSENGER when a <vType> omits vClass entirely.
        var vClass = string.IsNullOrEmpty(vType.VClass) ? "passenger" : vType.VClass;

        if (!RawDefaultsByVClass.TryGetValue(vClass, out var raw))
        {
            throw new NotSupportedException(
                $"VTypeDefaults.Resolve does not support vClass='{vClass}' (vType '{vType.Id}'). " +
                "Only the curated road-vClass table (passenger, truck, bus, coach, delivery, trailer, " +
                "bicycle, motorcycle, moped, emergency) and the rail family (tram, rail_urban, subway, " +
                "rail, rail_electric, rail_fast) are in scope.");
        }

        // SUMOVTypeParameter.cpp getDefaultDecel: overridable via rou.xml's decel="...".
        var decel = vType.Decel ?? raw.Decel;

        // Rung R6: MSCFModel_Rail (carFollowModel="Rail"). The Rail model OVERRIDES length, maxSpeed
        // and decel from its trainType parameter set (MSCFModel_Rail.cpp:91-106 setLength/setMaxSpeed/
        // setMaxDecel), so those come from the trainType (only "custom" is in scope) rather than the
        // vClass table. rotWeight = weight*massFactor; the parametric traction (maxPower/maxTraction)
        // and resistance (resCoef_*) curves are required (the built-in lookup tables are deferred).
        var isRail = vType.CarFollowModel == "Rail";
        var railLength = raw.Length;
        var railMaxSpeed = raw.MaxSpeed;
        var railDecel = decel;
        var railEmergency = vType.EmergencyDecel ?? Math.Max(decel, raw.VcDecel);
        var railWeight = 0.0;
        var railMassFactor = 1.0;
        var railMaxPower = double.NaN;
        var railMaxTraction = double.NaN;
        var railResC = double.NaN;
        var railResL = double.NaN;
        var railResQ = double.NaN;
        if (isRail)
        {
            var trainType = vType.TrainType ?? "NGT400"; // MSCFModel_Rail.cpp:62 default
            if (trainType != "custom")
            {
                throw new NotSupportedException(
                    $"VTypeDefaults.Resolve supports only trainType=\"custom\" for carFollowModel=\"Rail\" " +
                    $"(vType '{vType.Id}' uses trainType=\"{trainType}\"); the built-in NGT400/ICE/RB* lookup " +
                    "tables are out of scope (rung R6 minimal).");
            }

            // initCustomParams (MSCFModel_Rail.h:882): weight=100 t, mf=1.05, length=100 m, decl=1,
            // vmax=200/3.6. Then overridden by explicit vType attributes (MSCFModel_Rail.cpp:91-101).
            railWeight = vType.Mass.HasValue ? vType.Mass.Value / 1000.0 : 100.0; // mass is kg -> tons
            railMassFactor = vType.MassFactor ?? 1.05;
            railLength = vType.Length ?? 100.0;
            railMaxSpeed = vType.MaxSpeed ?? (200.0 / 3.6);
            railDecel = vType.Decel ?? 1.0;
            // setEmergencyDecel(getCFParam(EMERGENCYDECEL, decl + 0.3)) (MSCFModel_Rail.cpp:103).
            railEmergency = vType.EmergencyDecel ?? (railDecel + 0.3);
            railMaxPower = vType.MaxPower
                ?? throw new NotSupportedException($"carFollowModel=\"Rail\" vType '{vType.Id}' requires maxPower (parametric traction curve; tables out of scope).");
            railMaxTraction = vType.MaxTraction
                ?? throw new NotSupportedException($"carFollowModel=\"Rail\" vType '{vType.Id}' requires maxTraction.");
            railResC = vType.ResCoefConstant
                ?? throw new NotSupportedException($"carFollowModel=\"Rail\" vType '{vType.Id}' requires resCoef_constant (parametric resistance curve; tables out of scope).");
            railResL = vType.ResCoefLinear ?? 0.0;
            railResQ = vType.ResCoefQuadratic ?? 0.0;
        }

        return new ResolvedVType(
            Id: vType.Id,
            VClass: vClass,
            // SUMOVTypeParameter.cpp:331 cfModel(SUMO_TAG_CF_KRAUSS) -- default CF model;
            // overridable via rou.xml's carFollowModel="..." (C11-i: "IDM" is the only other
            // value in scope -- SUMOXMLDefinitions::CarFollowModels' "IDM" -> SUMO_TAG_CF_IDM).
            CarFollowModel: vType.CarFollowModel ?? "Krauss",
            // SUMOVehicleClass.cpp getDefaultVehicleLength; overridable via rou.xml's length="...".
            // R6: for the Rail model, length comes from the trainType (setLength), see railLength.
            Length: isRail ? railLength : vType.Length ?? raw.Length,
            // SUMOVTypeParameter.cpp:61 (or per-vclass override); overridable via rou.xml's
            // minGap="...".
            MinGap: vType.MinGap ?? raw.MinGap,
            // SUMOVTypeParameter.cpp:63 (or per-vclass override); overridable via rou.xml's
            // maxSpeed="..." (rung 4's leader sets maxSpeed="5.00" so the fast follower catches
            // up and settles into the Krauss steady-state gap).
            // R6: for the Rail model, maxSpeed = trainType vmax (setMaxSpeed), see railMaxSpeed.
            MaxSpeed: isRail ? railMaxSpeed : vType.MaxSpeed ?? raw.MaxSpeed,
            // SUMOVTypeParameter.cpp getDefaultAccel; overridable via rou.xml's accel="...". Unused
            // by the Rail model (its acceleration comes from the traction curve), left at the vClass
            // default there.
            Accel: vType.Accel ?? raw.Accel,
            // R6: for the Rail model, decel = trainType decl (setMaxDecel), see railDecel.
            Decel: isRail ? railDecel : decel,
            // getDefaultEmergencyDecel default option -> MAX2(decel, vcDecel); overridable via
            // rou.xml's emergencyDecel="..." (default computed from the possibly-overridden
            // decel and the per-vclass vcDecel, matching SUMOVTypeParameter.cpp's
            // MAX2(decel, vcDecel) fallback -- never hardcoded per-class). R6: Rail uses decl+0.3.
            EmergencyDecel: isRail ? railEmergency : vType.EmergencyDecel ?? Math.Max(decel, raw.VcDecel),
            // MSCFModel.cpp:61 getCFParam(SUMO_ATTR_APPARENTDECEL, myDecel) -- defaults to
            // (possibly-overridden) decel; not independently overridable in our scope.
            ApparentDecel: isRail ? railDecel : decel,
            // SUMOVTypeParameter.cpp getDefaultImperfection: vClass-dependent -- 0.5 for the road
            // classes, 0.0 for the rail family (tram/rail_urban/rail/rail_electric/rail_fast);
            // carried per-class in raw.Sigma above. Overridable via rou.xml's sigma="...". Rail
            // scenarios rely on this default being 0 for phase-1 determinism (no explicit sigma).
            Sigma: vType.Sigma ?? raw.Sigma,
            // MSCFModel.cpp:63 getCFParam(SUMO_ATTR_TAU, 1.0); overridable via rou.xml's
            // tau="...".
            Tau: vType.Tau ?? 1.0,
            // SUMOVTypeParameter.cpp:317 speedFactor("normc", 1.0, 0.0, 0.2, 2.0) -- mean 1.0.
            // Phase 1 has no System.Random / RNG at all (CLAUDE.md), and rung 1/4's config.sumocfg
            // additionally forces default.speeddev="0", so the drawn speedFactor is exactly its
            // mean, 1.0, with no per-vehicle deviation to model yet; overridable via rou.xml's
            // speedFactor="..." for a fixed (non-distributional) override.
            SpeedFactor: vType.SpeedFactor ?? 1.0,
            // Per-vclass width default (SUMOVTypeParameter.cpp constructor default 1.8, or the
            // vclass-specific override in VClassDefaultValues).
            Width: raw.Width,
            // Per-vclass height default (SUMOVTypeParameter.cpp constructor default 1.5, or the
            // vclass-specific override in VClassDefaultValues).
            Height: raw.Height,
            // MSVehicle.cpp:7266 getJMParam(SUMO_ATTR_JM_DRIVE_AFTER_RED_TIME, -1) -- not a
            // per-vClass table value (see field comment on ResolvedVType); overridable via
            // rou.xml's jmDriveAfterRedTime="..." (rung A3's emergency vType sets "1000").
            JmDriveAfterRedTime: vType.JmDriveAfterRedTime ?? -1.0,
            // Rung ER2: MSVehicle/MSLink getJMParam(..., 0) defaults -- 0 == "never ignore a foe",
            // so every vType that omits these attributes is inert (byte-identical to pre-ER2).
            JmIgnoreFoeProb: vType.JmIgnoreFoeProb ?? 0.0,
            JmIgnoreFoeSpeed: vType.JmIgnoreFoeSpeed ?? 0.0,
            JmIgnoreJunctionFoeProb: vType.JmIgnoreJunctionFoeProb ?? 0.0,
            // Rung ER3: false default -> give-way detection is inert for every non-bluelight vType.
            HasBluelight: vType.HasBluelight ?? false,
            // Rung OV1: false default -> opposite-direction overtaking is inert for every vType.
            LcOpposite: vType.LcOpposite ?? false,
            // Phase 2 (sublane): SUMOVTypeParameter.cpp:333 maxSpeedLat(1.0),
            // :335 latAlignmentProcedure(CENTER). Overridable via the vType's maxSpeedLat /
            // latAlignment attributes; inert (never read) unless lateral-resolution > 0.
            MaxSpeedLat: vType.MaxSpeedLat ?? 1.0,
            LatAlignment: vType.LatAlignment ?? "center",
            // Rung P2-core: SUMOVTypeParameter.cpp:62 minGapLat(0.6).
            MinGapLat: vType.MinGapLat ?? 0.6,
            // Rung R6: MSCFModel_Rail traction params (inert NaN/0 for every non-Rail vType).
            Weight: railWeight,
            MassFactor: railMassFactor,
            MaxPower: railMaxPower,
            MaxTraction: railMaxTraction,
            ResCoefConstant: railResC,
            ResCoefLinear: railResL,
            ResCoefQuadratic: railResQ);
    }
}
