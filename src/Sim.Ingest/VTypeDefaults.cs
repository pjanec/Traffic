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
    double Height);

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
    // sigma default is 0.5 for every vClass in this table (getDefaultImperfection's non-rail/ship
    // `default:` branch, SUMOVTypeParameter.cpp:952); carFollowModel="Krauss", tau=1.0,
    // speedFactor mean=1.0 are likewise uniform across this table (SUMOVTypeParameter.cpp:331,
    // MSCFModel.cpp:63, SUMOVTypeParameter.cpp:317) and are not part of the per-class lookup.
    private readonly record struct RawDefaults(
        double Length,
        double MinGap,
        double MaxSpeed,
        double Accel,
        double Decel,
        double VcDecel,
        double Width,
        double Height);

    private static readonly Dictionary<string, RawDefaults> RawDefaultsByVClass = new()
    {
        // length=5 (getDefaultVehicleLength `default:`), minGap=2.5, maxSpeed=200/3.6, width=1.8,
        // height=1.5 (VClassDefaultValues `default:` -- SVC_PASSENGER falls through with only
        // `shape` and `speedFactor` sigma set, neither tracked here), accel=2.6, decel=4.5
        // (both `default:` branches), vcDecel=9.0 (getDefaultEmergencyDecel `default:`).
        ["passenger"] = new RawDefaults(Length: 5.0, MinGap: 2.5, MaxSpeed: 200.0 / 3.6, Accel: 2.6, Decel: 4.5, VcDecel: 9.0, Width: 1.8, Height: 1.5),

        // SUMOVehicleClass.cpp:559 length=7.1. SUMOVTypeParameter.cpp SVC_TRUCK case: maxSpeed
        // =130/3.6, width=2.4, height=2.4 (minGap stays at the constructor default 2.5).
        // getDefaultAccel SVC_TRUCK=1.3, getDefaultDecel SVC_TRUCK=4.0,
        // getDefaultEmergencyDecel SVC_TRUCK vcDecel=7.0.
        ["truck"] = new RawDefaults(Length: 7.1, MinGap: 2.5, MaxSpeed: 130.0 / 3.6, Accel: 1.3, Decel: 4.0, VcDecel: 7.0, Width: 2.4, Height: 2.4),

        // SUMOVehicleClass.cpp:563 length=12. SVC_BUS case: maxSpeed=100/3.6, width=2.5,
        // height=3.4. getDefaultAccel SVC_BUS=1.2, getDefaultDecel SVC_BUS=4.0,
        // getDefaultEmergencyDecel SVC_BUS vcDecel=7.0.
        ["bus"] = new RawDefaults(Length: 12.0, MinGap: 2.5, MaxSpeed: 100.0 / 3.6, Accel: 1.2, Decel: 4.0, VcDecel: 7.0, Width: 2.5, Height: 3.4),

        // SUMOVehicleClass.cpp:565 length=14. SVC_COACH case: maxSpeed=100/3.6, width=2.6,
        // height=4.0. getDefaultAccel SVC_COACH=2.0, getDefaultDecel SVC_COACH=4.0,
        // getDefaultEmergencyDecel SVC_COACH vcDecel=7.0.
        ["coach"] = new RawDefaults(Length: 14.0, MinGap: 2.5, MaxSpeed: 100.0 / 3.6, Accel: 2.0, Decel: 4.0, VcDecel: 7.0, Width: 2.6, Height: 4.0),

        // SUMOVehicleClass.cpp:577 length=6.5 (shared with SVC_EMERGENCY). SVC_DELIVERY case:
        // width=2.16, height=2.86 (maxSpeed stays at the constructor default 200/3.6).
        // getDefaultAccel/getDefaultDecel/getDefaultEmergencyDecel all fall through to their
        // `default:` branches (2.6 / 4.5 / vcDecel=9.0) -- SVC_DELIVERY has no case in any of
        // those three switches.
        ["delivery"] = new RawDefaults(Length: 6.5, MinGap: 2.5, MaxSpeed: 200.0 / 3.6, Accel: 2.6, Decel: 4.5, VcDecel: 9.0, Width: 2.16, Height: 2.86),

        // SUMOVehicleClass.cpp:561 length=16.5. SVC_TRAILER case: maxSpeed=130/3.6, width=2.55,
        // height=4.0. getDefaultAccel SVC_TRAILER=1.1, getDefaultDecel SVC_TRAILER=4.0,
        // getDefaultEmergencyDecel SVC_TRAILER vcDecel=7.0.
        ["trailer"] = new RawDefaults(Length: 16.5, MinGap: 2.5, MaxSpeed: 130.0 / 3.6, Accel: 1.1, Decel: 4.0, VcDecel: 7.0, Width: 2.55, Height: 4.0),

        // SUMOVehicleClass.cpp:552 length=1.6. SVC_BICYCLE case: minGap=0.5, maxSpeed=50/3.6,
        // width=0.65, height=1.7. getDefaultAccel SVC_BICYCLE=1.2, getDefaultDecel
        // SVC_BICYCLE=3.0, getDefaultEmergencyDecel SVC_BICYCLE vcDecel=7.0.
        ["bicycle"] = new RawDefaults(Length: 1.6, MinGap: 0.5, MaxSpeed: 50.0 / 3.6, Accel: 1.2, Decel: 3.0, VcDecel: 7.0, Width: 0.65, Height: 1.7),

        // SUMOVehicleClass.cpp:557 length=2.2. SVC_MOTORCYCLE case: width=0.9, height=1.5
        // (maxSpeed stays at the constructor default 200/3.6; minGap stays at 2.5).
        // getDefaultAccel SVC_MOTORCYCLE=6.0, getDefaultDecel SVC_MOTORCYCLE=10.0,
        // getDefaultEmergencyDecel SVC_MOTORCYCLE vcDecel=10.0.
        ["motorcycle"] = new RawDefaults(Length: 2.2, MinGap: 2.5, MaxSpeed: 200.0 / 3.6, Accel: 6.0, Decel: 10.0, VcDecel: 10.0, Width: 0.9, Height: 1.5),

        // SUMOVehicleClass.cpp:555 length=2.1. SVC_MOPED case: maxSpeed=60/3.6, width=0.78,
        // height=1.7 (minGap stays at 2.5). getDefaultAccel SVC_MOPED=1.1, getDefaultDecel
        // SVC_MOPED=7.0, getDefaultEmergencyDecel SVC_MOPED vcDecel=10.0.
        ["moped"] = new RawDefaults(Length: 2.1, MinGap: 2.5, MaxSpeed: 60.0 / 3.6, Accel: 1.1, Decel: 7.0, VcDecel: 10.0, Width: 0.78, Height: 1.7),

        // SUMOVehicleClass.cpp:578 length=6.5 (shared with SVC_DELIVERY). SVC_EMERGENCY case:
        // width=2.16, height=2.86 (maxSpeed stays at the constructor default 200/3.6).
        // getDefaultAccel/getDefaultDecel/getDefaultEmergencyDecel all fall through to their
        // `default:` branches (2.6 / 4.5 / vcDecel=9.0) -- SVC_EMERGENCY has no case in any of
        // those three switches.
        ["emergency"] = new RawDefaults(Length: 6.5, MinGap: 2.5, MaxSpeed: 200.0 / 3.6, Accel: 2.6, Decel: 4.5, VcDecel: 9.0, Width: 2.16, Height: 2.86),
    };

    // Generalized (rung A1) so that any attribute the rou.xml <vType> sets EXPLICITLY overrides
    // the vClass-default table above, via `override ?? default` per attribute -- e.g. rung 4's
    // leader sets maxSpeed="5.00" and both its vTypes set sigma="0". ApparentDecel is not itself
    // overridable from rou.xml in our scope; it derives from the (possibly overridden) decel,
    // matching MSCFModel.cpp:61's getCFParam(SUMO_ATTR_APPARENTDECEL, myDecel) default-to-decel
    // fallback.
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
                "bicycle, motorcycle, moped, emergency) is in scope.");
        }

        // SUMOVTypeParameter.cpp getDefaultDecel: overridable via rou.xml's decel="...".
        var decel = vType.Decel ?? raw.Decel;

        return new ResolvedVType(
            Id: vType.Id,
            VClass: vClass,
            // SUMOVTypeParameter.cpp:331 cfModel(SUMO_TAG_CF_KRAUSS) -- default CF model.
            CarFollowModel: "Krauss",
            // SUMOVehicleClass.cpp getDefaultVehicleLength; overridable via rou.xml's length="...".
            Length: vType.Length ?? raw.Length,
            // SUMOVTypeParameter.cpp:61 (or per-vclass override); overridable via rou.xml's
            // minGap="...".
            MinGap: vType.MinGap ?? raw.MinGap,
            // SUMOVTypeParameter.cpp:63 (or per-vclass override); overridable via rou.xml's
            // maxSpeed="..." (rung 4's leader sets maxSpeed="5.00" so the fast follower catches
            // up and settles into the Krauss steady-state gap).
            MaxSpeed: vType.MaxSpeed ?? raw.MaxSpeed,
            // SUMOVTypeParameter.cpp getDefaultAccel; overridable via rou.xml's accel="...".
            Accel: vType.Accel ?? raw.Accel,
            Decel: decel,
            // getDefaultEmergencyDecel default option -> MAX2(decel, vcDecel); overridable via
            // rou.xml's emergencyDecel="..." (default computed from the possibly-overridden
            // decel and the per-vclass vcDecel, matching SUMOVTypeParameter.cpp's
            // MAX2(decel, vcDecel) fallback -- never hardcoded per-class).
            EmergencyDecel: vType.EmergencyDecel ?? Math.Max(decel, raw.VcDecel),
            // MSCFModel.cpp:61 getCFParam(SUMO_ATTR_APPARENTDECEL, myDecel) -- defaults to
            // (possibly-overridden) decel; not independently overridable in our scope.
            ApparentDecel: decel,
            // SUMOVTypeParameter.cpp getDefaultImperfection: non-rail/non-ship `default:` branch
            // returns 0.5 for every vClass in this table; overridable via rou.xml's sigma="..."
            // (this scenario's rou.xml sets sigma="0" for determinism).
            Sigma: vType.Sigma ?? 0.5,
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
            Height: raw.Height);
    }
}
