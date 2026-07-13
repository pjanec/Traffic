namespace Sim.Core.Mixed;

// The heterogeneous fleet of a believable Indian street (docs/INDIA-TRAFFIC.md). Each class carries
// the two things the mixed-traffic model needs beyond a plain disc:
//   * a SHAPE (ConvexShape prototype in the local frame) so avoidance is anisotropic -- a bus blocks
//     a long swath broadside, a motorcycle threads gaps;
//   * an ASSERTIVENESS scalar so priority is soft and emergent -- bigger/faster/more-committed
//     vehicles get de-facto right of way without any hard rule (see MixedTrafficCrowd's per-pair
//     responsibility split).
// Dimensions are in metres, speeds in m/s -- illustrative but realistic ratios (a bus is ~12 m and
// slow to assert-through, a motorcycle ~1.8 m and nimble).
public enum VehicleKind
{
    Bus,
    Car,
    AutoRickshaw,   // three-wheeled "tuk-tuk"
    Motorcycle,
}

public sealed record VehicleClass(
    VehicleKind Kind,
    double Length,
    double Width,
    double MaxSpeed,
    double Assertiveness,
    int VizShape,          // 0 = rectangle, 1 = hexagon (matches template.js drawShaped)
    int VizColorKind,      // disc "kind" colour index used by the viz palette
    double MinTurnRadius,  // m -- car-like steering limit; heading can change at most speed*dt/R per step
    double MaxAccel,       // m/s^2 -- longitudinal acceleration cap
    double MaxDecel,       // m/s^2 -- braking cap (larger: vehicles stop faster than they start)
    double Wheelbase)      // m -- front-to-rear axle distance; the body pivots about the REAR axle, so
                           // a long vehicle's rear tracks inside the turn while the front sweeps wide
                           // (real off-tracking, not a body-centre spin)
{
    // The footprint prototype in the LOCAL frame (+X forward). Motorcycles use a hexagon (compact,
    // near-round, still filters); everything else is an oriented box. Buses/cars could use a capsule
    // for rounded ends, but the rectangle reads correctly in the viz and keeps corners honest.
    public ConvexShape Shape() => Kind == VehicleKind.Motorcycle
        ? ConvexShape.RegularPolygon(6, Math.Max(Length, Width) * 0.5)
        : ConvexShape.Rectangle(Length, Width);

    public static readonly VehicleClass Bus =
        new(VehicleKind.Bus, 11.0, 2.5, 11.0, 4.0, VizShape: 0, VizColorKind: 3,
            MinTurnRadius: 11.0, MaxAccel: 1.0, MaxDecel: 3.0, Wheelbase: 6.0);

    public static readonly VehicleClass Car =
        new(VehicleKind.Car, 4.3, 1.8, 14.0, 2.0, VizShape: 0, VizColorKind: 0,
            MinTurnRadius: 5.0, MaxAccel: 2.5, MaxDecel: 4.5, Wheelbase: 2.6);

    public static readonly VehicleClass AutoRickshaw =
        new(VehicleKind.AutoRickshaw, 2.6, 1.4, 9.0, 1.2, VizShape: 0, VizColorKind: 2,
            MinTurnRadius: 3.0, MaxAccel: 2.2, MaxDecel: 4.0, Wheelbase: 1.9);

    public static readonly VehicleClass Motorcycle =
        new(VehicleKind.Motorcycle, 1.9, 0.8, 16.0, 0.8, VizShape: 1, VizColorKind: 1,
            MinTurnRadius: 1.5, MaxAccel: 3.5, MaxDecel: 5.0, Wheelbase: 1.3);
}
