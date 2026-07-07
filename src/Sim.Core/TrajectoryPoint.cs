namespace Sim.Core;

// Lane-relative (Lane, Pos) is the source of truth per DESIGN.md; X/Y are derived.
public sealed record TrajectoryPoint(
    string VehicleId,
    double Time,
    string Lane,
    double Pos,
    double Speed,
    double X,
    double Y,
    double Angle,
    double? Acceleration);
