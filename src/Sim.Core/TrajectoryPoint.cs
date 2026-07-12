namespace Sim.Core;

// Lane-relative (Lane, Pos) is the source of truth per DESIGN.md; X/Y are derived.
//
// Perf (PERF-ROADMAP.md Layer 0a): a `readonly record struct`, NOT a heap `record`. The engine emits
// one of these per vehicle per step; as a struct it is stored inline in TrajectorySet's backing
// List<TrajectoryPoint> (no per-point object header, no GC object per emitted point). Field set and
// value semantics are byte-identical to the former record (record struct keeps positional
// construction + value equality), so every consumer -- TrajectoryComparator, FcdParser, Sim.Viz,
// the benches -- is unchanged.
public readonly record struct TrajectoryPoint(
    string VehicleId,
    double Time,
    string Lane,
    double Pos,
    double Speed,
    double X,
    double Y,
    double Angle,
    double? Acceleration)
{
    // Phase 2 (sublane): continuous lateral offset from the lane centreline, +left of travel --
    // the same value as Kinematics.LatOffset, and SUMO's FCD `posLat` attribute. An init-only
    // property (NOT a positional member) precisely so it is ADDITIVE: every existing positional
    // construction stays valid and leaves this 0, and value-equality over lane-mode trajectories
    // (LatOffset always 0) is unchanged. The determinism hash (Sim.Bench) mixes only Pos/Speed and
    // is likewise unaffected. Compared only by scenarios that list "posLat" in comparedAttributes.
    public double PosLat { get; init; }
}
