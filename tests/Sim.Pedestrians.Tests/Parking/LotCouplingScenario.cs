using Sim.Core.Orca;
using Sim.Pedestrians.Parking;

namespace Sim.Pedestrians.Tests.Parking;

// POC-6b (docs/PEDESTRIAN-POC-PLAN.md POC-6, success condition 3): shared geometry for the LotCoupling
// tests. A single maneuvering car drives straight across an open lot (0,0) -> (40,0); a row of three
// parked-car boxes sits above the car's lane (y in [4.1,5.9]) with two 1.7 m gaps a pedestrian must
// weave through; three pedestrians cross the car's lane top-to-bottom, two of them routed straight
// through a row gap first. Mirrors ParkingScenarioFixture's role for POC-6a (docs/PEDESTRIAN-POC-PLAN.md
// POC-6, conditions 1/2) -- shared constants + a builder, reused by the no-overlap, car-yields, and
// determinism tests.
internal static class LotCouplingScenario
{
    public const double PedRadius = 0.3;
    public const double PedMaxSpeed = 1.4;
    public const double CarMaxSpeed = 3.0;
    // Finer dt than the usual 1 Hz engine tick: the coupling exchanges FROZEN start-of-step snapshots in
    // both directions (LotCoupling's class remarks), so at 3 m/s car / 1.4 m/s ped, a 0.2 s step lets
    // either side close up to ~0.3-0.6 m before the next rebuild reacts -- measured to bite into the
    // ped-radius margin almost exactly. 0.05 s quarters that one-step displacement and restores a
    // comfortable no-overlap margin without changing the coupling's algorithm.
    public const double Dt = 0.05;
    public const int MaxSteps = 1000;   // 50 s budget

    public static readonly Vec2 CarStart = new(0.0, 0.0);
    public static readonly Vec2 CarGoal = new(40.0, 0.0);

    // Row of parked-car boxes (car-sized: 4.3 x 1.8 m), centred at y=5, gaps at x in [10.15,11.85] and
    // [16.15,17.85].
    public static readonly (Vec2 Center, double HalfLength, double HalfWidth)[] ParkedBoxes =
    {
        (new Vec2(8.0, 5.0), 2.15, 0.9),
        (new Vec2(14.0, 5.0), 2.15, 0.9),
        (new Vec2(20.0, 5.0), 2.15, 0.9),
    };

    // Pedestrian start/goal pairs: peds 0 and 1 cross straight down through the two row gaps (weaving
    // between the parked boxes); ped 2 crosses the car's lane directly, away from the row.
    public static readonly (Vec2 Start, Vec2 Goal)[] PedRoutes =
    {
        (new Vec2(11.0, 9.0), new Vec2(11.0, -9.0)),
        (new Vec2(17.0, 9.0), new Vec2(17.0, -9.0)),
        (new Vec2(25.0, 9.0), new Vec2(25.0, -9.0)),
    };

    // Builds the coupling with the car + all three parked boxes but NO pedestrians -- the caller adds
    // pedestrians (or not, for a no-peds baseline).
    public static LotCoupling NewCouplingWithCarAndBoxes(out int carId)
    {
        var coupling = new LotCoupling();
        foreach (var (center, halfLength, halfWidth) in ParkedBoxes)
        {
            coupling.AddParkedCarBox(center, halfLength, halfWidth);
        }

        carId = coupling.AddCar(CarStart, CarGoal, CarMaxSpeed);
        return coupling;
    }

    // The full condition-3 scenario: car + parked boxes + all three crossing pedestrians.
    public static LotCoupling NewFullScenario(out int carId, out OrcaHandle[] pedIds)
    {
        var coupling = NewCouplingWithCarAndBoxes(out carId);
        pedIds = new OrcaHandle[PedRoutes.Length];
        for (var i = 0; i < PedRoutes.Length; i++)
        {
            var (start, goal) = PedRoutes[i];
            pedIds[i] = coupling.AddPedestrian(start, PedRadius, PedMaxSpeed, goal);
        }

        return coupling;
    }
}
