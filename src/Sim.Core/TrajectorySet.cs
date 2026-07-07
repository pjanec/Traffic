namespace Sim.Core;

public sealed class TrajectorySet
{
    private readonly Dictionary<string, SortedDictionary<double, TrajectoryPoint>> _byVehicle = new();

    public IReadOnlyCollection<string> VehicleIds => _byVehicle.Keys;

    public void Add(TrajectoryPoint point)
    {
        if (!_byVehicle.TryGetValue(point.VehicleId, out var byTime))
        {
            byTime = new SortedDictionary<double, TrajectoryPoint>();
            _byVehicle[point.VehicleId] = byTime;
        }

        byTime[point.Time] = point;
    }

    public IReadOnlyDictionary<double, TrajectoryPoint> PointsFor(string vehicleId) =>
        _byVehicle.TryGetValue(vehicleId, out var byTime) ? byTime : EmptyTimePoints;

    public bool TryGet(string vehicleId, double time, out TrajectoryPoint point)
    {
        if (_byVehicle.TryGetValue(vehicleId, out var byTime) && byTime.TryGetValue(time, out var found))
        {
            point = found;
            return true;
        }

        point = null!;
        return false;
    }

    public IEnumerable<TrajectoryPoint> AllPoints => _byVehicle.Values.SelectMany(byTime => byTime.Values);

    private static readonly SortedDictionary<double, TrajectoryPoint> EmptyTimePoints = new();
}
