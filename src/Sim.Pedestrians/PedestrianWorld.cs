using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation;

namespace Sim.Pedestrians;

/// <summary>
/// The production entry point for <c>Sim.Pedestrians</c> (P1-2, docs/PEDESTRIAN-DESIGN.md §10;
/// docs/PEDESTRIAN-TASKS.md P1-2). A thin facade that owns the pieces a caller would otherwise have to
/// hand-wire together every time: one <see cref="PedLodManager"/> (sim-LOD population + promotion/
/// demotion + PathArc/FreeKinematic switching), one <see cref="InterestField"/> (the movable promotion
/// sources), one <see cref="PedPublisher"/> (the wire-event stream), and the facade-tracked live-id set
/// (<see cref="PedLodManager"/> itself does not expose one). A consumer drives a whole pedestrian
/// population through this one type and never needs to reach for <see cref="PedLodManager.Step"/>'s
/// <c>InterestField</c>/<c>externalEntities</c> arguments directly -- <see cref="Step"/> supplies them
/// from the state this facade already owns.
///
/// The <c>Lod</c>, <c>Navigation</c>, <c>Crossing</c>, and <c>Parking</c> namespaces remain public and
/// available for advanced use (a caller that needs, say, direct access to the persistent high-power
/// crowd, or a custom <see cref="ILocalSteering"/>) -- this facade is the common path, not the only path.
/// </summary>
public sealed class PedestrianWorld
{
    private readonly PedLodManager _lod;
    private readonly InterestField _field;
    private readonly PedPublisher _publisher;
    private readonly IPedNavigation _navigation;

    private readonly HashSet<int> _liveIds = new();
    private IReadOnlyList<WorldDisc> _obstacles = Array.Empty<WorldDisc>();

    public PedestrianWorld(IPedNavigation navigation, double arriveRadius = 0.3, double dwellSeconds = 1.0)
    {
        _navigation = navigation ?? throw new ArgumentNullException(nameof(navigation));
        _publisher = new PedPublisher();
        _field = new InterestField();
        _lod = new PedLodManager(_navigation, _publisher, arriveRadius, dwellSeconds);
    }

    // ---- population ------------------------------------------------------------------------------

    /// <summary>
    /// Routes <paramref name="origin"/> -&gt; <paramref name="destination"/> via the navigation
    /// provider and, on success, registers the ped as a low-power (PathArc) walker. Returns
    /// <c>false</c> (without registering anything) when the destination is unroutable -- this never
    /// throws, mirroring <see cref="IPedNavigation.FindPath"/>'s own "null when unreachable" contract.
    /// </summary>
    public bool AddWalker(int id, Vec2 origin, Vec2 destination, double maxSpeed, double radius, double now)
    {
        var path = _navigation.FindPath(origin, destination);
        if (path is null)
        {
            return false;
        }

        _lod.AddPed(id, path, maxSpeed, radius, now);
        _liveIds.Add(id);
        return true;
    }

    /// <summary>
    /// Registers a lively low-power ped (LIVE-PROD-1a) whose pose/animation comes from
    /// <paramref name="timeline"/> (<see cref="ActivityTimeline.PoseAt"/>) rather than a bare path leg.
    /// </summary>
    public void AddLivelyWalker(int id, ActivityTimeline timeline, double maxSpeed, double radius, double now)
    {
        _lod.AddPedLively(id, timeline, maxSpeed, radius, now);
        _liveIds.Add(id);
    }

    /// <summary>Removes a ped entirely (arrived at its OD destination, or otherwise despawned).</summary>
    public void Remove(int id)
    {
        _lod.RemovePed(id);
        _liveIds.Remove(id);
    }

    /// <summary>
    /// Pins/unpins a ped high-power (evac panic, docs/PEDESTRIAN-DESIGN.md §6): while pinned it
    /// promotes on the next <see cref="Step"/> regardless of any interest source and never demotes;
    /// unpinning lets it demote normally once it is outside every demote radius.
    /// </summary>
    public void SetForcedHighPower(int id, bool on) => _lod.SetForcedHighPower(id, on);

    // ---- movable interest sources (area-of-interest that drives promotion) ------------------------

    public InterestSourceId AddInterestSource(
        Vec2 pos,
        double promoteRadius,
        double demoteRadius,
        InterestSourceKind kind = InterestSourceKind.StaticAoI) =>
        _field.Register(new InterestSource(pos, promoteRadius, demoteRadius), kind);

    public void MoveInterestSource(InterestSourceId src, Vec2 pos) => _field.Move(src, pos);

    public void RemoveInterestSource(InterestSourceId src) => _field.Remove(src);

    // ---- external obstacles (cars, avatars) the high-power crowd avoids -----------------------------

    /// <summary>
    /// Sets the external-entity discs (cars, player avatars, ...) the high-power crowd avoids for the
    /// next <see cref="Step"/>. The facade holds this list internally and passes it to
    /// <see cref="PedLodManager.Step"/> so a caller never threads it through the tick call itself.
    /// </summary>
    public void SetExternalObstacles(IReadOnlyList<WorldDisc> discs) =>
        _obstacles = discs ?? Array.Empty<WorldDisc>();

    // ---- tick + query ------------------------------------------------------------------------------

    public void Step(double now, double dt) => _lod.Step(now, dt, _field, _obstacles);

    public Vec2 PositionOf(int id, double now) => _lod.PositionOf(id, now);

    public PedDrModel ModelOf(int id) => _lod.ModelOf(id);

    public int HighPowerCount => _lod.HighPowerCount;

    /// <summary>The set of ids currently registered (added and not yet <see cref="Remove"/>-d).</summary>
    public IReadOnlyCollection<int> LiveIds => _liveIds;

    /// <summary>The wire-event stream: wire this to <c>PedReplicationPublisher</c> (P3) for networking.</summary>
    public PedPublisher Publisher => _publisher;
}
