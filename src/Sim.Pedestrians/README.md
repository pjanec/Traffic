# SumoSharp.Pedestrians

Pedestrian crowd subsystem for the **SumoSharp** traffic engine — a reciprocal-collision-avoidance
(ORCA) crowd with sim-LOD promotion/demotion, strategic navigation over a SUMO-geometry-baked
walkable network, and a networking-ready wire-event stream. Layered over the engine's public seams
(it does **not** touch the deterministic simulation core, so it cannot move parity — see
`docs/PEDESTRIAN-DESIGN.md` §0 Principle 6).

Validated on the **live-reactivity axis** (behavioral/property tests: no overlap, arrives-within-N,
promotion/demotion never flaps, run-to-run determinism), not by golden FCD — see `docs/PEDESTRIAN-
DESIGN.md` §8.

## The production entry point: `PedestrianWorld`

`PedestrianWorld` (`PedestrianWorld.cs`) is a thin facade over the three pieces a caller would
otherwise have to hand-wire together every step: a `PedLodManager` (the sim-LOD population, promotion/
demotion, and PathArc↔FreeKinematic switching), an `InterestField` (the movable promotion sources),
and a `PedPublisher` (the wire-event stream) — plus a facade-tracked live-id set (`PedLodManager`
itself does not expose one).

```csharp
var world = new PedestrianWorld(navigation); // IPedNavigation, e.g. SumoNavMesh

world.AddWalker(id: 1, origin, destination, maxSpeed: 1.4, radius: 0.3, now: 0.0);
var camera = world.AddInterestSource(pos, promoteRadius: 3.0, demoteRadius: 6.0, InterestSourceKind.Camera);

for (var step = 0; step < steps; step++)
{
    world.MoveInterestSource(camera, cameraPos);     // if the source moves
    world.SetExternalObstacles(carDiscs);            // cars/avatars the crowd avoids this step
    world.Step(now, dt);
    now += dt;
}

var pos = world.PositionOf(1, now);
var model = world.ModelOf(1);                        // PathArc | FreeKinematic | ActivityTimeline
```

`SetForcedHighPower(id, true)` pins a ped high-power regardless of any interest source — the evac
panic control (`docs/PEDESTRIAN-DESIGN.md` §6): it promotes on the next `Step` and never demotes
while pinned.

## The seams

- **Navigation** (`Navigation/INavigation.cs`) — `IWalkableSpace` / `IPedNavigation` / `ILocalSteering`.
  Two dev providers ship behind these seams: a SUMO-geometry bake (`Navigation/Bake/SumoNavMesh.cs`,
  built from ingested `net.xml` sidewalks/crossings/walkingAreas) and DotRecast
  (`Sim.Pedestrians.Nav.DotRecast`, its own package so the dependency stays off a consumer who only
  wants the bake provider). A production navmesh implements the same interfaces and drops in
  unchanged.
- **Interest sources** (`AddInterestSource`/`MoveInterestSource`/`RemoveInterestSource`) — the sim-LOD
  promotion volumes: an avatar/camera bubble, a static area of interest, or an intrinsic source
  (crosswalk, incident). See `docs/PEDESTRIAN-DESIGN.md` §5.
- **External obstacles** (`SetExternalObstacles`) — moving discs (cars, player avatars) the
  high-power crowd reactively avoids.
- **Networking** (`Publisher`) — the in-memory wire-event stream (`PathArcRecord` / `ActivityTimelineRecord`
  / `DrSwitchEvent` / `FreeKinematicSample` / `HeartbeatEvent`). Wire it to the P3
  `PedReplicationPublisher`/`PedReplicationReceiver` (`Lod/PedReplicationPublisher.cs`,
  `Lod/PedReplicationReceiver.cs`) to carry it over a real transport.

## Advanced use

The `Lod`, `Navigation`, `Crossing`, and `Parking` namespaces remain public: reach into
`PedLodManager`, `InterestField`, or a custom `ILocalSteering` directly when the facade's shape
doesn't fit (e.g. a caller that needs the persistent high-power `OrcaCrowd` itself). `PedestrianWorld`
is the common path, not the only path.

## License

Dual-licensed **`EPL-2.0 OR GPL-2.0-or-later`** (SumoSharp is a derivative work of Eclipse SUMO and
cannot be relicensed). EPL-2.0 is weak, file-level copyleft: a proprietary app may link SumoSharp and
keep its own source closed, but must keep the SUMO-derived files under EPL and publish modifications
to *those* files. This is not legal advice — get counsel for commercial use.

## Disclaimer

Unofficial, independent C# reimplementation of Eclipse SUMO's microscopic simulation core. Not
affiliated with or endorsed by the Eclipse SUMO project. "SUMO" is an Eclipse trademark.
