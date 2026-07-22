# `_ig/` — IgBridge visualization test beds (NOT parity scenarios)

These scenarios exist only to exercise the IgBridge motion reconstruction / viewer on
non-grid geometry. They have **no goldens and no `tolerance.json`** and are never touched by
`dotnet test` (the offline parity loop). Point the host at one with `IGBRIDGE_SCENARIO=_ig/<name>`.

- `roundabout/` — the 4-node diamond ring from parity scenario `32-roundabout`, re-fed with 60
  inline-route circulating vehicles (the original parity demand uses *named* routes the IgBridge
  RouteDemand parser skips, and only 2 vehicles). Topological roundabout / 4-junction loop.
- `round12/` — a hand-built 12-gon (near-circular) roundabout (netconvert from `nodes/edges`,
  `--roundabouts.guess`), 4 arms (E/N/W/S), 64 circulating vehicles. Sustained ring curvature.
- `organic/` — a random `netgenerate --rand` net: 112 irregular junctions, 154 edges, 164
  vehicles (randomTrips + duarouter, fringe-to-fringe). "Weird junction shapes" stress test.

Nets/demand generated with SUMO 1.18 (`netgenerate`/`netconvert`/`duarouter`/`randomTrips.py`).
Version pinning (`SUMO_VERSION`) governs parity goldens, which these do not have, so the tool
version is irrelevant here — they are render inputs only.
