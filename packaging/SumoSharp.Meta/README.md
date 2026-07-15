# SumoSharp

Meta-package for **SumoSharp** — a convenience bundle that installs the simulate-and-stream
core in one shot:

- **`SumoSharp.Core`** — the microsimulation engine (load a SUMO network, spawn/route vehicles
  at runtime, step deterministically or run async, read vehicle state).
- **`SumoSharp.Ingest`** — parsers and data model for SUMO network (`.net.xml`), demand
  (`.rou.xml`), and config (`.sumocfg`) files.
- **`SumoSharp.Replication`** — transport-agnostic dead-reckoning replication (compact
  handle-based records, a packed blob codec, an adaptive publish policy) for streaming
  simulation state to a client/game/viewer.

This package carries no code of its own — it is a pure dependency bundle. Other SumoSharp
packages (`SumoSharp.Testing`, `SumoSharp.Evac`, `SumoSharp.Viewer.Motion`, ...) are opt-in and
installed separately; in particular the Unity/Godot-facing `SumoSharp.Viewer.Motion` package is
**not** pulled in by this bundle, to keep it un-opinionated about rendering/engine choice.

## License

Dual-licensed **`EPL-2.0 OR GPL-2.0-or-later`** (SumoSharp is a derivative work of Eclipse SUMO
and cannot be relicensed). EPL-2.0 is weak, file-level copyleft: a proprietary app may link
SumoSharp and keep its own source closed, but must keep the SUMO-derived files under EPL and
publish modifications to *those* files. This is not legal advice — get counsel for commercial use.

## Disclaimer

Unofficial, independent C# reimplementation of Eclipse SUMO's microscopic simulation core. Not
affiliated with or endorsed by the Eclipse SUMO project. "SUMO" is an Eclipse trademark.
