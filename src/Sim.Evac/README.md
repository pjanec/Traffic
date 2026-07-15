# SumoSharp.Evac

Panic-evacuation subsystem for the **SumoSharp** traffic engine — an optional domain extension
layered over the engine's public seams (it does **not** touch the deterministic simulation core,
so it cannot move parity).

Models a localized incident triggering fear/panic, congestion and jamming, drivers abandoning
their vehicles, and a foot exodus (pedestrians as obstacles over a lightweight navmesh). Built on
`SumoSharp.Core`'s frozen seams (dead-reckoning model, vehicle params, destinations, despawn,
obstacle/crowd sources) — the driving core never references it.

Install this only if you need evacuation/crowd scenarios; a plain traffic simulation does not
need it.

## License

Dual-licensed **`EPL-2.0 OR GPL-2.0-or-later`** (SumoSharp is a derivative work of Eclipse SUMO
and cannot be relicensed). EPL-2.0 is weak, file-level copyleft: a proprietary app may link
SumoSharp and keep its own source closed, but must keep the SUMO-derived files under EPL and
publish modifications to *those* files. This is not legal advice — get counsel for commercial use.

## Disclaimer

Unofficial, independent C# reimplementation of Eclipse SUMO's microscopic simulation core. Not
affiliated with or endorsed by the Eclipse SUMO project. "SUMO" is an Eclipse trademark.
