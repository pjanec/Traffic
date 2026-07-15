# SumoSharp.Testing

Test/validation helpers for the **SumoSharp** traffic engine — the parity harness used to
check a simulation run against Eclipse SUMO reference output.

- **FCD / TripInfo / Summary parsers** for SUMO-schema XML output.
- **Trajectory and aggregate comparators** (per-attribute max-abs / RMSE, presence mismatches,
  first-divergence step).
- **Tolerance configuration** (`tolerance.json`) — per-scenario parity modes and bounds.

Use it to validate your own networks/demand against committed SUMO goldens, or to build a
regression gate around your integration. It depends only on `SumoSharp.Core`; it does **not**
require a SUMO install at runtime.

## License

Dual-licensed **`EPL-2.0 OR GPL-2.0-or-later`** (SumoSharp is a derivative work of Eclipse SUMO
and cannot be relicensed). EPL-2.0 is weak, file-level copyleft: a proprietary app may link
SumoSharp and keep its own source closed, but must keep the SUMO-derived files under EPL and
publish modifications to *those* files. This is not legal advice — get counsel for commercial use.

## Disclaimer

Unofficial, independent C# reimplementation of Eclipse SUMO's microscopic simulation core. Not
affiliated with or endorsed by the Eclipse SUMO project. "SUMO" is an Eclipse trademark.
