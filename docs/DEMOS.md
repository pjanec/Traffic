# Interactive demo gallery

An auto-generated gallery of interactive, self-contained traffic-simulation replays. Each demo is a
single `replay.html`-style page (vanilla Canvas 2D — width-accurate lanes, junction fills,
SUMO-native signal heads, true-size oriented vehicle boxes, play/pause/scrub/speed/zoom/pan). No
install, no server, no SUMO: it runs entirely in the browser once the page is loaded.

<!-- Pages base URL: update ONLY if the repo is not named "SumoSharp". Everything else is relative. -->
https://pjanec.github.io/SumoSharp/

## Demos

| Demo | Shows |
|---|---|
| Single free-flow | A single vehicle cruising free-flow on an open road — the simplest parity scenario. |
| Traffic light | Vehicles queuing and releasing at a signalized intersection with SUMO-native signal heads. |
| Priority junction | Right-of-way negotiation at an unsignalized priority junction. |
| External agents showcase | Five external (non-SUMO) agent reactions — stop, swerve, spill, follow, junction-yield — injected alongside engine traffic. |
| Evacuation (organic town) | A realistic organic town under panic evacuation: congestion plus a large local foot exodus. |
| City-30 (scaled town) | A 3x3-grid town at ~30 concurrent vehicles — engine run rendered against the SUMO aggregate-parity reference. |

The curated set is defined in `scripts/gen-demos.sh`; only demos that actually generate are listed
on the gallery's landing page (a broken demo is skipped and logged, never faked).

## Run the demos locally

```bash
scripts/gen-demos.sh
open site/index.html      # or just double-click it in a file browser
```

Windows users: run it via WSL or Git Bash (the script is bash-only; there is no `.ps1` twin).

`site/` is git-ignored — it is always regenerated, never committed.

## Maintainer setup

1. **Settings → Pages → Source: "GitHub Actions"** (one-time, per repo).
2. The gallery deploys automatically from `main` whenever `scenarios/**`, `src/Sim.Viz/**`,
   `src/Sim.Run/**`, `src/Sim.ExtDemo/**`, or `scripts/gen-demos.sh` change (see
   `.github/workflows/demos.yml`), or on demand via **Actions → demos → Run workflow**.
3. Feature branches never publish: the push trigger is restricted to `main`.

## Not part of the web gallery

The native desktop viewer (raylib + Dear ImGui, 10k-scale) is a separate desktop application, not
a browser page — see `docs/PACKAGES.md` and the "Live & native viewers" section of the README.
