# City3D demo — integration handoff (for the main session merging into `main`)

**Branch:** `claude/3d-city-demo-design-conmq6`. **Scope:** a Godot 4 (.NET) 3D city viewer that consumes
the `SumoSharp.*` packages from a local feed (never nuget.org, never a `ProjectReference` into `src/`),
proving the packaged-consumer experience and the render-side dead-reckoning story — **local** (co-hosted)
and **remote** (decoupled host → viewer over DDS), plus a CLI-driven off-axis video wall. Full design in
`docs/DEMO-CITY3D-{DESIGN,TASKS,TRACKER}.md`; user-facing run docs in `demos/City3D/README.md`.

**Bottom line for integration:** every engine-side change is **additive** (new projects) or **render-side
DRY refactor** (moving already-existing code between non-parity projects, wire bytes unchanged). The
offline parity gate and determinism hash are **untouched** end to end:

- `dotnet test Traffic.sln -c Release` → **465 passed / 0 failed / 3 skipped** (same as pre-branch baseline;
  the 3 skips are the pre-existing multilane/sublane rungs).
- `dotnet run --project src/Sim.Bench -c Release` → `hashA=909605E965BFFE59`, `hashPar=909605E965BFFE59`
  (single == parallel), unchanged.
- The native DDS loopback proof still passes:
  `dotnet run --project src/Sim.Viewer -c Release -- --selftest scenarios/09-traffic-light` → `SELFTEST: PASS`.

Review the branch with `git log --stat main..claude/3d-city-demo-design-conmq6`.

---

## What changed, by risk tier

### Tier A — additive `src/` (new projects, no existing behaviour touched)
- **`src/Sim.Host/`** — `SumoSharp.Host`, a NEW packable library (`net8.0;netstandard2.1`,
  `IsPackable=true`). `ReplicationPublisher`: transport-neutral `SimulationSnapshot`+`NetworkModel` →
  `IReplicationSink` (geometry once, per-step vehicle records + TL, gated by the existing
  `PublishScheduler`). It is the reusable extraction of the snapshot→wire logic that previously existed
  only inside the DDS-hardwired viewer publisher. **Added to `Traffic.sln`** (so it's built + its tests run
  in the gate).
- **`tests/Sim.Host.Tests/`** — xUnit for `ReplicationPublisher` (added to `Traffic.sln`; runs in the gate,
  1 test, green).
- **`src/Sim.Host.App/`** — a generic headless "publish any `--scenario` over DDS|inmem" console tool
  (`IsPackable=false`), sibling to `Sim.LiveHost`. **NOT in `Traffic.sln`** because it references the native
  `Sim.Replication.Dds` (which itself isn't in the solution — same reason `Sim.Viewer` isn't).

### Tier B — render-side DRY refactors (parity-neutral; these projects are NOT in `Traffic.sln`)
These move code that already existed; **wire bytes / topics / QoS are unchanged**, proven by the DDS
loopback self-test above.
- **DDS sink** extracted to `src/Sim.Replication.Dds/DdsReplicationSink.cs` (was inline in
  `Sim.Viewer.Core/DdsPublisher.cs`). `DdsPublisher` is now a 46-line delegator over `ReplicationPublisher`
  + `DdsReplicationSink`; it no longer implements `IReplicationSink` (no external caller used it that way).
  `Sim.Viewer.Core.csproj` gained a `ProjectReference` → `Sim.Host`.
- **DDS source** moved `src/Sim.Viewer.Raylib/DdsSubscriber.cs` → `src/Sim.Replication.Dds/DdsSubscriber.cs`
  (namespace `Sim.Viewer.Raylib` → `Sim.Replication.Dds`; it had no raylib coupling). Callers updated with a
  `using` only: `Sim.Viewer.Raylib/RenderHelpers.cs`, `Sim.Viewer/{Program,LoopbackSelfTest}.cs`.
  **Net effect:** `SumoSharp.Replication.Dds` now owns BOTH sides of the DDS binding (sink + source), which
  is what the demo (and any consumer) needs to talk DDS without pulling the raylib viewer.

### Tier C — new, fully isolated demo tree (touches nothing else)
- **`demos/City3D/`** — the Godot demo: `CityLib/` (pure `net8.0` classlib, Godot-free, consumes
  `SumoSharp.*` from the local feed), `CityLib.Tests/` (41 xUnit tests), `Viewer/` (the Godot 4 app),
  and scripts (`build.sh`, `fetch-godot.sh`, `run-local.sh`, `run-remote.sh`, `run-smoke.sh`,
  `screenshot.sh`, `perf-ladder.sh`), `nuget.config`, `README.md`, `.gitignore`. **None of it is in
  `Traffic.sln`** — it is a package consumer, deliberately out of the parity solution.
- **`docs/DEMO-CITY3D-{DESIGN,TASKS,TRACKER,HANDOFF}.md`** — design-first docs.

---

## Decisions left to the main session (integration-time judgment)

1. **Promote `SumoSharp.Host` to a published package?** Right now it is packable and consumed locally by
   the demo, but deliberately **NOT** added to `pack-check.yml`'s "assert all 9 packages" list or to
   `publish.yml`. If you want it published (it's genuinely reusable — it's the only way a package consumer
   can drive replication from a running engine): add it to both workflows and add a row to
   `docs/PACKAGES.md`. If not, leave as-is (the demo still works from the local feed).

2. **`Sim.Replication.Dds` now contains `DdsSubscriber` + `DdsReplicationSink`.** If any external consumer
   referenced `Sim.Viewer.Raylib.DdsSubscriber` by its old namespace, that's a (pre-publish) breaking
   rename to `Sim.Replication.Dds.DdsSubscriber`. Nothing is published yet, so this is free to take now.

3. **Curated-doc pointers** (I intentionally did NOT edit the shared curated docs, to avoid conflicting
   with concurrent sessions — tenet 7). Suggested additions at integration:
   - `docs/PACKAGES.md`, "Examples & samples" table: the `SumoSharp.Viewer.Motion` row currently says
     "*no standalone sample yet*" — point it at `demos/City3D` (which exercises it as a real package
     consumer). Optionally add a `SumoSharp.Host` row if promoted (decision 1).
   - `docs/DEMOS.md` and the root `README.md` (visual-demos section): a one-line pointer to
     `demos/City3D/` ("a Godot 4 3D city viewer consuming the packages; local + remote/DDS + video wall").

---

## Known findings / limitations (documented, not bugs)
- **Wire geometry is 2-D:** `GeometryCodec.LaneGeo` carries no `Z`. Local mode gets elevation from the
  Z-aware `NetworkLaneSource`; a **remote** viewer renders flat until a small future `GeometryCodec`
  Z-extension. (No committed scenario carries Z today anyway — `_bench/city-organic-L2` "L2" = 2 *lanes*,
  not 2 *levels*.) This is the one obvious follow-up if remote 3D elevation matters.
- **Remote buildings skipped:** buildings need per-edge data the wire doesn't carry; the remote viewer
  renders roads + cars + signals and skips buildings (logged honestly). Local renders the full city.
- **`DrClock` is wall-clock-driven:** a fast headless loop shows near-frozen render motion (screenshots are
  stills; motion *fidelity* is proven in `CityLib.Tests`' real-time-sleep tests). Real-time desktop = smooth.
- **Video wall** is CLI-configured off-axis frusta composited as SubViewports (one process, screenshottable,
  = a single wide-desktop wall). Real one-`Window`-per-physical-monitor is a desktop-only variant, not built.

## Committed vs ephemeral (nothing heavy in git)
- **Committed:** all `.cs`, `.tscn`, `project.godot`, scripts, `nuget.config`, docs.
- **Git-ignored / ephemeral (regenerated by tooling):** `demos/City3D/local-nuget/` (the `dotnet pack`
  feed), `.godot/`, `bin/`, `obj/`, the Godot engine binary (fetched to `/opt/godot` by `fetch-godot.sh`
  from `downloads.godotengine.org` — a non-GitHub mirror; the repo-scoped GitHub token 403s on GitHub
  release assets), and all screenshots.

## How to verify after merge
```bash
# Offline parity gate (the iron law) — must be unchanged:
dotnet test Traffic.sln -c Release            # 465 passed / 0 failed / 3 skipped
dotnet run --project src/Sim.Bench -c Release # hashA=hashPar=909605E965BFFE59
dotnet run --project src/Sim.Viewer -c Release -- --selftest scenarios/09-traffic-light  # SELFTEST: PASS

# The demo (headless; needs network for the Godot binary + nuget):
demos/City3D/build.sh                          # feed packs; SumoSharp.* resolves from it
dotnet test demos/City3D/CityLib.Tests -c Release   # 41/41
demos/City3D/run-smoke.sh                      # Godot headless scene smoke
demos/City3D/run-remote.sh                     # two-process DDS host -> viewer round-trip
```

## Not done (by design — needs a human on a GPU desktop)
The interactive/aesthetic look, smooth-motion feel at 60 fps, and a real multi-monitor wall. Launchers:
`run-local.sh` (opens a window when `DISPLAY` is set) and `run-remote.sh`. Everything else is proven
headless in-repo.

## Rebase note
All engine-side changes are additive or render-side; a concurrent engine bug-fix on `main` should merge
cleanly. The files most likely to see concurrent edits are `src/Sim.Viewer.Core/DdsPublisher.cs` and
`src/Sim.Replication.Dds/*` (rare). Rebase onto latest `main`, re-run the gate + `--selftest`, and you're
done.
