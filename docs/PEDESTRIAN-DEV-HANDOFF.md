# Pedestrian dev session — handoff

**You are the continuation developer session.** You own the pedestrian subsystem **and** (now) `Sim.Core` —
the parallel Engine track finished its work and merged it to `main`, and the core is handed to us. Continue
the production build-out (DDS transport next; see below). Branch:
**`claude/pedestrian-simulation-design-d8fbme`**.

Read `CLAUDE.md` first (the rules of the road), then `docs/PEDESTRIAN-DESIGN.md` (HOW) and
`docs/PEDESTRIAN-TRACKER.md` (at-a-glance status). This file is the "where we are / what's next" pointer.

## Where we are (all green, all pushed)

- **`main` is merged into this branch** (merge commit on top of the finished P2-G/junction/teleport core
  work + ~34 new parity tests). Full gate green: **630 parity (+3 skip) / 142 ped / 2 DotRecast / 1 host**.
  Branch is even with `main` + our pedestrian commits on top.
- **Stage P7 (visualization) done:** P7-1 native in-process (`PedOverlay`), P7-2 native remote-over-the-wire
  (`RemotePedOverlay`, server==IG, 0.213 m peak render error), P7-3 Godot City3D peds
  (`PedSimSource`/`PedReconstructor`/`PedTransform` + `--peds`). A mobile showcase artifact of the three
  renders is published.
- **P4-1 done:** `Engine.TryGetTlLinkState(tlId, linkIndex, out char)` — a parity-inert read-only projection
  of the live crossing signal; ped side reads it via `LiveCrossingSignal` + `CrossingSignalFactory.
  ForCrossingLive` (delegate seam, so `Sim.Pedestrians` still doesn't reference the Engine). Opt-in for
  Engine-coupled integrations.
- Everything through P0–P6-3 (except the two blocked items below) is complete — see the tracker.

## Two invariants you must keep

1. **The offline gate is the law.** `dotnet test Traffic.sln -c Release` must stay
   **630 (+3 skip) / 142 / 2 / 1** green (numbers grow as you add tests — the parity 630 must never *drop*
   or change hash). No SUMO in that loop.
2. **Parity hash `909605E965BFFE59` is frozen.** You now own `Sim.Core`, so you *can* edit `Engine.cs` — but
   any change must be parity-inert unless it's a deliberate, golden-regenerated behavior change. The P4-1
   projection is the template: read-only, off the golden path, never called from `Step()`.

## Next up — recommended: **DDS transport for pedestrians** (design-first)

The one big, genuinely-unblocked piece. Today P7-2 (native remote) and P7-3 (Godot remote) prove
server==IG over the **in-process byte loopback** (`InMemoryPedReplicationBus`). The **live CycloneDDS
binding for the ped wire does not exist** — `src/Sim.Replication.Dds` carries only vehicle/TL/command
topics. Building it:
- completes **Requirement 7** (one server → many IG over DDS multicast) for pedestrians end-to-end;
- lights up the real "remote (DDS)" path in the native viewer (`--mode remote`) and Godot City3D
  (`--transport=dds`), both of which are wired for vehicles and stubbed/deferred for peds;
- unblocks the *live-DDS* half of **P6-1** success condition 3 (the Windows session is measuring the
  single-stream codec bytes/sec now; the live-transport confirmation waits on this).

Mirror the **vehicle** DDS binding (`src/Sim.Replication.Dds`: `DdsPublisher`/`DdsSubscriber`,
`StructuredTopics`) for the ped wire (`IPedReplicationSink`/`Source`, the `CrowdFrame`/`PathArc`/
`ActivityTimeline`/`Lifecycle` topics `Sim.Replication/PedReplication.cs` defines). **Design first** per
`CLAUDE.md` (a short HOW doc + a task breakdown with success conditions, then implement) — it's a new
transport feature, not a within-pattern tweak. Keep it out of `Traffic.sln` / the parity gate exactly as the
vehicle DDS binding is.

### Other directions (if you'd rather)
- **Wire the P4-1 live signal into a real car+ped coupling** — no consumer needs it *today* (the crossing
  gate is only used in tests/demos), so this is speculative until a mixed car+ped integration exists.
- **Blocked, don't attempt solo:** P6-1 (on-target perf — the Windows session is on it), P6-2 (region
  decomposition — *only if* the Windows session's thread sweep shows a flat parallel plateau), P8 +
  LIVE-POC-4 (subarea — still needs a real cropped sub-area net from SumoData; `main` did not bring it).

## Coordination — a Windows perf session shares this branch

A **dedicated Windows perf-test session** is running P6-1 on a 16+‑core box on **this same branch**
(`docs/PEDESTRIAN-P6-1-WINDOWS-PERF-INSTRUCTIONS.md` is its brief). To stay out of each other's way:
- **It only edits the perf docs**: `PEDESTRIAN-POC7A/B/C-FINDINGS.md`, a new `PEDESTRIAN-P6-1-RESULTS.md`,
  and the P6-1/P6-2 lines of `PEDESTRIAN-TRACKER.md`. **You stay off those files** so merges are trivial.
- Both sessions push here, so **`git fetch && git rebase origin/<branch>` before you push**. If you edit the
  tracker, avoid the P6-1/P6-2 lines it owns.
- It will report P6-1 results back (committed `PEDESTRIAN-P6-1-RESULTS.md` + a summary). When those land,
  fold the verdict into planning (e.g. whether P6-2 is triggered).

## Fast-ramp pointers

- **Run the gate:** `dotnet test Traffic.sln -c Release`. **Build:** `dotnet build Traffic.sln -c Release`.
- **Native viewer demos** (out of `Traffic.sln`): `dotnet run --project src/Sim.Viewer -c Release -- --mode
  local --demo "Pedestrian remote (over the wire)" --screenshot out.png --frames 700 --sim-rate 90` (Xvfb
  headless). Demos: "Pedestrian evac (district)", "Pedestrian remote (over the wire)".
- **Godot City3D peds:** `demos/City3D/screenshot.sh out.png --peds --shot-delay=10` (Xvfb + llvmpipe;
  Godot at `/opt/godot/...`). CityLib is package-fed — run `demos/City3D/build.sh --pack-only` after changing
  `src/` libs it consumes.
- **HTML demo gallery builder:** `scripts/build-ped-artifact.py` (mobile-watchable). The render-showcase
  builder lives in the session scratchpad, not the repo.
- **Ped replication seam:** `src/Sim.Replication/PedReplication.cs` (topics + `InMemoryPedReplicationBus`),
  `src/Sim.Pedestrians/Lod/PedReplicationPublisher.cs` / `PedRemoteReconstructor.cs`. **Vehicle DDS binding
  to mirror:** `src/Sim.Replication.Dds/`.
- **Subagents:** delegate volume/mechanical work to Sonnet implementors; keep the accept/reject parity gate
  on Opus (`CLAUDE.md` "The orchestration loop"). Verify every "done" first-hand — re-read the diff, re-run
  the gate — before ticking the tracker.
