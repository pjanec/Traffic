# PEDESTRIAN-DDS-TRANSPORT-TASKS.md — stages, tasks, success conditions

Task breakdown for the live CycloneDDS ped wire. HOW: `PEDESTRIAN-DDS-TRANSPORT-DESIGN.md` (referenced by
section, not restated). A box is ticked only when its **success conditions** pass, verified first-hand
(read the diff, build/run the proof) per `CLAUDE.md`.

Legend: `[ ]` not started · `[~]` in progress · `[x]` done · `[!]` blocked/needs-decision

## At-a-glance tracker

- [x] **D1** — DDS ped topics + names + QoS + `DdsPedReplicationSink`/`Source` binding
      *(`PedTopics.cs` + 4 topic names + sink/source; `dotnet build src/Sim.Replication.Dds -c Release` green;
      gate untouched)*
- [x] **D2** — Live DDS loopback proof: `server == IG` over real CycloneDDS (+ bytes/sec readout)
      *(`src/Sim.PedDdsLoopback`, out of `Traffic.sln`: PathArc + ActivityTimeline reconstruct exactly
      (0/801 mismatches), promoted FreeKinematic error 2.67e-7 m ≤ 0.02 m, promotion observed over the wire;
      live single-stream bandwidth readout emitted; PASS/exit 0)*
- [x] **D3** — wire the live DDS ped path into the viewers (native + Godot, both two-process verified):
  - [x] **D3a** — native viewer: `RemotePedOverlay` transport-pluggable (InMemory | `Dds`); new demo
        "Pedestrian remote (DDS multicast)" reconstructs the crowd over the live CycloneDDS binding.
        Headless-verified: renders server==IG over real DDS, **max |server−IG| 0.203 m (run max 0.232 m)**,
        consistent with the P7-2 in-process 0.213 m peak. *(Note: a pre-existing headless Raylib/Xvfb
        teardown segfault (exit 139) occurs AFTER the screenshot is written — reproduces on the InMemory
        demo too, so it is unrelated to the DDS change.)*
  - [~] **D3b** — genuine two-process, cross-process DDS ped path:
    - [x] **native** — `RemotePedServer` extracted (shared sim); `--mode ped-publish` (headless ped
          publisher over live DDS) + demo "Pedestrian remote (DDS subscribe)" (`DdsSubscribeOnly`: no local
          sim, renders purely from the wire). Verified two-process: the subscriber received all **14 peds**
          and their high-power promotions from a SEPARATE publisher process over CycloneDDS; D3a single-
          process re-verified (no regression). *(Same pre-existing headless-teardown segfault applies.)*
    - [x] **Godot** — City3D `--transport=dds --peds`: `ReadyPeds` + `ProcessPeds` branch on transport, feeding
          `PedReconstructor` from a `DdsPedReplicationSource` (fed by `--mode ped-publish`) instead of the
          local byte-loopback `PedSimSource`; server clock driven off the wire (`LatestCrowdTime`, monotonic).
          Compiles under `-p:City3DRemote=true` (CITY3D_REMOTE). The data path is the SAME
          `DdsPedReplicationSource` + `PedRemoteReconstructor` the native two-process render verified; Godot's
          `PedReconstructor` is a thin wrapper over it. **Live Godot screenshot verified** (Godot 4.7.1 under
          Xvfb + llvmpipe, fed by a separate `--mode ped-publish` process): the City3D crossing plaza rendered
          `peds=8` cross-process over CycloneDDS including a **cyan high-power** ped (DR-switch crossed the
          wire); wire-authoritative `pedTime` advanced to 29.40. Compiles under `-p:City3DRemote=true`.

## Standing invariants (every task)

- [ ] Offline gate unchanged: `dotnet test Traffic.sln -c Release` stays green (**634 (+3 skip) / 142 / 2 /
      1**, never drops, hash `909605E965BFFE59` unchanged). The binding + proof are **never** in
      `Traffic.sln`.
- [ ] `dotnet build src/Sim.Replication.Dds -c Release` stays green (the shipped vehicle binding is not
      disturbed by the additive ped types).

---

## D1 — DDS ped topics + QoS + sink/source binding

Design ref: `PEDESTRIAN-DDS-TRANSPORT-DESIGN.md` §4, §5. Files (all in `src/Sim.Replication.Dds/`):
`PedTopics.cs` (new), `DdsTopicNames.cs` (add 4 constants), `DdsPedReplicationSink.cs` (new),
`DdsPedReplicationSource.cs` (new). No changes to `Sim.Replication` / `Sim.Pedestrians`.

**Success conditions**
1. `DdsPedReplicationSink : IPedReplicationSink` and `DdsPedReplicationSource : IPedReplicationSource` both
   compile against the existing surface with **no change** to `PedReplication.cs`.
2. Crowd frame reuses `DdsWireFrame` (`Kind = FrameCodec.KindPedFreeKinematic`), chunked via
   `FrameChunker.MaxRecordsForPayload(DdsWire.MaxPayload, FrameCodec.PedFreeKinematicRecordSize)`; PathArc +
   ActivityTimeline ride the new `DdsPedLeg` on distinct topic names; lifecycle rides `DdsPedLifecycle`.
   Every writer/reader is built with an **explicit `DdsTopicNames.*` string** (2-arg-plus-QoS ctor), and a
   writer/reader pair uses the **identical** string and the **matching** QoS profile (crowd
   `VolatileLatest`; the other three `DurableLatest`).
3. `dotnet build src/Sim.Replication.Dds -c Release` succeeds (CycloneDDS code-gen runs clean) and the
   offline gate is untouched (both standing invariants hold).

## D2 — Live DDS loopback: `server == IG` over real CycloneDDS

Design ref: `PEDESTRIAN-DDS-TRANSPORT-DESIGN.md` §6. File: `src/Sim.PedDdsLoopback/` (new net8.0 console;
references `Sim.Pedestrians` + `Sim.Replication.Dds`; **not** added to `Traffic.sln`). Mirrors
`src/Sim.Viewer/LoopbackSelfTest.cs` (discovery settle → publish/pump loop → assert) and the population +
bounds of `tests/Sim.Pedestrians.Tests/Lod/PedReplicationRoundTripTests.cs`.

**Success conditions**
1. Running it (`dotnet run --project src/Sim.PedDdsLoopback -c Release`) drives a real `PedLodManager`
   population through `PedReplicationPublisher → DdsPedReplicationSink → CycloneDDS →
   DdsPedReplicationSource → PedReplicationReceiver` and prints **PASS** (exit 0).
2. Reconstruction bounds over live DDS match the hermetic test: PathArc ped and ActivityTimeline ped
   reconstruct **exactly** (position X/Y equal) across a fine time sweep; the promoted FreeKinematic ped
   reconstructs within **≤ 0.02 m**; the receiver's `Ig.ModelOf(promotedId)` flips to `FreeKinematic` after
   the promote lifecycle event arrives over the wire. Any violation → exit 1.
3. It prints the **measured live single-stream bytes/sec per topic** (crowd / patharc / activity /
   lifecycle) for the run — the live-transport figure that complements the logical `PedBandwidthMeter`
   (the live-DDS half of P6-1 SC3). This is a reported measurement, not a pass/fail bound.
4. The offline gate remains green and CycloneDDS-free (standing invariant), since this project is outside
   `Traffic.sln`.

## D3 — Viewer + Godot live-DDS ped path *(follow-on; confirm before starting)*

Design ref: `PEDESTRIAN-DDS-TRANSPORT-DESIGN.md` §8. Consume `DdsPedReplicationSource` in the native
viewer's `--mode remote` render loop (alongside `RemotePedOverlay`) and in Godot City3D's
`--transport=dds` path (alongside `PedReconstructor`). Larger, viewer-side surface — **held for user
confirmation** before implementation.

**Success conditions** *(to be finalized when D3 is greenlit)*
1. Native viewer `--mode remote` renders peds fed from a separate live-DDS publisher process (server == IG
   visually; no promotion pop), mirroring the vehicle remote path.
2. Godot City3D `--transport=dds` renders peds from the same live stream (ground-placed, heading, LOD-cull).
3. Both remain out of `Traffic.sln`; the offline gate is untouched.
