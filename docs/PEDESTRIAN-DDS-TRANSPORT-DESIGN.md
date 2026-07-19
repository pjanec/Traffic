# PEDESTRIAN-DDS-TRANSPORT-DESIGN.md — HOW the live CycloneDDS ped wire works

**Status:** design (design-first per `CLAUDE.md`). Task breakdown + success conditions: `PEDESTRIAN-DDS-TRANSPORT-TASKS.md`.

## 1. What this is (and the WHAT it satisfies)

The pedestrian replication surface (`src/Sim.Replication/PedReplication.cs`:
`IPedReplicationSink` / `IPedReplicationSource`) is **transport-neutral** and already has exactly one
binding: `InMemoryPedReplicationBus`, a genuine byte-loopback (it serializes every publish to a `byte[]`
via `FrameCodec` and deserializes on `Pump()`). Every ped-networking proof to date (P3-1/2/3, P7-2, P7-3)
runs `server == IG survives serialization` over that in-process bus.

This feature adds the **second binding: a live CycloneDDS transport**, so the same `IPedReplicationSink` /
`IPedReplicationSource` a caller is already coded against can carry the ped stream over DDS multicast
(one server → many IG). It is the ped mirror of the *already-shipped* vehicle DDS binding
(`src/Sim.Replication.Dds/`, see its `README.md`).

It satisfies, for pedestrians:
- **Requirement 7** ("one server → many IG over DDS multicast") — end-to-end, not just in-process.
- The **live "remote (DDS)"** paths stubbed in the native viewer (`--mode remote`) and Godot City3D
  (`--transport=dds`), both wired for vehicles and deferred for peds (see `PEDESTRIAN-TRACKER.md` P7-2/P7-3
  notes). *(That viewer wiring is Stage D3 — a follow-on to the binding itself.)*
- The **live-DDS half of P6-1 SC3** (single-stream codec bytes/sec confirmed on the real transport, not
  just the logical `PedBandwidthMeter`).

## 2. Non-negotiable constraints (inherited)

1. **Out of `Traffic.sln` / the parity gate — exactly as the vehicle DDS binding is.** The offline gate
   (`dotnet test Traffic.sln -c Release`) must stay green (**634 (+3 skip) / 142 / 2 / 1** at time of
   writing) with **no** CycloneDDS dependency and **no** native DDS library on a fresh VM. The binding
   project and its live loopback proof are built/run explicitly, never by `Traffic.sln`.
2. **Parity hash `909605E965BFFE59` frozen.** This feature touches *no* engine/step path. It lives
   entirely in `Sim.Replication.Dds` (out-of-gate) plus a new out-of-gate console proof; it references
   `Sim.Replication` / `Sim.Pedestrians` read-only through their existing public seams. Parity is inert by
   construction — nothing here is reachable from `Engine.Step()`.
3. **One wire format, any transport.** The DDS layer transports the **exact bytes** `FrameCodec` /
   `ActivityTimelineWire` already produce. It adds *no* new codec and *no* new quantization — the same
   int32-cm crowd packing and double-precision timeline format the InMemory bus round-trips.

## 3. The surface being bound (recap, from `PedReplication.cs`)

`IPedReplicationSink` (SEND) has four publish calls; `IPedReplicationSource` (RECEIVE) is `Pump()`-then-read:

| Sink call | Cadence / durability | Payload | Source exposes |
|---|---|---|---|
| `PublishCrowdFrame(step, time, records)` | high-rate, per-step, **volatile** | `PedFreeKinematicRecord[]`, int32-cm quantized via `FrameCodec.WritePedFreeKinematicFrame` | `LatestCrowdStep/Time/Frame` (newest frame **only**) |
| `PublishPathArc(record)` | once per PathArc leg (spawn + each demotion), **durable** | `PathArcRecord`, via `FrameCodec.WritePathArcFrame` | `PathArcs` (all, arrival order) |
| `PublishActivityTimeline(handle, bytes)` | once per timeline leg, **durable** | **opaque** `ActivityTimelineWire`-encoded bytes | `ActivityTimelines` (all `(handle, bytes)`) |
| `PublishPedLifecycle(record)` | low-rate keyed spawn/despawn/DR-switch, **durable** | `(handle, kind, time)` | `Lifecycles` (all) |

The bridge that maps `Sim.Pedestrians.Lod.PedEvent` onto this surface
(`PedReplicationPublisher` / `PedReplicationReceiver`) is **unchanged** — it already drives *any*
`IPedReplicationSink` / drains *any* `IPedReplicationSource`. The DDS binding is just a new pair of those.

## 4. Topic mapping (the core design decision)

Faithful to the vehicle binding, which reuses one canonical opaque-blob struct (`DdsWireFrame`) for the
high-rate `FrameCodec` stream and adds dedicated structs where the semantics differ
(`DdsVehicleLifecycle` keyed; `DdsNetworkGeometry` / `DdsTlState` durable blobs). The ped mirror:

| Ped surface call | DDS topic type | Topic name (`DdsTopicNames`) | QoS profile | Key | Chunked? |
|---|---|---|---|---|---|
| `PublishCrowdFrame` | **`DdsWireFrame` (reused)** | `sumo/ped-crowd` | `VolatileLatest` (BEST_EFFORT / KEEP_LAST) | `(Kind, ChunkIndex)` | yes (byte budget) |
| `PublishPathArc` | **`DdsPedLeg` (new)** | `sumo/ped-patharc` | `DurableLatest` (RELIABLE / TRANSIENT_LOCAL) | `Index` (ped handle) | no |
| `PublishActivityTimeline` | **`DdsPedLeg` (new, same type)** | `sumo/ped-activity` | `DurableLatest` | `Index` | no |
| `PublishPedLifecycle` | **`DdsPedLifecycle` (new)** | `sumo/ped-lifecycle` | `DurableLatest` | `Index` | no |

### Why these choices

- **Crowd frame reuses `DdsWireFrame`.** The crowd frame *is* the canonical high-rate `FrameCodec` blob
  (`Kind = FrameCodec.KindPedFreeKinematic = 3`, distinct from `KindVehicle = 1` / `KindCrowd = 2`). No new
  type is warranted; the byte layout, `SetPayload`/`ReadPayload` bridge, `(Kind, ChunkIndex)` keying, and
  `VolatileLatest` QoS are identical to the vehicle high-rate path. Chunking uses the shared
  `FrameChunker.MaxRecordsForPayload(DdsWire.MaxPayload, FrameCodec.PedFreeKinematicRecordSize)` → **3640
  peds / 64 KiB chunk**; realistic high-power populations are one chunk, chunking is the scale-safety path
  (mirrors vehicles).
- **PathArc + ActivityTimeline share one new type `DdsPedLeg`.** Both are "**one durable, per-ped,
  keyed opaque blob** sent once per leg": a keyed handle + a byte payload. They differ only in *which*
  bytes (`FrameCodec` PathArc frame vs. `ActivityTimelineWire` blob) and *which topic name* — exactly the
  "one struct, several topic names" pattern the vehicle binding already uses for `DdsWireFrame`. Keying by
  the ped's `Index` (`RELIABLE`/`TRANSIENT_LOCAL`) is the DDS "track many objects" idiom: a **late-joining
  IG gets the latest leg per ped** without waiting for a fresh publish — the whole point of Requirement 7's
  multicast late-join. `DdsPedLeg` carries `Generation` too, so the source can present the full
  `VehicleHandle` for `ActivityTimelines` (the receiver only reads `Index`, but the field is faithful).
- **Lifecycle mirrors `DdsVehicleLifecycle`.** A tiny keyed struct (`Index` key, `Generation`, `Kind`
  byte = `PedLifecycleKind`, `Time` f64), `DurableLatest`. Late joiners get each ped's latest DR-model
  state. `DdsSubscriber`'s idempotent-replay argument (`DdsQos` doc comment) applies verbatim.

### Reassembly & accumulation semantics (source side)

- **Crowd (volatile, newest-only):** `Pump()` `Take`s the crowd samples and keeps the **newest step's**
  records. Rule per decoded sample: `step > latest` → start a fresh frame (clear, set `latest`);
  `step == latest` → append (accumulate this step's chunks, possibly across `Pump()` calls);
  `step < latest` → skip (stale). A dropped best-effort chunk means a few high-power peds miss one step and
  are extrapolated on the IG — within DR tolerance, matching the vehicle best-effort philosophy. (For the
  D2 proof the population is one chunk, so no cross-pump reassembly is exercised; the logic is the
  scale-safety path.)
- **PathArc / ActivityTimeline / Lifecycle (durable, accumulate):** each `Pump()` `Take`s new samples and
  **appends** to the growing list, in arrival order — matching `InMemoryPedReplicationBus` and the
  `IPedReplicationSource` contract. `Take` removes a sample from the reader once, so no live duplication; a
  late joiner receives the latest sample per key once, appended once. A re-routed ped adds a second
  PathArc entry (same key overwrites the durable cache to the newest; a live subscriber that pumps between
  the two publishes sees both — reconstruction is latest-wins either way, so `HeadlessIg`'s result is
  identical). `DurableLatest(depth)` uses a modest history depth so a rare burst of >1 leg per key between
  pumps is not collapsed.

## 5. Classes to add (all in `src/Sim.Replication.Dds/`, out of `Traffic.sln`)

1. **`PedTopics.cs`** — the new DDS topic types:
   - `DdsPedLeg` — `[DdsKey] uint Index; ushort Generation; int ByteCount; fixed byte Payload[DdsWire.MaxPayload];`
     with `SetPayload`/`ReadPayload` bridging exactly like `DdsNetworkGeometry`.
   - `DdsPedLifecycle` — `[DdsKey] uint Index; ushort Generation; byte Kind; double Time;` (mirrors
     `DdsVehicleLifecycle`).
   *(Crowd reuses the existing `DdsWireFrame` in `DdsTopics.cs`; no change there.)*
2. **`DdsTopicNames.cs`** — add `PedCrowd = "sumo/ped-crowd"`, `PedPathArc = "sumo/ped-patharc"`,
   `PedActivity = "sumo/ped-activity"`, `PedLifecycle = "sumo/ped-lifecycle"`. (Additive; existing names
   untouched, so it cannot disturb the vehicle binding.)
3. **`DdsPedReplicationSink.cs`** — `: IPedReplicationSink`. Four `DdsWriter<T>` (crowd = `VolatileLatest`;
   the other three = `DurableLatest`), each constructed with an **explicit topic-name string** (the
   critical CycloneDDS.NET 0.3.2 constraint, see `README.md`). Encode + write is byte-identical to
   `InMemoryPedReplicationBus.SinkImpl` — same `FrameCodec` calls, same chunk planning as
   `DdsReplicationSink`. Exposes cumulative **bytes-published-per-topic** counters for D3's live-bandwidth
   readout (a pure diagnostic, off the reconstruction path).
4. **`DdsPedReplicationSource.cs`** — `: IPedReplicationSource`. Four `DdsReader<T>` with **matching** QoS
   (a RELIABLE reader will not match a BEST_EFFORT writer). `Pump()` decodes all four per §4. Exposes the
   contract's `LatestCrowd*` / `PathArcs` / `ActivityTimelines` / `Lifecycles`, plus a `Connected` flag off
   the crowd reader's publication-matched status (mirrors `DdsSubscriber.Connected`).

Nothing in `Sim.Replication` or `Sim.Pedestrians` changes. The DDS project already references
`Sim.Replication`; it does **not** reference `Sim.Pedestrians` and does not need to (it binds the
transport-neutral surface only).

## 6. Verifying it (out-of-gate, live DDS)

Live DDS needs async discovery (a `Thread.Sleep` settle) and a native library, so — exactly like the
vehicle `LoopbackSelfTest` — it **cannot** live in the deterministic offline gate. The proof is a new
out-of-`Traffic.sln` **console self-test**, `src/Sim.PedDdsLoopback/` (net8.0, references
`Sim.Pedestrians` + `Sim.Replication.Dds`; **not** added to `Traffic.sln`, so CycloneDDS never enters the
gate). It mirrors `PedReplicationRoundTripTests` but swaps the InMemory bus for
`DdsPedReplicationSink`/`Source` over a real `DdsParticipant`:

- builds the **same** real `PedLodManager` population (a PathArc ped, an ActivityTimeline ped, and a ped
  promoted to FreeKinematic by an interest source);
- drives `PedReplicationPublisher` → `DdsPedReplicationSink` → **CycloneDDS** → `DdsPedReplicationSource`
  → `PedReplicationReceiver` (`HeadlessIg`);
- asserts the **same bounds** as the hermetic test: low-power PathArc + ActivityTimeline reconstruct
  **exactly**; the promoted FreeKinematic ped reconstructs within the **≤ 0.02 m** cm-quantization +
  one-step-extrapolation budget; the receiver's `ModelOf` flips on the promote lifecycle event;
- returns `0` on PASS / `1` on any violation (mirrors `LoopbackSelfTest`'s console contract), and prints
  the **measured live single-stream bytes/sec per topic** (D3) at a representative mix.

This proves `server == IG` survives **real DDS serialization + transport + async discovery**, not merely
an in-process struct hand-off — the same standard the vehicle `LoopbackSelfTest` meets, and already
confirmed feasible in this sandbox (the vehicle self-test round-trips geometry + vehicles over live
CycloneDDS here).

## 7. Determinism / parity argument

- **Zero engine reach.** No file here is called from `Engine.Step()` or any golden path. The parity gate
  neither builds nor references `Sim.Replication.Dds`; the hash is inert by construction.
- **No new numerics.** The DDS layer moves bytes `FrameCodec`/`ActivityTimelineWire` already produce; the
  quantization that defines reconstruction error is unchanged, so the D2 bounds are identical to the
  hermetic P3-1 test's — a divergence would indicate a transport bug, not a numeric one.
- **Additive only.** New topic types, new topic-name constants, new sink/source classes, one new console
  project. No existing type, QoS profile, topic name, or codec is modified, so the shipped vehicle binding
  is untouched.

## 8. Out of scope (this design) / follow-ons

- **D3 — viewer wiring.** Turning on the *real* remote-ped path in the native viewer (`--mode remote`) and
  Godot City3D (`--transport=dds`) consumes `DdsPedReplicationSource` in those render loops. Scoped in the
  task doc but held for confirmation before implementing — it edits viewer/Godot code, a larger surface.
- **Region / AoI filtering.** Explicitly none — Requirement 7 is one multicast stream, no per-channel
  culling (`PEDESTRIAN-DESIGN.md` §1/§7). The DDS binding is a single shared stream by construction.
- **A combined car+ped DDS process.** Vehicles and peds are independent topic sets on one participant; a
  host can run both sinks together, but no unified "publish everything" façade is built here.
