# SumoSharp.Replication.Dds

CycloneDDS transport for SumoSharp dead-reckoning replication (`SUMOSHARP-DEADRECKONING.md` §4.3).

> Unofficial, independent C# reimplementation of Eclipse SUMO's microscopic simulation core. Not
> affiliated with or endorsed by the Eclipse SUMO project.

## What this is

DDS topic types (over [CycloneDDS.NET](https://github.com/pjanec/CycloneDds.NET)) that carry the
**canonical packed blob** produced by `SumoSharp.Replication.FrameCodec`. One wire format serves DDS,
TCP, and UDP; DDS just transports the bytes.

- **`DdsWireFrame`** — the high-rate **opaque blob** state topic (canonical). Keyed by `(Kind, ChunkIndex)`
  so a tick's chunks are distinct instances. `SetPayload` fills it straight from a `FrameCodec`-written
  buffer; `ReadPayload` hands the valid prefix back to `FrameCodec.Read*`. Batching many movers into one
  sample (per 64 KiB chunk) amortizes DDS per-sample overhead — the key to 10k+ movers (one keyed sample
  *per mover* would be untenable).
- **`DdsVehicleBatch`** — the high-rate **structured** state topic (the *option* alongside the blob). Same
  data as typed DDS fields instead of an opaque payload, so DDS tooling (introspection, content filters,
  keyed QoS) sees per-field values without the SumoSharp codec. It is **columnar** (one typed array per
  field, up to `DdsStructured.MaxSamples = 256` movers per sample); `SetBatch` fills it from a
  `FrameChunker`-sized chunk of `VehicleRecord`s, `ReadBatch` hands them back. Use it when a subscriber wants
  to consume fields directly; prefer `DdsWireFrame` (fewer bytes, one codec across all transports) otherwise.
- **`DdsVehicleLifecycle`** — the low-rate, **keyed** (by vehicle index) spawn/despawn + dims/type registry
  (DDS's "track many objects" sweet spot; affordable because it is low-rate).
- **`DdsNetworkGeometry`** — durable-intent lane-polyline topic (opaque blob carrying `GeometryCodec` bytes),
  published once so a remote viewer can draw roads without the net file. Chunked (`GeometryCodec.PlanChunks`)
  across `DdsWire.MaxPayload`-sized samples for a large network; keyed by `ChunkIndex`, with `TotalChunks` so
  a subscriber knows when it has the whole net.
- **`DdsTlState`** — low-rate traffic-light state (opaque blob carrying `TlCodec` bytes): controlled-lane
  handle + signal char pairs. Kept off the high-rate vehicle topic by design (`SUMOSHARP-DEADRECKONING.md`
  §5.2); every committed scenario's TL state fits in one chunk.

> **CRITICAL — always pass an explicit topic-name string.** The `[DdsTopic]` attribute on these structs does
> **not** give the CycloneDDS.NET 0.3.2 generator a usable name: the 1-arg `new DdsWriter<T>(participant)` /
> `new DdsReader<T>(participant)` ctor resolves to a NULL topic name and throws `ArgumentNullException`.
> Every writer and reader in this repo is constructed with the 2-arg ctor and a name from
> `DdsTopicNames` (`Vehicles`/`Lifecycle`/`Geometry`/`Tl`) — a writer and its matching reader **must** use the
> identical string, e.g. `new DdsWriter<DdsWireFrame>(participant, DdsTopicNames.Vehicles)` and
> `new DdsReader<DdsWireFrame>(participant, DdsTopicNames.Vehicles)`.

> **Deliberately NOT in `Traffic.sln`.** It depends on the external `CycloneDDS.NET` package and a native
> DDS library, so the hermetic offline parity gate (`dotnet test Traffic.sln`, no network / no native DDS)
> never needs it. Build it explicitly: `dotnet build src/Sim.Replication.Dds`. net8.0 only; CycloneDDS.NET
> 0.3.2 ships native runtimes for **both `linux-x64` and `win-x64`** (`libddsc.so` / `ddsc.dll`), so it runs
> on either.

## Publishing (host wrapper)

```csharp
using CycloneDDS.Runtime;
using Sim.Replication;        // FrameCodec, VehicleRecord
using Sim.Replication.Dds;    // DdsWireFrame

using var participant = new DdsParticipant();
using var writer = new DdsWriter<DdsWireFrame>(participant, DdsTopicNames.Vehicles);

// Per tick (<=10 Hz), for each chunk of vehicles:
Span<byte> buf = new byte[FrameCodec.VehicleFrameSize(chunk.Length)];
FrameCodec.WriteVehicleFrame(buf, step, time, chunk);
var frame = new DdsWireFrame();
frame.SetPayload(FrameCodec.KindVehicle, step, time, buf, chunkIndex);
writer.Write(frame);
```

## Reading (renderer wrapper)

```csharp
using var reader = new DdsReader<DdsWireFrame>(participant, DdsTopicNames.Vehicles);
using var loan = reader.Take(maxSamples: 32);
Span<byte> bytes = stackalloc byte[DdsWire.MaxPayload];
var recs = new VehicleRecord[2048];
foreach (var sample in loan)
{
    var f = sample.Data;
    var n = f.ReadPayload(bytes);
    var count = FrameCodec.ReadVehicleFrame(bytes[..n], recs);
    // feed each record + the once-received lane geometry to PoseResolver.Resolve(...)
}
```

## Structured option (`DdsVehicleBatch`)

Same per-tick chunking, but typed columns instead of a blob. `FrameChunker` sizes the chunks either way —
by byte budget for the blob, by sample count here:

```csharp
using var writer = new DdsWriter<DdsVehicleBatch>(participant, "sumo/vehicle-batch"); // pick your own name; must match the reader

var maxPerChunk = DdsStructured.MaxSamples;                      // 256 movers / structured sample
for (var c = 0; c < FrameChunker.ChunkCount(recs.Length, maxPerChunk); c++)
{
    var (start, count) = FrameChunker.ChunkRange(c, recs.Length, maxPerChunk);
    var batch = new DdsVehicleBatch();
    batch.SetBatch(step, time, recs.AsSpan(start, count), c);
    writer.Write(batch);
}
// reader: var n = batch.ReadBatch(recs);  // -> VehicleRecord[], then PoseResolver.Resolve(...)
```

Recommended QoS: `BEST_EFFORT` / `KEEP_LAST(1)` for `DdsWireFrame` / `DdsVehicleBatch` state; `RELIABLE` /
`TRANSIENT_LOCAL` for `DdsVehicleLifecycle` (and a geometry topic) so late joiners get the current fleet.
