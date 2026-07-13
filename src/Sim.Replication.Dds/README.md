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
- **`DdsVehicleLifecycle`** — the low-rate, **keyed** (by vehicle index) spawn/despawn + dims/type registry
  (DDS's "track many objects" sweet spot; affordable because it is low-rate).

> **Deliberately NOT in `Traffic.sln`.** It depends on the external `CycloneDDS.NET` package and a native
> DDS library, so the hermetic offline parity gate (`dotnet test Traffic.sln`, no network / no native DDS)
> never needs it. Build it explicitly: `dotnet build src/Sim.Replication.Dds`. net8.0 only; the CycloneDDS
> native runtime currently ships win-x64 (so it compiles anywhere, runs where the native lib is present).

## Publishing (host wrapper)

```csharp
using CycloneDDS.Runtime;
using Sim.Replication;        // FrameCodec, VehicleRecord
using Sim.Replication.Dds;    // DdsWireFrame

using var participant = new DdsParticipant();
using var writer = new DdsWriter<DdsWireFrame>(participant);

// Per tick (<=10 Hz), for each chunk of vehicles:
Span<byte> buf = new byte[FrameCodec.VehicleFrameSize(chunk.Length)];
FrameCodec.WriteVehicleFrame(buf, step, time, chunk);
var frame = new DdsWireFrame();
frame.SetPayload(FrameCodec.KindVehicle, step, time, buf, chunkIndex);
writer.Write(frame);
```

## Reading (renderer wrapper)

```csharp
using var reader = new DdsReader<DdsWireFrame>(participant);
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

Recommended QoS: `BEST_EFFORT` / `KEEP_LAST(1)` for `DdsWireFrame` state; `RELIABLE` /
`TRANSIENT_LOCAL` for `DdsVehicleLifecycle` (and a geometry topic) so late joiners get the current fleet.
