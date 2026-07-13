using System.Runtime.CompilerServices;
using CycloneDDS.Schema;
using Sim.Core;
using Sim.Replication;

namespace Sim.Replication.Dds;

// SUMOSHARP-DEADRECKONING.md §4.3 — the STRUCTURED-CHUNK state topic: the OPTION alongside the canonical
// opaque blob (DdsWireFrame). Same data, but as typed DDS fields instead of an opaque byte payload, so a
// subscriber's DDS tooling (introspection, content filters, keyed QoS) sees the fields without the SumoSharp
// codec. It is COLUMNAR (structure-of-arrays): one typed array per field, up to MaxSamples movers per DDS
// sample. That batches many movers into one sample (FrameChunker caps by SAMPLE count here, by BYTE budget
// for the blob) so the "one keyed sample per vehicle" cost stays away at 10k+; and columnar arrays of
// primitives are exactly what this CycloneDDS generator serializes (a nested-struct array would need a
// bounded sequence -- columns are simpler, allocation-free, and mirror the engine's own SoA layout).
//
// COMPILE-CHECKED here (a live DDS round-trip is possible too -- CycloneDDS.NET 0.3.2 ships a linux-x64
// native lib, not win-x64 only). The column<->VehicleRecord field mapping mirrors FrameCodec's exactly
// (same scalars, same order), which the hermetic gate round-trips (RungB22); the chunk arithmetic that
// feeds it is RungB25. Both live in SumoSharp.Replication.
public static class DdsStructured
{
    // Movers per structured batch (column length). ~ (4+2+1+4 + 5*4 + 4*4) = 47 B/mover -> ~12 KiB/sample at
    // 256, comfortably inside a datagram. Kept OUT of the [DdsTopic] struct: the CycloneDDS code generator
    // treats a topic's public members as serializable fields, so a public const there mis-generates.
    public const int MaxSamples = 256;
}

// Fixed-length primitive columns (InlineArray -> the generator emits a C `fixed` buffer, legal for
// primitives). Indexed safely in the fill/read helpers below -- no unsafe in this file's own code.
[InlineArray(DdsStructured.MaxSamples)] public struct DdsU32Col { private uint _e0; }
[InlineArray(DdsStructured.MaxSamples)] public struct DdsU16Col { private ushort _e0; }
[InlineArray(DdsStructured.MaxSamples)] public struct DdsU8Col { private byte _e0; }
[InlineArray(DdsStructured.MaxSamples)] public struct DdsI32Col { private int _e0; }
[InlineArray(DdsStructured.MaxSamples)] public struct DdsF32Col { private float _e0; }

// The structured state topic: one instance per (tick, chunk). Keyed by ChunkIndex so a tick's chunks are
// distinct instances (mirrors DdsWireFrame's chunking). `Count` is the valid prefix of every column.
[DdsTopic]
public partial struct DdsVehicleBatch
{
    [DdsKey, DdsId(0)] public int ChunkIndex;
    [DdsId(1)] public uint Step;
    [DdsId(2)] public float Time;
    [DdsId(3)] public int Count;
    [DdsId(4)] public DdsU32Col Index;
    [DdsId(5)] public DdsU16Col Generation;
    [DdsId(6)] public DdsU8Col Model;          // DrModel (byte-backed)
    [DdsId(7)] public DdsI32Col LaneHandle;
    [DdsId(8)] public DdsF32Col Pos;
    [DdsId(9)] public DdsF32Col PosLat;
    [DdsId(10)] public DdsF32Col Speed;
    [DdsId(11)] public DdsF32Col Accel;
    [DdsId(12)] public DdsF32Col LatSpeed;
    [DdsId(13)] public DdsI32Col Up0;
    [DdsId(14)] public DdsI32Col Up1;
    [DdsId(15)] public DdsI32Col Up2;
    [DdsId(16)] public DdsI32Col Up3;

    // Fill from one FrameChunker chunk of records. Throws if the chunk exceeds the column bound.
    public void SetBatch(uint step, float time, ReadOnlySpan<VehicleRecord> chunk, int chunkIndex)
    {
        if (chunk.Length > DdsStructured.MaxSamples)
        {
            throw new ArgumentException(
                $"chunk {chunk.Length} exceeds DdsStructured.MaxSamples {DdsStructured.MaxSamples}; use FrameChunker to size chunks.",
                nameof(chunk));
        }

        ChunkIndex = chunkIndex;
        Step = step;
        Time = time;
        Count = chunk.Length;
        for (var i = 0; i < chunk.Length; i++)
        {
            ref readonly var r = ref chunk[i];
            Index[i] = r.Handle.Index;
            Generation[i] = r.Handle.Generation;
            Model[i] = (byte)r.Model;
            LaneHandle[i] = r.LaneHandle;
            Pos[i] = (float)r.Pos;
            PosLat[i] = (float)r.PosLat;
            Speed[i] = (float)r.Speed;
            Accel[i] = (float)r.Accel;
            LatSpeed[i] = (float)r.LatSpeed;
            Up0[i] = r.Upcoming[0];
            Up1[i] = r.Upcoming[1];
            Up2[i] = r.Upcoming[2];
            Up3[i] = r.Upcoming[3];
        }
    }

    // Copy the valid columns into `dst` as VehicleRecords; returns the number read (min of Count, dst.Length).
    public readonly int ReadBatch(Span<VehicleRecord> dst)
    {
        var n = Math.Min(Count, dst.Length);
        Span<int> up = stackalloc int[UpcomingLanes.Count];
        for (var i = 0; i < n; i++)
        {
            up[0] = Up0[i]; up[1] = Up1[i]; up[2] = Up2[i]; up[3] = Up3[i];
            dst[i] = new VehicleRecord(
                new VehicleHandle(Index[i], Generation[i]), (DrModel)Model[i], LaneHandle[i],
                Pos[i], PosLat[i], Speed[i], Accel[i], LatSpeed[i], new UpcomingLanes(up));
        }

        return n;
    }
}
