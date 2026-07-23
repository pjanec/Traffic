using System;
using System.Collections.Generic;
using System.IO;
using Sim.Core;

namespace Sim.Replication.Recording;

// docs/LIVE-CITY-VIEWERS-DESIGN.md §2.1/§2.2, docs/LIVE-CITY-VIEWERS-TASKS.md Stage C (C1) — the on-disk
// `.simrec` container: a header, then a time-ordered stream of typed records each framed
// `[byte type][double time][payload]`, capturing BOTH the vehicle (wire-codec) track and the pedestrian
// (plain-snapshot) track in one file so replay reconstructs the coupled scene. No footer time-index is
// written — per the task's explicit "OR document that seek replays from the start" allowance, seeking
// (ReplicationFileSource.SeekTo) simply rewinds to the header and replays linearly, which is cheap at the
// minute-scale recordings this feature targets (a downtown-crop live-city run is a few thousand records).
//
// Every non-GEOMETRY record carries an explicit `time` (double, full precision) right after its type byte,
// independent of whatever time field a wrapped codec's OWN header might also carry (e.g. FrameCodec's
// float32 `Time`) — this lets a reader (ReplicationFileSource/PedFrameTrack) decide "is this record due
// yet?" by peeking 9 bytes, without touching the wrapped codec at all, and gives full double precision
// where the wire codec would otherwise truncate to float32. GEOMETRY has no time semantics (it is
// durable/once, like the live wire) — its `time` field is written as 0.0 and ignored by readers, which
// apply it unconditionally the first time they see it.
//
// Payload per record type (reusing the existing wire codecs verbatim where one exists):
//   GEOMETRY       int byteLen + GeometryCodec.WriteGeometry(...) bytes (GeometryCodec itself is
//                  version-dispatched -- see its own header comment -- so bumping ITS version needed no
//                  change here; the blob is opaque to SimRecFormat)
//   VLIFECYCLE     handle.Index(u32) handle.Generation(u16) isSpawn(bool) vTypeId(i32) length(f32) width(f32)
//                  name(string)  -- FormatVersion 2 (docs/LIVE-CITY-VIEWERS-DESIGN.md §3.1,
//                  -TASKS.md E2): LifecycleRecord.Name landed, so it is recorded here too (BinaryWriter's
//                  length-prefixed string; "" on despawn, matching the wire). FormatVersion 1 files had no
//                  `name` field at this position -- SimRecReader REJECTS anything but the current
//                  FormatVersion (see its ctor) rather than silently misreading an old file, and no
//                  FormatVersion-1 `.simrec` is committed anywhere in this repo (recordings are ephemeral,
//                  regenerated on demand), so there is nothing to migrate.
//   VFRAME         int byteLen + FrameCodec.WriteVehicleFrame(...) bytes (its own step/time/count header
//                  travels inside the blob too; the OUTER `time` above is what readers actually key off)
//   TL             step(u32) + int byteLen + TlCodec.WriteTl(...) bytes
//   PEDFRAME       count(i32) + count * { id(i32) x(f32) y(f32) z(f32) regime(u8) animTag(string) }
public static class SimRecFormat
{
    public const uint Magic = 0x43455253U; // ASCII "SREC" read little-endian as a uint
    public const int FormatVersion = 2;

    public enum RecordType : byte
    {
        Geometry = 1,
        Lifecycle = 2,
        VehicleFrame = 3,
        TrafficLights = 4,
        PedFrame = 5,
    }
}

// One decoded record, in a caller-friendly discriminated-union shape (Kind selects the meaningful
// fields). A class (not a struct) so the byte[]/array payload fields can be null without extra ceremony.
public sealed class SimRecEntry
{
    private SimRecEntry(
        SimRecFormat.RecordType kind, double time, byte[]? geometryBytes, LifecycleRecord lifecycle,
        byte[]? vehicleFrameBytes, uint tlStep, byte[]? tlBytes,
        (int Id, float X, float Y, float Z, byte Regime, string AnimTag)[]? peds)
    {
        Kind = kind;
        Time = time;
        GeometryBytes = geometryBytes;
        Lifecycle = lifecycle;
        VehicleFrameBytes = vehicleFrameBytes;
        TlStep = tlStep;
        TlBytes = tlBytes;
        Peds = peds;
    }

    public SimRecFormat.RecordType Kind { get; }
    public double Time { get; }
    public byte[]? GeometryBytes { get; }
    public LifecycleRecord Lifecycle { get; }
    public byte[]? VehicleFrameBytes { get; }
    public uint TlStep { get; }
    public byte[]? TlBytes { get; }
    public (int Id, float X, float Y, float Z, byte Regime, string AnimTag)[]? Peds { get; }

    public static SimRecEntry ForGeometry(byte[] bytes) =>
        new(SimRecFormat.RecordType.Geometry, 0.0, bytes, default, null, 0, null, null);

    public static SimRecEntry ForLifecycle(double time, in LifecycleRecord rec) =>
        new(SimRecFormat.RecordType.Lifecycle, time, null, rec, null, 0, null, null);

    public static SimRecEntry ForVehicleFrame(double time, byte[] bytes) =>
        new(SimRecFormat.RecordType.VehicleFrame, time, null, default, bytes, 0, null, null);

    public static SimRecEntry ForTrafficLights(double time, uint step, byte[] bytes) =>
        new(SimRecFormat.RecordType.TrafficLights, time, null, default, null, step, bytes, null);

    public static SimRecEntry ForPedFrame(double time, (int Id, float X, float Y, float Z, byte Regime, string AnimTag)[] peds) =>
        new(SimRecFormat.RecordType.PedFrame, time, null, default, null, 0, null, peds);
}

// Sequential writer: opens/creates the file, writes the header once, then appends framed records in
// whatever order the caller calls the Write* methods -- the caller (RecordingReplicationSink /
// RunLiveCity's record loop) is responsible for calling them in time order, matching how LiveCitySim
// already ticks (spawn/despawn -> frame -> TL -> peds, once per Step()).
public sealed class SimRecWriter : IDisposable
{
    private readonly FileStream _stream;
    private readonly BinaryWriter _w;
    private byte[] _scratch = new byte[4096];

    public SimRecWriter(string path, double dt, string datasetId)
    {
        _stream = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.Read);
        _w = new BinaryWriter(_stream);
        _w.Write(SimRecFormat.Magic);
        _w.Write(SimRecFormat.FormatVersion);
        _w.Write(dt);
        _w.Write(datasetId ?? string.Empty);
    }

    public void WriteGeometry(IReadOnlyList<GeometryCodec.LaneGeo> lanes)
    {
        var arr = AsArray(lanes);
        var size = GeometryCodec.GeometrySize(arr);
        EnsureScratch(size);
        var written = GeometryCodec.WriteGeometry(_scratch, arr);

        _w.Write((byte)SimRecFormat.RecordType.Geometry);
        _w.Write(0.0);
        _w.Write(written);
        _w.Write(_scratch, 0, written);
    }

    public void WriteLifecycle(in LifecycleRecord rec, double time)
    {
        _w.Write((byte)SimRecFormat.RecordType.Lifecycle);
        _w.Write(time);
        _w.Write(rec.Handle.Index);
        _w.Write(rec.Handle.Generation);
        _w.Write(rec.IsSpawn);
        _w.Write(rec.VTypeId);
        _w.Write(rec.Length);
        _w.Write(rec.Width);
        _w.Write(rec.Name ?? string.Empty);
    }

    public void WriteVehicleFrame(uint step, double time, ReadOnlySpan<VehicleRecord> movers)
    {
        var size = FrameCodec.VehicleFrameSize(movers.Length);
        EnsureScratch(size);
        var written = FrameCodec.WriteVehicleFrame(_scratch, step, (float)time, movers);

        _w.Write((byte)SimRecFormat.RecordType.VehicleFrame);
        _w.Write(time);
        _w.Write(written);
        _w.Write(_scratch, 0, written);
    }

    public void WriteTrafficLights(uint step, double time, IReadOnlyList<TlCodec.TlEntry> lights)
    {
        var arr = AsArray(lights);
        var size = TlCodec.TlSize(arr.Length);
        EnsureScratch(size);
        var written = TlCodec.WriteTl(_scratch, arr);

        _w.Write((byte)SimRecFormat.RecordType.TrafficLights);
        _w.Write(time);
        _w.Write(step);
        _w.Write(written);
        _w.Write(_scratch, 0, written);
    }

    public void WritePedFrame(double time, IReadOnlyList<(int Id, float X, float Y, float Z, byte Regime, string AnimTag)> peds)
    {
        _w.Write((byte)SimRecFormat.RecordType.PedFrame);
        _w.Write(time);
        _w.Write(peds.Count);
        for (var i = 0; i < peds.Count; i++)
        {
            var p = peds[i];
            _w.Write(p.Id);
            _w.Write(p.X);
            _w.Write(p.Y);
            _w.Write(p.Z);
            _w.Write(p.Regime);
            _w.Write(p.AnimTag ?? string.Empty);
        }
    }

    public void Flush() => _w.Flush();

    public void Dispose()
    {
        _w.Flush();
        _w.Dispose();
    }

    private void EnsureScratch(int size)
    {
        if (_scratch.Length < size)
        {
            _scratch = new byte[size];
        }
    }

    private static GeometryCodec.LaneGeo[] AsArray(IReadOnlyList<GeometryCodec.LaneGeo> lanes) =>
        lanes as GeometryCodec.LaneGeo[] ?? new List<GeometryCodec.LaneGeo>(lanes).ToArray();

    private static TlCodec.TlEntry[] AsArray(IReadOnlyList<TlCodec.TlEntry> entries) =>
        entries as TlCodec.TlEntry[] ?? new List<TlCodec.TlEntry>(entries).ToArray();
}

// Sequential reader: parses the header once, then TryReadNext decodes one framed record per call, in
// file order. `Rewind()` resets the cursor to just past the header, for SeekTo's from-start replay.
public sealed class SimRecReader : IDisposable
{
    private readonly FileStream _stream;
    private readonly BinaryReader _r;

    public SimRecReader(string path)
    {
        _stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read);
        _r = new BinaryReader(_stream);

        var magic = _r.ReadUInt32();
        if (magic != SimRecFormat.Magic)
        {
            throw new InvalidDataException($"'{path}' is not a .simrec file (bad magic).");
        }

        var version = _r.ReadInt32();
        if (version != SimRecFormat.FormatVersion)
        {
            throw new InvalidDataException(
                $"'{path}' is .simrec format version {version}, expected {SimRecFormat.FormatVersion}.");
        }

        Dt = _r.ReadDouble();
        DatasetId = _r.ReadString();
        HeaderEnd = _stream.Position;
    }

    public double Dt { get; }
    public string DatasetId { get; }
    public long HeaderEnd { get; }

    public void Rewind() => _stream.Position = HeaderEnd;

    public bool TryReadNext(out SimRecEntry entry)
    {
        if (_stream.Position >= _stream.Length)
        {
            entry = null!;
            return false;
        }

        var typeByte = _r.ReadByte();
        var type = (SimRecFormat.RecordType)typeByte;

        switch (type)
        {
            case SimRecFormat.RecordType.Geometry:
            {
                _r.ReadDouble(); // written as 0.0; geometry has no time semantics
                var len = _r.ReadInt32();
                entry = SimRecEntry.ForGeometry(_r.ReadBytes(len));
                return true;
            }

            case SimRecFormat.RecordType.Lifecycle:
            {
                var time = _r.ReadDouble();
                var index = _r.ReadUInt32();
                var gen = _r.ReadUInt16();
                var isSpawn = _r.ReadBoolean();
                var vTypeId = _r.ReadInt32();
                var length = _r.ReadSingle();
                var width = _r.ReadSingle();
                var name = _r.ReadString();
                var rec = new LifecycleRecord(new VehicleHandle(index, gen), isSpawn, vTypeId, length, width, name);
                entry = SimRecEntry.ForLifecycle(time, rec);
                return true;
            }

            case SimRecFormat.RecordType.VehicleFrame:
            {
                var time = _r.ReadDouble();
                var len = _r.ReadInt32();
                entry = SimRecEntry.ForVehicleFrame(time, _r.ReadBytes(len));
                return true;
            }

            case SimRecFormat.RecordType.TrafficLights:
            {
                var time = _r.ReadDouble();
                var step = _r.ReadUInt32();
                var len = _r.ReadInt32();
                entry = SimRecEntry.ForTrafficLights(time, step, _r.ReadBytes(len));
                return true;
            }

            case SimRecFormat.RecordType.PedFrame:
            {
                var time = _r.ReadDouble();
                var count = _r.ReadInt32();
                var peds = new (int Id, float X, float Y, float Z, byte Regime, string AnimTag)[count];
                for (var i = 0; i < count; i++)
                {
                    var id = _r.ReadInt32();
                    var x = _r.ReadSingle();
                    var y = _r.ReadSingle();
                    var z = _r.ReadSingle();
                    var regime = _r.ReadByte();
                    var animTag = _r.ReadString();
                    peds[i] = (id, x, y, z, regime, animTag);
                }

                entry = SimRecEntry.ForPedFrame(time, peds);
                return true;
            }

            default:
                throw new InvalidDataException(
                    $"unknown .simrec record type {typeByte} at byte offset {_stream.Position - 1}.");
        }
    }

    public void Dispose() => _r.Dispose(); // disposes the underlying FileStream too
}
