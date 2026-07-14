using CycloneDDS.Runtime;
using Sim.Replication.Dds;

namespace Sim.Viewer.Core;

// docs/SUMOSHARP-NATIVE-VIEWER.md "remote control": the reverse DDS channel that lets a view-only `--mode
// remote` (no engine) drive the `--mode publish` process's EngineHost. The remote WRITES DdsViewerCommand;
// the publisher READS them and applies to its host. Kind values are the wire contract (DdsViewerCommand.Kind).
public enum ViewerCommandKind : byte
{
    Pause = 0,            // Flag: 1 = pause, 0 = resume
    SetSpeed = 1,         // Value = x real-time
    Restart = 2,
    ClearObstacles = 3,
    SetRandomTraffic = 4, // Flag: 1 = on, 0 = off
    InjectObstacle = 5,   // X,Y = world point
    SetStepLength = 6,    // Value = sim step length seconds (sandbox only)
}

// Remote side: publishes commands. WriterId keys the instance (unique per remote process) so several remotes
// don't clobber each other's samples; Seq monotonically increases so the publisher can dedup re-deliveries.
public sealed class DdsCommandWriter : IDisposable
{
    private readonly DdsWriter<DdsViewerCommand> _writer;
    private readonly int _writerId;
    private uint _seq;

    public DdsCommandWriter(DdsParticipant participant)
    {
        // QoS (reliable + transient-local + deep keep-last) is declared on the topic type via [DdsQos] and
        // applied automatically -- so the 2-arg ctor (no explicit qos handle) is all that's needed.
        _writer = new DdsWriter<DdsViewerCommand>(participant, DdsTopicNames.Commands);
        _writerId = Environment.ProcessId;
    }

    public void Send(ViewerCommandKind kind, double value = 0.0, double x = 0.0, double y = 0.0, bool flag = false)
    {
        _writer.Write(new DdsViewerCommand
        {
            WriterId = _writerId,
            Seq = ++_seq,
            Kind = (byte)kind,
            Value = value,
            X = x,
            Y = y,
            Flag = (byte)(flag ? 1 : 0),
        });
        Console.WriteLine($"CMD SENT kind={kind} seq={_seq} value={value:F2} flag={flag}");
    }

    public void Dispose() => _writer.Dispose();
}

// Publisher side: reads commands and applies them to the EngineHost. Call PumpApply() once per publish loop.
// Per-WriterId Seq dedup so each command applies exactly once even under RELIABLE re-delivery / late join.
public sealed class DdsCommandReader : IDisposable
{
    private readonly DdsReader<DdsViewerCommand> _reader;
    private readonly Dictionary<int, uint> _lastSeqByWriter = new();

    public DdsCommandReader(DdsParticipant participant)
    {
        _reader = new DdsReader<DdsViewerCommand>(participant, DdsTopicNames.Commands);
    }

    public void PumpApply(EngineHost host)
    {
        using var loan = _reader.Take(maxSamples: 64);
        foreach (var sample in loan)
        {
            if (!sample.IsValid)
            {
                continue;
            }

            var c = sample.Data;
            if (_lastSeqByWriter.TryGetValue(c.WriterId, out var last) && c.Seq <= last)
            {
                continue; // already applied (or a stale re-delivery)
            }

            _lastSeqByWriter[c.WriterId] = c.Seq;
            Console.WriteLine($"CMD APPLIED kind={(ViewerCommandKind)c.Kind} seq={c.Seq} value={c.Value:F2} flag={c.Flag}");
            Apply(host, c);
        }
    }

    private static void Apply(EngineHost host, DdsViewerCommand c)
    {
        switch ((ViewerCommandKind)c.Kind)
        {
            case ViewerCommandKind.Pause: host.SetPaused(c.Flag != 0); break;
            case ViewerCommandKind.SetSpeed: host.SetSpeed(c.Value); break;
            case ViewerCommandKind.Restart: host.Restart(); break;
            case ViewerCommandKind.ClearObstacles: host.ClearObstacles(); break;
            case ViewerCommandKind.SetRandomTraffic: host.SetRandomTraffic(c.Flag != 0); break;
            case ViewerCommandKind.InjectObstacle: host.InjectObstacleAtWorld(c.X, c.Y); break;
            case ViewerCommandKind.SetStepLength: host.SetStepLength(c.Value); break;
        }
    }

    public void Dispose() => _reader.Dispose();
}

// Decoded snapshot of the publisher's engine state (from DdsViewerStatus). `Present` is false until the
// first status sample arrives, so a remote can gate its controls on "is the host actually there".
public readonly struct ViewerStatus
{
    public bool Present { get; init; }
    public bool Sandbox { get; init; }
    public bool Paused { get; init; }
    public double Speed { get; init; }
    public double StepLength { get; init; }
    public double SimTime { get; init; }
    public int VehicleCount { get; init; }
}

// Publisher side: publishes its engine state so remotes can reflect it and disable inapplicable controls.
// Call Publish() periodically (cheap; the topic is KEEP_LAST(1) durable, so a late remote gets it at once).
public sealed class DdsStatusWriter : IDisposable
{
    private readonly DdsWriter<DdsViewerStatus> _writer;
    private readonly int _publisherId;

    public DdsStatusWriter(DdsParticipant participant)
    {
        _writer = new DdsWriter<DdsViewerStatus>(participant, DdsTopicNames.Status);
        _publisherId = Environment.ProcessId;
    }

    public void Publish(EngineHost host)
    {
        var snap = host.Snapshot;
        _writer.Write(new DdsViewerStatus
        {
            PublisherId = _publisherId,
            Sandbox = (byte)(host.ScenarioMode ? 0 : 1),
            Paused = (byte)(host.IsPaused ? 1 : 0),
            Speed = host.Speed,
            StepLength = host.StepLength,
            SimTime = snap.Time,
            VehicleCount = snap.Count,
        });
    }

    public void Dispose() => _writer.Dispose();
}

// Remote side: reads the publisher's status. Pump() each frame; Current holds the latest (Present=false
// until the first sample). Takes the newest sample only (KEEP_LAST(1)).
public sealed class DdsStatusReader : IDisposable
{
    private readonly DdsReader<DdsViewerStatus> _reader;

    public ViewerStatus Current { get; private set; }

    public DdsStatusReader(DdsParticipant participant)
    {
        _reader = new DdsReader<DdsViewerStatus>(participant, DdsTopicNames.Status);
    }

    public void Pump()
    {
        using var loan = _reader.Take(maxSamples: 8);
        foreach (var sample in loan)
        {
            if (!sample.IsValid)
            {
                continue;
            }

            var s = sample.Data;
            Current = new ViewerStatus
            {
                Present = true,
                Sandbox = s.Sandbox != 0,
                Paused = s.Paused != 0,
                Speed = s.Speed,
                StepLength = s.StepLength,
                SimTime = s.SimTime,
                VehicleCount = s.VehicleCount,
            };
        }
    }

    public void Dispose() => _reader.Dispose();
}
