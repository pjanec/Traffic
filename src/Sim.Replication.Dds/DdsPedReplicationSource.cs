using CycloneDDS.Runtime;
using Sim.Core;
using Sim.Replication;

namespace Sim.Replication.Dds;

// docs/PEDESTRIAN-DDS-TRANSPORT-DESIGN.md §4/§5 -- the live CycloneDDS binding of IPedReplicationSource, the
// ped mirror of DdsSubscriber (the vehicle RECEIVE side). Pump()-then-read: every topic is drained on
// Pump(), decoded with the SAME FrameCodec the InMemory bus uses, into the transport-neutral state a
// PedReplicationReceiver (HeadlessIg) reconstructs from. The DDS layer interprets no ActivityTimeline bytes
// -- it hands them back opaque, exactly as InMemoryPedReplicationBus does.
//
// Reader QoS MUST mirror the sink's per-topic profile exactly (a RELIABLE reader will not match a
// BEST_EFFORT writer): crowd = VolatileLatest; patharc/activity/lifecycle = DurableLatest.
//
// CRITICAL (see README.md): every reader is constructed with the SAME explicit DdsTopicNames.Ped* string
// the matching sink writer used.
public sealed class DdsPedReplicationSource : IPedReplicationSource
{
    private const int DurableDepth = 16;

    private readonly DdsReader<DdsWireFrame> _crowdReader;
    private readonly DdsReader<DdsPedLeg> _pathArcReader;
    private readonly DdsReader<DdsPedLeg> _activityReader;
    private readonly DdsReader<DdsPedLifecycle> _lifecycleReader;

    // Newest crowd frame only (docs §4): records are accumulated per step (across a step's chunks, possibly
    // across Pump() calls) and reset when a newer step arrives.
    private uint _latestCrowdStep;
    private float _latestCrowdTime;
    private bool _haveCrowd;
    private readonly List<PedFreeKinematicRecord> _latestCrowdFrame = new();

    // Durable topics accumulate in arrival order, matching the InMemory bus and the source contract.
    private readonly List<PathArcRecord> _pathArcs = new();
    private readonly List<(VehicleHandle Handle, byte[] TimelineBytes)> _timelines = new();
    private readonly List<PedLifecycleRecord> _lifecycles = new();

    // Decode scratch (sized to the max payload; the ped frames are far smaller in practice).
    private byte[] _crowdBytes = new byte[DdsWire.MaxPayload];
    private PedFreeKinematicRecord[] _crowdRecs = new PedFreeKinematicRecord[4096];
    private byte[] _legBytes = new byte[DdsWire.MaxPayload];

    public DdsPedReplicationSource(DdsParticipant participant)
    {
        _crowdReader = new DdsReader<DdsWireFrame>(participant, DdsTopicNames.PedCrowd, DdsQos.VolatileLatest());
        _pathArcReader = new DdsReader<DdsPedLeg>(participant, DdsTopicNames.PedPathArc, DdsQos.DurableLatest(DurableDepth));
        _activityReader = new DdsReader<DdsPedLeg>(participant, DdsTopicNames.PedActivity, DdsQos.DurableLatest(DurableDepth));
        _lifecycleReader = new DdsReader<DdsPedLifecycle>(participant, DdsTopicNames.PedLifecycle, DdsQos.DurableLatest(DurableDepth));
    }

    public uint LatestCrowdStep => _latestCrowdStep;
    public float LatestCrowdTime => _latestCrowdTime;
    public IReadOnlyList<PedFreeKinematicRecord> LatestCrowdFrame => _latestCrowdFrame;
    public IReadOnlyList<PathArcRecord> PathArcs => _pathArcs;
    public IReadOnlyList<(VehicleHandle Handle, byte[] TimelineBytes)> ActivityTimelines => _timelines;
    public IReadOnlyList<PedLifecycleRecord> Lifecycles => _lifecycles;

    // P6-1/remote-view diagnostic (mirrors DdsSubscriber.Connected): true once this crowd reader is matched
    // with at least one live writer -- the transport-level "connected", not "have I decoded a frame".
    public bool Connected => _crowdReader.CurrentStatus.CurrentCount > 0;

    public void Pump()
    {
        PumpCrowd();
        PumpPathArc();
        PumpActivity();
        PumpLifecycle();
    }

    // Keep the newest step's records. Rule per decoded sample (docs §4): step > latest -> fresh frame;
    // step == latest -> append (this step's other chunks); step < latest -> stale, skip. Handles chunks
    // arriving in any order within a Take and split across Pump() calls.
    private void PumpCrowd()
    {
        using var loan = _crowdReader.Take(maxSamples: 64);
        foreach (var sample in loan)
        {
            if (!sample.IsValid)
            {
                continue;
            }

            var f = sample.Data;
            if (f.Kind != FrameCodec.KindPedFreeKinematic)
            {
                continue; // defensive: this topic only carries ped crowd frames.
            }

            var n = f.ReadPayload(_crowdBytes);
            var header = FrameCodec.ReadHeader(_crowdBytes.AsSpan(0, n));
            EnsureCrowdCapacity(header.Count);
            var count = FrameCodec.ReadPedFreeKinematicFrame(_crowdBytes.AsSpan(0, n), _crowdRecs);

            if (!_haveCrowd || header.Step > _latestCrowdStep)
            {
                _haveCrowd = true;
                _latestCrowdStep = header.Step;
                _latestCrowdTime = header.Time;
                _latestCrowdFrame.Clear();
            }
            else if (header.Step < _latestCrowdStep)
            {
                continue; // stale chunk from an older step.
            }

            for (var i = 0; i < count; i++)
            {
                _latestCrowdFrame.Add(_crowdRecs[i]);
            }
        }
    }

    private void PumpPathArc()
    {
        using var loan = _pathArcReader.Take(maxSamples: 256);
        foreach (var sample in loan)
        {
            if (!sample.IsValid)
            {
                continue;
            }

            var d = sample.Data;
            var n = d.ReadPayload(_legBytes);
            var recs = FrameCodec.ReadPathArcFrame(_legBytes.AsSpan(0, n));
            _pathArcs.AddRange(recs);
        }
    }

    private void PumpActivity()
    {
        using var loan = _activityReader.Take(maxSamples: 256);
        foreach (var sample in loan)
        {
            if (!sample.IsValid)
            {
                continue;
            }

            var d = sample.Data;
            var n = d.ReadPayload(_legBytes);
            // Opaque bytes: only the Sim.Pedestrians bridge (which owns ActivityTimelineWire) decodes them.
            _timelines.Add((new VehicleHandle(d.Index, d.Generation), _legBytes.AsSpan(0, n).ToArray()));
        }
    }

    private void PumpLifecycle()
    {
        using var loan = _lifecycleReader.Take(maxSamples: 256);
        foreach (var sample in loan)
        {
            if (!sample.IsValid)
            {
                continue;
            }

            var d = sample.Data;
            _lifecycles.Add(new PedLifecycleRecord(new VehicleHandle(d.Index, d.Generation), (PedLifecycleKind)d.Kind, d.Time));
        }
    }

    private void EnsureCrowdCapacity(int count)
    {
        if (_crowdRecs.Length < count)
        {
            _crowdRecs = new PedFreeKinematicRecord[Math.Max(count, _crowdRecs.Length * 2)];
        }
    }

    public void Dispose()
    {
        _crowdReader.Dispose();
        _pathArcReader.Dispose();
        _activityReader.Dispose();
        _lifecycleReader.Dispose();
    }
}
