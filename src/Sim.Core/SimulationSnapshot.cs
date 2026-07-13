namespace Sim.Core;

// SUMOSHARP-API.md §7: an IMMUTABLE, self-contained copy of one published simulation frame -- the async
// runner's hand-off to the host. The engine's own read spans (Engine.PosX, ...) alias reused buffers and
// are only safe between steps on the engine's thread; a SimulationSnapshot is a stable copy the host can
// read from any thread for as long as it holds the reference. Built once per SimulationRunner.Tick.
//
// Columns are structure-of-arrays for a renderer (upload a slice to a vertex buffer); TryGetVehicle gives
// array-of-structures random access. Per-frame allocation for now (correctness-first); a snapshot pool is
// a later refinement (documented in §7).
public sealed class SimulationSnapshot
{
    public int Count { get; init; }
    public double Time { get; init; }
    public int StepCount { get; init; }

    public VehicleHandle[] Handles { get; init; } = Array.Empty<VehicleHandle>();
    public float[] PosX { get; init; } = Array.Empty<float>();
    public float[] PosY { get; init; } = Array.Empty<float>();
    public float[] PosZ { get; init; } = Array.Empty<float>();
    public float[] Angle { get; init; } = Array.Empty<float>();
    public float[] Speed { get; init; } = Array.Empty<float>();          // render-facing float
    public float[] Length { get; init; } = Array.Empty<float>();         // body dims (metres) for sized rendering
    public float[] Width { get; init; } = Array.Empty<float>();
    public double[] SpeedExact { get; init; } = Array.Empty<double>();   // parity-exact double
    public double[] Accel { get; init; } = Array.Empty<double>();        // longitudinal accel (DR extrapolation)
    public int[] LaneHandle { get; init; } = Array.Empty<int>();
    public int[] NextLaneHandle { get; init; } = Array.Empty<int>();     // next lane on route, -1 if none (DR lookahead)
    public int[] PrevLaneHandle { get; init; } = Array.Empty<int>();     // previous lane on route, -1 if none (chord back-walk)
    public int[] LaneWindow { get; init; } = Array.Empty<int>();         // flattened [p2,p1,cur,n1,n2,n3] per vehicle (multi-lane DR walk)
    public int LaneWindowStride { get; init; }
    public double[] Pos { get; init; } = Array.Empty<double>();
    public double[] PosLat { get; init; } = Array.Empty<double>();
    // Per-vehicle dead-reckoning regime (DrModel as byte) + the mid-manoeuvre bit (issue #3/#4 seam). The
    // DR publisher reads DrModel to pick the extrapolator and Manoeuvring as the adaptive-rate signal, so
    // it no longer has to assume LaneArc. Populated in the Step projection only (off the parity path).
    public byte[] DrModels { get; init; } = Array.Empty<byte>();
    public bool[] Manoeuvring { get; init; } = Array.Empty<bool>();
    public string[] VehicleId { get; init; } = Array.Empty<string>();
    public string[] VehicleType { get; init; } = Array.Empty<string>();
    public string[] LaneId { get; init; } = Array.Empty<string>();

    public SimEvent[] Events { get; init; } = Array.Empty<SimEvent>();
    public int EventCount { get; init; }

    // §5.2 traffic-light state: controlled approach lane handles + their current signal chars (byte, e.g.
    // 'G'/'g'/'y'/'r'), aligned index-for-index over [0, TlCount). For rendering junction signals.
    public int[] TlLaneHandle { get; init; } = Array.Empty<int>();
    public byte[] TlState { get; init; } = Array.Empty<byte>();
    public int TlCount { get; init; }

    public static readonly SimulationSnapshot Empty = new();

    // Copy the engine's current read spans + events into a fresh immutable snapshot. Called on the engine
    // thread (SimulationRunner.Tick), immediately after Engine.Step.
    public static SimulationSnapshot Capture(Engine engine)
    {
        var n = engine.VehicleCount;
        var snap = new SimulationSnapshot
        {
            Count = n,
            Time = engine.CurrentTime,
            StepCount = engine.StepCount,
            Handles = engine.VehicleHandles.ToArray(),
            PosX = engine.PosX.ToArray(),
            PosY = engine.PosY.ToArray(),
            PosZ = engine.PosZ.ToArray(),
            Angle = engine.Angle.ToArray(),
            Speed = engine.Speed.ToArray(),
            Length = engine.VehicleLengths.ToArray(),
            Width = engine.VehicleWidths.ToArray(),
            SpeedExact = engine.SpeedExactColumn.ToArray(),
            Accel = engine.Acceleration.ToArray(),
            LaneHandle = engine.LaneHandles.ToArray(),
            NextLaneHandle = engine.NextLaneHandles.ToArray(),
            PrevLaneHandle = engine.PrevLaneHandles.ToArray(),
            LaneWindow = engine.LaneWindows.ToArray(),
            LaneWindowStride = Engine.LaneWindowStride,
            Pos = engine.Pos.ToArray(),
            PosLat = engine.PosLat.ToArray(),
            DrModels = engine.DrModels.ToArray(),
            Manoeuvring = engine.Manoeuvring.ToArray(),
            VehicleId = engine.VehicleIds.ToArray(),
            VehicleType = engine.VehicleTypes.ToArray(),
            LaneId = engine.LaneIds.ToArray(),
            Events = engine.Events.ToArray(),
            EventCount = engine.Events.Length,
            TlLaneHandle = engine.TlLaneHandles.ToArray(),
            TlState = engine.TlStates.ToArray(),
            TlCount = engine.TlStates.Length,
        };
        return snap;
    }

    // Random access by handle (linear -- fine for occasional correlation; the columnar arrays are the
    // per-frame render path). False if the handle is not present in this frame.
    public bool TryGetVehicle(VehicleHandle handle, out VehicleState state)
    {
        for (var i = 0; i < Count; i++)
        {
            if (Handles[i] == handle)
            {
                state = new VehicleState(
                    Handles[i], (int)Handles[i].Index, VehicleId[i], VehicleType[i],
                    LaneHandle[i], LaneId[i], Pos[i], SpeedExact[i], PosLat[i],
                    PosX[i], PosY[i], PosZ[i], Angle[i]);
                return true;
            }
        }

        state = default;
        return false;
    }
}
