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
    public double[] SpeedExact { get; init; } = Array.Empty<double>();   // parity-exact double
    public int[] LaneHandle { get; init; } = Array.Empty<int>();
    public double[] Pos { get; init; } = Array.Empty<double>();
    public double[] PosLat { get; init; } = Array.Empty<double>();
    public string[] VehicleId { get; init; } = Array.Empty<string>();
    public string[] VehicleType { get; init; } = Array.Empty<string>();
    public string[] LaneId { get; init; } = Array.Empty<string>();

    public SimEvent[] Events { get; init; } = Array.Empty<SimEvent>();
    public int EventCount { get; init; }

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
            SpeedExact = engine.SpeedExactColumn.ToArray(),
            LaneHandle = engine.LaneHandles.ToArray(),
            Pos = engine.Pos.ToArray(),
            PosLat = engine.PosLat.ToArray(),
            VehicleId = engine.VehicleIds.ToArray(),
            VehicleType = engine.VehicleTypes.ToArray(),
            LaneId = engine.LaneIds.ToArray(),
            Events = engine.Events.ToArray(),
            EventCount = engine.Events.Length,
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
