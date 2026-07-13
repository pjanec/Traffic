namespace Sim.Core;

// SUMOSHARP-API.md §5: the per-step published read snapshot behind Engine's columnar read spans and
// TryGetVehicle. A struct-of-arrays PROJECTION refreshed each Step from the live active vehicles (it is
// NOT the engine's source of truth, so it never reshapes vehicle storage): Engine walks ActiveVehicles(),
// projects each to (x, y, angle) with the SAME LaneGeometry.PositionAtOffset call EmitTrajectory uses,
// and appends a dense row here. Columns are contiguous (index 0..Count-1) so a renderer can upload a
// slice straight to a GPU vertex buffer, or a host can iterate whole rows.
//
// Precision split (D7): the render-facing geometry is float (PosX/Y/Z, Angle, SpeedF); the parity-exact
// lane-relative values are double (Pos, PosLat, SpeedD). Random access (TryGetVehicle) resolves an
// EntityIndex to its dense slot via a frame-stamped side map -- no per-frame clearing.
internal sealed class VehicleReadBuffer
{
    private const int InitialCapacity = 16;

    public int Count { get; private set; }

    // Dense columns, valid for [0, Count).
    public VehicleHandle[] Handles = new VehicleHandle[InitialCapacity];
    public int[] EntityIndex = new int[InitialCapacity];
    public string[] VehicleId = new string[InitialCapacity];
    public string[] VehicleType = new string[InitialCapacity];
    public int[] LaneHandle = new int[InitialCapacity];
    public int[] NextLane = new int[InitialCapacity];       // next lane handle on the route (-1 if none) -- DR lookahead
    public string[] LaneId = new string[InitialCapacity];
    public double[] Pos = new double[InitialCapacity];
    public double[] SpeedD = new double[InitialCapacity];
    public double[] AccelD = new double[InitialCapacity];   // longitudinal acceleration (getAcceleration analog)
    public double[] PosLat = new double[InitialCapacity];
    public float[] PosX = new float[InitialCapacity];
    public float[] PosY = new float[InitialCapacity];
    public float[] PosZ = new float[InitialCapacity];
    public float[] Angle = new float[InitialCapacity];
    public float[] SpeedF = new float[InitialCapacity];
    public float[] Length = new float[InitialCapacity];     // vehicle body dims (render: sized rectangles)
    public float[] Width = new float[InitialCapacity];

    // EntityIndex -> dense slot, frame-stamped so BeginFrame never has to clear it: a slot is current
    // only if its stamp equals the live frame counter.
    private int[] _slotByEntity = new int[InitialCapacity];
    private int[] _frameOfEntity = new int[InitialCapacity];
    private int _frame;

    // Start a fresh frame. `maxEntityExclusive` is the current vehicle count (EntityIndex upper bound).
    public void BeginFrame(int maxEntityExclusive)
    {
        Count = 0;
        _frame++;
        EnsureEntityCapacity(maxEntityExclusive);
    }

    public void Add(
        VehicleHandle handle, int entityIndex, string vehicleId, string vehicleType,
        int laneHandle, int nextLane, string laneId, double pos, double speed, double accel, double posLat,
        float x, float y, float z, float angle, float length, float width)
    {
        EnsureColumnCapacity(Count + 1);

        var i = Count;
        Handles[i] = handle;
        EntityIndex[i] = entityIndex;
        VehicleId[i] = vehicleId;
        VehicleType[i] = vehicleType;
        LaneHandle[i] = laneHandle;
        NextLane[i] = nextLane;
        LaneId[i] = laneId;
        Pos[i] = pos;
        SpeedD[i] = speed;
        AccelD[i] = accel;
        PosLat[i] = posLat;
        PosX[i] = x;
        PosY[i] = y;
        PosZ[i] = z;
        Angle[i] = angle;
        SpeedF[i] = (float)speed;
        Length[i] = length;
        Width[i] = width;

        _slotByEntity[entityIndex] = i;
        _frameOfEntity[entityIndex] = _frame;
        Count = i + 1;
    }

    public bool TryGetSlot(int entityIndex, out int slot)
    {
        if (entityIndex >= 0 && entityIndex < _frameOfEntity.Length && _frameOfEntity[entityIndex] == _frame)
        {
            slot = _slotByEntity[entityIndex];
            return true;
        }

        slot = -1;
        return false;
    }

    private void EnsureColumnCapacity(int needed)
    {
        var cap = Handles.Length;
        if (needed <= cap)
        {
            return;
        }

        var newCap = cap;
        while (newCap < needed)
        {
            newCap *= 2;
        }

        Array.Resize(ref Handles, newCap);
        Array.Resize(ref EntityIndex, newCap);
        Array.Resize(ref VehicleId, newCap);
        Array.Resize(ref VehicleType, newCap);
        Array.Resize(ref LaneHandle, newCap);
        Array.Resize(ref NextLane, newCap);
        Array.Resize(ref LaneId, newCap);
        Array.Resize(ref Pos, newCap);
        Array.Resize(ref SpeedD, newCap);
        Array.Resize(ref AccelD, newCap);
        Array.Resize(ref PosLat, newCap);
        Array.Resize(ref PosX, newCap);
        Array.Resize(ref PosY, newCap);
        Array.Resize(ref PosZ, newCap);
        Array.Resize(ref Angle, newCap);
        Array.Resize(ref SpeedF, newCap);
        Array.Resize(ref Length, newCap);
        Array.Resize(ref Width, newCap);
    }

    private void EnsureEntityCapacity(int needed)
    {
        var cap = _slotByEntity.Length;
        if (needed <= cap)
        {
            return;
        }

        var newCap = cap;
        while (newCap < needed)
        {
            newCap *= 2;
        }

        Array.Resize(ref _slotByEntity, newCap);
        Array.Resize(ref _frameOfEntity, newCap);
        // New _frameOfEntity entries default to 0; the live frame counter is >= 1 after BeginFrame, so
        // they never falsely resolve as "current".
    }
}
