using System.Text;
using Sim.Ingest;

namespace CityLib;

// docs/DEMO-CITY3D-DESIGN.md "Procedural scene generation -> Buildings" -- pure box-placement math, no
// Godot type anywhere (CityLib stays engine-agnostic; the Viewer project turns this into a
// MultiMeshInstance3D). Static geometry only, built once at load, same as RoadMeshBuilder.
public readonly struct BuildingBox
{
    public BuildingBox(float cx, float cy, float cz, float sizeX, float sizeY, float sizeZ, float yawRad)
    {
        CX = cx;
        CY = cy;
        CZ = cz;
        SizeX = sizeX;
        SizeY = sizeY;
        SizeZ = sizeZ;
        YawRad = yawRad;
    }

    // Center, already in GODOT space (CoordinateTransform.SumoToGodot applied to the footprint's ground
    // point, with the elevation term raised by half the box height so the box sits ON the ground plane
    // rather than being bisected by it).
    public float CX { get; }
    public float CY { get; }
    public float CZ { get; }

    // Footprint width along the road (SizeX), height (SizeY, ~6-60m / 2-20 storeys), depth away from the
    // road (SizeZ) -- all in the box's own (unrotated) local axes; YawRad below orients that local frame
    // into world space, so a unit BoxMesh scaled by (SizeX, SizeY, SizeZ) then yawed reproduces the box.
    public float SizeX { get; }
    public float SizeY { get; }
    public float SizeZ { get; }

    // Rotation about +Y (Godot yaw convention, see CoordinateTransform.NaviDegToGodotYawRad), derived
    // from the local road tangent at the placement step so the box's SizeX axis runs along the street
    // (its "front" faces the road).
    public float YawRad { get; }
}

// docs/DEMO-CITY3D-DESIGN.md "Buildings": march each non-internal edge's representative centerline,
// dropping a box on both sides at every interior stride step. Determinism (design "Determinism & scale"
// + task T1.4): every footprint/height comes from a hash of stable ids (edge id, step index, side, a
// single scene seed) -- NEVER `string.GetHashCode()` (per-process salted, not reproducible across
// processes) and NEVER `System.Random` (repo rule) -- so the same network + seed produces byte-identical
// BuildingBox lists on every run, in every process, regardless of thread order (there is no parallelism
// here to begin with, but the hash itself has no hidden process-local state either).
public static class BuildingPlacer
{
    // Stride along the polyline between placement steps (design: "~15-25 m").
    private const double StrideMeters = 20.0;

    // Extra clearance beyond the edge's own half-width before a footprint may start (design: "~6 m").
    private const double SetbackMeters = 6.0;

    // Believable height band, ~2-20 storeys (design: "~6-60 m").
    private const float MinHeightMeters = 6f;
    private const float MaxHeightMeters = 60f;

    // Believable single/double-lot footprint band (along-road width and depth-away-from-road).
    private const float MinFootprintMeters = 8f;
    private const float MaxFootprintMeters = 22f;

    // Fixed default so `PlaceAll(network)` alone (no explicit seed) is still perfectly reproducible.
    private const int DefaultSeed = 20260716;

    private const ulong FnvOffsetBasis = 14695981039346656037UL;
    private const ulong FnvPrime = 1099511628211UL;

    public static IReadOnlyList<BuildingBox> PlaceAll(NetworkModel network, int seed = DefaultSeed)
    {
        var boxes = new List<BuildingBox>();

        foreach (var edge in network.Edges)
        {
            if (edge.Id.StartsWith(':') || edge.Lanes.Count == 0)
            {
                continue; // junction-interior edge, or a degenerate edge with no lanes at all.
            }

            // Representative centerline: the widest lane on the edge (ties keep the first/lowest-index
            // lane, since List<Lane> is parse-ordered and `>` never replaces on an exact tie).
            var centerline = edge.Lanes[0];
            for (var i = 1; i < edge.Lanes.Count; i++)
            {
                if (edge.Lanes[i].Width > centerline.Width)
                {
                    centerline = edge.Lanes[i];
                }
            }

            var shape = centerline.Shape;
            if (shape.Count < 2)
            {
                continue; // no segment to derive a tangent/normal from.
            }

            var totalLength = PolylineLength(shape);
            if (totalLength <= StrideMeters * 2.0)
            {
                // Too short to fit even one interior step once both end-margins (design: "skip steps
                // within ~one stride of an edge end -- leave junction corners open") are excluded.
                continue;
            }

            // Total half-width across every lane of the edge (design: "total half-width (sum of lane
            // widths / 2)"), not just the representative centerline lane's own width.
            var halfWidth = SumWidths(edge.Lanes) / 2.0;
            var setbackOffset = halfWidth + SetbackMeters;

            var stepIndex = 0;
            for (var s = StrideMeters; s < totalLength - StrideMeters; s += StrideMeters)
            {
                var (px, py, pz, tx, ty) = SampleAt(shape, centerline.ShapeZ, s);

                // 2-D unit normal, perpendicular to the tangent (rotate +90deg CCW in SUMO's (x,y) plane,
                // same convention RoadMeshBuilder uses: (dx,dy) -> (-dy,dx)).
                var nx = -ty;
                var ny = tx;

                var headingThetaRad = Math.Atan2(tx, ty); // navi convention: dir = (sin(theta), cos(theta))
                var headingThetaDeg = (float)(headingThetaRad * 180.0 / Math.PI);
                var yawRad = CoordinateTransform.NaviDegToGodotYawRad(headingThetaDeg);

                foreach (var side in Sides)
                {
                    var hash = Fnv1a(edge.Id, stepIndex, side, seed);
                    var footWidth = Lerp(MinFootprintMeters, MaxFootprintMeters, HashUnit(hash, 0));
                    var footDepth = Lerp(MinFootprintMeters, MaxFootprintMeters, HashUnit(hash, 1));
                    var height = Lerp(MinHeightMeters, MaxHeightMeters, HashUnit(hash, 2));

                    // Box center sits `setbackOffset + footDepth/2` away from the centerline, so its
                    // INNER face starts exactly at the setback line rather than straddling it.
                    var dist = setbackOffset + footDepth / 2.0;
                    var cxSumo = px + nx * dist * side;
                    var cySumo = py + ny * dist * side;
                    var czSumo = pz + height / 2.0; // base sits on the sampled ground z; center is raised by half the height

                    var (gx, gy, gz) = CoordinateTransform.SumoToGodot(cxSumo, cySumo, czSumo);

                    boxes.Add(new BuildingBox(gx, gy, gz, footWidth, height, footDepth, yawRad));
                }

                stepIndex++;
            }
        }

        return boxes;
    }

    private static readonly int[] Sides = { -1, 1 };

    private static double SumWidths(IReadOnlyList<Lane> lanes)
    {
        var sum = 0.0;
        foreach (var lane in lanes)
        {
            sum += lane.Width;
        }

        return sum;
    }

    private static double PolylineLength(IReadOnlyList<(double X, double Y)> shape)
    {
        var total = 0.0;
        for (var i = 0; i < shape.Count - 1; i++)
        {
            var (x1, y1) = shape[i];
            var (x2, y2) = shape[i + 1];
            var dx = x2 - x1;
            var dy = y2 - y1;
            total += Math.Sqrt(dx * dx + dy * dy);
        }

        return total;
    }

    // Walks the polyline to the point at arc-length `s`, returning that point (X,Y), its sampled
    // elevation (Z, linearly interpolated from `shapeZ` when present, else 0 -- same convention
    // RoadMeshBuilder uses for a null ShapeZ), and the unit tangent of the segment it falls in.
    private static (double X, double Y, double Z, double TX, double TY) SampleAt(
        IReadOnlyList<(double X, double Y)> shape, IReadOnlyList<double>? shapeZ, double s)
    {
        var remaining = s;
        for (var i = 0; i < shape.Count - 1; i++)
        {
            var (x1, y1) = shape[i];
            var (x2, y2) = shape[i + 1];
            var dx = x2 - x1;
            var dy = y2 - y1;
            var segLen = Math.Sqrt(dx * dx + dy * dy);
            if (segLen < 1e-9)
            {
                continue; // degenerate (repeated) vertex -- skip, contributes no length or direction.
            }

            var isLastUsableSegment = i == shape.Count - 2;
            if (remaining <= segLen || isLastUsableSegment)
            {
                var t = Math.Clamp(remaining / segLen, 0.0, 1.0);
                var px = x1 + dx * t;
                var py = y1 + dy * t;
                var tx = dx / segLen;
                var ty = dy / segLen;
                var z1 = shapeZ is not null && i < shapeZ.Count ? shapeZ[i] : 0.0;
                var z2 = shapeZ is not null && i + 1 < shapeZ.Count ? shapeZ[i + 1] : z1;
                var pz = z1 + (z2 - z1) * t;
                return (px, py, pz, tx, ty);
            }

            remaining -= segLen;
        }

        // Unreachable given the `totalLength > StrideMeters*2` guard in PlaceAll (every requested `s` is
        // strictly less than the polyline's own length), but fall back to the shape's last point/segment
        // rather than throw.
        var last = shape[^1];
        var prev = shape[^2];
        var fdx = last.X - prev.X;
        var fdy = last.Y - prev.Y;
        var flen = Math.Sqrt(fdx * fdx + fdy * fdy);
        var ftx = flen > 1e-9 ? fdx / flen : 0.0;
        var fty = flen > 1e-9 ? fdy / flen : 1.0;
        return (last.X, last.Y, 0.0, ftx, fty);
    }

    // Deterministic FNV-1a hash of (edgeId, stepIndex, side, seed). FNV-1a (not `string.GetHashCode()`,
    // which is randomized per process via a startup salt -- see the type's doc comment) so the exact same
    // inputs hash to the exact same 64-bit value in every process, on every run, forever.
    private static ulong Fnv1a(string edgeId, int stepIndex, int side, int seed)
    {
        var hash = FnvOffsetBasis;
        foreach (var b in Encoding.UTF8.GetBytes(edgeId))
        {
            hash ^= b;
            hash *= FnvPrime;
        }

        hash = MixInt(hash, stepIndex);
        hash = MixInt(hash, side);
        hash = MixInt(hash, seed);
        return hash;
    }

    // Folds a 32-bit int's 4 bytes through the same FNV-1a byte loop as the string above, so every input
    // (string or int) is mixed through one well-tested primitive rather than a bespoke bit trick.
    private static ulong MixInt(ulong hash, int value)
    {
        Span<byte> bytes = stackalloc byte[4];
        System.Buffers.Binary.BinaryPrimitives.WriteInt32LittleEndian(bytes, value);
        foreach (var b in bytes)
        {
            hash ^= b;
            hash *= FnvPrime;
        }

        return hash;
    }

    // Re-mixes the base hash with a small `channel` index to get several roughly-independent [0,1)
    // draws out of one FNV-1a hash (one per footprint attribute), rather than needing a separate RNG.
    private static double HashUnit(ulong baseHash, int channel)
    {
        var h = MixInt(baseHash, channel);
        // Top 53 bits -> a double in [0,1) (matches a double's mantissa precision).
        return (h >> 11) / (double)(1UL << 53);
    }

    private static float Lerp(float min, float max, double t) => (float)(min + (max - min) * t);
}
