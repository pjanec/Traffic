using System.Globalization;
using System.Text.RegularExpressions;
using System.Xml.Linq;
using Sim.Core.Orca;

namespace Sim.Pedestrians;

// Reads ONLY the pedestrian geometry out of a SUMO net.net.xml (+ optional walkable.add.xml),
// using System.Xml.Linq. This is a wholly separate ingest from the parity
// src/Sim.Ingest/NetworkParser.cs (docs/PEDESTRIAN-DESIGN.md §0 Principle 6) — it must never
// reference, call into, or be merged with that parser, and it must never be edited to serve
// parity needs.
//
// Classification rule (matches netconvert's own edge "function" attribute):
//   - no "function" attribute, lane has allow="pedestrian"  => sidewalk (PedLane)
//   - function="crossing"                                    => crossing (PedCrossing)
//   - function="walkingarea"                                 => walkingarea (PedWalkingArea)
// Internal (function="internal") edges are vehicle-only turn geometry and are ignored here.
public static class PedNetworkParser
{
    // Crossing/walkingarea internal edge ids follow SUMO's ":<junction>_c<N>" /
    // ":<junction>_w<N>" convention. Match the trailing "_c<digits>" or "_w<digits>" suffix and
    // take everything before it as the junction id (junction ids may themselves contain
    // underscores, so this must anchor on the suffix, not split naively).
    private static readonly Regex JunctionFromInternalEdgeId =
        new(@"^:(?<junction>.+)_[cw]\d+$", RegexOptions.Compiled);

    public static PedNetwork Load(string netPath, string? walkableAddPath = null)
    {
        var netDoc = XDocument.Load(netPath);
        var root = netDoc.Root ?? throw new InvalidOperationException($"'{netPath}' has no root element.");

        var tlLogicIds = new HashSet<string>(
            root.Elements("tlLogic").Select(e => (string)e.Attribute("id")!),
            StringComparer.Ordinal);

        var sidewalks = new List<PedLane>();
        var crossings = new List<PedCrossing>();
        var walkingAreas = new List<PedWalkingArea>();

        foreach (var edge in root.Elements("edge"))
        {
            var function = (string?)edge.Attribute("function");
            var edgeId = (string)edge.Attribute("id")!;

            if (function is null)
            {
                // Normal edge: any pedestrian-allowed lane on it is a sidewalk.
                foreach (var lane in edge.Elements("lane"))
                {
                    if (!AllowsPedestrian(lane))
                    {
                        continue;
                    }

                    sidewalks.Add(new PedLane(
                        Id: (string)lane.Attribute("id")!,
                        EdgeId: edgeId,
                        Width: ParseWidth(lane),
                        Shape: ParseShape(lane.Attribute("shape"))));
                }
            }
            else if (function == "crossing")
            {
                var lane = edge.Elements("lane").FirstOrDefault(AllowsPedestrian)
                    ?? throw new InvalidOperationException($"Crossing edge '{edgeId}' has no pedestrian lane.");

                var junctionId = JunctionIdFromInternalEdgeId(edgeId);
                var crossingEdges = ((string?)edge.Attribute("crossingEdges"))
                    ?.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries)
                    ?? Array.Empty<string>();

                crossings.Add(new PedCrossing(
                    Id: (string)lane.Attribute("id")!,
                    JunctionId: junctionId,
                    Width: ParseWidth(lane),
                    Shape: ParseShape(lane.Attribute("shape")),
                    Outline: ParseShape(lane.Attribute("outlineShape")),
                    CrossingEdges: crossingEdges,
                    TlLogicId: tlLogicIds.Contains(junctionId) ? junctionId : null));
            }
            else if (function == "walkingarea")
            {
                var lane = edge.Elements("lane").FirstOrDefault(AllowsPedestrian)
                    ?? throw new InvalidOperationException($"Walkingarea edge '{edgeId}' has no pedestrian lane.");

                walkingAreas.Add(new PedWalkingArea(
                    Id: (string)lane.Attribute("id")!,
                    JunctionId: JunctionIdFromInternalEdgeId(edgeId),
                    Width: ParseWidth(lane),
                    Polygon: ParseShape(lane.Attribute("shape"))));
            }
            // function="internal" (and any other function) is vehicle-only turn geometry; ignored.
        }

        var walkablePolygons = new List<WalkablePolygon>();
        var accessPoints = new List<WalkableAccessPoint>();

        if (walkableAddPath is not null)
        {
            var addDoc = XDocument.Load(walkableAddPath);
            var addRoot = addDoc.Root ?? throw new InvalidOperationException($"'{walkableAddPath}' has no root element.");

            foreach (var poly in addRoot.Elements("poly"))
            {
                walkablePolygons.Add(new WalkablePolygon(
                    Id: (string)poly.Attribute("id")!,
                    Type: (string?)poly.Attribute("type") ?? string.Empty,
                    Shape: ParseShape(poly.Attribute("shape"))));
            }

            foreach (var poi in addRoot.Elements("poi"))
            {
                accessPoints.Add(new WalkableAccessPoint(
                    Id: (string)poi.Attribute("id")!,
                    Type: (string?)poi.Attribute("type") ?? string.Empty,
                    Position: new Vec2(
                        ParseDouble((string)poi.Attribute("x")!),
                        ParseDouble((string)poi.Attribute("y")!))));
            }
        }

        return new PedNetwork(sidewalks, crossings, walkingAreas, walkablePolygons, accessPoints);
    }

    private static bool AllowsPedestrian(XElement lane)
    {
        var allow = (string?)lane.Attribute("allow");
        if (allow is null)
        {
            return false;
        }

        return allow.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries)
            .Contains("pedestrian", StringComparer.Ordinal);
    }

    private static double ParseWidth(XElement lane)
    {
        var width = (string?)lane.Attribute("width");
        return width is null ? 0.0 : ParseDouble(width);
    }

    private static string JunctionIdFromInternalEdgeId(string edgeId)
    {
        var match = JunctionFromInternalEdgeId.Match(edgeId);
        if (!match.Success)
        {
            throw new InvalidOperationException(
                $"Internal edge id '{edgeId}' does not match the ':<junction>_[cw]<N>' convention.");
        }

        return match.Groups["junction"].Value;
    }

    private static IReadOnlyList<Vec2> ParseShape(XAttribute? shapeAttr)
    {
        if (shapeAttr is null)
        {
            return Array.Empty<Vec2>();
        }

        var text = shapeAttr.Value;
        var tokens = text.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
        var points = new List<Vec2>(tokens.Length);

        foreach (var token in tokens)
        {
            var parts = token.Split(',');
            if (parts.Length < 2)
            {
                continue;
            }

            points.Add(new Vec2(ParseDouble(parts[0]), ParseDouble(parts[1])));
        }

        return points;
    }

    private static double ParseDouble(string s) =>
        double.Parse(s, NumberStyles.Float, CultureInfo.InvariantCulture);
}
