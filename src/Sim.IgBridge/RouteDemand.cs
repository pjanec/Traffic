using System.Globalization;
using System.Xml.Linq;

namespace Sim.IgBridge;

// One vehicle to spawn: a stable id, a depart time (sim seconds), the explicit edge-list route, and the
// requested depart lane (best/index) if the demand named one.
public sealed class RouteDemandEntry
{
    public RouteDemandEntry(string id, double depart, IReadOnlyList<string> edges, string? departLane)
    {
        Id = id;
        Depart = depart;
        Edges = edges;
        DepartLane = departLane;
    }

    public string Id { get; }
    public double Depart { get; }
    public IReadOnlyList<string> Edges { get; }
    public string? DepartLane { get; }
}

// Parses a SUMO .rou.xml into explicit-route vehicle spawns for the engine's
// SpawnVehicle(type, routeEdges, ...) API -- deliberately BYPASSING Sim.Ingest.DemandParser, which
// (correctly, for parity) rejects the demo_city/box demand's departPos="base", parkingArea <stop>s and
// vType distributions. IgBridge is a motion-smoothing source, not a parity run: it only needs each
// vehicle's id, depart time and edge path to reproduce the real city's junction turns / lane changes.
// Stops/parking are ignored (a vehicle simply drives its route and arrives). Deterministic: the result
// is sorted by (depart, id) so spawn order is a pure function of the file.
public static class RouteDemand
{
    private static readonly char[] EdgeSeparators = { ' ', '\t', '\n', '\r' };

    public static IReadOnlyList<RouteDemandEntry> Parse(string rouXmlPath)
    {
        var doc = XDocument.Load(rouXmlPath);
        var list = new List<RouteDemandEntry>();

        foreach (var v in doc.Descendants("vehicle"))
        {
            var id = (string?)v.Attribute("id");
            if (id is null)
            {
                continue;
            }

            // Only inline-route vehicles are supported (demo_city/box uses these). A <vehicle route="id">
            // referencing a standalone <route> is skipped -- none appear in the target scenario.
            var routeEl = v.Element("route");
            var edgesAttr = (string?)routeEl?.Attribute("edges");
            if (edgesAttr is null)
            {
                continue;
            }

            var edges = edgesAttr.Split(EdgeSeparators, StringSplitOptions.RemoveEmptyEntries);
            if (edges.Length == 0)
            {
                continue;
            }

            list.Add(new RouteDemandEntry(id, ParseDepart((string?)v.Attribute("depart")), edges,
                (string?)v.Attribute("departLane")));
        }

        // Deterministic spawn order: by depart, ties broken by ordinal id.
        list.Sort((a, b) => a.Depart != b.Depart
            ? a.Depart.CompareTo(b.Depart)
            : string.CompareOrdinal(a.Id, b.Id));
        return list;
    }

    // Symbolic depart values ("triggered"/"now"/…) collapse to 0.0 -- IgBridge spawns them at run start;
    // this is a smoothing source, not a parity replay of SUMO's insertion scheduler.
    private static double ParseDepart(string? s)
        => double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var d) ? d : 0.0;
}
