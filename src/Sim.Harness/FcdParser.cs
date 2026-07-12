using System.Globalization;
using System.Xml.Linq;
using Sim.Core;

namespace Sim.Harness;

public static class FcdParser
{
    public static TrajectorySet Parse(string path)
    {
        using var stream = File.OpenRead(path);
        return Parse(stream);
    }

    public static TrajectorySet Parse(Stream stream) => ParseDocument(XDocument.Load(stream));

    public static TrajectorySet ParseXml(string xml) => ParseDocument(XDocument.Parse(xml));

    private static TrajectorySet ParseDocument(XDocument doc)
    {
        var root = doc.Root ?? throw new InvalidDataException("FCD document has no root element.");
        var set = new TrajectorySet();

        foreach (var timestepEl in root.Elements("timestep"))
        {
            var time = double.Parse(RequireAttribute(timestepEl, "time"), CultureInfo.InvariantCulture);

            foreach (var vehicleEl in timestepEl.Elements("vehicle"))
            {
                set.Add(new TrajectoryPoint(
                    VehicleId: RequireAttribute(vehicleEl, "id"),
                    Time: time,
                    Lane: RequireAttribute(vehicleEl, "lane"),
                    Pos: ParseDouble(vehicleEl, "pos"),
                    Speed: ParseDouble(vehicleEl, "speed"),
                    X: ParseDouble(vehicleEl, "x"),
                    Y: ParseDouble(vehicleEl, "y"),
                    Angle: ParseDouble(vehicleEl, "angle"),
                    Acceleration: ParseNullableDouble(vehicleEl, "acceleration"))
                {
                    // Phase 2 (sublane): SUMO emits `posLat` in FCD only when
                    // --lateral-resolution > 0; absent (every phase-1 golden) => 0, matching a
                    // lane-centred vehicle. Optional so existing goldens parse unchanged.
                    PosLat = ParseNullableDouble(vehicleEl, "posLat") ?? 0.0,
                });
            }
        }

        return set;
    }

    private static string RequireAttribute(XElement element, string name) =>
        element.Attribute(name)?.Value
        ?? throw new InvalidDataException($"<{element.Name}> is missing required attribute '{name}'.");

    private static double ParseDouble(XElement element, string name) =>
        double.Parse(RequireAttribute(element, name), CultureInfo.InvariantCulture);

    private static double? ParseNullableDouble(XElement element, string name)
    {
        var value = element.Attribute(name)?.Value;
        return value is null ? null : double.Parse(value, CultureInfo.InvariantCulture);
    }
}
