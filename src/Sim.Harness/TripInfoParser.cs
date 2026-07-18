using System.Globalization;
using System.Xml.Linq;

namespace Sim.Harness;

/// <summary>
/// VB-6: reads a SUMO-schema tripinfo file into <see cref="TripInfoRecord"/>s. Originally read
/// ONLY the subset of attributes the aggregate comparator needed (id, depart, duration,
/// arrivalSpeed) -- which is exactly why this same parser can read BOTH a real SUMO
/// <c>--tripinfo-output</c> file (root <c>&lt;tripinfos&gt;</c>, many more attributes: routeLength,
/// waitingTime, rerouteNo, devices, vType, speedFactor, vaporized, ...) AND the engine's tripinfo
/// ANALOG emitted by the VB-7 benchmark runner (same root/element/attribute names, only the subset
/// below populated). One schema, one loader, two producers -- see VIZ_BENCH_TASKS.md VB-6's "clear
/// input schema/loader for both" requirement.
///
/// GAP-2 (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §2) additionally parses <c>arrivalLane</c>,
/// <c>arrivalPos</c>, <c>arrival</c>, <c>routeLength</c>, <c>waitingTime</c>, <c>timeLoss</c> --
/// all TOLERATED-ABSENT (null when missing), so this parser still reads every pre-GAP-2 tripinfo
/// file (real SUMO's without those attributes populated, or the VB-7 4-field analog) unchanged.
///
/// <c>duration</c> is read directly if present (both SUMO and the engine analog always write it);
/// if a producer ever omits it, it is derived from <c>arrival - depart</c> as a fallback.
/// </summary>
public static class TripInfoParser
{
    public static IReadOnlyList<TripInfoRecord> Parse(string path)
    {
        using var stream = File.OpenRead(path);
        return Parse(stream);
    }

    public static IReadOnlyList<TripInfoRecord> Parse(Stream stream) => ParseDocument(XDocument.Load(stream));

    public static IReadOnlyList<TripInfoRecord> ParseXml(string xml) => ParseDocument(XDocument.Parse(xml));

    private static IReadOnlyList<TripInfoRecord> ParseDocument(XDocument doc)
    {
        var root = doc.Root ?? throw new InvalidDataException("tripinfo document has no root element.");

        var records = new List<TripInfoRecord>();
        foreach (var el in root.Elements("tripinfo"))
        {
            var id = RequireAttribute(el, "id");
            var depart = ParseDouble(el, "depart");
            var arrival = TryParseDouble(el, "arrival");
            var duration = TryParseDouble(el, "duration")
                ?? (arrival is { } arrivalForDuration ? arrivalForDuration - depart : throw new InvalidDataException(
                    $"<tripinfo id='{id}'> has neither 'duration' nor 'arrival' -- cannot derive trip duration."));
            var arrivalSpeed = TryParseDouble(el, "arrivalSpeed");
            var arrivalLane = el.Attribute("arrivalLane")?.Value;
            var arrivalPos = TryParseDouble(el, "arrivalPos");
            var routeLength = TryParseDouble(el, "routeLength");
            var waitingTime = TryParseDouble(el, "waitingTime");
            var timeLoss = TryParseDouble(el, "timeLoss");

            records.Add(new TripInfoRecord(
                id, depart, duration, arrivalSpeed,
                ArrivalLane: arrivalLane,
                ArrivalPos: arrivalPos,
                ArrivalTime: arrival,
                RouteLength: routeLength,
                WaitingTime: waitingTime,
                TimeLoss: timeLoss));
        }

        return records;
    }

    private static string RequireAttribute(XElement element, string name) =>
        element.Attribute(name)?.Value
        ?? throw new InvalidDataException($"<{element.Name}> is missing required attribute '{name}'.");

    private static double ParseDouble(XElement element, string name) =>
        double.Parse(RequireAttribute(element, name), CultureInfo.InvariantCulture);

    private static double? TryParseDouble(XElement element, string name)
    {
        var value = element.Attribute(name)?.Value;
        return value is null ? null : double.Parse(value, CultureInfo.InvariantCulture);
    }
}
