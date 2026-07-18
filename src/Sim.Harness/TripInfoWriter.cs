using System.Globalization;
using System.Xml;

namespace Sim.Harness;

/// <summary>
/// GAP-2 (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §2, docs/SERVE-PATH-PLAN.md): writes a SUMO-schema
/// <c>&lt;tripinfos&gt;</c> file from <see cref="TripInfoRecord"/>s -- the engine-CLI counterpart to
/// <see cref="TripInfoParser"/> (which already reads this exact schema). Used by
/// <c>Sim.Sumo.SumoShim</c> for <c>--tripinfo-output</c>.
///
/// Emits the GAP-2-required attribute subset only (id, depart, arrival, arrivalLane, arrivalPos,
/// arrivalSpeed, duration, routeLength, waitingTime, timeLoss -- docs/SUMOSHARP-SERVE-PATH-DROP-
/// IN.md §2's exact list) -- NOT the full real-SUMO attribute set (departLane/departPos/
/// departDelay/waitingCount/stopTime/rerouteNo/devices/vType/speedFactor/vaporized), which no
/// current consumer (the parity comparator, SumoData's audit_nocheat.py per the spec) needs. A
/// record missing an optional field simply omits that attribute (TripInfoParser already tolerates
/// absence both ways).
///
/// Row order: SUMO's own tripinfo-output is in ARRIVAL order (verified empirically against
/// scenarios/_bench/city-30/tripinfo.xml -- monotonically increasing `arrival`, not depart or id),
/// so this writer sorts by ArrivalTime ascending, falling back to Depart+Duration when ArrivalTime
/// is absent, with Id as the tie-break for equal arrival times.
/// </summary>
public static class TripInfoWriter
{
    public static void Write(string path, IReadOnlyList<TripInfoRecord> trips)
    {
        var ordered = trips
            .OrderBy(t => t.ArrivalTime ?? (t.Depart + t.Duration))
            .ThenBy(t => t.Id, StringComparer.Ordinal)
            .ToList();

        var settings = new XmlWriterSettings
        {
            Indent = true,
            IndentChars = "    ",
            Encoding = System.Text.Encoding.UTF8,
        };

        using var writer = XmlWriter.Create(path, settings);
        writer.WriteStartDocument();
        writer.WriteStartElement("tripinfos");
        writer.WriteAttributeString("xmlns", "xsi", null, "http://www.w3.org/2001/XMLSchema-instance");
        writer.WriteAttributeString("xsi", "noNamespaceSchemaLocation", null, "http://sumo.dlr.de/xsd/tripinfo_file.xsd");

        foreach (var t in ordered)
        {
            writer.WriteStartElement("tripinfo");
            writer.WriteAttributeString("id", t.Id);
            writer.WriteAttributeString("depart", Format(t.Depart));
            if (t.ArrivalTime is { } arrival)
            {
                writer.WriteAttributeString("arrival", Format(arrival));
            }

            if (t.ArrivalLane is { } arrivalLane)
            {
                writer.WriteAttributeString("arrivalLane", arrivalLane);
            }

            if (t.ArrivalPos is { } arrivalPos)
            {
                writer.WriteAttributeString("arrivalPos", Format(arrivalPos));
            }

            if (t.ArrivalSpeed is { } arrivalSpeed)
            {
                writer.WriteAttributeString("arrivalSpeed", Format(arrivalSpeed));
            }

            writer.WriteAttributeString("duration", Format(t.Duration));

            if (t.RouteLength is { } routeLength)
            {
                writer.WriteAttributeString("routeLength", Format(routeLength));
            }

            if (t.WaitingTime is { } waitingTime)
            {
                writer.WriteAttributeString("waitingTime", Format(waitingTime));
            }

            if (t.TimeLoss is { } timeLoss)
            {
                writer.WriteAttributeString("timeLoss", Format(timeLoss));
            }

            writer.WriteEndElement();
        }

        writer.WriteEndElement();
        writer.WriteEndDocument();
    }

    // "R" (round-trip) so the comparator sees full double precision -- matching FcdWriterObserver's
    // own convention for parity-sensitive numeric output (see that class for the precedent).
    private static string Format(double value) => value.ToString("R", CultureInfo.InvariantCulture);
}
