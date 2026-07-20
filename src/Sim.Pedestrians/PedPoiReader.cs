using System;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using Sim.Core.Orca;

namespace Sim.Pedestrians;

// P8-3 (docs/PEDESTRIAN-P8-3-POI-REQUEST.md): reads the sub-area pipeline's POI companion file
// (deduce_pois.py's `pois.json`) into PedPoi records for the auto-deduced ped demand. The JSON is
// `{ ..metadata.., pois: [ { id, kind, pos:[x,y], edge, weight, facing?:[nx,ny], capacity?, land_use? } ] }`
// in the box SUMO-metre frame. Pure input parsing (System.Text.Json), hermetically testable against the
// committed fixture. The SUMO-native `pois.add.xml` (same info as <poi>+<param>) is the companion the
// scenario.sumocfg references; this reader consumes the richer JSON directly.
public static class PedPoiReader
{
    public static IReadOnlyList<PedPoi> LoadJson(string poisJsonPath)
    {
        if (poisJsonPath is null)
        {
            throw new ArgumentNullException(nameof(poisJsonPath));
        }

        using var doc = JsonDocument.Parse(File.ReadAllText(poisJsonPath));
        var arr = doc.RootElement.GetProperty("pois");

        var result = new List<PedPoi>(arr.GetArrayLength());
        foreach (var p in arr.EnumerateArray())
        {
            var pos = p.GetProperty("pos");
            var poi = new PedPoi(
                Id: p.GetProperty("id").GetString() ?? throw new FormatException("POI missing id"),
                Kind: ParseKind(p.GetProperty("kind").GetString()),
                Pos: new Vec2(pos[0].GetDouble(), pos[1].GetDouble()),
                // Edge/weight are tolerant for pois/v2: the polygon-anchored kinds (parking_lot, park) may
                // carry no O/D `weight` (they are not sidewalk O/D endpoints) -- default 0 rather than throw.
                Edge: p.TryGetProperty("edge", out var e) && e.ValueKind == JsonValueKind.String
                    ? e.GetString()!
                    : throw new FormatException("POI missing edge"),
                Weight: p.TryGetProperty("weight", out var w) && w.ValueKind == JsonValueKind.Number
                    ? w.GetDouble()
                    : 0.0,
                Facing: p.TryGetProperty("facing", out var f) && f.ValueKind == JsonValueKind.Array
                    ? new Vec2(f[0].GetDouble(), f[1].GetDouble())
                    : null,
                Capacity: p.TryGetProperty("capacity", out var c) && c.ValueKind == JsonValueKind.Number
                    ? c.GetInt32()
                    : null,
                LandUse: p.TryGetProperty("land_use", out var lu) && lu.ValueKind == JsonValueKind.String
                    ? lu.GetString()
                    : null);

            if (poi.Weight < 0.0)
            {
                throw new FormatException($"POI {poi.Id} has negative weight {poi.Weight}");
            }

            result.Add(poi);
        }

        return result;
    }

    private static PedPoiKind ParseKind(string? kind) => kind switch
    {
        "building_entrance" => PedPoiKind.BuildingEntrance,
        "venue" => PedPoiKind.Venue,
        "dwell_spot" => PedPoiKind.DwellSpot,
        "transit_stop" => PedPoiKind.TransitStop,
        "parking_access" => PedPoiKind.ParkingAccess,
        "parking_lot" => PedPoiKind.ParkingLot, // pois/v2: tolerated as a base POI (R3 productization parked)
        "park" => PedPoiKind.Park,               // pois/v2: tolerated as a base POI (R5 park wiring parked)
        _ => throw new FormatException($"unknown POI kind '{kind}' (expected building_entrance|venue|dwell_spot|transit_stop|parking_access|parking_lot|park)"),
    };
}
