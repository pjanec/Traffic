using System.Globalization;
using System.Xml.Linq;

namespace Sim.Ingest;

// Parses the rung-1 subset of .rou.xml: <vType>, <route>, <vehicle>. Missing optional
// attributes fall back to documented SUMO defaults where the value is purely numeric/simple.
// P0-C1: departSpeed="max", departLane="best", and departPos="stop" (lane <stop> only) plus
// departPos="base" are now resolved to their DepartValue.cs spec Kind here (ParseDepartSpeed/
// ParseDepartLane/ParseDepartPos); their concrete placement/value is still resolved later, at
// insertion (Engine.cs). Every OTHER symbolic keyword (e.g. departPos="random"/"free"/"last"/
// "random_free"/"speedLimit", departLane="free"/"random"/"allowed"/"first", departSpeed="desired"/
// "random"/"avg") throws a clear InvalidDataException naming the attribute+value -- not silently
// mishandled, not a raw FormatException.
public static class DemandParser
{
    // C2-iv: SUMO's built-in default vType id (SUMOVTypeParameter's DEFAULT_VEHTYPE) -- the type a
    // <vehicle> with no type= falls back to.
    private const string DefaultVehTypeId = "DEFAULT_VEHTYPE";

    // P0-B: whitespace separators accepted by SUMO's StringTokenizer for vTypeDistribution's
    // vTypes=/probabilities= attribute lists.
    private static readonly char[] WhitespaceChars = { ' ', '\t', '\n', '\r' };

    public static DemandModel Parse(string path) => Parse(new[] { path });

    public static DemandModel ParseXml(string xml) => ParseDocument(XDocument.Parse(xml));

    // P0-A: multi-file <route-files>. Each path is parsed and its <vType>/<route>/<vehicle>/<flow>
    // elements are accumulated into ONE shared set of collections (mirroring SUMO's own behaviour
    // of loading all route-files into a single demand universe), and the DEFAULT_VEHTYPE synthesis
    // + final DemandModel assembly happen ONCE, after every file has been folded in. The single-
    // path case (`paths.Count == 1`) walks the exact same accumulate-then-assemble path as before
    // this rung, so it stays byte-identical.
    public static DemandModel Parse(IReadOnlyList<string> paths)
    {
        var acc = new Accumulator();
        foreach (var path in paths)
        {
            using var stream = File.OpenRead(path);
            AccumulateDocument(XDocument.Load(stream), acc);
        }

        return Assemble(acc);
    }

    private static DemandModel ParseDocument(XDocument doc)
    {
        var acc = new Accumulator();
        AccumulateDocument(doc, acc);
        return Assemble(acc);
    }

    // Shared mutable state folded into across one or more source documents. Kept as one small
    // class (rather than threading five separate ref/out params through AccumulateDocument) purely
    // for readability; it is never exposed outside this file.
    private sealed class Accumulator
    {
        public readonly List<VType> VTypes = new();
        public readonly Dictionary<string, VType> VTypesById = new();
        public readonly List<Route> Routes = new();
        public readonly Dictionary<string, Route> RoutesById = new();
        public readonly List<VehicleDef> Vehicles = new();
        public readonly List<ProbabilisticFlow> ProbFlows = new();
        public readonly Dictionary<string, VTypeDistribution> VTypeDistributionsById = new();
        public bool NeedsDefaultVehType;
    }

    private static void AccumulateDocument(XDocument doc, Accumulator acc)
    {
        var root = doc.Root ?? throw new InvalidDataException("rou.xml has no root element.");

        var vTypes = acc.VTypes;
        var vTypesById = acc.VTypesById;
        foreach (var vTypeEl in root.Elements("vType"))
        {
            var vType = ParseVTypeElement(vTypeEl);

            // P0-A: SUMO errors on a duplicate vType id -- whether the duplicate comes from the
            // same file or a second route-file in a multi-file <route-files> list, since both
            // land in the same shared vTypesById the parser assembles into one DemandModel.
            if (!vTypesById.TryAdd(vType.Id, vType))
            {
                throw new InvalidDataException($"duplicate <vType id='{vType.Id}'>.");
            }

            vTypes.Add(vType);
        }

        // P0-B: <vTypeDistribution id="..."> -- either the ATTRIBUTE form (`vTypes="a:0.7 b:0.3"`,
        // referencing vTypes declared elsewhere) or the NESTED form (child <vType probability=.../>
        // elements, which are ALSO folded into the same vTypesById set above so they resolve by id
        // like any other vType). Both forms normalise their raw weights to sum to 1 -- see
        // DemandModel.VTypeDistributionMember's own comment for why Probability is always
        // pre-normalised here rather than left as a raw SUMO weight.
        var vTypeDistributionsById = acc.VTypeDistributionsById;
        foreach (var vTypeDistEl in root.Elements("vTypeDistribution"))
        {
            var distId = RequireAttribute(vTypeDistEl, "id");
            var rawMembers = new List<VTypeDistributionMember>();

            var vTypesAttr = vTypeDistEl.Attribute("vTypes")?.Value;
            if (vTypesAttr is not null)
            {
                // REAL SUMO syntax (verified against the vendored source,
                // sumo/src/microsim/MSRouteHandler.cpp:246-288 openVehicleTypeDistribution):
                // `vTypes` is a whitespace-separated list of PLAIN ids; a separate, OPTIONAL
                // `probabilities` attribute carries the parallel whitespace-separated weight list,
                // index-aligned with vTypes (a vTypes entry past the end of a shorter/absent
                // probabilities list defaults to weight 1 -- MSRouteHandler.cpp:266,277's
                // `probs.size() > probIndex ? probs[probIndex] : 1.` fallback). We ALSO accept an
                // "id:weight" colon-embedded token as a harmless convenience extension (NOT itself
                // SUMO syntax) when no `probabilities` attribute is present, so either authoring
                // style parses.
                var vTypeTokens = vTypesAttr.Split(WhitespaceChars, StringSplitOptions.RemoveEmptyEntries);
                var probsAttr = vTypeDistEl.Attribute("probabilities")?.Value;
                if (probsAttr is not null)
                {
                    var probTokens = probsAttr.Split(WhitespaceChars, StringSplitOptions.RemoveEmptyEntries);
                    for (var i = 0; i < vTypeTokens.Length; i++)
                    {
                        var weight = i < probTokens.Length
                            ? double.Parse(probTokens[i], CultureInfo.InvariantCulture)
                            : 1.0;
                        rawMembers.Add(new VTypeDistributionMember(vTypeTokens[i], weight));
                    }
                }
                else
                {
                    foreach (var token in vTypeTokens)
                    {
                        var colonIdx = token.IndexOf(':');
                        if (colonIdx < 0)
                        {
                            rawMembers.Add(new VTypeDistributionMember(token, 1.0));
                        }
                        else
                        {
                            var memberId = token[..colonIdx];
                            var weight = double.Parse(token[(colonIdx + 1)..], CultureInfo.InvariantCulture);
                            rawMembers.Add(new VTypeDistributionMember(memberId, weight));
                        }
                    }
                }
            }

            // Nested form: <vType .../ probability="w"> children -- full vType parsing (same
            // attributes a top-level <vType> accepts), added to vTypesById/vTypes so a plain
            // direct-lookup by member id also works, plus its own probability= weight.
            foreach (var nestedVTypeEl in vTypeDistEl.Elements("vType"))
            {
                var nestedVType = ParseVTypeElement(nestedVTypeEl);
                if (!vTypesById.TryAdd(nestedVType.Id, nestedVType))
                {
                    throw new InvalidDataException($"duplicate <vType id='{nestedVType.Id}'>.");
                }

                vTypes.Add(nestedVType);
                var weight = ParseNullableDouble(nestedVTypeEl, "probability") ?? 1.0;
                rawMembers.Add(new VTypeDistributionMember(nestedVType.Id, weight));
            }

            if (rawMembers.Count == 0)
            {
                throw new InvalidDataException(
                    $"<vTypeDistribution id='{distId}'> has no members (neither vTypes= nor a nested <vType>).");
            }

            var totalWeight = 0.0;
            foreach (var m in rawMembers)
            {
                totalWeight += m.Probability;
            }

            if (!(totalWeight > 0.0))
            {
                throw new InvalidDataException($"<vTypeDistribution id='{distId}'> total weight must be > 0.");
            }

            var normalisedMembers = new List<VTypeDistributionMember>(rawMembers.Count);
            foreach (var m in rawMembers)
            {
                normalisedMembers.Add(new VTypeDistributionMember(m.VTypeId, m.Probability / totalWeight));
            }

            if (!vTypeDistributionsById.TryAdd(distId, new VTypeDistribution(distId, normalisedMembers)))
            {
                throw new InvalidDataException($"duplicate <vTypeDistribution id='{distId}'>.");
            }
        }

        var routes = acc.Routes;
        var routesById = acc.RoutesById;
        foreach (var routeEl in root.Elements("route"))
        {
            var route = new Route(
                Id: RequireAttribute(routeEl, "id"),
                Edges: RequireAttribute(routeEl, "edges").Split(' ', StringSplitOptions.RemoveEmptyEntries));

            // P0-A: same duplicate-id treatment as vType (see above), across the merged set.
            if (!routesById.TryAdd(route.Id, route))
            {
                throw new InvalidDataException($"duplicate <route id='{route.Id}'>.");
            }

            routes.Add(route);
        }

        var vehicles = acc.Vehicles;
        var probFlows = acc.ProbFlows;
        foreach (var vehicleEl in root.Elements("vehicle"))
        {
            var vehId = RequireAttribute(vehicleEl, "id");

            var stops = vehicleEl.Elements("stop")
                .Select(ParseStop)
                .ToList();

            // C2-iv: a vehicle's route is either a `route=` reference to a top-level <route id=...>
            // OR a nested <route edges="..."/> (duarouter's default EMBEDDED-route output). For the
            // embedded form, synthesize a named Route so the rest of the pipeline (route-by-id) is
            // unchanged -- keyed by a per-vehicle id (SUMO names an embedded route after its vehicle;
            // the '!' prefix keeps it from clashing with a user route id).
            var routeId = vehicleEl.Attribute("route")?.Value;
            if (routeId is null)
            {
                var embeddedRouteEl = vehicleEl.Element("route")
                    ?? throw new InvalidDataException(
                        $"<vehicle id='{vehId}'> has neither a route= attribute nor a nested <route>.");
                routeId = $"!{vehId}";
                var embeddedRoute = new Route(
                    routeId,
                    RequireAttribute(embeddedRouteEl, "edges").Split(' ', StringSplitOptions.RemoveEmptyEntries));
                routes.Add(embeddedRoute);
                routesById[routeId] = embeddedRoute;
            }

            // C2-iv: a <vehicle> with no type= uses SUMO's built-in DEFAULT_VEHTYPE (synthesized
            // after the loop), rather than throwing on an empty vType id.
            var typeId = vehicleEl.Attribute("type")?.Value;
            if (typeId is null)
            {
                typeId = DefaultVehTypeId;
                acc.NeedsDefaultVehType = true;
            }

            vehicles.Add(new VehicleDef(
                Id: vehId,
                TypeId: typeId,
                RouteId: routeId,
                Depart: ParseNullableDouble(vehicleEl, "depart") ?? 0.0,
                DepartPos: ParseDepartPos(vehicleEl, $"<vehicle id='{vehId}'>"),
                DepartSpeed: ParseDepartSpeed(vehicleEl, $"<vehicle id='{vehId}'>"),
                DepartLaneIndex: ParseDepartLane(vehicleEl, $"<vehicle id='{vehId}'>"),
                Stops: stops,
                // Phase 2 (sublane): SUMO's departPosLat vehicle attribute. Absent -> centre.
                DepartPosLat: vehicleEl.Attribute("departPosLat")?.Value));
        }

        // F1 (deterministic flow demand): a <flow> is a template that expands to many <vehicle>s
        // over [begin, end). Only the DETERMINISTIC forms are handled here (number / period /
        // vehsPerHour); the probabilistic `probability` form is a later statistical-parity rung and
        // is rejected loudly rather than silently mis-expanded. Expansion happens at LOAD (cold
        // path) into concrete VehicleDefs with ids "<flowId>.<k>" (SUMO's convention), which then
        // flow through the SAME depart-gated insertion path as hand-listed <vehicle>s -- so the
        // engine is untouched and any rou.xml with no <flow> is byte-identical to before this rung.
        foreach (var flowEl in root.Elements("flow"))
        {
            var flowId = RequireAttribute(flowEl, "id");
            var begin = ParseNullableDouble(flowEl, "begin") ?? 0.0;
            var end = ParseNullableDouble(flowEl, "end");
            var number = ParseNullableInt(flowEl, "number");
            var periodAttr = ParseNullableDouble(flowEl, "period");
            var vehsPerHour = ParseNullableDouble(flowEl, "vehsPerHour");
            var probability = ParseNullableDouble(flowEl, "probability");

            // Shared route/type/depart resolution -- identical for the deterministic and the
            // probabilistic forms (a probability flow is inserted per-step at runtime, but resolves
            // its route/type/depart placement exactly like a hand-listed <vehicle>).
            var resolvedRouteId = ResolveRouteId(flowEl, flowId, routes, routesById);
            string ResolveFlowType()
            {
                var t = flowEl.Attribute("type")?.Value;
                if (t is null)
                {
                    acc.NeedsDefaultVehType = true;
                    return DefaultVehTypeId;
                }

                return t;
            }

            var departPos = ParseDepartPos(flowEl, $"<flow id='{flowId}'>");
            var departSpeed = ParseDepartSpeed(flowEl, $"<flow id='{flowId}'>");
            var departLane = ParseDepartLane(flowEl, $"<flow id='{flowId}'>");

            // F2 (probabilistic flow): `probability=` is a per-second Bernoulli insertion, decided at
            // runtime by the engine from a per-flow seeded RNG -- NOT expanded here. It is mutually
            // exclusive with the deterministic rate forms (period / vehsPerHour / number).
            if (probability is double prob)
            {
                if (periodAttr is not null || vehsPerHour is not null || number is not null)
                {
                    throw new InvalidDataException(
                        $"<flow id='{flowId}'> combines probability= with period/vehsPerHour/number; use exactly one insertion form.");
                }

                if (prob <= 0.0 || prob > 1.0)
                {
                    throw new InvalidDataException($"<flow id='{flowId}'> probability must be in (0, 1].");
                }

                var probEnd = end ?? double.PositiveInfinity;
                if (probEnd <= begin)
                {
                    throw new InvalidDataException($"<flow id='{flowId}'> end must be > begin.");
                }

                probFlows.Add(new ProbabilisticFlow(
                    Id: flowId,
                    TypeId: ResolveFlowType(),
                    RouteId: resolvedRouteId,
                    Begin: begin,
                    End: probEnd,
                    Probability: prob,
                    DepartPos: departPos,
                    DepartSpeed: departSpeed,
                    DepartLaneIndex: departLane));
                continue;
            }

            // Resolve the insertion PERIOD (seconds between departs). Exactly one rate source.
            double period;
            if (periodAttr is double p)
            {
                if (p <= 0.0) throw new InvalidDataException($"<flow id='{flowId}'> period must be > 0.");
                period = p;
            }
            else if (vehsPerHour is double vph)
            {
                if (vph <= 0.0) throw new InvalidDataException($"<flow id='{flowId}'> vehsPerHour must be > 0.");
                period = 3600.0 / vph;
            }
            else if (number is int nEven && end is double eEven)
            {
                if (nEven <= 0) throw new InvalidDataException($"<flow id='{flowId}'> number must be > 0.");
                if (eEven <= begin) throw new InvalidDataException($"<flow id='{flowId}'> end must be > begin.");
                period = (eEven - begin) / nEven; // N vehicles spread evenly over [begin, end)
            }
            else
            {
                throw new InvalidDataException(
                    $"<flow id='{flowId}'> needs one of period, vehsPerHour, or number+end.");
            }

            if (number is null && end is null)
            {
                throw new InvalidDataException($"<flow id='{flowId}'> needs a bound: number or end.");
            }

            var flowRouteId = resolvedRouteId;
            var flowTypeId = ResolveFlowType();
            var flowDepartPos = departPos;
            var flowDepartSpeed = departSpeed;
            var flowDepartLane = departLane;

            for (var k = 0; ; k++)
            {
                if (number is int n && k >= n)
                {
                    break;
                }

                var depart = begin + k * period;
                // Half-open [begin, end): the eps keeps a depart landing exactly on `end` out (and
                // absorbs float drift in begin + k*period). Only applies when `number` is unbounded.
                if (number is null && end is double e && depart >= e - 1e-9)
                {
                    break;
                }

                // Backstop against a misconfigured unbounded/huge flow (never a real scenario).
                if (k > 1_000_000)
                {
                    throw new InvalidDataException($"<flow id='{flowId}'> expands to over 1,000,000 vehicles.");
                }

                vehicles.Add(new VehicleDef(
                    Id: $"{flowId}.{k}",
                    TypeId: flowTypeId,
                    RouteId: flowRouteId,
                    Depart: depart,
                    DepartPos: flowDepartPos,
                    DepartSpeed: flowDepartSpeed,
                    DepartLaneIndex: flowDepartLane));
            }
        }
    }

    // Runs ONCE, after every source document has been folded into `acc` -- the DEFAULT_VEHTYPE
    // synthesis and the final DemandModel construction must not repeat per file (that would, e.g.,
    // re-add DEFAULT_VEHTYPE or duplicate it across a multi-file merge).
    private static DemandModel Assemble(Accumulator acc)
    {
        // C2-iv: synthesize DEFAULT_VEHTYPE when referenced-but-undeclared -- SUMOVTypeParameter's
        // built-in default (vClass passenger, every param at its class default incl. sigma 0.5;
        // VTypeDefaults.Resolve fills the nulls). If the user DID declare their own DEFAULT_VEHTYPE,
        // that one is already in the map and wins.
        if (acc.NeedsDefaultVehType && !acc.VTypesById.ContainsKey(DefaultVehTypeId))
        {
            var defaultVehType = new VType(Id: DefaultVehTypeId, VClass: null, Sigma: null);
            acc.VTypes.Add(defaultVehType);
            acc.VTypesById[DefaultVehTypeId] = defaultVehType;
        }

        return new DemandModel(
            acc.VTypes, acc.VTypesById, acc.Routes, acc.RoutesById, acc.Vehicles, acc.ProbFlows,
            acc.VTypeDistributionsById);
    }

    // Extracted so P0-B's nested <vTypeDistribution><vType probability=.../></vTypeDistribution>
    // members parse through the EXACT same attribute set as a top-level <vType> (the only
    // difference is the caller: the top-level loop above, or the nested-vType loop inside the
    // vTypeDistribution loop below).
    private static VType ParseVTypeElement(XElement vTypeEl) => new(
        Id: RequireAttribute(vTypeEl, "id"),
        VClass: vTypeEl.Attribute("vClass")?.Value,
        Sigma: ParseNullableDouble(vTypeEl, "sigma"),
        MaxSpeed: ParseNullableDouble(vTypeEl, "maxSpeed"),
        Accel: ParseNullableDouble(vTypeEl, "accel"),
        Decel: ParseNullableDouble(vTypeEl, "decel"),
        Tau: ParseNullableDouble(vTypeEl, "tau"),
        MinGap: ParseNullableDouble(vTypeEl, "minGap"),
        Length: ParseNullableDouble(vTypeEl, "length"),
        EmergencyDecel: ParseNullableDouble(vTypeEl, "emergencyDecel"),
        SpeedFactor: ParseNullableDouble(vTypeEl, "speedFactor"),
        // Rung A3: a vType ATTRIBUTE, not a <param> child -- SUMO's getJMParam reads the
        // attribute map (SUMOVTypeParameter's map of junction-model params populated
        // straight from the <vType>'s own XML attributes for jm* names).
        JmDriveAfterRedTime: ParseNullableDouble(vTypeEl, "jmDriveAfterRedTime"),
        // Rung ER2: emergency ignore-FOE junction-model attributes, read from the <vType>'s
        // own XML attribute map exactly like jmDriveAfterRedTime (SUMO's getJMParam).
        JmIgnoreFoeProb: ParseNullableDouble(vTypeEl, "jmIgnoreFoeProb"),
        JmIgnoreFoeSpeed: ParseNullableDouble(vTypeEl, "jmIgnoreFoeSpeed"),
        JmIgnoreJunctionFoeProb: ParseNullableDouble(vTypeEl, "jmIgnoreJunctionFoeProb"),
        // Rung ER3 (give-way): whether this vType carries an active blue-light siren (our
        // opt-in model of SUMO's MSDevice_Bluelight assignment, `has.bluelight.device`).
        // Default false -> no give-way is ever induced, so every existing scenario (incl.
        // the emergency-privilege scenarios 16/50/51/52, whose EVs set NO bluelight) is
        // byte-identical. Read as a vType attribute `hasBluelight="true"`.
        HasBluelight: ParseNullableBool(vTypeEl, "hasBluelight"),
        // Rung OV1: opt-in opposite-direction overtaking (SUMO's lcOpposite analog).
        LcOpposite: ParseNullableBool(vTypeEl, "lcOpposite"),
        // C11-i: SUMOVTypeParameter.cpp's carFollowModel="..." vType attribute (a plain
        // string tag name -- "Krauss", "IDM", etc. -- SUMOXMLDefinitions::CarFollowModels).
        CarFollowModel: vTypeEl.Attribute("carFollowModel")?.Value,
        // Rung R6: MSCFModel_Rail cf-params, read straight from the <vType> attributes (the
        // same attribute map SUMO's getCFParam/getCFParamString reads). Absent for every
        // non-Rail vType.
        TrainType: vTypeEl.Attribute("trainType")?.Value,
        MaxPower: ParseNullableDouble(vTypeEl, "maxPower"),
        MaxTraction: ParseNullableDouble(vTypeEl, "maxTraction"),
        ResCoefConstant: ParseNullableDouble(vTypeEl, "resCoef_constant"),
        ResCoefLinear: ParseNullableDouble(vTypeEl, "resCoef_linear"),
        ResCoefQuadratic: ParseNullableDouble(vTypeEl, "resCoef_quadratic"),
        MassFactor: ParseNullableDouble(vTypeEl, "massFactor"),
        Mass: ParseNullableDouble(vTypeEl, "mass"),
        // Phase 2 (sublane): SUMO's maxSpeedLat / latAlignment vType attributes. Absent
        // for every phase-1 vType -> resolve to SUMO defaults (1.0 / "center") in
        // VTypeDefaults.Resolve; inert unless the scenario sets lateral-resolution > 0.
        MaxSpeedLat: ParseNullableDouble(vTypeEl, "maxSpeedLat"),
        LatAlignment: vTypeEl.Attribute("latAlignment")?.Value,
        // Rung P2-core (keepLatGap): SUMO's minGapLat vType attribute. Absent for every
        // pre-existing vType -> resolves to SUMO's 0.6 default in VTypeDefaults.Resolve;
        // inert unless lateral-resolution > 0.
        MinGapLat: ParseNullableDouble(vTypeEl, "minGapLat"));

    private static double? ParseNullableDouble(XElement element, string name)
    {
        var value = element.Attribute(name)?.Value;
        return value is null ? null : double.Parse(value, CultureInfo.InvariantCulture);
    }

    // Rung ER3: SUMO's boolean attribute forms (utils/common/StringUtils::toBool accepts
    // true/1/x/t/yes and false/0/-/f/no). null when the attribute is absent (-> the vType-default
    // false in VTypeDefaults.Resolve).
    private static bool? ParseNullableBool(XElement element, string name)
    {
        var value = element.Attribute(name)?.Value;
        if (value is null)
        {
            return null;
        }

        return value.Trim().ToLowerInvariant() is "true" or "1" or "x" or "t" or "yes";
    }

    private static int? ParseNullableInt(XElement element, string name)
    {
        var value = element.Attribute(name)?.Value;
        return value is null ? null : int.Parse(value, CultureInfo.InvariantCulture);
    }

    // P0-C1: SUMO's parseDepartSpeed (SUMOVehicleParameter-parsing helpers) -- a numeric literal
    // parses as Given (identical to the old ParseNullableDouble ?? 0.0 path -- see DepartValue.cs'
    // header comment), "max" resolves to Max (SUMOVehicleParameter.h's DepartSpeedDefinition::MAX),
    // and any OTHER symbolic keyword (e.g. "desired"/"random"/"avg") is loudly rejected -- named
    // consumers are not yet in scope, not silently mishandled as 0 or crashed as a raw
    // FormatException.
    private static DepartSpeedValue ParseDepartSpeed(XElement element, string ownerDesc)
    {
        var value = element.Attribute("departSpeed")?.Value;
        if (value is null)
        {
            return DepartSpeedValue.Given(0.0);
        }

        if (double.TryParse(value, NumberStyles.Float | NumberStyles.AllowThousands, CultureInfo.InvariantCulture, out var literal))
        {
            return DepartSpeedValue.Given(literal);
        }

        if (value == "max")
        {
            return DepartSpeedValue.Max;
        }

        throw new InvalidDataException(
            $"{ownerDesc} has unsupported departSpeed=\"{value}\" (only a numeric literal or \"max\" is supported).");
    }

    // P0-C1: SUMO's parseDepartLane -- a numeric literal parses as Given, "best" resolves to Best
    // (DepartLaneDefinition::BEST_FREE), any other symbolic keyword (e.g. "free"/"random"/
    // "allowed"/"first") is loudly rejected.
    private static DepartLaneValue ParseDepartLane(XElement element, string ownerDesc)
    {
        var value = element.Attribute("departLane")?.Value;
        if (value is null)
        {
            return DepartLaneValue.Given(0);
        }

        if (int.TryParse(value, NumberStyles.Integer, CultureInfo.InvariantCulture, out var literal))
        {
            return DepartLaneValue.Given(literal);
        }

        if (value == "best")
        {
            return DepartLaneValue.Best;
        }

        throw new InvalidDataException(
            $"{ownerDesc} has unsupported departLane=\"{value}\" (only a numeric index or \"best\" is supported).");
    }

    // P0-C1: SUMO's parseDepartPos -- a numeric literal parses as Given, "stop" resolves to Stop
    // (DepartPosDefinition::STOP, lane <stop> only -- see DepartValue.cs), any other symbolic
    // keyword (e.g. "random"/"free"/"base"/"last"/"random_free"/"speedLimit") is loudly rejected.
    private static DepartPosValue ParseDepartPos(XElement element, string ownerDesc)
    {
        var value = element.Attribute("departPos")?.Value;
        if (value is null)
        {
            return DepartPosValue.Given(0.0);
        }

        if (double.TryParse(value, NumberStyles.Float | NumberStyles.AllowThousands, CultureInfo.InvariantCulture, out var literal))
        {
            return DepartPosValue.Given(literal);
        }

        if (value == "stop")
        {
            return DepartPosValue.Stop;
        }

        // DepartPosDefinition::BASE -- resolved at insertion to MSBaseVehicle::basePos (see
        // DepartValue.cs / Engine.TryInsertOnLane). This is also SUMO's DEFAULT departPos, so a
        // vehicle with no departPos attribute at all behaves identically; we keep the null case as
        // Given(0.0) for byte-identical parity with every pre-existing golden (none of which used
        // "base" or relied on the >0 basePos offset) and resolve "base" explicitly here.
        if (value == "base")
        {
            return DepartPosValue.Base;
        }

        throw new InvalidDataException(
            $"{ownerDesc} has unsupported departPos=\"{value}\" (only a numeric literal, \"stop\", or \"base\" is supported; " +
            "\"random\"/\"free\"/\"last\"/\"random_free\"/\"speedLimit\" are not yet ported).");
    }

    // Rung 5 + P0-C2: a <stop> is EITHER a plain lane stop (`lane=` with startPos/endPos/duration)
    // OR a parkingArea stop (`parkingArea=` with NO lane -- the lane/endPos are resolved later from
    // the parkingArea registry, in Engine.LoadScenario). A <stop> with neither `lane=` nor
    // `parkingArea=` is rejected loudly (SUMO likewise requires a stopping-place reference).
    private static StopDef ParseStop(XElement stopEl)
    {
        var parkingAreaId = stopEl.Attribute("parkingArea")?.Value;
        if (parkingAreaId is not null)
        {
            // Placeholder lane/pos: DemandParser (route-files) can't see the additional-file that
            // declares the parkingArea. Engine.ResolveParkingAreaStops fills LaneId/StartPos/EndPos
            // from the registry after both are parsed.
            return new StopDef(
                LaneId: string.Empty,
                StartPos: 0.0,
                EndPos: 0.0,
                Duration: ParseNullableDouble(stopEl, "duration") ?? 0.0,
                ParkingAreaId: parkingAreaId);
        }

        if (stopEl.Attribute("lane")?.Value is null)
        {
            throw new InvalidDataException(
                "<stop> requires either a 'lane' attribute (lane stop) or a 'parkingArea' attribute " +
                "(parkingArea stop); neither was present.");
        }

        return new StopDef(
            LaneId: RequireAttribute(stopEl, "lane"),
            StartPos: ParseNullableDouble(stopEl, "startPos") ?? 0.0,
            EndPos: ParseNullableDouble(stopEl, "endPos") ?? 0.0,
            Duration: ParseNullableDouble(stopEl, "duration") ?? 0.0);
    }

    private static string RequireAttribute(XElement element, string name) =>
        element.Attribute(name)?.Value
        ?? throw new InvalidDataException($"<{element.Name}> is missing required attribute '{name}'.");

    // F1: resolve a <vehicle>/<flow>'s route -- either a `route=` reference to a top-level
    // <route id=...> or a nested <route edges="..."/> (the embedded form, synthesized into a named
    // route keyed "!<ownerId>" so route-by-id lookups downstream are unchanged). Mirrors the
    // inline <vehicle> route logic; factored so <flow> resolves routes identically.
    private static string ResolveRouteId(
        XElement el, string ownerId, List<Route> routes, Dictionary<string, Route> routesById)
    {
        var routeId = el.Attribute("route")?.Value;
        if (routeId is not null)
        {
            return routeId;
        }

        var embeddedRouteEl = el.Element("route")
            ?? throw new InvalidDataException(
                $"<{el.Name.LocalName} id='{ownerId}'> has neither a route= attribute nor a nested <route>.");
        routeId = $"!{ownerId}";
        var embeddedRoute = new Route(
            routeId,
            RequireAttribute(embeddedRouteEl, "edges").Split(' ', StringSplitOptions.RemoveEmptyEntries));
        routes.Add(embeddedRoute);
        routesById[routeId] = embeddedRoute;
        return routeId;
    }
}
