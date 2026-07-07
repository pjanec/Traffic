using System.Text.Json;
using Sim.Ingest;
using Xunit;

namespace Sim.ParityTests;

// Rung B2 -- network routing layer. Validates NetworkRouter (Dijkstra over the edge-connectivity
// graph, effort = free-flow travel time) ALONE, against the routing-diamond fixture, before B3
// wires it to a live reroute trigger (see DESIGN.md "Two futures" and NetworkRouter's own doc
// comment). This is standalone infrastructure -- it does not touch Engine.cs or any golden.fcd.xml
// parity path.
//
// Fixture: scenarios/_fixtures/routing-diamond/net.net.xml -- a diamond SA -> A -> {AB,AC} ->
// {B,C} -> {BD,CD} -> D -> DE, with the top path (AB+BD, 505.07 each) shorter than the bottom
// path (AC+CD, 634.63 each); all edges share speed 13.89 so min-time selection == min-length
// selection. scenarios/_fixtures/routing-diamond/routing-golden.json is the committed SUMO
// duarouter (DijkstraRouter, default travel-time effort) cross-check: the exact edge sequences
// duarouter chose for three FROM->TO queries.
public class RungB2RouterTests
{
    private static readonly string FixtureDir =
        Path.Combine(RepoRoot(), "scenarios", "_fixtures", "routing-diamond");

    private static NetworkModel LoadNetwork() =>
        NetworkParser.Parse(Path.Combine(FixtureDir, "net.net.xml"));

    private static NetworkRouter LoadRouter() => new(LoadNetwork());

    // Data-driven cross-check against routing-golden.json's "routes" map: for each "FROM->TO"
    // key, NetworkRouter.Route(FROM, TO) must reproduce duarouter's exact chosen edge sequence.
    public static IEnumerable<object[]> GoldenRouteCases()
    {
        var json = File.ReadAllText(Path.Combine(FixtureDir, "routing-golden.json"));
        using var doc = JsonDocument.Parse(json);
        var routes = doc.RootElement.GetProperty("routes");

        foreach (var entry in routes.EnumerateObject())
        {
            var parts = entry.Name.Split("->", 2);
            var from = parts[0];
            var to = parts[1];
            var expected = entry.Value.EnumerateArray().Select(e => e.GetString()!).ToArray();
            yield return new object[] { from, to, expected };
        }
    }

    [Theory]
    [MemberData(nameof(GoldenRouteCases))]
    public void Route_MatchesDuarouterGolden(string from, string to, string[] expected)
    {
        var router = LoadRouter();
        var actual = router.Route(from, to);

        Assert.NotNull(actual);
        Assert.Equal(expected, actual);
    }

    [Fact]
    public void Route_SameEdge_ReturnsSingletonPath()
    {
        var router = LoadRouter();
        Assert.Equal(new[] { "SA" }, router.Route("SA", "SA"));
    }

    [Fact]
    public void Route_NoBackwardConnection_IsUnreachable()
    {
        var router = LoadRouter();
        Assert.Null(router.Route("DE", "SA"));
    }

    [Fact]
    public void Route_UnknownEdge_ReturnsNull()
    {
        var router = LoadRouter();
        Assert.Null(router.Route("SA", "ZZ"));
    }

    // Turn-permission honored: the SA->CD golden route must take the bottom path (AC/CD) and
    // never touch the top path's edges, since there is no <connection from="AB" to="CD"> (or
    // any other AB/BD <-> AC/CD cross-link) in the net -- the router can only ever reach CD via
    // AC. This also exercises min-cost selection: SA->DE picks the top (AB/BD) path because it
    // is shorter (505.07x2 vs 634.63x2), confirming Dijkstra actually compares costs rather than
    // e.g. always preferring the first successor in file order.
    [Fact]
    public void Route_HonorsTurnPermissions_BottomPathNeverUsesTopEdges()
    {
        var router = LoadRouter();

        var bottom = router.Route("SA", "CD");
        Assert.NotNull(bottom);
        Assert.DoesNotContain("AB", bottom);
        Assert.DoesNotContain("BD", bottom);

        var top = router.Route("SA", "DE");
        Assert.NotNull(top);
        Assert.Contains("AB", top);
        Assert.Contains("BD", top);
        Assert.DoesNotContain("AC", top);
        Assert.DoesNotContain("CD", top);
    }

    // B3 prerequisite: the avoid-edges overload must force the detour (bottom path) when the
    // top path's BD edge is excluded, while the plain 2-arg overload is untouched (still picks
    // the shorter top path) -- proves the new overload is additive, not a behavior change to the
    // existing Route(from, to) call every other B2 test above exercises.
    [Fact]
    public void Route_WithAvoidSet_DetoursAroundExcludedEdge()
    {
        var router = LoadRouter();

        var avoiding = router.Route("SA", "DE", new HashSet<string> { "BD" });
        Assert.Equal(new[] { "SA", "AC", "CD", "DE" }, avoiding);

        var unavoided = router.Route("SA", "DE");
        Assert.Equal(new[] { "SA", "AB", "BD", "DE" }, unavoided);
    }

    // Mirrors EngineRung1PlumbingTests.RepoRoot(): resolve the repo root by walking up from the
    // test assembly's output directory to find Traffic.sln.
    private static string RepoRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "Traffic.sln")))
        {
            dir = dir.Parent;
        }

        return dir?.FullName
            ?? throw new InvalidOperationException("Could not locate repo root (Traffic.sln not found above test assembly).");
    }
}
