using Sim.Ingest;
using Xunit;

namespace Sim.ParityTests;

// Rung 9b-i: junction right-of-way data + conflict geometry ingest. INERT increment -- this
// test only exercises NetworkParser's new <junction> parsing (Links/Requests/Conflicts); it
// makes no yielding decision and touches no engine/reducer code (that is rung 9b-ii/iii).
//
// Fixture: scenarios/11-priority-junction/net.net.xml, junction "J" -- a priority junction
// where SJ (minor, priority=1) and WJ (major, priority=10) cross. intLanes order IS link-index
// order: link 0 = :J_0_0 (SJ->JE right turn), link 1 = :J_1_0 (SJ->JN straight, minor), link 2
// = :J_2_0 (WJ->JE straight, major), link 3 = :J_3_0 (WJ->JN left turn, major). :J_1_0 (vertical,
// x=201.60) and :J_2_0 (horizontal, y=198.40) cross at (201.60, 198.40), 5.60 m along each
// (both are straight 11.20 m internal lanes, so the crossing sits exactly at midspan).
public class Rung9biJunctionGeometryTests
{
    private const double Tolerance = 1e-6;

    private static NetworkModel LoadNetwork() =>
        NetworkParser.Parse(Path.Combine(RepoRoot(), "scenarios", "11-priority-junction", "net.net.xml"));

    [Fact]
    public void Junction_HasExpectedTypeAndIntLaneOrder()
    {
        var network = LoadNetwork();
        var junction = network.JunctionsById["J"];

        Assert.Equal("priority", junction.Type);
        Assert.Equal(new[] { ":J_0_0", ":J_1_0", ":J_2_0", ":J_3_0" }, junction.IntLanes);
    }

    [Fact]
    public void Links_ResolveToTheirConnections()
    {
        var junction = LoadNetwork().JunctionsById["J"];

        var link1 = junction.Links.Single(l => l.Index == 1);
        Assert.Equal(":J_1_0", link1.InternalLaneId);
        Assert.Equal("SJ", link1.Connection.From);
        Assert.Equal("JN", link1.Connection.To);

        var link2 = junction.Links.Single(l => l.Index == 2);
        Assert.Equal(":J_2_0", link2.InternalLaneId);
        Assert.Equal("WJ", link2.Connection.From);
        Assert.Equal("JE", link2.Connection.To);
    }

    [Fact]
    public void Requests_MinorYieldsToMajor_NotViceVersa()
    {
        var junction = LoadNetwork().JunctionsById["J"];
        var requestsByIndex = junction.Requests.ToDictionary(r => r.Index);

        // Link 1 (SJ->JN, minor straight) must yield to link 2 (WJ->JE, major straight).
        Assert.True(requestsByIndex[1].RespondsTo(2));
        // Link 2 (major) yields to nobody.
        Assert.False(requestsByIndex[2].RespondsTo(1));
    }

    [Fact]
    public void Conflicts_ContainTheLink1Link2Crossing_BothDirections()
    {
        var junction = LoadNetwork().JunctionsById["J"];

        var forward = Assert.Single(junction.Conflicts, c => c.EgoLink == 1 && c.FoeLink == 2);
        Assert.Equal(5.60, forward.EgoCrossingArc, Tolerance);
        Assert.Equal(5.60, forward.FoeCrossingArc, Tolerance);
        Assert.Equal(201.60, forward.CrossingPoint.X, Tolerance);
        Assert.Equal(198.40, forward.CrossingPoint.Y, Tolerance);

        var backward = Assert.Single(junction.Conflicts, c => c.EgoLink == 2 && c.FoeLink == 1);
        Assert.Equal(5.60, backward.EgoCrossingArc, Tolerance);
        Assert.Equal(5.60, backward.FoeCrossingArc, Tolerance);
        Assert.Equal(201.60, backward.CrossingPoint.X, Tolerance);
        Assert.Equal(198.40, backward.CrossingPoint.Y, Tolerance);
    }

    // Rung 9b-ii: MSLink::setRequestInformation's crossing-width computation
    // (sumo/src/microsim/MSLink.cpp:354-382) for the link1<->link2 (:J_1_0 <-> :J_2_0)
    // crossing -- both internal lanes are 3.2m wide (default, no `width` attribute in this
    // net's <lane>s) and cross at a perpendicular (90 degree) angle, so widthFactor == 1 and
    // the conflict size collapses to exactly each lane's own width: EgoConflictSize ==
    // FoeConflictSize == 3.2, and both lanes' LengthBehindCrossing == 11.20 - (5.60 - 1.6) ==
    // 7.20 (raw crossing arc 5.60, shifted back by half the 3.2m conflict size).
    [Fact]
    public void Conflicts_HaveExpectedConflictSizeAndLengthBehindCrossing()
    {
        var junction = LoadNetwork().JunctionsById["J"];

        var forward = Assert.Single(junction.Conflicts, c => c.EgoLink == 1 && c.FoeLink == 2);
        Assert.Equal(3.2, forward.EgoConflictSize, Tolerance);
        Assert.Equal(3.2, forward.FoeConflictSize, Tolerance);
        Assert.Equal(7.20, forward.EgoLengthBehindCrossing, Tolerance);
        Assert.Equal(7.20, forward.FoeLengthBehindCrossing, Tolerance);

        var backward = Assert.Single(junction.Conflicts, c => c.EgoLink == 2 && c.FoeLink == 1);
        Assert.Equal(3.2, backward.EgoConflictSize, Tolerance);
        Assert.Equal(3.2, backward.FoeConflictSize, Tolerance);
        Assert.Equal(7.20, backward.EgoLengthBehindCrossing, Tolerance);
        Assert.Equal(7.20, backward.FoeLengthBehindCrossing, Tolerance);
    }

    [Fact]
    public void LinkByInternalLane_FindsLink1()
    {
        var network = LoadNetwork();

        var (junction, link) = network.LinkByInternalLane[":J_1_0"];
        Assert.Equal("J", junction.Id);
        Assert.Equal(1, link.Index);
    }

    // Mirrors EngineRung1PlumbingTests.RepoRoot(): resolve the repo root by walking up from the
    // test assembly's runtime location, so the test does not depend on the working directory
    // `dotnet test` happens to be invoked from.
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
