using Sim.Harness;
using Sim.Sumo;
using Xunit;

namespace Sim.ParityTests;

// Low-density spurious-teleport regression guard (docs/SUMOSHARP-LOWDENSITY-TELEPORT-DESIGN.md).
//
// The committed repro scenarios/_repro/synthetic-junction2 (an irregular synthetic net with a handful
// of TLS junctions on short approaches) runs uncongested (~124 peak concurrent) yet SumoSharp used to
// fire far more jam/yield teleports than vanilla SUMO 1.20.0 (vanilla = 0). Root cause (mechanism A):
// JunctionYieldConstraint decided right-of-way from the static netconvert <request> matrix, which is
// TL-blind, so a vehicle holding a protected-green ('G') traffic signal still yielded to junction foes
// and froze at the stop line until it hit time-to-teleport. The havePriority-aware gate
// (Engine.EgoLinkHasSignalPriority, wired into the cautious-approach / crossing-yield / sameTarget
// arms) restores the signal's authority: a 'G' movement yields to no one. That cut this scenario's
// teleports from 10 to 5.
//
// This is an ENGINE-ONLY, offline check (no SUMO): it drives the committed scenario through the same
// in-process SumoShim path the SumoData serve pipeline uses and reads the produced <teleports> count.
// The bound guards the mechanism-A fix from regressing (back toward 10). The 5 remaining teleports are
// a SEPARATE, pre-existing priority-junction on-junction wedge (mechanism B, tracked as task T3); when
// that lands this bound tightens toward vanilla's 0.
public class LowDensityTeleportTests
{
    [Fact]
    public void SyntheticJunction2_TlPriorityVehiclesDoNotSpuriouslyTeleport()
    {
        var scenarioDir = Path.Combine(RepoRoot(), "scenarios", "_repro", "synthetic-junction2");
        var cfg = Path.Combine(scenarioDir, "scenario.sumocfg");
        Assert.True(File.Exists(cfg), $"repro scenario missing: {cfg}");

        var outDir = Path.Combine(Path.GetTempPath(), "sumosharp-lowdens-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(outDir);
        try
        {
            var statistic = Path.Combine(outDir, "stat.xml");
            var exit = SumoShim.Run(
                new[]
                {
                    "-c", cfg,
                    "--statistic-output", statistic,
                    "--end", "2000",
                    "--no-step-log", "true",
                },
                new StringWriter(), new StringWriter());

            Assert.Equal(0, exit);

            var stats = StatisticOutputParser.Parse(statistic);

            // The havePriority fix drops this scenario from 10 -> 5. Guard the TLS half: any regression
            // in the signal-priority gate re-inflates the count toward 10, so cap it at the post-fix
            // level. (Vanilla SUMO fires 0 here; the residual 5 is mechanism B / task T3.)
            Assert.True(
                stats.TeleportsTotal <= 5,
                $"synthetic-junction2 fired {stats.TeleportsTotal} teleports (jam={stats.TeleportsJam}, " +
                $"yield={stats.TeleportsYield}); the havePriority junction-yield fix should hold it at <= 5 " +
                "(pre-fix was 10; vanilla SUMO is 0).");
        }
        finally
        {
            Directory.Delete(outDir, recursive: true);
        }
    }

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
