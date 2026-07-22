using System.Text;
using System.Text.Json;
using Sim.IgBridge;
using Xunit;

namespace Sim.IgBridge.Tests;

// T1.2 (docs/IGBRIDGE-TASKS.md): the emit stage resamples the reused reconstruction stack at the emit
// cadence and writes IG-native new/upd/del records. These pin (a) valid, lifecycle-correct JSONL -- every
// entity has exactly one `new` before its first `upd` and one `del` after its last, and no record after a
// `del`; and (b) byte-deterministic emission across two independent runs.
public sealed class IgBridgeSessionTests
{
    private const int Steps = 150; // 15 s @ 10 Hz

    private static IgBridgeConfig BoxConfig()
    {
        var box = Path.Combine(RepoRoot(), "scenarios", "_ped", "demo_city", "box");
        return new IgBridgeConfig(Path.Combine(box, "net.xml"), Path.Combine(box, "scenario.rou.xml"))
        {
            StepLength = 0.1,
            Seed = 42,
        };
    }

    private static string RunToTrace(int steps)
    {
        var sw = new StringWriter();
        var runner = new IgBridgeRunner(BoxConfig());
        var session = new IgBridgeSession(runner, new IgEmitConfig { EmitHz = 20.0, LookaheadSeconds = 0.1 },
            new IgTraceWriter(sw));
        for (var i = 0; i < steps; i++)
        {
            runner.Tick();
            session.Advance();
        }

        session.Finish();
        return sw.ToString();
    }

    [Fact]
    public void Emit_IsDeterministic_TwoRunsProduceByteIdenticalTrace()
    {
        var a = RunToTrace(Steps);
        var b = RunToTrace(Steps);

        Assert.False(string.IsNullOrEmpty(a));
        Assert.Equal(a, b);
    }

    [Fact]
    public void Emit_TraceIsLifecycleCorrectAndNonVacuous()
    {
        var trace = RunToTrace(Steps);
        var lines = trace.Split('\n', StringSplitOptions.RemoveEmptyEntries);
        Assert.True(lines.Length > 1000, $"only {lines.Length} records emitted");

        var seenNew = new HashSet<string>();
        var seenUpd = new HashSet<string>();
        var deleted = new HashSet<string>();
        int news = 0, dels = 0, upds = 0;
        double lastT = double.NegativeInfinity;

        foreach (var line in lines)
        {
            using var doc = JsonDocument.Parse(line);
            var root = doc.RootElement;
            var kind = root.GetProperty("k").GetString()!;
            var id = root.GetProperty("id").GetString()!;
            var t = root.GetProperty("t").GetDouble();

            Assert.True(t >= lastT - 1e-9, $"timestamps must be non-decreasing (id={id} t={t} < {lastT})");
            lastT = t;

            switch (kind)
            {
                case "new":
                    Assert.False(seenUpd.Contains(id), $"`new` after `upd` for {id}");
                    Assert.False(seenNew.Contains(id), $"duplicate `new` for {id}");
                    Assert.True(root.TryGetProperty("model", out _), "`new` must carry a model");
                    seenNew.Add(id);
                    news++;
                    break;
                case "upd":
                    Assert.True(seenNew.Contains(id), $"`upd` before `new` for {id}");
                    Assert.False(deleted.Contains(id), $"`upd` after `del` for {id}");
                    seenUpd.Add(id);
                    upds++;
                    break;
                case "del":
                    Assert.True(seenNew.Contains(id), $"`del` before `new` for {id}");
                    Assert.False(deleted.Contains(id), $"duplicate `del` for {id}");
                    deleted.Add(id);
                    dels++;
                    break;
                default:
                    Assert.Fail($"unknown record kind '{kind}'");
                    break;
            }
        }

        // Finish() closes every opened entity, so new == del; and there must be real motion (many upds).
        Assert.Equal(news, dels);
        Assert.True(news >= 10, $"only {news} entities emitted");
        Assert.True(upds > news * 5, "expected many upd records per entity");
        Assert.Contains("\"model\":\"car\"", trace);
        Assert.Contains("\"model\":\"ped\"", trace);
    }

    private static string RepoRoot()
    {
        var d = new DirectoryInfo(AppContext.BaseDirectory);
        while (d is not null && !File.Exists(Path.Combine(d.FullName, "Traffic.sln")))
        {
            d = d.Parent;
        }

        return d?.FullName ?? throw new InvalidOperationException("repo root not found");
    }
}
