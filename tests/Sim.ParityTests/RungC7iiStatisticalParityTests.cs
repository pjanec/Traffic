using Sim.Core; using Sim.Harness; using Xunit; using Xunit.Abstractions;
namespace Sim.ParityTests;
// Rung C7-ii: ENSEMBLE statistical parity of the per-vehicle speedFactor distribution vs SUMO.
// scenarios/20-speedfactor-freeflow: single free-flow vehicle, speeddev=0.1, sigma=0. Each seed
// draws a different speedFactor -> different steady speed. Our engine's speedFactor distribution
// (C7-i, normc ported faithfully; RNG stream ours) must match SUMO's ensemble speed distribution
// (the ensemble-not-RNG-exact bar). Golden = 50 committed SUMO runs.
public class RungC7iiStatisticalParityTests {
  private const int N = 50; private const int Steps = 80;
  private readonly ITestOutputHelper _o; public RungC7iiStatisticalParityTests(ITestOutputHelper o)=>_o=o;
  [Fact] public void SpeedFactorFreeFlow_EngineEnsembleMatchesSumoEnsembleStatistically(){
    var dir=Path.Combine(RepoRoot(),"scenarios","20-speedfactor-freeflow");
    var tol=ToleranceConfig.Load(Path.Combine(dir,"tolerance.json"));
    var expected=Directory.EnumerateFiles(Path.Combine(dir,"golden.ensemble"),"*.fcd.xml")
      .OrderBy(p=>p,StringComparer.Ordinal).Select(FcdParser.Parse).ToList();
    Assert.Equal(N, expected.Count);
    var actual=new List<TrajectorySet>(N);
    for(var seed=1; seed<=N; seed++){
      var e=new Engine{Seed=(ulong)seed};
      e.LoadScenario(Path.Combine(dir,"net.net.xml"),Path.Combine(dir,"rou.rou.xml"),Path.Combine(dir,"config.sumocfg"));
      actual.Add(e.Run(Steps));
    }
    var r=TrajectoryComparator.CompareEnsemble(actual,expected,tol);
    foreach(var a in r.Attributes){
      _o.WriteLine($"[{a.Attribute}] mean: engine={a.MeanActual:F4} sumo={a.MeanExpected:F4} |d|={a.MeanError:F4} (tol {tol.MeanToleranceFor(a.Attribute)}) ok={a.MeanWithinTolerance}");
      _o.WriteLine($"[{a.Attribute}] std : engine={a.StdActual:F4} sumo={a.StdExpected:F4} |d|={a.StdError:F4} (tol {tol.StdToleranceFor(a.Attribute)}) ok={a.StdWithinTolerance}");
    }
    Assert.True(r.IsMatch, "ensemble speedFactor stats out of tolerance");
  }
  private static string RepoRoot(){var d=new DirectoryInfo(AppContext.BaseDirectory);while(d is not null&&!File.Exists(Path.Combine(d.FullName,"Traffic.sln")))d=d.Parent;return d!.FullName;}
}
