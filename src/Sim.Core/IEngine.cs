namespace Sim.Core;

// The engine seam (DESIGN.md "build order"): implemented starting Task 2. Not implemented here.
public interface IEngine
{
    void LoadScenario(string netXmlPath, string rouXmlPath, string sumocfgPath);

    TrajectorySet Run(int steps);
}
