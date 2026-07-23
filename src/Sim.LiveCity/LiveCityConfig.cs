using System;
using System.IO;

namespace Sim.LiveCity;

// docs/LIVE-CITY-VIEWERS-DESIGN.md §1: the constructor knobs for LiveCitySim, mirroring the constants
// SceneGen.BuildLiveCity hard-codes (the PINNED downtown-HERO crop, the car/ped seeds, the demo's tuned
// car cap) so a fresh LiveCitySim reproduces the reference recipe byte-for-byte unless a caller
// deliberately overrides a knob. Env-var overrides (LIVECITY_CARS/LCMIN/YIELD) keep the same semantics
// as the reference so an existing shell habit ("run it with LIVECITY_CARS=300") still works against the
// new host.
public sealed class LiveCityConfig
{
    // The demo_city/box dataset directory (contains net.xml + scenario.rou.xml). Set by ForRepoRoot or
    // by the caller directly.
    public string DatasetDir { get; set; } = string.Empty;

    // PINNED crop = SumoData's co-located downtown HERO block (SUMOSHARP-LIVE-CITY-DECISIONS.md Q7).
    public double X0 { get; set; } = 2055;
    public double Y0 { get; set; } = 2055;
    public double X1 { get; set; } = 2895;
    public double Y1 { get; set; } = 2895;

    // Max density: with the multi-lane overlap fix on main, the downtown crop holds ~157 concurrent
    // cars + 160 peds cleanly (SceneGen.BuildLiveCity's remarks). Overridable via LIVECITY_CARS.
    public int CarTargetConcurrent { get; set; } = 160;

    // A queued/standing car must not snap sideways a full lane -- it sorts into its lane only while moving.
    // 1.5 clamps more of the standing/crawling snaps than the 2D path's 1.0 (which still left ~15% residual)
    // for the 3D impression; keep <= ~2.0 so legitimate turn-lane sorting still happens and saturated queues
    // don't deadlock (any forward creep clears the gate). Overridable via LIVECITY_LCMIN.
    public double LaneChangeMinSpeed { get; set; } = 1.5;

    // A/B switch: full crossing-yield gate + ped signal compliance vs the baseline (no coupling).
    // Overridable via LIVECITY_YIELD (0 = off).
    public bool YieldEnabled { get; set; } = true;

    public int CarSpawnPerStep { get; set; } = 5;

    // step-length 0.5 == the ped/frame Dt, so cars and peds advance the same sim-time per Step().
    public double Dt { get; set; } = 0.5;

    // docs/LIVE-CITY-VISUALS-NOTES.md (tick-rate task): a convenience Hz view of Dt -- Dt = 1.0/Hz, so
    // SimHz = 20 <=> Dt = 0.05. This is the SAME knob as Dt, just expressed the way a CLI flag
    // (`--sim-hz`) or a viewer control naturally wants it; setting either one is visible through the
    // other immediately (no separate backing field). LiveCityConfig itself does NOT validate Hz against
    // the allowed set {1,2,5,10,20} -- any positive Dt/Hz is accepted here -- the CLI layer
    // (Sim.Viewer/Program.cs, City3D/Viewer/Main.cs) is where that enum is enforced, per the design's
    // "LiveCityConfig itself just takes a Dt" instruction.
    public double SimHz
    {
        get => Dt > 0.0 ? 1.0 / Dt : 0.0;
        set { if (value > 0.0) Dt = 1.0 / value; }
    }

    // Ped demand seed (SceneGen.BuildLiveCity's PedDemandConfig.Seed).
    public ulong PedSeed { get; set; } = 20260721UL;

    // Car spawn PRNG seed (SceneGen.BuildLiveCity's `rng` initializer for the deterministic SplitMix64).
    public ulong CarRngSeed { get; set; } = 0x243F6A8885A308D3UL;

    // docs/LIVE-CITY-VIEWERS-TASKS.md A2: env knobs with the same semantics as the reference
    // (SceneGen.BuildLiveCity), resolved once here so callers get the exact same defaults/overrides.
    public static LiveCityConfig ForRepoRoot(string repoRoot)
    {
        var cfg = new LiveCityConfig
        {
            DatasetDir = Path.Combine(repoRoot, "scenarios", "_ped", "demo_city", "box"),
        };

        if (int.TryParse(Environment.GetEnvironmentVariable("LIVECITY_CARS"), out var cars))
        {
            cfg.CarTargetConcurrent = cars;
        }

        if (double.TryParse(Environment.GetEnvironmentVariable("LIVECITY_LCMIN"), out var lcMin))
        {
            cfg.LaneChangeMinSpeed = lcMin;
        }

        cfg.YieldEnabled = Environment.GetEnvironmentVariable("LIVECITY_YIELD") != "0";

        // LIVECITY_HZ: same env-knob convention as LIVECITY_CARS/LCMIN above, expressed in Hz (via
        // SimHz) rather than raw Dt seconds since that's how a shell habit is more likely to want it.
        // No {1,2,5,10,20} validation here -- ForRepoRoot mirrors LIVECITY_CARS/LCMIN's own "any parsed
        // value is accepted" behavior; the CLI-facing --sim-hz flags do the enum validation.
        if (double.TryParse(Environment.GetEnvironmentVariable("LIVECITY_HZ"), out var hz) && hz > 0.0)
        {
            cfg.SimHz = hz;
        }

        return cfg;
    }
}
