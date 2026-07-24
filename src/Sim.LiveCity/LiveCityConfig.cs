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

    // #15 into-occupied cut-in floor (Engine.MergeStoppedMinGap). A moving car must not slot into the target
    // lane within this many metres AHEAD of a STANDING (near-stopped) follower there -- IsTargetLaneSafe is a
    // braking-gap check that a stopped follower satisfies at ~any closeness, so without this floor cars cut in
    // 2-5 m ahead of standing cars (the residual after the cooperative-LC float fix). Only active when
    // CooperativeLaneChange is on (high realism); low realism keeps the cheap tight merge. 0 = off (parity).
    // Overridable via LIVECITY_MERGEGAP. Default 5 m covers the measured 2-5 m follow-side cut-in band.
    public double MergeStoppedMinGap { get; set; } = 5.0;

    // #15 into-occupied, STRATEGIC (required) path only (Engine.MergeStoppedStrategicDeferDist). Urgency-gated
    // deferral: defer a tight cut-in into a stopped turn-lane queue only while ego still has more than this
    // much usable distance to complete the change; allow it once urgent so ego never strands. 0 = off (the
    // required merge is never deferred). Overridable via LIVECITY_MERGEDEFER (only active when
    // MergeStoppedMinGap>0 and coop on). Default 15 m: an A/B sweep found a sharp cliff -- <=20 m reduces the
    // strategic tight cut-ins (44->16, -64%) with NO flow change (arrivals 1068, stoppedFrac 0.34, identical
    // progression to no-defer), while >=25 m tips into congestion (arrivals 959, stoppedFrac 0.91) that
    // paradoxically breeds MORE stopped-follower cut-ins. 15 m sits comfortably below that cliff.
    public double MergeStoppedStrategicDeferDist { get; set; } = 15.0;

    // A/B switch: full crossing-yield gate + ped signal compliance vs the baseline (no coupling).
    // Overridable via LIVECITY_YIELD (0 = off).
    public bool YieldEnabled { get; set; } = true;

    // realism #1/#2 (docs/LIVE-CITY-REALISM-1-2-DESIGN.md): the CrossingOccupancySource gate-disc radius.
    // The stock 0.3 m point disc only enters a car's ~1.2 m wheel-path corridor when the crossing ped is
    // nearly in front -> the car noses in (too late to stop a 5 m body). A larger footprint ("the ped
    // occupies this patch of the zebra") makes the car brake for a ped on the crossing ahead earlier AND
    // opens the longitudinal gap sooner, while staying lane-LOCAL: at 1.5 m the corridor half is
    // egoHalf+r = 0.9+1.5 = 2.4 m < the 4 m lane spacing, so it does NOT bleed onto adjacent lanes.
    // 0.3 = stock behaviour. Overridable via LIVECITY_GATE_RADIUS. Only the demo path; goldens/bench drive
    // the Engine directly with CrowdSource null, so this is parity-inert.
    public double CrossingGateRadius { get; set; } = 1.5;

    // realism #1/#2 fix (A): also feed low-power peds that are PAUSED on a crossing (AnimTag != Walk) to the
    // occupancy gate, not just walking ones. A ped standing on a crosswalk is MORE reason to yield; the stock
    // WalkAnimTag-only filter dropped them entirely -> cars drove over standing peds (the biggest nose-in
    // bucket). The crossing polygon test still restricts discs to peds actually ON a crossing. false =
    // stock (walking-only). Overridable via LIVECITY_GATE_PAUSED (0 = off).
    public bool GatePausedPedsOnCrossing { get; set; } = true;

    // realism #1 (density) fix: also feed HIGH-power (ORCA) peds on a crossing to the wide occupancy gate, not
    // just their 0.3 m ORCA physics footprint. The footprint's narrow radius only enters a car's ~1.2 m wheel
    // corridor when the ORCA ped is nearly in front -> cars nose over ORCA peds on crossings (rampant at high
    // density). Feeding them here gives the wide CrossingGateRadius gate. Trade-off: the gate disc is velocity
    // 0 (a car treats it as a STOPPED obstacle and waits), which over-brakes for an ORCA ped merely walking
    // across -- measured to cost car throughput. Off-crossing ORCA peds are unaffected (footprint only).
    // false = ORCA peds gate cars via footprint only. Overridable via LIVECITY_GATE_ORCA (1 = on).
    // DEFAULT OFF: measured to cut car throughput ~15% (velocity-0 gate over-brakes a walking ORCA ped) for
    // little nose-in gain once the crowd-disc query cap is fixed (that un-truncation alone cut ORCA nose-ins
    // 11->3 at 10x density). Superseded by OrcaFootprintExtraRadius below (velocity-preserving); kept for A/B.
    public bool GateOrcaPedsOnCrossing { get; set; } = false;

    // realism #1 (mid-junction ORCA): inflate the ORCA ped's VEHICLE-FACING footprint disc by this many
    // metres (InflatedFootprintSource), so a car "sees" and preventively slows for an ORCA ped ANYWHERE --
    // including the junction interior, not just marked crossings -- from farther out, and the wider corridor
    // survives the lane-projection error that lets a fast car drive through an ORCA ped on a curved internal
    // junction lane. Velocity is PRESERVED (unlike the velocity-0 crossing gate), so the car follows a walking
    // ORCA ped's motion rather than stopping dead. 0.6 -> a 0.3 m footprint becomes 0.9 m (corridor half
    // egoHalf+0.9 = 1.8 m, well < 4 m lane spacing). 0 = off (footprint only). Overridable via
    // LIVECITY_ORCA_RADIUS. 0.6 chosen by A/B sweep: drives ORCA drive-throughs (incl. mid-junction) to ZERO
    // AND RAISES car throughput (490->684 at 1x/2000 steps -- preventive smooth slowing avoids the abrupt
    // junction conflicts that stall flow). 0.8+ over-brakes and cliffs throughput (372), so 0.6 sits safely
    // below that edge.
    public double OrcaFootprintExtraRadius { get; set; } = 0.6;

    // docs/LIVE-CITY-15-YIELD-TIMEOUT-DESIGN.md: after this many seconds waiting at a junction, a car
    // forces its gap through APPROACHING cross-traffic (impatience) instead of yielding forever -- the
    // "driver who didn't notice the gap, then recovers" behaviour. 0 = off (SUMO-parity). Only affects
    // the demo; never a parity/bench scenario. Overridable via LIVECITY_YIELDTIMEOUT.
    public double JunctionYieldTimeoutSeconds { get; set; } = 5.0;

    // SUMO's own jam escape valve (time-to-teleport): a vehicle stuck/jammed for this many seconds is
    // lifted past the blockage (CheckJamTeleports, already ported; gated off at <=0). SUMO default is
    // 300 s; the demo wants a SHORT recovery ("driver didn't notice the gap, recovers quickly"). At 5 s
    // the downtown crop goes from ~0.39 stopped to ~0.10 (free flow) and arrivals 81 -> 188 over 200 s.
    // 0 = OFF (default). Owner rejected teleport as an unrealistic cure ("the car needs to travel THROUGH
    // the junction, not jump across"), so it is off by default; the knob stays for experimentation only.
    // Overridable via LIVECITY_TELEPORT. Only the demo could enable it; scenarios/bench always leave it off.
    public double TimeToTeleportSeconds { get; set; } = 0.0;

    // docs/LIVE-CITY-15-DEADLANE-DRIVETHROUGH-DESIGN.md: never let a dead-ended car freeze forever --
    // free-flow-reroute or drive through on any forward connection instead. RE-MEASURED after the
    // lane-change-straddles-junction CURE (docs/LIVE-CITY-15-LANECHANGE-JUNCTION-FIX-DESIGN.md): with the
    // desync cascade gone, this + WrongLaneRerouteAtApproach make every wrong-lane car RECOVER (strand
    // reasons collapse to reResolveOK+rerouteOK only; strandedDeadEnd=0, stuckInternal=0-3, no capSpent
    // clamp), so a single wrong-lane car can no longer clamp Speed=0 and wall its queue. Default ON for
    // the demo (owner priority: floaters must not cause blockage). off = SUMO-parity clamp; every
    // parity/bench scenario leaves the underlying Engine property false (byte-identical).
    // Overridable via LIVECITY_DRIVETHROUGH (0 = off).
    public bool DeadLaneDriveThrough { get; set; } = true;

    // Issue #15: generalises TryReResolveFromActualLane/TryRerouteFromDeadLane to fire while a
    // wrong-lane car is still APPROACHING the junction (within its own brake distance of the dead
    // lane's end) and to retry every step rather than permanently one-shot-capping after
    // MaxDeadLaneReroutes -- see Engine.WrongLaneRerouteAtApproach's own header comment for the full
    // mechanism. ORIGINALLY measured as a regression (box-blocking) -- but that was BEFORE the
    // lane-change-straddles-junction CURE (docs/LIVE-CITY-15-LANECHANGE-JUNCTION-FIX-DESIGN.md). RE-
    // MEASURED after the cure: with the desync cascade gone, this + DeadLaneDriveThrough make every
    // wrong-lane car RECOVER instead of clamping -- strand reasons collapse to reResolveOK+rerouteOK
    // only (0 capSpent/poolEdgeMismatch), strandedDeadEnd=0, stuckInternal=0-3, stoppedFrac 0.99->~0.2-0.4,
    // arrivals 258->800+. A single wrong-lane car can no longer clamp Speed=0 and wall its queue (owner
    // priority: floaters must not cause blockage). Default ON for the demo; every parity/bench scenario
    // leaves the underlying Engine property false (byte-identical). Overridable via LIVECITY_WRONGLANE.
    public bool WrongLaneRerouteAtApproach { get; set; } = true;

    // docs/LIVE-CITY-15-COOPERATIVE-LC-DESIGN.md: cooperative lane change -- when a car needs a lane a
    // neighbour occupies, the neighbour (follower) eases off (one gentle helpDecel step) to open a gap
    // instead of the car stalling/floating. Sets BOTH Engine.CoordinatedLaneChange and
    // Engine.CooperativeInformFollower. Default ON for the demo (a saturated grid, the good case for this
    // mechanism -- see CooperativeInformFollower's own header comment for why it is organic-net poison
    // but saturated-grid medicine); every parity/bench scenario leaves both underlying Engine properties
    // false (byte-identical). Overridable via LIVECITY_COOP (0 = off).
    public bool CooperativeLaneChange { get; set; } = true;

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

    // Ped crowd size knobs (SceneGen.BuildLiveCity's PedDemandConfig). Overridable via LIVECITY_PEDS,
    // which sets the concurrent cap and scales the spawn rate proportionally so the crowd fills to the
    // new cap at about the same wall-time as the default 160 does.
    public int PedPopulationCap { get; set; } = 160;
    public double PedSpawnRatePerSecond { get; set; } = 8.0;

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

        // LIVECITY_PEDS: concurrent ped cap; spawn rate scales with it so it fills at ~the default's pace.
        if (int.TryParse(Environment.GetEnvironmentVariable("LIVECITY_PEDS"), out var peds) && peds > 0)
        {
            cfg.PedPopulationCap = peds;
            cfg.PedSpawnRatePerSecond = 8.0 * System.Math.Max(1.0, peds / 160.0);
        }

        if (double.TryParse(Environment.GetEnvironmentVariable("LIVECITY_LCMIN"), out var lcMin))
        {
            cfg.LaneChangeMinSpeed = lcMin;
        }

        if (double.TryParse(Environment.GetEnvironmentVariable("LIVECITY_MERGEGAP"), out var mergeGap) && mergeGap >= 0.0)
        {
            cfg.MergeStoppedMinGap = mergeGap;
        }

        if (double.TryParse(Environment.GetEnvironmentVariable("LIVECITY_MERGEDEFER"), out var mergeDefer) && mergeDefer >= 0.0)
        {
            cfg.MergeStoppedStrategicDeferDist = mergeDefer;
        }

        cfg.YieldEnabled = Environment.GetEnvironmentVariable("LIVECITY_YIELD") != "0";

        if (double.TryParse(Environment.GetEnvironmentVariable("LIVECITY_GATE_RADIUS"), out var gateR) && gateR > 0.0)
        {
            cfg.CrossingGateRadius = gateR;
        }

        var gatePausedEnv = Environment.GetEnvironmentVariable("LIVECITY_GATE_PAUSED");
        if (gatePausedEnv != null)
        {
            cfg.GatePausedPedsOnCrossing = gatePausedEnv != "0";
        }

        var gateOrcaEnv = Environment.GetEnvironmentVariable("LIVECITY_GATE_ORCA");
        if (gateOrcaEnv != null)
        {
            cfg.GateOrcaPedsOnCrossing = gateOrcaEnv != "0";
        }

        if (double.TryParse(Environment.GetEnvironmentVariable("LIVECITY_ORCA_RADIUS"), out var orcaR) && orcaR >= 0.0)
        {
            cfg.OrcaFootprintExtraRadius = orcaR;
        }

        if (double.TryParse(Environment.GetEnvironmentVariable("LIVECITY_YIELDTIMEOUT"), out var yto) && yto >= 0.0)
        {
            cfg.JunctionYieldTimeoutSeconds = yto;
        }

        if (double.TryParse(Environment.GetEnvironmentVariable("LIVECITY_TELEPORT"), out var tel) && tel >= 0.0)
        {
            cfg.TimeToTeleportSeconds = tel;
        }

        // Default OFF (measured regression, see the property's header); only an explicit
        // LIVECITY_WRONGLANE toggles it: "0" forces off, anything else forces on for experimentation.
        var wrongLaneEnv = Environment.GetEnvironmentVariable("LIVECITY_WRONGLANE");
        if (wrongLaneEnv != null)
        {
            cfg.WrongLaneRerouteAtApproach = wrongLaneEnv != "0";
        }

        // LIVECITY_DRIVETHROUGH: experimental "never freeze -- take any forward connection" fallback
        // (Engine.DeadLaneDriveThrough). Only overrides when explicitly set.
        var driveThroughEnv = Environment.GetEnvironmentVariable("LIVECITY_DRIVETHROUGH");
        if (driveThroughEnv != null)
        {
            cfg.DeadLaneDriveThrough = driveThroughEnv != "0";
        }

        // LIVECITY_COOP: cooperative lane change (Engine.CoordinatedLaneChange + CooperativeInformFollower).
        // Only overrides when explicitly set.
        var coopEnv = Environment.GetEnvironmentVariable("LIVECITY_COOP");
        if (coopEnv != null)
        {
            cfg.CooperativeLaneChange = coopEnv != "0";
        }

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
