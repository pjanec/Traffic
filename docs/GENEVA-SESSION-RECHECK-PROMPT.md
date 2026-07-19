# Prompt for the SumoData / Geneva session — RE-CHECK after the P2-G gridlock fixes

Copy everything in the fenced block below into the SumoData (Geneva) session. This is a **re-run** of
the ship-acceptance you did before — the one that correctly rejected the drop-in for progressive
gridlock. The SumoSharp side has since landed the junction fixes that caused it. The acceptance metric
is the **clearing / halting curve**, exactly as you specified last time — NOT the teleport count.

---

```
You are working in the SumoData repo (the sub-area preprocessing pipeline). You previously ran a
ship-acceptance of SumoSharp as a drop-in for the vanilla `sumo` binary and correctly REJECTED it:
the real box progressively gridlocked (on-net halting climbed to ~89% of running vehicles vs vanilla's
~11%, mean relative speed 0.55 vs 0.84), and the teleport count massively understated that. You noted
the fast-follow had to be a PRE-MERGE blocker, and that the acceptance had to be judged on the HALTING
curve, not the teleport count. The SumoSharp side has now landed the junction fixes. Please re-run.

WHAT CHANGED ON THE SUMOSHARP SIDE (branch, NOT yet on main)
  Repo:   SumoSharp (sibling repo)
  Branch: claude/sumosharp-drop-in-binary-vq7u9p   (HEAD 5ca7315 or later)
  Three faithful, golden-safe fixes to the traffic-light junction handling landed. All 622 committed
  parity goldens stay byte-identical; each fix mirrors a specific SUMO mechanism:
    - Bug-1: the config parser now reads device.rerouting.* / routing-algorithm from a <routing>
             section (SUMO's canonical layout), not only from <processing>. Rerouting was silently
             inert for canonical configs before -- vehicles could not route around jams vanilla routes
             around.
    - Bug-2: the deterministic right-before-left cycle resolver no longer fires at traffic_light
             junctions (it is SUMO's RNG-deadlock-abort analogue, which only applies to uncontrolled
             equal-priority links) -- it was spuriously holding green movements a full signal cycle.
    - Bug-3: a foe held at a RED light no longer makes a crossing (green) vehicle yield to it. SUMO's
             MSLink::opened only yields to foes that currently hold right-of-way; the engine was
             yielding to a red-held foe that was still rolling toward its stop line. This was the
             dominant driver of the progressive gridlock you measured.
  Measured on a geometry-free synthetic (10 TL junctions, the SumoSharp side's stand-in for your box):
  teleports 24->10, peak on-net halting 101->85 (vanilla 45), mid-run trips-cleared gap vs vanilla
  roughly halved. NOT fully closed on the synthetic (peak halting 85 vs 45), and the SumoSharp side
  cannot see your real box -- so your re-run is the acceptance signal.

BUILD THE BINARY FRESH -- AND DO NOT RUN A STALE ONE (this bit the SumoSharp side hard)
  cd <sumosharp-checkout>
  git fetch origin && git checkout claude/sumosharp-drop-in-binary-vq7u9p && git pull
  git log --oneline -1        # confirm HEAD is 5ca7315 or later
  dotnet test                 # offline sanity: 622 parity / 3 skipped, all green
  # Build the shim FRESH and point SUMO_BINARY at THIS build's output, not any older publish:
  dotnet build src/Sim.Sumo/Sim.Sumo.csproj -c Release
  export SUMO_BINARY="dotnet $(pwd)/src/Sim.Sumo/bin/Release/net8.0/sumosharp.dll"
  # (Or scripts/publish-sumosharp.sh for a self-contained exe -- but if you publish, DELETE any old
  #  artifacts/sumosharp/<rid>/ first and re-publish, and verify the timestamp. A stale self-contained
  #  publish silently running instead of your fresh build produced hours of wrong measurements on the
  #  SumoSharp side. Whatever you point SUMO_BINARY at, confirm its mtime is from THIS checkout.)

THE ACCEPTANCE MEASUREMENT -- the clearing / halting curve, not teleports
  Run the SAME real box (same net + demand + flags) through BOTH engines to t_end, emitting a summary:
    vanilla:   sumo      -c <cfg> --end <N> --no-step-log true --summary-output van.sum.xml --statistic-output van.stat.xml --tripinfo-output van.ti.xml
    sumosharp: <SUMO_BINARY> -c <cfg> --end <N> --no-step-log true --summary-output ss.sum.xml  --statistic-output ss.stat.xml  --tripinfo-output ss.ti.xml
  Then compare, over time (e.g. every 100 s), from the two summary files:
    - running    (should be near-identical -- same vehicles inserted)
    - halting    (THE metric: does SumoSharp's halting track vanilla's, or climb monotonically?)
    - meanSpeedRelative  (fair-flow speed; SumoSharp's should stay close to vanilla's, not collapse)
    - trips cleared (arrived) over time -- does SumoSharp clear the demand on vanilla's schedule, or
      lag and leave vehicles stuck?
  Note: SumoSharp counts park-and-stay SINK vehicles as "halting" while SUMO excludes parked vehicles,
  so the raw end-of-run halting has a fixed offset equal to your parked-sink count -- use trips-cleared
  and the MID-RUN halting trajectory as the un-confounded signals, and subtract the parked floor if you
  compare end-state halting directly.

WHAT TO REPORT BACK (geometry-free is fine and preferred -- summaries/statistics, never the box itself)
  - The halting trajectory table: t, van running/halting/relSpd, ss running/halting/relSpd (the same
    shape you sent last time -- that table is exactly what settles this).
  - Trips cleared over time, both engines, and the final arrived count.
  - Teleports both engines (as a secondary indicator only -- you already established it understates).
  - no-cheating audit verdict on ss.ti.xml (PASS/FAIL + birth/death/FCD counts).
  - Your verdict: is the progressive gridlock GONE (halting tracks vanilla instead of climbing to
    ~89%)? Does the residual (SumoSharp still halts somewhat more than vanilla) clear the "believable
    dense traffic" bar for the visible product, or not?

If the halting curve now tracks vanilla and the residual is acceptable, tell the SumoSharp side
"gridlock re-check green on branch claude/sumosharp-drop-in-binary-vq7u9p" so they can move to merge.
If SumoSharp still halts materially more than vanilla, send the halting trajectory + trips-cleared
tables (geometry-free) so the SumoSharp side can localize the remaining source -- on the synthetic the
residual is now small and diffuse, so a real-box curve is what would point at the next mechanism.
```
