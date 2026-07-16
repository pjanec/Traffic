using System.Numerics;
using ImGuiNET;
using Sim.Viewer.Core;
using Sim.Viewer.Raylib;

namespace Sim.Viewer;

// NB: `Raylib.SetTargetFPS` below is fully-qualified as `global::Raylib_cs.Raylib.SetTargetFPS` rather
// than via a plain `using Raylib_cs;` -- this file's own namespace, Sim.Viewer, directly contains the
// nested namespace Sim.Viewer.Raylib (the packaged renderer project, P3.3), and C# simple-name lookup
// resolves that same-name nested-namespace member of THIS file's own enclosing namespace before any
// using-directive is even considered, so an unqualified `Raylib.XXX` call here would otherwise resolve
// to the "Sim.Viewer.Raylib" namespace itself (CS0234) rather than the Raylib_cs static class.

// docs/SUMOSHARP-PACKAGING-DESIGN.md D5/§5 (P3.3): these three ImGui control panels drive an EngineHost
// (DrawControlsPanel/DrawLoopbackControlsPanel) or the remote-command channel (DrawRemoteControlsPanel)
// directly -- both Sim.Viewer.Core types the packaged, GENERIC Sim.Viewer.Raylib renderer must not
// depend on (Tier-2 diagram, §5: Viewer.Raylib -> Viewer.Motion, Replication.Dds only). Moved here
// VERBATIM out of Renderer.cs (only the class/qualification changed) -- this is demo-tool wiring for a
// host THIS exe owns, not part of the reusable rendering package.
public static class ViewerControlsPanels
{
    // P1 controls panel (SUMOSHARP-NATIVE-VIEWER.md P1): mode label, restart, clear obstacles, and the
    // random-traffic toggle. Sized explicitly (SetNextWindowSize) so its text is never clipped -- P0's HUD
    // was cut off at the default auto-size. Must be called between rlImGui.Begin()/End() (see Program.cs).
    // `fpsCap` is the render frame-rate cap in fps, with 0 meaning "unlimited"; the radio here mutates it and
    // pushes the change straight to Raylib.SetTargetFPS so the choice takes effect on the next frame. It's a
    // ref because Program.cs owns the value (it sets the initial cap before the loop) -- see RunLocal.
    // docs/SUMOSHARP-PACKAGING-DESIGN.md D10 (P3.2): the "inject random traffic" checkbox is shown only for
    // a SANDBOX host (`!host.ScenarioMode`) -- a scenario/custom-source host carries its own committed
    // demand, so there is no random-traffic knob to offer. This replaces the old evac-specific `isEvac`
    // param: the generic renderer decides purely from EngineHost's own generic `ScenarioMode`, never from
    // a domain flag. The obstacle-drop hint is unconditional (a domain overlay that wants clicks routes
    // them away in Program.cs's click handler before InjectObstacleAtWorld is ever called).
    public static void DrawControlsPanel(EngineHost host, ref int fpsCap, ref bool smooth)
    {
        ImGui.SetNextWindowPos(new Vector2(10, 10), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(360, 390), ImGuiCond.FirstUseEver);
        ImGui.Begin("SumoSharp - controls");
        ImGui.Text(host.ScenarioMode ? "mode: SCENARIO" : "mode: SANDBOX");
        ImGui.Separator();
        if (ImGui.Button("restart"))
        {
            host.Restart();
        }

        ImGui.SameLine();
        if (ImGui.Button("clear obstacles"))
        {
            host.ClearObstacles();
        }

        ImGui.SameLine();
        if (ImGui.Button(host.IsPaused ? "resume" : "pause"))
        {
            host.SetPaused(!host.IsPaused);
        }

        if (!host.ScenarioMode)
        {
            var randomTraffic = host.RandomTraffic;
            if (ImGui.Checkbox("inject random traffic", ref randomTraffic))
            {
                host.SetRandomTraffic(randomTraffic);
            }
        }

        ImGui.Separator();

        // Playback speed relative to real time (LIVE, no rebuild): 1x = real speed, 3x = triple, etc. The
        // engine may not sustain a high factor at a large fleet (CPU-bound, ~3x max at 10k) -- the render
        // clock paces to the ACTUAL delivered rate (see the diagnostics "actual" line), so the slider is a
        // target, not a promise.
        var speed = (float)host.Speed;
        if (ImGui.SliderFloat("speed", ref speed, 0.25f, 10f, "%.2fx real-time"))
        {
            host.SetSpeed(speed);
        }

        // Sim resolution = 1 / step-length = how finely the engine integrates (SUMO's step-length). Higher Hz
        // = smoother physics + more snapshots to interpolate + better turns, but proportionally MORE compute,
        // so a high resolution at a large fleet just lowers the achievable speed. Changing it REBUILDS the sim
        // from t=0 (step-length is baked into the loaded config), hence discrete buttons, not a live slider.
        var hz = (int)Math.Round(1.0 / host.StepLength);
        ImGui.Text("sim resolution (restarts):");
        if (ImGui.RadioButton("1Hz", hz == 1)) host.SetStepLength(1.0);
        ImGui.SameLine();
        if (ImGui.RadioButton("2Hz", hz == 2)) host.SetStepLength(0.5);
        ImGui.SameLine();
        if (ImGui.RadioButton("5Hz", hz == 5)) host.SetStepLength(0.2);
        ImGui.SameLine();
        if (ImGui.RadioButton("10Hz", hz == 10)) host.SetStepLength(0.1);

        // Render-behind interpolation: blend the two latest authoritative snapshots up to the render rate so
        // vehicles glide instead of teleporting once per sim step. Costs ~one sim-step of latency; off = draw
        // the raw newest snapshot (the old jumpy behaviour), useful for an A/B or exact-frame inspection.
        ImGui.Checkbox("smooth (interpolate)", ref smooth);

        // Render FPS cap: 30 / 60 / unlimited. Rendering at the hundreds of fps the GPU allows for a 10 Hz
        // sim is wasted work, so default to a real cap; "unlimited" (0) stays available for perf measurement.
        ImGui.Text("render fps cap:");
        ImGui.SameLine();
        if (ImGui.RadioButton("30", fpsCap == 30)) { fpsCap = 30; global::Raylib_cs.Raylib.SetTargetFPS(30); }
        ImGui.SameLine();
        if (ImGui.RadioButton("60", fpsCap == 60)) { fpsCap = 60; global::Raylib_cs.Raylib.SetTargetFPS(60); }
        ImGui.SameLine();
        if (ImGui.RadioButton("unlimited", fpsCap == 0)) { fpsCap = 0; global::Raylib_cs.Raylib.SetTargetFPS(0); }

        ImGui.TextWrapped("click a road to drop an obstacle - drag to pan - wheel to zoom - 'd' toggles diagnostics");
        ImGui.End();
    }

    // P2b controls panel: mode label, restart/clear/random (identical semantics to DrawControlsPanel,
    // driving the SAME publisher-owned EngineHost), plus the DR delay slider (0 = extrapolate, higher =
    // interpolate) and the smoothing toggle (extrapolation-only low-pass, HtmlPage.cs's `smooth`).
    public static void DrawLoopbackControlsPanel(EngineHost host, ref float delaySeconds, ref bool smooth, ref bool alwaysInterpolate)
    {
        ImGui.SetNextWindowPos(new Vector2(10, 10), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(380, 370), ImGuiCond.FirstUseEver);
        ImGui.Begin("SumoSharp - controls (loopback)");
        ImGui.Text(host.ScenarioMode ? "mode: SCENARIO" : "mode: SANDBOX");
        ImGui.Separator();
        if (ImGui.Button("restart"))
        {
            host.Restart();
        }

        ImGui.SameLine();
        if (ImGui.Button("clear obstacles"))
        {
            host.ClearObstacles();
        }

        ImGui.SameLine();
        if (ImGui.Button(host.IsPaused ? "resume" : "pause"))
        {
            host.SetPaused(!host.IsPaused);
        }

        var randomTraffic = host.RandomTraffic;
        if (ImGui.Checkbox("inject random traffic", ref randomTraffic))
        {
            host.SetRandomTraffic(randomTraffic);
        }

        // Sim controls (loopback owns the engine, so these apply here, unlike view-only remote). See
        // DrawControlsPanel for the semantics of speed (x real-time, live) and sim resolution (step-length,
        // rebuilds).
        var speed = (float)host.Speed;
        if (ImGui.SliderFloat("speed", ref speed, 0.25f, 10f, "%.2fx real-time"))
        {
            host.SetSpeed(speed);
        }

        // Sim resolution is a sandbox-only control (scenario mode's step-length is fixed by its .sumocfg).
        if (host.ScenarioMode)
        {
            ImGui.Text($"sim resolution: {1.0 / host.StepLength:F0}Hz (scenario-fixed)");
        }
        else
        {
            var hz = (int)Math.Round(1.0 / host.StepLength);
            ImGui.Text("sim resolution (restarts):");
            if (ImGui.RadioButton("1Hz", hz == 1)) host.SetStepLength(1.0);
            ImGui.SameLine();
            if (ImGui.RadioButton("2Hz", hz == 2)) host.SetStepLength(0.5);
            ImGui.SameLine();
            if (ImGui.RadioButton("5Hz", hz == 5)) host.SetStepLength(0.2);
            ImGui.SameLine();
            if (ImGui.RadioButton("10Hz", hz == 10)) host.SetStepLength(0.1);
        }

        ImGui.Separator();
        // "always interpolate" auto-sets the delay slider each frame (Program.cs) to ~1.5x the measured DDS
        // packet interval, so the render clock always sits behind the newest packet -> Resolve always
        // interpolates instead of extrapolating. The manual slider is disabled while it's driving the value.
        ImGui.Checkbox("always interpolate (auto delay)", ref alwaysInterpolate);
        ImGui.BeginDisabled(alwaysInterpolate);
        ImGui.SliderFloat("DR delay (s)", ref delaySeconds, 0f, 1.5f, "%.2f");
        ImGui.EndDisabled();
        ImGui.Checkbox("smooth (extrap only)", ref smooth);
        ImGui.TextWrapped("delay 0 = extrapolate (predict ahead, may snap); raise = interpolate between DDS packets (smooth, delayed)");
        ImGui.End();
    }

    // P3 ("remote mode + QoS"): the view-only counterpart to DrawLoopbackControlsPanel -- same DR-delay
    // slider + smoothing toggle, but no restart/clear-obstacles/random-traffic buttons (a remote viewer has
    // no EngineHost to command -- docs/SUMOSHARP-NATIVE-VIEWER.md's "Delegation model": remote is
    // view-only). Adds the two indicators a remote viewer needs that a loopback viewer doesn't: whether the
    // Vehicles topic currently has a matched (live) writer, and whether the durable geometry topic has
    // delivered the whole network yet -- both meaningful only when there's no local publisher to fall back on.
    // `status` is the publisher's live engine state (DdsViewerStatus). It makes the command widgets
    // AUTHORITATIVE (pause label, speed value, tick rate reflect the real host) instead of optimistic, and
    // disables what can't act: all commands until a host is present, and the sim-tick-rate radios unless the
    // host is a sandbox (a scenario's step-length is fixed). `speed`/`random` stay refs -- speed is
    // draggable and re-synced to the host value when idle; random has no status field yet.
    public static void DrawRemoteControlsPanel(
        DdsCommandWriter cmd, ViewerStatus status, ref float speed, ref bool random,
        ref float delaySeconds, ref bool smooth, ref bool alwaysInterpolate, bool connected, bool geometryComplete)
    {
        ImGui.SetNextWindowPos(new Vector2(10, 10), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(380, 380), ImGuiCond.FirstUseEver);
        ImGui.Begin("SumoSharp - controls (remote)");
        ImGui.Text("mode: REMOTE (drives the publisher via DDS)");
        ImGui.Separator();
        ImGui.Text(connected ? "connected: yes" : "connected: NO (waiting for a publisher)");
        ImGui.Text(geometryComplete ? "geometry: received" : "geometry: waiting...");
        ImGui.Text(status.Present
            ? $"host: {(status.Sandbox ? "SANDBOX" : "SCENARIO")}  {(status.Paused ? "PAUSED" : "running")}  sim {status.SimTime:F0}s  veh {status.VehicleCount}"
            : "host: (no status yet)");
        ImGui.Separator();

        // Command controls need a live host -> disabled until status arrives (nothing to drive otherwise).
        ImGui.BeginDisabled(!status.Present);

        if (ImGui.Button("restart")) cmd.Send(ViewerCommandKind.Restart);
        ImGui.SameLine();
        if (ImGui.Button("clear obstacles")) cmd.Send(ViewerCommandKind.ClearObstacles);
        ImGui.SameLine();
        if (ImGui.Button(status.Paused ? "resume" : "pause")) // label from the HOST's real pause state
        {
            cmd.Send(ViewerCommandKind.Pause, flag: !status.Paused);
        }

        if (ImGui.Checkbox("inject random traffic", ref random))
        {
            cmd.Send(ViewerCommandKind.SetRandomTraffic, flag: random);
        }

        var speedChanged = ImGui.SliderFloat("speed", ref speed, 0.25f, 10f, "%.2fx real-time");
        var speedActive = ImGui.IsItemActive();
        if (speedChanged)
        {
            cmd.Send(ViewerCommandKind.SetSpeed, value: speed);
        }

        // Sim tick rate (= 1/step-length): disabled unless the host is a sandbox (scenario step-length is
        // fixed). Reflects the host's actual step, not an optimistic guess.
        var hz = status.StepLength > 1e-6 ? (int)Math.Round(1.0 / status.StepLength) : 1;
        ImGui.BeginDisabled(!status.Sandbox);
        ImGui.Text("sim tick rate:");
        if (ImGui.RadioButton("1Hz", hz == 1)) cmd.Send(ViewerCommandKind.SetStepLength, value: 1.0);
        ImGui.SameLine();
        if (ImGui.RadioButton("2Hz", hz == 2)) cmd.Send(ViewerCommandKind.SetStepLength, value: 0.5);
        ImGui.SameLine();
        if (ImGui.RadioButton("5Hz", hz == 5)) cmd.Send(ViewerCommandKind.SetStepLength, value: 0.2);
        ImGui.SameLine();
        if (ImGui.RadioButton("10Hz", hz == 10)) cmd.Send(ViewerCommandKind.SetStepLength, value: 0.1);
        ImGui.EndDisabled();

        ImGui.EndDisabled(); // command controls

        // Re-sync the speed slider to the host's actual value when the user isn't actively dragging it.
        if (!speedActive && status.Present)
        {
            speed = (float)status.Speed;
        }

        // --- these are LOCAL to this viewer (client-side dead-reckoning playout, not engine state) ---
        ImGui.Separator();
        // "always interpolate" auto-sets the delay slider each frame (Program.cs) to ~1.5x the measured DDS
        // packet interval, so the render clock always sits behind the newest packet -> Resolve always
        // interpolates instead of extrapolating. The manual slider is disabled while it's driving the value.
        ImGui.Checkbox("always interpolate (auto delay)", ref alwaysInterpolate);
        ImGui.BeginDisabled(alwaysInterpolate);
        ImGui.SliderFloat("DR delay (s)", ref delaySeconds, 0f, 1.5f, "%.2f");
        ImGui.EndDisabled();
        ImGui.Checkbox("smooth (extrap only)", ref smooth);
        ImGui.TextWrapped("click a road to drop an obstacle (remote). delay 0 = extrapolate; raise = interpolate");
        ImGui.End();
    }
}
