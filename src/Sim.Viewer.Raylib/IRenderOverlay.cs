using System.Collections.Generic;
using Raylib_cs;
using Sim.Core;

namespace Sim.Viewer.Raylib;

// docs/SUMOSHARP-PACKAGING-DESIGN.md D10: the GENERIC render-overlay seam. The packaged viewer (once
// split out, see D5) must stay domain-agnostic -- it renders any SUMO stream (roads/lanes/TLs/vehicles/
// HUD/camera) but knows nothing about any specific domain (e.g. panic-evacuation). A domain layer plugs
// extra world-space draw passes / UI / click-handling in by implementing this interface; the generic
// render loop (RunLocal in Program.cs) calls it through the `IRenderOverlay?` seam without ever knowing
// the concrete type. Default-interface-method bodies (all no-ops) so a trivial overlay implements only
// what it needs -- e.g. a pure click-handler need not override either Draw method.
public interface IRenderOverlay
{
    // Extra world-space layers drawn UNDER the vehicles (called immediately BEFORE
    // Renderer.DrawDynamicWorld). `vehicles` is the same already-resolved draw-pose list the generic
    // loop is about to hand to DrawDynamicWorld, so an overlay can key off vehicle identity (see
    // DrVehicleDraw.Handle) without re-deriving it.
    void DrawWorldUnder(Camera2D camera, SimulationSnapshot snapshot, IReadOnlyList<Renderer.DrVehicleDraw> vehicles) { }

    // Extra world-space layers drawn OVER the vehicles (called immediately AFTER Renderer.DrawDynamicWorld).
    void DrawWorldOver(Camera2D camera, SimulationSnapshot snapshot, IReadOnlyList<Renderer.DrVehicleDraw> vehicles) { }

    // An optional ImGui panel (legend / counters / hints). Called inside the rlImGui Begin()/End() frame,
    // alongside the generic controls/diagnostics panels.
    void DrawUi() { }

    // True if this overlay wants world clicks routed to OnWorldClick instead of the generic viewer's
    // default "drop an obstacle" behaviour. False (the default) preserves today's click handling for any
    // overlay that doesn't care about clicks.
    bool HandlesWorldClick => false;

    // Called with the WORLD-space point (already un-flipped: SUMO's Y-up convention, not raylib's
    // Y-down screen space) that was clicked, when HandlesWorldClick is true.
    void OnWorldClick(double worldX, double worldY) { }
}
