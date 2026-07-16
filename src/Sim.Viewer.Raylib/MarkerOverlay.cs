using System.Collections.Generic;
using System.Numerics;
using Sim.Core;

namespace Sim.Viewer.Raylib;

// `using Raylib_cs;` is deliberately placed HERE (namespace-body level) -- see RoadLayerCache.cs's
// identical comment for why a compilation-unit-level using loses to this namespace's own trailing
// "Raylib" segment.
using Raylib_cs;

// docs/SUMOSHARP-PACKAGING-DESIGN.md D10 (P3.1): a trivial, domain-agnostic IRenderOverlay used ONLY to
// prove the seam works end-to-end (the hidden `--overlay-test` CLI flag) -- a bright magenta filled
// circle at a fixed world point, drawn in DrawWorldOver. The generic render loop (RunLocal) never
// references this type directly except to construct it behind the flag; it is handed around purely as
// an `IRenderOverlay?`, exactly as a real domain overlay (e.g. the evac layer, P3.2) would be.
public sealed class MarkerOverlay : IRenderOverlay
{
    private static readonly Color Magenta = new(255, 0, 255, 255);

    private readonly double _worldX;
    private readonly double _worldY;

    public MarkerOverlay(double worldX, double worldY)
    {
        _worldX = worldX;
        _worldY = worldY;
    }

    public void DrawWorldOver(Camera2D camera, SimulationSnapshot snapshot, IReadOnlyList<Renderer.DrVehicleDraw> vehicles)
    {
        Raylib.BeginMode2D(camera);
        // World Y-up -> screen Y-down, same convention every other draw call in Renderer.cs uses.
        Raylib.DrawCircleV(new Vector2((float)_worldX, (float)-_worldY), 8f, Magenta);
        Raylib.EndMode2D();
    }
}
