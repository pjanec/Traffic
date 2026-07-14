using System.Numerics;
using Raylib_cs;

namespace Sim.Viewer;

// docs/SUMOSHARP-NATIVE-VIEWER-TESTING.md TASK 1 (10k perf pass): the static road layer (background + road
// casing/surface + dashed centrelines + chevrons) is geometry that only changes when the CAMERA changes
// (pan/zoom) -- not per frame, and not with the traffic. On a large net (scenarios/_bench/city-15000 has
// ~13k edges) re-stroking it every frame with immediate-mode DrawLineEx is ~80% of the frame cost (measured:
// ~8ms of an ~10ms frame at 10k vehicles). This bakes that layer into a RenderTexture ONCE per camera state
// and blits it each frame, so a static (or slowly panning) view pays the road cost only when the view
// actually moves. Visuals are byte-identical to the un-cached path because the bake runs the exact same
// DrawStaticWorld at the exact same camera -- including its zoom-dependent thickness floor -- so re-baking on
// any camera change preserves the on-screen appearance the immediate-mode renderer produced.
public sealed class RoadLayerCache : IDisposable
{
    private int _width;
    private int _height;
    private RenderTexture2D _rt;
    private bool _baked;
    private Camera2D _bakedCamera;

    public RoadLayerCache(int width, int height)
    {
        _width = width;
        _height = height;
        _rt = Raylib.LoadRenderTexture(width, height);
    }

    // Resize the offscreen layer to a new window size (on a resizable-window resize event). Reloads the
    // RenderTexture and forces a re-bake, since the old texture no longer matches the framebuffer.
    public void Resize(int width, int height)
    {
        if (width == _width && height == _height)
        {
            return;
        }

        Raylib.UnloadRenderTexture(_rt);
        _width = width;
        _height = height;
        _rt = Raylib.LoadRenderTexture(width, height);
        _baked = false;
    }

    // Re-bake the static layer into the RenderTexture iff the camera changed since the last bake (or nothing
    // is baked yet), then blit the cached layer to the current render target. `drawStatic` is
    // Renderer.DrawStaticWorld bound to this frame's network -- it clears the background and strokes the
    // roads/markings under BeginMode2D(camera), so the baked texture is fully opaque.
    //
    // Safe to call inside BeginDrawing(): raylib flushes the active batch on Begin/EndTextureMode and restores
    // the previous target on EndTextureMode, so switching to the offscreen FBO and back mid-frame is fine.
    public void EnsureAndBlit(Camera2D camera, Action<Camera2D> drawStatic)
    {
        if (!_baked || !SameCamera(camera, _bakedCamera))
        {
            Raylib.BeginTextureMode(_rt);
            drawStatic(camera);
            Raylib.EndTextureMode();
            _bakedCamera = camera;
            _baked = true;
        }

        // RenderTexture colour buffers are stored bottom-up (OpenGL origin), so blit with a negative source
        // height to flip Y back to the screen's top-down convention.
        var source = new Rectangle(0, 0, _width, -_height);
        Raylib.DrawTextureRec(_rt.Texture, source, Vector2.Zero, Color.White);
    }

    private static bool SameCamera(Camera2D a, Camera2D b) =>
        a.Target == b.Target && a.Offset == b.Offset && a.Rotation == b.Rotation && a.Zoom == b.Zoom;

    public void Dispose() => Raylib.UnloadRenderTexture(_rt);
}
