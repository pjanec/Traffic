using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using Sim.Core;
using Sim.LiveHost;

// SUMOSHARP-API.md §11: browser-live demo host. Usage:
//   dotnet run --project src/Sim.LiveHost -- [<scenarioDir> | <net.net.xml>]
// Defaults to scenarios/15-reroute if no argument is given. Open the printed URL, watch runtime-spawned
// traffic, and CLICK the canvas to drop an obstacle the cars react to.

var netPath = ResolveNetPath(args);
if (netPath is null)
{
    Console.Error.WriteLine("error: could not find a .net.xml (pass a scenario dir or net file as the first arg).");
    return 2;
}

using var host = new SimHost(netPath, ParseRealism(args));

var builder = WebApplication.CreateBuilder(args);
builder.Logging.ClearProviders();
var app = builder.Build();
app.UseWebSockets();

app.MapGet("/", () => Results.Content(HtmlPage.Html, "text/html; charset=utf-8"));

app.MapGet("/ws", async (HttpContext ctx) =>
{
    if (!ctx.WebSockets.IsWebSocketRequest)
    {
        ctx.Response.StatusCode = 400;
        return;
    }

    using var ws = await ctx.WebSockets.AcceptWebSocketAsync();
    await ServeSocketAsync(ws, host, ctx.RequestAborted);
});

const string url = "http://0.0.0.0:5055";
Console.WriteLine($"SumoSharp LiveHost — network: {netPath}");
Console.WriteLine($"open  http://localhost:5055   (click the canvas to drop an obstacle)");
app.Run(url);
return 0;

static async Task ServeSocketAsync(WebSocket ws, SimHost host, CancellationToken ct)
{
    await SendTextAsync(ws, host.NetworkJson, ct);
    var send = SendLoopAsync(ws, host, ct);
    var recv = ReceiveLoopAsync(ws, host, ct);
    await Task.WhenAny(send, recv);
}

static async Task SendLoopAsync(WebSocket ws, SimHost host, CancellationToken ct)
{
    try
    {
        while (ws.State == WebSocketState.Open && !ct.IsCancellationRequested)
        {
            await SendTextAsync(ws, host.BuildFrameJson(), ct);
            await Task.Delay(50, ct); // ~20 fps to the browser (decoupled from the 30 Hz sim tick)
        }
    }
    catch (OperationCanceledException) { }
    catch (WebSocketException) { }
}

static async Task ReceiveLoopAsync(WebSocket ws, SimHost host, CancellationToken ct)
{
    var buffer = new byte[8192];
    try
    {
        while (ws.State == WebSocketState.Open && !ct.IsCancellationRequested)
        {
            var result = await ws.ReceiveAsync(buffer, ct);
            if (result.MessageType == WebSocketMessageType.Close)
            {
                break;
            }

            HandleClientMessage(host, Encoding.UTF8.GetString(buffer, 0, result.Count));
        }
    }
    catch (OperationCanceledException) { }
    catch (WebSocketException) { }
}

static void HandleClientMessage(SimHost host, string message)
{
    try
    {
        using var doc = JsonDocument.Parse(message);
        var type = doc.RootElement.GetProperty("type").GetString();
        switch (type)
        {
            case "obstacle":
                host.InjectObstacleAtWorld(
                    doc.RootElement.GetProperty("x").GetDouble(),
                    doc.RootElement.GetProperty("y").GetDouble());
                break;
            case "clear":
                host.ClearObstacles();
                break;
            case "restart":
                host.Restart();
                break;
            case "random":
                host.SetRandomTraffic(doc.RootElement.GetProperty("on").GetBoolean());
                break;
        }
    }
    catch (JsonException) { }
}

static async Task SendTextAsync(WebSocket ws, string text, CancellationToken ct) =>
    await ws.SendAsync(Encoding.UTF8.GetBytes(text), WebSocketMessageType.Text, endOfMessage: true, ct);

// Optional production render mode (SUMOSHARP-DEADRECKONING.md §6.3): pass "chord" or "corner"/"offtrack"
// on the command line to render the SUMO chord heading / swept-path off-tracking (visible on curvy nets /
// long vehicles). Default is SUMO-exact tangent.
static RenderRealism ParseRealism(string[] args)
{
    foreach (var a in args)
    {
        var s = a.ToLowerInvariant();
        if (s.Contains("corner") || s.Contains("offtrack")) return RenderRealism.CornerCutCorrected;
        if (s.Contains("chord")) return RenderRealism.ChordHeading;
    }

    return RenderRealism.ParityTangent;
}

static string? ResolveNetPath(string[] args)
{
    if (args.Length > 0)
    {
        var arg = args[0];
        if (File.Exists(arg))
        {
            return arg;
        }

        if (Directory.Exists(arg))
        {
            return Directory.GetFiles(arg, "*.net.xml").FirstOrDefault();
        }
    }

    // Default: scenarios/15-reroute/net.net.xml, found by walking up to the repo root (Traffic.sln).
    var dir = new DirectoryInfo(AppContext.BaseDirectory);
    while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "Traffic.sln")))
    {
        dir = dir.Parent;
    }

    var def = dir is null ? null : Path.Combine(dir.FullName, "scenarios", "15-reroute", "net.net.xml");
    return def is not null && File.Exists(def) ? def : null;
}
