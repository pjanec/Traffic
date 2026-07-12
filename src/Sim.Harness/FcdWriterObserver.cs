using System.Globalization;
using Sim.Core;

namespace Sim.Harness;

// VB-0 (VIZ_BENCH_TASKS.md Phase 0): the first real consumer of the D9 export seam
// (ISimExportObserver). It writes a SUMO-schema `--fcd-output` file AS the engine runs, so
// `Sim.Viz` and the benchmark can consume an engine trajectory through the exact same FCD path
// they already use for `golden.fcd.xml` (parsed by FcdParser). It does NOT touch
// Engine.EmitTrajectory -- HANDOFF.md's rule is to extend the engine THROUGH the D-seams, and
// this is a plain ISimExportObserver attached via Engine.AddExportObserver.
//
// Precision: numeric fields are written round-trippable ("R"), i.e. FULL double precision. The
// engine emits full precision and must NEVER round down to a coarse golden (CLAUDE.md /
// HANDOFF.md "goldens are --precision 6; the engine emits full precision"), so re-parsing this
// file yields back the engine's exact in-memory trajectory -- feeding it + the SUMO golden to
// TrajectoryComparator reproduces the in-memory parity result attribute-for-attribute.
//
// Schema mirrors SUMO's FCD (see any scenarios/*/golden.fcd.xml): <fcd-export> root, one
// <timestep time="..."> per frame, child <vehicle id x y angle type speed pos lane/> rows.
// slope/acceleration are omitted (FcdParser treats acceleration as optional and the viz reads
// none of them); add them here if a future consumer needs them.
public sealed class FcdWriterObserver : ISimExportObserver, IDisposable
{
    private readonly TextWriter _writer;
    private readonly bool _ownsWriter;
    private bool _rootOpen;
    private bool _closed;

    public FcdWriterObserver(string path)
        : this(new StreamWriter(path, append: false), ownsWriter: true)
    {
    }

    public FcdWriterObserver(TextWriter writer, bool ownsWriter = false)
    {
        _writer = writer ?? throw new ArgumentNullException(nameof(writer));
        _ownsWriter = ownsWriter;
        _writer.WriteLine("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
        _writer.WriteLine("<fcd-export>");
        _rootOpen = true;
    }

    public void OnFrameBegin(double time)
    {
        _writer.Write("    <timestep time=\"");
        _writer.Write(Fmt(time));
        _writer.WriteLine("\">");
    }

    public void OnVehicleExported(in VehicleExportSnapshot s)
    {
        // Attribute order matches SUMO's own FCD writer for readability; the parser is
        // order-independent so it does not strictly matter.
        _writer.Write("        <vehicle id=\"");
        _writer.Write(Escape(s.VehicleId));
        _writer.Write("\" x=\"");
        _writer.Write(Fmt(s.X));
        _writer.Write("\" y=\"");
        _writer.Write(Fmt(s.Y));
        _writer.Write("\" angle=\"");
        _writer.Write(Fmt(s.Angle));
        _writer.Write("\" type=\"");
        _writer.Write(Escape(s.VehicleType));
        _writer.Write("\" speed=\"");
        _writer.Write(Fmt(s.Speed));
        _writer.Write("\" pos=\"");
        _writer.Write(Fmt(s.Pos));
        // Phase 2 (sublane): SUMO places `posLat` right after `pos` in FCD. 0 for a lane-centred
        // vehicle; emitted always so a written engine trajectory round-trips the lateral offset.
        _writer.Write("\" posLat=\"");
        _writer.Write(Fmt(s.PosLat));
        _writer.Write("\" lane=\"");
        _writer.Write(Escape(s.Lane));
        _writer.WriteLine("\"/>");
    }

    public void OnFrameEnd(double time) => _writer.WriteLine("    </timestep>");

    // Full double precision (round-trippable) -- never a lossy fixed-decimal that would cap
    // parity sensitivity below tolerance.json (see class header).
    private static string Fmt(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    private static string Escape(string s) => s
        .Replace("&", "&amp;")
        .Replace("<", "&lt;")
        .Replace(">", "&gt;")
        .Replace("\"", "&quot;");

    private void CloseRoot()
    {
        if (_closed)
        {
            return;
        }

        if (_rootOpen)
        {
            _writer.WriteLine("</fcd-export>");
            _writer.Flush();
            _rootOpen = false;
        }

        _closed = true;
    }

    public void Dispose()
    {
        CloseRoot();
        if (_ownsWriter)
        {
            _writer.Dispose();
        }
    }
}
