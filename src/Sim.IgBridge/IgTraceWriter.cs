using System.Globalization;

namespace Sim.IgBridge;

// Writes the IG-native trace (docs/IGBRIDGE-DECISIONS.md §3) as JSONL -- one record per line:
//   {"k":"new","id":"v42","t":12.300,"model":"car","x":100.400,"y":22.100,"h":271.30}
//   {"k":"upd","id":"v42","t":12.350,"x":101.100,"y":22.000,"h":271.00}
//   {"k":"del","id":"v42","t":40.100}
// Hand-rolled (not System.Text.Json) for BYTE-DETERMINISTIC output: fixed invariant-culture decimal
// formats and an explicit "\n" terminator (never Environment.NewLine), so two runs of the producer emit
// an identical trace file. Planar only -- no z/pitch/roll (the IG owns its terrain, Q1/Q5). The net8
// host/tests read it back with System.Text.Json (in-box there); the producer only writes.
public sealed class IgTraceWriter : IDisposable
{
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    private readonly TextWriter _writer;
    private readonly bool _ownsWriter;

    public IgTraceWriter(TextWriter writer)
    {
        _writer = writer;
        _ownsWriter = false;
    }

    public IgTraceWriter(string path)
    {
        _writer = new StreamWriter(path, append: false);
        _ownsWriter = true;
    }

    public int Count { get; private set; }

    public void Write(in IgSample s)
    {
        switch (s.Kind)
        {
            case IgRecordKind.New:
                _writer.Write("{\"k\":\"new\",\"id\":\"");
                _writer.Write(s.Id);
                _writer.Write("\",\"t\":");
                _writer.Write(s.T.ToString("0.000", Inv));
                _writer.Write(",\"model\":\"");
                _writer.Write(ModelString(s.Model));
                _writer.Write("\",\"x\":");
                _writer.Write(s.X.ToString("0.000", Inv));
                _writer.Write(",\"y\":");
                _writer.Write(s.Y.ToString("0.000", Inv));
                _writer.Write(",\"z\":");
                _writer.Write(s.Z.ToString("0.000", Inv));
                _writer.Write(",\"h\":");
                _writer.Write(s.HeadingDeg.ToString("0.00", Inv));
                _writer.Write("}\n");
                break;

            case IgRecordKind.Upd:
                _writer.Write("{\"k\":\"upd\",\"id\":\"");
                _writer.Write(s.Id);
                _writer.Write("\",\"t\":");
                _writer.Write(s.T.ToString("0.000", Inv));
                _writer.Write(",\"x\":");
                _writer.Write(s.X.ToString("0.000", Inv));
                _writer.Write(",\"y\":");
                _writer.Write(s.Y.ToString("0.000", Inv));
                _writer.Write(",\"z\":");
                _writer.Write(s.Z.ToString("0.000", Inv));
                _writer.Write(",\"h\":");
                _writer.Write(s.HeadingDeg.ToString("0.00", Inv));
                _writer.Write("}\n");
                break;

            case IgRecordKind.Del:
                _writer.Write("{\"k\":\"del\",\"id\":\"");
                _writer.Write(s.Id);
                _writer.Write("\",\"t\":");
                _writer.Write(s.T.ToString("0.000", Inv));
                _writer.Write("}\n");
                break;
        }

        Count++;
    }

    public static string ModelString(IgEntityModel model) => model switch
    {
        IgEntityModel.Car => "car",
        IgEntityModel.Ped => "ped",
        _ => "car",
    };

    public void Flush() => _writer.Flush();

    public void Dispose()
    {
        _writer.Flush();
        if (_ownsWriter)
        {
            _writer.Dispose();
        }
    }
}
