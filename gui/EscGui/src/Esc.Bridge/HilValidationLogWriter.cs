using System.Text.Json;
using Esc.Protocol;

namespace Esc.Bridge;

internal sealed class HilValidationLogWriter
{
    private static readonly JsonSerializerOptions JsonOptions = new(JsonSerializerDefaults.Web);
    private readonly object _gate = new();
    private readonly string _directory = Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
        "EscGui",
        "hil-validation");

    public void Append(
        HilInputs inputs,
        uint sourceSequence,
        string request,
        string response,
        HilOutputs? outputs,
        bool freshOutput,
        double cacheAgeMs)
    {
        if (!inputs.HasValidationProvenance)
        {
            return;
        }

        var entry = new HilValidationLogEntry(
            DateTimeOffset.UtcNow,
            inputs.RunId!.Value,
            sourceSequence,
            inputs.SpeedRpm,
            inputs.Enable,
            request,
            response,
            freshOutput,
            cacheAgeMs,
            outputs);

        lock (_gate)
        {
            Directory.CreateDirectory(_directory);
            string path = Path.Combine(_directory, $"run-{entry.RunId:X8}.jsonl");
            File.AppendAllText(path, JsonSerializer.Serialize(entry, JsonOptions) + Environment.NewLine);
        }
    }
}

internal sealed record HilValidationLogEntry(
    DateTimeOffset HostTimestampUtc,
    uint RunId,
    uint SourceSequence,
    ushort SpeedRpm,
    bool Enable,
    string Request,
    string Response,
    bool FreshOutput,
    double CacheAgeMs,
    HilOutputs? Outputs);
