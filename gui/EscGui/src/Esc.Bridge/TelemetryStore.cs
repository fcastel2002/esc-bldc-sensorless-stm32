using Esc.Protocol;

namespace Esc.Bridge;

public sealed class TelemetryStore
{
    private const int DefaultCapacity = 2_000;
    private readonly object _gate = new();
    private readonly Dictionary<string, Queue<TelemetrySample>> _samples = new(StringComparer.OrdinalIgnoreCase);
    private readonly Dictionary<string, uint> _droppedSamples = new(StringComparer.OrdinalIgnoreCase);
    private readonly int _capacity;

    public TelemetryStore()
        : this(DefaultCapacity)
    {
    }

    public TelemetryStore(int capacity)
    {
        _capacity = Math.Max(10, capacity);
    }

    public void Add(TelemetrySample sample)
    {
        lock (_gate)
        {
            if (!_samples.TryGetValue(sample.Variable, out Queue<TelemetrySample>? queue))
            {
                queue = new Queue<TelemetrySample>();
                _samples[sample.Variable] = queue;
            }

            queue.Enqueue(sample);
            while (queue.Count > _capacity)
            {
                queue.Dequeue();
                _droppedSamples[sample.Variable] = _droppedSamples.GetValueOrDefault(sample.Variable) + 1;
            }
        }
    }

    public IReadOnlyList<TelemetrySample> GetSamples(string variable)
    {
        lock (_gate)
        {
            return _samples.TryGetValue(variable, out Queue<TelemetrySample>? queue)
                ? queue.ToArray()
                : Array.Empty<TelemetrySample>();
        }
    }

    public TelemetryStats GetStats(string variable, string unit)
    {
        lock (_gate)
        {
            if (!_samples.TryGetValue(variable, out Queue<TelemetrySample>? queue) || queue.Count == 0)
            {
                return new TelemetryStats(0, null, unit, null, null, 0, _droppedSamples.GetValueOrDefault(variable));
            }

            TelemetrySample[] values = queue.ToArray();
            TelemetrySample first = values[0];
            TelemetrySample last = values[^1];
            double seconds = Math.Max(0, (last.HostTimestamp - first.HostTimestamp).TotalSeconds);
            double rate = seconds > 0 && values.Length > 1 ? (values.Length - 1) / seconds : 0;

            return new TelemetryStats(
                values.Length,
                last.DisplayValue,
                last.Unit,
                last.TargetTickMs,
                last.HostTimestamp,
                rate,
                _droppedSamples.GetValueOrDefault(variable));
        }
    }

    public void Clear(string? variable = null)
    {
        lock (_gate)
        {
            if (string.IsNullOrWhiteSpace(variable))
            {
                _samples.Clear();
                _droppedSamples.Clear();
                return;
            }

            _samples.Remove(variable);
            _droppedSamples.Remove(variable);
        }
    }
}
