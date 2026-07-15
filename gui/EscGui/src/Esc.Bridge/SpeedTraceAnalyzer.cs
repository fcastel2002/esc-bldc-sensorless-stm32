using Esc.Protocol;
using System.Globalization;
using System.Text;

namespace Esc.Bridge;

public enum SpeedTransientDirection
{
    None,
    Rising,
    Falling
}

public sealed record SpeedTracePoint(
    int SampleIndex,
    TelemetrySample Sample,
    double ElapsedSeconds)
{
    public double SpeedRpm => Sample.DisplayValue;
}

public sealed record SpeedIntervalMeasurement(
    int StartIndex,
    int EndIndex,
    SpeedTracePoint Start,
    SpeedTracePoint End,
    double DeltaSeconds,
    double DeltaRpm,
    double ReferenceRpm,
    SpeedTransientDirection Direction,
    SpeedTracePoint Extreme,
    double OvershootRpm,
    double? OvershootPercent);

public static class SpeedTraceAnalyzer
{
    public static IReadOnlyList<SpeedTracePoint> Normalize(IReadOnlyList<TelemetrySample> samples)
    {
        if (samples.Count == 0)
        {
            return Array.Empty<SpeedTracePoint>();
        }

        var points = new SpeedTracePoint[samples.Count];
        ulong elapsedMs = 0;
        points[0] = new SpeedTracePoint(0, samples[0], 0);

        for (var index = 1; index < samples.Count; index++)
        {
            TelemetrySample previous = samples[index - 1];
            TelemetrySample current = samples[index];
            ulong deltaMs;

            if (current.TargetTickMs >= previous.TargetTickMs)
            {
                deltaMs = current.TargetTickMs - previous.TargetTickMs;
            }
            else if (previous.TargetTickMs - current.TargetTickMs > int.MaxValue)
            {
                deltaMs = (ulong)uint.MaxValue - previous.TargetTickMs + 1UL + current.TargetTickMs;
            }
            else
            {
                deltaMs = (ulong)Math.Max(0, Math.Round((current.HostTimestamp - previous.HostTimestamp).TotalMilliseconds));
            }

            elapsedMs += deltaMs;
            points[index] = new SpeedTracePoint(index, current, elapsedMs / 1000.0);
        }

        return points;
    }

    public static SpeedIntervalMeasurement? Measure(
        IReadOnlyList<SpeedTracePoint> points,
        int cursorA,
        int cursorB,
        double referenceRpm)
    {
        if (points.Count == 0 || cursorA < 0 || cursorB < 0 || cursorA >= points.Count || cursorB >= points.Count)
        {
            return null;
        }

        int startIndex = Math.Min(cursorA, cursorB);
        int endIndex = Math.Max(cursorA, cursorB);
        SpeedTracePoint start = points[startIndex];
        SpeedTracePoint end = points[endIndex];
        SpeedTransientDirection direction = referenceRpm.CompareTo(start.SpeedRpm) switch
        {
            > 0 => SpeedTransientDirection.Rising,
            < 0 => SpeedTransientDirection.Falling,
            _ => SpeedTransientDirection.None
        };

        SpeedTracePoint extreme = direction == SpeedTransientDirection.Falling
            ? points.Skip(startIndex).Take(endIndex - startIndex + 1).MinBy(point => point.SpeedRpm)!
            : points.Skip(startIndex).Take(endIndex - startIndex + 1).MaxBy(point => point.SpeedRpm)!;

        double overshootRpm = direction switch
        {
            SpeedTransientDirection.Rising => Math.Max(0, extreme.SpeedRpm - referenceRpm),
            SpeedTransientDirection.Falling => Math.Max(0, referenceRpm - extreme.SpeedRpm),
            _ => 0
        };
        double stepAmplitude = Math.Abs(referenceRpm - start.SpeedRpm);
        double? overshootPercent = stepAmplitude > double.Epsilon
            ? overshootRpm / stepAmplitude * 100.0
            : null;

        return new SpeedIntervalMeasurement(
            startIndex,
            endIndex,
            start,
            end,
            end.ElapsedSeconds - start.ElapsedSeconds,
            end.SpeedRpm - start.SpeedRpm,
            referenceRpm,
            direction,
            extreme,
            overshootRpm,
            overshootPercent);
    }

    public static string ToCsv(IReadOnlyList<SpeedTracePoint> points, int? cursorA = null, int? cursorB = null)
    {
        int startIndex = 0;
        int endIndex = points.Count - 1;
        if (cursorA.HasValue && cursorB.HasValue && points.Count > 0)
        {
            startIndex = Math.Clamp(Math.Min(cursorA.Value, cursorB.Value), 0, points.Count - 1);
            endIndex = Math.Clamp(Math.Max(cursorA.Value, cursorB.Value), 0, points.Count - 1);
        }

        var csv = new StringBuilder("target_tick_ms,elapsed_s,host_timestamp,speed_rpm\r\n");
        for (var index = startIndex; index <= endIndex; index++)
        {
            SpeedTracePoint point = points[index];
            csv.Append(point.Sample.TargetTickMs.ToString(CultureInfo.InvariantCulture)).Append(',')
                .Append(point.ElapsedSeconds.ToString("0.###", CultureInfo.InvariantCulture)).Append(',')
                .Append(point.Sample.HostTimestamp.ToString("O", CultureInfo.InvariantCulture)).Append(',')
                .Append(point.SpeedRpm.ToString("0.###", CultureInfo.InvariantCulture)).Append("\r\n");
        }

        return csv.ToString();
    }
}
