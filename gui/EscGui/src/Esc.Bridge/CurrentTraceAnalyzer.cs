using Esc.Protocol;
using System.Globalization;
using System.Text;

namespace Esc.Bridge;

public sealed record CurrentTracePoint(
    int SampleIndex,
    TelemetrySample Sample,
    double ElapsedSeconds)
{
    public double ValueMa => Sample.DisplayValue;
}

public static class CurrentTraceAnalyzer
{
    private const double MergeToleranceSeconds = 0.001;

    public static IReadOnlyList<CurrentTracePoint> Normalize(IReadOnlyList<TelemetrySample> samples)
    {
        if (samples.Count == 0)
        {
            return Array.Empty<CurrentTracePoint>();
        }

        var points = new CurrentTracePoint[samples.Count];
        ulong elapsedMs = 0;
        points[0] = new CurrentTracePoint(0, samples[0], 0);

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
            points[index] = new CurrentTracePoint(index, current, elapsedMs / 1000.0);
        }

        return points;
    }

    public static string ToCsv(
        IReadOnlyList<CurrentTracePoint> pointsU,
        IReadOnlyList<CurrentTracePoint> pointsV,
        double startSeconds = 0,
        double endSeconds = double.MaxValue)
    {
        IReadOnlyList<CurrentTracePoint> u = Slice(pointsU, startSeconds, endSeconds);
        IReadOnlyList<CurrentTracePoint> v = Slice(pointsV, startSeconds, endSeconds);

        var csv = new StringBuilder("elapsed_s,iu_ma,iv_ma\r\n");
        int i = 0;
        int j = 0;
        while (i < u.Count && j < v.Count)
        {
            CurrentTracePoint pu = u[i];
            CurrentTracePoint pv = v[j];
            if (Math.Abs(pu.ElapsedSeconds - pv.ElapsedSeconds) <= MergeToleranceSeconds)
            {
                AppendCsvRow(csv, pu.ElapsedSeconds, pu.ValueMa, pv.ValueMa);
                i++;
                j++;
            }
            else if (pu.ElapsedSeconds < pv.ElapsedSeconds)
            {
                AppendCsvRow(csv, pu.ElapsedSeconds, pu.ValueMa, null);
                i++;
            }
            else
            {
                AppendCsvRow(csv, pv.ElapsedSeconds, null, pv.ValueMa);
                j++;
            }
        }

        while (i < u.Count)
        {
            CurrentTracePoint point = u[i++];
            AppendCsvRow(csv, point.ElapsedSeconds, point.ValueMa, null);
        }

        while (j < v.Count)
        {
            CurrentTracePoint point = v[j++];
            AppendCsvRow(csv, point.ElapsedSeconds, null, point.ValueMa);
        }

        return csv.ToString();
    }

    private static IReadOnlyList<CurrentTracePoint> Slice(
        IReadOnlyList<CurrentTracePoint> points,
        double startSeconds,
        double endSeconds)
    {
        if (points.Count == 0)
        {
            return Array.Empty<CurrentTracePoint>();
        }

        int start = 0;
        while (start < points.Count && points[start].ElapsedSeconds < startSeconds)
        {
            start++;
        }

        int end = points.Count;
        while (end > start && points[end - 1].ElapsedSeconds > endSeconds)
        {
            end--;
        }

        var result = new CurrentTracePoint[end - start];
        for (var index = 0; index < result.Length; index++)
        {
            result[index] = points[start + index];
        }

        return result;
    }

    private static void AppendCsvRow(StringBuilder csv, double elapsedSeconds, double? iuMa, double? ivMa)
    {
        csv.Append(elapsedSeconds.ToString("0.###", CultureInfo.InvariantCulture)).Append(',')
            .Append(iuMa?.ToString("0.###", CultureInfo.InvariantCulture) ?? string.Empty).Append(',')
            .Append(ivMa?.ToString("0.###", CultureInfo.InvariantCulture) ?? string.Empty).Append("\r\n");
    }
}
