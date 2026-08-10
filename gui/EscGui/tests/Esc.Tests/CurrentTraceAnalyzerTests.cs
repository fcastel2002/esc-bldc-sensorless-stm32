using Esc.Bridge;
using Esc.Protocol;

namespace Esc.Tests;

public sealed class CurrentTraceAnalyzerTests
{
    [Fact]
    public void NormalizeBuildsElapsedSecondsFromTargetTickDelays()
    {
        IReadOnlyList<CurrentTracePoint> points = CurrentTraceAnalyzer.Normalize(
        [
            SampleU(100, 1000),
            SampleU(130, 1400),
            SampleU(120, 2000)
        ]);

        Assert.Equal(3, points.Count);
        Assert.Equal(0, points[0].ElapsedSeconds, 3);
        Assert.Equal(0.4, points[1].ElapsedSeconds, 3);
        Assert.Equal(1.0, points[2].ElapsedSeconds, 3);
        Assert.Equal(120, points[2].ValueMa);
    }

    [Fact]
    public void NormalizeKeepsTimeMonotonicAcrossTargetTickWraparound()
    {
        IReadOnlyList<CurrentTracePoint> points = CurrentTraceAnalyzer.Normalize(
        [
            SampleU(100, uint.MaxValue - 49),
            SampleU(110, 50)
        ]);

        Assert.Equal(0.1, points[1].ElapsedSeconds, 3);
    }

    [Fact]
    public void NormalizeReturnsEmptyForEmptyInput()
    {
        Assert.Empty(CurrentTraceAnalyzer.Normalize(Array.Empty<TelemetrySample>()));
    }

    [Fact]
    public void CsvMergesBothChannelsByElapsedTime()
    {
        IReadOnlyList<CurrentTracePoint> u = CurrentTraceAnalyzer.Normalize(
        [
            SampleU(100, 1000),
            SampleU(200, 1100),
            SampleU(300, 1200)
        ]);
        IReadOnlyList<CurrentTracePoint> v = CurrentTraceAnalyzer.Normalize(
        [
            SampleV(400, 1000),
            SampleV(500, 1100),
            SampleV(600, 1200)
        ]);

        string csv = CurrentTraceAnalyzer.ToCsv(u, v);

        string[] rows = csv.Split("\r\n", StringSplitOptions.RemoveEmptyEntries);
        Assert.Equal("elapsed_s,iu_ma,iv_ma", rows[0]);
        Assert.Equal(4, rows.Length);
        Assert.Contains("0,100,400", rows[1]);
        Assert.Contains("0.1,200,500", rows[2]);
        Assert.Contains("0.2,300,600", rows[3]);
    }

    [Fact]
    public void CsvBlanksMissingChannelRowsWhenOnlyOneChannelHasASample()
    {
        IReadOnlyList<CurrentTracePoint> u = CurrentTraceAnalyzer.Normalize(
        [
            SampleU(100, 1000),
            SampleU(200, 1100)
        ]);
        IReadOnlyList<CurrentTracePoint> v = CurrentTraceAnalyzer.Normalize(
        [
            SampleV(400, 1000),
            SampleV(500, 1005)
        ]);

        string csv = CurrentTraceAnalyzer.ToCsv(u, v);
        string[] rows = csv.Split("\r\n", StringSplitOptions.RemoveEmptyEntries);

        Assert.Contains("0,100,400", rows[1]);
        Assert.Contains("0.005,,500", rows[2]);
        Assert.Contains("0.1,200,", rows[3]);
    }

    [Fact]
    public void CsvCanBeLimitedToCursorInterval()
    {
        IReadOnlyList<CurrentTracePoint> u = CurrentTraceAnalyzer.Normalize(
        [
            SampleU(100, 1000),
            SampleU(200, 1100),
            SampleU(300, 1200)
        ]);
        IReadOnlyList<CurrentTracePoint> v = CurrentTraceAnalyzer.Normalize(
        [
            SampleV(400, 1000),
            SampleV(500, 1100),
            SampleV(600, 1200)
        ]);

        string csv = CurrentTraceAnalyzer.ToCsv(u, v, 0.05, 0.15);

        Assert.DoesNotContain("0,100,400", csv);
        Assert.Contains("0.1,200,500", csv);
        Assert.DoesNotContain("0.2,300,600", csv);
    }

    private static TelemetrySample SampleU(int value, uint tickMs)
    {
        return new TelemetrySample("current_u", LogParam.CurrentU, value, "mA", tickMs, DateTimeOffset.UnixEpoch.AddMilliseconds(tickMs));
    }

    private static TelemetrySample SampleV(int value, uint tickMs)
    {
        return new TelemetrySample("current_v", LogParam.CurrentV, value, "mA", tickMs, DateTimeOffset.UnixEpoch.AddMilliseconds(tickMs));
    }
}