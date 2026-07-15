using Esc.Bridge;
using Esc.Protocol;

namespace Esc.Tests;

public sealed class SpeedTraceAnalyzerTests
{
    [Fact]
    public void MeasureCalculatesRisingOvershootInsideCursorInterval()
    {
        IReadOnlyList<SpeedTracePoint> points = SpeedTraceAnalyzer.Normalize(
        [
            Sample(900, 1000),
            Sample(1500, 1100),
            Sample(2250, 1200),
            Sample(2050, 1300)
        ]);

        SpeedIntervalMeasurement measurement = Assert.IsType<SpeedIntervalMeasurement>(
            SpeedTraceAnalyzer.Measure(points, 0, 3, 2000));

        Assert.Equal(SpeedTransientDirection.Rising, measurement.Direction);
        Assert.Equal(2250, measurement.Extreme.SpeedRpm);
        Assert.Equal(250, measurement.OvershootRpm);
        Assert.Equal(22.727, measurement.OvershootPercent!.Value, 3);
        Assert.Equal(0.3, measurement.DeltaSeconds, 3);
    }

    [Fact]
    public void MeasureCalculatesFallingUndershoot()
    {
        IReadOnlyList<SpeedTracePoint> points = SpeedTraceAnalyzer.Normalize(
        [
            Sample(3000, 1000),
            Sample(1800, 1100),
            Sample(1300, 1200),
            Sample(1550, 1300)
        ]);

        SpeedIntervalMeasurement measurement = Assert.IsType<SpeedIntervalMeasurement>(
            SpeedTraceAnalyzer.Measure(points, 0, 3, 1500));

        Assert.Equal(SpeedTransientDirection.Falling, measurement.Direction);
        Assert.Equal(1300, measurement.Extreme.SpeedRpm);
        Assert.Equal(200, measurement.OvershootRpm);
        Assert.Equal(13.333, measurement.OvershootPercent!.Value, 3);
    }

    [Fact]
    public void MeasureOrdersReversedCursorsChronologically()
    {
        IReadOnlyList<SpeedTracePoint> points = SpeedTraceAnalyzer.Normalize(
        [
            Sample(1000, 1000),
            Sample(1500, 1100),
            Sample(2100, 1200)
        ]);

        SpeedIntervalMeasurement measurement = Assert.IsType<SpeedIntervalMeasurement>(
            SpeedTraceAnalyzer.Measure(points, 2, 0, 2000));

        Assert.Equal(0, measurement.StartIndex);
        Assert.Equal(2, measurement.EndIndex);
        Assert.Equal(0.2, measurement.DeltaSeconds, 3);
    }

    [Fact]
    public void MeasureReturnsNoPercentageForZeroAmplitudeStep()
    {
        IReadOnlyList<SpeedTracePoint> points = SpeedTraceAnalyzer.Normalize(
        [
            Sample(1500, 1000),
            Sample(1700, 1100)
        ]);

        SpeedIntervalMeasurement measurement = Assert.IsType<SpeedIntervalMeasurement>(
            SpeedTraceAnalyzer.Measure(points, 0, 1, 1500));

        Assert.Equal(SpeedTransientDirection.None, measurement.Direction);
        Assert.Equal(0, measurement.OvershootRpm);
        Assert.Null(measurement.OvershootPercent);
    }

    [Fact]
    public void NormalizeKeepsTimeMonotonicAcrossTargetTickWraparound()
    {
        IReadOnlyList<SpeedTracePoint> points = SpeedTraceAnalyzer.Normalize(
        [
            Sample(1000, uint.MaxValue - 49),
            Sample(1100, 50)
        ]);

        Assert.Equal(0.1, points[1].ElapsedSeconds, 3);
    }

    [Fact]
    public void CsvCanBeLimitedToCursorInterval()
    {
        IReadOnlyList<SpeedTracePoint> points = SpeedTraceAnalyzer.Normalize(
        [
            Sample(1000, 100),
            Sample(1100, 200),
            Sample(1200, 300)
        ]);

        string csv = SpeedTraceAnalyzer.ToCsv(points, 1, 2);

        Assert.DoesNotContain("100,0,", csv);
        Assert.Contains("200,0.1,", csv);
        Assert.Contains("300,0.2,", csv);
    }

    private static TelemetrySample Sample(int rpm, uint tickMs)
    {
        return new TelemetrySample("speed", LogParam.Speed, rpm, "rpm", tickMs, DateTimeOffset.UnixEpoch.AddMilliseconds(tickMs));
    }
}
