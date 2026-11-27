using Esc.Bridge;
using Esc.Protocol;

namespace Esc.Tests;

public sealed class LoadCurveAnalysisTests
{
    [Fact]
    public void CurveOrdersBySpeedAndSkipsPointsWithoutTheMetric()
    {
        LoadSweepPoint[] points =
        [
            Point(0, 2000, busX10: 900),
            Point(1, 1000, busX10: 800),
            Point(2, 1500, busX10: 0, flags: LoadPointFlags.NoPower),
        ];

        IReadOnlyList<LoadCurvePoint> curve = LoadCurveAnalysis.Curve(points, LoadCurveMetric.BusCurrent);

        Assert.Equal([new LoadCurvePoint(1000, 80), new LoadCurvePoint(2000, 90)], curve);
    }

    [Fact]
    public void DutyAndSpeedDispersionCurvesKeepPointsWithoutPower()
    {
        LoadSweepPoint[] points = [Point(0, 1000, dutyE4: 2500, flags: LoadPointFlags.NoPower)];

        Assert.Single(LoadCurveAnalysis.Curve(points, LoadCurveMetric.Duty));
        Assert.Single(LoadCurveAnalysis.Curve(points, LoadCurveMetric.RpmStd));
        Assert.Empty(LoadCurveAnalysis.Curve(points, LoadCurveMetric.BusCurrent));
    }

    [Theory]
    [InlineData(1000, 10.0)]
    [InlineData(1500, 15.0)]
    [InlineData(1750, 17.5)]
    [InlineData(2000, 20.0)]
    public void InterpolateIsLinearInsideTheCurve(double rpm, double expected)
    {
        LoadCurvePoint[] curve = [new(1000, 10), new(2000, 20)];

        Assert.Equal(expected, LoadCurveAnalysis.Interpolate(curve, rpm)!.Value, 9);
    }

    [Theory]
    [InlineData(999)]
    [InlineData(2001)]
    public void InterpolateNeverExtrapolates(double rpm)
    {
        LoadCurvePoint[] curve = [new(1000, 10), new(2000, 20)];

        Assert.Null(LoadCurveAnalysis.Interpolate(curve, rpm));
    }

    [Fact]
    public void SubtractRemovesTheTareOnlyWhereItIsDefined()
    {
        LoadCurvePoint[] measured = [new(800, 9), new(1000, 15), new(1500, 30), new(2500, 70)];
        LoadCurvePoint[] tare = [new(1000, 10), new(2000, 20)];

        IReadOnlyList<LoadCurvePoint> delta = LoadCurveAnalysis.Subtract(measured, tare);

        Assert.Equal([new LoadCurvePoint(1000, 5), new LoadCurvePoint(1500, 15)], delta);
    }

    [Fact]
    public void LoadCurrentVsTareDividesTheBusDifferenceByTheMeasuredDuty()
    {
        LoadSweepPoint[] air = [Point(0, 1000, busX10: 800), Point(1, 2000, busX10: 900)];
        LoadSweepPoint[] water = [Point(0, 1500, dutyE4: 4000, busX10: 1050), Point(1, 2500, dutyE4: 5000, busX10: 1200)];

        IReadOnlyList<LoadCurvePoint> load = LoadCurveAnalysis.LoadCurrentVsTare(water, air);

        LoadCurvePoint only = Assert.Single(load);
        Assert.Equal(1500, only.Rpm);
        Assert.Equal((105.0 - 85.0) / 0.40, only.Value, 9);
    }

    [Fact]
    public void CsvRoundTripsThroughTheReader()
    {
        var status = new LoadSweepStatus(LoadSweepState.Done, LoadSweepAbortReason.None, LoadSweepMode.Ramp,
            LoadSweepOptions.None, 1, 128, 1000, 2000, 50, 0, 500, 2000, 12_500, 1_000, true);
        LoadSweepPoint[] points = [Point(0, 1234, dutyE4: 3456, busX10: 987, dutyStdE4: 12)];

        string csv = LoadSweepCsvWriter.Write(status, points);
        string[] lines = csv.Split('\n', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
        LoadSweepPoint parsed = Assert.Single(LoadSweepCsvReader.Parse(csv));

        Assert.Contains("ramp_rpm_s=50", lines[0], StringComparison.Ordinal);
        Assert.Contains("idle_bus_ma=12.500", lines[0], StringComparison.Ordinal);
        Assert.Equal(LoadSweepCsvWriter.Header, lines[1]);
        Assert.StartsWith("0,1234,1234,0,34.56,0.12,", lines[2], StringComparison.Ordinal);
        Assert.Equal(points[0], parsed);
    }

    [Fact]
    public void CsvReaderAcceptsTheFirstExportFormatWithLoadColumn()
    {
        const string csv = """
            # mode=Staircase start_rpm=700 end_rpm=3000 step_rpm=250 settle_ms=1500 window_ms=2000 idle_current_ma=19.908 state=Done abort=None
            index,reference_rpm,rpm_mean,rpm_std,duty_pct,bus_v,bus_ma,bus_std_ma,bus_w,load_ma,flags
            1,950,939,21,23.79,12.289,76.6,5.9,0.941,238.3,0
            2,1200,1194,17,26.93,12.288,,4.5,,219.1,24
            """;

        IReadOnlyList<LoadSweepPoint> points = LoadSweepCsvReader.Parse(csv);

        Assert.Equal(2, points.Count);
        Assert.Equal((ushort)939, points[0].RpmMean);
        Assert.Equal(76.6, points[0].BusCurrentMa, 6);
        Assert.Equal(23.79, points[0].DutyPercent, 6);
        Assert.True(points[1].Flags.HasFlag(LoadPointFlags.NoPower));
    }

    [Fact]
    public void CsvReaderRejectsFilesWithoutRequiredColumns()
    {
        Assert.Throws<FormatException>(() => LoadSweepCsvReader.Parse("a,b,c\n1,2,3"));
    }

    private static LoadSweepPoint Point(
        int index,
        ushort rpm,
        ushort dutyE4 = 3000,
        short busX10 = 500,
        ushort dutyStdE4 = 0,
        LoadPointFlags flags = LoadPointFlags.None) =>
        new(index, rpm, rpm, dutyE4, 12_000, busX10, dutyStdE4, 10, 0, flags);
}
