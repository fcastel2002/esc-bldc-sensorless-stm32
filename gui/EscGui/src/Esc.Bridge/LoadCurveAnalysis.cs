using Esc.Protocol;

namespace Esc.Bridge;

public enum LoadCurveMetric
{
    /// <summary>Mean DC-bus current, mA (measured by the INA219).</summary>
    BusCurrent,
    /// <summary>Mean PWM duty, percent.</summary>
    Duty,
    /// <summary>Mean DC-bus power, W.</summary>
    BusPower,
    /// <summary>Speed standard deviation inside the window, rpm (rises when the stir bar slips).</summary>
    RpmStd,
    /// <summary>
    /// Load current versus the tare: (I_bus - I_bus,tare) / d, mA. The bridge's own consumption
    /// cancels in the difference; dividing by the duty turns the bus-current difference into the
    /// extra winding current, which is proportional to the extra torque.
    /// </summary>
    LoadCurrentVsTare,
}

public sealed record LoadCurvePoint(double Rpm, double Value);

/// <summary>Pure helpers to turn sweep points into plottable curves and compare them with a tare.</summary>
public static class LoadCurveAnalysis
{
    public static bool NeedsTare(LoadCurveMetric metric) => metric == LoadCurveMetric.LoadCurrentVsTare;

    public static string Unit(LoadCurveMetric metric) => metric switch
    {
        LoadCurveMetric.Duty => "%",
        LoadCurveMetric.BusPower => "W",
        LoadCurveMetric.RpmStd => "rpm",
        _ => "mA",
    };

    public static double? Value(LoadSweepPoint point, LoadCurveMetric metric) => metric switch
    {
        LoadCurveMetric.Duty => point.DutyPercent,
        LoadCurveMetric.BusCurrent => point.HasPower ? point.BusCurrentMa : null,
        LoadCurveMetric.BusPower => point.HasPower ? point.BusPowerWatts : null,
        LoadCurveMetric.RpmStd => point.RpmStd,
        _ => null,
    };

    /// <summary>Points ordered by mean speed, skipping points without the requested metric.</summary>
    public static IReadOnlyList<LoadCurvePoint> Curve(IEnumerable<LoadSweepPoint> points, LoadCurveMetric metric)
    {
        ArgumentNullException.ThrowIfNull(points);
        return points
            .Select(point => (point.RpmMean, Value: Value(point, metric)))
            .Where(pair => pair.Value.HasValue)
            .Select(pair => new LoadCurvePoint(pair.RpmMean, pair.Value!.Value))
            .OrderBy(point => point.Rpm)
            .ToArray();
    }

    /// <summary>(I_bus - I_bus,tare(rpm)) / d for every point the tare covers.</summary>
    public static IReadOnlyList<LoadCurvePoint> LoadCurrentVsTare(
        IEnumerable<LoadSweepPoint> points,
        IEnumerable<LoadSweepPoint> tare)
    {
        ArgumentNullException.ThrowIfNull(points);
        IReadOnlyList<LoadCurvePoint> tareCurve = Curve(tare, LoadCurveMetric.BusCurrent);
        var result = new List<LoadCurvePoint>();
        foreach (LoadSweepPoint point in points.Where(p => p.HasPower && p.DutyE4 > 0).OrderBy(p => p.RpmMean))
        {
            double? baseline = Interpolate(tareCurve, point.RpmMean);
            if (baseline.HasValue)
            {
                result.Add(new LoadCurvePoint(point.RpmMean, (point.BusCurrentMa - baseline.Value) / (point.DutyE4 / 10000.0)));
            }
        }

        return result;
    }

    /// <summary>
    /// Linear interpolation of a curve (sorted by rpm) at the given speed. Returns null outside
    /// the curve span so a tare is never extrapolated.
    /// </summary>
    public static double? Interpolate(IReadOnlyList<LoadCurvePoint> curve, double rpm)
    {
        ArgumentNullException.ThrowIfNull(curve);
        if (curve.Count == 0 || rpm < curve[0].Rpm || rpm > curve[^1].Rpm) return null;
        for (int i = 1; i < curve.Count; i++)
        {
            LoadCurvePoint left = curve[i - 1];
            LoadCurvePoint right = curve[i];
            if (rpm > right.Rpm) continue;
            double span = right.Rpm - left.Rpm;
            if (span <= 0) return (left.Value + right.Value) / 2.0;
            double t = (rpm - left.Rpm) / span;
            return left.Value + t * (right.Value - left.Value);
        }

        return curve[^1].Value;
    }

    /// <summary>Difference against a tare (measured minus tare) where the tare covers the speed.</summary>
    public static IReadOnlyList<LoadCurvePoint> Subtract(
        IReadOnlyList<LoadCurvePoint> measured,
        IReadOnlyList<LoadCurvePoint> tare)
    {
        ArgumentNullException.ThrowIfNull(measured);
        ArgumentNullException.ThrowIfNull(tare);
        var result = new List<LoadCurvePoint>(measured.Count);
        foreach (LoadCurvePoint point in measured)
        {
            double? baseline = Interpolate(tare, point.Rpm);
            if (baseline.HasValue)
            {
                result.Add(point with { Value = point.Value - baseline.Value });
            }
        }

        return result;
    }
}
