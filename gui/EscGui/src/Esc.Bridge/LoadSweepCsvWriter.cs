using System.Globalization;
using System.Text;
using Esc.Protocol;

namespace Esc.Bridge;

/// <summary>CSV export of a firmware load sweep (one row per averaged point).</summary>
public static class LoadSweepCsvWriter
{
    public const string Header =
        "index,reference_rpm,rpm_mean,rpm_std,duty_pct,duty_std_pct,bus_v,bus_ma,bus_std_ma,bus_w,flags";

    public static string Write(LoadSweepStatus? status, IReadOnlyList<LoadSweepPoint> points)
    {
        ArgumentNullException.ThrowIfNull(points);
        CultureInfo inv = CultureInfo.InvariantCulture;
        var csv = new StringBuilder();

        if (status is not null)
        {
            csv.Append("# mode=").Append(status.Mode)
               .Append(" start_rpm=").Append(status.StartRpm.ToString(inv))
               .Append(" end_rpm=").Append(status.EndRpm.ToString(inv))
               .Append(status.Mode == LoadSweepMode.Ramp ? " ramp_rpm_s=" : " step_rpm=")
               .Append(status.StepOrRate.ToString(inv))
               .Append(" settle_ms=").Append(status.SettleMs.ToString(inv))
               .Append(" window_ms=").Append(status.WindowMs.ToString(inv))
               .Append(" idle_bus_ma=")
               .Append((status.IdleCurrentMicroamps / 1000.0).ToString("0.000", inv))
               .Append(status.IdleCurrentValid ? string.Empty : " (unmeasured)")
               .Append(" state=").Append(status.State)
               .Append(" abort=").Append(status.AbortReason)
               .AppendLine();
        }

        csv.AppendLine(Header);
        foreach (LoadSweepPoint point in points)
        {
            csv.Append(point.Index.ToString(inv)).Append(',')
               .Append(point.ReferenceRpm.ToString(inv)).Append(',')
               .Append(point.RpmMean.ToString(inv)).Append(',')
               .Append(point.RpmStd.ToString(inv)).Append(',')
               .Append(point.DutyPercent.ToString("0.00", inv)).Append(',')
               .Append(point.DutyStdPercent.ToString("0.00", inv)).Append(',')
               .Append(point.HasPower ? point.BusVolts.ToString("0.000", inv) : string.Empty).Append(',')
               .Append(point.HasPower ? point.BusCurrentMa.ToString("0.0", inv) : string.Empty).Append(',')
               .Append(point.HasPower ? point.BusCurrentStdMa.ToString("0.0", inv) : string.Empty).Append(',')
               .Append(point.HasPower ? point.BusPowerWatts.ToString("0.000", inv) : string.Empty).Append(',')
               .Append(((byte)point.Flags).ToString(inv))
               .AppendLine();
        }

        return csv.ToString();
    }
}

/// <summary>Reads CSVs written by <see cref="LoadSweepCsvWriter"/>, including older files with extra columns.</summary>
public static class LoadSweepCsvReader
{
    public static IReadOnlyList<LoadSweepPoint> Parse(string csv)
    {
        ArgumentNullException.ThrowIfNull(csv);
        string[] lines = csv.Split('\n')
            .Select(line => line.Trim())
            .Where(line => line.Length > 0 && !line.StartsWith('#'))
            .ToArray();
        if (lines.Length == 0)
        {
            return Array.Empty<LoadSweepPoint>();
        }

        string[] header = lines[0].Split(',').Select(column => column.Trim().ToLowerInvariant()).ToArray();
        int Column(string name) => Array.IndexOf(header, name);
        int rpmColumn = Column("rpm_mean");
        int dutyColumn = Column("duty_pct");
        int busColumn = Column("bus_ma");
        if (rpmColumn < 0 || dutyColumn < 0 || busColumn < 0)
        {
            throw new FormatException("faltan columnas rpm_mean, duty_pct o bus_ma.");
        }

        int indexColumn = Column("index");
        int referenceColumn = Column("reference_rpm");
        int rpmStdColumn = Column("rpm_std");
        int dutyStdColumn = Column("duty_std_pct");
        int voltColumn = Column("bus_v");
        int busStdColumn = Column("bus_std_ma");
        int flagsColumn = Column("flags");

        var points = new List<LoadSweepPoint>(lines.Length - 1);
        for (int row = 1; row < lines.Length; row++)
        {
            string[] cells = lines[row].Split(',');
            double? Value(int column) =>
                column >= 0 && column < cells.Length &&
                double.TryParse(cells[column], NumberStyles.Float, CultureInfo.InvariantCulture, out double parsed)
                    ? parsed
                    : null;

            double rpm = Value(rpmColumn) ?? throw new FormatException($"fila {row}: rpm_mean invalido.");
            double duty = Value(dutyColumn) ?? throw new FormatException($"fila {row}: duty_pct invalido.");
            double? bus = Value(busColumn);
            var flags = (LoadPointFlags)((int)(Value(flagsColumn) ?? 0) & 0x07);
            if (bus is null)
            {
                flags |= LoadPointFlags.NoPower;
            }

            points.Add(new LoadSweepPoint(
                (int)(Value(indexColumn) ?? row - 1),
                ToUShort(Value(referenceColumn) ?? rpm),
                ToUShort(rpm),
                ToUShort(duty * 100),
                ToUShort((Value(voltColumn) ?? 0) * 1000),
                (short)Math.Clamp(Math.Round((bus ?? 0) * 10), short.MinValue, short.MaxValue),
                ToUShort((Value(dutyStdColumn) ?? 0) * 100),
                ToUShort((Value(busStdColumn) ?? 0) * 10),
                (byte)Math.Clamp(Math.Round(Value(rpmStdColumn) ?? 0), 0, 255),
                flags));
        }

        return points;
    }

    private static ushort ToUShort(double value) => (ushort)Math.Clamp(Math.Round(value), 0, ushort.MaxValue);
}
