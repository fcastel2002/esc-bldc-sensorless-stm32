namespace Esc.Protocol;

/// <summary>LOAD_SWEEP START request (12-byte payload).</summary>
/// <param name="StepOrRate">Staircase: step in rpm. Ramp: slew rate in rpm/s.</param>
/// <param name="SettleMs">Wait after the reference reaches each step (ramp: before the ramp starts).</param>
/// <param name="WindowMs">Averaging window per recorded point.</param>
public sealed record LoadSweepRequest(
    LoadSweepMode Mode,
    ushort StartRpm,
    ushort EndRpm,
    ushort StepOrRate,
    ushort SettleMs,
    ushort WindowMs,
    LoadSweepOptions Options = LoadSweepOptions.None);

/// <summary>LOAD_SWEEP GET_STATUS response (29-byte payload).</summary>
public sealed record LoadSweepStatus(
    LoadSweepState State,
    LoadSweepAbortReason AbortReason,
    LoadSweepMode Mode,
    LoadSweepOptions Options,
    ushort PointCount,
    ushort Capacity,
    ushort StartRpm,
    ushort EndRpm,
    ushort StepOrRate,
    ushort SettleMs,
    ushort WindowMs,
    ushort TargetRpm,
    int IdleCurrentMicroamps,
    uint ElapsedMs,
    bool IdleCurrentValid)
{
    public bool IsActive => State is >= LoadSweepState.MeasuringIdleCurrent and <= LoadSweepState.Measuring;
}

/// <summary>One averaged operating point recorded by the firmware sweep (16 bytes on the wire).</summary>
public sealed record LoadSweepPoint(
    int Index,
    ushort ReferenceRpm,
    ushort RpmMean,
    ushort DutyE4,
    ushort BusMillivolts,
    short BusCurrentMaX10,
    ushort DutyStdE4,
    ushort BusCurrentStdMaX10,
    byte RpmStd,
    LoadPointFlags Flags)
{
    public double DutyPercent => DutyE4 / 100.0;
    public double BusVolts => BusMillivolts / 1000.0;
    public double BusCurrentMa => BusCurrentMaX10 / 10.0;
    public double DutyStdPercent => DutyStdE4 / 100.0;
    public double BusCurrentStdMa => BusCurrentStdMaX10 / 10.0;
    public double BusPowerWatts => BusVolts * BusCurrentMa / 1000.0;
    public bool HasPower => !Flags.HasFlag(LoadPointFlags.NoPower);
}

/// <summary>LOAD_SWEEP GET_POINTS response: up to three consecutive points.</summary>
public sealed record LoadSweepPointBatch(ushort FirstIndex, IReadOnlyList<LoadSweepPoint> Points);
