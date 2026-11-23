namespace Esc.Protocol;

public sealed record EscStatus(
    byte AppState,
    string AppStateName,
    string Transport,
    bool MotorStalled,
    bool ConsistentZeroCrossing,
    ushort SpeedSetpointRpm,
    ushort ActualSpeedRpm,
    ushort MaxPwm,
    PowerMonitorReading? Power = null);

[Flags]
public enum PowerMonitorFlags : byte
{
    None = 0,
    Present = 1 << 0,
    Valid = 1 << 1,
    Overflow = 1 << 2,
    Disabled = 1 << 3,
}

public enum PowerMonitorError : byte
{
    None = 0,
    NotFound = 1,
    Nack = 2,
    Timeout = 3,
    Bus = 4,
    Busy = 5,
    Other = 6,
}

/// <summary>INA219 DC-bus reading carried by the extended GET_STATUS payload (bytes 10..17).</summary>
public sealed record PowerMonitorReading(
    PowerMonitorFlags Flags,
    byte I2cAddress,
    ushort BusMillivolts,
    int CurrentMicroamps,
    PowerMonitorError LastError = PowerMonitorError.None,
    byte ErrorCount = 0)
{
    public bool Present => Flags.HasFlag(PowerMonitorFlags.Present);
    public bool Valid => Flags.HasFlag(PowerMonitorFlags.Valid);
    public bool Overflow => Flags.HasFlag(PowerMonitorFlags.Overflow);
    public bool Disabled => Flags.HasFlag(PowerMonitorFlags.Disabled);
    public double BusVolts => BusMillivolts / 1000.0;
    public double CurrentAmps => CurrentMicroamps / 1_000_000.0;
    public double PowerWatts => BusVolts * CurrentAmps;
}
