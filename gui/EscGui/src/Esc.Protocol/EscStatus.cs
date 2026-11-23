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

public enum PowerMonitorStage : byte
{
    None = 0,
    Probe = 1,
    ConfigWrite = 2,
    ConfigVerify = 3,
    ReadShunt = 4,
    ReadBus = 5,
    IdleBusy = 6,
}

/// <summary>INA219 DC-bus reading carried by the extended GET_STATUS payload (bytes 10..17).</summary>
public sealed record PowerMonitorReading(
    PowerMonitorFlags Flags,
    byte I2cAddress,
    ushort BusMillivolts,
    int CurrentMicroamps,
    PowerMonitorError LastError = PowerMonitorError.None,
    byte ErrorCount = 0,
    byte? LineState = null,
    PowerMonitorStage FailStage = PowerMonitorStage.None,
    ushort ConfigReadback = 0,
    ushort BusyRecoveries = 0)
{
    public bool? SclIdleHigh => LineState is { } s ? (s & 0x01) != 0 : null;
    public bool? SdaIdleHigh => LineState is { } s ? (s & 0x02) != 0 : null;
    public bool Present => Flags.HasFlag(PowerMonitorFlags.Present);
    public bool Valid => Flags.HasFlag(PowerMonitorFlags.Valid);
    public bool Overflow => Flags.HasFlag(PowerMonitorFlags.Overflow);
    public bool Disabled => Flags.HasFlag(PowerMonitorFlags.Disabled);
    public double BusVolts => BusMillivolts / 1000.0;
    public double CurrentAmps => CurrentMicroamps / 1_000_000.0;
    public double PowerWatts => BusVolts * CurrentAmps;
}
