namespace Esc.Protocol;

public static class CommConstants
{
    public const byte Magic0 = 0xEC;
    public const byte Magic1 = 0xB1;
    public const byte Version = 0x01;

    public const int FrameSize = 64;
    public const int HeaderSize = 10;
    public const int PayloadOffset = 10;
    public const int CrcOffset = 62;
    public const int PayloadMax = CrcOffset - HeaderSize;

    public const int DefaultVendorId = 0x3232;
    public const int DefaultProductId = 0xEC32;

    public const double MinSineFrequencyHz = 2.0;
    public const double MaxSineFrequencyHz = 10.0;
    public const double MaxSineAmplitudePercent = 100.0;
    public const double MaxStepTestDutyPercent = 30.0;
    public const double MinZcAutoDetectDutyPercent = 5.0;
    public const double MaxZcAutoDetectDutyPercent = 25.0;
    public const double DefaultZcAutoDetectDutyPercent = 15.0;
    public const double MinStartupDurationSeconds = 0.5;
    public const double MaxStartupDurationSeconds = 10.0;
    public const int MaxSpeedRampRpmPerSecond = 20000;
    public const int LoadSweepPointSize = 16;
    public const int LoadSweepPointsPerFrame = 3;
    public const int LoadSweepMinWindowMs = 100;
    public const int LoadSweepMaxWindowMs = 10000;
    public const int LoadSweepMaxSettleMs = 30000;
    public const int LoadSweepMinStepRpm = 10;
    public const int LoadSweepMaxRampRpmPerSecond = 5000;
}

public enum CommFrameType : byte
{
    Request = 0x01,
    Response = 0x81,
    Event = 0x82,
}

public enum CommOpcode : byte
{
    Ping = 0x01,
    GetStatus = 0x02,
    Run = 0x10,
    Stop = 0x11,
    EmergencyStop = 0x12,
    SetSpeedRpm = 0x13,
    GetConfig = 0x20,
    SetConfig = 0x21,
    ResetConfig = 0x22,
    SaveConfig = 0x23,
    LogStart = 0x30,
    LogStop = 0x31,
    LogRate = 0x32,
    TelemetryEvent = 0x33,
    SineDrive = 0x50,
    StepTest = 0x51,
    ZcDiag = 0x52,
    LoadSweep = 0x60,
}

public enum CommStatus : byte
{
    Ok = 0x00,
    BadMagic = 0x01,
    BadVersion = 0x02,
    BadCrc = 0x03,
    BadLength = 0x04,
    UnknownOpcode = 0x05,
    UnknownParam = 0x06,
    InvalidState = 0x07,
    UnderLimit = 0x08,
    OverLimit = 0x09,
    NotImplemented = 0x0A,
    FlashError = 0x0B,
}

public enum SineDriveCommand : byte
{
    Apply = 0,
    KeepAlive = 1,
}

public enum StepTestCommand : byte
{
    Apply = 0,
    KeepAlive = 1,
}

public enum ZcDiagCommand : byte
{
    Start = 0,
    Stop = 1,
    GetReport = 2,
    AutoDetect = 3,
}

public enum ZcAutoDetectStatus : byte
{
    Ok = 0,
    OkInverted = 1,
    NoSignal = 2,
    Ambiguous = 3,
}

[Flags]
public enum ZcHealthFlags : byte
{
    None = 0,
    HasMotion = 1 << 0,
    SequenceValid = 1 << 1,
    DirectionForward = 1 << 2,
    DirectionReverse = 1 << 3,
    Channel0Dead = 1 << 4,
    Channel1Dead = 1 << 5,
    Channel2Dead = 1 << 6,
    NoiseOrInvalid = 1 << 7,
}

/// <summary>Mismo orden que POS_UV..POS_WV en firmware (basic_config.h).</summary>
public enum CommutationStep : byte
{
    UV = 0,
    UW = 1,
    VW = 2,
    VU = 3,
    WU = 4,
    WV = 5,
}

public enum ConfigParam : byte
{
    PwmFreq = 0x01,
    PolePairs = 0x02,
    MaxSpeed = 0x06,
    MinSpeed = 0x07,
    KpRpm = 0x0A,
    KiRpm = 0x0B,
    KdRpm = 0x0C,
    StartupInitialAmplitude = 0x0D,
    StartupFinalAmplitude = 0x0E,
    StartupInitialFrequency = 0x0F,
    StartupFinalFrequency = 0x10,
    StartupDuration = 0x11,
    BemfBlankingUs = 0x12,
    ZcMap = 0x13,
    SpeedRamp = 0x14,
    All = 0xFF,
}

public enum LogParam : byte
{
    Speed = 0x01,
    BusVoltage = 0x06,
    BusCurrent = 0x07,
    /// <summary>Commanded PWM duty, 1/10000 of the TIM1 period.</summary>
    Duty = 0x08,
    All = 0xFF,
}

[Flags]
public enum TelemetryQuality : byte
{
    None = 0,
    Valid = 1 << 0,
}

public enum LoadSweepCommand : byte
{
    Start = 0,
    Abort = 1,
    GetStatus = 2,
    GetPoints = 3,
}

public enum LoadSweepMode : byte
{
    Staircase = 0,
    Ramp = 1,
}

public enum LoadSweepState : byte
{
    Idle = 0,
    MeasuringIdleCurrent = 1,
    Starting = 2,
    Moving = 3,
    Settling = 4,
    Measuring = 5,
    Done = 6,
    Aborted = 7,
}

public enum LoadSweepAbortReason : byte
{
    None = 0,
    User = 1,
    MotorLost = 2,
    StartupTimeout = 3,
    TableFull = 4,
    Stopped = 5,
}

[Flags]
public enum LoadSweepOptions : byte
{
    None = 0,
    StopAtEnd = 1 << 0,
}

[Flags]
public enum LoadPointFlags : byte
{
    None = 0,
    NoPower = 1 << 0,
    PartialPower = 1 << 1,
    DutySaturated = 1 << 2,
}
