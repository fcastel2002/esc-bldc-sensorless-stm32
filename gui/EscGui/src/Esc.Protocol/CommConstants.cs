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
    SetControlMode = 0x14,
    GetConfig = 0x20,
    SetConfig = 0x21,
    ResetConfig = 0x22,
    SaveConfig = 0x23,
    LogStart = 0x30,
    LogStop = 0x31,
    LogRate = 0x32,
    TelemetryEvent = 0x33,
    HilStart = 0x40,
    HilStop = 0x41,
    HilSetInputs = 0x42,
    HilGetOutputs = 0x43,
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

public enum ControlRuntimeMode : byte
{
    Normal = 0,
    MonitorOnly = 1,
    HilSim = 2,
}

public enum ConfigParam : byte
{
    PwmFreq = 0x01,
    PolePairs = 0x02,
    Kp = 0x03,
    Ki = 0x04,
    Kd = 0x05,
    MaxSpeed = 0x06,
    MinSpeed = 0x07,
    CurrentLimit = 0x08,
    TempLimit = 0x09,
    All = 0xFF,
}

public enum LogParam : byte
{
    Speed = 0x01,
    Temperature = 0x02,
    Current = 0x03,
    All = 0xFF,
}
