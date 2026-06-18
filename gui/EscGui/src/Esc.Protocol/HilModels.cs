namespace Esc.Protocol;

public sealed record HilInputs(
    ushort SpeedRpm,
    ushort ZeroCrossingPeriod,
    short LoadTorque,
    byte Flags,
    bool Enable);

public sealed record HilOutputs(
    uint TargetTickMs,
    byte AppState,
    ControlRuntimeMode Mode,
    ushort SetpointRpm,
    ushort MeasuredRpm,
    ushort PwmCommand,
    sbyte CommutationStep,
    byte Flags,
    bool TimedOut);
