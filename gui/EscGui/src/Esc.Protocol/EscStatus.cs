namespace Esc.Protocol;

public sealed record EscStatus(
    byte AppState,
    string AppStateName,
    string Transport,
    bool MotorStalled,
    bool ConsistentZeroCrossing,
    ushort SpeedSetpointRpm,
    ushort ActualSpeedRpm,
    ushort MaxPwm);
