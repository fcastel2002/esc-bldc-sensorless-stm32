using Esc.Protocol;
using Esc.Transport;

namespace Esc.Bridge;

public sealed record BridgeSnapshot(
    DeviceConnectionState State,
    ControlMode Mode,
    IReadOnlyList<HidDeviceDescriptor> Devices,
    HidDeviceDescriptor? CurrentDevice,
    EscStatus? Status,
    string? LastError,
    DateTimeOffset LastUpdated,
    bool SpeedLoggingEnabled,
    ushort LogRateMs,
    TelemetryStats SpeedTelemetry);

public sealed record TelemetryStats(
    int SampleCount,
    double? LastValue,
    string Unit,
    uint? LastTargetTickMs,
    DateTimeOffset? LastHostTimestamp,
    double EffectiveRateHz,
    uint DroppedSamples);
