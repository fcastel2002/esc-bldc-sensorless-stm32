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
    TelemetryStats SpeedTelemetry,
    HilBridgeStats Hil,
    ValidationReference? ValidationReference,
    ActiveControllerConfig? ActiveControllerConfig,
    ActiveSpeedLimits? ActiveSpeedLimits,
    DateTimeOffset? ValidationReferenceCapturedAt,
    string? ValidationReferenceError);

public sealed record ActiveControllerConfig(
    double Kp,
    double Ki,
    double Kd,
    byte PolePairs);

public sealed record ActiveSpeedLimits(
    ushort MinRpm,
    ushort MaxRpm);

public sealed record TelemetryStats(
    int SampleCount,
    double? LastValue,
    string Unit,
    uint? LastTargetTickMs,
    DateTimeOffset? LastHostTimestamp,
    double EffectiveRateHz,
    uint DroppedSamples);

public sealed record HilBridgeStats(
    bool Enabled,
    uint RxFrames,
    uint LostFrames,
    double EffectiveRateHz,
    double AverageRoundTripMs,
    double JitterMs,
    HilOutputs? LastOutputs,
    HilInputs? LastInputs)
{
    public IReadOnlyList<HilFrameTraceEntry> RecentFrames { get; init; } = Array.Empty<HilFrameTraceEntry>();
}

public sealed record HilFrameTraceEntry(
    DateTimeOffset Timestamp,
    string Direction,
    string Transport,
    uint? Sequence,
    string Summary,
    string Payload,
    uint RepeatCount = 1);
