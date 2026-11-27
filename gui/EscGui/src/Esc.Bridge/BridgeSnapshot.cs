using Esc.Protocol;
using Esc.Transport;

namespace Esc.Bridge;

public sealed record BridgeSnapshot(
    DeviceConnectionState State,
    bool TransportOpen,
    ControlMode Mode,
    IReadOnlyList<HidDeviceDescriptor> Devices,
    HidDeviceDescriptor? CurrentDevice,
    EscStatus? Status,
    string? LastError,
    DateTimeOffset LastUpdated,
    bool SpeedLoggingEnabled,
    ushort LogRateMs,
    TelemetryStats SpeedTelemetry,
    ActiveControllerConfig? ActiveControllerConfig,
    ActiveSpeedLimits? ActiveSpeedLimits,
    StartupConfiguration? StartupConfiguration,
    SineDriveStatus SineDrive,
    StepTestStatus StepTest,
    FreewheelStatus Freewheel,
    LoadSweepSnapshot LoadSweep)
{
    public bool BusVoltageLoggingEnabled { get; init; }
    public bool BusCurrentLoggingEnabled { get; init; }
    public bool DutyLoggingEnabled { get; init; }
    public TelemetryStats BusVoltageTelemetry { get; init; } = TelemetryStats.Empty("mV");
    public TelemetryStats BusCurrentTelemetry { get; init; } = TelemetryStats.Empty("mA");
    public TelemetryStats DutyTelemetry { get; init; } = TelemetryStats.Empty("%");
}

/// <summary>Controller configuration read back from the MCU after connecting or changing it.</summary>
public sealed record ActiveControllerConfig(
    double Kp,
    double Ki,
    double Kd,
    byte PolePairs,
    ushort PwmFrequencyHz,
    ushort SpeedRampRpmPerSecond);

public sealed record ActiveSpeedLimits(
    ushort MinRpm,
    ushort MaxRpm);

public sealed record StartupConfiguration(
    double InitialAmplitudePercent,
    double FinalAmplitudePercent,
    double InitialFrequencyHz,
    double FinalFrequencyHz,
    double DurationSeconds);

public sealed record SineDriveStatus(
    bool Active,
    SineDriveSettings? Requested,
    SineDriveSettings? Applied,
    DateTimeOffset? LastAppliedAt);

public sealed record StepTestStatus(
    bool Active,
    StepTestSettings? Requested,
    StepTestSettings? Applied,
    DateTimeOffset? LastAppliedAt);

public sealed record FreewheelStatus(
    bool Active,
    ZcDiagReport? Report,
    ZcChannelMap? ActiveMap,
    ZcAutoDetectResult? LastAutoDetect,
    DateTimeOffset? LastUpdatedAt);

/// <summary>Last firmware sweep status plus the points already copied from the MCU table.</summary>
public sealed record LoadSweepSnapshot(
    LoadSweepStatus? Status,
    IReadOnlyList<LoadSweepPoint> Points,
    DateTimeOffset? LastUpdatedAt,
    string? LastError)
{
    public static LoadSweepSnapshot Empty { get; } = new(null, Array.Empty<LoadSweepPoint>(), null, null);
    public bool Active => Status?.IsActive == true;

    /// <summary>Curves kept on the host for comparison (saved from the MCU or imported from CSV).</summary>
    public IReadOnlyList<SavedLoadCurve> SavedCurves { get; init; } = Array.Empty<SavedLoadCurve>();

    /// <summary>Saved curve used as tare (for example the stir bar in air), if any.</summary>
    public Guid? TareCurveId { get; init; }

    public SavedLoadCurve? TareCurve => SavedCurves.FirstOrDefault(curve => curve.Id == TareCurveId);
}

/// <summary>A load curve kept on the host; never sent to the MCU.</summary>
public sealed record SavedLoadCurve(
    Guid Id,
    string Label,
    IReadOnlyList<LoadSweepPoint> Points,
    string Source,
    DateTimeOffset CreatedAt);

public sealed record TelemetryStats(
    int SampleCount,
    double? LastValue,
    string Unit,
    uint? LastTargetTickMs,
    DateTimeOffset? LastHostTimestamp,
    double EffectiveRateHz,
    uint DroppedSamples)
{
    public static TelemetryStats Empty(string unit) => new(0, null, unit, null, null, 0, 0);
}
