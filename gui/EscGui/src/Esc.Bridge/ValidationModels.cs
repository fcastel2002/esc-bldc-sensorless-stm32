namespace Esc.Bridge;

public enum ValidationRunStatus
{
    Ready,
    Running,
    Completed,
    CompletedWithWarnings,
    Failed,
    Aborted,
}

public enum ValidationSampleStatus
{
    Pending,
    Passed,
    OutOfTolerance,
    Timeout,
    MismatchedOutput,
    TransportError,
}

public sealed record ValidationReferenceConfig(
    double Kp,
    double Ki,
    double Kd,
    int PwmFrequency,
    int PolePairs,
    int PwmArr,
    double Dt);

public sealed record ValidationManifest(
    uint SchemaVersion,
    string ExperimentName,
    string Description,
    string CreatedAtUtc,
    ulong SamplePeriodUs,
    ushort TargetRpm,
    ValidationReferenceConfig ReferenceConfig);

public sealed record ValidationInputSample(
    uint SourceSequence,
    double SimulationTimeSeconds,
    ushort SpeedRpm,
    bool Enable,
    ushort TargetRpm,
    ushort ExpectedPwm);

public sealed record ImportedValidationVector(
    uint SourceRunId,
    ValidationManifest Manifest,
    IReadOnlyList<ValidationInputSample> Samples,
    string SourcePath);

public sealed record ValidationRunOptions(
    ushort AbsoluteToleranceCounts = 0,
    int ResponseDeadlineMs = 100,
    int WarmupSamples = 0,
    int MaximumTimeouts = int.MaxValue,
    ushort HilInputTimeoutMs = 500);

public sealed record ValidationRunSummary(
    Guid Id,
    uint SourceRunId,
    string ExperimentName,
    string Description,
    ValidationRunStatus Status,
    DateTimeOffset CreatedAt,
    DateTimeOffset? StartedAt,
    DateTimeOffset? CompletedAt,
    int SampleCount,
    int PassedCount,
    int OutOfToleranceCount,
    int TimeoutCount,
    int MismatchedOutputCount,
    ushort AbsoluteToleranceCounts,
    int ResponseDeadlineMs,
    int WarmupSamples,
    int MaximumTimeouts,
    ushort HilInputTimeoutMs,
    string? FailureReason);

public sealed record ValidationSampleResult(
    uint SourceSequence,
    double SimulationTimeSeconds,
    ushort SpeedRpm,
    bool Enable,
    ushort TargetRpm,
    ushort ExpectedPwm,
    ushort? ActualPwm,
    ValidationSampleStatus Status,
    double? RoundTripMs,
    uint? McuTickMs,
    uint? OutputGeneration,
    uint? AppliedRunId,
    uint? AppliedSourceSequence,
    int? AbsoluteError);

public sealed record ValidationRunDetail(
    ValidationRunSummary Summary,
    ValidationManifest Manifest,
    IReadOnlyList<ValidationSampleResult> Samples);

public sealed record ValidationRunProgress(
    Guid RunId,
    int CompletedSamples,
    int TotalSamples,
    ValidationRunStatus Status,
    string? LastError);
