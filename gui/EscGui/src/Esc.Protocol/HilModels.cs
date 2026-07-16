namespace Esc.Protocol;

public sealed record HilInputs(
    ushort SpeedRpm,
    short LoadTorque,
    byte Flags,
    bool Enable,
    uint? RunId = null,
    uint? SourceSequence = null)
{
    public bool HasValidationProvenance => RunId.HasValue && SourceSequence.HasValue;
}

public sealed record HilOutputs(
    uint TargetTickMs,
    byte AppState,
    ControlRuntimeMode Mode,
    ushort SetpointRpm,
    ushort MeasuredRpm,
    ushort PwmCommand,
    sbyte CommutationStep,
    byte Flags,
    bool TimedOut,
    uint AcceptedRunId = 0,
    uint AcceptedSourceSequence = 0,
    uint AcceptedGeneration = 0,
    uint AppliedRunId = 0,
    uint AppliedSourceSequence = 0,
    uint OutputGeneration = 0,
    uint PwmUpdateTickMs = 0,
    bool HasValidationProvenance = false);

public sealed record HilStepRequest(
    ushort SpeedRpm,
    short LoadTorque,
    byte Flags,
    bool Enable,
    uint RunId,
    uint SourceSequence,
    ushort Steps);

public sealed record HilStepResult(
    HilOutputs Outputs,
    ushort RequestedSteps,
    ushort AppliedSteps,
    byte Flags)
{
    public bool Replayed => (Flags & 0x01) != 0;
}
