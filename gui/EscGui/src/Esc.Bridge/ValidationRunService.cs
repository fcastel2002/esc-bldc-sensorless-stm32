using System.Diagnostics;
using Esc.Protocol;

namespace Esc.Bridge;

public sealed class ValidationRunService
{
    public const int MaximumExperimentNameLength = 200;
    public const int MaximumDescriptionLength = 2000;

    private readonly MatValidationImporter _importer;
    private readonly ValidationRunStore _store;
    private readonly EscBridgeService _bridge;
    private readonly SemaphoreSlim _executionLock = new(1, 1);

    public ValidationRunService(MatValidationImporter importer, ValidationRunStore store, EscBridgeService bridge)
    {
        _importer = importer;
        _store = store;
        _bridge = bridge;
    }

    public event EventHandler<ValidationRunProgress>? ProgressChanged;

    public async Task<ValidationRunSummary> ImportAsync(string sourcePath, ValidationRunOptions options, CancellationToken cancellationToken = default)
    {
        ValidateOptions(options);
        ImportedValidationVector vector = _importer.Import(sourcePath);
        return await _store.CreateAsync(vector, options, cancellationToken).ConfigureAwait(false);
    }

    public Task<IReadOnlyList<ValidationRunSummary>> ListAsync(CancellationToken cancellationToken = default) =>
        _store.ListAsync(cancellationToken);

    public Task<ValidationRunDetail?> GetAsync(Guid id, CancellationToken cancellationToken = default) =>
        _store.GetAsync(id, cancellationToken);

    public Task<bool> UpdateMetadataAsync(Guid id, string experimentName, string? description, CancellationToken cancellationToken = default)
    {
        string normalizedName = experimentName?.Trim() ?? string.Empty;
        string normalizedDescription = description?.Trim() ?? string.Empty;
        if (normalizedName.Length == 0)
        {
            throw new ArgumentException("Experiment name cannot be empty.", nameof(experimentName));
        }
        if (normalizedName.Length > MaximumExperimentNameLength)
        {
            throw new ArgumentException($"Experiment name cannot exceed {MaximumExperimentNameLength} characters.", nameof(experimentName));
        }
        if (normalizedDescription.Length > MaximumDescriptionLength)
        {
            throw new ArgumentException($"Description cannot exceed {MaximumDescriptionLength} characters.", nameof(description));
        }

        return _store.UpdateMetadataAsync(id, normalizedName, normalizedDescription, cancellationToken);
    }

    public async Task<bool> DeleteAsync(Guid id, CancellationToken cancellationToken = default)
    {
        await _executionLock.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            return await _store.DeleteAsync(id, cancellationToken).ConfigureAwait(false);
        }
        finally
        {
            _executionLock.Release();
        }
    }

    public async Task<ValidationRunSummary> ExecuteAsync(Guid id, CancellationToken cancellationToken = default)
    {
        await _executionLock.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            ValidationRunDetail detail = await _store.GetAsync(id, cancellationToken).ConfigureAwait(false)
                ?? throw new KeyNotFoundException($"Validation run '{id}' was not found.");
            if (detail.Summary.Status is ValidationRunStatus.Running or ValidationRunStatus.Completed or ValidationRunStatus.CompletedWithWarnings)
            {
                throw new InvalidOperationException($"Validation run '{id}' is already {detail.Summary.Status}.");
            }

            ValidateOptions(new ValidationRunOptions(
                detail.Summary.AbsoluteToleranceCounts,
                detail.Summary.ResponseDeadlineMs,
                detail.Summary.WarmupSamples,
                detail.Summary.MaximumTimeouts,
                detail.Summary.HilInputTimeoutMs));
            if (detail.Manifest.SchemaVersion != 2)
            {
                throw new InvalidDataException(
                    $"Deterministic replay requires esc_validation_v2; this run uses schema v{detail.Manifest.SchemaVersion}.");
            }
            ValidateReplaySamples(detail);
            IReadOnlyList<ValidationSampleResult> executableSamples = detail.Samples;
            if (executableSamples.Count == 0)
            {
                throw new InvalidDataException("The MAT simulation horizon does not contain any executable samples.");
            }

            await _store.MarkRunningAsync(id, cancellationToken).ConfigureAwait(false);
            foreach (ValidationSampleResult skipped in detail.Samples.Skip(executableSamples.Count))
            {
                await _store.UpdateSampleAsync(id, skipped with { Status = ValidationSampleStatus.Skipped }, cancellationToken).ConfigureAwait(false);
            }
            Publish(id, 0, executableSamples.Count, ValidationRunStatus.Running, null);
            ControllerSnapshot? controllerSnapshot = null;
            bool hilStartAttempted = false;
            try
            {
                ushort controllerSteps = await PreflightDeterministicReplayAsync(detail.Manifest, cancellationToken).ConfigureAwait(false);
                controllerSnapshot = await CaptureControllerAsync(cancellationToken).ConfigureAwait(false);
                await ConfigureControllerAsync(detail.Manifest, cancellationToken).ConfigureAwait(false);
                hilStartAttempted = true;
                CommandResult start = await _bridge.HilStartAsync(
                    detail.Summary.HilInputTimeoutMs,
                    HilExecutionMode.Stepped,
                    cancellationToken).ConfigureAwait(false);
                EnsureSuccess(start, "HIL_START");

                bool hasWarnings = false;
                string? replayFailure = null;
                for (int index = 0; index < executableSamples.Count; index++)
                {
                    (ValidationSampleResult result, string? sampleFailure) = await ExecuteSampleAsync(
                        detail.Summary.SourceRunId,
                        executableSamples[index],
                        controllerSteps,
                        detail.Summary,
                        cancellationToken).ConfigureAwait(false);

                    bool isMeasuredSample = index >= detail.Summary.WarmupSamples;
                    if (isMeasuredSample && result.Status is not ValidationSampleStatus.Passed)
                    {
                        hasWarnings = true;
                    }

                    await _store.UpdateSampleAsync(id, result, cancellationToken).ConfigureAwait(false);
                    Publish(id, index + 1, executableSamples.Count, ValidationRunStatus.Running, null);
                    if (result.Status is ValidationSampleStatus.Timeout or ValidationSampleStatus.TransportError or ValidationSampleStatus.MismatchedOutput)
                    {
                        replayFailure = sampleFailure ?? $"Deterministic replay failed at source sequence {result.SourceSequence}: {result.Status}.";
                        foreach (ValidationSampleResult remaining in executableSamples.Skip(index + 1))
                        {
                            await _store.UpdateSampleAsync(
                                id, remaining with { Status = ValidationSampleStatus.Skipped }, cancellationToken).ConfigureAwait(false);
                        }
                        break;
                    }
                }

                ValidationRunStatus finalStatus = replayFailure is not null
                    ? ValidationRunStatus.Failed
                    : hasWarnings ? ValidationRunStatus.CompletedWithWarnings : ValidationRunStatus.Completed;
                string? failureReason = replayFailure;
                await _store.CompleteAsync(id, finalStatus, failureReason, cancellationToken).ConfigureAwait(false);
                Publish(id, executableSamples.Count, executableSamples.Count, finalStatus, failureReason);
            }
            catch (OperationCanceledException)
            {
                await _store.CompleteAsync(id, ValidationRunStatus.Aborted, "Validation cancelled.", CancellationToken.None).ConfigureAwait(false);
                Publish(id, 0, executableSamples.Count, ValidationRunStatus.Aborted, "Validation cancelled.");
                throw;
            }
            catch (Exception exception)
            {
                await _store.CompleteAsync(id, ValidationRunStatus.Failed, exception.Message, CancellationToken.None).ConfigureAwait(false);
                Publish(id, 0, executableSamples.Count, ValidationRunStatus.Failed, exception.Message);
                throw;
            }
            finally
            {
                string? cleanupFailure = await RestoreControllerAsync(controllerSnapshot, hilStartAttempted).ConfigureAwait(false);
                if (cleanupFailure is not null)
                {
                    await RecordCleanupFailureAsync(id, cleanupFailure).ConfigureAwait(false);
                }
            }

            return (await _store.GetAsync(id, cancellationToken).ConfigureAwait(false))!.Summary;
        }
        finally
        {
            _executionLock.Release();
        }
    }

    private async Task ConfigureControllerAsync(ValidationManifest manifest, CancellationToken cancellationToken)
    {
        await _bridge.SetModeAsync(ControlMode.GuiControl).ConfigureAwait(false);
        EnsureSuccess(await _bridge.SetConfigAsync(ConfigParam.KpRpm, manifest.ReferenceConfig.Kp, cancellationToken).ConfigureAwait(false), "SET_CONFIG KP_RPM");
        EnsureSuccess(await _bridge.SetConfigAsync(ConfigParam.KiRpm, manifest.ReferenceConfig.Ki, cancellationToken).ConfigureAwait(false), "SET_CONFIG KI_RPM");
        EnsureSuccess(await _bridge.SetConfigAsync(ConfigParam.KdRpm, manifest.ReferenceConfig.Kd, cancellationToken).ConfigureAwait(false), "SET_CONFIG KD_RPM");
        EnsureSuccess(await _bridge.SetConfigAsync(ConfigParam.PolePairs, manifest.ReferenceConfig.PolePairs, cancellationToken).ConfigureAwait(false), "SET_CONFIG POLE_PAIRS");
        await VerifyCommandedConfigAsync(manifest.ReferenceConfig, cancellationToken).ConfigureAwait(false);
        EnsureSuccess(await _bridge.SetSpeedRpmAsync(manifest.TargetRpm, cancellationToken).ConfigureAwait(false), "SET_SPEED_RPM");
        await _bridge.SetModeAsync(ControlMode.SimulinkControl).ConfigureAwait(false);
    }

    private async Task<ushort> PreflightDeterministicReplayAsync(ValidationManifest manifest, CancellationToken cancellationToken)
    {
        ValidationReference actual = await _bridge.RefreshValidationReferenceAsync(cancellationToken).ConfigureAwait(false);
        if (!actual.SupportsDeterministicHil)
        {
            throw new InvalidOperationException("El firmware conectado no anuncia la capability HIL determinista.");
        }
        if (actual.HilStepOperationVersion != 1)
        {
            throw new InvalidOperationException(
                $"Version HIL_STEP no soportada: active {actual.HilStepOperationVersion}, expected 1.");
        }
        if (actual.ControllerDtUs == 0 || manifest.SamplePeriodUs % actual.ControllerDtUs != 0)
        {
            throw new InvalidOperationException(
                $"El sample period {manifest.SamplePeriodUs} us no es divisible exactamente por controller dt {actual.ControllerDtUs} us.");
        }

        ulong stepCount = manifest.SamplePeriodUs / actual.ControllerDtUs;
        if (stepCount == 0 || stepCount > actual.MaximumHilSteps)
        {
            throw new InvalidOperationException(
                $"El replay requiere {stepCount} HIL steps por sample; el firmware admite {actual.MaximumHilSteps}.");
        }

        ValidationReferenceConfig expected = manifest.ReferenceConfig;
        var mismatches = new List<string>();
        Compare("PWM frequency", expected.PwmFrequency, actual.PwmFrequencyHz, mismatches);
        Compare("PWM ARR", expected.PwmArr, actual.PwmArrCounts, mismatches);
        Compare("speed timer", expected.SpeedTimerHz, actual.SpeedTimerHz, mismatches);
        Compare("speed minimum period", expected.SpeedMinPeriod, actual.SpeedMinPeriodTicks, mismatches);
        Compare("speed maximum period", expected.SpeedMaxPeriod, actual.SpeedMaxPeriodTicks, mismatches);
        Compare("minimum PWM", expected.MinimumPwm, actual.MinimumPwmCounts, mismatches);
        Compare("algorithm version", expected.AlgorithmVersion, actual.AlgorithmVersion, mismatches);
        if (Math.Abs(expected.Dt - actual.ControllerDtSeconds) > 1e-9)
        {
            mismatches.Add($"controller dt expected {expected.Dt:R}, active {actual.ControllerDtSeconds:R}");
        }

        if (mismatches.Count > 0)
        {
            throw new InvalidOperationException(
                $"La base estructural del ESC no coincide con el MAT: {string.Join("; ", mismatches)}. No se modifico el ESC.");
        }

        return checked((ushort)stepCount);
    }

    private static void ValidateReplaySamples(ValidationRunDetail detail)
    {
        ValidationSampleResult? disabled = detail.Samples.FirstOrDefault(sample => !sample.Enable);
        if (disabled is not null)
        {
            throw new InvalidDataException(
                $"Deterministic replay v2 requires enable=true; source sequence {disabled.SourceSequence} is disabled.");
        }

        ValidationSampleResult? wrongTarget = detail.Samples.FirstOrDefault(
            sample => sample.TargetRpm != detail.Manifest.TargetRpm);
        if (wrongTarget is not null)
        {
            throw new InvalidDataException(
                $"Target RPM at source sequence {wrongTarget.SourceSequence} is {wrongTarget.TargetRpm}; manifest target is {detail.Manifest.TargetRpm}.");
        }

        double samplePeriodSeconds = detail.Manifest.SamplePeriodUs / 1_000_000d;
        for (int index = 0; index < detail.Samples.Count; index++)
        {
            double expectedTime = index * samplePeriodSeconds;
            if (Math.Abs(detail.Samples[index].SimulationTimeSeconds - expectedTime) > 1e-9)
            {
                throw new InvalidDataException(
                    $"Stored validation grid is incompatible at source sequence {detail.Samples[index].SourceSequence}.");
            }
        }

        double expectedStopTime = detail.Samples.Count * samplePeriodSeconds;
        if (Math.Abs(detail.Manifest.StopTimeSeconds - expectedStopTime) > 1e-9)
        {
            throw new InvalidDataException(
                "Stored validation horizon does not end after the final complete sample interval.");
        }
    }

    private async Task VerifyCommandedConfigAsync(ValidationReferenceConfig expected, CancellationToken cancellationToken)
    {
        var mismatches = new List<string>();
        Compare("KP", expected.Kp, await ReadConfigAsync(ConfigParam.KpRpm, cancellationToken).ConfigureAwait(false), mismatches);
        Compare("KI", expected.Ki, await ReadConfigAsync(ConfigParam.KiRpm, cancellationToken).ConfigureAwait(false), mismatches);
        Compare("KD", expected.Kd, await ReadConfigAsync(ConfigParam.KdRpm, cancellationToken).ConfigureAwait(false), mismatches);
        Compare("pole pairs", expected.PolePairs, await ReadConfigAsync(ConfigParam.PolePairs, cancellationToken).ConfigureAwait(false), mismatches);
        if (mismatches.Count > 0)
        {
            throw new InvalidOperationException($"El MCU no aplico la configuracion comandada: {string.Join("; ", mismatches)}.");
        }
    }

    private static void Compare(string name, double expected, double actual, List<string> mismatches)
    {
        if (Math.Abs(expected - actual) > 0.005)
        {
            mismatches.Add($"{name} expected {expected}, active {actual}");
        }
    }

    private async Task<ControllerSnapshot> CaptureControllerAsync(CancellationToken cancellationToken)
    {
        ControlMode mode = _bridge.Snapshot.Mode;
        HilOutputs hilOutputs = await _bridge.HilGetOutputsAsync(cancellationToken).ConfigureAwait(false);
        if (hilOutputs.Mode != ControlRuntimeMode.Normal)
        {
            throw new InvalidOperationException(
                $"Deterministic validation requires MCU runtime mode NORMAL; active mode is {hilOutputs.Mode}.");
        }
        if (hilOutputs.AppState != 0)
        {
            throw new InvalidOperationException(
                $"Deterministic validation requires MCU app state IDLE; active state is {hilOutputs.AppState}.");
        }
        await _bridge.RefreshStatusAsync(cancellationToken).ConfigureAwait(false);
        ushort setpointRpm = _bridge.Snapshot.Status?.SpeedSetpointRpm
            ?? throw new InvalidOperationException("Cannot capture the current MCU speed setpoint.");

        return new ControllerSnapshot(
            mode,
            hilOutputs.Mode,
            setpointRpm,
            await ReadConfigAsync(ConfigParam.KpRpm, cancellationToken).ConfigureAwait(false),
            await ReadConfigAsync(ConfigParam.KiRpm, cancellationToken).ConfigureAwait(false),
            await ReadConfigAsync(ConfigParam.KdRpm, cancellationToken).ConfigureAwait(false),
            await ReadConfigAsync(ConfigParam.PolePairs, cancellationToken).ConfigureAwait(false));
    }

    private async Task<string?> RestoreControllerAsync(ControllerSnapshot? snapshot, bool hilStartAttempted)
    {
        var failures = new List<string>();

        async Task TryAsync(string operation, Func<Task> action)
        {
            try
            {
                await action().ConfigureAwait(false);
            }
            catch (Exception exception)
            {
                failures.Add($"{operation}: {exception.Message}");
            }
        }

        if (hilStartAttempted)
        {
            await TryAsync("HIL_STOP", async () =>
                EnsureSuccess(await _bridge.HilStopAsync(CancellationToken.None).ConfigureAwait(false), "HIL_STOP")).ConfigureAwait(false);
        }

        if (snapshot is null)
        {
            return failures.Count == 0 ? null : string.Join("; ", failures);
        }

        await TryAsync("mode GUI", () => _bridge.SetModeAsync(ControlMode.GuiControl)).ConfigureAwait(false);
        await RestoreConfigAsync(ConfigParam.PolePairs, snapshot.PolePairs, failures).ConfigureAwait(false);
        await RestoreConfigAsync(ConfigParam.KpRpm, snapshot.Kp, failures).ConfigureAwait(false);
        await RestoreConfigAsync(ConfigParam.KiRpm, snapshot.Ki, failures).ConfigureAwait(false);
        await RestoreConfigAsync(ConfigParam.KdRpm, snapshot.Kd, failures).ConfigureAwait(false);
        await TryAsync("restore setpoint", async () =>
            EnsureSuccess(await _bridge.SetSpeedRpmAsync(snapshot.SetpointRpm, CancellationToken.None).ConfigureAwait(false), "restore setpoint")).ConfigureAwait(false);
        if (hilStartAttempted)
        {
            await TryAsync("restore MCU runtime mode", async () =>
                EnsureSuccess(await _bridge.SetControlRuntimeModeAsync(snapshot.RuntimeMode, CancellationToken.None).ConfigureAwait(false),
                    "restore MCU runtime mode")).ConfigureAwait(false);
        }
        await TryAsync("restore mode", () => _bridge.SetModeAsync(snapshot.Mode)).ConfigureAwait(false);

        return failures.Count == 0 ? null : string.Join("; ", failures);
    }

    private async Task RestoreConfigAsync(ConfigParam parameter, double expectedValue, List<string> failures)
    {
        try
        {
            EnsureSuccess(await _bridge.SetConfigAsync(parameter, expectedValue, CancellationToken.None).ConfigureAwait(false), $"restore {parameter}");
            double actualValue = await ReadConfigAsync(parameter, CancellationToken.None).ConfigureAwait(false);
            if (Math.Abs(actualValue - expectedValue) > 0.005)
            {
                failures.Add($"verify {parameter}: expected {expectedValue}, got {actualValue}");
            }
        }
        catch (Exception exception)
        {
            failures.Add($"restore {parameter}: {exception.Message}");
        }
    }

    private async Task<double> ReadConfigAsync(ConfigParam parameter, CancellationToken cancellationToken)
    {
        object? value = await _bridge.GetConfigAsync(parameter, cancellationToken).ConfigureAwait(false);
        return value is null
            ? throw new InvalidOperationException($"GET_CONFIG {parameter} returned no value.")
            : Convert.ToDouble(value, System.Globalization.CultureInfo.InvariantCulture);
    }

    private async Task RecordCleanupFailureAsync(Guid id, string cleanupFailure)
    {
        ValidationRunDetail? detail = await _store.GetAsync(id, CancellationToken.None).ConfigureAwait(false);
        if (detail is null)
        {
            return;
        }

        ValidationRunStatus status = detail.Summary.Status is ValidationRunStatus.Completed
            ? ValidationRunStatus.CompletedWithWarnings
            : detail.Summary.Status;
        string reason = string.IsNullOrWhiteSpace(detail.Summary.FailureReason)
            ? $"Cleanup failed: {cleanupFailure}"
            : $"{detail.Summary.FailureReason} Cleanup failed: {cleanupFailure}";
        await _store.CompleteAsync(id, status, reason, CancellationToken.None).ConfigureAwait(false);
        Publish(id, 0, detail.Summary.SampleCount, status, reason);
    }

    private sealed record ControllerSnapshot(
        ControlMode Mode,
        ControlRuntimeMode RuntimeMode,
        ushort SetpointRpm,
        double Kp,
        double Ki,
        double Kd,
        double PolePairs);

    private async Task<(ValidationSampleResult Result, string? FailureReason)> ExecuteSampleAsync(
        uint sourceRunId,
        ValidationSampleResult sample,
        ushort controllerSteps,
        ValidationRunSummary run,
        CancellationToken cancellationToken)
    {
        var stopwatch = Stopwatch.StartNew();
        HilStepResult stepResult;
        try
        {
            stepResult = await _bridge.HilStepAsync(
                new HilStepRequest(sample.SpeedRpm, 0, 0, true, sourceRunId, sample.SourceSequence, controllerSteps),
                run.ResponseDeadlineMs,
                cancellationToken).ConfigureAwait(false);
        }
        catch (TimeoutException exception)
        {
            stopwatch.Stop();
            return (
                sample with { Status = ValidationSampleStatus.Timeout, RoundTripMs = stopwatch.Elapsed.TotalMilliseconds },
                $"HIL_STEP timed out at source sequence {sample.SourceSequence} after one retry: {exception.Message}");
        }
        catch (Exception exception) when (exception is not OperationCanceledException)
        {
            stopwatch.Stop();
            return (
                sample with { Status = ValidationSampleStatus.TransportError, RoundTripMs = stopwatch.Elapsed.TotalMilliseconds },
                $"HIL_STEP transport/protocol error at source sequence {sample.SourceSequence}: {exception.Message}");
        }

        stopwatch.Stop();
        HilOutputs output = stepResult.Outputs;
        int error = Math.Abs(output.PwmCommand - sample.ExpectedPwm);
        ValidationSampleResult measured = sample with
        {
            ActualPwm = output.PwmCommand,
            RoundTripMs = stopwatch.Elapsed.TotalMilliseconds,
            McuTickMs = output.PwmUpdateTickMs,
            OutputGeneration = output.OutputGeneration,
            AppliedRunId = output.AppliedRunId,
            AppliedSourceSequence = output.AppliedSourceSequence,
            AbsoluteError = error
        };

        var mismatches = new List<string>();
        if (stepResult.RequestedSteps != controllerSteps)
        {
            mismatches.Add($"requested steps {stepResult.RequestedSteps}, expected {controllerSteps}");
        }
        if (stepResult.AppliedSteps != controllerSteps)
        {
            mismatches.Add($"applied steps {stepResult.AppliedSteps}, expected {controllerSteps}");
        }
        if (!output.HasValidationProvenance)
        {
            mismatches.Add("validation provenance is missing");
        }
        if (output.AcceptedRunId != sourceRunId || output.AcceptedSourceSequence != sample.SourceSequence)
        {
            mismatches.Add($"accepted provenance {output.AcceptedRunId}/{output.AcceptedSourceSequence}");
        }
        if (output.AppliedRunId != sourceRunId || output.AppliedSourceSequence != sample.SourceSequence)
        {
            mismatches.Add($"applied provenance {output.AppliedRunId}/{output.AppliedSourceSequence}");
        }
        if (output.AcceptedGeneration > uint.MaxValue - controllerSteps ||
            output.OutputGeneration != output.AcceptedGeneration + controllerSteps)
        {
            mismatches.Add(
                $"generation delta from {output.AcceptedGeneration} to {output.OutputGeneration}, expected {controllerSteps} without overflow");
        }

        if (mismatches.Count > 0)
        {
            return (
                measured with { Status = ValidationSampleStatus.MismatchedOutput },
                $"HIL_STEP mismatch at source sequence {sample.SourceSequence}: {string.Join("; ", mismatches)}.");
        }

        return (
            measured with
            {
                Status = error <= run.AbsoluteToleranceCounts
                    ? ValidationSampleStatus.Passed
                    : ValidationSampleStatus.OutOfTolerance
            },
            null);
    }

    private static void ValidateOptions(ValidationRunOptions options)
    {
        if (options.ResponseDeadlineMs is < 1 or > 2474)
        {
            throw new ArgumentOutOfRangeException(nameof(options), "Response deadline must be between 1 and 2474 ms.");
        }

        if (options.WarmupSamples < 0 || options.MaximumTimeouts != 0 || options.HilInputTimeoutMs is < 10 or > 5000)
        {
            throw new ArgumentOutOfRangeException(
                nameof(options),
                "Warm-up and HIL timeout must be valid, and deterministic replay requires MaximumTimeouts=0.");
        }
        if (options.HilInputTimeoutMs <= options.ResponseDeadlineMs * 2 + 50)
        {
            throw new ArgumentOutOfRangeException(
                nameof(options),
                "HIL input timeout must exceed two response deadlines plus a 50 ms processing margin.");
        }
    }

    private static void EnsureSuccess(CommandResult result, string operation)
    {
        if (!result.Success)
        {
            throw new InvalidOperationException($"{operation} failed: {result.Message}");
        }
    }

    private void Publish(Guid id, int completed, int total, ValidationRunStatus status, string? error) =>
        ProgressChanged?.Invoke(this, new ValidationRunProgress(id, completed, total, status, error));
}
