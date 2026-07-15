using System.Diagnostics;
using Esc.Protocol;

namespace Esc.Bridge;

public sealed class ValidationRunService
{
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

            double firstSimulationTime = detail.Samples[0].SimulationTimeSeconds;
            IReadOnlyList<ValidationSampleResult> executableSamples = detail.Samples
                .Where(sample => sample.SimulationTimeSeconds - firstSimulationTime <= detail.Manifest.StopTimeSeconds + 1e-9)
                .ToArray();
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
            try
            {
                controllerSnapshot = await CaptureControllerAsync(cancellationToken).ConfigureAwait(false);
                await ConfigureControllerAsync(detail.Manifest, cancellationToken).ConfigureAwait(false);
                CommandResult start = await _bridge.HilStartAsync(detail.Summary.HilInputTimeoutMs, cancellationToken).ConfigureAwait(false);
                EnsureSuccess(start, "HIL_START");

                uint previousGeneration = 0;
                var referenceState = new ReferenceControllerState();
                int timeoutCount = 0;
                bool hasWarnings = false;
                for (int index = 0; index < executableSamples.Count; index++)
                {
                    ValidationSampleResult result = await ExecuteSampleAsync(
                        detail.Summary.SourceRunId,
                        executableSamples[index],
                        previousGeneration,
                        detail.Summary,
                        detail.Manifest.ReferenceConfig,
                        referenceState,
                        cancellationToken).ConfigureAwait(false);
                    if (result.OutputGeneration is uint generation)
                    {
                        previousGeneration = Math.Max(previousGeneration, generation);
                    }

                    bool isMeasuredSample = index >= detail.Summary.WarmupSamples;
                    if (isMeasuredSample && result.Status == ValidationSampleStatus.Timeout)
                    {
                        timeoutCount++;
                    }

                    if (isMeasuredSample && result.Status is not ValidationSampleStatus.Passed)
                    {
                        hasWarnings = true;
                    }

                    await _store.UpdateSampleAsync(id, result, cancellationToken).ConfigureAwait(false);
                    Publish(id, index + 1, executableSamples.Count, ValidationRunStatus.Running, null);
                }

                ValidationRunStatus finalStatus = timeoutCount > detail.Summary.MaximumTimeouts
                    ? ValidationRunStatus.Failed
                    : hasWarnings ? ValidationRunStatus.CompletedWithWarnings : ValidationRunStatus.Completed;
                string? failureReason = finalStatus == ValidationRunStatus.Failed
                    ? $"Timeout limit exceeded: {timeoutCount} of {detail.Summary.MaximumTimeouts}."
                    : null;
                await _store.CompleteAsync(id, finalStatus, failureReason, cancellationToken).ConfigureAwait(false);
                Publish(id, executableSamples.Count, executableSamples.Count, finalStatus, null);
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
                string? cleanupFailure = await RestoreControllerAsync(controllerSnapshot).ConfigureAwait(false);
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
        await EnsureStructuralReferenceAsync(manifest.ReferenceConfig, cancellationToken).ConfigureAwait(false);
        EnsureSuccess(await _bridge.SetConfigAsync(ConfigParam.KpRpm, manifest.ReferenceConfig.Kp, cancellationToken).ConfigureAwait(false), "SET_CONFIG KP_RPM");
        EnsureSuccess(await _bridge.SetConfigAsync(ConfigParam.KiRpm, manifest.ReferenceConfig.Ki, cancellationToken).ConfigureAwait(false), "SET_CONFIG KI_RPM");
        EnsureSuccess(await _bridge.SetConfigAsync(ConfigParam.KdRpm, manifest.ReferenceConfig.Kd, cancellationToken).ConfigureAwait(false), "SET_CONFIG KD_RPM");
        EnsureSuccess(await _bridge.SetConfigAsync(ConfigParam.PolePairs, manifest.ReferenceConfig.PolePairs, cancellationToken).ConfigureAwait(false), "SET_CONFIG POLE_PAIRS");
        await VerifyCommandedConfigAsync(manifest.ReferenceConfig, cancellationToken).ConfigureAwait(false);
        EnsureSuccess(await _bridge.SetSpeedRpmAsync(manifest.TargetRpm, cancellationToken).ConfigureAwait(false), "SET_SPEED_RPM");
        await _bridge.SetModeAsync(ControlMode.SimulinkControl).ConfigureAwait(false);
    }

    private async Task EnsureStructuralReferenceAsync(ValidationReferenceConfig expected, CancellationToken cancellationToken)
    {
        ValidationReference actual = await _bridge.RefreshValidationReferenceAsync(cancellationToken).ConfigureAwait(false);
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
        await _bridge.RefreshStatusAsync(cancellationToken).ConfigureAwait(false);
        ushort setpointRpm = _bridge.Snapshot.Status?.SpeedSetpointRpm
            ?? throw new InvalidOperationException("Cannot capture the current MCU speed setpoint.");

        return new ControllerSnapshot(
            mode,
            setpointRpm,
            await ReadConfigAsync(ConfigParam.KpRpm, cancellationToken).ConfigureAwait(false),
            await ReadConfigAsync(ConfigParam.KiRpm, cancellationToken).ConfigureAwait(false),
            await ReadConfigAsync(ConfigParam.KdRpm, cancellationToken).ConfigureAwait(false),
            await ReadConfigAsync(ConfigParam.PolePairs, cancellationToken).ConfigureAwait(false));
    }

    private async Task<string?> RestoreControllerAsync(ControllerSnapshot? snapshot)
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

        await TryAsync("HIL_STOP", async () =>
            EnsureSuccess(await _bridge.HilStopAsync(CancellationToken.None).ConfigureAwait(false), "HIL_STOP")).ConfigureAwait(false);

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
        ushort SetpointRpm,
        double Kp,
        double Ki,
        double Kd,
        double PolePairs);

    private async Task<ValidationSampleResult> ExecuteSampleAsync(
        uint sourceRunId,
        ValidationSampleResult sample,
        uint previousGeneration,
        ValidationRunSummary run,
        ValidationReferenceConfig referenceConfig,
        ReferenceControllerState referenceState,
        CancellationToken cancellationToken)
    {
        if (!sample.Enable)
        {
            CommandResult disabled = await _bridge.HilSetInputsAsync(new HilInputs(sample.SpeedRpm, 0, 0, false, sourceRunId, sample.SourceSequence), cancellationToken).ConfigureAwait(false);
            referenceState.Reset();
            ValidationSampleStatus disabledStatus = disabled.Success
                ? ValidationSampleStatus.Passed
                : ValidationSampleStatus.OutOfTolerance;
            return sample with { ExpectedPwm = 0, ActualPwm = 0, Status = disabledStatus, AbsoluteError = 0 };
        }

        var stopwatch = Stopwatch.StartNew();
        CommandResult accepted = await _bridge.HilSetInputsAsync(new HilInputs(sample.SpeedRpm, 0, 0, true, sourceRunId, sample.SourceSequence), cancellationToken).ConfigureAwait(false);
        if (!accepted.Success)
        {
            stopwatch.Stop();
            return sample with { Status = ValidationSampleStatus.TransportError, RoundTripMs = stopwatch.Elapsed.TotalMilliseconds };
        }

        HilOutputs? lastOutput = null;
        while (stopwatch.ElapsedMilliseconds < run.ResponseDeadlineMs)
        {
            HilOutputs output = await _bridge.HilGetOutputsAsync(cancellationToken).ConfigureAwait(false);
            lastOutput = output;
            bool matchesInput = output.AppliedRunId == sourceRunId && output.AppliedSourceSequence == sample.SourceSequence;
            bool isNewGeneration = output.OutputGeneration > previousGeneration;
            if (matchesInput && isNewGeneration)
            {
                stopwatch.Stop();
                ushort expectedPwm = referenceState.AdvanceTo(
                    output, sample.SpeedRpm, sample.TargetRpm, run.SourceRunId, referenceConfig);
                int error = Math.Abs(output.PwmCommand - expectedPwm);
                return sample with
                {
                    ExpectedPwm = expectedPwm,
                    ActualPwm = output.PwmCommand,
                    Status = error <= run.AbsoluteToleranceCounts ? ValidationSampleStatus.Passed : ValidationSampleStatus.OutOfTolerance,
                    RoundTripMs = stopwatch.Elapsed.TotalMilliseconds,
                    McuTickMs = output.PwmUpdateTickMs,
                    OutputGeneration = output.OutputGeneration,
                    AppliedRunId = output.AppliedRunId,
                    AppliedSourceSequence = output.AppliedSourceSequence,
                    AbsoluteError = error
                };
            }

            await Task.Delay(1, cancellationToken).ConfigureAwait(false);
        }

        stopwatch.Stop();
        ushort observedExpectedPwm = sample.ExpectedPwm;
        if (lastOutput is { HasValidationProvenance: true } &&
            lastOutput.AcceptedRunId == sourceRunId &&
            lastOutput.AcceptedSourceSequence == sample.SourceSequence &&
            lastOutput.OutputGeneration > referenceState.Generation)
        {
            observedExpectedPwm = referenceState.AdvanceTo(
                lastOutput, sample.SpeedRpm, sample.TargetRpm, run.SourceRunId, referenceConfig);
        }

        return sample with
        {
            ExpectedPwm = observedExpectedPwm,
            ActualPwm = lastOutput?.PwmCommand,
            Status = lastOutput is null ? ValidationSampleStatus.Timeout : ValidationSampleStatus.MismatchedOutput,
            RoundTripMs = stopwatch.Elapsed.TotalMilliseconds,
            McuTickMs = lastOutput?.PwmUpdateTickMs,
            OutputGeneration = lastOutput?.OutputGeneration,
            AppliedRunId = lastOutput?.AppliedRunId,
            AppliedSourceSequence = lastOutput?.AppliedSourceSequence
        };
    }

    private sealed class ReferenceControllerState
    {
        private readonly RpmPiReferenceModel _controller = new();
        private ushort _activeSpeedRpm;
        private uint _generation;

        public uint Generation => _generation;

        public void Reset()
        {
            _controller.Reset();
            _activeSpeedRpm = 0;
            _generation = 0;
        }

        public ushort AdvanceTo(
            HilOutputs output,
            ushort newSpeedRpm,
            ushort targetRpm,
            uint runId,
            ValidationReferenceConfig config)
        {
            if (output.AcceptedRunId != runId || output.OutputGeneration < _generation)
            {
                throw new InvalidOperationException("Invalid HIL generation provenance for reference replay.");
            }

            ushort pwm = 0;
            for (uint generation = _generation + 1; generation <= output.OutputGeneration; generation++)
            {
                ushort speed = generation <= output.AcceptedGeneration ? _activeSpeedRpm : newSpeedRpm;
                pwm = _controller.Step(speed, targetRpm, config);
            }

            _generation = output.OutputGeneration;
            _activeSpeedRpm = newSpeedRpm;
            return pwm;
        }

    }

    private static void ValidateOptions(ValidationRunOptions options)
    {
        if (options.ResponseDeadlineMs < 1)
        {
            throw new ArgumentOutOfRangeException(nameof(options), "Response deadline must be at least 1 ms.");
        }

        if (options.WarmupSamples < 0 || options.MaximumTimeouts < 0 || options.HilInputTimeoutMs is < 10 or > 5000)
        {
            throw new ArgumentOutOfRangeException(nameof(options), "Warm-up, timeout limits, and HIL timeout are invalid.");
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
