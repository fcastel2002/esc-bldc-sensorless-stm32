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

            await _store.MarkRunningAsync(id, cancellationToken).ConfigureAwait(false);
            Publish(id, 0, detail.Samples.Count, ValidationRunStatus.Running, null);
            try
            {
                await ConfigureControllerAsync(detail.Manifest, cancellationToken).ConfigureAwait(false);
                CommandResult start = await _bridge.HilStartAsync(detail.Summary.HilInputTimeoutMs, cancellationToken).ConfigureAwait(false);
                EnsureSuccess(start, "HIL_START");

                uint previousGeneration = 0;
                int timeoutCount = 0;
                bool hasWarnings = false;
                for (int index = 0; index < detail.Samples.Count; index++)
                {
                    ValidationSampleResult result = await ExecuteSampleAsync(detail.Summary.SourceRunId, detail.Samples[index], previousGeneration, detail.Summary, cancellationToken).ConfigureAwait(false);
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
                    Publish(id, index + 1, detail.Samples.Count, ValidationRunStatus.Running, null);
                }

                ValidationRunStatus finalStatus = timeoutCount > detail.Summary.MaximumTimeouts
                    ? ValidationRunStatus.Failed
                    : hasWarnings ? ValidationRunStatus.CompletedWithWarnings : ValidationRunStatus.Completed;
                string? failureReason = finalStatus == ValidationRunStatus.Failed
                    ? $"Timeout limit exceeded: {timeoutCount} of {detail.Summary.MaximumTimeouts}."
                    : null;
                await _store.CompleteAsync(id, finalStatus, failureReason, cancellationToken).ConfigureAwait(false);
                Publish(id, detail.Samples.Count, detail.Samples.Count, finalStatus, null);
            }
            catch (OperationCanceledException)
            {
                await _store.CompleteAsync(id, ValidationRunStatus.Aborted, "Validation cancelled.", CancellationToken.None).ConfigureAwait(false);
                Publish(id, 0, detail.Samples.Count, ValidationRunStatus.Aborted, "Validation cancelled.");
                throw;
            }
            catch (Exception exception)
            {
                await _store.CompleteAsync(id, ValidationRunStatus.Failed, exception.Message, CancellationToken.None).ConfigureAwait(false);
                Publish(id, 0, detail.Samples.Count, ValidationRunStatus.Failed, exception.Message);
                throw;
            }
            finally
            {
                try
                {
                    await _bridge.HilStopAsync(CancellationToken.None).ConfigureAwait(false);
                }
                catch
                {
                    // The persisted run already contains the root failure; cleanup must not hide it.
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
        EnsureSuccess(await _bridge.SetConfigAsync(ConfigParam.Kp, manifest.ReferenceConfig.Kp, cancellationToken).ConfigureAwait(false), "SET_CONFIG KP");
        EnsureSuccess(await _bridge.SetConfigAsync(ConfigParam.Ki, manifest.ReferenceConfig.Ki, cancellationToken).ConfigureAwait(false), "SET_CONFIG KI");
        EnsureSuccess(await _bridge.SetConfigAsync(ConfigParam.Kd, manifest.ReferenceConfig.Kd, cancellationToken).ConfigureAwait(false), "SET_CONFIG KD");
        EnsureSuccess(await _bridge.SetConfigAsync(ConfigParam.PolePairs, manifest.ReferenceConfig.PolePairs, cancellationToken).ConfigureAwait(false), "SET_CONFIG POLE_PAIRS");
        EnsureSuccess(await _bridge.SetConfigAsync(ConfigParam.PwmFreq, manifest.ReferenceConfig.PwmFrequency, cancellationToken).ConfigureAwait(false), "SET_CONFIG PWM_FREQ");
        EnsureSuccess(await _bridge.SetSpeedRpmAsync(manifest.TargetRpm, cancellationToken).ConfigureAwait(false), "SET_SPEED_RPM");
        await _bridge.SetModeAsync(ControlMode.SimulinkControl).ConfigureAwait(false);
    }

    private async Task<ValidationSampleResult> ExecuteSampleAsync(uint sourceRunId, ValidationSampleResult sample, uint previousGeneration, ValidationRunSummary run, CancellationToken cancellationToken)
    {
        if (!sample.Enable)
        {
            CommandResult disabled = await _bridge.HilSetInputsAsync(new HilInputs(sample.SpeedRpm, 0, 0, false, sourceRunId, sample.SourceSequence), cancellationToken).ConfigureAwait(false);
            ValidationSampleStatus disabledStatus = disabled.Success && sample.ExpectedPwm <= run.AbsoluteToleranceCounts
                ? ValidationSampleStatus.Passed
                : ValidationSampleStatus.OutOfTolerance;
            return sample with { ActualPwm = 0, Status = disabledStatus, AbsoluteError = sample.ExpectedPwm };
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
                int error = Math.Abs(output.PwmCommand - sample.ExpectedPwm);
                return sample with
                {
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
        return sample with
        {
            ActualPwm = lastOutput?.PwmCommand,
            Status = lastOutput is null ? ValidationSampleStatus.Timeout : ValidationSampleStatus.MismatchedOutput,
            RoundTripMs = stopwatch.Elapsed.TotalMilliseconds,
            McuTickMs = lastOutput?.PwmUpdateTickMs,
            OutputGeneration = lastOutput?.OutputGeneration,
            AppliedRunId = lastOutput?.AppliedRunId,
            AppliedSourceSequence = lastOutput?.AppliedSourceSequence
        };
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
