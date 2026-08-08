using Esc.Bridge;
using Esc.Protocol;
using Esc.Transport;
using Microsoft.Extensions.Logging;
using System.Net;

namespace Esc.Tests;

public sealed class BridgeTests
{
    [Fact]
    public async Task ConnectValidatesDeviceWithPingAndStatus()
    {
        var bridge = CreateBridge();

        CommandResult result = await bridge.ConnectAsync();

        Assert.True(result.Success);
        Assert.Equal(DeviceConnectionState.Connected, bridge.Snapshot.State);
        Assert.Equal("USB", bridge.Snapshot.Status?.Transport);
        Assert.Equal(new ActiveSpeedLimits(200, 5400), bridge.Snapshot.ActiveSpeedLimits);
        Assert.Equal(3.0, bridge.Snapshot.StartupConfiguration?.DurationSeconds);
    }

    [Fact]
    public async Task EmergencyStopIsAllowedInEveryControlModeWithoutStatusRefresh()
    {
        foreach (ControlMode mode in Enum.GetValues<ControlMode>())
        {
            FakeEscTransport transport = new();
            var bridge = CreateBridge(transport);
            await bridge.ConnectAsync();
            await bridge.SetModeAsync(mode);
            int statusRequestsBeforeEstop = transport.RequestedOpcodes.Count(opcode => opcode == CommOpcode.GetStatus);

            CommandResult estop = await bridge.EmergencyStopAsync();

            Assert.True(estop.Success);
            Assert.Contains(CommOpcode.EmergencyStop, transport.RequestedOpcodes);
            Assert.Equal(
                statusRequestsBeforeEstop,
                transport.RequestedOpcodes.Count(opcode => opcode == CommOpcode.GetStatus));
        }
    }

    [Fact]
    public async Task SimulinkModeStillBlocksGuiRun()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.SetModeAsync(ControlMode.SimulinkControl);

        CommandResult run = await bridge.RunAsync();

        Assert.False(run.Success);
        Assert.DoesNotContain(CommOpcode.Run, transport.RequestedOpcodes);
    }

    [Fact]
    public async Task RunReportsInvalidEscState()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        transport.ResponseStatuses[CommOpcode.Run] = CommStatus.InvalidState;

        CommandResult result = await bridge.RunAsync();

        Assert.False(result.Success);
        Assert.Equal(CommStatus.InvalidState, result.DeviceStatus);
        Assert.Equal("RUN no permitido en el estado actual del ESC.", result.Message);
    }

    [Fact]
    public async Task SimulinkModeAllowsExternalSetpointWhileBlockingGuiSetpoint()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.SetModeAsync(ControlMode.SimulinkControl);

        CommandResult guiSetpoint = await bridge.SetSpeedRpmAsync(1400);
        CommandResult simulinkSetpoint = await bridge.SetSpeedRpmFromSimulinkAsync(1600);

        Assert.False(guiSetpoint.Success);
        Assert.True(simulinkSetpoint.Success);
        Assert.Single(transport.RequestedOpcodes, opcode => opcode == CommOpcode.SetSpeedRpm);
    }

    [Fact]
    public async Task TelemetryEventIsBufferedInBridge()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.StartSpeedLogAsync();

        transport.EnqueueTelemetry(2200, 1000);
        await bridge.ReadTelemetryOnceAsync();

        Assert.Single(bridge.SpeedSamples);
        Assert.Equal(2200, bridge.Snapshot.SpeedTelemetry.LastValue);
    }

    [Fact]
    public async Task CurrentLoggingIsIndependentAndBuffersExtendedSample()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();

        await bridge.StartLogAsync(LogParam.CurrentU);
        transport.EnqueueTelemetry(
            LogParam.CurrentU, 1200, 1000, 700,
            TelemetryQuality.Valid | TelemetryQuality.Calibrated, 25, 3);
        await bridge.ReadTelemetryOnceAsync();

        TelemetrySample sample = Assert.Single(bridge.CurrentUSamples);
        Assert.Equal((ushort)700, sample.AdcRawValue);
        Assert.True(sample.IsValid);
        Assert.True(bridge.Snapshot.CurrentULoggingEnabled);
        Assert.False(bridge.Snapshot.SpeedLoggingEnabled);
        Assert.Equal(1200, bridge.Snapshot.CurrentUTelemetry.LastValue);

        await bridge.StopLogAsync(LogParam.CurrentU);
        Assert.False(bridge.Snapshot.CurrentULoggingEnabled);
    }

    [Fact]
    public async Task ClearSpeedTelemetryEmptiesBufferAndNotifiesSubscribers()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.StartSpeedLogAsync();
        transport.EnqueueTelemetry(2200, 1000);
        await bridge.ReadTelemetryOnceAsync();
        var notificationCount = 0;
        bridge.SnapshotChanged += (_, _) => notificationCount++;

        bridge.ClearSpeedTelemetry();

        Assert.Empty(bridge.SpeedSamples);
        Assert.Equal(0, bridge.Snapshot.SpeedTelemetry.SampleCount);
        Assert.Equal(1, notificationCount);
        Assert.True(bridge.Snapshot.SpeedLoggingEnabled);
    }

    [Fact]
    public async Task SetConfigDoesNotPersistUntilSaveConfigIsSent()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();

        CommandResult apply = await bridge.SetConfigAsync(ConfigParam.KpRpm, 1.25);
        CommandResult save = await bridge.SaveConfigAsync(ConfigParam.KpRpm);

        Assert.True(apply.Success);
        Assert.True(save.Success);
        Assert.Contains(CommOpcode.SetConfig, transport.RequestedOpcodes);
        Assert.Contains(CommOpcode.SaveConfig, transport.RequestedOpcodes);
        Assert.True(transport.RequestedOpcodes.IndexOf(CommOpcode.SetConfig) < transport.RequestedOpcodes.IndexOf(CommOpcode.SaveConfig));
    }

    [Fact]
    public async Task GuiSetpointUsesActiveFirmwareSpeedLimits()
    {
        FakeEscTransport transport = new();
        transport.ConfigValues[ConfigParam.MinSpeed] = 750;
        transport.ConfigValues[ConfigParam.MaxSpeed] = 8200;
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();

        CommandResult below = await bridge.SetSpeedRpmAsync(749);
        CommandResult lowerBoundary = await bridge.SetSpeedRpmAsync(750);
        CommandResult above = await bridge.SetSpeedRpmAsync(8201);

        Assert.False(below.Success);
        Assert.True(lowerBoundary.Success);
        Assert.False(above.Success);
        Assert.Single(transport.RequestedOpcodes, opcode => opcode == CommOpcode.SetSpeedRpm);
    }

    [Fact]
    public async Task SpeedLimitConfigRefreshesActiveSnapshot()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();

        CommandResult result = await bridge.SetConfigAsync(ConfigParam.MaxSpeed, 6000);

        Assert.True(result.Success);
        Assert.Equal(new ActiveSpeedLimits(200, 6000), bridge.Snapshot.ActiveSpeedLimits);
    }

    [Fact]
    public async Task SineDriveUsesAtomicPayloadAndUpdatesSnapshot()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();

        CommandResult result = await bridge.SetSineDriveAsync(7.25, 42.5);

        Assert.True(result.Success);
        Assert.True(bridge.Snapshot.SineDrive.Active);
        Assert.Equal(new SineDriveSettings(7.25, 42.5), bridge.Snapshot.SineDrive.Applied);
        byte[] payload = Assert.Single(
            transport.RequestPayloads,
            candidate => candidate.Length == 6 && BitConverter.ToUInt32(candidate, 0) == 7250);
        Assert.Equal((ushort)425, BitConverter.ToUInt16(payload, 4));

        await bridge.MaintainSineDriveAsync();
        Assert.Contains(
            transport.Requests,
            request => request.Opcode == CommOpcode.SineDrive &&
                       request.Parameter == (byte)SineDriveCommand.KeepAlive);

        transport.SimulateSineWatchdogTimeout();
        await bridge.MaintainSineDriveAsync();
        Assert.Null(bridge.Snapshot.SineDrive.Requested);
        Assert.True(bridge.Snapshot.SineDrive.Active);
        await bridge.RefreshStatusAsync();
        Assert.False(bridge.Snapshot.SineDrive.Active);
    }

    [Fact]
    public async Task DynamicSineUpdateUsesReadbackWithoutStatusRoundTrip()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.SetSineDriveAsync(5, 20);
        int statusRequestsBeforeUpdate = transport.RequestedOpcodes.Count(
            opcode => opcode == CommOpcode.GetStatus);

        CommandResult result = await bridge.UpdateSineDriveAsync(6.4, 37);

        Assert.True(result.Success);
        Assert.Equal(
            statusRequestsBeforeUpdate,
            transport.RequestedOpcodes.Count(opcode => opcode == CommOpcode.GetStatus));
        Assert.Equal(new SineDriveSettings(6.4, 37), bridge.Snapshot.SineDrive.Requested);
        Assert.Equal(new SineDriveSettings(6.4, 37), bridge.Snapshot.SineDrive.Applied);
        Assert.True(bridge.Snapshot.SineDrive.Active);
        Assert.Contains(
            transport.Requests,
            request => request.Opcode == CommOpcode.SineDrive &&
                       request.Parameter == (byte)SineDriveCommand.KeepAlive);
    }

    [Fact]
    public async Task DynamicSineUpdateCannotRestartAfterFirmwareWatchdog()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.SetSineDriveAsync(5, 20);
        transport.SimulateSineWatchdogTimeout();

        CommandResult result = await bridge.UpdateSineDriveAsync(6, 30);
        await bridge.RefreshStatusAsync();

        Assert.False(result.Success);
        Assert.False(bridge.Snapshot.SineDrive.Active);
        Assert.Single(
            transport.Requests,
            request => request.Opcode == CommOpcode.SineDrive &&
                       request.Parameter == (byte)SineDriveCommand.Apply);
    }

    [Fact]
    public async Task SineDriveIsBlockedOutsideGuiModeAndStopClearsIt()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.SetModeAsync(ControlMode.SimulinkControl);

        CommandResult blocked = await bridge.SetSineDriveAsync(5, 20);
        await bridge.SetModeAsync(ControlMode.GuiControl);
        CommandResult started = await bridge.SetSineDriveAsync(5, 20);
        CommandResult stopped = await bridge.StopAsync();

        Assert.False(blocked.Success);
        Assert.True(started.Success);
        Assert.True(stopped.Success);
        Assert.False(bridge.Snapshot.SineDrive.Active);
        Assert.Single(transport.RequestedOpcodes, opcode => opcode == CommOpcode.SineDrive);
        Assert.Contains(CommOpcode.Stop, transport.RequestedOpcodes);
    }

    [Fact]
    public async Task FailedEmergencyStopCancelsRenewalAndKeepsRetryAvailable()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.SetSineDriveAsync(5, 20);
        transport.ResponseStatuses[CommOpcode.EmergencyStop] = CommStatus.InvalidState;
        int sineRequestsBeforeEstop = transport.RequestedOpcodes.Count(
            opcode => opcode == CommOpcode.SineDrive);

        CommandResult result = await bridge.EmergencyStopAsync();
        await bridge.MaintainSineDriveAsync();

        Assert.False(result.Success);
        Assert.True(bridge.Snapshot.TransportOpen);
        Assert.True(bridge.Snapshot.SineDrive.Active);
        Assert.Equal(
            sineRequestsBeforeEstop,
            transport.RequestedOpcodes.Count(opcode => opcode == CommOpcode.SineDrive));
    }

    [Fact]
    public async Task StartupConfigAppliesInRamAndRefreshesSnapshot()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();

        CommandResult result = await bridge.SetConfigAsync(ConfigParam.StartupDuration, 4.5);

        Assert.True(result.Success);
        Assert.Equal(4.5, bridge.Snapshot.StartupConfiguration?.DurationSeconds);
        Assert.Contains(
            transport.Requests,
            request => request.Opcode == CommOpcode.SetConfig &&
                       request.Parameter == (byte)ConfigParam.StartupDuration);
    }

    [Fact]
    public async Task HilInputsAndOutputsUseDedicatedOpcodes()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.SetModeAsync(ControlMode.SimulinkControl);

        CommandResult input = await bridge.HilSetInputsAsync(new HilInputs(1800, 0, 2, true));
        HilOutputs output = await bridge.HilGetOutputsAsync();

        Assert.True(input.Success);
        Assert.Equal(ControlRuntimeMode.HilSim, output.Mode);
        Assert.Equal(1800, output.MeasuredRpm);
        Assert.Contains(CommOpcode.HilSetInputs, transport.RequestedOpcodes);
        Assert.Contains(CommOpcode.HilGetOutputs, transport.RequestedOpcodes);
    }

    [Fact]
    public async Task HilStartCanConfigureValidationInputTimeout()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.SetModeAsync(ControlMode.SimulinkControl);

        CommandResult result = await bridge.HilStartAsync(500);

        Assert.True(result.Success);
        byte[] payload = Assert.Single(transport.RequestPayloads, payload => payload.Length == 2);
        Assert.Equal(500, BitConverter.ToUInt16(payload));
    }

    [Fact]
    public async Task HilStartCanSelectSteppedExecutionWithoutChangingLegacyPayloads()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.SetModeAsync(ControlMode.SimulinkControl);

        CommandResult result = await bridge.HilStartAsync(500, executionMode: HilExecutionMode.Stepped);

        Assert.True(result.Success);
        byte[] payload = Assert.Single(transport.RequestPayloads, payload => payload.Length == 3);
        Assert.Equal([0xF4, 0x01, 0x01], payload);
    }

    [Fact]
    public async Task HilStepRetriesOneTimeoutWithTheSamePayload()
    {
        FakeEscTransport transport = new() { HilStepResponsesToDrop = 1 };
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.SetModeAsync(ControlMode.SimulinkControl);
        await bridge.HilStartAsync(500, executionMode: HilExecutionMode.Stepped);
        var request = new HilStepRequest(900, 0, 0, true, 44, 1, 10);

        HilStepResult result = await bridge.HilStepAsync(request, 1);

        Assert.True(result.Replayed);
        byte[][] payloads = transport.RequestPayloads.Where(payload => payload.Length == 18).ToArray();
        Assert.Equal(2, payloads.Length);
        Assert.Equal(payloads[0], payloads[1]);
        Assert.Equal((ushort)10, result.RequestedSteps);
        Assert.Equal((ushort)10, result.AppliedSteps);
        Assert.Equal(10u, result.Outputs.OutputGeneration - result.Outputs.AcceptedGeneration);
    }

    [Fact]
    public async Task ValidatedHilInputsPropagateRunAndSourceSequence()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.SetModeAsync(ControlMode.SimulinkControl);

        CommandResult result = await bridge.HilSetInputsAsync(new HilInputs(1800, 0, 0, true, 44, 55));

        Assert.True(result.Success);
        byte[] payload = Assert.Single(transport.RequestPayloads, payload => payload.Length == 16);
        Assert.Equal(44u, BitConverter.ToUInt32(payload, 8));
        Assert.Equal(55u, BitConverter.ToUInt32(payload, 12));
    }

    [Fact]
    public async Task ValidationUsesMatHorizonAndRestoresActiveConfiguration()
    {
        string root = Path.Combine(Path.GetTempPath(), $"esc-validation-service-{Guid.NewGuid():N}");
        Directory.CreateDirectory(root);
        string artifact = Path.Combine(root, "input.mat");
        await File.WriteAllTextAsync(artifact, "validation artifact");
        try
        {
            FakeEscTransport transport = new();
            EscBridgeService bridge = CreateBridge(transport);
            await bridge.ConnectAsync();
            var manifest = new ValidationManifest(2, "Temporary gains", "restore test", "2026-01-01T00:00:00Z",
                0.02, 20_000, 1000, new ValidationReferenceConfig(0.01, 0.01, 0, 18_000, 2, 2_000, 0.002));
            var vector = new ImportedValidationVector(44, manifest,
            [
                new ValidationInputSample(1, 0, 900, true, 1000, 700)
            ], artifact);
            var store = new ValidationRunStore(root);
            ValidationRunSummary run = await store.CreateAsync(vector, new ValidationRunOptions());
            var service = new ValidationRunService(new MatValidationImporter(), store, bridge);

            await service.ExecuteAsync(run.Id);

            ValidationRunDetail detail = (await store.GetAsync(run.Id))!;
            Assert.Equal(ValidationRunStatus.Completed, detail.Summary.Status);
            Assert.Equal(ValidationSampleStatus.Passed, detail.Samples[0].Status);
            Assert.Equal(0.28, transport.ConfigValues[ConfigParam.KpRpm]);
            Assert.Equal(1.00, transport.ConfigValues[ConfigParam.KiRpm]);
            Assert.Equal(0, transport.ConfigValues[ConfigParam.KdRpm]);
            Assert.DoesNotContain(CommOpcode.SaveConfig, transport.RequestedOpcodes);
            Assert.DoesNotContain(
                transport.Requests,
                request => request.Opcode == CommOpcode.SetConfig && request.Parameter == (byte)ConfigParam.PwmFreq);
            byte[] stepPayload = Assert.Single(transport.RequestPayloads, payload => payload.Length == 18);
            Assert.Equal((ushort)10, BitConverter.ToUInt16(stepPayload, 16));
            Assert.Single(transport.RequestedOpcodes, opcode => opcode == CommOpcode.HilStep);
            Assert.DoesNotContain(CommOpcode.HilSetInputs, transport.RequestedOpcodes);
            Assert.Single(transport.RequestedOpcodes, opcode => opcode == CommOpcode.HilGetOutputs);
        }
        finally
        {
            Microsoft.Data.Sqlite.SqliteConnection.ClearAllPools();
            if (Directory.Exists(root))
            {
                Directory.Delete(root, recursive: true);
            }
        }
    }

    [Fact]
    public async Task ValidationComparesMcuPwmWithImportedExpectedPwm()
    {
        string root = Path.Combine(Path.GetTempPath(), $"esc-validation-pwm-{Guid.NewGuid():N}");
        Directory.CreateDirectory(root);
        string artifact = Path.Combine(root, "input.mat");
        await File.WriteAllTextAsync(artifact, "validation artifact");
        try
        {
            FakeEscTransport transport = new() { HilPwmCommand = 700 };
            EscBridgeService bridge = CreateBridge(transport);
            await bridge.ConnectAsync();
            var manifest = new ValidationManifest(2, "PWM comparison", "MAT expectation", "2026-01-01T00:00:00Z",
                0.02, 20_000, 1000, new ValidationReferenceConfig(0.28, 1.00, 0, 18_000, 2, 2_000, 0.002));
            var vector = new ImportedValidationVector(44, manifest,
                [new ValidationInputSample(1, 0, 900, true, 1000, 650)], artifact);
            var store = new ValidationRunStore(root);
            ValidationRunSummary run = await store.CreateAsync(vector, new ValidationRunOptions(2));
            var service = new ValidationRunService(new MatValidationImporter(), store, bridge);

            await service.ExecuteAsync(run.Id);

            ValidationRunDetail detail = (await store.GetAsync(run.Id))!;
            ValidationSampleResult sample = Assert.Single(detail.Samples);
            Assert.Equal(ValidationRunStatus.CompletedWithWarnings, detail.Summary.Status);
            Assert.Equal(ValidationSampleStatus.OutOfTolerance, sample.Status);
            Assert.Equal((ushort)650, sample.ExpectedPwm);
            Assert.Equal<ushort?>((ushort)700, sample.ActualPwm);
            Assert.Equal(50, sample.AbsoluteError);
            Assert.Single(transport.RequestedOpcodes, opcode => opcode == CommOpcode.HilStep);
        }
        finally
        {
            Microsoft.Data.Sqlite.SqliteConnection.ClearAllPools();
            if (Directory.Exists(root))
            {
                Directory.Delete(root, recursive: true);
            }
        }
    }

    [Fact]
    public async Task ValidationStopsReplayWhenHilStepGenerationDeltaMismatches()
    {
        string root = Path.Combine(Path.GetTempPath(), $"esc-validation-delta-{Guid.NewGuid():N}");
        Directory.CreateDirectory(root);
        string artifact = Path.Combine(root, "input.mat");
        await File.WriteAllTextAsync(artifact, "validation artifact");
        try
        {
            FakeEscTransport transport = new() { HilGenerationDeltaAdjustment = -1 };
            EscBridgeService bridge = CreateBridge(transport);
            await bridge.ConnectAsync();
            var manifest = new ValidationManifest(2, "Delta mismatch", "stop replay", "2026-01-01T00:00:00Z",
                0.04, 20_000, 1000, new ValidationReferenceConfig(0.28, 1.00, 0, 18_000, 2, 2_000, 0.002));
            var vector = new ImportedValidationVector(44, manifest,
            [
                new ValidationInputSample(1, 0, 900, true, 1000, 700),
                new ValidationInputSample(2, 0.02, 950, true, 1000, 700)
            ], artifact);
            var store = new ValidationRunStore(root);
            ValidationRunSummary run = await store.CreateAsync(vector, new ValidationRunOptions());
            var service = new ValidationRunService(new MatValidationImporter(), store, bridge);

            await service.ExecuteAsync(run.Id);

            ValidationRunDetail detail = (await store.GetAsync(run.Id))!;
            Assert.Equal(ValidationRunStatus.Failed, detail.Summary.Status);
            Assert.Contains("generation delta", detail.Summary.FailureReason);
            Assert.Equal(ValidationSampleStatus.MismatchedOutput, detail.Samples[0].Status);
            Assert.Equal(ValidationSampleStatus.Skipped, detail.Samples[1].Status);
            Assert.Single(transport.RequestedOpcodes, opcode => opcode == CommOpcode.HilStep);
        }
        finally
        {
            Microsoft.Data.Sqlite.SqliteConnection.ClearAllPools();
            if (Directory.Exists(root))
            {
                Directory.Delete(root, recursive: true);
            }
        }
    }

    [Fact]
    public async Task ValidationRejectsLegacyFirmwareBeforeConfigurationOrStart()
    {
        string root = Path.Combine(Path.GetTempPath(), $"esc-validation-legacy-{Guid.NewGuid():N}");
        Directory.CreateDirectory(root);
        string artifact = Path.Combine(root, "input.mat");
        await File.WriteAllTextAsync(artifact, "validation artifact");
        try
        {
            FakeEscTransport transport = new() { LegacyValidationReference = true };
            EscBridgeService bridge = CreateBridge(transport);
            await bridge.ConnectAsync();
            var manifest = new ValidationManifest(2, "Legacy firmware", "preflight", "2026-01-01T00:00:00Z",
                0.02, 20_000, 1000, new ValidationReferenceConfig(0.28, 1.00, 0, 18_000, 2, 2_000, 0.002));
            var vector = new ImportedValidationVector(44, manifest,
                [new ValidationInputSample(1, 0, 900, true, 1000, 700)], artifact);
            var store = new ValidationRunStore(root);
            ValidationRunSummary run = await store.CreateAsync(vector, new ValidationRunOptions());
            var service = new ValidationRunService(new MatValidationImporter(), store, bridge);

            InvalidOperationException exception = await Assert.ThrowsAsync<InvalidOperationException>(() => service.ExecuteAsync(run.Id));

            ValidationRunDetail detail = (await store.GetAsync(run.Id))!;
            Assert.Contains("capability HIL determinista", exception.Message);
            Assert.Equal(ValidationRunStatus.Failed, detail.Summary.Status);
            Assert.DoesNotContain(CommOpcode.HilStart, transport.RequestedOpcodes);
            Assert.DoesNotContain(CommOpcode.SetConfig, transport.RequestedOpcodes);
        }
        finally
        {
            Microsoft.Data.Sqlite.SqliteConnection.ClearAllPools();
            if (Directory.Exists(root))
            {
                Directory.Delete(root, recursive: true);
            }
        }
    }

    [Fact]
    public void ConsecutiveIdenticalHilFramesAreCollapsedWithRepeatCount()
    {
        var bridge = CreateBridge();

        bridge.RecordHilFrame("Simulink -> Bridge", "UDP", 7, "HilInputs", "7,137,1");
        bridge.RecordHilFrame("Simulink -> Bridge", "UDP", 7, "HilInputs", "7,137,1");
        bridge.RecordHilFrame("Simulink -> Bridge", "UDP", 7, "HilInputs", "7,138,1");

        IReadOnlyList<HilFrameTraceEntry> frames = bridge.Snapshot.Hil.RecentFrames;

        Assert.Equal(2, frames.Count);
        Assert.Equal((uint)1, frames[0].RepeatCount);
        Assert.Equal("7,138,1", frames[0].Payload);
        Assert.Equal((uint)2, frames[1].RepeatCount);
        Assert.Equal("7,137,1", frames[1].Payload);
    }

    [Fact]
    public void HilSnapshotCanExposeLastAcceptedUdpInputs()
    {
        var bridge = CreateBridge();

        bridge.UpdateHilStats(new HilBridgeStats(
            true,
            12,
            0,
            10,
            3,
            0.5,
            null,
            new HilInputs(245, 0, 0, true)));

        Assert.Equal((ushort)245, bridge.Snapshot.Hil.LastInputs?.SpeedRpm);
        Assert.True(bridge.Snapshot.Hil.LastInputs?.Enable);
    }

    [Fact]
    public void LowerSequenceCanTakeOwnershipFromAnotherUdpSender()
    {
        HilUdpSenderSession session = new(250);
        IPEndPoint senderA = new(IPAddress.Loopback, 1);
        IPEndPoint senderB = new(IPAddress.Loopback, 50571);

        session.Accept(senderA, 101, 0);

        Assert.True(session.CanAccept(senderB, 2, false, false, 50));

        session.Accept(senderB, 2, 50);

        Assert.False(session.CanAccept(senderA, 101, false, false, 60));
    }

    [Fact]
    public void StartCommandCanForceTakeoverFromAnotherUdpSender()
    {
        HilUdpSenderSession session = new(250);
        IPEndPoint senderA = new(IPAddress.Loopback, 1);
        IPEndPoint senderB = new(IPAddress.Loopback, 50571);

        session.Accept(senderA, 101, 0);

        Assert.True(session.WouldTakeOver(senderB, null, true, 50));
        Assert.False(session.WouldTakeOver(senderA, 102, false, 50));
    }

    [Fact]
    public void SenderLeaseExpiresAfterIdleGap()
    {
        HilUdpSenderSession session = new(250);
        IPEndPoint senderA = new(IPAddress.Loopback, 1);
        IPEndPoint senderB = new(IPAddress.Loopback, 50571);

        session.Accept(senderA, 40, 0);

        Assert.False(session.CanAccept(senderB, 41, false, false, 100));
        Assert.True(session.CanAccept(senderB, 41, false, false, 400));
    }

    private static EscBridgeService CreateBridge(FakeEscTransport? transport = null)
    {
        return new EscBridgeService(
            new FakeDeviceEnumerator(),
            transport ?? new FakeEscTransport(),
            new TelemetryStore(),
            new TestLogger<EscBridgeService>());
    }

    private sealed class FakeDeviceEnumerator : IEscDeviceEnumerator
    {
        private readonly HidDeviceDescriptor[] _devices =
        [
            new(
                "fake-hid-path",
                CommConstants.DefaultVendorId,
                CommConstants.DefaultProductId,
                "ESC BLDC fake",
                "UNCuyo",
                "ESC BLDC Custom HID",
                "TEST001",
                65,
                65)
        ];

        public Task<IReadOnlyList<HidDeviceDescriptor>> ListAsync(CancellationToken cancellationToken = default)
        {
            return Task.FromResult<IReadOnlyList<HidDeviceDescriptor>>(_devices);
        }
    }

    private sealed class FakeEscTransport : IEscTransport
    {
        private readonly Queue<EscFrame> _readQueue = new();

        public bool IsOpen { get; private set; }
        public HidDeviceDescriptor? CurrentDevice { get; private set; }
        public List<CommOpcode> RequestedOpcodes { get; } = [];
        public List<byte[]> RequestPayloads { get; } = [];
        public List<(CommOpcode Opcode, byte Parameter)> Requests { get; } = [];
        public Dictionary<CommOpcode, CommStatus> ResponseStatuses { get; } = [];
        public ushort HilPwmCommand { get; init; } = 700;
        public int HilGenerationDeltaAdjustment { get; init; }
        public int HilStepResponsesToDrop { get; set; }
        public bool LegacyValidationReference { get; init; }
        public Dictionary<ConfigParam, double> ConfigValues { get; } = new()
        {
            [ConfigParam.KpRpm] = 0.28,
            [ConfigParam.KiRpm] = 1.00,
            [ConfigParam.KdRpm] = 0,
            [ConfigParam.PolePairs] = 2,
            [ConfigParam.PwmFreq] = 18_000,
            [ConfigParam.MinSpeed] = 200,
            [ConfigParam.MaxSpeed] = 5400,
            [ConfigParam.StartupInitialAmplitude] = 20,
            [ConfigParam.StartupFinalAmplitude] = 100,
            [ConfigParam.StartupInitialFrequency] = 2.09,
            [ConfigParam.StartupFinalFrequency] = 9.28,
            [ConfigParam.StartupDuration] = 3.0
        };

        public Task OpenAsync(HidDeviceDescriptor descriptor, CancellationToken cancellationToken = default)
        {
            IsOpen = true;
            CurrentDevice = descriptor;
            return Task.CompletedTask;
        }

        public Task CloseAsync(CancellationToken cancellationToken = default)
        {
            IsOpen = false;
            CurrentDevice = null;
            return Task.CompletedTask;
        }

        public Task WriteFrameAsync(byte[] frame, CancellationToken cancellationToken = default)
        {
            EscFrame request = EscProtocol.Parse(frame);
            CommStatus responseStatus = ResponseStatuses.GetValueOrDefault(request.Opcode, CommStatus.Ok);
            RequestedOpcodes.Add(request.Opcode);
            RequestPayloads.Add(request.Payload);
            Requests.Add((request.Opcode, request.Parameter));

            if (request.Opcode == CommOpcode.SetConfig)
            {
                ConfigParam parameter = (ConfigParam)request.Parameter;
                ConfigValues[parameter] = DecodeConfigPayload(parameter, request.Payload);
            }
            if (request.Opcode == CommOpcode.HilStart)
            {
                _runtimeMode = ControlRuntimeMode.HilSim;
                _appState = 6;
            }
            else if (request.Opcode == CommOpcode.SetControlMode && request.Payload.Length == 1)
            {
                _runtimeMode = (ControlRuntimeMode)request.Payload[0];
            }
            else if (request.Opcode == CommOpcode.HilStop)
            {
                _runtimeMode = ControlRuntimeMode.Normal;
                _appState = 0;
            }
            else if (request.Opcode == CommOpcode.SineDrive)
            {
                if (request.Parameter == (byte)SineDriveCommand.KeepAlive && _appState != 10)
                {
                    responseStatus = CommStatus.InvalidState;
                }
                else
                {
                    _appState = 10;
                }
            }
            else if ((request.Opcode is CommOpcode.Stop or CommOpcode.EmergencyStop) &&
                     responseStatus == CommStatus.Ok)
            {
                _appState = 0;
            }

            if (request.Opcode == CommOpcode.HilSetInputs && request.Payload.Length >= 16)
            {
                _hilRunId = BitConverter.ToUInt32(request.Payload, 8);
                _hilSourceSequence = BitConverter.ToUInt32(request.Payload, 12);
            }
            if (request.Opcode == CommOpcode.HilSetInputs && request.Payload.Length >= 8 && request.Payload[7] != 0)
            {
                _runtimeMode = ControlRuntimeMode.HilSim;
                _appState = 6;
            }

            if (request.Opcode == CommOpcode.HilStep)
            {
                ApplyHilStep(request.Payload);
            }

            byte[] payload = request.Opcode switch
            {
                CommOpcode.Ping => request.Payload,
                CommOpcode.GetStatus => StatusPayload(),
                CommOpcode.GetConfig => ConfigPayload((ConfigParam)request.Parameter),
                CommOpcode.GetValidationReference => ValidationReferencePayload(),
                CommOpcode.HilGetOutputs => HilOutputsPayload(),
                CommOpcode.HilStep => HilStepResultPayload(),
                CommOpcode.SineDrive => request.Payload,
                _ => []
            };

            byte[] response = EscProtocol.BuildFrame(
                request.Sequence,
                request.Opcode,
                request.Parameter,
                payload,
                CommFrameType.Response,
                responseStatus);

            if (request.Opcode == CommOpcode.HilStep && HilStepResponsesToDrop > 0)
            {
                HilStepResponsesToDrop--;
            }
            else
            {
                _readQueue.Enqueue(EscProtocol.Parse(response));
            }
            return Task.CompletedTask;
        }

        public Task<EscFrame?> ReadFrameAsync(TimeSpan timeout, CancellationToken cancellationToken = default)
        {
            return Task.FromResult(_readQueue.Count > 0 ? _readQueue.Dequeue() : null);
        }

        public void EnqueueTelemetry(int rpm, uint tickMs) =>
            EnqueueTelemetry(LogParam.Speed, rpm, tickMs);

        public void EnqueueTelemetry(
            LogParam parameter,
            int value,
            uint tickMs,
            ushort rawValue = 0,
            TelemetryQuality quality = TelemetryQuality.None,
            ushort validSamples = 0,
            sbyte commutationStep = 0)
        {
            bool extended = parameter is LogParam.CurrentU or LogParam.CurrentV or LogParam.BemfPeriod;
            byte[] payload = new byte[extended ? 15 : 9];
            payload[0] = (byte)parameter;
            BitConverter.GetBytes(value).CopyTo(payload, 1);
            BitConverter.GetBytes(tickMs).CopyTo(payload, 5);
            if (extended)
            {
                BitConverter.GetBytes(rawValue).CopyTo(payload, 9);
                payload[11] = (byte)quality;
                BitConverter.GetBytes(validSamples).CopyTo(payload, 12);
                payload[14] = unchecked((byte)commutationStep);
            }
            byte[] raw = EscProtocol.BuildFrame(
                100, CommOpcode.TelemetryEvent, (byte)parameter,
                payload, CommFrameType.Event, CommStatus.Ok);
            _readQueue.Enqueue(EscProtocol.Parse(raw));
        }

        public void SimulateSineWatchdogTimeout()
        {
            _appState = 0;
        }

        public ValueTask DisposeAsync()
        {
            return ValueTask.CompletedTask;
        }

        private byte[] StatusPayload()
        {
            byte[] payload = new byte[10];
            payload[0] = _appState;
            payload[1] = 1;
            BitConverter.GetBytes((ushort)1000).CopyTo(payload, 4);
            BitConverter.GetBytes((ushort)980).CopyTo(payload, 6);
            BitConverter.GetBytes((ushort)2000).CopyTo(payload, 8);
            return payload;
        }

        private byte[] ValidationReferencePayload()
        {
            byte[] payload = new byte[LegacyValidationReference ? 20 : 24];
            payload[0] = 1;
            payload[1] = 2;
            BitConverter.GetBytes((ushort)18_000).CopyTo(payload, 2);
            BitConverter.GetBytes((ushort)2_000).CopyTo(payload, 4);
            BitConverter.GetBytes(180_000u).CopyTo(payload, 6);
            BitConverter.GetBytes((ushort)14_000).CopyTo(payload, 10);
            BitConverter.GetBytes((ushort)200).CopyTo(payload, 12);
            BitConverter.GetBytes(2_000u).CopyTo(payload, 14);
            BitConverter.GetBytes((ushort)100).CopyTo(payload, 18);
            if (!LegacyValidationReference)
            {
                payload[20] = ValidationReference.DeterministicHilCapability;
                payload[21] = 1;
                BitConverter.GetBytes((ushort)1000).CopyTo(payload, 22);
            }
            return payload;
        }

        private uint _hilRunId;
        private uint _hilSourceSequence;
        private uint _hilAcceptedGeneration;
        private uint _hilOutputGeneration;
        private ushort _hilRequestedSteps;
        private bool _hilStepReplayed;
        private ControlRuntimeMode _runtimeMode = ControlRuntimeMode.Normal;
        private byte _appState;

        private void ApplyHilStep(byte[] payload)
        {
            uint runId = BitConverter.ToUInt32(payload, 8);
            uint sourceSequence = BitConverter.ToUInt32(payload, 12);
            ushort steps = BitConverter.ToUInt16(payload, 16);
            _hilStepReplayed = runId == _hilRunId && sourceSequence == _hilSourceSequence;
            if (_hilStepReplayed)
            {
                return;
            }

            _hilRunId = runId;
            _hilSourceSequence = sourceSequence;
            _hilRequestedSteps = steps;
            _hilAcceptedGeneration = _hilOutputGeneration;
            _hilOutputGeneration = checked((uint)(_hilAcceptedGeneration + steps + HilGenerationDeltaAdjustment));
        }

        private byte[] HilOutputsPayload()
        {
            byte[] payload = new byte[43];
            BitConverter.GetBytes(1000u).CopyTo(payload, 0);
            payload[4] = _appState;
            payload[5] = (byte)_runtimeMode;
            BitConverter.GetBytes((ushort)1000).CopyTo(payload, 6);
            BitConverter.GetBytes((ushort)1800).CopyTo(payload, 8);
            BitConverter.GetBytes(HilPwmCommand).CopyTo(payload, 10);
            payload[12] = 1;
            BitConverter.GetBytes(_hilRunId).CopyTo(payload, 15);
            BitConverter.GetBytes(_hilSourceSequence).CopyTo(payload, 19);
            BitConverter.GetBytes(_hilAcceptedGeneration).CopyTo(payload, 23);
            BitConverter.GetBytes(_hilRunId).CopyTo(payload, 27);
            BitConverter.GetBytes(_hilSourceSequence).CopyTo(payload, 31);
            BitConverter.GetBytes(_hilOutputGeneration).CopyTo(payload, 35);
            BitConverter.GetBytes(_hilOutputGeneration).CopyTo(payload, 39);
            return payload;
        }

        private byte[] HilStepResultPayload()
        {
            byte[] payload = new byte[48];
            HilOutputsPayload().CopyTo(payload, 0);
            BitConverter.GetBytes(_hilRequestedSteps).CopyTo(payload, 43);
            BitConverter.GetBytes(_hilRequestedSteps).CopyTo(payload, 45);
            payload[47] = _hilStepReplayed ? (byte)1 : (byte)0;
            return payload;
        }

        private byte[] ConfigPayload(ConfigParam parameter)
        {
            double value = ConfigValues[parameter];
            return parameter switch
            {
                ConfigParam.KpRpm or ConfigParam.KiRpm or ConfigParam.KdRpm => BitConverter.GetBytes((short)Math.Round(value * 100)),
                ConfigParam.PolePairs => [(byte)value],
                ConfigParam.StartupInitialAmplitude or ConfigParam.StartupFinalAmplitude
                    => BitConverter.GetBytes((ushort)Math.Round(value * 10)),
                ConfigParam.StartupInitialFrequency or ConfigParam.StartupFinalFrequency or ConfigParam.StartupDuration
                    => BitConverter.GetBytes((uint)Math.Round(value * 1000)),
                _ => BitConverter.GetBytes((ushort)value)
            };
        }

        private static double DecodeConfigPayload(ConfigParam parameter, byte[] payload)
        {
            return parameter switch
            {
                ConfigParam.KpRpm or ConfigParam.KiRpm or ConfigParam.KdRpm => BitConverter.ToInt16(payload) / 100d,
                ConfigParam.PolePairs => payload[0],
                ConfigParam.StartupInitialAmplitude or ConfigParam.StartupFinalAmplitude
                    => BitConverter.ToUInt16(payload) / 10d,
                ConfigParam.StartupInitialFrequency or ConfigParam.StartupFinalFrequency or ConfigParam.StartupDuration
                    => BitConverter.ToUInt32(payload) / 1000d,
                _ => BitConverter.ToUInt16(payload)
            };
        }
    }

    private sealed class TestLogger<T> : ILogger<T>
    {
        public IDisposable? BeginScope<TState>(TState state)
            where TState : notnull
        {
            return null;
        }

        public bool IsEnabled(LogLevel logLevel) => false;

        public void Log<TState>(
            LogLevel logLevel,
            EventId eventId,
            TState state,
            Exception? exception,
            Func<TState, Exception?, string> formatter)
        {
        }
    }
}
