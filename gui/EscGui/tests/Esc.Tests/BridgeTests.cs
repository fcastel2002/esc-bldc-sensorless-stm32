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
    public async Task MonitorOnlyModeBlocksGuiRun()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.SetModeAsync(ControlMode.MonitorOnly);

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
    public async Task BusCurrentLoggingIsIndependentOfSpeedLogging()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();

        await bridge.StartLogAsync(LogParam.BusCurrent);
        transport.EnqueueTelemetry(LogParam.BusCurrent, 95, 1000);
        await bridge.ReadTelemetryOnceAsync();

        Assert.Single(bridge.GetTelemetrySamples(LogParam.BusCurrent));
        Assert.True(bridge.Snapshot.BusCurrentLoggingEnabled);
        Assert.False(bridge.Snapshot.SpeedLoggingEnabled);
        Assert.Equal(95, bridge.Snapshot.BusCurrentTelemetry.LastValue);

        await bridge.StopLogAsync(LogParam.BusCurrent);
        Assert.False(bridge.Snapshot.BusCurrentLoggingEnabled);
    }

    [Fact]
    public async Task DutyTelemetryIsReportedInPercent()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();

        await bridge.StartLogAsync(LogParam.Duty);
        transport.EnqueueTelemetry(LogParam.Duty, 3250, 1000, 650, TelemetryQuality.Valid);
        await bridge.ReadTelemetryOnceAsync();

        Assert.True(bridge.Snapshot.DutyLoggingEnabled);
        Assert.Equal(32.5, bridge.Snapshot.DutyTelemetry.LastValue);
        Assert.Equal("%", bridge.Snapshot.DutyTelemetry.Unit);
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
    public async Task BemfBlankingUsesGenericConfigCommand()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();

        CommandResult result = await bridge.SetConfigAsync(ConfigParam.BemfBlankingUs, 125);

        Assert.True(result.Success);
        Assert.Equal(125, transport.ConfigValues[ConfigParam.BemfBlankingUs]);
        int requestIndex = transport.Requests.FindIndex(
            request => request.Opcode == CommOpcode.SetConfig &&
                       request.Parameter == (byte)ConfigParam.BemfBlankingUs);
        Assert.True(requestIndex >= 0);
        Assert.Equal([0x7D, 0x00], transport.RequestPayloads[requestIndex]);
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
    public async Task StepTestHoldsCommutationStepKeepsLeaseAndStopsWithStop()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        var settings = new StepTestSettings(CommutationStep.VW, 12.5);

        CommandResult result = await bridge.SetStepTestAsync(settings);

        Assert.True(result.Success);
        Assert.True(bridge.Snapshot.StepTest.Active);
        Assert.Equal("STEP_TEST", bridge.Snapshot.Status?.AppStateName);
        Assert.Equal(settings, bridge.Snapshot.StepTest.Applied);
        Assert.Contains(
            transport.RequestPayloads,
            candidate => candidate.SequenceEqual(new byte[] { 2, 125, 0 }));

        await bridge.MaintainStepTestAsync();
        Assert.Contains(
            transport.Requests,
            request => request.Opcode == CommOpcode.StepTest &&
                       request.Parameter == (byte)StepTestCommand.KeepAlive);

        CommandResult stop = await bridge.StopAsync();
        Assert.True(stop.Success);
        Assert.False(bridge.Snapshot.StepTest.Active);
        Assert.Null(bridge.Snapshot.StepTest.Requested);
    }

    [Fact]
    public async Task FreewheelDiagAutoDetectAndSoftwareZcRemapWorkEndToEnd()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();

        CommandResult start = await bridge.StartFreewheelAsync();
        Assert.True(start.Success);
        Assert.True(bridge.Snapshot.Freewheel.Active);
        Assert.Equal("FREEWHEEL", bridge.Snapshot.Status?.AppStateName);
        Assert.NotNull(bridge.Snapshot.Freewheel.Report);
        Assert.Equal(ZcChannelMap.Default, bridge.Snapshot.Freewheel.ActiveMap);

        await bridge.PollFreewheelAsync();
        Assert.Contains(
            transport.Requests,
            req => req.Opcode == CommOpcode.ZcDiag && req.Parameter == (byte)ZcDiagCommand.GetReport);

        CommandResult stop = await bridge.StopFreewheelAsync();
        Assert.True(stop.Success);
        Assert.False(bridge.Snapshot.Freewheel.Active);
        Assert.Equal("IDLE", bridge.Snapshot.Status?.AppStateName);

        CommandResult detect = await bridge.RunZcAutoDetectAsync(15);
        Assert.True(detect.Success);
        Assert.NotNull(bridge.Snapshot.Freewheel.LastAutoDetect);
        Assert.Equal(new ZcChannelMap(1, 2, 0), bridge.Snapshot.Freewheel.LastAutoDetect!.DetectedMap);

        CommandResult applyMap = await bridge.SetZcMapAsync(new ZcChannelMap(1, 2, 0));
        Assert.True(applyMap.Success);
        Assert.Equal(new ZcChannelMap(1, 2, 0), bridge.Snapshot.Freewheel.ActiveMap);
        Assert.Equal(new ZcChannelMap(1, 2, 0), await bridge.GetZcMapAsync());
    }

    [Fact]
    public async Task StepTestRejectsDutyAboveLimitWithoutSending()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();

        CommandResult result = await bridge.SetStepTestAsync(new StepTestSettings(CommutationStep.UV, 30.1));

        Assert.False(result.Success);
        Assert.DoesNotContain(transport.RequestedOpcodes, opcode => opcode == CommOpcode.StepTest);
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
        await bridge.SetModeAsync(ControlMode.MonitorOnly);

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
    public async Task ConnectReadsControllerConfigIncludingSpeedRamp()
    {
        FakeEscTransport transport = new();
        transport.ConfigValues[ConfigParam.SpeedRamp] = 600;
        var bridge = CreateBridge(transport);

        await bridge.ConnectAsync();

        ActiveControllerConfig? config = bridge.Snapshot.ActiveControllerConfig;
        Assert.NotNull(config);
        Assert.Equal(0.28, config!.Kp, 6);
        Assert.Equal((byte)2, config.PolePairs);
        Assert.Equal((ushort)18_000, config.PwmFrequencyHz);
        Assert.Equal((ushort)600, config.SpeedRampRpmPerSecond);
        Assert.DoesNotContain((CommOpcode)0x24, transport.RequestedOpcodes);
    }

    [Fact]
    public async Task SpeedRampConfigRefreshesControllerConfig()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();

        CommandResult result = await bridge.SetConfigAsync(ConfigParam.SpeedRamp, 1200);

        Assert.True(result.Success);
        Assert.Equal((ushort)1200, bridge.Snapshot.ActiveControllerConfig?.SpeedRampRpmPerSecond);
        int requestIndex = transport.Requests.FindIndex(
            request => request.Opcode == CommOpcode.SetConfig && request.Parameter == (byte)ConfigParam.SpeedRamp);
        Assert.Equal([0xB0, 0x04], transport.RequestPayloads[requestIndex]);
    }

    [Fact]
    public async Task StatusSnapshotCarriesReferenceDutyAndSweepFlag()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);

        await bridge.ConnectAsync();

        Assert.Equal((ushort)950, bridge.Snapshot.Status?.ReferenceRpm);
        Assert.Equal(31.25, bridge.Snapshot.Status?.DutyPercent);
        Assert.False(bridge.Snapshot.Status?.LoadSweepActive);
    }

    [Fact]
    public async Task LoadSweepStartSendsExactPayloadAndRefreshDrainsAllPoints()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        var request = new LoadSweepRequest(
            LoadSweepMode.Staircase, 1000, 2000, 250, 1500, 1000, LoadSweepOptions.StopAtEnd);

        CommandResult started = await bridge.StartLoadSweepAsync(request);
        transport.AddSweepPoints(5);
        LoadSweepSnapshot sweep = await bridge.RefreshLoadSweepAsync();

        Assert.True(started.Success);
        int startIndex = transport.Requests.FindIndex(
            r => r.Opcode == CommOpcode.LoadSweep && r.Parameter == (byte)LoadSweepCommand.Start);
        Assert.Equal(EscProtocol.LoadSweepStartPayload(request), transport.RequestPayloads[startIndex]);
        Assert.Equal(5, sweep.Points.Count);
        Assert.Equal(Enumerable.Range(0, 5), sweep.Points.Select(point => point.Index));
        Assert.True(sweep.Active);
        List<ushort> pointRequests = transport.Requests
            .Select((r, index) => (r, index))
            .Where(item => item.r.Opcode == CommOpcode.LoadSweep && item.r.Parameter == (byte)LoadSweepCommand.GetPoints)
            .Select(item => BitConverter.ToUInt16(transport.RequestPayloads[item.index]))
            .ToList();
        Assert.Equal([(ushort)0, (ushort)3], pointRequests);
    }

    [Fact]
    public async Task LoadSweepPollStopsFollowingOnceFinishedAndDrained()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.StartLoadSweepAsync(new LoadSweepRequest(LoadSweepMode.Ramp, 1000, 2000, 100, 0, 500));
        transport.AddSweepPoints(2);
        transport.FinishSweep();

        await bridge.PollLoadSweepAsync();
        int requestsAfterDrain = transport.RequestedOpcodes.Count;
        await bridge.PollLoadSweepAsync();

        Assert.Equal(2, bridge.Snapshot.LoadSweep.Points.Count);
        Assert.Equal(LoadSweepState.Done, bridge.Snapshot.LoadSweep.Status?.State);
        Assert.Equal(requestsAfterDrain, transport.RequestedOpcodes.Count);
    }

    [Fact]
    public async Task LoadSweepIsBlockedInMonitorOnlyModeButAbortIsAllowed()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.SetModeAsync(ControlMode.MonitorOnly);

        CommandResult start = await bridge.StartLoadSweepAsync(
            new LoadSweepRequest(LoadSweepMode.Staircase, 1000, 2000, 250, 0, 500));
        CommandResult abort = await bridge.AbortLoadSweepAsync();

        Assert.False(start.Success);
        Assert.True(abort.Success);
        Assert.DoesNotContain(transport.Requests,
            r => r.Opcode == CommOpcode.LoadSweep && r.Parameter == (byte)LoadSweepCommand.Start);
        Assert.Contains(transport.Requests,
            r => r.Opcode == CommOpcode.LoadSweep && r.Parameter == (byte)LoadSweepCommand.Abort);
    }

    [Theory]
    [InlineData(LoadSweepMode.Staircase, 100, 2000, 250, 1000)]
    [InlineData(LoadSweepMode.Staircase, 1000, 6000, 250, 1000)]
    [InlineData(LoadSweepMode.Staircase, 1000, 5000, 5, 1000)]
    [InlineData(LoadSweepMode.Staircase, 1000, 5000, 20, 1000)]
    [InlineData(LoadSweepMode.Ramp, 1000, 1000, 100, 1000)]
    [InlineData(LoadSweepMode.Ramp, 1000, 5000, 10, 1000)]
    [InlineData(LoadSweepMode.Ramp, 1000, 2000, 100, 50)]
    public async Task LoadSweepRejectsInvalidPlansWithoutSending(
        LoadSweepMode mode, int start, int end, int stepOrRate, int windowMs)
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();

        CommandResult result = await bridge.StartLoadSweepAsync(new LoadSweepRequest(
            mode, (ushort)start, (ushort)end, (ushort)stepOrRate, 0, (ushort)windowMs));

        Assert.False(result.Success);
        Assert.DoesNotContain(CommOpcode.LoadSweep, transport.RequestedOpcodes);
    }

    [Fact]
    public async Task LoadSweepStartReportsFirmwareRejection()
    {
        FakeEscTransport transport = new();
        transport.ResponseStatuses[CommOpcode.LoadSweep] = CommStatus.InvalidState;
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();

        CommandResult result = await bridge.StartLoadSweepAsync(
            new LoadSweepRequest(LoadSweepMode.Staircase, 1000, 2000, 250, 0, 500));

        Assert.False(result.Success);
        Assert.Equal(CommStatus.InvalidState, result.DeviceStatus);
    }

    [Fact]
    public async Task SavedCurvesSurviveNewSweepAndOneCanBeTare()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.StartLoadSweepAsync(new LoadSweepRequest(LoadSweepMode.Staircase, 1000, 2000, 500, 0, 500));
        transport.AddSweepPoints(3);
        await bridge.RefreshLoadSweepAsync();

        CommandResult saved = bridge.SaveCurrentLoadCurve("Aire");
        string csv = bridge.LoadSweepCsv();
        CommandResult imported = bridge.ImportLoadCurveCsv("Agua", csv);
        Guid air = bridge.Snapshot.LoadSweep.SavedCurves[0].Id;
        bridge.SetLoadCurveTare(air);
        await bridge.StartLoadSweepAsync(new LoadSweepRequest(LoadSweepMode.Staircase, 1000, 2000, 500, 0, 500));

        Assert.True(saved.Success);
        Assert.True(imported.Success);
        Assert.Equal(["Aire", "Agua"], bridge.Snapshot.LoadSweep.SavedCurves.Select(curve => curve.Label));
        Assert.All(bridge.Snapshot.LoadSweep.SavedCurves, curve => Assert.Equal(3, curve.Points.Count));
        Assert.Equal("Aire", bridge.Snapshot.LoadSweep.TareCurve?.Label);
        Assert.Empty(bridge.Snapshot.LoadSweep.Points);

        bridge.RemoveSavedLoadCurve(air);
        Assert.Null(bridge.Snapshot.LoadSweep.TareCurveId);
        Assert.Single(bridge.Snapshot.LoadSweep.SavedCurves);
        string[] lines = csv.Split('\n', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
        Assert.StartsWith("# mode=Staircase", lines[0]);
        Assert.Equal(LoadSweepCsvWriter.Header, lines[1]);
        Assert.Equal(5, lines.Length);
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
        private byte[] _zcMap = [2, 1, 0];

        public bool IsOpen { get; private set; }
        public HidDeviceDescriptor? CurrentDevice { get; private set; }
        public List<CommOpcode> RequestedOpcodes { get; } = [];
        public List<byte[]> RequestPayloads { get; } = [];
        public List<(CommOpcode Opcode, byte Parameter)> Requests { get; } = [];
        public Dictionary<CommOpcode, CommStatus> ResponseStatuses { get; } = [];
        private readonly List<byte[]> _sweepPoints = [];
        private LoadSweepState _sweepState = LoadSweepState.Idle;
        private byte[] _sweepConfig = new byte[12];
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
            [ConfigParam.StartupDuration] = 3.0,
            [ConfigParam.SpeedRamp] = 0
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
                if (parameter == ConfigParam.ZcMap && request.Payload.Length == 3)
                {
                    _zcMap = [request.Payload[0], request.Payload[1], request.Payload[2]];
                }
                else
                {
                    ConfigValues[parameter] = DecodeConfigPayload(parameter, request.Payload);
                }
            }
            if (request.Opcode == CommOpcode.LoadSweep && responseStatus == CommStatus.Ok)
            {
                if (request.Parameter == (byte)LoadSweepCommand.Start)
                {
                    _sweepPoints.Clear();
                    _sweepConfig = request.Payload;
                    _sweepState = LoadSweepState.Measuring;
                }
                else if (request.Parameter == (byte)LoadSweepCommand.Abort && _sweepState is >= LoadSweepState.MeasuringIdleCurrent and <= LoadSweepState.Measuring)
                {
                    _sweepState = LoadSweepState.Aborted;
                }
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
            else if (request.Opcode == CommOpcode.StepTest)
            {
                if (request.Parameter == (byte)StepTestCommand.KeepAlive && _appState != 11)
                {
                    responseStatus = CommStatus.InvalidState;
                }
                else
                {
                    _appState = 11;
                }
            }
            else if (request.Opcode == CommOpcode.ZcDiag)
            {
                if (request.Parameter == (byte)ZcDiagCommand.Start)
                {
                    _appState = 12;
                }
                else if (request.Parameter == (byte)ZcDiagCommand.Stop)
                {
                    _appState = 0;
                }
            }
            else if ((request.Opcode is CommOpcode.Stop or CommOpcode.EmergencyStop) &&
                     responseStatus == CommStatus.Ok)
            {
                _appState = 0;
            }

            byte[] payload = request.Opcode switch
            {
                CommOpcode.Ping => request.Payload,
                CommOpcode.GetStatus => StatusPayload(),
                CommOpcode.GetConfig => ConfigPayload((ConfigParam)request.Parameter),
                CommOpcode.LoadSweep => LoadSweepPayload((LoadSweepCommand)request.Parameter, request.Payload),
                CommOpcode.SineDrive => request.Payload,
                CommOpcode.StepTest => request.Payload,
                CommOpcode.ZcDiag => ZcDiagResponsePayload((ZcDiagCommand)request.Parameter),
                _ => []
            };

            byte[] response = EscProtocol.BuildFrame(
                request.Sequence,
                request.Opcode,
                request.Parameter,
                payload,
                CommFrameType.Response,
                responseStatus);

            _readQueue.Enqueue(EscProtocol.Parse(response));
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
            bool extended = parameter is LogParam.Duty;
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

        public void AddSweepPoints(int count)
        {
            for (int i = 0; i < count; i++)
            {
                int index = _sweepPoints.Count;
                byte[] point = new byte[16];
                BitConverter.GetBytes((ushort)(1000 + index * 250)).CopyTo(point, 0);
                BitConverter.GetBytes((ushort)(998 + index * 250)).CopyTo(point, 2);
                BitConverter.GetBytes((ushort)(2000 + index * 300)).CopyTo(point, 4);
                BitConverter.GetBytes((ushort)12_000).CopyTo(point, 6);
                BitConverter.GetBytes((short)(800 + index * 50)).CopyTo(point, 8);
                BitConverter.GetBytes((short)(3000 + index * 100)).CopyTo(point, 10);
                BitConverter.GetBytes((ushort)40).CopyTo(point, 12);
                point[14] = 9;
                _sweepPoints.Add(point);
            }
        }

        public void FinishSweep() => _sweepState = LoadSweepState.Done;

        private byte[] LoadSweepPayload(LoadSweepCommand command, byte[] request)
        {
            switch (command)
            {
                case LoadSweepCommand.GetStatus:
                {
                    byte[] payload = new byte[29];
                    payload[0] = (byte)_sweepState;
                    payload[2] = _sweepConfig[0];
                    payload[3] = _sweepConfig[11];
                    BitConverter.GetBytes((ushort)_sweepPoints.Count).CopyTo(payload, 4);
                    BitConverter.GetBytes((ushort)128).CopyTo(payload, 6);
                    Array.Copy(_sweepConfig, 1, payload, 8, 10);
                    BitConverter.GetBytes(12_500).CopyTo(payload, 20);
                    payload[28] = 1;
                    return payload;
                }
                case LoadSweepCommand.GetPoints:
                {
                    ushort first = BitConverter.ToUInt16(request);
                    int count = Math.Clamp(_sweepPoints.Count - first, 0, 3);
                    byte[] payload = new byte[3 + count * 16];
                    BitConverter.GetBytes(first).CopyTo(payload, 0);
                    payload[2] = (byte)count;
                    for (int i = 0; i < count; i++)
                    {
                        _sweepPoints[first + i].CopyTo(payload, 3 + i * 16);
                    }
                    return payload;
                }
                default:
                    return [];
            }
        }

        public ValueTask DisposeAsync()
        {
            return ValueTask.CompletedTask;
        }

        private byte[] StatusPayload()
        {
            byte[] payload = new byte[31];
            payload[0] = _appState;
            payload[1] = 1;
            BitConverter.GetBytes((ushort)1000).CopyTo(payload, 4);
            BitConverter.GetBytes((ushort)980).CopyTo(payload, 6);
            BitConverter.GetBytes((ushort)2000).CopyTo(payload, 8);
            BitConverter.GetBytes((ushort)950).CopyTo(payload, 26);
            BitConverter.GetBytes((ushort)3125).CopyTo(payload, 28);
            payload[30] = _sweepState is >= LoadSweepState.MeasuringIdleCurrent and <= LoadSweepState.Measuring ? (byte)1 : (byte)0;
            return payload;
        }

        private byte _appState;

        private byte[] ZcDiagResponsePayload(ZcDiagCommand command)
        {
            switch (command)
            {
                case ZcDiagCommand.Start:
                case ZcDiagCommand.GetReport:
                {
                    byte[] payload = new byte[32];
                    payload[0] = _appState == 12 ? (byte)1 : (byte)0;
                    payload[1] = 0b101;
                    payload[2] = 0x7E;
                    payload[3] = (byte)(ZcHealthFlags.HasMotion | ZcHealthFlags.SequenceValid | ZcHealthFlags.DirectionForward);
                    BitConverter.GetBytes((ushort)12).CopyTo(payload, 4);
                    BitConverter.GetBytes((ushort)12).CopyTo(payload, 6);
                    BitConverter.GetBytes((ushort)12).CopyTo(payload, 8);
                    BitConverter.GetBytes((ushort)35).CopyTo(payload, 10);
                    BitConverter.GetBytes((ushort)300).CopyTo(payload, 18);
                    BitConverter.GetBytes((ushort)3000).CopyTo(payload, 20);
                    payload[22] = 3;
                    payload[23] = _zcMap[0];
                    payload[24] = _zcMap[1];
                    payload[25] = _zcMap[2];
                    payload[26] = 0x80 | 0x04 | 2;
                    return payload;
                }
                case ZcDiagCommand.AutoDetect:
                    return [0, 1, 2, 0, _zcMap[0], _zcMap[1], _zcMap[2], 8, 96, 4, 5, 6, 94, 95, 5, 7];
                default:
                    return [];
            }
        }

        private byte[] ConfigPayload(ConfigParam parameter)
        {
            if (parameter == ConfigParam.ZcMap)
            {
                return [_zcMap[0], _zcMap[1], _zcMap[2]];
            }

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
