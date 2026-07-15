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
    }

    [Fact]
    public async Task SimulinkModeBlocksGuiControlButAllowsEmergencyStop()
    {
        FakeEscTransport transport = new();
        var bridge = CreateBridge(transport);
        await bridge.ConnectAsync();
        await bridge.SetModeAsync(ControlMode.SimulinkControl);

        CommandResult run = await bridge.RunAsync();
        CommandResult estop = await bridge.EmergencyStopAsync();

        Assert.False(run.Success);
        Assert.True(estop.Success);
        Assert.DoesNotContain(CommOpcode.Run, transport.RequestedOpcodes);
        Assert.Contains(CommOpcode.EmergencyStop, transport.RequestedOpcodes);
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
            var manifest = new ValidationManifest(1, "Temporary gains", "restore test", "2026-01-01T00:00:00Z",
                0.02, 20_000, 1000, new ValidationReferenceConfig(0.01, 0.01, 0, 18_000, 2, 2_000, 0.002));
            var vector = new ImportedValidationVector(44, manifest,
            [
                new ValidationInputSample(1, 0, 900, false, 1000, 0),
                new ValidationInputSample(2, 0.04, 950, false, 1000, 0)
            ], artifact);
            var store = new ValidationRunStore(root);
            ValidationRunSummary run = await store.CreateAsync(vector, new ValidationRunOptions());
            var service = new ValidationRunService(new MatValidationImporter(), store, bridge);

            await service.ExecuteAsync(run.Id);

            ValidationRunDetail detail = (await store.GetAsync(run.Id))!;
            Assert.Equal(ValidationRunStatus.Completed, detail.Summary.Status);
            Assert.Equal(ValidationSampleStatus.Passed, detail.Samples[0].Status);
            Assert.Equal(ValidationSampleStatus.Skipped, detail.Samples[1].Status);
            Assert.Equal(0.28, transport.ConfigValues[ConfigParam.KpRpm]);
            Assert.Equal(1.00, transport.ConfigValues[ConfigParam.KiRpm]);
            Assert.Equal(0, transport.ConfigValues[ConfigParam.KdRpm]);
            Assert.DoesNotContain(CommOpcode.SaveConfig, transport.RequestedOpcodes);
            Assert.DoesNotContain(
                transport.Requests,
                request => request.Opcode == CommOpcode.SetConfig && request.Parameter == (byte)ConfigParam.PwmFreq);
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
        public Dictionary<ConfigParam, double> ConfigValues { get; } = new()
        {
            [ConfigParam.KpRpm] = 0.28,
            [ConfigParam.KiRpm] = 1.00,
            [ConfigParam.KdRpm] = 0,
            [ConfigParam.PolePairs] = 2,
            [ConfigParam.PwmFreq] = 18_000
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
            RequestedOpcodes.Add(request.Opcode);
            RequestPayloads.Add(request.Payload);
            Requests.Add((request.Opcode, request.Parameter));

            if (request.Opcode == CommOpcode.SetConfig)
            {
                ConfigParam parameter = (ConfigParam)request.Parameter;
                ConfigValues[parameter] = DecodeConfigPayload(parameter, request.Payload);
            }

            byte[] payload = request.Opcode switch
            {
                CommOpcode.Ping => request.Payload,
                CommOpcode.GetStatus => StatusPayload(),
                CommOpcode.GetConfig => ConfigPayload((ConfigParam)request.Parameter),
                CommOpcode.GetValidationReference => ValidationReferencePayload(),
                CommOpcode.HilGetOutputs => HilOutputsPayload(),
                _ => []
            };

            byte[] response = EscProtocol.BuildFrame(
                request.Sequence,
                request.Opcode,
                request.Parameter,
                payload,
                CommFrameType.Response,
                ResponseStatuses.GetValueOrDefault(request.Opcode, CommStatus.Ok));

            _readQueue.Enqueue(EscProtocol.Parse(response));
            return Task.CompletedTask;
        }

        public Task<EscFrame?> ReadFrameAsync(TimeSpan timeout, CancellationToken cancellationToken = default)
        {
            return Task.FromResult(_readQueue.Count > 0 ? _readQueue.Dequeue() : null);
        }

        public void EnqueueTelemetry(int rpm, uint tickMs)
        {
            byte[] payload = new byte[9];
            payload[0] = (byte)LogParam.Speed;
            BitConverter.GetBytes(rpm).CopyTo(payload, 1);
            BitConverter.GetBytes(tickMs).CopyTo(payload, 5);
            byte[] raw = EscProtocol.BuildFrame(100, CommOpcode.TelemetryEvent, (byte)LogParam.Speed, payload, CommFrameType.Event, CommStatus.Ok);
            _readQueue.Enqueue(EscProtocol.Parse(raw));
        }

        public ValueTask DisposeAsync()
        {
            return ValueTask.CompletedTask;
        }

        private static byte[] StatusPayload()
        {
            byte[] payload = new byte[10];
            payload[0] = 0;
            payload[1] = 1;
            BitConverter.GetBytes((ushort)1000).CopyTo(payload, 4);
            BitConverter.GetBytes((ushort)980).CopyTo(payload, 6);
            BitConverter.GetBytes((ushort)2000).CopyTo(payload, 8);
            return payload;
        }

        private static byte[] ValidationReferencePayload()
        {
            byte[] payload = new byte[20];
            payload[0] = 1;
            payload[1] = 2;
            BitConverter.GetBytes((ushort)18_000).CopyTo(payload, 2);
            BitConverter.GetBytes((ushort)2_000).CopyTo(payload, 4);
            BitConverter.GetBytes(180_000u).CopyTo(payload, 6);
            BitConverter.GetBytes((ushort)14_000).CopyTo(payload, 10);
            BitConverter.GetBytes((ushort)200).CopyTo(payload, 12);
            BitConverter.GetBytes(2_000u).CopyTo(payload, 14);
            BitConverter.GetBytes((ushort)100).CopyTo(payload, 18);
            return payload;
        }

        private static byte[] HilOutputsPayload()
        {
            byte[] payload = new byte[15];
            BitConverter.GetBytes(1000u).CopyTo(payload, 0);
            payload[4] = 6;
            payload[5] = (byte)ControlRuntimeMode.HilSim;
            BitConverter.GetBytes((ushort)1000).CopyTo(payload, 6);
            BitConverter.GetBytes((ushort)1800).CopyTo(payload, 8);
            BitConverter.GetBytes((ushort)700).CopyTo(payload, 10);
            payload[12] = 1;
            return payload;
        }

        private byte[] ConfigPayload(ConfigParam parameter)
        {
            double value = ConfigValues[parameter];
            return parameter switch
            {
                ConfigParam.KpRpm or ConfigParam.KiRpm or ConfigParam.KdRpm => BitConverter.GetBytes((short)Math.Round(value * 100)),
                ConfigParam.PolePairs => [(byte)value],
                _ => BitConverter.GetBytes((ushort)value)
            };
        }

        private static double DecodeConfigPayload(ConfigParam parameter, byte[] payload)
        {
            return parameter switch
            {
                ConfigParam.KpRpm or ConfigParam.KiRpm or ConfigParam.KdRpm => BitConverter.ToInt16(payload) / 100d,
                ConfigParam.PolePairs => payload[0],
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
