using Esc.Bridge;
using Esc.Protocol;
using Esc.Transport;
using Microsoft.Extensions.Logging;

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

        CommandResult apply = await bridge.SetConfigAsync(ConfigParam.Kp, 1.25);
        CommandResult save = await bridge.SaveConfigAsync(ConfigParam.Kp);

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

        CommandResult input = await bridge.HilSetInputsAsync(new HilInputs(1800, 0, 0, 2, true));
        HilOutputs output = await bridge.HilGetOutputsAsync();

        Assert.True(input.Success);
        Assert.Equal(ControlRuntimeMode.HilSim, output.Mode);
        Assert.Equal(1800, output.MeasuredRpm);
        Assert.Contains(CommOpcode.HilSetInputs, transport.RequestedOpcodes);
        Assert.Contains(CommOpcode.HilGetOutputs, transport.RequestedOpcodes);
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

            byte[] payload = request.Opcode switch
            {
                CommOpcode.Ping => request.Payload,
                CommOpcode.GetStatus => StatusPayload(),
                CommOpcode.HilGetOutputs => HilOutputsPayload(),
                _ => []
            };

            byte[] response = EscProtocol.BuildFrame(
                request.Sequence,
                request.Opcode,
                request.Parameter,
                payload,
                CommFrameType.Response,
                CommStatus.Ok);

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
            BitConverter.GetBytes((ushort)999).CopyTo(payload, 8);
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
