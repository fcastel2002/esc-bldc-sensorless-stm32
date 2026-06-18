using Esc.Protocol;
using Esc.Transport;
using Microsoft.Extensions.Logging;

namespace Esc.Bridge;

public sealed class EscBridgeService
{
    private readonly IEscDeviceEnumerator _deviceEnumerator;
    private readonly IEscTransport _transport;
    private readonly TelemetryStore _telemetryStore;
    private readonly ILogger<EscBridgeService> _logger;
    private readonly SemaphoreSlim _ioLock = new(1, 1);
    private readonly object _stateGate = new();

    private IReadOnlyList<HidDeviceDescriptor> _devices = Array.Empty<HidDeviceDescriptor>();
    private HidDeviceDescriptor? _currentDevice;
    private EscStatus? _status;
    private DeviceConnectionState _state = DeviceConnectionState.NotDetected;
    private ControlMode _mode = ControlMode.GuiControl;
    private string? _lastError;
    private bool _speedLoggingEnabled;
    private ushort _logRateMs = 500;
    private byte _sequence;

    public EscBridgeService(
        IEscDeviceEnumerator deviceEnumerator,
        IEscTransport transport,
        TelemetryStore telemetryStore,
        ILogger<EscBridgeService> logger)
    {
        _deviceEnumerator = deviceEnumerator;
        _transport = transport;
        _telemetryStore = telemetryStore;
        _logger = logger;
    }

    public event EventHandler? SnapshotChanged;

    public BridgeSnapshot Snapshot
    {
        get
        {
            lock (_stateGate)
            {
                return BuildSnapshot();
            }
        }
    }

    public IReadOnlyList<TelemetrySample> SpeedSamples => _telemetryStore.GetSamples("speed");

    public async Task ScanAsync(CancellationToken cancellationToken = default)
    {
        IReadOnlyList<HidDeviceDescriptor> devices = await _deviceEnumerator.ListAsync(cancellationToken).ConfigureAwait(false);
        bool currentStillPresent = _currentDevice is not null &&
            devices.Any(device => string.Equals(device.DevicePath, _currentDevice.DevicePath, StringComparison.Ordinal));

        lock (_stateGate)
        {
            _devices = devices;

            if (_transport.IsOpen && !currentStillPresent)
            {
                _state = DeviceConnectionState.Disconnected;
                _lastError = "HID device disconnected.";
                _status = null;
                _currentDevice = null;
            }
            else if (!_transport.IsOpen && devices.Count > 0 && _state is DeviceConnectionState.NotDetected or DeviceConnectionState.Disconnected)
            {
                _state = DeviceConnectionState.Detected;
                _lastError = null;
            }
            else if (!_transport.IsOpen && devices.Count == 0)
            {
                _state = DeviceConnectionState.NotDetected;
                _lastError = null;
                _currentDevice = null;
                _status = null;
            }
        }

        if (_transport.IsOpen && !currentStillPresent)
        {
            await _transport.CloseAsync(cancellationToken).ConfigureAwait(false);
        }

        Notify();
    }

    public async Task<CommandResult> ConnectAsync(string? devicePath = null, CancellationToken cancellationToken = default)
    {
        await ScanAsync(cancellationToken).ConfigureAwait(false);

        HidDeviceDescriptor? descriptor = _devices.FirstOrDefault(device =>
            string.IsNullOrWhiteSpace(devicePath) ||
            string.Equals(device.DevicePath, devicePath, StringComparison.Ordinal));

        if (descriptor is null)
        {
            SetError(DeviceConnectionState.NotDetected, "No ESC HID device was detected.");
            return CommandResult.Failed("No ESC HID device was detected.");
        }

        SetState(DeviceConnectionState.Connecting, descriptor, null);

        try
        {
            await _transport.OpenAsync(descriptor, cancellationToken).ConfigureAwait(false);
            _currentDevice = descriptor;

            EscFrame ping = await SendRequestCoreAsync(CommOpcode.Ping, 0, "ping"u8.ToArray(), cancellationToken).ConfigureAwait(false);
            EscProtocol.EnsureOkResponse(ping, CommOpcode.Ping);

            EscFrame statusFrame = await SendRequestCoreAsync(CommOpcode.GetStatus, 0, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
            EscStatus status = EscProtocol.DecodeStatus(statusFrame);

            lock (_stateGate)
            {
                _state = DeviceConnectionState.Connected;
                _currentDevice = descriptor;
                _status = status;
                _lastError = null;
            }

            Notify();
            return CommandResult.Ok("Connected.");
        }
        catch (Exception ex)
        {
            _logger.LogWarning(ex, "Failed to connect to ESC HID device.");
            await _transport.CloseAsync(cancellationToken).ConfigureAwait(false);
            SetError(DeviceConnectionState.Error, ex.Message);
            return CommandResult.Failed(ex.Message);
        }
    }

    public async Task DisconnectAsync(CancellationToken cancellationToken = default)
    {
        await _transport.CloseAsync(cancellationToken).ConfigureAwait(false);
        _speedLoggingEnabled = false;
        lock (_stateGate)
        {
            _state = _devices.Count > 0 ? DeviceConnectionState.Detected : DeviceConnectionState.NotDetected;
            _currentDevice = null;
            _status = null;
            _lastError = null;
        }

        Notify();
    }

    public Task SetModeAsync(ControlMode mode)
    {
        lock (_stateGate)
        {
            _mode = mode;
        }

        Notify();
        return Task.CompletedTask;
    }

    public Task<CommandResult> RunAsync(CancellationToken cancellationToken = default)
    {
        return SendControlCommandAsync(CommOpcode.Run, cancellationToken);
    }

    public Task<CommandResult> StopAsync(CancellationToken cancellationToken = default)
    {
        return SendControlCommandAsync(CommOpcode.Stop, cancellationToken);
    }

    public Task<CommandResult> EmergencyStopAsync(CancellationToken cancellationToken = default)
    {
        return SendCommandAsync(CommOpcode.EmergencyStop, 0, Array.Empty<byte>(), cancellationToken);
    }

    public Task<CommandResult> SetSpeedRpmAsync(int rpm, CancellationToken cancellationToken = default)
    {
        if (!AllowsGuiControl())
        {
            return Task.FromResult(CommandResult.Failed($"Control is locked by {_mode}."));
        }

        return SendCommandAsync(CommOpcode.SetSpeedRpm, 0, EscProtocol.UInt16Payload(rpm), cancellationToken);
    }

    public async Task<object?> GetConfigAsync(ConfigParam parameter, CancellationToken cancellationToken = default)
    {
        EscFrame response = await SendRequestAsync(CommOpcode.GetConfig, (byte)parameter, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
        EscProtocol.EnsureOkResponse(response, CommOpcode.GetConfig);
        return EscProtocol.DecodeConfigValue(parameter, response.Payload);
    }

    public Task<CommandResult> SetConfigAsync(ConfigParam parameter, double value, CancellationToken cancellationToken = default)
    {
        if (!AllowsGuiControl())
        {
            return Task.FromResult(CommandResult.Failed($"Control is locked by {_mode}."));
        }

        byte[] payload = EscProtocol.ConfigPayload(parameter, value);
        return SendCommandAsync(CommOpcode.SetConfig, (byte)parameter, payload, cancellationToken);
    }

    public Task<CommandResult> ResetConfigAsync(ConfigParam parameter, CancellationToken cancellationToken = default)
    {
        if (!AllowsGuiControl())
        {
            return Task.FromResult(CommandResult.Failed($"Control is locked by {_mode}."));
        }

        return SendCommandAsync(CommOpcode.ResetConfig, (byte)parameter, Array.Empty<byte>(), cancellationToken);
    }

    public Task<CommandResult> SaveConfigAsync(ConfigParam parameter = ConfigParam.All, CancellationToken cancellationToken = default)
    {
        if (!AllowsGuiControl())
        {
            return Task.FromResult(CommandResult.Failed($"Control is locked by {_mode}."));
        }

        return SendCommandAsync(CommOpcode.SaveConfig, (byte)parameter, Array.Empty<byte>(), cancellationToken);
    }

    public async Task<CommandResult> SetLogRateAsync(ushort rateMs, CancellationToken cancellationToken = default)
    {
        CommandResult result = await SendCommandAsync(CommOpcode.LogRate, 0, EscProtocol.UInt16Payload(rateMs), cancellationToken).ConfigureAwait(false);
        if (result.Success)
        {
            _logRateMs = rateMs;
            Notify();
        }

        return result;
    }

    public async Task<CommandResult> StartSpeedLogAsync(CancellationToken cancellationToken = default)
    {
        CommandResult result = await SendCommandAsync(CommOpcode.LogStart, (byte)LogParam.Speed, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
        if (result.Success)
        {
            _speedLoggingEnabled = true;
            Notify();
        }

        return result;
    }

    public async Task<CommandResult> StopSpeedLogAsync(CancellationToken cancellationToken = default)
    {
        CommandResult result = await SendCommandAsync(CommOpcode.LogStop, (byte)LogParam.Speed, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
        if (result.Success)
        {
            _speedLoggingEnabled = false;
            Notify();
        }

        return result;
    }

    public async Task RefreshStatusAsync(CancellationToken cancellationToken = default)
    {
        if (!_transport.IsOpen)
        {
            return;
        }

        try
        {
            EscFrame frame = await SendRequestAsync(CommOpcode.GetStatus, 0, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
            EscStatus status = EscProtocol.DecodeStatus(frame);
            lock (_stateGate)
            {
                _status = status;
                _lastError = null;
                _state = DeviceConnectionState.Connected;
            }

            Notify();
        }
        catch (Exception ex)
        {
            _logger.LogDebug(ex, "Status refresh failed.");
            SetError(_transport.IsOpen ? DeviceConnectionState.Connected : DeviceConnectionState.Error, ex.Message);
        }
    }

    public async Task ReadTelemetryOnceAsync(CancellationToken cancellationToken = default)
    {
        if (!_transport.IsOpen || !_speedLoggingEnabled)
        {
            return;
        }

        if (!await _ioLock.WaitAsync(0, cancellationToken).ConfigureAwait(false))
        {
            return;
        }

        try
        {
            EscFrame? frame = await _transport.ReadFrameAsync(TimeSpan.FromMilliseconds(25), cancellationToken).ConfigureAwait(false);
            if (frame is not null)
            {
                HandleFrame(frame);
            }
        }
        catch (Exception ex)
        {
            _logger.LogDebug(ex, "Telemetry read failed.");
        }
        finally
        {
            _ioLock.Release();
        }
    }

    private async Task<CommandResult> SendControlCommandAsync(CommOpcode opcode, CancellationToken cancellationToken)
    {
        if (!AllowsGuiControl())
        {
            return CommandResult.Failed($"Control is locked by {_mode}.");
        }

        return await SendCommandAsync(opcode, 0, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
    }

    private async Task<CommandResult> SendCommandAsync(CommOpcode opcode, byte parameter, byte[] payload, CancellationToken cancellationToken)
    {
        try
        {
            EscFrame response = await SendRequestAsync(opcode, parameter, payload, cancellationToken).ConfigureAwait(false);
            CommandResult result = CommandResult.FromStatus(response.Status);
            await RefreshStatusAfterCommandAsync(cancellationToken).ConfigureAwait(false);
            return result;
        }
        catch (Exception ex)
        {
            SetError(DeviceConnectionState.Error, ex.Message);
            return CommandResult.Failed(ex.Message);
        }
    }

    private async Task RefreshStatusAfterCommandAsync(CancellationToken cancellationToken)
    {
        try
        {
            EscFrame statusFrame = await SendRequestAsync(CommOpcode.GetStatus, 0, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
            EscStatus status = EscProtocol.DecodeStatus(statusFrame);
            lock (_stateGate)
            {
                _status = status;
            }
        }
        catch (Exception ex)
        {
            _logger.LogDebug(ex, "Post-command status refresh failed.");
        }

        Notify();
    }

    private async Task<EscFrame> SendRequestAsync(CommOpcode opcode, byte parameter, byte[] payload, CancellationToken cancellationToken)
    {
        if (!_transport.IsOpen)
        {
            throw new InvalidOperationException("ESC is not connected.");
        }

        await _ioLock.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            return await SendRequestCoreAsync(opcode, parameter, payload, cancellationToken).ConfigureAwait(false);
        }
        finally
        {
            _ioLock.Release();
        }
    }

    private async Task<EscFrame> SendRequestCoreAsync(CommOpcode opcode, byte parameter, byte[] payload, CancellationToken cancellationToken)
    {
        byte sequence = unchecked(++_sequence);
        byte[] request = EscProtocol.BuildRequest(sequence, opcode, parameter, payload);
        await _transport.WriteFrameAsync(request, cancellationToken).ConfigureAwait(false);

        DateTimeOffset deadline = DateTimeOffset.UtcNow.AddMilliseconds(1_000);
        while (DateTimeOffset.UtcNow < deadline)
        {
            TimeSpan remaining = deadline - DateTimeOffset.UtcNow;
            EscFrame? frame = await _transport.ReadFrameAsync(remaining, cancellationToken).ConfigureAwait(false);
            if (frame is null)
            {
                continue;
            }

            if (HandleFrame(frame))
            {
                continue;
            }

            if (frame.Type == CommFrameType.Response && frame.Sequence == sequence && frame.Opcode == opcode)
            {
                return frame;
            }
        }

        throw new TimeoutException($"Timed out waiting for {opcode} response.");
    }

    private bool HandleFrame(EscFrame frame)
    {
        if (frame.Type == CommFrameType.Event && frame.Opcode == CommOpcode.TelemetryEvent)
        {
            TelemetrySample sample = EscProtocol.DecodeTelemetry(frame, DateTimeOffset.UtcNow);
            _telemetryStore.Add(sample);
            Notify();
            return true;
        }

        return false;
    }

    private bool AllowsGuiControl()
    {
        lock (_stateGate)
        {
            return _mode == ControlMode.GuiControl;
        }
    }

    private void SetState(DeviceConnectionState state, HidDeviceDescriptor? device, string? error)
    {
        lock (_stateGate)
        {
            _state = state;
            _currentDevice = device;
            _lastError = error;
        }

        Notify();
    }

    private void SetError(DeviceConnectionState state, string error)
    {
        lock (_stateGate)
        {
            _state = state;
            _lastError = error;
        }

        Notify();
    }

    private BridgeSnapshot BuildSnapshot()
    {
        return new BridgeSnapshot(
            _state,
            _mode,
            _devices,
            _currentDevice,
            _status,
            _lastError,
            DateTimeOffset.UtcNow,
            _speedLoggingEnabled,
            _logRateMs,
            _telemetryStore.GetStats("speed", "rpm"));
    }

    private void Notify()
    {
        SnapshotChanged?.Invoke(this, EventArgs.Empty);
    }
}
