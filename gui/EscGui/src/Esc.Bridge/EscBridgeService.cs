using Esc.Protocol;
using Esc.Transport;
using Microsoft.Extensions.Logging;

namespace Esc.Bridge;

public sealed class EscBridgeService
{
    private const int MaxHilTraceEntries = 80;

    private readonly IEscDeviceEnumerator _deviceEnumerator;
    private readonly IEscTransport _transport;
    private readonly TelemetryStore _telemetryStore;
    private readonly ILogger<EscBridgeService> _logger;
    private readonly SemaphoreSlim _ioLock = new(1, 1);
    private readonly object _stateGate = new();
    private readonly List<HilFrameTraceEntry> _hilFrameTrace = new();

    private IReadOnlyList<HidDeviceDescriptor> _devices = Array.Empty<HidDeviceDescriptor>();
    private HidDeviceDescriptor? _currentDevice;
    private EscStatus? _status;
    private DeviceConnectionState _state = DeviceConnectionState.NotDetected;
    private ControlMode _mode = ControlMode.GuiControl;
    private string? _lastError;
    private bool _speedLoggingEnabled;
    private ushort _logRateMs = 500;
    private byte _sequence;
    private HilBridgeStats _hilStats = new(false, 0, 0, 0, 0, 0, null, null);
    private ValidationReference? _validationReference;
    private ActiveControllerConfig? _activeControllerConfig;
    private ActiveSpeedLimits? _activeSpeedLimits;
    private DateTimeOffset? _validationReferenceCapturedAt;
    private string? _validationReferenceError;

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

    public void ClearSpeedTelemetry()
    {
        _telemetryStore.Clear("speed");
        Notify();
    }

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
                _lastError = "Dispositivo HID desconectado.";
                _status = null;
                _currentDevice = null;
                ClearValidationReference();
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
                ClearValidationReference();
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
            SetError(DeviceConnectionState.NotDetected, "No se detecto ningun dispositivo ESC HID.");
            return CommandResult.Failed("No se detecto ningun dispositivo ESC HID.");
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
            (ValidationReference validationReference, ActiveControllerConfig activeControllerConfig) =
                await ReadValidationReferenceCoreAsync(cancellationToken).ConfigureAwait(false);
            ActiveSpeedLimits activeSpeedLimits = await ReadSpeedLimitsCoreAsync(cancellationToken).ConfigureAwait(false);

            lock (_stateGate)
            {
                _state = DeviceConnectionState.Connected;
                _currentDevice = descriptor;
                _status = status;
                _lastError = null;
                _validationReference = validationReference;
                _activeControllerConfig = activeControllerConfig;
                _activeSpeedLimits = activeSpeedLimits;
                _validationReferenceCapturedAt = DateTimeOffset.UtcNow;
                _validationReferenceError = null;
            }

            Notify();
            return CommandResult.Ok("Conectado.");
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
            ClearValidationReference();
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

    public async Task<CommandResult> SetSpeedRpmAsync(int rpm, CancellationToken cancellationToken = default)
    {
        if (!AllowsGuiControl())
        {
            return CommandResult.Failed("Control bloqueado por el modo actual.");
        }

        ActiveSpeedLimits? limits;
        lock (_stateGate)
        {
            limits = _activeSpeedLimits;
        }

        if (limits is not null && (limits.MinRpm > limits.MaxRpm || rpm < limits.MinRpm || rpm > limits.MaxRpm))
        {
            return CommandResult.Failed(
                $"Setpoint fuera del rango activo del ESC: {limits.MinRpm}-{limits.MaxRpm} rpm.");
        }

        byte[] payload;
        try
        {
            payload = EscProtocol.UInt16Payload(rpm);
        }
        catch (OverflowException)
        {
            return CommandResult.Failed($"Valor de velocidad fuera de rango: {rpm} rpm.");
        }

        return await SendCommandAsync(CommOpcode.SetSpeedRpm, 0, payload, cancellationToken);
    }

    public Task<CommandResult> RunFromSimulinkAsync(CancellationToken cancellationToken = default)
    {
        return SendSimulinkControlCommandAsync(CommOpcode.Run, cancellationToken);
    }

    public Task<CommandResult> StopFromSimulinkAsync(CancellationToken cancellationToken = default)
    {
        return SendSimulinkControlCommandAsync(CommOpcode.Stop, cancellationToken);
    }

    public async Task<CommandResult> SetSpeedRpmFromSimulinkAsync(int rpm, bool refreshStatus = true, CancellationToken cancellationToken = default)
    {
        if (!AllowsSimulinkControl())
        {
            return CommandResult.Failed("Cambia el modo a Simulink control para enviar comandos de motor.");
        }

        byte[] payload;
        try
        {
            payload = EscProtocol.UInt16Payload(rpm);
        }
        catch (OverflowException)
        {
            return CommandResult.Failed($"Valor de velocidad fuera de rango: {rpm} rpm.");
        }

        CommandResult result = await SendCommandAsync(CommOpcode.SetSpeedRpm, 0, payload, cancellationToken, refreshStatus).ConfigureAwait(false);
        if (result.Success && !refreshStatus)
        {
            UpdateCachedSpeedSetpoint((ushort)rpm);
        }

        return result;
    }

    public async Task<object?> GetConfigAsync(ConfigParam parameter, CancellationToken cancellationToken = default)
    {
        EscFrame response = await SendRequestAsync(CommOpcode.GetConfig, (byte)parameter, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
        EscProtocol.EnsureOkResponse(response, CommOpcode.GetConfig);
        return EscProtocol.DecodeConfigValue(parameter, response.Payload);
    }

    public async Task<ValidationReference> RefreshValidationReferenceAsync(CancellationToken cancellationToken = default)
    {
        await _ioLock.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            (ValidationReference validationReference, ActiveControllerConfig activeControllerConfig) =
                await ReadValidationReferenceCoreAsync(cancellationToken).ConfigureAwait(false);
            lock (_stateGate)
            {
                _validationReference = validationReference;
                _activeControllerConfig = activeControllerConfig;
                _validationReferenceCapturedAt = DateTimeOffset.UtcNow;
                _validationReferenceError = null;
            }

            Notify();
            return validationReference;
        }
        catch (Exception exception)
        {
            lock (_stateGate)
            {
                _validationReferenceError = exception.Message;
            }

            Notify();
            throw;
        }
        finally
        {
            _ioLock.Release();
        }
    }

    public async Task<CommandResult> SetConfigAsync(ConfigParam parameter, double value, CancellationToken cancellationToken = default)
    {
        if (!AllowsGuiControl())
        {
            return CommandResult.Failed("Control bloqueado por el modo actual.");
        }

        byte[] payload;
        try
        {
            payload = EscProtocol.ConfigPayload(parameter, value);
        }
        catch (Exception ex) when (ex is OverflowException or ArgumentOutOfRangeException)
        {
            return CommandResult.Failed($"Valor fuera de rango para {parameter}: {value}.");
        }

        CommandResult result = await SendCommandAsync(CommOpcode.SetConfig, (byte)parameter, payload, cancellationToken);
        if (result.Success && parameter is ConfigParam.KpRpm or ConfigParam.KiRpm or ConfigParam.KdRpm or ConfigParam.PolePairs or ConfigParam.PwmFreq)
        {
            await RefreshValidationReferenceAsync(cancellationToken).ConfigureAwait(false);
        }
        if (result.Success && parameter is ConfigParam.MinSpeed or ConfigParam.MaxSpeed)
        {
            await RefreshSpeedLimitsAsync(cancellationToken).ConfigureAwait(false);
        }

        return result;
    }

    public async Task<CommandResult> ResetConfigAsync(ConfigParam parameter, CancellationToken cancellationToken = default)
    {
        if (!AllowsGuiControl())
        {
            return CommandResult.Failed("Control bloqueado por el modo actual.");
        }

        CommandResult result = await SendCommandAsync(
            CommOpcode.ResetConfig, (byte)parameter, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
        if (result.Success)
        {
            await RefreshValidationReferenceAsync(cancellationToken).ConfigureAwait(false);
            if (parameter is ConfigParam.MinSpeed or ConfigParam.MaxSpeed or ConfigParam.All)
            {
                await RefreshSpeedLimitsAsync(cancellationToken).ConfigureAwait(false);
            }
        }

        return result;
    }

    public Task<CommandResult> SaveConfigAsync(ConfigParam parameter = ConfigParam.All, CancellationToken cancellationToken = default)
    {
        if (!AllowsGuiControl())
        {
            return Task.FromResult(CommandResult.Failed("Control bloqueado por el modo actual."));
        }

        return SendCommandAsync(CommOpcode.SaveConfig, (byte)parameter, Array.Empty<byte>(), cancellationToken);
    }

    public async Task<CommandResult> SetControlRuntimeModeAsync(ControlRuntimeMode mode, CancellationToken cancellationToken = default)
    {
        if (!AllowsHilControl())
        {
            return CommandResult.Failed("Control bloqueado por el modo actual.");
        }

        return await SendCommandAsync(CommOpcode.SetControlMode, 0, EscProtocol.ControlModePayload(mode), cancellationToken).ConfigureAwait(false);
    }

    public async Task<CommandResult> HilStartAsync(ushort? inputTimeoutMs = null, CancellationToken cancellationToken = default)
    {
        if (!AllowsHilControl())
        {
            return CommandResult.Failed("Control bloqueado por el modo actual.");
        }

        byte[] payload = inputTimeoutMs.HasValue ? EscProtocol.HilStartPayload(inputTimeoutMs.Value) : Array.Empty<byte>();
        CommandResult result = await SendCommandAsync(CommOpcode.HilStart, 0, payload, cancellationToken, refreshStatus: false).ConfigureAwait(false);
        if (result.Success)
        {
            SetHilEnabled(true);
        }

        return result;
    }

    public async Task<CommandResult> HilStopAsync(CancellationToken cancellationToken = default)
    {
        CommandResult result = await SendCommandAsync(CommOpcode.HilStop, 0, Array.Empty<byte>(), cancellationToken, refreshStatus: false).ConfigureAwait(false);
        if (result.Success)
        {
            SetHilEnabled(false);
        }

        return result;
    }

    public async Task<CommandResult> HilSetInputsAsync(HilInputs inputs, CancellationToken cancellationToken = default)
    {
        if (!AllowsHilControl())
        {
            return CommandResult.Failed("Control bloqueado por el modo actual.");
        }

        CommandResult result = await SendCommandAsync(CommOpcode.HilSetInputs, 0, EscProtocol.HilInputsPayload(inputs), cancellationToken, refreshStatus: false).ConfigureAwait(false);
        if (result.Success)
        {
            SetHilEnabled(inputs.Enable);
        }

        return result;
    }

    public async Task<HilOutputs> HilGetOutputsAsync(CancellationToken cancellationToken = default)
    {
        EscFrame response = await SendRequestAsync(CommOpcode.HilGetOutputs, 0, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
        return EscProtocol.DecodeHilOutputs(response);
    }

    public void UpdateHilStats(HilBridgeStats stats)
    {
        lock (_stateGate)
        {
            _hilStats = stats;
        }

        Notify();
    }

    public void ClearHilOutputs()
    {
        lock (_stateGate)
        {
            _hilStats = _hilStats with { LastOutputs = null };
        }

        Notify();
    }

    public void RecordHilFrame(
        string direction,
        string transport,
        uint? sequence,
        string summary,
        string payload)
    {
        bool collapsed = false;
        lock (_stateGate)
        {
            DateTimeOffset timestamp = DateTimeOffset.Now;
            if (_hilFrameTrace.Count > 0)
            {
                HilFrameTraceEntry last = _hilFrameTrace[^1];
                if (last.Direction == direction &&
                    last.Transport == transport &&
                    last.Sequence == sequence &&
                    last.Summary == summary &&
                    last.Payload == payload)
                {
                    _hilFrameTrace[^1] = last with
                    {
                        Timestamp = timestamp,
                        RepeatCount = last.RepeatCount + 1
                    };
                    collapsed = true;
                }
            }

            if (!collapsed)
            {
                _hilFrameTrace.Add(new HilFrameTraceEntry(
                    timestamp,
                    direction,
                    transport,
                    sequence,
                    summary,
                    payload));

                while (_hilFrameTrace.Count > MaxHilTraceEntries)
                {
                    _hilFrameTrace.RemoveAt(0);
                }
            }
        }

        Notify();
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
            return CommandResult.Failed("Control bloqueado por el modo actual.");
        }

        return await SendCommandAsync(opcode, 0, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
    }

    private async Task<CommandResult> SendSimulinkControlCommandAsync(CommOpcode opcode, CancellationToken cancellationToken)
    {
        if (!AllowsSimulinkControl())
        {
            return CommandResult.Failed("Cambia el modo a Simulink control para enviar comandos de motor.");
        }

        return await SendCommandAsync(opcode, 0, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
    }

    private async Task<CommandResult> SendCommandAsync(CommOpcode opcode, byte parameter, byte[] payload, CancellationToken cancellationToken, bool refreshStatus = true)
    {
        try
        {
            EscFrame response = await SendRequestAsync(opcode, parameter, payload, cancellationToken).ConfigureAwait(false);
            CommandResult result = opcode == CommOpcode.Run && response.Status == CommStatus.InvalidState
                ? new CommandResult(false, response.Status, "RUN no permitido en el estado actual del ESC.")
                : CommandResult.FromStatus(response.Status);
            if (refreshStatus)
            {
                await RefreshStatusAfterCommandAsync(cancellationToken).ConfigureAwait(false);
            }
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
        if (IsPilOpcode(opcode))
        {
            RecordHilBinaryFrame("Bridge -> MCU", request);
        }

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
                if (IsPilOpcode(opcode))
                {
                    RecordHilBinaryFrame("MCU -> Bridge", frame);
                }

                return frame;
            }
        }

        throw new TimeoutException($"Timed out waiting for {opcode} response.");
    }

    private async Task<(ValidationReference Reference, ActiveControllerConfig Config)> ReadValidationReferenceCoreAsync(
        CancellationToken cancellationToken)
    {
        if (!_transport.IsOpen)
        {
            throw new InvalidOperationException("ESC is not connected.");
        }

        EscFrame referenceFrame = await SendRequestCoreAsync(
            CommOpcode.GetValidationReference, 0, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
        ValidationReference reference = EscProtocol.DecodeValidationReference(referenceFrame);
        if (reference.AlgorithmVersion != ValidationReference.SupportedAlgorithmVersion)
        {
            throw new InvalidOperationException(
                $"Unsupported controller algorithm {reference.AlgorithmVersion}; expected {ValidationReference.SupportedAlgorithmVersion}.");
        }
        double kp = await ReadConfigCoreAsync(ConfigParam.KpRpm, cancellationToken).ConfigureAwait(false);
        double ki = await ReadConfigCoreAsync(ConfigParam.KiRpm, cancellationToken).ConfigureAwait(false);
        double kd = await ReadConfigCoreAsync(ConfigParam.KdRpm, cancellationToken).ConfigureAwait(false);
        double polePairs = await ReadConfigCoreAsync(ConfigParam.PolePairs, cancellationToken).ConfigureAwait(false);
        return (reference, new ActiveControllerConfig(kp, ki, kd, checked((byte)polePairs)));
    }

    private async Task<ActiveSpeedLimits> RefreshSpeedLimitsAsync(CancellationToken cancellationToken)
    {
        await _ioLock.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            ActiveSpeedLimits limits = await ReadSpeedLimitsCoreAsync(cancellationToken).ConfigureAwait(false);
            lock (_stateGate)
            {
                _activeSpeedLimits = limits;
            }

            Notify();
            return limits;
        }
        finally
        {
            _ioLock.Release();
        }
    }

    private async Task<ActiveSpeedLimits> ReadSpeedLimitsCoreAsync(CancellationToken cancellationToken)
    {
        double minSpeed = await ReadConfigCoreAsync(ConfigParam.MinSpeed, cancellationToken).ConfigureAwait(false);
        double maxSpeed = await ReadConfigCoreAsync(ConfigParam.MaxSpeed, cancellationToken).ConfigureAwait(false);
        return new ActiveSpeedLimits(checked((ushort)minSpeed), checked((ushort)maxSpeed));
    }

    private async Task<double> ReadConfigCoreAsync(ConfigParam parameter, CancellationToken cancellationToken)
    {
        EscFrame frame = await SendRequestCoreAsync(
            CommOpcode.GetConfig, (byte)parameter, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
        EscProtocol.EnsureOkResponse(frame, CommOpcode.GetConfig);
        object value = EscProtocol.DecodeConfigValue(parameter, frame.Payload);
        return Convert.ToDouble(value, System.Globalization.CultureInfo.InvariantCulture);
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

    private bool AllowsHilControl()
    {
        lock (_stateGate)
        {
            return _mode != ControlMode.MonitorOnly;
        }
    }

    private bool AllowsSimulinkControl()
    {
        lock (_stateGate)
        {
            return _mode == ControlMode.SimulinkControl;
        }
    }

    private void SetHilEnabled(bool enabled)
    {
        lock (_stateGate)
        {
            _hilStats = _hilStats with { Enabled = enabled };
        }

        Notify();
    }

    private void UpdateCachedSpeedSetpoint(ushort rpm)
    {
        lock (_stateGate)
        {
            if (_status is not null)
            {
                _status = _status with { SpeedSetpointRpm = rpm };
            }
        }

        Notify();
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
        HilBridgeStats hilStats = _hilStats with { RecentFrames = _hilFrameTrace.AsEnumerable().Reverse().ToArray() };
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
            _telemetryStore.GetStats("speed", "rpm"),
            hilStats,
            _validationReference,
            _activeControllerConfig,
            _activeSpeedLimits,
            _validationReferenceCapturedAt,
            _validationReferenceError);
    }

    private void ClearValidationReference()
    {
        _validationReference = null;
        _activeControllerConfig = null;
        _activeSpeedLimits = null;
        _validationReferenceCapturedAt = null;
        _validationReferenceError = null;
    }

    private void Notify()
    {
        SnapshotChanged?.Invoke(this, EventArgs.Empty);
    }

    private static bool IsPilOpcode(CommOpcode opcode)
    {
        return opcode is CommOpcode.HilStart or CommOpcode.HilStop or CommOpcode.HilSetInputs or CommOpcode.HilGetOutputs;
    }

    private void RecordHilBinaryFrame(string direction, byte[] rawFrame)
    {
        try
        {
            RecordHilBinaryFrame(direction, EscProtocol.Parse(rawFrame));
        }
        catch (Exception ex)
        {
            RecordHilFrame(direction, "HID", null, "binary frame parse error", ex.Message);
        }
    }

    private void RecordHilBinaryFrame(string direction, EscFrame frame)
    {
        string summary = $"{frame.Type} {frame.Opcode} seq={frame.Sequence} len={frame.Payload.Length} status={frame.Status}";
        string payload = frame.Payload.Length == 0
            ? $"payload=- raw={Convert.ToHexString(frame.Raw)}"
            : $"payload={Convert.ToHexString(frame.Payload)} raw={Convert.ToHexString(frame.Raw)}";
        RecordHilFrame(direction, "HID", frame.Sequence, summary, payload);
    }
}
