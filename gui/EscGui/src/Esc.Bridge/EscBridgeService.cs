using Esc.Protocol;
using Esc.Transport;
using Microsoft.Extensions.Logging;

namespace Esc.Bridge;

public sealed class EscBridgeService
{
    private const byte SineDriveAppState = 10;
    private const byte StepTestAppState = 11;
    private const byte FreewheelAppState = 12;

    private readonly IEscDeviceEnumerator _deviceEnumerator;
    private readonly IEscTransport _transport;
    private readonly TelemetryStore _telemetryStore;
    private readonly ILogger<EscBridgeService> _logger;
    private readonly SemaphoreSlim _ioLock = new(1, 1);
    private readonly SemaphoreSlim _controlGate = new(1, 1);
    private readonly object _stateGate = new();

    private IReadOnlyList<HidDeviceDescriptor> _devices = Array.Empty<HidDeviceDescriptor>();
    private HidDeviceDescriptor? _currentDevice;
    private EscStatus? _status;
    private DeviceConnectionState _state = DeviceConnectionState.NotDetected;
    private ControlMode _mode = ControlMode.GuiControl;
    private string? _lastError;
    private readonly HashSet<LogParam> _enabledLogs = new();
    private ushort _logRateMs = 500;
    private byte _sequence;
    private int _emergencyStopPending;
    private int _emergencyStopGeneration;
    private ActiveControllerConfig? _activeControllerConfig;
    private ActiveSpeedLimits? _activeSpeedLimits;
    private StartupConfiguration? _startupConfiguration;
    private SineDriveSettings? _requestedSineDrive;
    private SineDriveSettings? _appliedSineDrive;
    private DateTimeOffset? _sineDriveLastAppliedAt;
    private StepTestSettings? _requestedStepTest;
    private StepTestSettings? _appliedStepTest;
    private DateTimeOffset? _stepTestLastAppliedAt;
    private ZcDiagReport? _freewheelReport;
    private ZcChannelMap? _activeZcMap;
    private ZcAutoDetectResult? _lastZcAutoDetect;
    private DateTimeOffset? _freewheelLastUpdatedAt;
    private LoadSweepStatus? _loadSweepStatus;
    private readonly List<LoadSweepPoint> _loadSweepPoints = new();
    private DateTimeOffset? _loadSweepUpdatedAt;
    private string? _loadSweepError;
    private readonly List<SavedLoadCurve> _savedCurves = new();
    private Guid? _tareCurveId;

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
    public bool IsTransportOpen => _transport.IsOpen;

    public void ClearSpeedTelemetry()
    {
        _telemetryStore.Clear("speed");
        Notify();
    }

    public IReadOnlyList<TelemetrySample> GetTelemetrySamples(LogParam parameter) =>
        _telemetryStore.GetSamples(EscProtocol.TelemetryVariable(parameter));

    public void ClearTelemetry(LogParam parameter)
    {
        _telemetryStore.Clear(EscProtocol.TelemetryVariable(parameter));
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
                _enabledLogs.Clear();
                _state = DeviceConnectionState.Disconnected;
                _lastError = "Dispositivo HID desconectado.";
                _status = null;
                _currentDevice = null;
                ClearManualOutputs();
                ClearDeviceCache();
            }
            else if (!_transport.IsOpen && devices.Count > 0 && _state is DeviceConnectionState.NotDetected or DeviceConnectionState.Disconnected)
            {
                _state = DeviceConnectionState.Detected;
                _lastError = null;
            }
            else if (!_transport.IsOpen && devices.Count == 0)
            {
                _enabledLogs.Clear();
                _state = DeviceConnectionState.NotDetected;
                _lastError = null;
                _currentDevice = null;
                _status = null;
                ClearManualOutputs();
                ClearDeviceCache();
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
            ActiveControllerConfig activeControllerConfig =
                await ReadControllerConfigCoreAsync(cancellationToken).ConfigureAwait(false);
            ActiveSpeedLimits activeSpeedLimits = await ReadSpeedLimitsCoreAsync(cancellationToken).ConfigureAwait(false);
            StartupConfiguration startupConfiguration = await ReadStartupConfigurationCoreAsync(cancellationToken).ConfigureAwait(false);

            lock (_stateGate)
            {
                _state = DeviceConnectionState.Connected;
                _currentDevice = descriptor;
                _status = status;
                _lastError = null;
                _activeControllerConfig = activeControllerConfig;
                _activeSpeedLimits = activeSpeedLimits;
                _startupConfiguration = startupConfiguration;
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
        await _controlGate.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            if (HasRequestedManualOutput() && _transport.IsOpen)
            {
                await StopSineDriveCoreAsync(false, false, cancellationToken).ConfigureAwait(false);
            }
            lock (_stateGate)
            {
                ClearManualOutputs();
            }
            await _transport.CloseAsync(cancellationToken).ConfigureAwait(false);
            lock (_stateGate)
            {
                _enabledLogs.Clear();
                _state = _devices.Count > 0 ? DeviceConnectionState.Detected : DeviceConnectionState.NotDetected;
                _currentDevice = null;
                _status = null;
                _lastError = null;
                ClearDeviceCache();
            }

            Notify();
        }
        finally
        {
            _controlGate.Release();
        }
    }

    public async Task<CommandResult> SetModeAsync(ControlMode mode, CancellationToken cancellationToken = default)
    {
        await _controlGate.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            if (mode != ControlMode.GuiControl && HasRequestedManualOutput())
            {
                CommandResult stop = await StopSineDriveCoreAsync(
                    false, true, cancellationToken).ConfigureAwait(false);
                if (!stop.Success)
                {
                    return CommandResult.Failed("No se pudo detener el lazo abierto antes de cambiar de modo.");
                }
            }

            lock (_stateGate)
            {
                _mode = mode;
            }

            Notify();
            return CommandResult.Ok("Modo actualizado.");
        }
        finally
        {
            _controlGate.Release();
        }
    }

    public Task<CommandResult> RunAsync(CancellationToken cancellationToken = default)
    {
        return SendControlCommandAsync(CommOpcode.Run, cancellationToken);
    }

    public async Task<CommandResult> StopAsync(CancellationToken cancellationToken = default)
    {
        if (!AllowsGuiControl())
        {
            return CommandResult.Failed("Control bloqueado por el modo actual.");
        }

        await _controlGate.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            if (!AllowsGuiControl()) return CommandResult.Failed("Control bloqueado por el modo actual.");
            return await StopSineDriveCoreAsync(false, true, cancellationToken).ConfigureAwait(false);
        }
        finally
        {
            _controlGate.Release();
        }
    }

    public async Task<CommandResult> EmergencyStopAsync(CancellationToken cancellationToken = default)
    {
        if (Interlocked.Exchange(ref _emergencyStopPending, 1) != 0)
        {
            return CommandResult.Failed("ESTOP ya esta en curso.");
        }
        Interlocked.Increment(ref _emergencyStopGeneration);

        lock (_stateGate)
        {
            _requestedSineDrive = null;
            _requestedStepTest = null;
        }
        Notify();
        try
        {
            return await StopSineDriveCoreAsync(true, false, cancellationToken).ConfigureAwait(false);
        }
        finally
        {
            Interlocked.Exchange(ref _emergencyStopPending, 0);
        }
    }

    public Task<CommandResult> SetSineDriveAsync(
        double electricalFrequencyHz,
        double amplitudePercent,
        CancellationToken cancellationToken = default)
    {
        return ApplySineDriveAsync(
            electricalFrequencyHz,
            amplitudePercent,
            SineDriveCommand.Apply,
            refreshStatus: true,
            requireActive: false,
            cancellationToken: cancellationToken);
    }

    public Task<CommandResult> UpdateSineDriveAsync(
        double electricalFrequencyHz,
        double amplitudePercent,
        CancellationToken cancellationToken = default)
    {
        return ApplySineDriveAsync(
            electricalFrequencyHz,
            amplitudePercent,
            SineDriveCommand.KeepAlive,
            refreshStatus: false,
            requireActive: true,
            cancellationToken: cancellationToken);
    }

    private async Task<CommandResult> ApplySineDriveAsync(
        double electricalFrequencyHz,
        double amplitudePercent,
        SineDriveCommand command,
        bool refreshStatus,
        bool requireActive,
        CancellationToken cancellationToken = default)
    {
        int emergencyGeneration = Volatile.Read(ref _emergencyStopGeneration);
        if (IsEmergencyStopPending()) return CommandResult.Failed("ESTOP en curso.");
        await _controlGate.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            if (!AllowsGuiControl())
            {
                return CommandResult.Failed("Control bloqueado por el modo actual.");
            }
            if (IsEmergencyStopPending() ||
                emergencyGeneration != Volatile.Read(ref _emergencyStopGeneration))
            {
                return CommandResult.Failed("ESTOP en curso.");
            }
            if (requireActive)
            {
                lock (_stateGate)
                {
                    if (_status?.AppState != 10 || _requestedSineDrive is null)
                    {
                        return CommandResult.Failed("El lazo abierto ya no esta activo.");
                    }
                }
            }

            byte[] payload;
            try
            {
                payload = EscProtocol.SineDrivePayload(electricalFrequencyHz, amplitudePercent);
            }
            catch (ArgumentOutOfRangeException)
            {
                return CommandResult.Failed("La frecuencia debe estar entre 2 y 10 Hz y la amplitud entre 0 y 100 %.");
            }

            EscFrame response = await SendRequestAsync(
                CommOpcode.SineDrive, (byte)command, payload, cancellationToken).ConfigureAwait(false);
            if (IsEmergencyStopPending() ||
                emergencyGeneration != Volatile.Read(ref _emergencyStopGeneration))
            {
                return CommandResult.Failed("ESTOP en curso; la salida sera detenida.");
            }
            CommandResult result = CommandResult.FromStatus(response.Status);
            if (!result.Success)
            {
                await RefreshStatusAfterCommandAsync(cancellationToken).ConfigureAwait(false);
                return result;
            }

            SineDriveSettings applied = EscProtocol.DecodeSineDrive(response);
            lock (_stateGate)
            {
                _requestedSineDrive = new SineDriveSettings(electricalFrequencyHz, amplitudePercent);
                _appliedSineDrive = applied;
                _sineDriveLastAppliedAt = DateTimeOffset.UtcNow;
            }
            if (refreshStatus)
            {
                await RefreshStatusAfterCommandAsync(cancellationToken).ConfigureAwait(false);
            }
            else
            {
                lock (_stateGate)
                {
                    if (_status is not null)
                    {
                        _status = _status with { AppState = 10, AppStateName = "SINE_DRIVE" };
                    }
                }
                Notify();
            }
            return CommandResult.Ok(
                $"Lazo abierto activo: {applied.ElectricalFrequencyHz:0.###} Hz, {applied.AmplitudePercent:0.#} %.");
        }
        catch (OperationCanceledException) when (IsEmergencyStopPending())
        {
            return CommandResult.Failed("Comando interrumpido por ESTOP.");
        }
        catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
        {
            return CommandResult.Failed("Actualizacion cancelada.");
        }
        catch (Exception ex)
        {
            SetError(DeviceConnectionState.Error, ex.Message);
            return CommandResult.Failed(ex.Message);
        }
        finally
        {
            _controlGate.Release();
        }
    }

    public async Task MaintainSineDriveAsync(CancellationToken cancellationToken = default)
    {
        if (!_transport.IsOpen) return;

        if (!await _controlGate.WaitAsync(0, cancellationToken).ConfigureAwait(false)) return;

        try
        {
            await _ioLock.WaitAsync(cancellationToken).ConfigureAwait(false);
            try
            {
                SineDriveSettings? requested;
                lock (_stateGate)
                {
                    requested = _mode == ControlMode.GuiControl ? _requestedSineDrive : null;
                }
                if (requested is null) return;

                byte[] payload = EscProtocol.SineDrivePayload(
                    requested.ElectricalFrequencyHz, requested.AmplitudePercent);
                EscFrame response = await SendRequestCoreAsync(
                    CommOpcode.SineDrive,
                    (byte)SineDriveCommand.KeepAlive,
                    payload,
                    cancellationToken,
                    responseDeadlineMs: 300).ConfigureAwait(false);
                if (response.Status != CommStatus.Ok)
                {
                    lock (_stateGate)
                    {
                        ClearSineDrive();
                    }
                    Notify();
                    return;
                }

                SineDriveSettings applied = EscProtocol.DecodeSineDrive(response);
                lock (_stateGate)
                {
                    _appliedSineDrive = applied;
                    _sineDriveLastAppliedAt = DateTimeOffset.UtcNow;
                }
            }
            finally
            {
                _ioLock.Release();
            }
        }
        finally
        {
            _controlGate.Release();
        }
    }

    public async Task<CommandResult> SetStepTestAsync(
        StepTestSettings settings,
        CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(settings);
        int emergencyGeneration = Volatile.Read(ref _emergencyStopGeneration);
        if (IsEmergencyStopPending()) return CommandResult.Failed("ESTOP en curso.");
        await _controlGate.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            if (!AllowsGuiControl())
            {
                return CommandResult.Failed("Control bloqueado por el modo actual.");
            }

            byte[] payload;
            try
            {
                payload = EscProtocol.StepTestPayload(settings);
            }
            catch (ArgumentOutOfRangeException)
            {
                return CommandResult.Failed(
                    $"El paso debe ser UV..WV y el duty estar entre 0 y {CommConstants.MaxStepTestDutyPercent:0} %.");
            }

            EscFrame response = await SendRequestAsync(
                CommOpcode.StepTest, (byte)StepTestCommand.Apply, payload, cancellationToken).ConfigureAwait(false);
            if (IsEmergencyStopPending() ||
                emergencyGeneration != Volatile.Read(ref _emergencyStopGeneration))
            {
                return CommandResult.Failed("ESTOP en curso; la salida sera detenida.");
            }
            CommandResult result = CommandResult.FromStatus(response.Status);
            if (!result.Success)
            {
                await RefreshStatusAfterCommandAsync(cancellationToken).ConfigureAwait(false);
                return result;
            }

            StepTestSettings applied = EscProtocol.DecodeStepTest(response);
            lock (_stateGate)
            {
                _requestedStepTest = settings;
                _appliedStepTest = applied;
                _stepTestLastAppliedAt = DateTimeOffset.UtcNow;
            }
            await RefreshStatusAfterCommandAsync(cancellationToken).ConfigureAwait(false);
            return CommandResult.Ok($"Paso {DescribeStepTest(applied)} energizado.");
        }
        catch (OperationCanceledException) when (IsEmergencyStopPending())
        {
            return CommandResult.Failed("Comando interrumpido por ESTOP.");
        }
        catch (Exception ex)
        {
            SetError(DeviceConnectionState.Error, ex.Message);
            return CommandResult.Failed(ex.Message);
        }
        finally
        {
            _controlGate.Release();
        }
    }

    public async Task MaintainStepTestAsync(CancellationToken cancellationToken = default)
    {
        if (!_transport.IsOpen) return;

        if (!await _controlGate.WaitAsync(0, cancellationToken).ConfigureAwait(false)) return;

        try
        {
            await _ioLock.WaitAsync(cancellationToken).ConfigureAwait(false);
            try
            {
                StepTestSettings? requested;
                lock (_stateGate)
                {
                    requested = _mode == ControlMode.GuiControl ? _requestedStepTest : null;
                }
                if (requested is null) return;

                EscFrame response = await SendRequestCoreAsync(
                    CommOpcode.StepTest,
                    (byte)StepTestCommand.KeepAlive,
                    EscProtocol.StepTestPayload(requested),
                    cancellationToken,
                    responseDeadlineMs: 300).ConfigureAwait(false);
                if (response.Status != CommStatus.Ok)
                {
                    lock (_stateGate)
                    {
                        ClearStepTest();
                    }
                    Notify();
                    return;
                }

                StepTestSettings applied = EscProtocol.DecodeStepTest(response);
                lock (_stateGate)
                {
                    _appliedStepTest = applied;
                    _stepTestLastAppliedAt = DateTimeOffset.UtcNow;
                }
            }
            finally
            {
                _ioLock.Release();
            }
        }
        finally
        {
            _controlGate.Release();
        }
    }

    private static string DescribeStepTest(StepTestSettings settings)
    {
        string step = settings.Step.ToString();
        return string.Create(
            System.Globalization.CultureInfo.InvariantCulture,
            $"{step} ({step[0]} PWM {settings.DutyPercent:0.#} %, {step[1]} LOW)");
    }

    public async Task<CommandResult> StartFreewheelAsync(CancellationToken cancellationToken = default)
    {
        if (IsEmergencyStopPending()) return CommandResult.Failed("ESTOP en curso.");
        await _controlGate.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            if (!AllowsGuiControl())
            {
                return CommandResult.Failed("Control bloqueado por el modo actual.");
            }

            EscFrame response = await SendRequestAsync(
                CommOpcode.ZcDiag, (byte)ZcDiagCommand.Start, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
            CommandResult result = CommandResult.FromStatus(response.Status);
            if (!result.Success)
            {
                await RefreshStatusAfterCommandAsync(cancellationToken).ConfigureAwait(false);
                return result;
            }

            ZcDiagReport report = EscProtocol.DecodeZcDiagReport(response);
            lock (_stateGate)
            {
                _freewheelReport = report;
                _activeZcMap = report.ActiveMap;
                _freewheelLastUpdatedAt = DateTimeOffset.UtcNow;
            }
            await RefreshStatusAfterCommandAsync(cancellationToken).ConfigureAwait(false);
            return CommandResult.Ok("Modo Freewheel activo (potencia apagada). Gira el rotor a mano.");
        }
        catch (Exception ex)
        {
            SetError(DeviceConnectionState.Error, ex.Message);
            return CommandResult.Failed(ex.Message);
        }
        finally
        {
            _controlGate.Release();
        }
    }

    public async Task<CommandResult> StopFreewheelAsync(CancellationToken cancellationToken = default)
    {
        if (IsEmergencyStopPending()) return CommandResult.Failed("ESTOP en curso.");
        await _controlGate.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            if (!AllowsGuiControl())
            {
                return CommandResult.Failed("Control bloqueado por el modo actual.");
            }

            EscFrame response = await SendRequestAsync(
                CommOpcode.ZcDiag, (byte)ZcDiagCommand.Stop, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
            CommandResult result = CommandResult.FromStatus(response.Status);
            if (result.Success)
            {
                lock (_stateGate)
                {
                    if (_freewheelReport is not null)
                    {
                        _freewheelReport = _freewheelReport with { Active = false };
                        _freewheelLastUpdatedAt = DateTimeOffset.UtcNow;
                    }
                }
            }
            await RefreshStatusAfterCommandAsync(cancellationToken).ConfigureAwait(false);
            return result.Success ? CommandResult.Ok("Modo Freewheel detenido.") : result;
        }
        catch (Exception ex)
        {
            SetError(DeviceConnectionState.Error, ex.Message);
            return CommandResult.Failed(ex.Message);
        }
        finally
        {
            _controlGate.Release();
        }
    }

    public async Task PollFreewheelAsync(CancellationToken cancellationToken = default)
    {
        if (!_transport.IsOpen) return;

        bool active;
        lock (_stateGate)
        {
            active = _status?.AppState == FreewheelAppState;
        }
        if (!active) return;

        if (!await _controlGate.WaitAsync(0, cancellationToken).ConfigureAwait(false)) return;
        try
        {
            await _ioLock.WaitAsync(cancellationToken).ConfigureAwait(false);
            try
            {
                lock (_stateGate)
                {
                    if (_status?.AppState != FreewheelAppState) return;
                }

                EscFrame response = await SendRequestCoreAsync(
                    CommOpcode.ZcDiag,
                    (byte)ZcDiagCommand.GetReport,
                    Array.Empty<byte>(),
                    cancellationToken,
                    responseDeadlineMs: 300).ConfigureAwait(false);
                if (response.Status != CommStatus.Ok) return;

                ZcDiagReport report = EscProtocol.DecodeZcDiagReport(response);
                lock (_stateGate)
                {
                    _freewheelReport = report;
                    _activeZcMap = report.ActiveMap;
                    _freewheelLastUpdatedAt = DateTimeOffset.UtcNow;
                }
                Notify();
            }
            finally
            {
                _ioLock.Release();
            }
        }
        finally
        {
            _controlGate.Release();
        }
    }

    public async Task<CommandResult> RunZcAutoDetectAsync(
        double dutyPercent = CommConstants.DefaultZcAutoDetectDutyPercent,
        CancellationToken cancellationToken = default)
    {
        if (IsEmergencyStopPending()) return CommandResult.Failed("ESTOP en curso.");
        await _controlGate.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            if (!AllowsGuiControl())
            {
                return CommandResult.Failed("Control bloqueado por el modo actual.");
            }

            byte[] payload;
            try
            {
                payload = EscProtocol.ZcAutoDetectPayload(dutyPercent);
            }
            catch (ArgumentOutOfRangeException)
            {
                return CommandResult.Failed(
                    $"El duty de autodeteccion debe estar entre {CommConstants.MinZcAutoDetectDutyPercent:0} y {CommConstants.MaxZcAutoDetectDutyPercent:0} %.");
            }

            EscFrame response = await SendRequestAsync(
                CommOpcode.ZcDiag, (byte)ZcDiagCommand.AutoDetect, payload, cancellationToken).ConfigureAwait(false);
            CommandResult result = CommandResult.FromStatus(response.Status);
            if (!result.Success)
            {
                await RefreshStatusAfterCommandAsync(cancellationToken).ConfigureAwait(false);
                return result;
            }

            ZcAutoDetectResult detected = EscProtocol.DecodeZcAutoDetect(response);
            lock (_stateGate)
            {
                _lastZcAutoDetect = detected;
                _activeZcMap = detected.ActiveMap;
                _freewheelLastUpdatedAt = DateTimeOffset.UtcNow;
            }
            await RefreshStatusAfterCommandAsync(cancellationToken).ConfigureAwait(false);

            return detected.Status switch
            {
                ZcAutoDetectStatus.Ok when detected.MatchesActiveMap =>
                    CommandResult.Ok($"Mapeo ZC confirmado: {detected.DetectedMap!.FormatSummary()}."),
                ZcAutoDetectStatus.Ok =>
                    CommandResult.Ok($"Mapeo ZC detectado: {detected.DetectedMap!.FormatSummary()} (difiere del activo; podes aplicarlo por software)."),
                ZcAutoDetectStatus.OkInverted =>
                    CommandResult.Ok($"Mapeo detectado con polaridad invertida: {detected.DetectedMap!.FormatSummary()}. Revisar comparadores."),
                ZcAutoDetectStatus.NoSignal =>
                    CommandResult.Failed("Sin señal en los comparadores ZC (verificar Vbus / neutro virtual)."),
                _ =>
                    CommandResult.Failed("Lectura ambigua al sondear las fases (verificar cableado o subir levemente el duty).")
            };
        }
        catch (Exception ex)
        {
            SetError(DeviceConnectionState.Error, ex.Message);
            return CommandResult.Failed(ex.Message);
        }
        finally
        {
            _controlGate.Release();
        }
    }

    public async Task<ZcChannelMap?> GetZcMapAsync(CancellationToken cancellationToken = default)
    {
        if (!_transport.IsOpen) return null;

        try
        {
            EscFrame response = await SendRequestAsync(
                CommOpcode.GetConfig, (byte)ConfigParam.ZcMap, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
            if (response.Status != CommStatus.Ok || response.Payload.Length != 3)
            {
                return null;
            }

            ZcChannelMap map = EscProtocol.DecodeZcMap(response.Payload);
            lock (_stateGate)
            {
                _activeZcMap = map;
            }
            Notify();
            return map;
        }
        catch (Exception ex)
        {
            _logger.LogDebug(ex, "GetZcMapAsync failed.");
            return null;
        }
    }

    public async Task<CommandResult> SetZcMapAsync(ZcChannelMap map, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(map);
        if (!AllowsGuiControl())
        {
            return CommandResult.Failed("Control bloqueado por el modo actual.");
        }
        if (!map.IsValid)
        {
            return CommandResult.Failed("Cada fase (U, V, W) debe apuntar a un pin ZC distinto (PA0, PA1, PB10).");
        }

        try
        {
            EscFrame response = await SendRequestAsync(
                CommOpcode.SetConfig, (byte)ConfigParam.ZcMap, EscProtocol.ZcMapPayload(map), cancellationToken).ConfigureAwait(false);
            CommandResult result = CommandResult.FromStatus(response.Status);
            if (result.Success)
            {
                lock (_stateGate)
                {
                    _activeZcMap = map;
                    if (_freewheelReport is not null)
                    {
                        _freewheelReport = _freewheelReport with { ActiveMap = map };
                    }
                    if (_lastZcAutoDetect is not null)
                    {
                        _lastZcAutoDetect = _lastZcAutoDetect with { ActiveMap = map };
                    }
                }
                Notify();
                return CommandResult.Ok($"Mapeo ZC aplicado en RAM: {map.FormatSummary()}. Guarda en Flash para persistirlo.");
            }
            return result;
        }
        catch (Exception ex)
        {
            SetError(DeviceConnectionState.Error, ex.Message);
            return CommandResult.Failed(ex.Message);
        }
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

    public async Task<object?> GetConfigAsync(ConfigParam parameter, CancellationToken cancellationToken = default)
    {
        EscFrame response = await SendRequestAsync(CommOpcode.GetConfig, (byte)parameter, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
        EscProtocol.EnsureOkResponse(response, CommOpcode.GetConfig);
        return EscProtocol.DecodeConfigValue(parameter, response.Payload);
    }

    public async Task<ActiveControllerConfig> RefreshControllerConfigAsync(CancellationToken cancellationToken = default)
    {
        await _ioLock.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            ActiveControllerConfig config = await ReadControllerConfigCoreAsync(cancellationToken).ConfigureAwait(false);
            lock (_stateGate)
            {
                _activeControllerConfig = config;
            }

            Notify();
            return config;
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
        if (result.Success && IsControllerParameter(parameter))
        {
            await RefreshControllerConfigAsync(cancellationToken).ConfigureAwait(false);
        }
        if (result.Success && parameter is ConfigParam.MinSpeed or ConfigParam.MaxSpeed)
        {
            await RefreshSpeedLimitsAsync(cancellationToken).ConfigureAwait(false);
        }
        if (result.Success && IsStartupParameter(parameter))
        {
            await RefreshStartupConfigurationAsync(cancellationToken).ConfigureAwait(false);
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
            await RefreshControllerConfigAsync(cancellationToken).ConfigureAwait(false);
            if (parameter is ConfigParam.MinSpeed or ConfigParam.MaxSpeed or ConfigParam.All)
            {
                await RefreshSpeedLimitsAsync(cancellationToken).ConfigureAwait(false);
            }
            if (IsStartupParameter(parameter) || parameter == ConfigParam.All)
            {
                await RefreshStartupConfigurationAsync(cancellationToken).ConfigureAwait(false);
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

    public async Task<CommandResult> StartLoadSweepAsync(
        LoadSweepRequest request,
        CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(request);
        if (IsEmergencyStopPending()) return CommandResult.Failed("ESTOP en curso.");
        if (!AllowsGuiControl())
        {
            return CommandResult.Failed("Control bloqueado por el modo actual.");
        }

        string? invalid = ValidateLoadSweep(request);
        if (invalid is not null)
        {
            return CommandResult.Failed(invalid);
        }

        try
        {
            EscFrame response = await SendRequestAsync(
                CommOpcode.LoadSweep,
                (byte)LoadSweepCommand.Start,
                EscProtocol.LoadSweepStartPayload(request),
                cancellationToken).ConfigureAwait(false);
            CommandResult result = response.Status switch
            {
                CommStatus.Ok => CommandResult.Ok("Barrido iniciado."),
                CommStatus.InvalidState => new CommandResult(
                    false, response.Status, "El barrido solo arranca en IDLE o CLOSEDLOOP y sin otro barrido activo."),
                CommStatus.UnderLimit or CommStatus.OverLimit => new CommandResult(
                    false, response.Status, "El ESC rechazo los limites del barrido (rpm fuera del rango activo o demasiados puntos)."),
                _ => CommandResult.FromStatus(response.Status)
            };
            if (result.Success)
            {
                lock (_stateGate)
                {
                    _loadSweepPoints.Clear();
                    _loadSweepError = null;
                }
            }

            await RefreshLoadSweepAsync(cancellationToken).ConfigureAwait(false);
            await RefreshStatusAfterCommandAsync(cancellationToken).ConfigureAwait(false);
            return result;
        }
        catch (Exception ex)
        {
            SetError(DeviceConnectionState.Error, ex.Message);
            return CommandResult.Failed(ex.Message);
        }
    }

    /// <summary>Abort the running sweep; the motor keeps the last reference. Allowed in every mode.</summary>
    public async Task<CommandResult> AbortLoadSweepAsync(CancellationToken cancellationToken = default)
    {
        try
        {
            CommandResult result = await SendCommandAsync(
                CommOpcode.LoadSweep, (byte)LoadSweepCommand.Abort, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
            await RefreshLoadSweepAsync(cancellationToken).ConfigureAwait(false);
            return result.Success ? CommandResult.Ok("Barrido abortado; el motor mantiene la referencia actual.") : result;
        }
        catch (Exception ex)
        {
            SetError(DeviceConnectionState.Error, ex.Message);
            return CommandResult.Failed(ex.Message);
        }
    }

    /// <summary>Read the sweep status and copy any new points from the MCU table.</summary>
    public async Task<LoadSweepSnapshot> RefreshLoadSweepAsync(CancellationToken cancellationToken = default)
    {
        if (!_transport.IsOpen)
        {
            return Snapshot.LoadSweep;
        }

        try
        {
            EscFrame statusFrame = await SendRequestAsync(
                CommOpcode.LoadSweep, (byte)LoadSweepCommand.GetStatus, Array.Empty<byte>(), cancellationToken,
                responseDeadlineMs: 500).ConfigureAwait(false);
            LoadSweepStatus status = EscProtocol.DecodeLoadSweepStatus(statusFrame);

            int known;
            lock (_stateGate)
            {
                if (status.PointCount < _loadSweepPoints.Count)
                {
                    _loadSweepPoints.Clear();
                }
                known = _loadSweepPoints.Count;
            }

            var fetched = new List<LoadSweepPoint>();
            while (known + fetched.Count < status.PointCount)
            {
                ushort first = checked((ushort)(known + fetched.Count));
                byte[] payload = EscProtocol.UInt16Payload(first);
                EscFrame pointsFrame = await SendRequestAsync(
                    CommOpcode.LoadSweep, (byte)LoadSweepCommand.GetPoints, payload, cancellationToken,
                    responseDeadlineMs: 500).ConfigureAwait(false);
                LoadSweepPointBatch batch = EscProtocol.DecodeLoadSweepPoints(pointsFrame);
                if (batch.Points.Count == 0)
                {
                    break;
                }
                fetched.AddRange(batch.Points);
            }

            lock (_stateGate)
            {
                if (_loadSweepPoints.Count == known)
                {
                    _loadSweepPoints.AddRange(fetched);
                }
                _loadSweepStatus = status;
                _loadSweepUpdatedAt = DateTimeOffset.UtcNow;
                _loadSweepError = null;
                if (_status is not null && _status.LoadSweepActive != status.IsActive)
                {
                    // Keep the cached GET_STATUS flag coherent until the next status refresh.
                    _status = _status with { LoadSweepActive = status.IsActive };
                }
            }
        }
        catch (Exception ex) when (ex is EscProtocolException or TimeoutException)
        {
            _logger.LogDebug(ex, "Load sweep refresh failed.");
            lock (_stateGate)
            {
                _loadSweepError = ex.Message;
            }
        }

        Notify();
        return Snapshot.LoadSweep;
    }

    /// <summary>Background poll: follows a running sweep and drains the points of a finished one.</summary>
    public async Task PollLoadSweepAsync(CancellationToken cancellationToken = default)
    {
        if (!_transport.IsOpen) return;

        bool follow;
        lock (_stateGate)
        {
            follow = _status?.LoadSweepActive == true ||
                     _loadSweepStatus?.IsActive == true ||
                     (_loadSweepStatus is { } status && status.PointCount > _loadSweepPoints.Count);
        }
        if (!follow) return;

        await RefreshLoadSweepAsync(cancellationToken).ConfigureAwait(false);
    }

    /// <summary>Forget the points copied to the host; the MCU table is untouched.</summary>
    public void ClearLoadSweepPoints()
    {
        lock (_stateGate)
        {
            _loadSweepPoints.Clear();
            _loadSweepStatus = null;
            _loadSweepError = null;
        }

        Notify();
    }

    /// <summary>Keep the points of the current sweep as a named curve for comparison.</summary>
    public CommandResult SaveCurrentLoadCurve(string? label = null)
    {
        lock (_stateGate)
        {
            if (_loadSweepPoints.Count == 0)
            {
                return CommandResult.Failed("No hay puntos para guardar.");
            }

            string name = string.IsNullOrWhiteSpace(label)
                ? $"Curva {DateTime.Now:HH:mm:ss}"
                : label.Trim();
            _savedCurves.Add(new SavedLoadCurve(
                Guid.NewGuid(), name, _loadSweepPoints.ToArray(), "MCU", DateTimeOffset.Now));
        }

        Notify();
        return CommandResult.Ok("Curva guardada para comparar.");
    }

    /// <summary>Import a CSV exported by this GUI as a named curve.</summary>
    public CommandResult ImportLoadCurveCsv(string label, string csv)
    {
        IReadOnlyList<LoadSweepPoint> points;
        try
        {
            points = LoadSweepCsvReader.Parse(csv);
        }
        catch (FormatException ex)
        {
            return CommandResult.Failed($"No se pudo leer el CSV: {ex.Message}");
        }

        if (points.Count == 0)
        {
            return CommandResult.Failed("El CSV no tiene puntos.");
        }

        lock (_stateGate)
        {
            _savedCurves.Add(new SavedLoadCurve(
                Guid.NewGuid(), string.IsNullOrWhiteSpace(label) ? "CSV" : label.Trim(), points, "CSV", DateTimeOffset.Now));
        }

        Notify();
        return CommandResult.Ok($"Curva importada ({points.Count} puntos).");
    }

    public void RemoveSavedLoadCurve(Guid id)
    {
        lock (_stateGate)
        {
            _savedCurves.RemoveAll(curve => curve.Id == id);
            if (_tareCurveId == id)
            {
                _tareCurveId = null;
            }
        }

        Notify();
    }

    /// <summary>Use a saved curve as tare; null clears it.</summary>
    public void SetLoadCurveTare(Guid? id)
    {
        lock (_stateGate)
        {
            _tareCurveId = id is { } value && _savedCurves.Any(curve => curve.Id == value) ? value : null;
        }

        Notify();
    }

    public string? SavedLoadCurveCsv(Guid id)
    {
        SavedLoadCurve? curve = Snapshot.LoadSweep.SavedCurves.FirstOrDefault(item => item.Id == id);
        return curve is null ? null : LoadSweepCsvWriter.Write(null, curve.Points);
    }

    public string LoadSweepCsv()
    {
        LoadSweepSnapshot sweep = Snapshot.LoadSweep;
        return LoadSweepCsvWriter.Write(sweep.Status, sweep.Points);
    }

    private string? ValidateLoadSweep(LoadSweepRequest request)
    {
        ActiveSpeedLimits? limits;
        lock (_stateGate)
        {
            limits = _activeSpeedLimits;
        }

        if (limits is not null &&
            (request.StartRpm < limits.MinRpm || request.EndRpm < limits.MinRpm ||
             request.StartRpm > limits.MaxRpm || request.EndRpm > limits.MaxRpm))
        {
            return $"Las rpm del barrido deben estar dentro del rango activo {limits.MinRpm}-{limits.MaxRpm} rpm.";
        }
        if (request.WindowMs is < CommConstants.LoadSweepMinWindowMs or > CommConstants.LoadSweepMaxWindowMs)
        {
            return $"La ventana debe estar entre {CommConstants.LoadSweepMinWindowMs} y {CommConstants.LoadSweepMaxWindowMs} ms.";
        }
        if (request.SettleMs > CommConstants.LoadSweepMaxSettleMs)
        {
            return $"El asentamiento no puede superar {CommConstants.LoadSweepMaxSettleMs} ms.";
        }

        int span = Math.Abs(request.EndRpm - request.StartRpm);
        if (request.Mode == LoadSweepMode.Staircase)
        {
            if (request.StepOrRate < CommConstants.LoadSweepMinStepRpm)
            {
                return $"El escalon debe ser de al menos {CommConstants.LoadSweepMinStepRpm} rpm.";
            }
            int steps = (span + request.StepOrRate - 1) / request.StepOrRate + 1;
            if (steps > 128)
            {
                return $"La escalera tendria {steps} escalones; el maximo es 128.";
            }
        }
        else
        {
            if (span == 0)
            {
                return "La rampa necesita rpm inicial y final distintas.";
            }
            if (request.StepOrRate is < 1 or > CommConstants.LoadSweepMaxRampRpmPerSecond)
            {
                return $"La pendiente debe estar entre 1 y {CommConstants.LoadSweepMaxRampRpmPerSecond} rpm/s.";
            }
            long points = span * 1000L / request.StepOrRate / request.WindowMs + 1;
            if (points > 128)
            {
                return $"La rampa generaria ~{points} puntos; el maximo es 128 (ampliar la ventana o la pendiente).";
            }
        }

        return null;
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

    public Task<CommandResult> StartSpeedLogAsync(CancellationToken cancellationToken = default) =>
        StartLogAsync(LogParam.Speed, cancellationToken);

    public async Task<CommandResult> StartLogAsync(LogParam parameter, CancellationToken cancellationToken = default)
    {
        if (parameter == LogParam.All)
        {
            return CommandResult.Failed("Use un canal de telemetria concreto.");
        }

        CommandResult result = await SendCommandAsync(
            CommOpcode.LogStart, (byte)parameter, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
        if (result.Success)
        {
            lock (_stateGate)
            {
                _enabledLogs.Add(parameter);
            }
            Notify();
        }

        return result;
    }

    public Task<CommandResult> StopSpeedLogAsync(CancellationToken cancellationToken = default) =>
        StopLogAsync(LogParam.Speed, cancellationToken);

    public async Task<CommandResult> StopLogAsync(LogParam parameter, CancellationToken cancellationToken = default)
    {
        if (parameter == LogParam.All)
        {
            return CommandResult.Failed("Use un canal de telemetria concreto.");
        }

        CommandResult result = await SendCommandAsync(
            CommOpcode.LogStop, (byte)parameter, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
        if (result.Success)
        {
            lock (_stateGate)
            {
                _enabledLogs.Remove(parameter);
            }
            Notify();
        }

        return result;
    }

    public async Task RefreshStatusAsync(
        CancellationToken cancellationToken = default,
        int responseDeadlineMs = 1_000)
    {
        if (!_transport.IsOpen)
        {
            return;
        }

        try
        {
            EscFrame frame = await SendRequestAsync(
                CommOpcode.GetStatus,
                0,
                Array.Empty<byte>(),
                cancellationToken,
                responseDeadlineMs).ConfigureAwait(false);
            EscStatus status = EscProtocol.DecodeStatus(frame);
            lock (_stateGate)
            {
                _status = status;
                SynchronizeManualOutputs(status);
                _lastError = null;
                _state = DeviceConnectionState.Connected;
            }

            Notify();
        }
        catch (OperationCanceledException) when (IsEmergencyStopPending())
        {
            return;
        }
        catch (Exception ex)
        {
            _logger.LogDebug(ex, "Status refresh failed.");
            SetError(_transport.IsOpen ? DeviceConnectionState.Connected : DeviceConnectionState.Error, ex.Message);
        }
    }

    public async Task ReadTelemetryOnceAsync(CancellationToken cancellationToken = default)
    {
        bool loggingEnabled;
        lock (_stateGate)
        {
            loggingEnabled = _enabledLogs.Count > 0;
        }
        if (!_transport.IsOpen || !loggingEnabled)
        {
            return;
        }

        if (!await _ioLock.WaitAsync(0, cancellationToken).ConfigureAwait(false))
        {
            return;
        }

        try
        {
            EscFrame? frame = await _transport.ReadFrameAsync(TimeSpan.FromMilliseconds(5), cancellationToken).ConfigureAwait(false);
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

        return await SendMotorCommandWithEstopGuardAsync(
            opcode, cancellationToken).ConfigureAwait(false);
    }

    private async Task<CommandResult> StopSineDriveCoreAsync(
        bool emergency,
        bool refreshStatus,
        CancellationToken cancellationToken)
    {
        CommOpcode opcode = emergency ? CommOpcode.EmergencyStop : CommOpcode.Stop;
        CommandResult result = await SendCommandAsync(
            opcode, 0, Array.Empty<byte>(), cancellationToken, refreshStatus).ConfigureAwait(false);
        if (!result.Success) return result;

        lock (_stateGate)
        {
            ClearManualOutputs();
            if (_status is not null)
            {
                _status = _status with
                {
                    AppState = 0,
                    AppStateName = "IDLE",
                    MotorStalled = false,
                    ConsistentZeroCrossing = false
                };
            }
        }
        Notify();
        return result;
    }

    private async Task<CommandResult> SendMotorCommandWithEstopGuardAsync(
        CommOpcode opcode,
        CancellationToken cancellationToken)
    {
        int emergencyGeneration = Volatile.Read(ref _emergencyStopGeneration);
        if (IsEmergencyStopPending()) return CommandResult.Failed("ESTOP en curso.");

        try
        {
            EscFrame response;
            await _ioLock.WaitAsync(cancellationToken).ConfigureAwait(false);
            try
            {
                if (IsEmergencyStopPending() ||
                    emergencyGeneration != Volatile.Read(ref _emergencyStopGeneration))
                {
                    return CommandResult.Failed("Comando cancelado por ESTOP.");
                }
                response = await SendRequestCoreAsync(
                    opcode, 0, Array.Empty<byte>(), cancellationToken).ConfigureAwait(false);
            }
            finally
            {
                _ioLock.Release();
            }

            CommandResult result = opcode == CommOpcode.Run && response.Status == CommStatus.InvalidState
                ? new CommandResult(false, response.Status, "RUN no permitido en el estado actual del ESC.")
                : CommandResult.FromStatus(response.Status);
            await RefreshStatusAfterCommandAsync(cancellationToken).ConfigureAwait(false);
            return result;
        }
        catch (OperationCanceledException) when (IsEmergencyStopPending())
        {
            return CommandResult.Failed("Comando interrumpido por ESTOP.");
        }
        catch (Exception ex)
        {
            SetError(DeviceConnectionState.Error, ex.Message);
            return CommandResult.Failed(ex.Message);
        }
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
            if (IsEmergencyStopPending())
            {
                return CommandResult.Failed("Comando interrumpido por ESTOP.");
            }
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
                SynchronizeManualOutputs(status);
            }
        }
        catch (Exception ex)
        {
            _logger.LogDebug(ex, "Post-command status refresh failed.");
        }

        Notify();
    }

    private async Task<EscFrame> SendRequestAsync(
        CommOpcode opcode,
        byte parameter,
        byte[] payload,
        CancellationToken cancellationToken,
        int responseDeadlineMs = 1_000)
    {
        if (!_transport.IsOpen)
        {
            throw new InvalidOperationException("ESC is not connected.");
        }

        await _ioLock.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            return await SendRequestCoreAsync(opcode, parameter, payload, cancellationToken, responseDeadlineMs).ConfigureAwait(false);
        }
        finally
        {
            _ioLock.Release();
        }
    }

    private async Task<EscFrame> SendRequestCoreAsync(
        CommOpcode opcode,
        byte parameter,
        byte[] payload,
        CancellationToken cancellationToken,
        int responseDeadlineMs = 1_000)
    {
        if (opcode != CommOpcode.EmergencyStop && IsEmergencyStopPending())
        {
            throw new OperationCanceledException("Request interrupted by ESTOP.");
        }
        byte sequence = unchecked(++_sequence);
        byte[] request = EscProtocol.BuildRequest(sequence, opcode, parameter, payload);

        await _transport.WriteFrameAsync(request, cancellationToken).ConfigureAwait(false);

        DateTimeOffset deadline = DateTimeOffset.UtcNow.AddMilliseconds(responseDeadlineMs);
        while (DateTimeOffset.UtcNow < deadline)
        {
            if (opcode != CommOpcode.EmergencyStop && IsEmergencyStopPending())
            {
                throw new OperationCanceledException("Request interrupted by ESTOP.");
            }
            TimeSpan remaining = deadline - DateTimeOffset.UtcNow;
            TimeSpan readSlice = remaining > TimeSpan.FromMilliseconds(50)
                ? TimeSpan.FromMilliseconds(50)
                : remaining;
            EscFrame? frame = await _transport.ReadFrameAsync(readSlice, cancellationToken).ConfigureAwait(false);
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

    private async Task<ActiveControllerConfig> ReadControllerConfigCoreAsync(CancellationToken cancellationToken)
    {
        if (!_transport.IsOpen)
        {
            throw new InvalidOperationException("ESC is not connected.");
        }

        double kp = await ReadConfigCoreAsync(ConfigParam.KpRpm, cancellationToken).ConfigureAwait(false);
        double ki = await ReadConfigCoreAsync(ConfigParam.KiRpm, cancellationToken).ConfigureAwait(false);
        double kd = await ReadConfigCoreAsync(ConfigParam.KdRpm, cancellationToken).ConfigureAwait(false);
        double polePairs = await ReadConfigCoreAsync(ConfigParam.PolePairs, cancellationToken).ConfigureAwait(false);
        double pwmFrequency = await ReadConfigCoreAsync(ConfigParam.PwmFreq, cancellationToken).ConfigureAwait(false);
        double speedRamp = await ReadConfigCoreAsync(ConfigParam.SpeedRamp, cancellationToken).ConfigureAwait(false);
        return new ActiveControllerConfig(
            kp,
            ki,
            kd,
            checked((byte)polePairs),
            checked((ushort)pwmFrequency),
            checked((ushort)speedRamp));
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

    public async Task<StartupConfiguration> RefreshStartupConfigurationAsync(
        CancellationToken cancellationToken = default)
    {
        await _ioLock.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            StartupConfiguration configuration =
                await ReadStartupConfigurationCoreAsync(cancellationToken).ConfigureAwait(false);
            lock (_stateGate)
            {
                _startupConfiguration = configuration;
            }
            Notify();
            return configuration;
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

    private async Task<StartupConfiguration> ReadStartupConfigurationCoreAsync(CancellationToken cancellationToken)
    {
        double initialAmplitude = await ReadConfigCoreAsync(
            ConfigParam.StartupInitialAmplitude, cancellationToken).ConfigureAwait(false);
        double finalAmplitude = await ReadConfigCoreAsync(
            ConfigParam.StartupFinalAmplitude, cancellationToken).ConfigureAwait(false);
        double initialFrequency = await ReadConfigCoreAsync(
            ConfigParam.StartupInitialFrequency, cancellationToken).ConfigureAwait(false);
        double finalFrequency = await ReadConfigCoreAsync(
            ConfigParam.StartupFinalFrequency, cancellationToken).ConfigureAwait(false);
        double duration = await ReadConfigCoreAsync(
            ConfigParam.StartupDuration, cancellationToken).ConfigureAwait(false);
        return new StartupConfiguration(
            initialAmplitude,
            finalAmplitude,
            initialFrequency,
            finalFrequency,
            duration);
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
            _transport.IsOpen,
            _mode,
            _devices,
            _currentDevice,
            _status,
            _lastError,
            DateTimeOffset.UtcNow,
            _enabledLogs.Contains(LogParam.Speed),
            _logRateMs,
            _telemetryStore.GetStats("speed", "rpm"),
            _activeControllerConfig,
            _activeSpeedLimits,
            _startupConfiguration,
            new SineDriveStatus(
                _status?.AppState == SineDriveAppState,
                _requestedSineDrive,
                _appliedSineDrive,
                _sineDriveLastAppliedAt),
            new StepTestStatus(
                _status?.AppState == StepTestAppState,
                _requestedStepTest,
                _appliedStepTest,
                _stepTestLastAppliedAt),
            new FreewheelStatus(
                _status?.AppState == FreewheelAppState,
                _freewheelReport,
                _activeZcMap,
                _lastZcAutoDetect,
                _freewheelLastUpdatedAt),
            new LoadSweepSnapshot(
                _loadSweepStatus,
                _loadSweepPoints.ToArray(),
                _loadSweepUpdatedAt,
                _loadSweepError)
            {
                SavedCurves = _savedCurves.ToArray(),
                TareCurveId = _tareCurveId,
            })
        {
            BusVoltageLoggingEnabled = _enabledLogs.Contains(LogParam.BusVoltage),
            BusCurrentLoggingEnabled = _enabledLogs.Contains(LogParam.BusCurrent),
            DutyLoggingEnabled = _enabledLogs.Contains(LogParam.Duty),
            BusVoltageTelemetry = _telemetryStore.GetStats("bus_voltage", "mV"),
            BusCurrentTelemetry = _telemetryStore.GetStats("bus_current", "mA"),
            DutyTelemetry = _telemetryStore.GetStats("duty", "%"),
        };
    }

    private void ClearDeviceCache()
    {
        _activeControllerConfig = null;
        _activeSpeedLimits = null;
        _startupConfiguration = null;
        _freewheelReport = null;
        _activeZcMap = null;
        _lastZcAutoDetect = null;
        _freewheelLastUpdatedAt = null;
    }

    private void SynchronizeManualOutputs(EscStatus status)
    {
        if (status.AppState != SineDriveAppState)
        {
            ClearSineDrive();
        }
        if (status.AppState != StepTestAppState)
        {
            ClearStepTest();
        }
    }

    private void ClearManualOutputs()
    {
        ClearSineDrive();
        ClearStepTest();
    }

    private void ClearSineDrive()
    {
        _requestedSineDrive = null;
        _appliedSineDrive = null;
        _sineDriveLastAppliedAt = null;
    }

    private void ClearStepTest()
    {
        _requestedStepTest = null;
        _appliedStepTest = null;
        _stepTestLastAppliedAt = null;
    }

    private bool HasRequestedManualOutput()
    {
        lock (_stateGate)
        {
            return _requestedSineDrive is not null || _requestedStepTest is not null;
        }
    }

    private bool IsEmergencyStopPending() => Volatile.Read(ref _emergencyStopPending) != 0;

    private static bool IsControllerParameter(ConfigParam parameter)
    {
        return parameter is ConfigParam.KpRpm
            or ConfigParam.KiRpm
            or ConfigParam.KdRpm
            or ConfigParam.PolePairs
            or ConfigParam.PwmFreq
            or ConfigParam.SpeedRamp;
    }

    private static bool IsStartupParameter(ConfigParam parameter)
    {
        return parameter is ConfigParam.StartupInitialAmplitude
            or ConfigParam.StartupFinalAmplitude
            or ConfigParam.StartupInitialFrequency
            or ConfigParam.StartupFinalFrequency
            or ConfigParam.StartupDuration;
    }

    private void Notify()
    {
        SnapshotChanged?.Invoke(this, EventArgs.Empty);
    }
}
