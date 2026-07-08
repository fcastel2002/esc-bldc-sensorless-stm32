using System.Diagnostics;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Esc.Protocol;
using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;

namespace Esc.Bridge;

public sealed class HilUdpBridgeService : BackgroundService
{
    public const int DefaultPort = 5055;
    private const int MaxDrainPackets = 256;
    private const double HilOutputPollPeriodMs = 100;
    private const double ActiveSenderLeaseMs = 250;

    private readonly EscBridgeService _bridge;
    private readonly ILogger<HilUdpBridgeService> _logger;
    private readonly HilUdpSenderSession _senderSession = new(ActiveSenderLeaseMs);
    private readonly Stopwatch _uptime = Stopwatch.StartNew();
    private uint _rxFrames;
    private uint _lostFrames;
    private uint? _lastSequence;
    private double _averageRoundTripMs;
    private double _jitterMs;
    private double? _lastArrivalMs;
    private double? _lastPeriodMs;
    private double _lastHilOutputPollMs = double.NegativeInfinity;

    public HilUdpBridgeService(EscBridgeService bridge, ILogger<HilUdpBridgeService> logger)
    {
        _bridge = bridge;
        _logger = logger;
    }

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        using var udp = new UdpClient(new IPEndPoint(IPAddress.Loopback, DefaultPort));
        _logger.LogInformation("HIL UDP bridge listening on 127.0.0.1:{Port}.", DefaultPort);

        while (!stoppingToken.IsCancellationRequested)
        {
            try
            {
                UdpReceiveResult received = await udp.ReceiveAsync(stoppingToken).ConfigureAwait(false);
                received = await ReceiveLatestPendingPacketAsync(udp, received, stoppingToken).ConfigureAwait(false);
                string requestText = DecodePacket(received.Buffer);
                _bridge.RecordHilFrame(
                    "Simulink -> Bridge",
                    "UDP",
                    TryReadSequence(requestText),
                    ClassifyPacket(received.Buffer).ToString(),
                    requestText);

                string response = await HandlePacketAsync(received.Buffer, received.RemoteEndPoint, stoppingToken).ConfigureAwait(false);
                _bridge.RecordHilFrame(
                    "Bridge -> Simulink",
                    "UDP",
                    TryReadSequence(response),
                    response.StartsWith("ok,", StringComparison.OrdinalIgnoreCase) ? "ok" : "err",
                    response);

                byte[] bytes = Encoding.ASCII.GetBytes(response);
                await udp.SendAsync(bytes, received.RemoteEndPoint, stoppingToken).ConfigureAwait(false);
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
                return;
            }
            catch (Exception ex)
            {
                _logger.LogDebug(ex, "HIL UDP iteration failed.");
            }
        }
    }

    private async Task<UdpReceiveResult> ReceiveLatestPendingPacketAsync(
        UdpClient udp,
        UdpReceiveResult selected,
        CancellationToken cancellationToken)
    {
        string selectedText = DecodePacket(selected.Buffer);
        PacketKind selectedKind = ClassifyPacket(selected.Buffer);
        if (selectedKind == PacketKind.EmergencyStop)
        {
            return selected;
        }

        bool selectedAllowed = CanAcceptSender(selected.RemoteEndPoint, selectedKind, selectedText);
        IPEndPoint? preferredSender = WouldSenderTakeOver(selected.RemoteEndPoint, selectedKind, selectedText)
            ? CloneEndPoint(selected.RemoteEndPoint)
            : null;
        uint? selectedSequence = TryReadSequence(selectedText);

        for (int drained = 0; udp.Available > 0 && drained < MaxDrainPackets; drained++)
        {
            UdpReceiveResult next = await udp.ReceiveAsync(cancellationToken).ConfigureAwait(false);
            string nextText = DecodePacket(next.Buffer);
            PacketKind nextKind = ClassifyPacket(next.Buffer);

            if (nextKind == PacketKind.EmergencyStop)
            {
                return next;
            }

            if (nextKind == PacketKind.Invalid)
            {
                continue;
            }

            if (preferredSender is not null && !EndpointsEqual(next.RemoteEndPoint, preferredSender))
            {
                continue;
            }

            bool nextAllowed = CanAcceptSender(next.RemoteEndPoint, nextKind, nextText);
            if (!nextAllowed)
            {
                continue;
            }

            uint? nextSequence = TryReadSequence(nextText);
            if (selectedAllowed &&
                IsCoalescible(selectedKind) &&
                IsCoalescible(nextKind) &&
                !EndpointsEqual(selected.RemoteEndPoint, next.RemoteEndPoint) &&
                selectedSequence.HasValue &&
                nextSequence.HasValue)
            {
                if (nextSequence.Value < selectedSequence.Value)
                {
                    selected = next;
                    selectedText = nextText;
                    selectedKind = nextKind;
                    selectedAllowed = true;
                    selectedSequence = nextSequence;
                    preferredSender = CloneEndPoint(next.RemoteEndPoint);
                    continue;
                }

                preferredSender ??= CloneEndPoint(selected.RemoteEndPoint);
                continue;
            }

            if (WouldSenderTakeOver(next.RemoteEndPoint, nextKind, nextText))
            {
                selected = next;
                selectedText = nextText;
                selectedKind = nextKind;
                selectedAllowed = true;
                selectedSequence = nextSequence;
                preferredSender = CloneEndPoint(next.RemoteEndPoint);
                continue;
            }

            if (!selectedAllowed ||
                selectedKind == PacketKind.Invalid ||
                nextKind == PacketKind.Discrete ||
                IsCoalescible(nextKind) && IsCoalescible(selectedKind))
            {
                selected = next;
                selectedText = nextText;
                selectedKind = nextKind;
                selectedAllowed = true;
                selectedSequence = nextSequence;
            }
        }

        return selected;
    }

    private static bool IsCoalescible(PacketKind kind)
    {
        return kind is PacketKind.Setpoint or PacketKind.HilInputs;
    }

    private static PacketKind ClassifyPacket(byte[] buffer)
    {
        string text = DecodePacket(buffer);
        if (string.IsNullOrWhiteSpace(text))
        {
            return PacketKind.Invalid;
        }

        string command = text.Split(',', StringSplitOptions.TrimEntries)[0].ToUpperInvariant();
        return command switch
        {
            "ESTOP" => PacketKind.EmergencyStop,
            "RUN" or "MOTOR_RUN" or "MOTOR_STOP" or "REAL_STOP" or "HIL_START" or "START" or "HIL_STOP" or "STOP" => PacketKind.Discrete,
            "SETPOINT" or "SP" or "SPEED" => PacketKind.Setpoint,
            _ => TryParseInputs(text, out _, out _, out _) ? PacketKind.HilInputs : PacketKind.Invalid
        };
    }

    private async Task<string> HandlePacketAsync(
        byte[] buffer,
        IPEndPoint remoteEndPoint,
        CancellationToken cancellationToken)
    {
        string text = DecodePacket(buffer);
        if (string.IsNullOrWhiteSpace(text))
        {
            return "err,empty packet";
        }

        string[] commandParts = text.Split(',', StringSplitOptions.TrimEntries);
        string command = commandParts[0].ToUpperInvariant();
        uint? requestSequence = TryReadSequence(text);
        double requestNowMs = _uptime.Elapsed.TotalMilliseconds;

        if (!CanAcceptSender(remoteEndPoint, ClassifyPacket(buffer), text))
        {
            return $"err,active PIL sender {_senderSession.DescribeActiveSender(requestNowMs)}";
        }

        if (command is "SETPOINT" or "SP" or "SPEED")
        {
            string response = await HandleSetpointCommandAsync(commandParts, cancellationToken).ConfigureAwait(false);
            if (response.StartsWith("ok,", StringComparison.OrdinalIgnoreCase))
            {
                _senderSession.Accept(remoteEndPoint, requestSequence, requestNowMs);
            }

            return response;
        }

        if (command is "RUN" or "MOTOR_RUN")
        {
            CommandResult run = await _bridge.RunFromSimulinkAsync(cancellationToken).ConfigureAwait(false);
            if (run.Success)
            {
                _senderSession.Accept(remoteEndPoint, requestSequence, requestNowMs);
            }

            return BuildCommandResponse("run", 0, run);
        }

        if (command is "MOTOR_STOP" or "REAL_STOP")
        {
            CommandResult stopped = await _bridge.StopFromSimulinkAsync(cancellationToken).ConfigureAwait(false);
            if (stopped.Success)
            {
                _senderSession.Release(remoteEndPoint);
            }

            return BuildCommandResponse("stop", 0, stopped);
        }

        if (command == "ESTOP")
        {
            CommandResult stopped = await _bridge.EmergencyStopAsync(cancellationToken).ConfigureAwait(false);
            if (stopped.Success)
            {
                _senderSession.Release();
            }

            return BuildCommandResponse("estop", 0, stopped);
        }

        if (command is "HIL_START" or "START")
        {
            CommandResult started = await _bridge.HilStartAsync(cancellationToken).ConfigureAwait(false);
            if (started.Success)
            {
                _senderSession.Accept(remoteEndPoint, requestSequence, requestNowMs);
            }

            return started.Success ? "ok,start" : $"err,{started.Message}";
        }

        if (command is "HIL_STOP" or "STOP")
        {
            CommandResult stopped = await _bridge.HilStopAsync(cancellationToken).ConfigureAwait(false);
            if (stopped.Success)
            {
                _senderSession.Release(remoteEndPoint);
            }

            return stopped.Success ? "ok,stop" : $"err,{stopped.Message}";
        }

        if (!TryParseInputs(text, out uint sequence, out HilInputs inputs, out string? error))
        {
            return $"err,{error}";
        }

        TrackReceive(sequence);

        Stopwatch roundTrip = Stopwatch.StartNew();
        CommandResult result = await _bridge.HilSetInputsAsync(inputs, cancellationToken).ConfigureAwait(false);
        if (!result.Success)
        {
            return $"err,{result.Message}";
        }

        if (inputs.Enable)
        {
            _senderSession.Accept(remoteEndPoint, sequence, requestNowMs);
        }
        else
        {
            _senderSession.Release(remoteEndPoint);
        }

        HilOutputs? outputs = null;
        HilBridgeStats currentStats = _bridge.Snapshot.Hil;
        double nowMs = _uptime.Elapsed.TotalMilliseconds;
        if (currentStats.LastOutputs is null || nowMs - _lastHilOutputPollMs >= HilOutputPollPeriodMs)
        {
            outputs = await _bridge.HilGetOutputsAsync(cancellationToken).ConfigureAwait(false);
            _lastHilOutputPollMs = _uptime.Elapsed.TotalMilliseconds;
        }

        roundTrip.Stop();
        TrackRoundTrip(roundTrip.Elapsed.TotalMilliseconds, outputs, inputs, inputs.Enable);

        HilBridgeStats stats = _bridge.Snapshot.Hil;
        outputs ??= stats.LastOutputs;
        if (outputs is null)
        {
            return BuildHilInputResponse(sequence, inputs, stats);
        }

        return string.Create(
            CultureInfo.InvariantCulture,
            $"ok,{sequence},{outputs.TargetTickMs},{outputs.AppState},{(byte)outputs.Mode},{outputs.SetpointRpm},{outputs.MeasuredRpm},{outputs.PwmCommand},{outputs.CommutationStep},{outputs.Flags},{(outputs.TimedOut ? 1 : 0)},{stats.RxFrames},{stats.LostFrames},{stats.EffectiveRateHz:0.###},{stats.AverageRoundTripMs:0.###},{stats.JitterMs:0.###}");
    }

    private async Task<string> HandleSetpointCommandAsync(string[] parts, CancellationToken cancellationToken)
    {
        if (parts.Length is not (2 or 3))
        {
            return "err,expected SETPOINT,rpm or SETPOINT,seq,rpm";
        }

        int rpmIndex = parts.Length == 3 ? 2 : 1;
        uint sequence = 0;
        if (parts.Length == 3 && !uint.TryParse(parts[1], NumberStyles.Integer, CultureInfo.InvariantCulture, out sequence))
        {
            return "err,invalid sequence";
        }

        if (!int.TryParse(parts[rpmIndex], NumberStyles.Integer, CultureInfo.InvariantCulture, out int rpm))
        {
            return "err,invalid rpm";
        }

        CommandResult result = await _bridge.SetSpeedRpmFromSimulinkAsync(rpm, refreshStatus: false, cancellationToken: cancellationToken).ConfigureAwait(false);
        return BuildCommandResponse("setpoint", sequence, result);
    }

    private string BuildCommandResponse(string command, uint sequence, CommandResult result)
    {
        if (!result.Success)
        {
            return $"err,{command},{sequence},{result.Message}";
        }

        EscStatus? status = _bridge.Snapshot.Status;
        return string.Create(
            CultureInfo.InvariantCulture,
            $"ok,{command},{sequence},{status?.SpeedSetpointRpm ?? 0},{status?.ActualSpeedRpm ?? 0},{status?.AppState ?? 0}");
    }

    private string BuildHilInputResponse(uint sequence, HilInputs inputs, HilBridgeStats stats)
    {
        EscStatus? status = _bridge.Snapshot.Status;
        byte mode = inputs.Enable ? (byte)ControlRuntimeMode.HilSim : (byte)ControlRuntimeMode.Normal;
        return string.Create(
            CultureInfo.InvariantCulture,
            $"ok,{sequence},0,{status?.AppState ?? 0},{mode},{status?.SpeedSetpointRpm ?? 0},{inputs.SpeedRpm},0,0,{inputs.Flags},0,{stats.RxFrames},{stats.LostFrames},{stats.EffectiveRateHz:0.###},{stats.AverageRoundTripMs:0.###},{stats.JitterMs:0.###}");
    }

    private static string DecodePacket(byte[] buffer)
    {
        return Encoding.ASCII.GetString(buffer).Trim('\0', '\r', '\n', ' ');
    }

    private static bool TryParseInputs(string text, out uint sequence, out HilInputs inputs, out string? error)
    {
        sequence = 0;
        inputs = new HilInputs(0, 0, 0, false);
        error = null;

        string[] parts = text.Split(',', StringSplitOptions.TrimEntries);
        if (parts.Length == 0)
        {
            error = "empty packet";
            return false;
        }

        string command = parts[0].ToUpperInvariant();
        int offset = command is "PIL" or "HIL" ? 1 : 0;
        int fieldCount = parts.Length - offset;

        if (fieldCount is 2 or 3)
        {
            bool hasSequence = fieldCount == 3;
            if (hasSequence && !uint.TryParse(parts[offset], NumberStyles.Integer, CultureInfo.InvariantCulture, out sequence))
            {
                error = "invalid sequence";
                return false;
            }

            int firstValue = offset + (hasSequence ? 1 : 0);
            if (!ushort.TryParse(parts[firstValue], NumberStyles.Integer, CultureInfo.InvariantCulture, out ushort speedRpm) ||
                !byte.TryParse(parts[firstValue + 1], NumberStyles.Integer, CultureInfo.InvariantCulture, out byte enable))
            {
                error = "invalid numeric value";
                return false;
            }

            inputs = new HilInputs(speedRpm, 0, 0, enable != 0);
            return true;
        }

        if (fieldCount is not (5 or 6))
        {
            error = "expected CSV: seq,speed_rpm,enable or seq,speed_rpm,reserved,load_torque,flags,enable";
            return false;
        }

        bool legacyHasSequence = fieldCount == 6;
        if (legacyHasSequence && !uint.TryParse(parts[offset], NumberStyles.Integer, CultureInfo.InvariantCulture, out sequence))
        {
            error = "invalid sequence";
            return false;
        }

        int legacyFirstValue = offset + (legacyHasSequence ? 1 : 0);
        if (!ushort.TryParse(parts[legacyFirstValue], NumberStyles.Integer, CultureInfo.InvariantCulture, out ushort legacySpeedRpm) ||
            !ushort.TryParse(parts[legacyFirstValue + 1], NumberStyles.Integer, CultureInfo.InvariantCulture, out _) ||
            !short.TryParse(parts[legacyFirstValue + 2], NumberStyles.Integer, CultureInfo.InvariantCulture, out short loadTorque) ||
            !byte.TryParse(parts[legacyFirstValue + 3], NumberStyles.Integer, CultureInfo.InvariantCulture, out byte flags) ||
            !byte.TryParse(parts[legacyFirstValue + 4], NumberStyles.Integer, CultureInfo.InvariantCulture, out byte legacyEnable))
        {
            error = "invalid numeric value";
            return false;
        }

        inputs = new HilInputs(legacySpeedRpm, loadTorque, flags, legacyEnable != 0);
        return true;
    }

    private static uint? TryReadSequence(string text)
    {
        if (TryParseInputs(text, out uint sequence, out _, out _))
        {
            return sequence;
        }

        string[] parts = text.Split(',', StringSplitOptions.TrimEntries);
        if (parts.Length >= 2 && uint.TryParse(parts[1], NumberStyles.Integer, CultureInfo.InvariantCulture, out sequence))
        {
            return sequence;
        }

        if (parts.Length >= 3 && uint.TryParse(parts[2], NumberStyles.Integer, CultureInfo.InvariantCulture, out sequence))
        {
            return sequence;
        }

        return null;
    }

    private bool CanAcceptSender(IPEndPoint sender, PacketKind kind, string text)
    {
        string command = ReadCommand(text);
        bool isStartCommand = command is "HIL_START" or "START";
        bool isEmergencyStop = kind == PacketKind.EmergencyStop;
        return _senderSession.CanAccept(
            sender,
            TryReadSequence(text),
            isStartCommand,
            isEmergencyStop,
            _uptime.Elapsed.TotalMilliseconds);
    }

    private bool WouldSenderTakeOver(IPEndPoint sender, PacketKind kind, string text)
    {
        string command = ReadCommand(text);
        bool isStartCommand = command is "HIL_START" or "START";
        return _senderSession.WouldTakeOver(
            sender,
            TryReadSequence(text),
            isStartCommand,
            _uptime.Elapsed.TotalMilliseconds);
    }

    private static string ReadCommand(string text)
    {
        string[] parts = text.Split(',', StringSplitOptions.TrimEntries);
        return parts.Length == 0 ? string.Empty : parts[0].ToUpperInvariant();
    }

    private static IPEndPoint CloneEndPoint(IPEndPoint endPoint)
    {
        return new IPEndPoint(endPoint.Address, endPoint.Port);
    }

    private static bool EndpointsEqual(IPEndPoint left, IPEndPoint right)
    {
        return left.Address.Equals(right.Address) && left.Port == right.Port;
    }

    private void TrackReceive(uint sequence)
    {
        _rxFrames++;
        if (_lastSequence is uint last && sequence > last + 1)
        {
            _lostFrames += sequence - last - 1;
        }
        _lastSequence = sequence;

        double nowMs = _uptime.Elapsed.TotalMilliseconds;
        if (_lastArrivalMs is double lastArrival)
        {
            double period = nowMs - lastArrival;
            if (_lastPeriodMs is double lastPeriod)
            {
                _jitterMs = _jitterMs == 0 ? Math.Abs(period - lastPeriod) : (_jitterMs * 0.9) + (Math.Abs(period - lastPeriod) * 0.1);
            }
            _lastPeriodMs = period;
        }
        _lastArrivalMs = nowMs;
    }

    private void TrackRoundTrip(double roundTripMs, HilOutputs? outputs, HilInputs inputs, bool hilEnabled)
    {
        _averageRoundTripMs = _averageRoundTripMs == 0 ? roundTripMs : (_averageRoundTripMs * 0.9) + (roundTripMs * 0.1);
        double effectiveRate = _uptime.Elapsed.TotalSeconds <= 0 ? 0 : _rxFrames / _uptime.Elapsed.TotalSeconds;
        HilOutputs? lastOutputs = outputs ?? _bridge.Snapshot.Hil.LastOutputs;
        HilInputs lastInputs = inputs;
        _bridge.UpdateHilStats(new HilBridgeStats(
            outputs?.Mode == ControlRuntimeMode.HilSim || (outputs is null && hilEnabled),
            _rxFrames,
            _lostFrames,
            effectiveRate,
            _averageRoundTripMs,
            _jitterMs,
            lastOutputs,
            lastInputs));
    }

    private enum PacketKind
    {
        Invalid,
        Discrete,
        EmergencyStop,
        Setpoint,
        HilInputs,
    }
}
