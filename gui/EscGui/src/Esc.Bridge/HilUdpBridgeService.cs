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

    private readonly EscBridgeService _bridge;
    private readonly ILogger<HilUdpBridgeService> _logger;
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
                string response = await HandlePacketAsync(received.Buffer, stoppingToken).ConfigureAwait(false);
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

    private static async Task<UdpReceiveResult> ReceiveLatestPendingPacketAsync(
        UdpClient udp,
        UdpReceiveResult selected,
        CancellationToken cancellationToken)
    {
        PacketKind selectedKind = ClassifyPacket(selected.Buffer);

        for (int drained = 0; udp.Available > 0 && drained < MaxDrainPackets; drained++)
        {
            UdpReceiveResult next = await udp.ReceiveAsync(cancellationToken).ConfigureAwait(false);
            PacketKind nextKind = ClassifyPacket(next.Buffer);

            if (nextKind == PacketKind.EmergencyStop)
            {
                return next;
            }

            if (selectedKind == PacketKind.EmergencyStop)
            {
                continue;
            }

            if (nextKind == PacketKind.Invalid)
            {
                continue;
            }

            if (selectedKind == PacketKind.Invalid ||
                nextKind == PacketKind.Discrete ||
                IsCoalescible(nextKind) && IsCoalescible(selectedKind))
            {
                selected = next;
                selectedKind = nextKind;
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

    private async Task<string> HandlePacketAsync(byte[] buffer, CancellationToken cancellationToken)
    {
        string text = DecodePacket(buffer);
        if (string.IsNullOrWhiteSpace(text))
        {
            return "err,empty packet";
        }

        string[] commandParts = text.Split(',', StringSplitOptions.TrimEntries);
        string command = commandParts[0].ToUpperInvariant();

        if (command is "SETPOINT" or "SP" or "SPEED")
        {
            return await HandleSetpointCommandAsync(commandParts, cancellationToken).ConfigureAwait(false);
        }

        if (command is "RUN" or "MOTOR_RUN")
        {
            CommandResult run = await _bridge.RunFromSimulinkAsync(cancellationToken).ConfigureAwait(false);
            return BuildCommandResponse("run", 0, run);
        }

        if (command is "MOTOR_STOP" or "REAL_STOP")
        {
            CommandResult stopped = await _bridge.StopFromSimulinkAsync(cancellationToken).ConfigureAwait(false);
            return BuildCommandResponse("stop", 0, stopped);
        }

        if (command == "ESTOP")
        {
            CommandResult stopped = await _bridge.EmergencyStopAsync(cancellationToken).ConfigureAwait(false);
            return BuildCommandResponse("estop", 0, stopped);
        }

        if (command is "HIL_START" or "START")
        {
            CommandResult started = await _bridge.HilStartAsync(cancellationToken).ConfigureAwait(false);
            return started.Success ? "ok,start" : $"err,{started.Message}";
        }

        if (command is "HIL_STOP" or "STOP")
        {
            CommandResult stopped = await _bridge.HilStopAsync(cancellationToken).ConfigureAwait(false);
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

        HilOutputs? outputs = null;
        HilBridgeStats currentStats = _bridge.Snapshot.Hil;
        double nowMs = _uptime.Elapsed.TotalMilliseconds;
        if (currentStats.LastOutputs is null || nowMs - _lastHilOutputPollMs >= HilOutputPollPeriodMs)
        {
            outputs = await _bridge.HilGetOutputsAsync(cancellationToken).ConfigureAwait(false);
            _lastHilOutputPollMs = _uptime.Elapsed.TotalMilliseconds;
        }

        roundTrip.Stop();
        TrackRoundTrip(roundTrip.Elapsed.TotalMilliseconds, outputs, inputs.Enable);

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
        inputs = new HilInputs(0, 0, 0, 0, false);
        error = null;

        string[] parts = text.Split(',', StringSplitOptions.TrimEntries);
        if (parts.Length is not (5 or 6))
        {
            error = "expected CSV: seq,speed_rpm,zero_crossing_period,load_torque,flags,enable";
            return false;
        }

        int offset = parts.Length == 6 ? 1 : 0;
        if (parts.Length == 6 && !uint.TryParse(parts[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out sequence))
        {
            error = "invalid sequence";
            return false;
        }

        if (!ushort.TryParse(parts[offset], NumberStyles.Integer, CultureInfo.InvariantCulture, out ushort speedRpm) ||
            !ushort.TryParse(parts[offset + 1], NumberStyles.Integer, CultureInfo.InvariantCulture, out ushort zeroPeriod) ||
            !short.TryParse(parts[offset + 2], NumberStyles.Integer, CultureInfo.InvariantCulture, out short loadTorque) ||
            !byte.TryParse(parts[offset + 3], NumberStyles.Integer, CultureInfo.InvariantCulture, out byte flags) ||
            !byte.TryParse(parts[offset + 4], NumberStyles.Integer, CultureInfo.InvariantCulture, out byte enable))
        {
            error = "invalid numeric value";
            return false;
        }

        inputs = new HilInputs(speedRpm, zeroPeriod, loadTorque, flags, enable != 0);
        return true;
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

    private void TrackRoundTrip(double roundTripMs, HilOutputs? outputs, bool hilEnabled)
    {
        _averageRoundTripMs = _averageRoundTripMs == 0 ? roundTripMs : (_averageRoundTripMs * 0.9) + (roundTripMs * 0.1);
        double effectiveRate = _uptime.Elapsed.TotalSeconds <= 0 ? 0 : _rxFrames / _uptime.Elapsed.TotalSeconds;
        HilOutputs? lastOutputs = outputs ?? _bridge.Snapshot.Hil.LastOutputs;
        _bridge.UpdateHilStats(new HilBridgeStats(
            outputs?.Mode == ControlRuntimeMode.HilSim || (outputs is null && hilEnabled),
            _rxFrames,
            _lostFrames,
            effectiveRate,
            _averageRoundTripMs,
            _jitterMs,
            lastOutputs));
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
