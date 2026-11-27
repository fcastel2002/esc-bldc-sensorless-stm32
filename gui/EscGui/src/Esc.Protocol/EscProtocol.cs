using System.Buffers.Binary;
using System.Text;

namespace Esc.Protocol;

public static class EscProtocol
{
    public static byte[] BuildRequest(byte sequence, CommOpcode opcode, byte parameter = 0, ReadOnlySpan<byte> payload = default)
    {
        return BuildFrame(sequence, opcode, parameter, payload, CommFrameType.Request, CommStatus.Ok);
    }

    public static byte[] BuildFrame(
        byte sequence,
        CommOpcode opcode,
        byte parameter,
        ReadOnlySpan<byte> payload,
        CommFrameType frameType,
        CommStatus status)
    {
        if (payload.Length > CommConstants.PayloadMax)
        {
            throw new ArgumentOutOfRangeException(nameof(payload), "Payload is too long for a 64 byte ESC frame.");
        }

        var frame = new byte[CommConstants.FrameSize];
        frame[0] = CommConstants.Magic0;
        frame[1] = CommConstants.Magic1;
        frame[2] = CommConstants.Version;
        frame[3] = (byte)frameType;
        frame[4] = sequence;
        frame[5] = (byte)opcode;
        frame[6] = parameter;
        frame[7] = (byte)status;
        BinaryPrimitives.WriteUInt16LittleEndian(frame.AsSpan(8, 2), (ushort)payload.Length);
        payload.CopyTo(frame.AsSpan(CommConstants.PayloadOffset));

        ushort crc = ComputeCrc16(frame.AsSpan(0, CommConstants.CrcOffset));
        BinaryPrimitives.WriteUInt16LittleEndian(frame.AsSpan(CommConstants.CrcOffset, 2), crc);
        return frame;
    }

    public static EscFrame Parse(ReadOnlySpan<byte> reportOrFrame)
    {
        ReadOnlySpan<byte> frame = NormalizeReport(reportOrFrame);
        if (frame.Length != CommConstants.FrameSize)
        {
            throw new EscProtocolException($"Incomplete frame: received {frame.Length} bytes.");
        }

        if (frame[0] != CommConstants.Magic0 || frame[1] != CommConstants.Magic1)
        {
            throw new EscProtocolException($"Invalid magic: {frame[0]:X2} {frame[1]:X2}.");
        }

        if (frame[2] != CommConstants.Version)
        {
            throw new EscProtocolException($"Invalid protocol version: {frame[2]}.");
        }

        ushort payloadLength = BinaryPrimitives.ReadUInt16LittleEndian(frame.Slice(8, 2));
        if (payloadLength > CommConstants.PayloadMax)
        {
            throw new EscProtocolException($"Invalid payload length: {payloadLength}.");
        }

        ushort expectedCrc = BinaryPrimitives.ReadUInt16LittleEndian(frame.Slice(CommConstants.CrcOffset, 2));
        ushort actualCrc = ComputeCrc16(frame[..CommConstants.CrcOffset]);
        if (expectedCrc != actualCrc)
        {
            throw new EscProtocolException($"Invalid CRC: received 0x{expectedCrc:X4}, calculated 0x{actualCrc:X4}.");
        }

        var raw = frame.ToArray();
        var payload = frame.Slice(CommConstants.PayloadOffset, payloadLength).ToArray();

        return new EscFrame(
            raw,
            frame[2],
            (CommFrameType)frame[3],
            frame[4],
            (CommOpcode)frame[5],
            frame[6],
            (CommStatus)frame[7],
            payload,
            expectedCrc);
    }

    public static ushort ComputeCrc16(ReadOnlySpan<byte> data)
    {
        ushort crc = 0xFFFF;

        foreach (byte value in data)
        {
            crc ^= (ushort)(value << 8);
            for (int bit = 0; bit < 8; bit++)
            {
                crc = (crc & 0x8000) != 0
                    ? (ushort)((crc << 1) ^ 0x1021)
                    : (ushort)(crc << 1);
            }
        }

        return crc;
    }

    public static byte[] UInt16Payload(int value)
    {
        var payload = new byte[2];
        BinaryPrimitives.WriteUInt16LittleEndian(payload, checked((ushort)value));
        return payload;
    }

    public static byte[] Int16CentiPayload(double value)
    {
        var payload = new byte[2];
        BinaryPrimitives.WriteInt16LittleEndian(payload, checked((short)Math.Round(value * 100.0)));
        return payload;
    }

    public static byte[] ConfigPayload(ConfigParam parameter, double value)
    {
        EnsureFinite(value, nameof(value));
        return parameter switch
        {
            ConfigParam.PwmFreq or ConfigParam.MaxSpeed or ConfigParam.MinSpeed or ConfigParam.BemfBlankingUs
                or ConfigParam.SpeedRamp
                => UInt16Payload((int)value),
            ConfigParam.PolePairs => new[] { checked((byte)value) },
            ConfigParam.KpRpm or ConfigParam.KiRpm or ConfigParam.KdRpm => Int16CentiPayload(value),
            ConfigParam.StartupInitialAmplitude or ConfigParam.StartupFinalAmplitude
                => UInt16Payload(checked((int)Math.Round(value * 10.0))),
            ConfigParam.StartupInitialFrequency or ConfigParam.StartupFinalFrequency
                => UInt32Payload(checked((uint)Math.Round(value * 1000.0))),
            ConfigParam.StartupDuration
                => UInt32Payload(checked((uint)Math.Round(value * 1000.0))),
            _ => throw new ArgumentOutOfRangeException(nameof(parameter), parameter, "Unsupported config parameter.")
        };
    }

    public static byte[] UInt32Payload(uint value)
    {
        var payload = new byte[4];
        BinaryPrimitives.WriteUInt32LittleEndian(payload, value);
        return payload;
    }

    public static byte[] SineDrivePayload(double electricalFrequencyHz, double amplitudePercent)
    {
        EnsureFinite(electricalFrequencyHz, nameof(electricalFrequencyHz));
        EnsureFinite(amplitudePercent, nameof(amplitudePercent));
        if (electricalFrequencyHz is < CommConstants.MinSineFrequencyHz or > CommConstants.MaxSineFrequencyHz)
        {
            throw new ArgumentOutOfRangeException(nameof(electricalFrequencyHz));
        }
        if (amplitudePercent is < 0 or > CommConstants.MaxSineAmplitudePercent)
        {
            throw new ArgumentOutOfRangeException(nameof(amplitudePercent));
        }

        var payload = new byte[6];
        BinaryPrimitives.WriteUInt32LittleEndian(
            payload.AsSpan(0, 4), checked((uint)Math.Round(electricalFrequencyHz * 1000.0)));
        BinaryPrimitives.WriteUInt16LittleEndian(
            payload.AsSpan(4, 2), checked((ushort)Math.Round(amplitudePercent * 10.0)));
        return payload;
    }

    public static SineDriveSettings DecodeSineDrive(EscFrame frame)
    {
        EnsureOkResponse(frame, CommOpcode.SineDrive);
        if (frame.Payload.Length != 6)
        {
            throw new EscProtocolException(
                $"SINE_DRIVE response payload must be exactly 6 bytes, received {frame.Payload.Length}.");
        }

        return new SineDriveSettings(
            BinaryPrimitives.ReadUInt32LittleEndian(frame.Payload.AsSpan(0, 4)) / 1000.0,
            BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(4, 2)) / 10.0);
    }

    public static byte[] StepTestPayload(StepTestSettings settings)
    {
        ArgumentNullException.ThrowIfNull(settings);
        if (!Enum.IsDefined(settings.Step))
        {
            throw new ArgumentOutOfRangeException(nameof(settings), settings.Step, "Unknown commutation step.");
        }
        EnsureFinite(settings.DutyPercent, nameof(settings));
        if (settings.DutyPercent is < 0 or > CommConstants.MaxStepTestDutyPercent)
        {
            throw new ArgumentOutOfRangeException(nameof(settings), settings.DutyPercent, "Duty out of range.");
        }

        var payload = new byte[3];
        payload[0] = (byte)settings.Step;
        BinaryPrimitives.WriteUInt16LittleEndian(
            payload.AsSpan(1, 2), checked((ushort)Math.Round(settings.DutyPercent * 10.0)));
        return payload;
    }

    public static StepTestSettings DecodeStepTest(EscFrame frame)
    {
        EnsureOkResponse(frame, CommOpcode.StepTest);
        if (frame.Payload.Length != 3)
        {
            throw new EscProtocolException(
                $"STEP_TEST response payload must be exactly 3 bytes, received {frame.Payload.Length}.");
        }

        return new StepTestSettings(
            (CommutationStep)frame.Payload[0],
            BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(1, 2)) / 10.0);
    }

    public static byte[] ZcMapPayload(ZcChannelMap map)
    {
        ArgumentNullException.ThrowIfNull(map);
        if (!map.IsValid)
        {
            throw new ArgumentOutOfRangeException(nameof(map), "ZC channel map must be a permutation of {0, 1, 2}.");
        }

        return [map.ChannelU, map.ChannelV, map.ChannelW];
    }

    public static ZcChannelMap DecodeZcMap(ReadOnlySpan<byte> payload)
    {
        if (payload.Length != 3)
        {
            throw new EscProtocolException($"ZC_MAP payload must be exactly 3 bytes, received {payload.Length}.");
        }

        var map = new ZcChannelMap(payload[0], payload[1], payload[2]);
        if (!map.IsValid)
        {
            throw new EscProtocolException($"Invalid ZC_MAP permutation: [{payload[0]}, {payload[1]}, {payload[2]}].");
        }

        return map;
    }

    public static byte[] ZcAutoDetectPayload(double dutyPercent)
    {
        EnsureFinite(dutyPercent, nameof(dutyPercent));
        if (dutyPercent is < CommConstants.MinZcAutoDetectDutyPercent or > CommConstants.MaxZcAutoDetectDutyPercent)
        {
            throw new ArgumentOutOfRangeException(nameof(dutyPercent), dutyPercent, "Auto-detect duty out of range.");
        }

        return UInt16Payload(checked((int)Math.Round(dutyPercent * 10.0)));
    }

    public static ZcDiagReport DecodeZcDiagReport(EscFrame frame)
    {
        EnsureOkResponse(frame, CommOpcode.ZcDiag);
        if (frame.Payload.Length != 32)
        {
            throw new EscProtocolException(
                $"ZC_DIAG report payload must be exactly 32 bytes, received {frame.Payload.Length}.");
        }

        ReadOnlySpan<byte> p = frame.Payload;
        var activeMap = new ZcChannelMap(p[23], p[24], p[25]);
        var recent = new List<ZcRecentEvent>(6);
        for (int i = 0; i < 6; i++)
        {
            byte raw = p[26 + i];
            if ((raw & 0x80) != 0)
            {
                recent.Add(new ZcRecentEvent((byte)(raw & 0x03), (raw & 0x04) != 0));
            }
        }

        return new ZcDiagReport(
            Active: p[0] != 0,
            LivePinsMask: p[1],
            SeenStatesMask: p[2],
            HealthFlags: (ZcHealthFlags)p[3],
            EdgesCh0: BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(4, 2)),
            EdgesCh1: BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(6, 2)),
            EdgesCh2: BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(8, 2)),
            ForwardSteps: BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(10, 2)),
            ReverseSteps: BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(12, 2)),
            InvalidSteps: BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(14, 2)),
            GlitchCount: BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(16, 2)),
            EstimatedRpm: BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(18, 2)),
            AvgStepTicks: BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(20, 2)),
            StepAsymmetryPct: p[22],
            ActiveMap: activeMap.IsValid ? activeMap : ZcChannelMap.Default,
            RecentEvents: recent);
    }

    public static ZcAutoDetectResult DecodeZcAutoDetect(EscFrame frame)
    {
        EnsureOkResponse(frame, CommOpcode.ZcDiag);
        if (frame.Payload.Length != 16)
        {
            throw new EscProtocolException(
                $"ZC_DIAG auto-detect payload must be exactly 16 bytes, received {frame.Payload.Length}.");
        }

        ReadOnlySpan<byte> p = frame.Payload;
        var status = (ZcAutoDetectStatus)p[0];
        var candidate = new ZcChannelMap(p[1], p[2], p[3]);
        var active = new ZcChannelMap(p[4], p[5], p[6]);
        byte[] matrix = p.Slice(7, 9).ToArray();

        return new ZcAutoDetectResult(
            status,
            candidate.IsValid ? candidate : null,
            active.IsValid ? active : ZcChannelMap.Default,
            matrix);
    }

    public static object DecodeConfigValue(ConfigParam parameter, ReadOnlySpan<byte> payload)
    {
        return parameter switch
        {
            ConfigParam.PolePairs when payload.Length >= 1 => payload[0],
            ConfigParam.PwmFreq or ConfigParam.MaxSpeed or ConfigParam.MinSpeed or ConfigParam.BemfBlankingUs
                or ConfigParam.SpeedRamp
                when payload.Length >= 2
                => BinaryPrimitives.ReadUInt16LittleEndian(payload[..2]),
            ConfigParam.KpRpm or ConfigParam.KiRpm or ConfigParam.KdRpm when payload.Length >= 2
                => BinaryPrimitives.ReadInt16LittleEndian(payload[..2]) / 100.0,
            ConfigParam.StartupInitialAmplitude or ConfigParam.StartupFinalAmplitude when payload.Length >= 2
                => BinaryPrimitives.ReadUInt16LittleEndian(payload[..2]) / 10.0,
            ConfigParam.StartupInitialFrequency or ConfigParam.StartupFinalFrequency when payload.Length >= 4
                => BinaryPrimitives.ReadUInt32LittleEndian(payload[..4]) / 1000.0,
            ConfigParam.StartupDuration when payload.Length >= 4
                => BinaryPrimitives.ReadUInt32LittleEndian(payload[..4]) / 1000.0,
            ConfigParam.ZcMap when payload.Length == 3
                => DecodeZcMap(payload),
            _ => payload.ToArray()
        };
    }

    public static EscStatus DecodeStatus(EscFrame frame)
    {
        EnsureOkResponse(frame, CommOpcode.GetStatus);
        if (frame.Payload.Length < 10)
        {
            throw new EscProtocolException("GET_STATUS response payload is shorter than 10 bytes.");
        }

        return new EscStatus(
            frame.Payload[0],
            AppStateName(frame.Payload[0]),
            frame.Payload[1] == 1 ? "USB" : "UART",
            frame.Payload[2] != 0,
            frame.Payload[3] != 0,
            BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(4, 2)),
            BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(6, 2)),
            BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(8, 2)),
            frame.Payload.Length >= 18
                ? new PowerMonitorReading(
                    (PowerMonitorFlags)frame.Payload[10],
                    frame.Payload[11],
                    BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(12, 2)),
                    BinaryPrimitives.ReadInt32LittleEndian(frame.Payload.AsSpan(14, 4)),
                    frame.Payload.Length >= 20 ? (PowerMonitorError)frame.Payload[18] : PowerMonitorError.None,
                    frame.Payload.Length >= 20 ? frame.Payload[19] : (byte)0,
                    frame.Payload.Length >= 24 ? frame.Payload[20] : null,
                    frame.Payload.Length >= 24 ? (PowerMonitorStage)frame.Payload[21] : PowerMonitorStage.None,
                    frame.Payload.Length >= 24 ? BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(22, 2)) : (ushort)0,
                    frame.Payload.Length >= 26 ? BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(24, 2)) : (ushort)0)
                : null,
            frame.Payload.Length >= 30 ? BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(26, 2)) : null,
            frame.Payload.Length >= 30 ? BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(28, 2)) : null,
            frame.Payload.Length >= 31 && frame.Payload[30] != 0);
    }

    public static byte[] LoadSweepStartPayload(LoadSweepRequest request)
    {
        ArgumentNullException.ThrowIfNull(request);
        if (!Enum.IsDefined(request.Mode))
        {
            throw new ArgumentOutOfRangeException(nameof(request), request.Mode, "Unknown sweep mode.");
        }

        var payload = new byte[12];
        payload[0] = (byte)request.Mode;
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(1, 2), request.StartRpm);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(3, 2), request.EndRpm);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(5, 2), request.StepOrRate);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(7, 2), request.SettleMs);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(9, 2), request.WindowMs);
        payload[11] = (byte)request.Options;
        return payload;
    }

    public static LoadSweepStatus DecodeLoadSweepStatus(EscFrame frame)
    {
        EnsureOkResponse(frame, CommOpcode.LoadSweep);
        if (frame.Payload.Length != 29)
        {
            throw new EscProtocolException(
                $"LOAD_SWEEP status payload must be exactly 29 bytes, received {frame.Payload.Length}.");
        }

        ReadOnlySpan<byte> p = frame.Payload;
        return new LoadSweepStatus(
            (LoadSweepState)p[0],
            (LoadSweepAbortReason)p[1],
            (LoadSweepMode)p[2],
            (LoadSweepOptions)p[3],
            BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(4, 2)),
            BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(6, 2)),
            BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(8, 2)),
            BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(10, 2)),
            BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(12, 2)),
            BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(14, 2)),
            BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(16, 2)),
            BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(18, 2)),
            BinaryPrimitives.ReadInt32LittleEndian(p.Slice(20, 4)),
            BinaryPrimitives.ReadUInt32LittleEndian(p.Slice(24, 4)),
            p[28] != 0);
    }

    public static LoadSweepPointBatch DecodeLoadSweepPoints(EscFrame frame)
    {
        EnsureOkResponse(frame, CommOpcode.LoadSweep);
        ReadOnlySpan<byte> p = frame.Payload;
        if (p.Length < 3)
        {
            throw new EscProtocolException("LOAD_SWEEP points payload is shorter than 3 bytes.");
        }

        ushort first = BinaryPrimitives.ReadUInt16LittleEndian(p.Slice(0, 2));
        int count = p[2];
        if (count > CommConstants.LoadSweepPointsPerFrame ||
            p.Length != 3 + count * CommConstants.LoadSweepPointSize)
        {
            throw new EscProtocolException(
                $"LOAD_SWEEP points payload has {p.Length} bytes for {count} points.");
        }

        var points = new List<LoadSweepPoint>(count);
        for (int i = 0; i < count; i++)
        {
            ReadOnlySpan<byte> q = p.Slice(3 + i * CommConstants.LoadSweepPointSize, CommConstants.LoadSweepPointSize);
            points.Add(new LoadSweepPoint(
                first + i,
                BinaryPrimitives.ReadUInt16LittleEndian(q.Slice(0, 2)),
                BinaryPrimitives.ReadUInt16LittleEndian(q.Slice(2, 2)),
                BinaryPrimitives.ReadUInt16LittleEndian(q.Slice(4, 2)),
                BinaryPrimitives.ReadUInt16LittleEndian(q.Slice(6, 2)),
                BinaryPrimitives.ReadInt16LittleEndian(q.Slice(8, 2)),
                BinaryPrimitives.ReadUInt16LittleEndian(q.Slice(10, 2)),
                BinaryPrimitives.ReadUInt16LittleEndian(q.Slice(12, 2)),
                q[14],
                (LoadPointFlags)q[15]));
        }

        return new LoadSweepPointBatch(first, points);
    }

    public static TelemetrySample DecodeTelemetry(EscFrame frame, DateTimeOffset hostTimestamp)
    {
        if (frame.Type != CommFrameType.Event || frame.Opcode != CommOpcode.TelemetryEvent)
        {
            throw new EscProtocolException("Frame is not a telemetry event.");
        }

        if (frame.Payload.Length < 9)
        {
            throw new EscProtocolException("Telemetry event payload is shorter than 9 bytes.");
        }
        if (frame.Payload.Length is > 9 and < 15)
        {
            throw new EscProtocolException("Extended telemetry payload must be at least 15 bytes.");
        }

        var logParam = (LogParam)frame.Payload[0];
        if (frame.Parameter != frame.Payload[0])
        {
            throw new EscProtocolException("Telemetry parameter does not match its payload identifier.");
        }
        int value = BinaryPrimitives.ReadInt32LittleEndian(frame.Payload.AsSpan(1, 4));
        uint targetTick = BinaryPrimitives.ReadUInt32LittleEndian(frame.Payload.AsSpan(5, 4));
        string unit = TelemetryUnit(logParam);
        string name = logParam switch
        {
            LogParam.Speed or LogParam.BusVoltage or LogParam.BusCurrent or LogParam.Duty
                => TelemetryVariable(logParam),
            _ => $"0x{frame.Payload[0]:X2}"
        };

        ushort? adcRaw = frame.Payload.Length >= 11
            ? BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(9, 2))
            : null;
        TelemetryQuality quality = frame.Payload.Length >= 12
            ? (TelemetryQuality)frame.Payload[11]
            : TelemetryQuality.None;
        ushort validSamples = frame.Payload.Length >= 14
            ? BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(12, 2))
            : (ushort)0;
        sbyte? commutationStep = frame.Payload.Length >= 15
            ? unchecked((sbyte)frame.Payload[14])
            : null;

        return new TelemetrySample(
            name, logParam, value, unit, targetTick, hostTimestamp,
            adcRaw, quality, validSamples, commutationStep);
    }

    /// <summary>Telemetry store key for a log channel.</summary>
    public static string TelemetryVariable(LogParam parameter) => parameter switch
    {
        LogParam.Speed => "speed",
        LogParam.BusVoltage => "bus_voltage",
        LogParam.BusCurrent => "bus_current",
        LogParam.Duty => "duty",
        _ => throw new ArgumentOutOfRangeException(nameof(parameter), parameter, "Canal de telemetria no soportado.")
    };

    /// <summary>Unit of the raw telemetry value for a log channel.</summary>
    public static string TelemetryUnit(LogParam parameter) => parameter switch
    {
        LogParam.Speed => "rpm",
        LogParam.BusVoltage => "mV",
        LogParam.BusCurrent => "mA",
        LogParam.Duty => "%", /* raw value is 1/10000; DisplayValue is percent */
        _ => "raw"
    };

    public static void EnsureOkResponse(EscFrame frame, CommOpcode expectedOpcode)
    {
        if (frame.Type != CommFrameType.Response)
        {
            throw new EscProtocolException($"Expected response frame, received {frame.Type}.");
        }

        if (frame.Opcode != expectedOpcode)
        {
            throw new EscProtocolException($"Expected {expectedOpcode} response, received {frame.Opcode}.");
        }

        if (frame.Status != CommStatus.Ok)
        {
            throw new EscProtocolException($"ESC returned {frame.Status} for {frame.Opcode}.");
        }
    }

    public static string PayloadText(ReadOnlySpan<byte> payload)
    {
        return Encoding.UTF8.GetString(payload).TrimEnd('\0');
    }

    private static ReadOnlySpan<byte> NormalizeReport(ReadOnlySpan<byte> reportOrFrame)
    {
        if (reportOrFrame.Length == CommConstants.FrameSize + 1 && reportOrFrame[0] == 0)
        {
            return reportOrFrame[1..];
        }

        return reportOrFrame;
    }

    private static string AppStateName(byte value)
    {
        return value switch
        {
            0 => "IDLE",
            1 => "STARTUP",
            2 => "FOC_STARTUP",
            3 => "CONFIG",
            4 => "RUNNING",
            5 => "READY",
            6 => "CLOSEDLOOP",
            7 => "STOPPED",
            8 => "HARD_ERROR",
            9 => "FINISH",
            10 => "SINE_DRIVE",
            11 => "STEP_TEST",
            12 => "FREEWHEEL",
            _ => $"STATE_{value}"
        };
    }

    private static void EnsureFinite(double value, string parameterName)
    {
        if (!double.IsFinite(value))
        {
            throw new ArgumentOutOfRangeException(parameterName, value, "Value must be finite.");
        }
    }
}
