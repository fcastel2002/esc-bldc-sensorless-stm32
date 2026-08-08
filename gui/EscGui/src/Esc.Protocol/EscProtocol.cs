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
            ConfigParam.PwmFreq or ConfigParam.MaxSpeed or ConfigParam.MinSpeed => UInt16Payload((int)value),
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

    public static byte[] ControlModePayload(ControlRuntimeMode mode)
    {
        return [(byte)mode];
    }

    public static byte[] HilStartPayload(ushort inputTimeoutMs)
    {
        return UInt16Payload(inputTimeoutMs);
    }

    public static byte[] HilStartPayload(ushort inputTimeoutMs, HilExecutionMode executionMode)
    {
        if (executionMode is not HilExecutionMode.Periodic and not HilExecutionMode.Stepped)
        {
            throw new ArgumentOutOfRangeException(nameof(executionMode), executionMode, "Unsupported HIL execution mode.");
        }

        var payload = new byte[3];
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(0, 2), inputTimeoutMs);
        payload[2] = (byte)executionMode;
        return payload;
    }

    public static byte[] HilInputsPayload(HilInputs inputs)
    {
        if (inputs.RunId.HasValue != inputs.SourceSequence.HasValue)
        {
            throw new ArgumentException("HIL validation inputs require both run ID and source sequence.", nameof(inputs));
        }

        var payload = new byte[inputs.HasValidationProvenance ? 16 : 8];
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(0, 2), inputs.SpeedRpm);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(2, 2), 0);
        BinaryPrimitives.WriteInt16LittleEndian(payload.AsSpan(4, 2), inputs.LoadTorque);
        payload[6] = inputs.Flags;
        payload[7] = inputs.Enable ? (byte)1 : (byte)0;
        if (inputs.HasValidationProvenance)
        {
            BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(8, 4), inputs.RunId!.Value);
            BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(12, 4), inputs.SourceSequence!.Value);
        }

        return payload;
    }

    public static byte[] HilStepPayload(HilStepRequest request)
    {
        ArgumentNullException.ThrowIfNull(request);

        if (request.RunId == 0)
        {
            throw new ArgumentOutOfRangeException(nameof(request), request.RunId, "HIL step run ID must be non-zero.");
        }

        if (request.SourceSequence == 0)
        {
            throw new ArgumentOutOfRangeException(nameof(request), request.SourceSequence, "HIL step source sequence must be non-zero.");
        }

        if (request.Steps is < 1 or > 1000)
        {
            throw new ArgumentOutOfRangeException(nameof(request), request.Steps, "HIL step count must be between 1 and 1000.");
        }
        if (!request.Enable)
        {
            throw new ArgumentException("Deterministic HIL steps require enable=true.", nameof(request));
        }

        var payload = new byte[18];
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(0, 2), request.SpeedRpm);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(2, 2), 0);
        BinaryPrimitives.WriteInt16LittleEndian(payload.AsSpan(4, 2), request.LoadTorque);
        payload[6] = request.Flags;
        payload[7] = request.Enable ? (byte)1 : (byte)0;
        BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(8, 4), request.RunId);
        BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(12, 4), request.SourceSequence);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(16, 2), request.Steps);
        return payload;
    }

    public static HilOutputs DecodeHilOutputs(EscFrame frame)
    {
        EnsureOkResponse(frame, CommOpcode.HilGetOutputs);
        if (frame.Payload.Length < 15)
        {
            throw new EscProtocolException("HIL_GET_OUTPUTS response payload is shorter than 15 bytes.");
        }

        return DecodeHilOutputsPayload(frame.Payload);
    }

    public static HilStepResult DecodeHilStepResult(EscFrame frame)
    {
        EnsureOkResponse(frame, CommOpcode.HilStep);
        if (frame.Payload.Length != 48)
        {
            throw new EscProtocolException($"HIL_STEP response payload must be exactly 48 bytes, received {frame.Payload.Length}.");
        }

        return new HilStepResult(
            DecodeHilOutputsPayload(frame.Payload.AsSpan(0, 43)),
            BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(43, 2)),
            BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(45, 2)),
            frame.Payload[47]);
    }

    private static HilOutputs DecodeHilOutputsPayload(ReadOnlySpan<byte> payload)
    {
        bool hasValidationProvenance = payload.Length >= 43;
        return new HilOutputs(
            BinaryPrimitives.ReadUInt32LittleEndian(payload.Slice(0, 4)),
            payload[4],
            (ControlRuntimeMode)payload[5],
            BinaryPrimitives.ReadUInt16LittleEndian(payload.Slice(6, 2)),
            BinaryPrimitives.ReadUInt16LittleEndian(payload.Slice(8, 2)),
            BinaryPrimitives.ReadUInt16LittleEndian(payload.Slice(10, 2)),
            unchecked((sbyte)payload[12]),
            payload[13],
            payload[14] != 0,
            hasValidationProvenance ? BinaryPrimitives.ReadUInt32LittleEndian(payload.Slice(15, 4)) : 0,
            hasValidationProvenance ? BinaryPrimitives.ReadUInt32LittleEndian(payload.Slice(19, 4)) : 0,
            hasValidationProvenance ? BinaryPrimitives.ReadUInt32LittleEndian(payload.Slice(23, 4)) : 0,
            hasValidationProvenance ? BinaryPrimitives.ReadUInt32LittleEndian(payload.Slice(27, 4)) : 0,
            hasValidationProvenance ? BinaryPrimitives.ReadUInt32LittleEndian(payload.Slice(31, 4)) : 0,
            hasValidationProvenance ? BinaryPrimitives.ReadUInt32LittleEndian(payload.Slice(35, 4)) : 0,
            hasValidationProvenance ? BinaryPrimitives.ReadUInt32LittleEndian(payload.Slice(39, 4)) : 0,
            hasValidationProvenance);
    }

    public static object DecodeConfigValue(ConfigParam parameter, ReadOnlySpan<byte> payload)
    {
        return parameter switch
        {
            ConfigParam.PolePairs when payload.Length >= 1 => payload[0],
            ConfigParam.PwmFreq or ConfigParam.MaxSpeed or ConfigParam.MinSpeed when payload.Length >= 2
                => BinaryPrimitives.ReadUInt16LittleEndian(payload[..2]),
            ConfigParam.KpRpm or ConfigParam.KiRpm or ConfigParam.KdRpm when payload.Length >= 2
                => BinaryPrimitives.ReadInt16LittleEndian(payload[..2]) / 100.0,
            ConfigParam.StartupInitialAmplitude or ConfigParam.StartupFinalAmplitude when payload.Length >= 2
                => BinaryPrimitives.ReadUInt16LittleEndian(payload[..2]) / 10.0,
            ConfigParam.StartupInitialFrequency or ConfigParam.StartupFinalFrequency when payload.Length >= 4
                => BinaryPrimitives.ReadUInt32LittleEndian(payload[..4]) / 1000.0,
            ConfigParam.StartupDuration when payload.Length >= 4
                => BinaryPrimitives.ReadUInt32LittleEndian(payload[..4]) / 1000.0,
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
            BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(8, 2)));
    }

    public static ValidationReference DecodeValidationReference(EscFrame frame)
    {
        EnsureOkResponse(frame, CommOpcode.GetValidationReference);
        if (frame.Payload.Length is not 20 and not 24)
        {
            throw new EscProtocolException($"GET_VALIDATION_REFERENCE response payload must be 20 or 24 bytes, received {frame.Payload.Length}.");
        }

        bool hasCapabilities = frame.Payload.Length == 24;

        return new ValidationReference(
            frame.Payload[0],
            frame.Payload[1],
            BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(2, 2)),
            BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(4, 2)),
            BinaryPrimitives.ReadUInt32LittleEndian(frame.Payload.AsSpan(6, 4)),
            BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(10, 2)),
            BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(12, 2)),
            BinaryPrimitives.ReadUInt32LittleEndian(frame.Payload.AsSpan(14, 4)),
            BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(18, 2)),
            hasCapabilities ? frame.Payload[20] : (byte)0,
            hasCapabilities ? frame.Payload[21] : (byte)0,
            hasCapabilities ? BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(22, 2)) : (ushort)0);
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
        string unit = logParam switch
        {
            LogParam.Speed => "rpm",
            LogParam.Temperature => "cdeg",
            LogParam.CurrentU or LogParam.CurrentV => "mA",
            LogParam.BemfPeriod => "ticks",
            _ => "raw"
        };

        string name = logParam switch
        {
            LogParam.Speed => "speed",
            LogParam.Temperature => "temperature",
            LogParam.CurrentU => "current_u",
            LogParam.CurrentV => "current_v",
            LogParam.BemfPeriod => "bemf_period",
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
