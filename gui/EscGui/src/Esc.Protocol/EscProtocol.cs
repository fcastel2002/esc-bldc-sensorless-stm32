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
        return parameter switch
        {
            ConfigParam.PwmFreq or ConfigParam.MaxSpeed or ConfigParam.MinSpeed => UInt16Payload((int)value),
            ConfigParam.PolePairs => new[] { checked((byte)value) },
            ConfigParam.Kp or ConfigParam.Ki or ConfigParam.Kd => Int16CentiPayload(value),
            _ => throw new ArgumentOutOfRangeException(nameof(parameter), parameter, "Unsupported config parameter.")
        };
    }

    public static byte[] ControlModePayload(ControlRuntimeMode mode)
    {
        return [(byte)mode];
    }

    public static byte[] HilInputsPayload(HilInputs inputs)
    {
        var payload = new byte[8];
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(0, 2), inputs.SpeedRpm);
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(2, 2), inputs.ZeroCrossingPeriod);
        BinaryPrimitives.WriteInt16LittleEndian(payload.AsSpan(4, 2), inputs.LoadTorque);
        payload[6] = inputs.Flags;
        payload[7] = inputs.Enable ? (byte)1 : (byte)0;
        return payload;
    }

    public static HilOutputs DecodeHilOutputs(EscFrame frame)
    {
        EnsureOkResponse(frame, CommOpcode.HilGetOutputs);
        if (frame.Payload.Length < 15)
        {
            throw new EscProtocolException("HIL_GET_OUTPUTS response payload is shorter than 15 bytes.");
        }

        return new HilOutputs(
            BinaryPrimitives.ReadUInt32LittleEndian(frame.Payload.AsSpan(0, 4)),
            frame.Payload[4],
            (ControlRuntimeMode)frame.Payload[5],
            BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(6, 2)),
            BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(8, 2)),
            BinaryPrimitives.ReadUInt16LittleEndian(frame.Payload.AsSpan(10, 2)),
            unchecked((sbyte)frame.Payload[12]),
            frame.Payload[13],
            frame.Payload[14] != 0);
    }

    public static object DecodeConfigValue(ConfigParam parameter, ReadOnlySpan<byte> payload)
    {
        return parameter switch
        {
            ConfigParam.PolePairs when payload.Length >= 1 => payload[0],
            ConfigParam.PwmFreq or ConfigParam.MaxSpeed or ConfigParam.MinSpeed when payload.Length >= 2
                => BinaryPrimitives.ReadUInt16LittleEndian(payload[..2]),
            ConfigParam.Kp or ConfigParam.Ki or ConfigParam.Kd when payload.Length >= 2
                => BinaryPrimitives.ReadInt16LittleEndian(payload[..2]) / 100.0,
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

        var logParam = (LogParam)frame.Payload[0];
        int value = BinaryPrimitives.ReadInt32LittleEndian(frame.Payload.AsSpan(1, 4));
        uint targetTick = BinaryPrimitives.ReadUInt32LittleEndian(frame.Payload.AsSpan(5, 4));
        string unit = logParam switch
        {
            LogParam.Speed => "rpm",
            LogParam.Temperature => "cdeg",
            LogParam.Current => "mA",
            _ => "raw"
        };

        string name = logParam switch
        {
            LogParam.Speed => "speed",
            LogParam.Temperature => "temperature",
            LogParam.Current => "current",
            _ => $"0x{frame.Payload[0]:X2}"
        };

        return new TelemetrySample(name, logParam, value, unit, targetTick, hostTimestamp);
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
            _ => $"STATE_{value}"
        };
    }
}
