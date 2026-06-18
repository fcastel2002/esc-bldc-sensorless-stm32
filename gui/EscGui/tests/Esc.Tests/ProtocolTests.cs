using Esc.Protocol;

namespace Esc.Tests;

public sealed class ProtocolTests
{
    [Fact]
    public void BuildPingFrameMatchesExistingPythonHelperCrc()
    {
        byte[] frame = EscProtocol.BuildRequest(1, CommOpcode.Ping, 0, "ping"u8.ToArray());

        Assert.Equal(CommConstants.FrameSize, frame.Length);
        Assert.Equal(0xEB, frame[62]);
        Assert.Equal(0x15, frame[63]);
    }

    [Fact]
    public void ParseRoundTripDecodesRequest()
    {
        byte[] frame = EscProtocol.BuildRequest(7, CommOpcode.SetSpeedRpm, payload: EscProtocol.UInt16Payload(1500));

        EscFrame parsed = EscProtocol.Parse(frame);

        Assert.Equal(CommFrameType.Request, parsed.Type);
        Assert.Equal((byte)7, parsed.Sequence);
        Assert.Equal(CommOpcode.SetSpeedRpm, parsed.Opcode);
        Assert.Equal(new byte[] { 0xDC, 0x05 }, parsed.Payload);
    }

    [Fact]
    public void ParseRejectsBadCrc()
    {
        byte[] frame = EscProtocol.BuildRequest(1, CommOpcode.GetStatus);
        frame[12] ^= 0x55;

        Assert.Throws<EscProtocolException>(() => EscProtocol.Parse(frame));
    }

    [Fact]
    public void DecodeTelemetryReadsSpeedEvent()
    {
        byte[] payload = new byte[9];
        payload[0] = (byte)LogParam.Speed;
        BitConverter.GetBytes(1234).CopyTo(payload, 1);
        BitConverter.GetBytes(5678u).CopyTo(payload, 5);

        byte[] raw = EscProtocol.BuildFrame(2, CommOpcode.TelemetryEvent, (byte)LogParam.Speed, payload, CommFrameType.Event, CommStatus.Ok);
        TelemetrySample sample = EscProtocol.DecodeTelemetry(EscProtocol.Parse(raw), DateTimeOffset.UnixEpoch);

        Assert.Equal("speed", sample.Variable);
        Assert.Equal(1234, sample.RawValue);
        Assert.Equal(5678u, sample.TargetTickMs);
        Assert.Equal("rpm", sample.Unit);
    }

    [Fact]
    public void DecodeStatusUsesFirmwareStateOrder()
    {
        byte[] payload = new byte[10];
        payload[0] = 2;
        payload[1] = 1;

        byte[] raw = EscProtocol.BuildFrame(3, CommOpcode.GetStatus, 0, payload, CommFrameType.Response, CommStatus.Ok);
        EscStatus status = EscProtocol.DecodeStatus(EscProtocol.Parse(raw));

        Assert.Equal("FOC_STARTUP", status.AppStateName);
    }
}
