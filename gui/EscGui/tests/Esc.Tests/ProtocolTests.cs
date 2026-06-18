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

    [Fact]
    public void HilInputsPayloadUsesExpectedWireShape()
    {
        byte[] payload = EscProtocol.HilInputsPayload(new HilInputs(1500, 123, -4, 0xA5, true));

        Assert.Equal([0xDC, 0x05, 0x7B, 0x00, 0xFC, 0xFF, 0xA5, 0x01], payload);
    }

    [Fact]
    public void DecodeHilOutputsReadsResponsePayload()
    {
        byte[] payload = new byte[15];
        BitConverter.GetBytes(1234u).CopyTo(payload, 0);
        payload[4] = 6;
        payload[5] = (byte)ControlRuntimeMode.HilSim;
        BitConverter.GetBytes((ushort)2000).CopyTo(payload, 6);
        BitConverter.GetBytes((ushort)1800).CopyTo(payload, 8);
        BitConverter.GetBytes((ushort)900).CopyTo(payload, 10);
        payload[12] = unchecked((byte)-1);
        payload[13] = 3;
        payload[14] = 0;

        byte[] raw = EscProtocol.BuildFrame(4, CommOpcode.HilGetOutputs, 0, payload, CommFrameType.Response, CommStatus.Ok);
        HilOutputs outputs = EscProtocol.DecodeHilOutputs(EscProtocol.Parse(raw));

        Assert.Equal(1234u, outputs.TargetTickMs);
        Assert.Equal(ControlRuntimeMode.HilSim, outputs.Mode);
        Assert.Equal(1800, outputs.MeasuredRpm);
        Assert.Equal(-1, outputs.CommutationStep);
    }
}
