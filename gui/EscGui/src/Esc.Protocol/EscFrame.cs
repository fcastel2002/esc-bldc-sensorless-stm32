namespace Esc.Protocol;

public sealed record EscFrame(
    byte[] Raw,
    byte Version,
    CommFrameType Type,
    byte Sequence,
    CommOpcode Opcode,
    byte Parameter,
    CommStatus Status,
    byte[] Payload,
    ushort Crc)
{
    public bool IsOk => Status == CommStatus.Ok;
}
