namespace Esc.Protocol;

public sealed class EscProtocolException : Exception
{
    public EscProtocolException(string message)
        : base(message)
    {
    }
}
