using Esc.Protocol;

namespace Esc.Bridge;

public sealed record CommandResult(bool Success, CommStatus? DeviceStatus, string Message)
{
    public static CommandResult Ok(string message = "Correcto") => new(true, CommStatus.Ok, message);

    public static CommandResult FromStatus(CommStatus status)
    {
        return status == CommStatus.Ok
            ? Ok()
            : new CommandResult(false, status, $"ESC respondio: {status}.");
    }

    public static CommandResult Failed(string message) => new(false, null, message);
}
