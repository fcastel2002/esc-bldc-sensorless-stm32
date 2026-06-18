using Esc.Protocol;

namespace Esc.Transport;

public interface IEscTransport : IAsyncDisposable
{
    bool IsOpen { get; }
    HidDeviceDescriptor? CurrentDevice { get; }

    Task OpenAsync(HidDeviceDescriptor descriptor, CancellationToken cancellationToken = default);
    Task CloseAsync(CancellationToken cancellationToken = default);
    Task WriteFrameAsync(byte[] frame, CancellationToken cancellationToken = default);
    Task<EscFrame?> ReadFrameAsync(TimeSpan timeout, CancellationToken cancellationToken = default);
}
