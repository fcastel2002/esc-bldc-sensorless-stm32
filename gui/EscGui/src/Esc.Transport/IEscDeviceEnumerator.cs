namespace Esc.Transport;

public interface IEscDeviceEnumerator
{
    Task<IReadOnlyList<HidDeviceDescriptor>> ListAsync(CancellationToken cancellationToken = default);
}
