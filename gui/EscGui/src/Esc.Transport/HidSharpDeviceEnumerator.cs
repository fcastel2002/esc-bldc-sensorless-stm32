using Esc.Protocol;
using HidSharp;

namespace Esc.Transport;

public sealed class HidSharpDeviceEnumerator : IEscDeviceEnumerator
{
    private readonly int _vendorId;
    private readonly int _productId;

    public HidSharpDeviceEnumerator()
        : this(CommConstants.DefaultVendorId, CommConstants.DefaultProductId)
    {
    }

    public HidSharpDeviceEnumerator(int vendorId, int productId)
    {
        _vendorId = vendorId;
        _productId = productId;
    }

    public Task<IReadOnlyList<HidDeviceDescriptor>> ListAsync(CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();

        var devices = DeviceList.Local
            .GetHidDevices(_vendorId, _productId, null, null)
            .Select(ToDescriptor)
            .OrderBy(device => device.DisplayName, StringComparer.OrdinalIgnoreCase)
            .ThenBy(device => device.DevicePath, StringComparer.OrdinalIgnoreCase)
            .ToArray();

        return Task.FromResult<IReadOnlyList<HidDeviceDescriptor>>(devices);
    }

    private static HidDeviceDescriptor ToDescriptor(HidDevice device)
    {
        return new HidDeviceDescriptor(
            device.DevicePath,
            device.VendorID,
            device.ProductID,
            SafeString(device.GetFriendlyName),
            SafeString(device.GetManufacturer),
            SafeString(device.GetProductName),
            SafeString(device.GetSerialNumber),
            SafeLength(device.GetMaxInputReportLength),
            SafeLength(device.GetMaxOutputReportLength));
    }

    private static string SafeString(Func<string> getter)
    {
        try
        {
            return getter() ?? string.Empty;
        }
        catch
        {
            return string.Empty;
        }
    }

    private static int SafeLength(Func<int> getter)
    {
        try
        {
            return getter();
        }
        catch
        {
            return 0;
        }
    }
}
