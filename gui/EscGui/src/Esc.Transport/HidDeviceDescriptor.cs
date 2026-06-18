namespace Esc.Transport;

public sealed record HidDeviceDescriptor(
    string DevicePath,
    int VendorId,
    int ProductId,
    string FriendlyName,
    string Manufacturer,
    string ProductName,
    string SerialNumber,
    int MaxInputReportLength,
    int MaxOutputReportLength)
{
    public string DisplayName => string.IsNullOrWhiteSpace(ProductName)
        ? FriendlyName
        : ProductName;
}
