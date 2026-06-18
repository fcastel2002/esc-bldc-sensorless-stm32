using Esc.Protocol;
using HidSharp;

namespace Esc.Transport;

public sealed class HidSharpEscTransport : IEscTransport
{
    private HidStream? _stream;

    public bool IsOpen => _stream is not null;
    public HidDeviceDescriptor? CurrentDevice { get; private set; }

    public async Task OpenAsync(HidDeviceDescriptor descriptor, CancellationToken cancellationToken = default)
    {
        await CloseAsync(cancellationToken).ConfigureAwait(false);

        HidDevice? device = DeviceList.Local
            .GetHidDevices(descriptor.VendorId, descriptor.ProductId, null, null)
            .FirstOrDefault(candidate => string.Equals(candidate.DevicePath, descriptor.DevicePath, StringComparison.Ordinal));

        device ??= DeviceList.Local
            .GetHidDevices(descriptor.VendorId, descriptor.ProductId, null, null)
            .FirstOrDefault();

        if (device is null)
        {
            throw new InvalidOperationException("ESC HID device is no longer available.");
        }

        _stream = await Task.Run(device.Open, cancellationToken).ConfigureAwait(false);
        _stream.ReadTimeout = 250;
        _stream.WriteTimeout = 250;
        CurrentDevice = descriptor;
    }

    public Task CloseAsync(CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();

        _stream?.Dispose();
        _stream = null;
        CurrentDevice = null;
        return Task.CompletedTask;
    }

    public async Task WriteFrameAsync(byte[] frame, CancellationToken cancellationToken = default)
    {
        if (frame.Length != CommConstants.FrameSize)
        {
            throw new ArgumentException("ESC frames must be exactly 64 bytes.", nameof(frame));
        }

        HidStream stream = RequireStream();
        int reportLength = Math.Max(CurrentDevice?.MaxOutputReportLength ?? 0, CommConstants.FrameSize + 1);
        var report = new byte[reportLength];

        // Windows HID APIs expect a leading report ID byte. The current firmware uses report ID 0.
        frame.CopyTo(report, reportLength >= CommConstants.FrameSize + 1 ? 1 : 0);

        await Task.Run(() => stream.Write(report), cancellationToken).ConfigureAwait(false);
    }

    public async Task<EscFrame?> ReadFrameAsync(TimeSpan timeout, CancellationToken cancellationToken = default)
    {
        HidStream stream = RequireStream();
        int reportLength = Math.Max(CurrentDevice?.MaxInputReportLength ?? 0, CommConstants.FrameSize);
        var report = new byte[reportLength];

        int previousTimeout = stream.ReadTimeout;
        stream.ReadTimeout = Math.Max(1, (int)timeout.TotalMilliseconds);

        try
        {
            int count = await Task.Run(() => stream.Read(report), cancellationToken).ConfigureAwait(false);
            if (count <= 0)
            {
                return null;
            }

            return EscProtocol.Parse(NormalizeReport(report.AsSpan(0, count)));
        }
        catch (TimeoutException)
        {
            return null;
        }
        finally
        {
            if (_stream is not null)
            {
                _stream.ReadTimeout = previousTimeout;
            }
        }
    }

    public async ValueTask DisposeAsync()
    {
        await CloseAsync().ConfigureAwait(false);
    }

    private HidStream RequireStream()
    {
        return _stream ?? throw new InvalidOperationException("ESC HID stream is not open.");
    }

    private static ReadOnlySpan<byte> NormalizeReport(ReadOnlySpan<byte> report)
    {
        if (report.Length >= CommConstants.FrameSize + 1 && report[0] == 0)
        {
            return report.Slice(1, CommConstants.FrameSize);
        }

        if (report.Length >= CommConstants.FrameSize)
        {
            return report[..CommConstants.FrameSize];
        }

        return report;
    }
}
