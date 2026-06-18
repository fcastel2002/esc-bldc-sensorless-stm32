using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;

namespace Esc.Bridge;

public sealed class EscBridgeWorker : BackgroundService
{
    private readonly EscBridgeService _bridge;
    private readonly ILogger<EscBridgeWorker> _logger;

    public EscBridgeWorker(EscBridgeService bridge, ILogger<EscBridgeWorker> logger)
    {
        _bridge = bridge;
        _logger = logger;
    }

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        while (!stoppingToken.IsCancellationRequested)
        {
            try
            {
                await _bridge.ScanAsync(stoppingToken).ConfigureAwait(false);
                await _bridge.ReadTelemetryOnceAsync(stoppingToken).ConfigureAwait(false);

                if (_bridge.Snapshot.State == DeviceConnectionState.Connected)
                {
                    await _bridge.RefreshStatusAsync(stoppingToken).ConfigureAwait(false);
                }
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
                return;
            }
            catch (Exception ex)
            {
                _logger.LogDebug(ex, "ESC bridge background iteration failed.");
            }

            await Task.Delay(500, stoppingToken).ConfigureAwait(false);
        }
    }
}
