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
        Task sineMaintenance = RunSineMaintenanceAsync(stoppingToken);
        Task telemetryPump = RunTelemetryPumpAsync(stoppingToken);
        try
        {
            while (!stoppingToken.IsCancellationRequested)
            {
                try
                {
                    await _bridge.ScanAsync(stoppingToken).ConfigureAwait(false);
                    if (_bridge.IsTransportOpen &&
                        _bridge.Snapshot.State is DeviceConnectionState.Connected or DeviceConnectionState.Error)
                    {
                        await _bridge.RefreshStatusAsync(
                            stoppingToken, responseDeadlineMs: 300).ConfigureAwait(false);
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
        finally
        {
            try
            {
                await sineMaintenance.ConfigureAwait(false);
                await telemetryPump.ConfigureAwait(false);
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
            }
        }
    }

    private async Task RunTelemetryPumpAsync(CancellationToken stoppingToken)
    {
        while (!stoppingToken.IsCancellationRequested)
        {
            try
            {
                await _bridge.ReadTelemetryOnceAsync(stoppingToken).ConfigureAwait(false);
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
                return;
            }
            catch (Exception ex)
            {
                _logger.LogDebug(ex, "ESC telemetry read failed.");
            }

            await Task.Delay(10, stoppingToken).ConfigureAwait(false);
        }
    }

    private async Task RunSineMaintenanceAsync(CancellationToken stoppingToken)
    {
        while (!stoppingToken.IsCancellationRequested)
        {
            try
            {
                await _bridge.MaintainSineDriveAsync(stoppingToken).ConfigureAwait(false);
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
                return;
            }
            catch (Exception ex)
            {
                _logger.LogDebug(ex, "Sine-drive keepalive failed.");
            }

            await Task.Delay(350, stoppingToken).ConfigureAwait(false);
        }
    }
}
