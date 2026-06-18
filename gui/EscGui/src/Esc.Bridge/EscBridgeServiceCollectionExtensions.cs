using Esc.Transport;
using Microsoft.Extensions.DependencyInjection;

namespace Esc.Bridge;

public static class EscBridgeServiceCollectionExtensions
{
    public static IServiceCollection AddEscBridge(this IServiceCollection services)
    {
        services.AddSingleton<IEscDeviceEnumerator, HidSharpDeviceEnumerator>();
        services.AddSingleton<IEscTransport, HidSharpEscTransport>();
        services.AddSingleton<TelemetryStore>();
        services.AddSingleton<EscBridgeService>();
        services.AddHostedService<EscBridgeWorker>();
        return services;
    }
}
