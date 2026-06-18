using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using Esc.Protocol;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Routing;

namespace Esc.Bridge;

public static class BridgeEndpointMapper
{
    private static readonly JsonSerializerOptions JsonOptions = new(JsonSerializerDefaults.Web);

    public static IEndpointRouteBuilder MapEscBridgeEndpoints(this IEndpointRouteBuilder endpoints)
    {
        RouteGroupBuilder group = endpoints.MapGroup("/api/bridge");

        group.MapGet("/snapshot", (EscBridgeService bridge) => bridge.Snapshot);
        group.MapGet("/devices", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
        {
            await bridge.ScanAsync(cancellationToken);
            return bridge.Snapshot.Devices;
        });
        group.MapGet("/telemetry/speed", (EscBridgeService bridge) => bridge.SpeedSamples);

        group.MapPost("/connect", async (ConnectRequest request, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.ConnectAsync(request.DevicePath, cancellationToken));

        group.MapPost("/disconnect", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
        {
            await bridge.DisconnectAsync(cancellationToken);
            return Results.Ok();
        });

        group.MapPost("/mode", async (ModeRequest request, EscBridgeService bridge) =>
        {
            await bridge.SetModeAsync(request.Mode);
            return Results.Ok(bridge.Snapshot);
        });

        group.MapPost("/run", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.RunAsync(cancellationToken));
        group.MapPost("/stop", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.StopAsync(cancellationToken));
        group.MapPost("/estop", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.EmergencyStopAsync(cancellationToken));
        group.MapPost("/set-speed", async (SetSpeedRequest request, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.SetSpeedRpmAsync(request.Rpm, cancellationToken));

        group.MapGet("/config/{parameter}", async (string parameter, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.GetConfigAsync(ParseConfigParam(parameter), cancellationToken));
        group.MapPost("/config/{parameter}", async (string parameter, SetConfigRequest request, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.SetConfigAsync(ParseConfigParam(parameter), request.Value, cancellationToken));
        group.MapPost("/config/{parameter}/reset", async (string parameter, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.ResetConfigAsync(ParseConfigParam(parameter), cancellationToken));
        group.MapPost("/config/{parameter}/save", async (string parameter, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.SaveConfigAsync(ParseConfigParam(parameter), cancellationToken));

        group.MapPost("/log/rate", async (LogRateRequest request, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.SetLogRateAsync(request.RateMs, cancellationToken));
        group.MapPost("/log/speed/start", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.StartSpeedLogAsync(cancellationToken));
        group.MapPost("/log/speed/stop", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.StopSpeedLogAsync(cancellationToken));

        endpoints.Map("/ws/bridge", HandleWebSocketAsync);
        return endpoints;
    }

    private static async Task HandleWebSocketAsync(HttpContext context, EscBridgeService bridge)
    {
        if (!context.WebSockets.IsWebSocketRequest)
        {
            context.Response.StatusCode = StatusCodes.Status400BadRequest;
            return;
        }

        using WebSocket socket = await context.WebSockets.AcceptWebSocketAsync();
        while (!context.RequestAborted.IsCancellationRequested && socket.State == WebSocketState.Open)
        {
            string json = JsonSerializer.Serialize(bridge.Snapshot, JsonOptions);
            byte[] bytes = Encoding.UTF8.GetBytes(json);
            await socket.SendAsync(bytes, WebSocketMessageType.Text, WebSocketMessageFlags.EndOfMessage, context.RequestAborted);
            await Task.Delay(500, context.RequestAborted);
        }
    }

    private static ConfigParam ParseConfigParam(string parameter)
    {
        if (Enum.TryParse(parameter, true, out ConfigParam parsed))
        {
            return parsed;
        }

        return parameter.ToUpperInvariant() switch
        {
            "PWM_FREQ" => ConfigParam.PwmFreq,
            "POLE_PAIRS" => ConfigParam.PolePairs,
            "KP" => ConfigParam.Kp,
            "KI" => ConfigParam.Ki,
            "KD" => ConfigParam.Kd,
            "MAX_SPEED" => ConfigParam.MaxSpeed,
            "MIN_SPEED" => ConfigParam.MinSpeed,
            "ALL" => ConfigParam.All,
            _ => throw new BadHttpRequestException($"Unknown config parameter '{parameter}'.")
        };
    }

    public sealed record ConnectRequest(string? DevicePath);
    public sealed record ModeRequest(ControlMode Mode);
    public sealed record SetSpeedRequest(int Rpm);
    public sealed record SetConfigRequest(double Value);
    public sealed record LogRateRequest(ushort RateMs);
}
