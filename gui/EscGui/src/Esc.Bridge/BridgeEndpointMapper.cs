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
        group.MapGet("/telemetry/{parameter}", (string parameter, EscBridgeService bridge) =>
            bridge.GetTelemetrySamples(ParseLogParam(parameter)));

        group.MapPost("/connect", async (ConnectRequest request, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.ConnectAsync(request.DevicePath, cancellationToken));

        group.MapPost("/disconnect", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
        {
            await bridge.DisconnectAsync(cancellationToken);
            return Results.Ok();
        });

        group.MapPost("/mode", async (ModeRequest request, EscBridgeService bridge) =>
        {
            CommandResult result = await bridge.SetModeAsync(request.Mode);
            return result.Success ? Results.Ok(bridge.Snapshot) : Results.Conflict(result);
        });

        group.MapPost("/run", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.RunAsync(cancellationToken));
        group.MapPost("/stop", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.StopAsync(cancellationToken));
        group.MapPost("/estop", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.EmergencyStopAsync(cancellationToken));
        group.MapPost("/set-speed", async (SetSpeedRequest request, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.SetSpeedRpmAsync(request.Rpm, cancellationToken));
        group.MapPost("/sine-drive", async (SineDriveRequest request, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.SetSineDriveAsync(request.ElectricalFrequencyHz, request.AmplitudePercent, cancellationToken));
        group.MapPost("/step-test", async (StepTestSettings request, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.SetStepTestAsync(request, cancellationToken));
        group.MapPost("/freewheel/start", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.StartFreewheelAsync(cancellationToken));
        group.MapPost("/freewheel/stop", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.StopFreewheelAsync(cancellationToken));
        group.MapPost("/zc-diag/auto-detect", async (ZcAutoDetectRequest? request, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.RunZcAutoDetectAsync(request?.DutyPercent ?? CommConstants.DefaultZcAutoDetectDutyPercent, cancellationToken));
        group.MapGet("/zc-map", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.GetZcMapAsync(cancellationToken));
        group.MapPost("/zc-map", async (ZcChannelMap map, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.SetZcMapAsync(map, cancellationToken));

        group.MapGet("/config/{parameter}", async (string parameter, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.GetConfigAsync(ParseConfigParam(parameter), cancellationToken));
        group.MapGet("/controller-config", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.RefreshControllerConfigAsync(cancellationToken));
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
        group.MapPost("/log/{parameter}/start", async (string parameter, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.StartLogAsync(ParseLogParam(parameter), cancellationToken));
        group.MapPost("/log/{parameter}/stop", async (string parameter, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.StopLogAsync(ParseLogParam(parameter), cancellationToken));

        group.MapGet("/load-sweep", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.RefreshLoadSweepAsync(cancellationToken));
        group.MapPost("/load-sweep/start", async (LoadSweepRequest request, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.StartLoadSweepAsync(request, cancellationToken));
        group.MapPost("/load-sweep/abort", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.AbortLoadSweepAsync(cancellationToken));
        group.MapGet("/load-sweep/csv", (EscBridgeService bridge) =>
            Results.Text(bridge.LoadSweepCsv(), "text/csv"));

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
            "KP" or "KP_RPM" => ConfigParam.KpRpm,
            "KI" or "KI_RPM" => ConfigParam.KiRpm,
            "KD" or "KD_RPM" => ConfigParam.KdRpm,
            "MAX_SPEED" => ConfigParam.MaxSpeed,
            "MIN_SPEED" => ConfigParam.MinSpeed,
            "ZC_MAP" => ConfigParam.ZcMap,
            "SPEED_RAMP" or "RAMP" => ConfigParam.SpeedRamp,
            "ALL" => ConfigParam.All,
            _ => throw new BadHttpRequestException($"Unknown config parameter '{parameter}'.")
        };
    }

    private static LogParam ParseLogParam(string parameter)
    {
        if (Enum.TryParse(parameter, true, out LogParam parsed) && parsed != LogParam.All)
        {
            return parsed;
        }

        return parameter.ToUpperInvariant() switch
        {
            "BUS_VOLTAGE" or "VBUS" or "VOLTAGE" => LogParam.BusVoltage,
            "BUS_CURRENT" or "IBUS" or "CURRENT" => LogParam.BusCurrent,
            "DUTY" => LogParam.Duty,
            _ => throw new BadHttpRequestException($"Unknown telemetry parameter '{parameter}'.")
        };
    }

    public sealed record ConnectRequest(string? DevicePath);
    public sealed record ModeRequest(ControlMode Mode);
    public sealed record SetSpeedRequest(int Rpm);
    public sealed record SineDriveRequest(double ElectricalFrequencyHz, double AmplitudePercent);
    public sealed record ZcAutoDetectRequest(double? DutyPercent);
    public sealed record SetConfigRequest(double Value);
    public sealed record LogRateRequest(ushort RateMs);
}
