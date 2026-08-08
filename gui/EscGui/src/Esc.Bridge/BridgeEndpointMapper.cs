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

        group.MapGet("/config/{parameter}", async (string parameter, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.GetConfigAsync(ParseConfigParam(parameter), cancellationToken));
        group.MapGet("/validation-reference", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.RefreshValidationReferenceAsync(cancellationToken));
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

        group.MapPost("/hil/start", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.HilStartAsync(cancellationToken: cancellationToken));
        group.MapPost("/hil/stop", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.HilStopAsync(cancellationToken));
        group.MapPost("/hil/inputs", async (HilInputRequest request, EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.HilSetInputsAsync(new HilInputs(
                request.SpeedRpm,
                request.LoadTorque,
                request.Flags,
                request.Enable),
                cancellationToken));
        group.MapGet("/hil/outputs", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
            await bridge.HilGetOutputsAsync(cancellationToken));

        RouteGroupBuilder validation = endpoints.MapGroup("/api/validation");
        validation.MapGet("/runs", async (ValidationRunService runs, CancellationToken cancellationToken) =>
            await runs.ListAsync(cancellationToken));
        validation.MapGet("/runs/{id:guid}", async (Guid id, ValidationRunService runs, CancellationToken cancellationToken) =>
        {
            ValidationRunDetail? run = await runs.GetAsync(id, cancellationToken);
            return run is null ? Results.NotFound() : Results.Ok(run);
        });
        validation.MapPost("/import", async (ValidationImportRequest request, ValidationRunService runs, CancellationToken cancellationToken) =>
        {
            ValidationRunSummary run = await runs.ImportAsync(request.Path, request.Options ?? new ValidationRunOptions(), cancellationToken);
            return Results.Created($"/api/validation/runs/{run.Id}", run);
        });
        validation.MapPost("/runs/{id:guid}/execute", async (Guid id, ValidationRunService runs, CancellationToken cancellationToken) =>
            await runs.ExecuteAsync(id, cancellationToken));
        validation.MapPatch("/runs/{id:guid}/metadata", async (Guid id, ValidationRunMetadataRequest request, ValidationRunService runs, CancellationToken cancellationToken) =>
        {
            try
            {
                return await runs.UpdateMetadataAsync(id, request.ExperimentName, request.Description, cancellationToken)
                    ? Results.NoContent()
                    : Results.NotFound();
            }
            catch (ArgumentException exception)
            {
                return Results.BadRequest(new { error = exception.Message });
            }
        });
        validation.MapDelete("/runs/{id:guid}", async (Guid id, ValidationRunService runs, CancellationToken cancellationToken) =>
        {
            try
            {
                return await runs.DeleteAsync(id, cancellationToken) ? Results.NoContent() : Results.NotFound();
            }
            catch (InvalidOperationException exception)
            {
                return Results.Conflict(new { error = exception.Message });
            }
        });

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
            "CURRENT_U" or "IU" => LogParam.CurrentU,
            "CURRENT_V" or "IV" => LogParam.CurrentV,
            "BEMF" or "BEMF_PERIOD" => LogParam.BemfPeriod,
            _ => throw new BadHttpRequestException($"Unknown telemetry parameter '{parameter}'.")
        };
    }

    public sealed record ConnectRequest(string? DevicePath);
    public sealed record ModeRequest(ControlMode Mode);
    public sealed record SetSpeedRequest(int Rpm);
    public sealed record SineDriveRequest(double ElectricalFrequencyHz, double AmplitudePercent);
    public sealed record SetConfigRequest(double Value);
    public sealed record LogRateRequest(ushort RateMs);
    public sealed record HilInputRequest(
        ushort SpeedRpm,
        ushort ZeroCrossingPeriod,
        short LoadTorque,
        byte Flags,
        bool Enable);
    public sealed record ValidationImportRequest(string Path, ValidationRunOptions? Options);
    public sealed record ValidationRunMetadataRequest(string ExperimentName, string? Description);
}
