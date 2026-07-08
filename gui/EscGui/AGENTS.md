# GUI Agent Instructions

## Scope

- This directory is the canonical local Blazor GUI and bridge project.
- Projects target `net10.0`.
- Use individual `.csproj` paths for commands. Do not assume a Visual Studio `.sln` workflow is required.
- The GUI shares the 64-byte ESC binary protocol with `firmware/`.

## Commands

- Run GUI from repo root: `dotnet run --project gui\EscGui\src\Esc.Web\Esc.Web.csproj`
- Expected local URL: `http://localhost:5187`
- Build GUI from repo root: `dotnet build gui\EscGui\src\Esc.Web\Esc.Web.csproj`
- Run tests from repo root: `dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj`
- Focus protocol tests: `dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj --filter "FullyQualifiedName~ProtocolTests"`
- Focus bridge tests: `dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj --filter "FullyQualifiedName~BridgeTests"`
- Publish Windows GUI exe from repo root: `powershell -ExecutionPolicy Bypass -File .\gui\EscGui\publish-gui-win-x64.ps1`

## Stack

- Blazor local web app.
- .NET `net10.0`.
- HidSharp for HID transport.
- xUnit for tests.

## Bridge Invariants

- Only `EscBridgeService` owns HID access.
- UI, HTTP endpoints, WebSocket, telemetry worker, and UDP/Simulink paths must go through the bridge.
- Do not access `HidSharpEscTransport` directly outside the bridge ownership boundary.
- HID I/O in the bridge is serialized by `_ioLock`.
- Bridge state and snapshot updates use `_stateGate`.
- If a bridge change affects observable UI state, update `Snapshot` and call `Notify()` so Blazor subscribers receive `SnapshotChanged`.
- `EscBridgeWorker` scans devices, reads at most one telemetry frame, and refreshes status on a 500 ms loop.
- Telemetry is not an independent high-rate stream.
- `EmergencyStopAsync()` is intentionally allowed regardless of control mode.

## UI And Control Notes

- `TelemetryStore` keeps up to 2000 samples per variable.
- The speed chart is manual SVG in `SpeedChart.razor`.
- `ScottPlot.Blazor` is referenced but not used by the current chart.
- `wwwroot/controls.json` is loaded once and cached by `ModularControlProvider`; restart the GUI after editing it.
- Modular controls currently recognize `set_speed_rpm`, `set_config`, `reset_config`, `log_speed_start`, and `log_speed_stop`.
- Config changes from controls still need explicit `SaveConfigAsync` to persist.

## Control Modes

- `GuiControl` permits GUI control and configuration.
- `SimulinkControl` blocks GUI control except ESTOP and permits UDP/Simulink control.
- `MonitorOnly` blocks control paths except ESTOP.

## HIL / UDP

- UDP bridge listens on `127.0.0.1:5055` in `HilUdpBridgeService`.
- HIL input CSVs include `seq,speed_rpm,enable`, `PIL,speed_rpm,enable`, `PIL,seq,speed_rpm,enable`, and legacy `seq,speed_rpm,reserved,load_torque,flags,enable`.
- High-rate UDP packets are latest-value signals, not FIFO.
- Coalesced `SETPOINT`/HIL inputs keep only the newest value while HID is busy.
- `ESTOP` has priority.

## Useful Files

- Startup and DI: `src/Esc.Web/Program.cs`, `src/Esc.Bridge/EscBridgeServiceCollectionExtensions.cs`
- Bridge service: `src/Esc.Bridge/EscBridgeService.cs`
- HTTP endpoints: `src/Esc.Bridge/BridgeEndpointMapper.cs`
- Main UI: `src/Esc.Web/Components/Pages/Home.razor`
- PIL monitor page: `src/Esc.Web/Components/Pages/PilFrames.razor`
- Protocol constants and serializers: `src/Esc.Protocol/CommConstants.cs`, `src/Esc.Protocol/EscProtocol.cs`
- HIL models and UDP: `src/Esc.Protocol/HilModels.cs`, `src/Esc.Bridge/HilUdpBridgeService.cs`
- Useful tests: `tests/Esc.Tests/ProtocolTests.cs`, `tests/Esc.Tests/BridgeTests.cs`

## Protocol Change Checklist

- Update `src/Esc.Protocol/CommConstants.cs` and `src/Esc.Protocol/EscProtocol.cs`.
- Update bridge/UI state if command behavior is observable.
- Update `tests/Esc.Tests/`, especially protocol and bridge coverage.
- Update `firmware/COMM_PROTOCOL.md`.
- Coordinate with firmware changes under `firmware/Core/Src/comm_protocol.c`.
