# GUI Agent Instructions

## Scope

- This directory is the canonical local Blazor GUI and bridge project.
- Projects target `net10.0`.
- Use individual `.csproj` paths for official commands. `EscGui.slnx` exists as an auxiliary Visual Studio solution file, but do not assume a solution workflow is required.
- The GUI shares the 64-byte ESC binary protocol with `firmware/`.

## Commands

- Run GUI from repo root: `dotnet run --project gui\EscGui\src\Esc.Web\Esc.Web.csproj`
- Expected local URL: `http://localhost:5187`
- Build GUI from repo root: `dotnet build gui\EscGui\src\Esc.Web\Esc.Web.csproj`
- Run tests from repo root: `dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj`
- Focus protocol tests: `dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj --filter "FullyQualifiedName~ProtocolTests"`
- Focus bridge tests: `dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj --filter "FullyQualifiedName~BridgeTests"`
- Focus app shell tests: `dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj --filter "FullyQualifiedName~AppShellTests"`
- Publish Windows GUI exe from repo root: `powershell -ExecutionPolicy Bypass -File .\gui\EscGui\publish-gui-win-x64.ps1`
- Before publishing, stop any running `Esc.Web` process without asking (the published `Esc.Web.dll` is locked while the app runs and the publish fails with `MSB3021`): `Get-Process -Name "Esc.Web" -ErrorAction SilentlyContinue | Stop-Process -Force`

## Stack

- Blazor local web app.
- .NET `net10.0`.
- HidSharp for HID transport.
- xUnit for tests.

## Bridge Invariants

- Only `EscBridgeService` owns HID access.
- UI, HTTP endpoints, WebSocket and the background worker must go through the bridge.
- Do not access `HidSharpEscTransport` directly outside the bridge ownership boundary.
- HID I/O in the bridge is serialized by `_ioLock`.
- Bridge state and snapshot updates use `_stateGate`.
- If a bridge change affects observable UI state, update `Snapshot` and call `Notify()` so Blazor subscribers receive `SnapshotChanged`.
- `EscBridgeWorker` scans devices and refreshes status every 500 ms, pumps telemetry frames every 10 ms, and every 350 ms renews manual outputs and follows an active load sweep.
- `EmergencyStopAsync()` is intentionally allowed regardless of control mode.

## UI And Control Notes

- `TelemetryStore` keeps up to 2000 samples per variable.
- The speed chart is manual SVG in `SpeedChart.razor`.
- `ScottPlot.Blazor` is referenced but not used by the current chart.
- `wwwroot/controls.json` is loaded once and cached by `ModularControlProvider`; restart the GUI after editing it.
- Modular controls currently recognize `set_speed_rpm`, `set_config`, `reset_config`, `log_speed_start`, and `log_speed_stop`.
- Config changes from controls still need explicit `SaveConfigAsync` to persist.
- Keep the interactive `HeadOutlet` in `Components/App.razor` before `ResourcePreloader`, stylesheet links, and `ImportMap`; it must never be the last child of `<head>`. Browser extensions and translation tools can otherwise detach Blazor component 1 during startup. Preserve `AppShellTests` as the enforcement guard.

## Control Modes

- `GuiControl` permits GUI control and configuration.
- `MonitorOnly` blocks control paths except ESTOP and load-sweep ABORT.
- The HIL/PIL UDP bridge, validation runs and `SimulinkControl` mode were removed in spec 002.

## Load Curves

- `/curvas` (`LoadCurves.razor`) starts/aborts `LOAD_SWEEP`, overlays the current sweep with saved and imported curves (`SaveCurrentLoadCurve`, `ImportLoadCurveCsv`), selects one as tare (`SetLoadCurveTare`) and exports CSV.
- The bridge keeps the points copied from the MCU table (`LoadSweepSnapshot.Points`); `StartLoadSweepAsync` clears them. Saved curves and the tare never reach the MCU.
- Load current is `(I_bus - I_bus,tare) / d` at equal speed (`LoadCurveMetric.LoadCurrentVsTare`); never divide the absolute bus current by the duty.
- `LoadCurveAnalysis`, `LoadSweepCsvWriter` and `LoadSweepCsvReader` are pure helpers; keep them free of bridge state.

## Useful Files

- Startup and DI: `src/Esc.Web/Program.cs`, `src/Esc.Bridge/EscBridgeServiceCollectionExtensions.cs`
- Bridge service: `src/Esc.Bridge/EscBridgeService.cs`
- HTTP endpoints: `src/Esc.Bridge/BridgeEndpointMapper.cs`
- Main UI: `src/Esc.Web/Components/Pages/Home.razor`
- Load curves page: `src/Esc.Web/Components/Pages/LoadCurves.razor`
- Protocol constants and serializers: `src/Esc.Protocol/CommConstants.cs`, `src/Esc.Protocol/EscProtocol.cs`
- Load sweep models and helpers: `src/Esc.Protocol/LoadSweepModels.cs`, `src/Esc.Bridge/LoadCurveAnalysis.cs`, `src/Esc.Bridge/LoadSweepCsvWriter.cs`
- Useful tests: `tests/Esc.Tests/ProtocolTests.cs`, `tests/Esc.Tests/BridgeTests.cs`

## Protocol Change Checklist

- Update `src/Esc.Protocol/CommConstants.cs` and `src/Esc.Protocol/EscProtocol.cs`.
- Update bridge/UI state if command behavior is observable.
- Update `tests/Esc.Tests/`, especially protocol and bridge coverage.
- Update `firmware/COMM_PROTOCOL.md`.
- Coordinate with firmware changes under `firmware/Core/Src/comm_protocol.c`.
