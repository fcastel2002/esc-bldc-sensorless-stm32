# Agent Instructions

Keep `AGENTS.md` and `CLAUDE.md` in the repository root synchronized when changing either one.

## Repo Shape

- This is a mixed STM32 firmware + local Blazor GUI ESC project. Firmware and GUI share a 64-byte binary protocol; protocol changes usually require coordinated edits in both trees.
- Firmware lives in `firmware/` and targets STM32F103C8T6 with CMake/Ninja and `arm-none-eabi-gcc`.
- GUI lives in `gui/EscGui/`; there is no solution file, so use the individual `.csproj` paths. Projects target `net10.0`.
- High-value docs before deep changes: `firmware/COMM_PROTOCOL.md` for frames/opcodes, `docs/PROJECT_FLOW.md` for firmware flow, `gui/EscGui/GUI_AGENT_CONTEXT.md` for GUI ownership, `gui/EscGui/BRIDGE_WALKTHROUGH.md` for bridge/HID/UDP details.

## Commands

- Firmware configure/build from `firmware/`: `cmake --preset Debug` then `cmake --build --preset Debug`.
- Firmware release build from `firmware/`: `cmake --preset Release` then `cmake --build --preset Release`.
- Firmware post-build Doxygen generation is enabled by default via `ESC_BUILD_CODE_DOCS`; disable for faster local builds with `cmake --preset Debug -DESC_BUILD_CODE_DOCS=OFF`.
- Run GUI from repo root: `dotnet run --project gui\EscGui\src\Esc.Web\Esc.Web.csproj`, then open `http://localhost:5187`.
- Build GUI project: `dotnet build gui\EscGui\src\Esc.Web\Esc.Web.csproj`.
- Run GUI tests: `dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj`.
- Run a focused GUI test by filter: `dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj --filter "FullyQualifiedName~ProtocolTests"` or replace `ProtocolTests` with the target class/method substring.
- Publish Windows GUI exe from repo root: `powershell -ExecutionPolicy Bypass -File .\gui\EscGui\publish-gui-win-x64.ps1`; output is `gui\EscGui\publish\win-x64\Esc.Web.exe` plus required web assets in the same folder.

## Git Workflow

- For each requested feature, create a new feature branch from the current base unless the user says otherwise.
- Commit each completed sub-feature separately. Commit messages must be clear and identify the sub-feature precisely.
- Before each commit, inspect `git status` and `git diff`; stage only files changed for that sub-feature and never include unrelated user changes.
- Push the feature branch after committing when the user has asked for the feature workflow to include pushing.
- When the overall feature is complete and verified, propose merging the feature branch into `main`; do not merge unless the user confirms.

## Firmware Invariants

- Communication transport is selected only at boot by `PB8`: HIGH/open is UART on `USART1 PB6/PB7 115200 8N1`; LOW is USB FS Custom HID on `PA11/PA12`. Do not add runtime transport switching.
- All protocol frames are fixed 64 bytes, little-endian, magic `0xEC 0xB1`, version `0x01`, CRC16-CCITT-FALSE over bytes `0..61`.
- Main protocol effects live in `firmware/Core/Src/comm_protocol.c`; transport selection is in `comm_transport.c`; periodic communication/telemetry facade is in `comm.c`.
- `foc_startup()` is historical naming: it is open-loop sinusoidal/SPWM startup, not field-oriented control and not a current loop.
- `SET_CONFIG` and `RESET_CONFIG` update active RAM only; flash persistence requires explicit `SAVE_CONFIG`.
- PI gains are floats internally but protocol payloads use `int16` hundredths, e.g. `135` means `1.35`.
- In `CLOSEDLOOP`, `PWM_FREQ`, `KP`, `KI`, and `KD` apply hot; other config changes use the deferred `CONFIG` path.
- `KD` acts on measured-speed derivative, not error derivative, to avoid setpoint derivative kick.
- `CURRENT_LIMIT` and `TEMP_LIMIT` are protocol placeholders and return/use `NOT_IMPLEMENTED`; current and temperature telemetry are placeholders too.
- CubeMX-generated files may contain `USER CODE BEGIN/END` regions; avoid editing generated sections outside those regions unless the task explicitly requires it.
- C formatting/lint config exists at repo root as `.clang-format` and `.clang-tidy`.

## GUI/Bridge Invariants

- Only `EscBridgeService` owns HID access. UI, HTTP endpoints, WebSocket, telemetry worker, and UDP/Simulink must all go through the bridge, never directly through `HidSharpEscTransport`.
- HID I/O in the bridge is serialized by `_ioLock`; state/snapshot updates use `_stateGate`.
- If a bridge change affects observable UI state, update `Snapshot` and call `Notify()` so Blazor subscribers receive `SnapshotChanged`.
- `EscBridgeWorker` scans devices, reads at most one telemetry frame, and refreshes status on a 500 ms loop; telemetry is not an independent high-rate stream.
- `TelemetryStore` keeps up to 2000 samples per variable. The speed chart is manual SVG in `SpeedChart.razor`; `ScottPlot.Blazor` is referenced but not used by the current chart.
- `wwwroot/controls.json` is loaded once and cached by `ModularControlProvider`; restart the GUI after editing it.
- Modular controls currently recognize `set_speed_rpm`, `set_config`, `reset_config`, `log_speed_start`, and `log_speed_stop`. Config changes from controls still need explicit `SaveConfigAsync` to persist.
- Control modes matter: `GuiControl` permits GUI control/config, `SimulinkControl` blocks GUI control except ESTOP and permits UDP/Simulink control, `MonitorOnly` blocks control paths except ESTOP.
- `EmergencyStopAsync()` is intentionally allowed regardless of control mode.

## Protocol Change Checklist

- Firmware: update `firmware/Core/Src/comm_protocol.c` and related config/control modules.
- GUI protocol: update `gui/EscGui/src/Esc.Protocol/CommConstants.cs` and `EscProtocol.cs`.
- Bridge/UI: update `EscBridgeService.cs`, `BridgeEndpointMapper.cs`, `Home.razor` or `controls.json` as needed.
- Tests/docs: update `gui/EscGui/tests/Esc.Tests/` and `firmware/COMM_PROTOCOL.md`.
- Decide whether any new bridge command must call `RefreshStatusAsync`; most state-changing commands do.

## HIL / Simulink

- UDP bridge listens on `127.0.0.1:5055` in `HilUdpBridgeService`.
- HIL input CSVs include `seq,speed_rpm,enable`, `PIL,speed_rpm,enable`, `PIL,seq,speed_rpm,enable`, and legacy `seq,speed_rpm,reserved,load_torque,flags,enable`.
- High-rate UDP packets are latest-value signals, not FIFO. Coalesced `SETPOINT`/HIL inputs keep only the newest value while HID is busy; `ESTOP` has priority.
- Firmware HIL timeout is 50 ms without input, then it stops logical HIL run and returns to `IDLE`.

## Fast File Map

- Firmware state machine: `firmware/Core/Src/state_machine.c`, `state_machine.h`.
- Motor control and stall/PI logic: `motor_control.c`; speed measurement: `speed_sensor.c`; commutation: `bldc_driver.c`; HAL callbacks: `isr_callbacks.c`.
- Firmware config and flash persistence: `hard_config.c/.h`, `flash_config.c/.h`.
- GUI startup/DI/endpoints: `gui/EscGui/src/Esc.Web/Program.cs`, `gui/EscGui/src/Esc.Bridge/EscBridgeServiceCollectionExtensions.cs`, `BridgeEndpointMapper.cs`.
- Main UI page: `gui/EscGui/src/Esc.Web/Components/Pages/Home.razor`; PIL monitor page: `Components/Pages/PilFrames.razor`.
- HIL models and UDP: `gui/EscGui/src/Esc.Protocol/HilModels.cs`, `gui/EscGui/src/Esc.Bridge/HilUdpBridgeService.cs`.
