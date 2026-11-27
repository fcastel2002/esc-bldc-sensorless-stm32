# Agent Instructions



## Repo Shape

- This is a mixed STM32 firmware + local Blazor GUI ESC project. Firmware and GUI share a 64-byte binary protocol; protocol changes usually require coordinated edits in both trees.
- Firmware lives in `firmware/` and targets STM32F103C8T6 with CMake/Ninja and `arm-none-eabi-gcc`.
- GUI lives in `gui/EscGui/`; `gui/EscGui/EscGui.slnx` exists as an auxiliary solution file, but official commands use individual `.csproj` paths. Projects target `net10.0`.
- High-value docs before deep changes: `firmware/COMM_PROTOCOL.md` for frames/opcodes, `docs/PROJECT_FLOW.md` for firmware flow, `gui/EscGui/GUI_AGENT_CONTEXT.md` for GUI ownership, `gui/EscGui/BRIDGE_WALKTHROUGH.md` for bridge/HID/UDP details.

## Knowledge Base and PDF Usage

- For questions based on `tf_control_y_sistemas/BaseConocimiento/`, always consult the corresponding Markdown (`.md`) knowledge-base file.
- Do not open, render, OCR, extract, or otherwise inspect the source PDF when answering those questions if a Markdown version exists. This avoids unnecessary token and compute usage.
- Access a source PDF only when the user explicitly asks to inspect, convert, or verify that PDF. A missing or incomplete Markdown file is not implicit permission: report the limitation and ask before opening the PDF.

## Codebase Memory MCP

- Prefer `codebase-memory` for architectural discovery, symbol lookup, callers/callees, data flow, cross-service paths, change impact, complexity analysis, and code relationships. Prefer `Glob`/`Grep` for raw file-name or exact-text searches and for content that is not represented in the graph.
- The indexed project for this repository is `D-06.-Proyectos-esc-bldc-stm32`. Confirm it with `list_projects` rather than assuming it exists in a new environment; index the repository from the Git worktree root, not from a firmware, GUI, or simulation subdirectory.
- Do not reindex on every task. Check `index_status`; call `index_repository` when the project is missing, indexing failed, or structural results are stale after relevant code changes. Use `full` with persistence for the initial/shared index, `moderate` when semantic relationships are needed, and `fast` for routine structural refreshes.
- Start unfamiliar or cross-cutting work with `get_architecture`, scoped by `path` and limited to the needed aspects. Use `overview` first; request `clusters`, `boundaries`, `layers`, `routes`, `hotspots`, or `file_tree` only when they help answer the task.
- Use `search_graph` instead of text search to discover definitions and relationships. Prefer a natural-language `query` for discovery, `name_pattern`/`qn_pattern` for exact symbol matching, and `semantic_query` when vocabulary differs. Narrow by label or path and paginate while `has_more` is true.
- Before reading a function or class through the graph, find its exact `qualified_name` with `search_graph`, then call `get_code_snippet`. Use ordinary file reads when broader surrounding file context is required.
- Use `trace_path` for inbound/outbound call impact and `data_flow` for value propagation; enable tests explicitly when test coverage is part of the question. Use `cross_service` for HTTP, async, channel, gRPC, GraphQL, or tRPC paths when cross-repository indexes exist.
- Use `detect_changes` before non-trivial edits to identify affected callers, tests, routes, and coupled files, and after edits when an impact summary is useful. Keep the scope and depth as narrow as the task permits.
- Use `query_graph` only for multi-hop, aggregate, complexity, or hotspot questions that higher-level tools cannot answer. Call `get_graph_schema` before writing Cypher that depends on unfamiliar labels, relationships, or properties, and always bound broad queries with `LIMIT` or `max_rows`.
- Compare result counts with limits. If `search_graph.has_more` is true, paginate; if `search_code.total_results` exceeds its limit, raise the limit or narrow the query. Do not treat truncated results as exhaustive.
- After substantial structural code changes, refresh the index with persistence so `.codebase-memory/graph.db.zst` remains useful to future sessions. Do not create or update a codebase-memory ADR unless the user requests architectural documentation or the task explicitly establishes a lasting architectural decision.

## Commands

- Firmware configure/build from `firmware/`: `cmake --preset Debug` then `cmake --build --preset Debug`.
- Firmware release build from `firmware/`: `cmake --preset Release` then `cmake --build --preset Release`.
- Firmware post-build Doxygen generation is enabled by default via `ESC_BUILD_CODE_DOCS`; disable for faster local builds with `cmake --preset Debug -DESC_BUILD_CODE_DOCS=OFF`.
- Run GUI from repo root: `dotnet run --project gui\EscGui\src\Esc.Web\Esc.Web.csproj`, then open `http://localhost:5187`.
- Build GUI project: `dotnet build gui\EscGui\src\Esc.Web\Esc.Web.csproj`.
- Run GUI tests: `dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj`.
- Run a focused GUI test by filter: `dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj --filter "FullyQualifiedName~ProtocolTests"` or replace `ProtocolTests` with the target class/method substring.
- Publish Windows GUI exe from repo root: `powershell -ExecutionPolicy Bypass -File .\gui\EscGui\publish-gui-win-x64.ps1`; output is `gui\EscGui\publish\win-x64\Esc.Web.exe` plus required web assets in the same folder.
- Run the Windows GUI publish command as final verification when a change affects `gui/EscGui/`, the shared GUI-facing protocol, bridge behavior, web assets, or Windows packaging. Do not publish the GUI for changes limited to firmware, documentation, repository metadata, or `simulacion_agitador/` unless they also affect the GUI.
- Before publishing, close any running `Esc.Web` process (the published `Esc.Web.dll` is locked while the app runs and the publish fails with `MSB3021`). Kill it without asking: `Get-Process -Name "Esc.Web" -ErrorAction SilentlyContinue | Stop-Process -Force`.

## Git Workflow

- `main` is the only long-lived branch. Start each normal change from an updated `main` on a short-lived branch.
- Use `feat/`, `fix/`, `docs/`, `refactor/`, `test/`, or `chore/` followed by a lowercase ASCII kebab-case description. SSD-backed work uses `feat/NNN-kebab-case-feature`.
- Do not commit directly to `main` unless the user explicitly authorizes an emergency repository recovery.
- Use focused Conventional Commits. The pull request title must also use Conventional Commit format because it becomes the squash commit subject.
- Before each commit, inspect `git status` and `git diff`; stage only files changed for that sub-feature and never include unrelated user changes.
- Every normal change reaches `main` through a pull request. Push or open the pull request only when the user has requested remote publication.
- Required checks are `repo-hygiene`, `gui-build-test`, `protocol-guard`, and `ssd-docs`. `firmware-debug` becomes required after its existing CI failure is repaired.
- Merge pull requests with squash only and delete the head branch after merge. Do not merge unless the user confirms.
- Follow [`docs/development-workflow.md`](docs/development-workflow.md) for SSD criteria, validation evidence, branch protection, Dependabot, and release rules.

## Firmware Invariants

- Communication transport is selected only at boot by `PB8`: HIGH/open is USB FS Custom HID on `PA11/PA12`; LOW is UART on `USART1 PB6/PB7 115200 8N1`. Do not add runtime transport switching.
- All protocol frames are fixed 64 bytes, little-endian, magic `0xEC 0xB1`, version `0x01`, CRC16-CCITT-FALSE over bytes `0..61`.
- Main protocol effects live in `firmware/Core/Src/comm_protocol.c`; transport selection is in `comm_transport.c`; periodic communication/telemetry facade is in `comm.c`.
- `foc_startup()` is historical naming: it is open-loop sinusoidal/SPWM startup, not field-oriented control and not a current loop.
- `SET_CONFIG` and `RESET_CONFIG` update active RAM only; flash persistence requires explicit `SAVE_CONFIG`.
- RPM PI v2 stores gains as floats but executes Q16.16; `KP_RPM/KI_RPM` payloads use `int16` hundredths. KD is fixed at zero.
- In `CLOSEDLOOP`, `PWM_FREQ`, `KP`, `KI`, and `KD` apply hot; other config changes use the deferred `CONFIG` path.
- RPM PI v2 controls direct RPM error with canonical ARR=2000 output and trapezoidal integration; KD is not implemented.
- There is no phase-current sensing: the ADC shunt path (`current_sense.c`, ADC1/ADC2, `CURRENT_U/V` logs) was removed in spec 002 together with the `TEMP` placeholder and `BEMF_PERIOD` log. The only current measurement is the INA219 DC bus. `CURRENT_LIMIT`/`TEMP_LIMIT` and legacy `KP/KI/KD` config params return `UNKNOWN_PARAM` (their `ESCparams` fields stay for flash-layout compatibility).
- The firmware reports only measured quantities (rpm, duty, INA219 bus V/I and their dispersion). Model-based estimates live in the GUI: the load current is `(I_bus - I_bus,tare) / d` at equal speed, so the bridge's own consumption cancels (an absolute `I_bus/d` with an idle-measured offset was tried and proved biased on hardware).
- `SPEED_RAMP` (param `0x14`, persisted in `ESCparams` V6) slew-limits the PI reference in the 2 ms ISR; `0` keeps the historical step behavior. `GET_STATUS` bytes `26..30` report the reference, duty (1/10000) and the sweep-active flag.
- `LOAD_SWEEP` (`0x60`, `load_sweep.c`) runs staircase/ramp load-curve sweeps from the main loop into a 128-point RAM table. While active it blocks every opcode except `PING`, `GET_STATUS`, `STOP`, `ESTOP`, `GET_CONFIG`, `LOG_*` and `LOAD_SWEEP`; `STOP`/`ESTOP` abort it.
- INA219 DC-bus monitor (`power_monitor.c`) uses a software I2C master on `PB6/PB7` as open-drain GPIOs, not the I2C1 peripheral (verified on hardware: hardware I2C in AF mode conflicts with remapped `USART1`). Never clear `USART1_REMAP`: `USART1` would fall back to the TIM1 phase PWM pins `PA8/PA9/PA10` and break the power stage / ZC auto-detect. It runs only in USB transport mode; `GET_STATUS` bytes `10..25` and log params `0x06/0x07` carry voltage/current/diagnostics. `PB6` is also `TIM4_CH1`: start TIM4 with `motor_control_timer_start()`, never `HAL_TIM_OC_Start_IT(&htim4, ...)`.
- CubeMX-generated files may contain `USER CODE BEGIN/END` regions; avoid editing generated sections outside those regions unless the task explicitly requires it.
- C formatting/lint config exists at repo root as `.clang-format` and `.clang-tidy`.

## GUI/Bridge Invariants

- Only `EscBridgeService` owns HID access. UI, HTTP endpoints, WebSocket and the background worker must all go through the bridge, never directly through `HidSharpEscTransport`. The HIL/PIL UDP bridge, validation runs and Simulink control mode were removed in spec 002.
- HID I/O in the bridge is serialized by `_ioLock`; state/snapshot updates use `_stateGate`.
- If a bridge change affects observable UI state, update `Snapshot` and call `Notify()` so Blazor subscribers receive `SnapshotChanged`.
- `EscBridgeWorker` scans devices and refreshes status on a 500 ms loop, pumps telemetry frames every 10 ms, and every 350 ms renews manual outputs and follows an active load sweep (`PollLoadSweepAsync` copies new points from the MCU table).
- `TelemetryStore` keeps up to 2000 samples per variable. The speed chart is manual SVG in `SpeedChart.razor`; `ScottPlot.Blazor` is referenced but not used by the current chart.
- `wwwroot/controls.json` is loaded once and cached by `ModularControlProvider`; restart the GUI after editing it.
- Modular controls currently recognize `set_speed_rpm`, `set_config`, `reset_config`, `log_speed_start`, and `log_speed_stop`. Config changes from controls still need explicit `SaveConfigAsync` to persist.
- Control modes matter: `GuiControl` permits GUI control/config; `MonitorOnly` blocks control paths except ESTOP and load-sweep ABORT.
- `EmergencyStopAsync()` is intentionally allowed regardless of control mode.

## Protocol Change Checklist

- Firmware: update `firmware/Core/Src/comm_protocol.c` and related config/control modules.
- GUI protocol: update `gui/EscGui/src/Esc.Protocol/CommConstants.cs` and `EscProtocol.cs`.
- Bridge/UI: update `EscBridgeService.cs`, `BridgeEndpointMapper.cs`, `Home.razor` or `controls.json` as needed.
- Tests/docs: update `gui/EscGui/tests/Esc.Tests/` and `firmware/COMM_PROTOCOL.md`.
- Decide whether any new bridge command must call `RefreshStatusAsync`; most state-changing commands do.

## Load Curves

- The `/curvas` page (`LoadCurves.razor`) configures and starts `LOAD_SWEEP`, overlays several curves (current MCU sweep, saved sweeps, imported CSVs) for bus current, duty, power, speed dispersion or load current vs a tare, and exports CSV. Saved curves and the tare selection live only in the bridge (`LoadSweepSnapshot.SavedCurves`, `TareCurveId`), never on the MCU.
- `LoadCurveAnalysis` (curve extraction, tare subtraction/load current with linear interpolation and no extrapolation), `LoadSweepCsvWriter` and `LoadSweepCsvReader` are pure helpers covered by `LoadCurveAnalysisTests`.
- Simulink remains an offline modelling tool (`simulacion_agitador/`); there is no live Simulink/PIL path to the MCU anymore.

## Fast File Map

- Firmware state machine: `firmware/Core/Src/state_machine.c`, `state_machine.h`.
- Motor control, stall/PI logic and setpoint slew limiter: `motor_control.c`; load-curve sweep: `load_sweep.c`; speed measurement: `speed_sensor.c`; commutation: `bldc_driver.c`; HAL callbacks: `isr_callbacks.c`.
- Firmware config and flash persistence: `hard_config.c/.h`, `flash_config.c/.h`.
- GUI startup/DI/endpoints: `gui/EscGui/src/Esc.Web/Program.cs`, `gui/EscGui/src/Esc.Bridge/EscBridgeServiceCollectionExtensions.cs`, `BridgeEndpointMapper.cs`.
- Main UI page: `gui/EscGui/src/Esc.Web/Components/Pages/Home.razor`; load curves page: `Components/Pages/LoadCurves.razor`.
- Load sweep models and helpers: `gui/EscGui/src/Esc.Protocol/LoadSweepModels.cs`, `gui/EscGui/src/Esc.Bridge/LoadCurveAnalysis.cs`, `LoadSweepCsvWriter.cs`.
