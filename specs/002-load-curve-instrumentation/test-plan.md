# 002 Load Curve Instrumentation Test Plan

## Local Checks

- [x] From `firmware/`: `cmake --preset Debug -DESC_BUILD_CODE_DOCS=OFF` and `cmake --build --preset Debug`
- [x] From `firmware/`: `cmake --preset Release -DESC_BUILD_CODE_DOCS=OFF` and `cmake --build --preset Release`
- [x] Changed firmware sources compiled with `-Wall -Wextra` (the project build uses `-w`): no new warnings.
- [x] From repo root: `dotnet build gui\EscGui\src\Esc.Web\Esc.Web.csproj`
- [x] From repo root: `dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj` (139 passed)
- [x] GUI rendered in a browser without a device: Dashboard, Debug and `/curvas` load without console errors.
- [x] Windows GUI publish: `powershell -ExecutionPolicy Bypass -File .\gui\EscGui\publish-gui-win-x64.ps1`
- [x] First Release firmware flashed through ST-Link with OpenOCD (`program ... verify reset`).
- [x] Flash the raw-only firmware (without `LOAD_CURRENT`) through ST-Link: `Verified OK`.

## CI Checks

- [ ] `repo-hygiene` pending GitHub Actions.
- [ ] `firmware-debug` pending GitHub Actions.
- [ ] `gui-build-test` pending GitHub Actions.
- [ ] `protocol-guard` pending GitHub Actions.
- [ ] `ssd-docs` pending GitHub Actions.

## Hardware Checks (USB transport, manual)

- [ ] Connect over USB HID; `GET_STATUS` returns 31 bytes and the rail shows KP/KI/KD, PWM and ramp.
- [ ] Flash migration: after the first boot the previous V5 configuration (gains, startup, ZC map) is intact and `SPEED_RAMP` reads `0`.
- [ ] `SPEED_RAMP = 0`: RUN and setpoint steps behave as before.
- [ ] `SPEED_RAMP = 500`: a setpoint step ramps at ~500 rpm/s (reference vs speed on the dashboard).
- [ ] Staircase sweep from `IDLE` in air (stir bar in empty beaker or removed): points recorded, motor stops at the end, CSV exported, pinned as tare.
- [x] Same staircase in water: bus current above the air tare by +14..+22 mA between 1240 and 1990 rpm.
- [x] Decoupling visible between 2000 and 2250 rpm: bus current drops ~31 mA (below the tare) and speed dispersion rises from ~8 to ~30 rpm.
- [ ] With the new GUI: import both CSVs, mark air as tare, check the load-current-vs-tare and speed-dispersion views.
- [ ] Slow ramp in water around the decoupling speed.
- [ ] `STOP`/`ESTOP` during a sweep abort it with reason `STOPPED`; user `ABORT` keeps the motor at the current reference.

## Regression Areas

- Closed-loop PI and handoff from SPWM startup (reference now goes through the slew limiter).
- Flash configuration load/migration and `SAVE_CONFIG`.
- `SINE_DRIVE`, `STEP_TEST`, `ZC_DIAG` and ZC map configuration.
- INA219 polling and telemetry on USB transport.
