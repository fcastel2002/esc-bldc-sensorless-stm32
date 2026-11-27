# 002 Load Curve Instrumentation

Status: Implemented

## Problem

With the magnetic stir bar coupled to the rotor, the speed PI needs noticeably
more duty to reach the same setpoints in water than in air: the fluid load is
visible in the control effort. The project wants to identify that load curve to
adapt speed setpoints and ramps and avoid stir-bar decoupling.

The firmware is not prepared for that work. Flash was at 80 % (Release) / 92 %
(Debug), mostly spent on communication features that are no longer used:
the HIL/PIL protocol, runtime control modes and validation descriptors, plus the
phase-current ADC path whose shunts were never populated. Load curves could only
be built by host scripts polling `GET_STATUS`, which does not even report the
commanded duty.

## Objective

1. Slim the firmware and GUI to what the stirrer needs: remove HIL/PIL,
   `SET_CONTROL_MODE`, `GET_VALIDATION_REFERENCE`, the ADC phase-current path and
   the placeholder/obsolete log channels. Keep transport selection, startup
   configuration, `ZC_DIAG`, `SINE_DRIVE` and `STEP_TEST`.
2. Add firmware-side instrumentation to measure load curves: a staircase/ramp
   sweep that records averaged measured operating points (rpm, duty, INA219 bus
   voltage/current and their dispersion), duty telemetry, and a persisted setpoint
   slew limiter. Model-based estimates stay on the host.
3. Give the GUI a page to run sweeps, overlay several curves (saved or imported
   from CSV), compare against a tare curve (stir bar in air) and export CSV.

## Scope

In scope:

- Firmware: delete `current_sense.c/.h`, `adc.c/.h`, the HAL ADC module, the HIL
  state machine branches and opcodes `0x14`, `0x24`, `0x40..0x44`; drop log params
  `0x02..0x05` and config params `0x03..0x05`, `0x08`, `0x09`.
- Firmware: new `load_sweep.c` and opcode `0x60 LOAD_SWEEP`; new log params
  `0x08 DUTY`; `GET_STATUS` bytes `26..30`; config param
  `0x14 SPEED_RAMP` persisted through `ESCparams` V6 with V5 migration.
- GUI: remove the UDP/HIL bridge, validation runs (MAT import, SQLite store), PIL
  monitor page, Simulink control mode and the current/BEMF chart; add the load
  sweep API to the bridge, the `/curvas` page, tare comparison and CSV export.
- Tests and docs for all of the above.

Out of scope:

- Automatic decoupling detection and automatic speed limits (next step, once
  real air/water curves exist).
- Viscosity estimation (needs a viscous calibration fluid; see the analysis in
  the design notes).
- Changes to startup, commutation, ZC detection or the PI algorithm itself.
- Regenerating CubeMX output; the `.ioc` files were already out of sync and still
  list ADC1.

## Affected Subprojects

- Firmware: Yes
- GUI/Bridge: Yes
- Protocol: Yes
- PIL/HIL/Simulink: Removed (Simulink stays as an offline modelling tool)
- Docs/CI: Yes

## Protocol Impact

Breaking for retired features only, frame format unchanged:

- Removed opcodes `0x14 SET_CONTROL_MODE`, `0x24 GET_VALIDATION_REFERENCE`,
  `0x40..0x44` HIL/PIL now return `UNKNOWN_OPCODE`.
- Removed config params `0x03..0x05`, `0x08`, `0x09` and log params `0x02..0x05`
  return `UNKNOWN_PARAM`.
- Added opcode `0x60 LOAD_SWEEP` (`START`, `ABORT`, `GET_STATUS`, `GET_POINTS`),
  config param `0x14 SPEED_RAMP`, log param `0x08 DUTY`. (`0x09 LOAD_CURRENT` existed
  briefly during development and was removed; see design notes.)
- `GET_STATUS` grows from 26 to 31 bytes (reference RPM, duty, sweep flag); older
  lengths stay decodable by the GUI.
- Flash layout: `ESCparams` V6 appends `speed_ramp_rpm_s`; V1..V5 blocks migrate.

## Acceptance Criteria

- Firmware Debug and Release build without errors; Release flash usage drops
  below the 50.5 KB baseline despite the new module.
- `SPEED_RAMP = 0` reproduces the previous closed-loop behavior.
- A sweep started from `IDLE` measures `I_q`, starts the motor, records one point
  per step/window and stops the motor at the end when requested.
- `STOP`, `ESTOP`, user `ABORT`, leaving `CLOSEDLOOP` or an automatic restart
  abort the sweep with the documented reason.
- GUI builds, all tests pass, the `/curvas` page runs, plots, pins a tare and
  exports CSV; no HIL/Simulink UI remains.
- `firmware/COMM_PROTOCOL.md` and the agent docs describe the new surface.

## References

- `firmware/COMM_PROTOCOL.md`
- `firmware/Core/Src/load_sweep.c`, `firmware/Core/Src/motor_control.c`
- `gui/EscGui/src/Esc.Web/Components/Pages/LoadCurves.razor`
- `docs/teoria_simulacion/fundamentos_fisicos_modelado.tex` (coupling and fluid load model)
