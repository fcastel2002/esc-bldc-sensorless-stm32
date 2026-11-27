# 002 Load Curve Instrumentation Tasks

## Tasks

- [x] T001 Create `feat/002-load-curve-instrumentation` from `feat/ina219-power-telemetry` (INA219 is required and not yet on `main`).
- [x] T002 Remove HIL/PIL, control modes and validation reference from firmware.
- [x] T003 Remove ADC phase-current sensing, `TEMP`/`BEMF_PERIOD`/`CURRENT_U/V` logs and retired config params.
- [x] T004 Add the setpoint slew limiter and `SPEED_RAMP` (`ESCparams` V6 + V5 migration).
- [x] T005 Add `load_sweep.c`, opcode `0x60`, `DUTY` log and `GET_STATUS` bytes `26..30`.
- [x] T006 Remove the UDP/HIL bridge, validation runs, PIL page, Simulink mode and current chart from the GUI.
- [x] T007 Add the load sweep bridge API, `/curvas` page, tare comparison and CSV export.
- [x] T008 Update protocol, bridge and UI tests.
- [x] T009 Update `COMM_PROTOCOL.md`, agent docs and project docs.
- [x] T010 Build firmware (Debug/Release), run GUI tests, publish the Windows GUI and flash the board.
- [x] T011 First bench curves: air tare (700–3000 rpm) and water staircase with decoupling between 2000 and 2250 rpm.
- [x] T012 Remove the biased firmware `I_hat` (`LOAD_CURRENT`, point field) after the air curve; store duty dispersion instead.
- [x] T013 GUI: overlay several curves, import CSV, select a tare, load current vs tare and speed-dispersion metrics.
- [ ] T014 Bench validation of the new firmware/GUI (user).
