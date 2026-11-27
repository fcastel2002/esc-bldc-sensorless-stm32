# 002 Load Curve Instrumentation Design

## Physical basis

In steady state with the stir bar coupled, the motor torque balances friction
plus the fluid load transmitted by the magnetic coupling:

`T_e = K_t * I_phase = T_f1(w) + T_L,tot(w)`

The power stage applies PWM on the high-side input with the other phase held
low, so during the off-time the current recirculates through the low side and
never crosses the INA219 shunt: `mean(I_bus) ~= d * mean(I_phase) + I_0`, where
`I_0` is the bridge's own consumption while running. Comparing against a tare
curve (stir bar in air) at the same speed cancels `I_0`:
`(I_bus - I_bus,tare) / d ~= delta I_phase ~= T_L / K_t`. The GUI computes this
"load current vs tare"; the firmware stores measured quantities only.

The first implementation estimated `I_hat = (I_bus - I_q) / d` in firmware with
`I_q` measured in `IDLE` with the power stage off. The first air curve showed it
is biased: fitting `I_bus = I_0 + d * I_phase` gave `I_0 ~= 50 mA` running versus
`I_q ~= 20 mA` idle, which made `I_hat` fall artificially from 238 to 167 mA with
speed. It was removed (decision: no model-based estimates in firmware).

Duty alone is a poor torque meter: it is dominated by the back-EMF term and
drifts with bus voltage, magnet temperature and the L298 drop. Both raw duty and
INA219 means are still recorded so the host can try other estimators.

Known limits: the duty differs slightly between fluid and tare at the same speed,
so `I_0` cancels only approximately; the L298 is slow at 34 kHz PWM, so the
effective duty differs a little from the commanded one; vortex formation and fill
volume change the curve. Absolute viscosity is not identifiable with air and water references only
(water is in the inertial regime); a viscous calibration fluid would be needed.

## Firmware

- `motor_control.c`: `speed_setpoint_rpm` is the commanded target; the PI
  follows a Q16.16 reference advanced in the 2 ms ISR by at most
  `rate * 65536 * 0.002` per tick. `motor_control_set_ramp_override()` lets the
  sweep impose its own rate temporarily. Bumpless entry: with a ramp the
  reference starts at the measured speed.
- `load_sweep.c` (main loop): states `IDLE -> MEASURING_IQ -> STARTING -> MOVING
  -> SETTLING -> MEASURING -> DONE/ABORTED`. Speed, duty and reference are
  sampled every 10 ms; INA219 samples are consumed as they arrive (20 ms poll).
  Per window it keeps sums and squared deviations (relative to the first sample)
  to output means and standard deviations with an integer square root. Points
  are 16 bytes, 128 entries (2 KB RAM).
- Safety: any change of `motor_restart_count` (stall or timeout restart) or
  leaving `CLOSEDLOOP` aborts with `MOTOR_LOST` and stops the motor. A ramp that
  ends early freezes the target at the current reference so clearing the
  override never jumps the speed to the end RPM.
- The protocol guard blocks every opcode except monitoring, stop and the sweep
  itself while a sweep owns the setpoint.

## GUI

- The bridge copies new points incrementally (`GET_POINTS`, 3 per frame) when
  the cached status or sweep state says a sweep is active or points are pending,
  from the existing 350 ms maintenance loop.
- Tare curves are host-only (`PinLoadSweepReference`), so comparing water against
  air never needs firmware storage. `LoadCurveAnalysis` interpolates the tare
  linearly and never extrapolates.

## Flash budget (Release, `arm-none-eabi-gcc -Os`)

| Build | Before | After |
| --- | --- | --- |
| Release | 50 556 B (79.6 %) | 48 592 B (76.5 %) |
| Debug | 57 KB (91.9 %) | 55 532 B (87.5 %) |

The largest remaining single consumer that could still be removed is the libm
`sinf` used once at startup to fill the SPWM tables (~4.6 KB); it was left
untouched because the startup path is out of scope.
