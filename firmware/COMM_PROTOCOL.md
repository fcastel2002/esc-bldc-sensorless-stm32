# ESC BLDC Binary Communication Protocol

## Transport selection

The communication mode is selected once at boot with `PB8` (`COMM_MODE`) using the internal pull-up:

| PB8 level | Mode |
| --- | --- |
| HIGH or open | USB FS Custom HID on `PA11/PA12` |
| LOW | UART binary frames on USART1 (`PB6/PB7`, 115200 8N1) |

USB uses a vendor-defined Custom HID interface with 64-byte IN and OUT reports. The current USB VID/PID/manufacturer/product strings are placeholders in `Core/Src/usbd_desc.c` and must be replaced before product use.

## Frame layout

All requests, responses, and events are fixed 64-byte frames. Multi-byte values are little-endian.

| Byte | Field | Value |
| --- | --- | --- |
| 0 | magic[0] | `0xEC` |
| 1 | magic[1] | `0xB1` |
| 2 | version | `0x01` |
| 3 | type | `0x01` request, `0x81` response, `0x82` event |
| 4 | seq | Host sequence, echoed by responses |
| 5 | opcode | Command/event ID |
| 6 | param | Command parameter |
| 7 | status/flags | Response status or event flags |
| 8..9 | payload_len | `uint16`, max `52` |
| 10..61 | payload | Up to 52 bytes |
| 62..63 | crc16 | CRC16-CCITT-FALSE over bytes `0..61`, init `0xFFFF`, poly `0x1021` |

Responses echo `seq`, `opcode`, and `param`, set `type=0x81`, and place the result in `status`.

## Status codes

| Code | Name |
| --- | --- |
| `0x00` | `OK` |
| `0x01` | `BAD_MAGIC` |
| `0x02` | `BAD_VERSION` |
| `0x03` | `BAD_CRC` |
| `0x04` | `BAD_LENGTH` |
| `0x05` | `UNKNOWN_OPCODE` |
| `0x06` | `UNKNOWN_PARAM` |
| `0x07` | `INVALID_STATE` |
| `0x08` | `UNDERLIMIT` |
| `0x09` | `OVERLIMIT` |
| `0x0A` | `NOT_IMPLEMENTED` |
| `0x0B` | `FLASH_ERROR` |

## Opcodes

| Opcode | Name | Param | Request payload | Response payload |
| --- | --- | --- | --- | --- |
| `0x01` | `PING` | ignored | any `0..52` bytes | echoed payload |
| `0x02` | `GET_STATUS` | ignored | empty | status payload below |
| `0x10` | `RUN` | ignored | empty | empty |
| `0x11` | `STOP` | ignored | empty | empty |
| `0x12` | `ESTOP` | ignored | empty | empty; immediate and accepted in every operational state |
| `0x13` | `SET_SPEED_RPM` | ignored | `uint16 rpm` | empty |
| `0x20` | `GET_CONFIG` | config param | empty | config value |
| `0x21` | `SET_CONFIG` | config param | config value | empty |
| `0x22` | `RESET_CONFIG` | config param or `0xFF` | empty | empty |
| `0x23` | `SAVE_CONFIG` | config param or `0xFF` | empty | empty |
| `0x30` | `LOG_START` | log param or `0xFF` | empty | empty |
| `0x31` | `LOG_STOP` | log param or `0xFF` | empty | empty |
| `0x32` | `LOG_RATE` | ignored | `uint16 ms` | empty |
| `0x33` | `TELEMETRY_EVENT` | log param | event only | event payload below |
| `0x50` | `SINE_DRIVE` | `0=APPLY`, `1=KEEPALIVE` | `uint32 frequency_mHz`, `uint16 amplitude_permille` | accepted/quantized values in the same 6-byte shape |
| `0x51` | `STEP_TEST` | `0=APPLY`, `1=KEEPALIVE` | `uint8 step`, `uint16 duty_permille` | same 3-byte shape |
| `0x52` | `ZC_DIAG` | `0=START`, `1=STOP`, `2=GET_REPORT`, `3=AUTO_DETECT` | empty (or optional `uint16 duty_permille` for `AUTO_DETECT`) | 32-byte freewheel report, empty (`STOP`), or 16-byte auto-detect matrix |
| `0x60` | `LOAD_SWEEP` | `0=START`, `1=ABORT`, `2=GET_STATUS`, `3=GET_POINTS` | see [Load sweep](#load-sweep-load_sweep-0x60) | see [Load sweep](#load-sweep-load_sweep-0x60) |

Retired opcodes: `0x14 SET_CONTROL_MODE`, `0x24 GET_VALIDATION_REFERENCE` and
`0x40..0x44` (HIL/PIL) were removed by spec 002 and now return
`UNKNOWN_OPCODE`.

While a load sweep is active (`GET_STATUS` byte 30 = 1) only `PING`,
`GET_STATUS`, `STOP`, `ESTOP`, `GET_CONFIG`, `LOG_*` and `LOAD_SWEEP` are
accepted; everything else returns `INVALID_STATE`. `STOP` and `ESTOP` abort the
sweep.

`RUN` is valid only while `app_state == IDLE`. In every other state it returns
`INVALID_STATE` without changing the application state, PWM outputs, or timer
configuration.

### `GET_STATUS` response payload

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | `app_state` enum value |
| 1 | `uint8` | transport: `0=UART`, `1=USB` |
| 2 | `uint8` | `motor_stalled` |
| 3 | `uint8` | `consistent_zero_crossing` |
| 4..5 | `uint16` | speed setpoint RPM |
| 6..7 | `uint16` | measured speed RPM |
| 8..9 | `uint16` | `max_pwm` |
| 10 | `uint8` | INA219 flags: bit 0 present, bit 1 last read valid, bit 2 shunt overflow, bit 3 disabled (UART transport) |
| 11 | `uint8` | INA219 7-bit I2C address (`0` when absent) |
| 12..13 | `uint16` | DC-bus voltage at INA219 `VIN-`, mV |
| 14..17 | `int32` | DC-bus current through the 0.1 ohm shunt, microamps |
| 18 | `uint8` | INA219 last I2C error: `0` none, `1` not found (no ACK at `0x40..0x4F`), `2` NACK, `3` timeout, `4` bus error/arbitration lost, `5` bus busy, `6` other |
| 19 | `uint8` | INA219 failed-transaction counter (saturates at 255) |
| 20 | `uint8` | idle line levels sampled as floating inputs before each probe: bit 0 SCL high, bit 1 SDA high (`0x03` = pull-ups present) |
| 21 | `uint8` | stage of the last failure: `0` none, `1` probe (address), `2` config write, `3` config read-back verify, `4` shunt read, `5` bus read, `6` BUSY latched while idle (peripheral re-initialised) |
| 22..23 | `uint16` | INA219 config register read back after the probe write (expected `0x3E67`) |
| 24..25 | `uint16` | I2C1 peripheral resets done because `BUSY` was latched with idle lines (saturating) |
| 26..27 | `uint16` | slew-limited speed reference applied to the PI, RPM (equals the setpoint when `SPEED_RAMP` is 0) |
| 28..29 | `uint16` | commanded PWM duty, 1/10000 of the active `TIM1` period |
| 30 | `uint8` | `1` while a `LOAD_SWEEP` is active |

The current payload is 31 bytes; 18-, 20-, 24- and 26-byte variants (without the
trailing fields) are also accepted by the GUI. The probe only reports the sensor as
present after the config register reads back the written value, so a floating
bus that fakes an address ACK is not mistaken for an INA219. Legacy firmware returns 10 bytes; clients must
treat a 10-byte status as "no power monitor".

INA219 notes: the sensor sits on `PB6` (SCL) / `PB7` (SDA), driven by a software
(bit-banged) I2C master on open-drain GPIOs at ~100 kHz. The pins
is the same pin pair used by remapped `USART1` for the UART transport. It is
therefore enabled only when the board boots in USB mode (`PB8` HIGH); in UART
mode bit 3 is set and all readings are zero. Firmware configures it for 32 V bus
range, PGA /8 (+/-320 mV shunt, +/-3.2 A with 0.1 ohm), 16-sample averaging on
both channels, and polls it every 20 ms from the main loop. Addresses
`0x40..0x4F` are probed; a missing sensor (or 5 consecutive read failures)
triggers a bus recovery, I2C re-init and re-probe every 1 s. The hardware
I2C1 peripheral is not used: it needs `PB6/PB7` in alternate-function mode,
where remapped `USART1` interferes (reads fail), and clearing `USART1_REMAP`
moves `USART1` onto the TIM1 phase PWM pins `PA8/PA9/PA10`, which breaks the
power-stage drive and the ZC auto-detect. `USART1_REMAP` must stay set. In GPIO
mode the pins are driven only by `ODR`, so nothing else can interfere. Byte 24..25
(BUSY resets) is kept for compatibility and is always `0`; stage `6` is no longer
reported. `PB6` is also
`TIM4_CH1`: the control-period timer is started with only the CC1 interrupt
(`motor_control_timer_start()`), never `HAL_TIM_OC_Start_IT`.

`app_state=10` identifies the continuous open-loop `SINE_DRIVE` state,
`app_state=11` the commutation-step debug `STEP_TEST` state, and `app_state=12`
the passive zero-crossing diagnostic `FREEWHEEL` state. Existing state values
`0..9` retain their previous numeric assignments.

## Config parameters

| Param | Name | Payload type | Implemented |
| --- | --- | --- | --- |
| `0x01` | `PWM_FREQ` | `uint16 Hz` | yes |
| `0x02` | `POLE_PAIRS` | `uint8` | yes |
| `0x06` | `MAX_SPEED` | `uint16 RPM` | yes |
| `0x07` | `MIN_SPEED` | `uint16 RPM` | yes |
| `0x0A` | `KP_RPM` | `int16` hundredths, canonical counts/RPM | yes |
| `0x0B` | `KI_RPM` | `int16` hundredths, canonical counts/(RPM s) | yes |
| `0x0C` | `KD_RPM` | `int16` hundredths | only zero; nonzero is not supported in algorithm 2 |
| `0x0D` | `STARTUP_INITIAL_AMPLITUDE` | `uint16` permille | yes, `0..1000` |
| `0x0E` | `STARTUP_FINAL_AMPLITUDE` | `uint16` permille | yes, `0..1000` |
| `0x0F` | `STARTUP_INITIAL_FREQUENCY` | `uint32` electrical mHz | yes, `2000..10000` |
| `0x10` | `STARTUP_FINAL_FREQUENCY` | `uint32` electrical mHz | yes, `2000..10000` |
| `0x11` | `STARTUP_DURATION` | `uint32 ms` | yes, `500..10000` |
| `0x12` | `BEMF_BLANKING_US` | `uint16 us` | yes, `0..200`; `0` disables blanking |
| `0x13` | `ZC_MAP` | `uint8[3]` (`ch_u, ch_v, ch_w`) | yes, permutation of `{0=PA0, 1=PA1, 2=PB10}`; default `[2, 1, 0]` |
| `0x14` | `SPEED_RAMP` | `uint16 RPM/s` | yes, `0..20000`; `0` disables the setpoint slew limit (default) |
| `0xFF` | `ALL` | reset/log only | yes for `RESET_CONFIG`, `LOG_START`, `LOG_STOP` |

Params `0x03..0x05` (legacy `KP/KI/KD`), `0x08 CURRENT_LIMIT` and
`0x09 TEMP_LIMIT` were removed by spec 002 and return `UNKNOWN_PARAM`; the
persisted `ESCparams` still carries the `current_limit`/`temp_limit` fields so the
flash layout is unchanged.

For gains, value `100` means `1.00`. Algorithm 2 computes RPM error and a
canonical PWM output referenced to ARR=2000 using Q16.16 state and standard
trapezoidal integration. The canonical output is scaled once to active ARR.
`SET_CONFIG` and `RESET_CONFIG` update active RAM and mark pending changes but
do not write flash. A v1 flash configuration preserves non-gain fields and
resets only gains to v2 defaults (`KP=0.28`, `KI=1.00`, `KD=0`). V1, V2, and
V3 flash blocks are validated with their original size and CRC and migrated to
V4. V1/V2 receive startup defaults of `20% -> 100%`, `2.09 Hz -> 9.28 Hz`, and
`3 s`; every legacy version receives the compatible blanking default of `0 us`.
The current block is V6 (`ESCparams` 68 bytes): it appends `speed_ramp_rpm_s`
(and a reserved `uint16`) to the V5 layout. V5 blocks are migrated verbatim with
`SPEED_RAMP = 0`, so existing behavior is preserved until a ramp is configured.

When `app_state == CLOSEDLOOP`, changes to `PWM_FREQ`, `KP_RPM`, `KI_RPM`,
zero `KD_RPM` and `SPEED_RAMP` apply immediately. KD remains blocked until a fresh-sample RPM
derivative and filter are implemented. Other parameters use the deferred
`CONFIG` path.

The five startup parameters are runtime-only until `SAVE_CONFIG` is sent. They
may be set, reset, or saved only in `IDLE` and are consumed by the next `RUN`.
The startup generator interpolates amplitude and electrical frequency linearly
over the configured duration before handing off to six-step operation.

`BEMF_BLANKING_US` follows the same explicit-save and `IDLE`-only update rule.
At the next `RUN`, firmware converts microseconds to TIM2 ticks by rounding up;
TIM2 runs at 180 kHz, so the effective resolution is about `5.56 us`. After the
SPWM handoff and every accepted six-step commutation, input captures inside the
configured interval are ignored. Rejected captures do not commutate, refresh
stall timing, or update speed-consensus buffers. PWM operation is unchanged.
`RESET_CONFIG ALL` and `SAVE_CONFIG ALL` are also restricted to `IDLE` because
the full configuration contains parameters that cannot change while running.
All forms of `SAVE_CONFIG` require `IDLE` because a named save still writes the
entire active configuration block.

`SPEED_RAMP` limits how fast the PI reference follows `SET_SPEED_RPM`: every
2 ms control tick the reference moves toward the target by at most
`SPEED_RAMP * 0.002` RPM (Q16.16). On entering `CLOSEDLOOP` with a non-zero ramp
the reference starts at the measured speed (bumpless) instead of jumping to the
target. Purpose: keep the stir-bar coupling below its pull-out torque while
accelerating.

## Continuous three-phase sine drive

`SINE_DRIVE` is an additional diagnostic operating mode; it does not replace
the normal `RUN` startup and never transitions to six-step. Frequency and
amplitude are applied atomically:

| Offset | Type | Meaning |
| --- | --- | --- |
| 0..3 | `uint32` | electrical frequency in mHz, `2000..10000` |
| 4..5 | `uint16` | modulation amplitude in permille, `0..1000` |

`APPLY` is accepted from `IDLE` to enter `SINE_DRIVE`, or from `SINE_DRIVE` to
update both values. `KEEPALIVE` is accepted only while already in `SINE_DRIVE`,
so a late host packet cannot restart the power stage after watchdog expiry. Both
forms renew the lease and echo the actual quantized electrical frequency and
accepted amplitude. While active, only `PING`, `GET_STATUS`, `SINE_DRIVE`,
`STEP_TEST`, `LOG_START`, `LOG_STOP`, `LOG_RATE`, `STOP`, and `ESTOP` are
accepted.

The host must renew `SINE_DRIVE` before the 1500 ms firmware watchdog expires.
The local bridge renews it on a dedicated 350 ms loop. `STOP`, `ESTOP`, watchdog expiry, or a
lost connection stop TIM4, clear all PWM compares, disable all three power-stage
enables, and return to `IDLE`. Conventional BEMF commutation is disabled because
all three phases are actively modulated.

## Commutation step debug drive

`STEP_TEST` (`0x51`) holds one fixed six-step commutation step for hardware
debugging. It reuses the same EN/compare sequence as normal commutation
(`bldc_apply_step` -> `commutate`): one phase PWM, one phase low, one floating.

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | step: `0=UV`, `1=UW`, `2=VW`, `3=VU`, `4=WU`, `5=WV` (first letter PWM, second LOW) |
| 1..2 | `uint16` | duty in permille, `0..300` |

The response echoes the same 3 bytes. Param `0=APPLY` is accepted from `IDLE`
(enters `app_state=11 STEP_TEST`) or from `STEP_TEST`; param `1=KEEPALIVE`
only while already in `STEP_TEST`. A step above `5` returns `UNKNOWN_PARAM`
and a duty above `300` permille returns `OVERLIMIT`. The 1500 ms watchdog,
command allow-list and `STOP`/`ESTOP` behavior match `SINE_DRIVE`; the bridge
renews the lease on its 350 ms loop. Zero-crossing captures never commutate
outside `RUNNING`/`CLOSEDLOOP`, so the step stays fixed. Current telemetry
is flagged valid for the shunt phase (U or V) that is low in the held step.

## Freewheel zero-crossing diagnostic & software remapping

`ZC_DIAG` (`0x52`) provides passive freewheel diagnostics and active low-duty
ZC-to-power phase auto-detection:

- `param=0 (START)`: disables the power stage (`EN_U=EN_V=EN_W=0`, `PWM_STOP`),
  enters `app_state=12 FREEWHEEL`, arms dual-edge capture on `TIM2` CH1/CH2/CH3
  (`PA0`/`PA1`/`PB10`), resets counters, and returns the 32-byte diagnostic report.
- `param=1 (STOP)`: exits `FREEWHEEL` to `IDLE` and restores `TIM2` polarity.
- `param=2 (GET_REPORT)`: returns the 32-byte diagnostic report (live pins, seen
  3-bit states, health flags, per-channel edge counts, forward/reverse/invalid
  step counts, glitches, estimated RPM, 60° step ticks & asymmetry %, active ZC
  map, and last 6 captured edges).
- `param=3 (AUTO_DETECT)`: accepted only in `IDLE`. Optional `uint16 duty_permille`
  (`50..250`, default `150` = `15%`). Probes U, V, W sequentially (~15 ms each)
  with all enables HIGH, samples the 3 ZC comparators synchronized inside the
  `PWM ON` window, disables the power stage immediately, and returns 16 bytes:
  `result_code` (`0=OK`, `1=OK_INVERTED`, `2=NO_SIGNAL`, `3=AMBIGUOUS`),
  `detected_map[3]`, `active_map[3]`, and row-major `high_pct[9]`.

`ZC_MAP` (`0x13`) stores the software permutation `[ch_u, ch_v, ch_w]` in RAM
(allowed in `IDLE` or `FREEWHEEL`) and persists in Flash `ESCparams` V5 on
explicit `SAVE_CONFIG` (in `IDLE`). Legacy V1..V4 flash blocks migrate cleanly
to V5 with default `[2, 1, 0]` (`U=PB10`, `V=PA1`, `W=PA0`).

## Logging and telemetry

Log params:

| Param | Name |
| --- | --- |
| `0x01` | `SPEED` (value RPM; raw = filtered `TIM2` period; flags = BEMF quality) |
| `0x06` | `BUS_VOLTAGE` (INA219, value en mV; raw = mV; flags = INA219 flags) |
| `0x07` | `BUS_CURRENT` (INA219, value en mA; raw = registro shunt `int16`, 10 uV/LSB; flags = INA219 flags) |
| `0x08` | `DUTY` (value = duty en 1/10000; raw = compare de `TIM1`; flags bit 0 = valido) |
| `0xFF` | `ALL` |

Params `0x02 TEMP`, `0x03 CURRENT_U`, `0x04 CURRENT_V`, `0x05 BEMF_PERIOD` and
`0x09 LOAD_CURRENT` were removed by spec 002 (no phase shunts, no temperature
sensor, no model-based estimates in firmware) and return `UNKNOWN_PARAM`.

`LOG_RATE` accepts `100..5000 ms`. Telemetry is sent as `type=0x82`, `opcode=0x33`
and only while `app_state != IDLE`.

Telemetry event payload:

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | log param |
| 1..4 | `int32` | value (unit per param above) |
| 5..8 | `uint32` | `HAL_GetTick()` timestamp |
| 9..10 | `uint16` | raw value (per param above) |
| 11 | `uint8` | flags: bit 0 valid (INA219 params carry the INA219 flag byte) |
| 12..13 | `uint16` | reserved (`0`) |
| 14 | `int8` | active commutation step |

The firmware only reports measured quantities. Load comparisons are done on the
host against a tare curve at the same speed: the bridge's own consumption cancels
in `I_bus - I_bus,tare`, and dividing that difference by the duty gives the extra
winding current (low-side recirculation keeps the PWM off-time current out of
the INA219 shunt, so `mean(I_bus) ~= d * mean(I_phase)`).

## Load sweep (`LOAD_SWEEP`, 0x60)

Firmware-side instrumentation to identify the load curve of the stirred fluid.
The sweep drives the speed target in closed loop and records averaged operating
points in a 128-entry RAM table that the host reads back. Points are not
persisted and are cleared by the next `START`.

### `START` (param `0`), 12-byte payload

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | mode: `0` staircase, `1` continuous ramp |
| 1..2 | `uint16` | start RPM |
| 3..4 | `uint16` | end RPM (may be below start) |
| 5..6 | `uint16` | staircase: step in RPM (`>= 10`); ramp: slew rate in RPM/s (`1..5000`) |
| 7..8 | `uint16` | settle time in ms (`0..30000`) |
| 9..10 | `uint16` | averaging window per point in ms (`100..10000`) |
| 11 | `uint8` | options: bit 0 stop the motor when the sweep completes normally |

Start and end must lie inside `MIN_SPEED..MAX_SPEED` (`UNDERLIMIT`/`OVERLIMIT`).
The plan must fit the table: staircase `ceil(span/step) + 1 <= 128`, ramp
`(span*1000/rate)/window + 1 <= 128`, otherwise `OVERLIMIT`. Accepted only in
`IDLE` or `CLOSEDLOOP` with no sweep active, otherwise `INVALID_STATE`.

Sequence:

1. From `IDLE`: average the bus current for 500 ms with the power stage off
   (reported as idle bus current), set the target to the start RPM and run the normal SPWM startup. If
   `CLOSEDLOOP` is not reached within 15 s the sweep aborts (`STARTUP_TIMEOUT`)
   and the motor is stopped. From `CLOSEDLOOP`: report the idle current tracked
   during the last `IDLE`.
2. Staircase: for each step set the target, wait until the slew-limited reference
   reaches it (`SPEED_RAMP` applies), wait `settle_ms`, then average one window
   into a point.
3. Ramp: move to the start RPM, wait `settle_ms`, then set the target to the end
   RPM with the sweep rate temporarily overriding `SPEED_RAMP` and record
   consecutive windows until the reference reaches the end.
4. On completion the configured `SPEED_RAMP` is restored. A ramp that stops early
   (abort, table full) freezes the target at the current reference so the motor
   never jumps to the end RPM.

The sweep aborts if the motor leaves `CLOSEDLOOP` or the state machine performs
an automatic restart (`MOTOR_LOST`, motor stopped), on `ABORT` (`USER`, motor
keeps the current reference), on `STOP`/`ESTOP` (`STOPPED`) and when the table
fills (`TABLE_FULL`).

Inside a window speed, duty and reference are sampled every 10 ms; INA219
readings are taken as they arrive (one every ~20 ms).

### `ABORT` (param `1`)

Empty payload. No-op when no sweep is active.

### `GET_STATUS` (param `2`), 29-byte response

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | state: `0` idle, `1` measuring `I_q`, `2` starting, `3` moving, `4` settling, `5` measuring, `6` done, `7` aborted |
| 1 | `uint8` | abort reason: `0` none, `1` user, `2` motor lost, `3` startup timeout, `4` table full, `5` STOP/ESTOP |
| 2 | `uint8` | mode of the last sweep |
| 3 | `uint8` | options of the last sweep |
| 4..5 | `uint16` | points recorded |
| 6..7 | `uint16` | table capacity (`128`) |
| 8..17 | `uint16[5]` | start, end, step-or-rate, settle ms, window ms of the last sweep |
| 18..19 | `uint16` | current speed target, RPM |
| 20..23 | `int32` | bus current measured in `IDLE` with the power stage off, uA (informative) |
| 24..27 | `uint32` | elapsed ms since `START` (frozen when finished) |
| 28 | `uint8` | `1` if the idle bus current has been measured |

### `GET_POINTS` (param `3`)

Request payload `uint16 first_index`. Response: `uint16 first_index`,
`uint8 count` (`0..3`), then `count` points of 16 bytes:

| Offset | Type | Meaning |
| --- | --- | --- |
| 0..1 | `uint16` | mean slew-limited reference, RPM |
| 2..3 | `uint16` | mean measured speed, RPM |
| 4..5 | `uint16` | mean duty, 1/10000 |
| 6..7 | `uint16` | mean bus voltage, mV |
| 8..9 | `int16` | mean bus current, 0.1 mA |
| 10..11 | `uint16` | standard deviation of the duty, 1/10000 |
| 12..13 | `uint16` | standard deviation of the bus current, 0.1 mA |
| 14 | `uint8` | standard deviation of the speed, RPM (saturates at 255) |
| 15 | `uint8` | flags: bit 0 no INA219 sample, bit 1 fewer than half the expected INA219 samples, bit 2 duty reached the PI ceiling |

## ASCII command equivalence

| Previous ASCII command | Binary equivalent |
| --- | --- |
| `:RUN` | opcode `0x10 RUN`, empty payload |
| `:STOP` | opcode `0x11 STOP`, empty payload |
| `:ESTOP` | opcode `0x12 ESTOP`, empty payload |
| `:SPEED:<rpm>` | opcode `0x13 SET_SPEED_RPM`, payload `uint16 rpm` |
| `:GET:PWM_FREQ` | opcode `0x20 GET_CONFIG`, param `0x01` |
| `:SET:PWM_FREQ:<hz>` | opcode `0x21 SET_CONFIG`, param `0x01`, payload `uint16 Hz` |
| `:RESET:PWM_FREQ` | opcode `0x22 RESET_CONFIG`, param `0x01` |
| `:GET:POLEP` | opcode `0x20 GET_CONFIG`, param `0x02` |
| `:SET:POLEP:<n>` | opcode `0x21 SET_CONFIG`, param `0x02`, payload `uint8` |
| `:RESET:POLEP` | opcode `0x22 RESET_CONFIG`, param `0x02` |
| `:GET:KP` | opcode `0x20 GET_CONFIG`, param `0x0A KP_RPM` |
| `:SET:KP:<value>` | opcode `0x21 SET_CONFIG`, param `0x0A`, payload `int16 value*100` |
| `:RESET:KP` | opcode `0x22 RESET_CONFIG`, param `0x0A` |
| `:GET:KI` | opcode `0x20 GET_CONFIG`, param `0x0B KI_RPM` |
| `:SET:KI:<value>` | opcode `0x21 SET_CONFIG`, param `0x0B`, payload `int16 value*100` |
| `:RESET:KI` | opcode `0x22 RESET_CONFIG`, param `0x0B` |
| `:GET:KD` | opcode `0x20 GET_CONFIG`, param `0x0C KD_RPM` (zero only) |
| `:SET:KD:<value>` | opcode `0x21 SET_CONFIG`, param `0x0C`; nonzero is rejected |
| `:RESET:KD` | opcode `0x22 RESET_CONFIG`, param `0x0C` |
| `:GET:MAXSPEED` | opcode `0x20 GET_CONFIG`, param `0x06` |
| `:SET:MAXSPEED:<rpm>` | opcode `0x21 SET_CONFIG`, param `0x06`, payload `uint16 RPM` |
| `:RESET:MAXSPEED` | opcode `0x22 RESET_CONFIG`, param `0x06` |
| `:GET:MINSPEED` | opcode `0x20 GET_CONFIG`, param `0x07` |
| `:SET:MINSPEED:<rpm>` | opcode `0x21 SET_CONFIG`, param `0x07`, payload `uint16 RPM` |
| `:RESET:MINSPEED` | opcode `0x22 RESET_CONFIG`, param `0x07` |
| `:LOG:START:SPEED` | opcode `0x30 LOG_START`, param `0x01` |
| `:LOG:START:ALL` | opcode `0x30 LOG_START`, param `0xFF` |
| `:LOG:STOP:SPEED` | opcode `0x31 LOG_STOP`, param `0x01` |
| `:LOG:STOP:ALL` | opcode `0x31 LOG_STOP`, param `0xFF` |
| `:LOG:RATE:<ms>` | opcode `0x32 LOG_RATE`, payload `uint16 ms` |
| `:HELP` | removed from main channel; use this document |
