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
| `0x14` | `SET_CONTROL_MODE` | ignored | `uint8 mode` | empty |
| `0x20` | `GET_CONFIG` | config param | empty | config value |
| `0x21` | `SET_CONFIG` | config param | config value | empty |
| `0x22` | `RESET_CONFIG` | config param or `0xFF` | empty | empty |
| `0x23` | `SAVE_CONFIG` | config param or `0xFF` | empty | empty |
| `0x24` | `GET_VALIDATION_REFERENCE` | ignored | empty | structural validation reference below |
| `0x30` | `LOG_START` | log param or `0xFF` | empty | empty |
| `0x31` | `LOG_STOP` | log param or `0xFF` | empty | empty |
| `0x32` | `LOG_RATE` | ignored | `uint16 ms` | empty |
| `0x33` | `TELEMETRY_EVENT` | log param | event only | event payload below |
| `0x40` | `HIL_START` | ignored | empty, `uint16 input_timeout_ms`, or 3-byte timeout/mode payload below | empty |
| `0x41` | `HIL_STOP` | ignored | empty | empty |
| `0x42` | `HIL_SET_INPUTS` | ignored | HIL input payload below | empty |
| `0x43` | `HIL_GET_OUTPUTS` | ignored | empty | HIL output payload below |
| `0x44` | `HIL_STEP` | ignored | exact 18-byte deterministic-step payload below | exact 48-byte deterministic-step response below |
| `0x50` | `SINE_DRIVE` | `0=APPLY`, `1=KEEPALIVE` | `uint32 frequency_mHz`, `uint16 amplitude_permille` | accepted/quantized values in the same 6-byte shape |
| `0x51` | `STEP_TEST` | `0=APPLY`, `1=KEEPALIVE` | `uint8 step`, `uint16 duty_permille` | same 3-byte shape |
| `0x52` | `ZC_DIAG` | `0=START`, `1=STOP`, `2=GET_REPORT`, `3=AUTO_DETECT` | empty (or optional `uint16 duty_permille` for `AUTO_DETECT`) | 32-byte freewheel report, empty (`STOP`), or 16-byte auto-detect matrix |

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

`app_state=10` identifies the continuous open-loop `SINE_DRIVE` state,
`app_state=11` the commutation-step debug `STEP_TEST` state, and `app_state=12`
the passive zero-crossing diagnostic `FREEWHEEL` state. Existing state values
`0..9` retain their previous numeric assignments.

### `GET_VALIDATION_REFERENCE` response payload

This read-only descriptor exposes the active PWM scale and compiled controller
constants needed by an external validation model. It does not include gains or
pole pairs because the validation experiment commands those values explicitly.

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | reference schema version (`1`) |
| 1 | `uint8` | controller algorithm version (`2`, RPM PI Q16.16) |
| 2..3 | `uint16` | active PWM frequency in Hz |
| 4..5 | `uint16` | active `TIM1->ARR` in PWM counts |
| 6..9 | `uint32` | TIM2 speed timer frequency in Hz |
| 10..11 | `uint16` | maximum measured period (`SPEED_MIN`) |
| 12..13 | `uint16` | minimum measured period (`SPEED_MAX`) |
| 14..17 | `uint32` | controller integration coefficient in microseconds |
| 18..19 | `uint16` | effective minimum PWM in counts |
| 20 | `uint8` | capability flags; bit 0 announces deterministic HIL stepping |
| 21 | `uint8` | HIL step operation version (`1`) |
| 22..23 | `uint16` | maximum ticks accepted by one `HIL_STEP` (`1000`) |

The current descriptor is 24 bytes. A legacy 20-byte descriptor does not
announce deterministic stepping; clients must test capability bit 0 rather
than infer support from payload length alone.

## Config parameters

| Param | Name | Payload type | Implemented |
| --- | --- | --- | --- |
| `0x01` | `PWM_FREQ` | `uint16 Hz` | yes |
| `0x02` | `POLE_PAIRS` | `uint8` | yes |
| `0x03` | legacy `KP` | reserved | no, returns `NOT_IMPLEMENTED` |
| `0x04` | legacy `KI` | reserved | no, returns `NOT_IMPLEMENTED` |
| `0x05` | legacy `KD` | reserved | no, returns `NOT_IMPLEMENTED` |
| `0x06` | `MAX_SPEED` | `uint16 RPM` | yes |
| `0x07` | `MIN_SPEED` | `uint16 RPM` | yes |
| `0x08` | `CURRENT_LIMIT` | reserved | no, returns `NOT_IMPLEMENTED` |
| `0x09` | `TEMP_LIMIT` | reserved | no, returns `NOT_IMPLEMENTED` |
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
| `0xFF` | `ALL` | reset/log only | yes for `RESET_CONFIG`, `LOG_START`, `LOG_STOP` |

For gains, value `100` means `1.00`. Algorithm 2 computes RPM error and a
canonical PWM output referenced to ARR=2000 using Q16.16 state and standard
trapezoidal integration. The canonical output is scaled once to active ARR.
`SET_CONFIG` and `RESET_CONFIG` update active RAM and mark pending changes but
do not write flash. A v1 flash configuration preserves non-gain fields and
resets only gains to v2 defaults (`KP=0.28`, `KI=1.00`, `KD=0`). V1, V2, and
V3 flash blocks are validated with their original size and CRC and migrated to
V4. V1/V2 receive startup defaults of `20% -> 100%`, `2.09 Hz -> 9.28 Hz`, and
`3 s`; every legacy version receives the compatible blanking default of `0 us`.

When `app_state == CLOSEDLOOP`, changes to `PWM_FREQ`, `KP_RPM`, `KI_RPM`, and
zero `KD_RPM` apply immediately. KD remains blocked until a fresh-sample RPM
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
| `0x01` | `SPEED` |
| `0x02` | `TEMP` |
| `0x03` | `CURRENT_U` (`PA3`, ADC1_IN3; hardware actualmente desconectado) |
| `0x04` | `CURRENT_V` (`PA4`, ADC2_IN4; hardware actualmente desconectado) |
| `0x05` | `BEMF_PERIOD` (periodo filtrado de capturas `TIM2`) |
| `0xFF` | `ALL` |

`LOG_RATE` accepts `100..5000 ms`. Telemetry is sent as `type=0x82`, `opcode=0x33`.

Telemetry event payload:

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | log param |
| 1..4 | `int32` | value: RPM, centi-degrees C, or mA |
| 5..8 | `uint32` | `HAL_GetTick()` timestamp |

Firmware actual emite una extension de 6 bytes, manteniendo intactos los primeros
9 bytes para receptores anteriores:

| Offset | Type | Meaning |
| --- | --- | --- |
| 9..10 | `uint16` | cuentas ADC raw; para velocidad/BEMF, periodo en ticks |
| 11 | `uint8` | flags: bit 0 valido, bit 1 calibrado, bit 2 saturado, bit 3 sobrecorriente |
| 12..13 | `uint16` | cantidad acumulada de muestras validas |
| 14 | `int8` | sector de conmutacion activo |

`CURRENT_U` y `CURRENT_V` usan conversiones inyectadas simultaneas de ADC1/ADC2,
disparadas internamente por `TIM1_CH4`. La escala nominal en firmware asume
`VDDA=3.3 V` y shunts de `0.47 ohm` (~`1.714 mA` por cuenta ADC). **Estado
actual del hardware:** el sensado de corriente no esta conectado en la placa
actual (se descarto usar `SENSE_A`/`SENSE_B` del L298; pendiente etapa dedicada
con acondicionamiento/op-amp por fase), por lo que estas lecturas no representan
corriente real hasta incorporar el hardware nuevo.

`BEMF_PERIOD` no representa una tension analogica. Expone el periodo y calidad
de la ruta existente LM339/TIM2; la conmutacion sensorless sigue usando esa ruta.

Temperature telemetry remains a placeholder until a real sensor exists.

## HIL mode

Control modes:

| Value | Name |
| --- | --- |
| `0` | `NORMAL` |
| `1` | `MONITOR_ONLY` |
| `2` | `HIL_SIM` |

In `HIL_SIM`, the MCU is blind to ESC commutation and real TIM2 input-capture
events are ignored. Simulink owns the simulated plant and ESC/commutation.
`HIL_START` selects one of two execution modes. Periodic mode keeps the existing
live/UDP behavior: `HIL_SET_INPUTS` updates the latest simulated speed, TIM4
runs the PI tick, and `HIL_GET_OUTPUTS` is read periodically for monitoring.
Stepped mode is reserved for deterministic offline replay: TIM4 remains
stopped and each `HIL_STEP` synchronously applies one input, executes exactly
the requested number of PI ticks, and returns the resulting output.

`HIL_START` accepts these payload shapes:

| Length | Offset | Type | Meaning |
| --- | --- | --- | --- |
| 0 | - | - | default 50 ms input timeout, periodic mode |
| 2 | 0..1 | `uint16` | `input_timeout_ms` (`10..5000`), periodic mode |
| 3 | 0..1 | `uint16` | `input_timeout_ms` (`10..5000`) |
| 3 | 2 | `uint8` | execution mode: `0=periodic`, `1=stepped` |

Starting HIL resets the PI dynamic state and validation provenance. A stepped
session cannot be restarted in place: issue `HIL_STOP` before another
`HIL_START`. `HIL_STOP` stops the logical controller, clears PWM, and restores
the runtime control mode to `NORMAL`.

`HIL_SET_INPUTS` payload:

| Offset | Type | Meaning |
| --- | --- | --- |
| 0..1 | `uint16` | simulated speed RPM |
| 2..3 | `uint16` | reserved, ignored |
| 4..5 | `int16` | load torque, reserved for future use |
| 6 | `uint8` | input flags |
| 7 | `uint8` | enable: `0=stop`, nonzero=start/update |
| 8..11 | `uint32` | optional validated-input `run_id` |
| 12..15 | `uint32` | optional validated-input `source_seq` |

The legacy payload is exactly 8 bytes and remains supported. The validated payload is exactly 16 bytes; its first 8 bytes are identical to the legacy payload. Other payload lengths are rejected. For a legacy input, validation provenance reports `run_id=0` and `source_seq=0`.

`HIL_GET_OUTPUTS` payload:

| Offset | Type | Meaning |
| --- | --- | --- |
| 0..3 | `uint32` | `HAL_GetTick()` timestamp |
| 4 | `uint8` | `app_state` |
| 5 | `uint8` | control mode |
| 6..7 | `uint16` | speed setpoint RPM |
| 8..9 | `uint16` | measured/simulated speed RPM |
| 10..11 | `uint16` | logical PWM command |
| 12 | `int8` | commutation step |
| 13 | `uint8` | HIL flags |
| 14 | `uint8` | HIL timeout |
| 15..18 | `uint32` | accepted `run_id` |
| 19..22 | `uint32` | accepted `source_seq` |
| 23..26 | `uint32` | output generation observed when that input was accepted |
| 27..30 | `uint32` | `run_id` used for the current logical PWM |
| 31..34 | `uint32` | `source_seq` used for the current logical PWM |
| 35..38 | `uint32` | current logical-PWM output generation |
| 39..42 | `uint32` | `HAL_GetTick()` when that logical PWM was updated |

`HIL_GET_OUTPUTS` now has a 43-byte payload. Bytes `0..14` retain their legacy layout exactly. The extension is an interrupt-safe snapshot: `accepted_*` identifies the most recently accepted HIL input and the controller output generation already present at acceptance, while `applied_*`, `output_generation`, and `pwm_update_tick` identify the input and control update that produced the current logical PWM. `HIL_START` resets the PI dynamic state and all validation provenance to zero; each HIL PI update increments `output_generation` and records its tick.

`HIL_STEP` request payload (exactly 18 bytes):

| Offset | Type | Meaning |
| --- | --- | --- |
| 0..1 | `uint16` | simulated speed RPM |
| 2..3 | `uint16` | reserved, must be zero |
| 4..5 | `int16` | load torque, reserved and currently required to be zero |
| 6 | `uint8` | input flags |
| 7 | `uint8` | enable, must be exactly `1` |
| 8..11 | `uint32` | nonzero `run_id` |
| 12..15 | `uint32` | nonzero, strictly increasing `source_seq` |
| 16..17 | `uint16` | requested PI ticks `N`, `1..maximum_hil_steps` |

`HIL_STEP` response payload (exactly 48 bytes):

| Offset | Type | Meaning |
| --- | --- | --- |
| 0..42 | bytes | complete 43-byte `HIL_GET_OUTPUTS` snapshot |
| 43..44 | `uint16` | requested ticks `N` |
| 45..46 | `uint16` | applied ticks; must equal `N` |
| 47 | `uint8` | result flags; bit 0 means idempotent replay |

For a newly accepted command, `accepted_generation` is captured before the PI
ticks and `output_generation - accepted_generation` must equal exactly `N`
without overflow. `accepted_*` and `applied_*` must identify the request. The
firmware retains the meaningful fields of the last stepped command. Repeating
the same `run_id`, `source_seq`, speed, flags, and `N` returns the current
logical result with the replay flag and executes no additional ticks. Reusing
the provenance with different content, changing `run_id` during a session, or
sending a non-increasing new `source_seq` is rejected.

`HIL_STEP` is valid only in an active stepped session. While that session is
active, `HIL_SET_INPUTS`, another `HIL_START`, setpoint/control-mode changes,
and configuration set/reset/save commands return `INVALID_STATE`; use
`HIL_STOP` to leave the session. The input timeout still bounds a stalled host,
but wall-clock timing never advances the controller.

The ESC Bridge exposes a UDP loopback bridge for Simulink on `127.0.0.1:5055`. Send ASCII CSV:

`seq,speed_rpm,enable`

The bridge also accepts command-prefixed packets:

`PIL,speed_rpm,enable`

`PIL,seq,speed_rpm,enable`

For the live UDP diagnostic path, the bridge also accepts provenance-tagged
inputs:

`PILV,run_id,seq,speed_rpm,enable`

`PILV` carries its `run_id` and `seq` in the validated 16-byte
`HIL_SET_INPUTS` payload. This remains part of periodic live HIL and does not
provide deterministic controller stepping. Persistent offline validation uses
the stepped `HIL_START` mode and one `HIL_STEP` request/response per MAT sample;
it does not poll `HIL_GET_OUTPUTS` to infer completion.

For compatibility with older HIL blocks, the bridge still accepts:

`seq,speed_rpm,reserved,load_torque,flags,enable`

The `reserved` field is ignored and no zero-crossing or commutation event is generated. The GUI exposes a `/pil` page that shows recent UDP frames from Simulink and recent binary HID frames exchanged with the MCU for PIL/HIL commands.

The legacy response prefix is ASCII CSV:

`ok,seq,tick_ms,app_state,mode,setpoint_rpm,measured_rpm,pwm_command,commutation_step,flags,timeout,rx_frames,lost_frames,effective_hz,avg_rtt_ms,jitter_ms`

Responses to `PILV` retain that numeric prefix and append:

`input_run_id,accepted_run_id,accepted_source_seq,accepted_generation,applied_run_id,applied_source_seq,output_generation,pwm_update_tick_ms,fresh_output,cache_age_ms`

The response `seq` remains the source sequence. In this live diagnostic,
`applied_*` and `output_generation` describe the cached PWM provenance; response
arrival time and the echoed input sequence alone do not prove which input
produced it. This diagnostic metadata is not a substitute for `HIL_STEP` during
deterministic replay.

The bridge treats high-rate UDP values as latest-value signals, not as a command
FIFO. If packets accumulate while the HID transaction is in progress, older
`SETPOINT` and HIL input packets are coalesced and the newest packet is applied.
This prevents stale Simulink samples from being replayed late. `ESTOP` keeps
priority over coalesced packets. HIL outputs are polled for monitoring at a
lower rate than HIL inputs, so the response may contain the latest cached output
sample while the newest input sample is still applied immediately. These
coalescing and polling rules apply to live periodic HIL/UDP only, never to a
deterministic stepped validation session.

The same UDP port also accepts real-motor commands that do not enter `HIL_SIM`.
The GUI must be in `Simulink control` mode for these commands, except `ESTOP`,
which is always allowed:

| UDP ASCII command | Meaning | Response |
| --- | --- | --- |
| `SETPOINT,<rpm>` | Send `SET_SPEED_RPM` to the MCU | `ok,setpoint,0,setpoint_rpm,actual_rpm,app_state` |
| `SETPOINT,<seq>,<rpm>` | Same, with sequence echoed in the response | `ok,setpoint,seq,setpoint_rpm,actual_rpm,app_state` |
| `RUN` | Send real `RUN` command to the MCU | `ok,run,0,setpoint_rpm,actual_rpm,app_state` |
| `MOTOR_STOP` or `REAL_STOP` | Send real `STOP` command to the MCU | `ok,stop,0,setpoint_rpm,actual_rpm,app_state` |
| `ESTOP` | Send real emergency stop | `ok,estop,0,setpoint_rpm,actual_rpm,app_state` |
| `HIL_START` or legacy `START` | Start HIL simulation mode | `ok,start` |
| `HIL_STOP` or legacy `STOP` | Stop HIL simulation mode | `ok,stop` |

For real-motor setpoint control from Simulink, use `SETPOINT,<seq>,<rpm>`.
Receiving an `ok,setpoint,...` response means the bridge received the UDP packet,
sent the binary HID command to the MCU, and received an OK response from the MCU.
For latency, this path updates the bridge cached setpoint but does not issue a
`GET_STATUS` after every packet.

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
| `:GET:CURRENT_LIMIT` | opcode `0x20 GET_CONFIG`, param `0x08`, returns `NOT_IMPLEMENTED` |
| `:SET:CURRENT_LIMIT:<value>` | opcode `0x21 SET_CONFIG`, param `0x08`, returns `NOT_IMPLEMENTED` |
| `:GET:TEMP_LIMIT` | opcode `0x20 GET_CONFIG`, param `0x09`, returns `NOT_IMPLEMENTED` |
| `:SET:TEMP_LIMIT:<value>` | opcode `0x21 SET_CONFIG`, param `0x09`, returns `NOT_IMPLEMENTED` |
| `:LOG:START:SPEED` | opcode `0x30 LOG_START`, param `0x01` |
| `:LOG:START:TEMP` | opcode `0x30 LOG_START`, param `0x02` |
| `:LOG:START:CRRT` | opcode `0x30 LOG_START`, param `0x03` |
| `:LOG:START:ALL` | opcode `0x30 LOG_START`, param `0xFF` |
| `:LOG:STOP:SPEED` | opcode `0x31 LOG_STOP`, param `0x01` |
| `:LOG:STOP:TEMP` | opcode `0x31 LOG_STOP`, param `0x02` |
| `:LOG:STOP:CRRT` | opcode `0x31 LOG_STOP`, param `0x03` |
| `:LOG:STOP:ALL` | opcode `0x31 LOG_STOP`, param `0xFF` |
| `:LOG:RATE:<ms>` | opcode `0x32 LOG_RATE`, payload `uint16 ms` |
| `:HELP` | removed from main channel; use this document |
