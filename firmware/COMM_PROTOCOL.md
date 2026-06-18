# ESC BLDC Binary Communication Protocol

## Transport selection

The communication mode is selected once at boot with `PB8` (`COMM_MODE`) using the internal pull-up:

| PB8 level | Mode |
| --- | --- |
| HIGH or open | UART binary frames on USART1 (`PB6/PB7`, 115200 8N1) |
| LOW | USB FS Custom HID on `PA11/PA12` |

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
| `0x12` | `ESTOP` | ignored | empty | empty |
| `0x13` | `SET_SPEED_RPM` | ignored | `uint16 rpm` | empty |
| `0x20` | `GET_CONFIG` | config param | empty | config value |
| `0x21` | `SET_CONFIG` | config param | config value | empty |
| `0x22` | `RESET_CONFIG` | config param or `0xFF` | empty | empty |
| `0x23` | `SAVE_CONFIG` | config param or `0xFF` | empty | empty |
| `0x30` | `LOG_START` | log param or `0xFF` | empty | empty |
| `0x31` | `LOG_STOP` | log param or `0xFF` | empty | empty |
| `0x32` | `LOG_RATE` | ignored | `uint16 ms` | empty |
| `0x33` | `TELEMETRY_EVENT` | log param | event only | event payload below |

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

## Config parameters

| Param | Name | Payload type | Implemented |
| --- | --- | --- | --- |
| `0x01` | `PWM_FREQ` | `uint16 Hz` | yes |
| `0x02` | `POLE_PAIRS` | `uint8` | yes |
| `0x03` | `KP` | `int16` centesimas | yes |
| `0x04` | `KI` | `int16` centesimas | yes |
| `0x05` | `KD` | `int16` centesimas | yes |
| `0x06` | `MAX_SPEED` | `uint16 RPM` | yes |
| `0x07` | `MIN_SPEED` | `uint16 RPM` | yes |
| `0x08` | `CURRENT_LIMIT` | reserved | no, returns `NOT_IMPLEMENTED` |
| `0x09` | `TEMP_LIMIT` | reserved | no, returns `NOT_IMPLEMENTED` |
| `0xFF` | `ALL` | reset/log only | yes for `RESET_CONFIG`, `LOG_START`, `LOG_STOP` |

For gains, value `135` means `1.35`. `SET_CONFIG` and `RESET_CONFIG` update the active RAM configuration and mark pending changes, but do not write flash. `SAVE_CONFIG` writes the current active configuration to flash. `RESET_CONFIG` supports each implemented config param and `0xFF` for all defaults.

## Logging and telemetry

Log params:

| Param | Name |
| --- | --- |
| `0x01` | `SPEED` |
| `0x02` | `TEMP` |
| `0x03` | `CURRENT` |
| `0xFF` | `ALL` |

`LOG_RATE` accepts `100..5000 ms`. Telemetry is sent as `type=0x82`, `opcode=0x33`.

Telemetry event payload:

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | `uint8` | log param |
| 1..4 | `int32` | value: RPM, centi-degrees C, or mA |
| 5..8 | `uint32` | `HAL_GetTick()` timestamp |

Temperature and current telemetry currently use placeholders until real sensing logic exists.

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
| `:GET:KP` | opcode `0x20 GET_CONFIG`, param `0x03` |
| `:SET:KP:<value>` | opcode `0x21 SET_CONFIG`, param `0x03`, payload `int16 value*100` |
| `:RESET:KP` | opcode `0x22 RESET_CONFIG`, param `0x03` |
| `:GET:KI` | opcode `0x20 GET_CONFIG`, param `0x04` |
| `:SET:KI:<value>` | opcode `0x21 SET_CONFIG`, param `0x04`, payload `int16 value*100` |
| `:RESET:KI` | opcode `0x22 RESET_CONFIG`, param `0x04` |
| `:GET:KD` | opcode `0x20 GET_CONFIG`, param `0x05` |
| `:SET:KD:<value>` | opcode `0x21 SET_CONFIG`, param `0x05`, payload `int16 value*100` |
| `:RESET:KD` | opcode `0x22 RESET_CONFIG`, param `0x05` |
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
