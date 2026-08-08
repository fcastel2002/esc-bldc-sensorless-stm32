# Firmware Agent Instructions

## Scope

- This directory is the canonical STM32 firmware project for the ESC.
- Target MCU: STM32F103C8T6.
- Build system: CMake presets with Ninja and `arm-none-eabi-gcc`.
- Protocol changes are shared work. Any frame/opcode/config change usually requires firmware, GUI protocol, bridge/UI, tests, and protocol docs updates.

## Commands

- Configure Debug from `firmware/`: `cmake --preset Debug`
- Build Debug from `firmware/`: `cmake --build --preset Debug`
- Configure Release from `firmware/`: `cmake --preset Release`
- Build Release from `firmware/`: `cmake --build --preset Release`
- Faster local Debug configure without Doxygen post-build docs: `cmake --preset Debug -DESC_BUILD_CODE_DOCS=OFF`

## Invariants

- Communication transport is selected only at boot by `PB8`.
- `PB8` HIGH/open selects USB FS Custom HID on `PA11/PA12`.
- `PB8` LOW selects UART on `USART1 PB6/PB7 115200 8N1`.
- Do not add runtime transport switching.
- Protocol frames are fixed at 64 bytes.
- Frames use little-endian fields, magic `0xEC 0xB1`, version `0x01`, and CRC16-CCITT-FALSE over bytes `0..61`.
- `SET_CONFIG` and `RESET_CONFIG` update active RAM only. Flash persistence requires explicit `SAVE_CONFIG`.
- RPM PI v2 stores gains as floats but executes Q16.16; `KP_RPM/KI_RPM` payloads use `int16` hundredths. KD is fixed at zero.
- In `CLOSEDLOOP`, `PWM_FREQ`, `KP`, `KI`, and `KD` apply hot. Other config changes use the deferred `CONFIG` path.
- `KD` acts on measured-speed derivative, not error derivative.
- `CURRENT_LIMIT` and `TEMP_LIMIT` remain `NOT_IMPLEMENTED`. `CURRENT_U`/`CURRENT_V` telemetry comes from synchronized low-side shunts; temperature telemetry remains a placeholder.
- `foc_startup()` is historical naming: it is open-loop sinusoidal/SPWM startup, not field-oriented control and not a current loop.

## Critical Files

- Protocol effects: `Core/Src/comm_protocol.c`
- Transport selection: `Core/Src/comm_transport.c`
- Periodic communication and telemetry facade: `Core/Src/comm.c`
- Firmware state machine: `Core/Src/state_machine.c`
- Motor control and stall/PI logic: `Core/Src/motor_control.c`
- Speed measurement: `Core/Src/speed_sensor.c`
- Commutation: `Core/Src/bldc_driver.c`
- HAL callbacks: `Core/Src/isr_callbacks.c`
- Runtime config and flash persistence: `Core/Src/hard_config.c`, `Core/Src/flash_config.c`

## Editing Rules

- CubeMX-generated files may contain `USER CODE BEGIN/END` regions. Avoid editing generated sections outside those regions unless the task explicitly requires it.
- C formatting and lint config live at repo root in `.clang-format` and `.clang-tidy`.
- Keep protocol behavior documented in `COMM_PROTOCOL.md`.

## Protocol Change Checklist

- Update firmware protocol/control modules.
- Update GUI protocol constants and serializers in `gui/EscGui/src/Esc.Protocol/`.
- Update bridge/UI paths when command behavior is observable.
- Update GUI tests under `gui/EscGui/tests/Esc.Tests/`.
- Update `firmware/COMM_PROTOCOL.md`.
- Decide whether state-changing bridge commands must call `RefreshStatusAsync`; most do.
