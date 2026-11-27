# Repository Map

This repository is a mixed embedded firmware and local GUI project. Firmware and GUI share a fixed binary protocol, while Simulink assets support offline modelling of the stirrer.

## Roots

```text
.
|-- firmware/              STM32F103C8T6 firmware
|-- gui/EscGui/            Local Blazor GUI, bridge, protocol, tests
|-- .agents/               Tool-neutral agent support conventions
|-- docs/                  Project documentation
|-- specs/                 SSD feature specs and templates
|-- simulacion_agitador/   Simulink models of the stirrer (offline)
|-- INFORME/               Paper/report and hardware documentation
```

## Firmware Ownership

`firmware/` is the canonical firmware root.

High-value files:

- `Core/Src/comm_protocol.c`: protocol command effects and frame handling.
- `Core/Src/comm_transport.c`: boot-time transport selection.
- `Core/Src/comm.c`: periodic communication and telemetry facade.
- `Core/Src/state_machine.c`: ESC state machine.
- `Core/Src/motor_control.c`: motor control, stall, and PI logic.
- `Core/Src/speed_sensor.c`: speed measurement.
- `COMM_PROTOCOL.md`: firmware-facing protocol documentation.

## GUI Ownership

`gui/EscGui/` is the canonical GUI and bridge root.

High-value files:

- `src/Esc.Protocol/CommConstants.cs`: protocol constants.
- `src/Esc.Protocol/EscProtocol.cs`: frame serialization and parsing.
- `src/Esc.Bridge/EscBridgeService.cs`: HID ownership and bridge state.
- `src/Esc.Bridge/BridgeEndpointMapper.cs`: HTTP endpoints.
- `src/Esc.Web/Components/Pages/Home.razor`: main UI.
- `tests/Esc.Tests/`: GUI protocol and bridge tests.

## Shared Surface

The protocol is shared across firmware and GUI. Changes to opcodes, frame payloads, constants, CRC behavior, config semantics, or telemetry fields must be treated as cross-project changes.

Simulink modelling is offline support, not a third primary application root. The live PIL/HIL path to the MCU was removed in spec 002.

## Generated Artifacts

Build outputs, IDE metadata, publish folders, Python caches, `slprj/`, and `out/` are generated or local-only artifacts and should remain ignored. `slprj/` is produced by Simulink, while `out/` holds auxiliary generated outputs such as diagrams.

If a future generated artifact needs to become source, document the ownership change before adding an exception to `.gitignore`.
