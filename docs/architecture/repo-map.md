# Repository Map

This repository is a mixed embedded firmware and local GUI project. Firmware and GUI share a fixed binary protocol, while PIL/HIL assets support integration and simulation.

## Roots

```text
.
|-- firmware/              STM32F103C8T6 firmware
|-- gui/EscGui/            Local Blazor GUI, bridge, protocol, tests
|-- .agents/               Tool-neutral agent support conventions
|-- docs/                  Project documentation
|-- specs/                 SSD feature specs and templates
|-- simulacion_agitador/   Simulink/PIL/HIL helper scripts and models
|-- slprj/                 Simulink generated support artifacts
|-- INFORME/               Paper/report and hardware documentation
|-- out/                   Generated diagrams and auxiliary outputs
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

PIL/HIL and Simulink integration are shared integration support, not a third primary application root.

## Generated And Historical Artifacts

The repository currently contains tracked build, IDE, publish, and Simulink generated artifacts. Do not clean them in unrelated feature work. A cleanup should be handled as a separate, explicit PR.
