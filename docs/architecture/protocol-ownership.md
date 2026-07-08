# Protocol Ownership

Firmware and GUI share a fixed 64-byte little-endian binary protocol. Protocol compatibility is a project-level contract, not an implementation detail of either subproject.

## Protocol Invariants

- Frame size: 64 bytes.
- Magic: `0xEC 0xB1`.
- Version: `0x01`.
- Endianness: little-endian.
- CRC: CRC16-CCITT-FALSE over bytes `0..61`.
- CRC storage: bytes `62..63`.
- Transport selection happens only at firmware boot through `PB8`.

## Change Checklist

For any protocol-sensitive change:

- Update firmware protocol handling in `firmware/Core/Src/comm_protocol.c`.
- Update related firmware config/control modules when command behavior changes.
- Update GUI protocol constants and serializers under `gui/EscGui/src/Esc.Protocol/`.
- Update bridge, endpoints, UI, or `controls.json` when behavior is observable.
- Update tests under `gui/EscGui/tests/Esc.Tests/`.
- Update `firmware/COMM_PROTOCOL.md`.
- Decide whether new bridge commands must call `RefreshStatusAsync`; most state-changing commands should.
- Record the change in the relevant SSD spec and test plan.

## CI Guard

The `protocol-guard` job treats these paths as protocol-sensitive:

- `firmware/Core/**/comm*`
- `firmware/COMM_PROTOCOL.md`
- `gui/EscGui/src/Esc.Protocol/**`

When a pull request changes protocol code, it must also include protocol documentation and protocol tests unless the change is documentation-only.

## Compatibility Notes

- `SET_CONFIG` and `RESET_CONFIG` update active RAM only.
- Flash persistence requires explicit `SAVE_CONFIG`.
- PI gains are floats internally but protocol payloads use `int16` hundredths.
- Current limit, temperature limit, current telemetry, and temperature telemetry remain placeholders unless explicitly implemented across firmware and GUI.
