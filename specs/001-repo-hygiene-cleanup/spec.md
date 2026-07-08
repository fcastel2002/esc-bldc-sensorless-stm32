# 001 Repo Hygiene Cleanup

Status: Verified

## Problem

The repository still tracks local IDE metadata, firmware build outputs, generated Simulink support artifacts, Python bytecode, and auxiliary output images. Some ignore rules are duplicated or too broad for the firmware build layout, and repository docs still describe generated directories as normal versioned roots.

## Objective

Remove generated and local-only artifacts from the Git index without deleting local files, harden ignore rules so they stay untracked, and update docs and CI so future changes preserve the intended source/generated boundary.

## Scope

In scope:

- Untrack files that already match `.gitignore`.
- Untrack `slprj/`, `out/`, and `simulacion_agitador/__pycache__/`.
- Preserve firmware, GUI, test, Simulink model, HIL/PIL script, documentation, report, and helper script sources.
- Add ignore rules for Simulink, auxiliary output, Python bytecode, test results, and firmware CMake build outputs.
- Update agent and repository docs for the `.slnx` convention and generated directory ownership.
- Extend `repo-hygiene` CI to fail when ignored files are committed.

Out of scope:

- Firmware runtime behavior changes.
- GUI runtime behavior changes.
- Binary protocol changes.
- Deleting local generated files from disk.
- Changing release packaging, HIL packet semantics, or hardware validation.

## Affected Subprojects

- Firmware: Ignore/build artifact ownership only
- GUI/Bridge: Agent guidance only
- Protocol: No
- PIL/HIL/Simulink: Generated artifact ownership only
- Docs/CI: Yes

## Protocol Impact

No protocol behavior changes. This feature does not modify frame layout, opcodes, constants, config semantics, telemetry, HID, UART, or UDP behavior.

## Acceptance Criteria

- `git ls-files -ci --exclude-standard` returns no files.
- `git ls-files slprj out simulacion_agitador/__pycache__` returns no files.
- `.gitignore` covers Simulink generated outputs, auxiliary outputs, Python caches, test results, and firmware CMake outputs without duplicate entries.
- `repo-hygiene` fails when committed files match ignore rules while preserving existing guards for agent files and transient HIL logs.
- Root and GUI `AGENTS.md` describe `.csproj` commands as official and `gui/EscGui/EscGui.slnx` as auxiliary.
- `README.md` and `docs/architecture/repo-map.md` describe `slprj/` and `out/` as generated/ignored rather than versioned source roots.

## References

- `AGENTS.md`
- `gui/EscGui/AGENTS.md`
- `docs/architecture/repo-map.md`
- `docs/ci-cd.md`
- `.github/workflows/ci.yml`
