# Project Documentation

This directory contains the durable project documentation for the mixed STM32 firmware and local Blazor GUI ESC project.

## Start Here

- [Build and test](build-and-test.md): local and CI command reference.
- [CI/CD](ci-cd.md): GitHub Actions workflows, artifacts, and release rules.
- [Architecture repo map](architecture/repo-map.md): ownership boundaries for firmware, GUI, protocol, and PIL/HIL support.
- [Protocol ownership](architecture/protocol-ownership.md): required checklist for shared binary protocol changes.
- [Project flow](PROJECT_FLOW.md): firmware execution flow.
- [PIL/Simulink](PIL_SIMULINK.md): PIL/HIL integration notes.
- [Bridge walkthrough](BRIDGE_WALKTHROUGH.md): GUI bridge and transport details.

## SSD Specs

Feature specs live under [`../specs/`](../specs/). Use one folder per feature:

```text
specs/NNN-kebab-case-feature/
```

`000-repo-ssd-setup` is reserved for the initial SSD and CI/CD repository setup.
