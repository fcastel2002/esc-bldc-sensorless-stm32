# Project Documentation

This directory contains the durable project documentation for the mixed STM32 firmware and local Blazor GUI ESC project.

## Start Here

- [Build and test](build-and-test.md): local and CI command reference.
- [Development workflow](development-workflow.md): branches, pull requests, SSD criteria, and merge policy.
- [CI/CD](ci-cd.md): GitHub Actions workflows, artifacts, and release rules.
- [Architecture repo map](architecture/repo-map.md): ownership boundaries for firmware, GUI, protocol, and PIL/HIL support.
- [Protocol ownership](architecture/protocol-ownership.md): required checklist for shared binary protocol changes.
- [Project flow](PROJECT_FLOW.md): firmware execution flow.
- [PIL/Simulink](PIL_SIMULINK.md): PIL/HIL integration notes.
- [Simulation validation runs](SIMULATION_RUNS.md): import, replay, and persistence workflow.
- [Bridge walkthrough](BRIDGE_WALKTHROUGH.md): GUI bridge and transport details.

## SSD Specs

Feature specs live under [`../specs/`](../specs/). Use one folder for each change that meets the SSD criteria:

```text
specs/NNN-kebab-case-feature/
```

`000-repo-ssd-setup` is reserved for the initial SSD and CI/CD repository setup.
