# 000 Repo SSD Setup Design

## Summary

Keep the existing firmware and GUI roots unchanged, add SSD as a documentation and process layer, and add CI/CD as GitHub Actions workflows. The protocol remains the shared contract between firmware and GUI.

## Repository Structure

```text
.
|-- AGENTS.md
|-- .agents/
|   |-- README.md
|-- .github/
|   |-- workflows/
|   |   |-- ci.yml
|   |   |-- release.yml
|   |   |-- docs.yml
|   |-- dependabot.yml
|-- docs/
|   |-- README.md
|   |-- build-and-test.md
|   |-- ci-cd.md
|   |-- architecture/
|       |-- repo-map.md
|       |-- protocol-ownership.md
|-- specs/
|   |-- README.md
|   |-- templates/
|   |-- 000-repo-ssd-setup/
|-- firmware/
|   |-- AGENTS.md
|-- gui/
|   |-- EscGui/
|       |-- AGENTS.md
```

## Ownership

- `firmware/` remains the canonical embedded firmware root.
- `gui/EscGui/` remains the canonical GUI and bridge root.
- `AGENTS.md` files are the canonical active agent instructions.
- `.agents/` is reserved for optional, tool-neutral agent support material.
- `.codex/` and root `CLAUDE.md` are intentionally not versioned.
- `firmware/COMM_PROTOCOL.md` and `gui/EscGui/src/Esc.Protocol/` define the shared protocol contract.
- `simulacion_agitador/` and `slprj/` remain PIL/HIL/Simulink support assets.

## CI/CD

The CI workflow separates independent concerns into jobs:

- repository hygiene;
- firmware Debug build;
- GUI build and tests;
- protocol guard;
- SSD docs validation.

Release and docs workflows are separate so normal PR validation stays focused and faster.

## Non-Goals

- No cleanup of tracked generated artifacts in this feature.
- No branch protection changes are made from the repository.
- No GitHub Pages publication is enabled.
- No hardware flashing is automated.
