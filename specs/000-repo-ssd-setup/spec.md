# 000 Repo SSD Setup

Status: Implemented

## Problem

The repository has firmware, GUI, bridge, protocol, and PIL/HIL assets, but no versioned SSD workflow and no GitHub Actions CI/CD configuration. Protocol changes also need clearer shared ownership across firmware and GUI.

## Objective

Create the initial SSD structure, documentation indexes, local agent guidance files, and GitHub Actions CI/CD scaffolding without cleaning existing tracked build or generated artifacts.

## Scope

In scope:

- Add SSD feature folder conventions and templates.
- Add initial `000-repo-ssd-setup` spec package.
- Add documentation for repo map, build/test commands, CI/CD, and protocol ownership.
- Add `CLAUDE.md` synchronized with root `AGENTS.md`.
- Add firmware and GUI local `AGENTS.md` files.
- Add GitHub Actions CI, release, docs workflows, and Dependabot configuration.
- Update root README links.

Out of scope:

- Firmware behavior changes.
- GUI behavior changes.
- Binary protocol changes.
- Cleanup of existing tracked build, IDE, publish, or Simulink artifacts.
- Hardware flashing or lab validation from CI.

## Affected Subprojects

- Firmware: Docs only
- GUI/Bridge: Docs only
- Protocol: Docs and CI guard only
- PIL/HIL/Simulink: Docs only
- Docs/CI: Yes

## Protocol Impact

No protocol behavior changes. This feature only documents protocol ownership and adds a CI guard for protocol-sensitive pull requests.

## Acceptance Criteria

- `CLAUDE.md` matches `AGENTS.md`.
- `firmware/AGENTS.md` and `gui/EscGui/AGENTS.md` exist with subproject-specific rules.
- `docs/` includes build/test, CI/CD, repo map, and protocol ownership docs.
- `specs/` includes templates and this initial setup spec.
- GitHub Actions workflows exist for CI, release packaging, and docs artifact generation.
- Dependabot is configured for GitHub Actions and NuGet.
- README links to the new docs and SSD workflow.
- Local firmware Debug build and GUI build/test are attempted and recorded.
