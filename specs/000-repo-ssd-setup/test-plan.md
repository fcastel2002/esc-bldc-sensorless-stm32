# 000 Repo SSD Setup Test Plan

## Local Checks

- [x] `git status --short --branch`
- [x] `Test-Path AGENTS.md`
- [x] `Test-Path .agents/README.md`
- [x] `git ls-files -- "CLAUDE.md" ".codex/*"` returns no files.
- [x] From `firmware/`: `cmake --preset Debug -DESC_BUILD_CODE_DOCS=OFF`
- [x] From `firmware/`: `cmake --build --preset Debug`
- [x] From repo root: `dotnet build gui\EscGui\src\Esc.Web\Esc.Web.csproj`
- [x] From repo root: `dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj`

## CI Checks

- [ ] `repo-hygiene` pending GitHub Actions.
- [ ] `firmware-debug` pending GitHub Actions.
- [ ] `gui-build-test` pending GitHub Actions.
- [ ] `protocol-guard` pending GitHub Actions.
- [ ] `ssd-docs` pending GitHub Actions.

## Hardware / HIL Checks

- Not required. This feature does not change firmware runtime behavior, GUI runtime behavior, protocol frames, or HIL packet semantics.

## Regression Areas

- Documentation links.
- Agent guidance synchronization.
- Workflow syntax.
- CI path filters and guard scripts.

## Results

Local execution on 2026-07-08 used a detached temporary worktree at `C:\Users\Francisco\AppData\Local\Temp\esc-bldc-stm32-verify-20260708171038` so tracked generated artifacts in the main worktree were not modified.

- PASS: `repo-hygiene` PowerShell validation.
- PASS: `ssd-docs` PowerShell validation.
- PASS: agent guidance decision recorded in `specs/000-repo-ssd-setup/decisions/001-agent-guidance-location.md`.
- PASS: firmware `cmake --preset Debug -DESC_BUILD_CODE_DOCS=OFF`.
- PASS: firmware `cmake --build --preset Debug`.
- PASS: `dotnet build gui\EscGui\src\Esc.Web\Esc.Web.csproj`.
- PASS: `dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj` with 18 passed, 0 failed, 0 skipped.

Notes:

- Firmware build emitted existing unused-parameter warnings in generated HAL/USB sources.
- GUI commands emitted `NETSDK1057` because the installed local .NET 10 SDK is a preview build.
- GitHub Actions jobs are not executed locally; they remain pending until the branch is pushed and a PR or matching branch workflow runs.
- `CLAUDE.md` was removed from the setup after review; `AGENTS.md` is the canonical guidance file.
