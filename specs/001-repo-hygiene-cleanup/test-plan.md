# 001 Repo Hygiene Cleanup Test Plan

## Local Checks

- [x] `git status --short --branch`
- [x] `git ls-files -ci --exclude-standard` returns no files.
- [x] `git ls-files slprj out simulacion_agitador/__pycache__` returns no files.
- [x] `git check-ignore -v slprj/_sfprj/sim_motor/amsi_serial.mat out/pseudo/app_statemachine/app_statemachine.png simulacion_agitador/__pycache__/x.pyc`
- [x] Local equivalent of `repo-hygiene`.
- [x] Local equivalent of `ssd-docs`.
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

- Repository source/generated boundary.
- `.gitignore` specificity.
- Agent guidance for GUI commands.
- CI hygiene guard behavior.
- SSD folder validation.

## Results

Local execution on 2026-07-08 used the main worktree. Generated outputs produced by build and test commands remained ignored after the cleanup.

- PASS: `git status --short --branch` showed a clean `feature/001-repo-hygiene-cleanup` branch after cleanup and after verification.
- PASS: `git ls-files -ci --exclude-standard` returned no files.
- PASS: `git ls-files slprj out simulacion_agitador/__pycache__` returned no files.
- PASS: `git check-ignore -v slprj/_sfprj/sim_motor/amsi_serial.mat out/pseudo/app_statemachine/app_statemachine.png simulacion_agitador/__pycache__/x.pyc` matched `slprj/`, `out/`, and `__pycache__/` rules.
- PASS: local `repo-hygiene` PowerShell validation.
- PASS: local `ssd-docs` PowerShell validation.
- PASS: firmware `cmake --preset Debug -DESC_BUILD_CODE_DOCS=OFF`.
- PASS: firmware `cmake --build --preset Debug`.
- PASS: `dotnet build gui\EscGui\src\Esc.Web\Esc.Web.csproj`.
- PASS: `dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj` with 18 passed, 0 failed, 0 skipped.

Notes:

- Firmware build reported `FLASH: 65312 B / 64 KB`, 99.66% used.
- GUI build and test emitted `NETSDK1057` because the installed local .NET 10 SDK is a preview build.
- GitHub Actions jobs are not executed locally; they remain pending until the branch is pushed and a PR or matching branch workflow runs.
