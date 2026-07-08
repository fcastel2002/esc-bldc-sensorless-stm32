# 001 Repo Hygiene Cleanup Test Plan

## Local Checks

- [ ] `git status --short --branch`
- [ ] `git ls-files -ci --exclude-standard` returns no files.
- [ ] `git ls-files slprj out simulacion_agitador/__pycache__` returns no files.
- [ ] `git check-ignore -v slprj/_sfprj/sim_motor/amsi_serial.mat out/pseudo/app_statemachine/app_statemachine.png simulacion_agitador/__pycache__/x.pyc`
- [ ] Local equivalent of `repo-hygiene`.
- [ ] Local equivalent of `ssd-docs`.
- [ ] From `firmware/`: `cmake --preset Debug -DESC_BUILD_CODE_DOCS=OFF`
- [ ] From `firmware/`: `cmake --build --preset Debug`
- [ ] From repo root: `dotnet build gui\EscGui\src\Esc.Web\Esc.Web.csproj`
- [ ] From repo root: `dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj`

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

Record exact commands, dates, and outcomes before moving the spec out of `Draft`.
