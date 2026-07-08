# 000 Repo SSD Setup Test Plan

## Local Checks

- [ ] `git status --short --branch`
- [ ] `if ((Get-FileHash AGENTS.md -Algorithm SHA256).Hash -eq (Get-FileHash CLAUDE.md -Algorithm SHA256).Hash) { "ok" } else { exit 1 }`
- [ ] From `firmware/`: `cmake --preset Debug -DESC_BUILD_CODE_DOCS=OFF`
- [ ] From `firmware/`: `cmake --build --preset Debug`
- [ ] From repo root: `dotnet build gui\EscGui\src\Esc.Web\Esc.Web.csproj`
- [ ] From repo root: `dotnet test gui\EscGui\tests\Esc.Tests\Esc.Tests.csproj`

## CI Checks

- [ ] `repo-hygiene`
- [ ] `firmware-debug`
- [ ] `gui-build-test`
- [ ] `protocol-guard`
- [ ] `ssd-docs`

## Hardware / HIL Checks

- Not required. This feature does not change firmware runtime behavior, GUI runtime behavior, protocol frames, or HIL packet semantics.

## Regression Areas

- Documentation links.
- Agent guidance synchronization.
- Workflow syntax.
- CI path filters and guard scripts.

## Results

Pending local execution in this branch.
