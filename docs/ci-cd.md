# CI/CD

The repository uses GitHub Actions for pull request validation, release packaging, and generated documentation artifacts.

## Workflows

### `ci.yml`

Runs on pushes to `main` and `pil_simul`, and on pull requests targeting those branches.

Jobs:

- `repo-hygiene`: validates spec folder naming, blocks known large transient logs, and checks that `AGENTS.md` and `CLAUDE.md` stay synchronized.
- `firmware-debug`: installs CMake, Ninja, and the Arm GNU toolchain, then runs `cmake --preset Debug -DESC_BUILD_CODE_DOCS=OFF` and `cmake --build --preset Debug`.
- `gui-build-test`: installs .NET `10.0.x`, restores the Blazor GUI project, builds it, and runs xUnit tests.
- `protocol-guard`: when protocol-sensitive files change, requires protocol tests and protocol documentation changes in the same pull request.
- `ssd-docs`: validates SSD feature folders and required files.

### `release.yml`

Runs on tags matching `v*` and on manual dispatch.

Jobs:

- firmware Release build on Ubuntu;
- Windows GUI publish through `gui/EscGui/publish-gui-win-x64.ps1`;
- GitHub Release creation with generated artifacts attached.

Releases are intended for tags named `vMAJOR.MINOR.PATCH`.

### `docs.yml`

Runs on pushes to `main`, tags matching `v*`, and manual dispatch.

It builds Doxygen HTML docs and uploads them as a workflow artifact. GitHub Pages publication is intentionally left out until public docs hosting is explicitly enabled.

## Artifacts

- Pull request and branch build artifacts use short retention.
- Release artifacts use longer retention and are also attached to the GitHub Release.
- Firmware release artifacts include `.elf`, `.map`, and any generated `.bin` or `.hex` files.
- GUI release artifacts include a compressed `gui/EscGui/publish/win-x64` folder.

## Dependabot

Dependabot checks GitHub Actions and NuGet dependencies weekly. Automatic merge is not enabled in this repository configuration; branch protection and CI should be configured before considering automerge.

## Branch Protection Recommendation

Protect `main` with required PR review and required passing checks:

- `repo-hygiene`
- `firmware-debug`
- `gui-build-test`

During the PIL/Simulink migration stage, protect `pil_simul` with the same checks if it remains an active integration branch.

## Hardware Policy

CI does not flash hardware, run HIL against physical devices, or assume USB/HID access. Hardware validation remains a manual or lab-specific step.
