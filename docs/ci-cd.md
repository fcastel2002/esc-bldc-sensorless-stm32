# CI/CD

The repository uses GitHub Actions for pull request validation, release packaging, and generated documentation artifacts.

## Workflows

### `ci.yml`

Runs on pushes to `main` and on pull requests targeting `main`.

Jobs:

- `repo-hygiene`: validates spec folder naming, rejects tracked files that match `.gitignore`, blocks known large transient logs, checks that root `AGENTS.md` and `.agents/README.md` exist, and blocks committed `.codex/` or root `CLAUDE.md` files.
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

Dependabot checks GitHub Actions and NuGet dependencies weekly. Version updates are grouped into one pull request per ecosystem, with at most two open version-update pull requests per ecosystem. Automatic merge is disabled; Dependabot pull requests follow the same checks and squash policy as human changes.

## Branch Protection

`main` is the only long-lived branch. Normal changes require a pull request, a current branch, resolved review conversations, linear history, and these passing checks:

- `repo-hygiene`
- `gui-build-test`
- `protocol-guard`
- `ssd-docs`

`firmware-debug` is temporarily non-blocking because its existing CI build fails. Add it to the required checks after the build is repaired and the job succeeds on `main`.

Pull requests use squash merge only, and GitHub automatically deletes merged head branches. Merge commits, rebase merges, force pushes to `main`, and deletion of `main` are disabled. Because the repository has one maintainer, zero approving reviews are required; review conversations must still be resolved.

See [Development Workflow](development-workflow.md) for branch names, SSD criteria, commit conventions, and the complete pull request lifecycle.

## Hardware Policy

CI does not flash hardware, run HIL against physical devices, or assume USB/HID access. Hardware validation remains a manual or lab-specific step.
