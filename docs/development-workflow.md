# Development Workflow

This repository uses a lightweight GitHub Flow. `main` is the only long-lived
branch. All normal changes are developed on short-lived branches, validated in
a pull request, squash-merged, and then deleted.

## Start A Change

Start from an up-to-date `main`:

```powershell
git switch main
git pull --ff-only origin main
git switch -c <type>/<short-kebab-case-description>
```

Use one of these branch prefixes:

- `feat/` for user-visible behavior;
- `fix/` for defect corrections;
- `docs/` for documentation-only changes;
- `refactor/` for behavior-preserving code changes;
- `test/` for test-only changes;
- `chore/` for dependencies, CI, tooling, and repository maintenance.

Use lowercase ASCII kebab-case after the prefix. When a change has an SSD spec,
use `feat/NNN-kebab-case-feature` so the branch maps directly to its spec.
Dependabot-managed branches are the only exception to these names.

Do not commit directly to `main` during normal development. The repository
administrator may bypass protection only for an emergency repository recovery;
the bypass must be followed by a normal pull request that records any remaining
work.

## SSD Scope

Create `specs/NNN-kebab-case-feature/` when a change needs durable design and
traceability. SSD is required for:

- binary protocol or persisted-data contract changes;
- architectural changes that alter ownership or subsystem boundaries;
- coordinated behavior changes across firmware, GUI, and/or Simulink;
- multi-stage features with material design decisions or migration work.

SSD is not required for localized bug fixes, routine UI changes,
documentation-only work, dependency updates, or CI/repository maintenance. A
pull request without SSD must state `SSD: not required` and briefly identify why.

## Commits

Keep commits focused and use Conventional Commit subjects:

```text
<type>(<optional-scope>): <imperative summary>
```

Examples:

```text
feat(protocol): report applied HIL generation
fix(gui): preserve validation run metadata
docs(workflow): document branch protection
chore(deps): update NuGet dependencies
```

Useful scopes include `firmware`, `gui`, `protocol`, `simulink`, `ci`, and
`docs`. Before every commit, inspect `git status` and `git diff`, and stage only
the intended files. Never include unrelated local changes or generated output.

## Pull Requests

Open every normal change as a pull request targeting `main`. Keep the branch
short-lived and the pull request focused on one change. Draft pull requests are
appropriate for early CI feedback.

The pull request title must use the Conventional Commit format because it
becomes the squash commit subject. Complete the repository pull request
template, including:

- a concise change summary;
- validation commands and results;
- SSD applicability and link, when required;
- protocol and hardware impact statements.

The repository has one maintainer, so an approving review is not required.
Review conversations must still be resolved before merge.

## Required Checks

`main` requires the branch to be current and these pull request checks to pass:

- `repo-hygiene`;
- `gui-build-test`;
- `protocol-guard`;
- `ssd-docs`.

`firmware-debug` remains visible but is temporarily non-blocking because the
current firmware build is already failing in CI. Make it required after the
underlying build is repaired and the job has completed successfully on `main`.
Hardware, USB/HID, and physical HIL validation remain manual and must be
reported in the pull request when relevant.

## Merge And Cleanup

Use squash merge only. Merge after the required checks pass and the change has
the requested approval or confirmation. GitHub automatically deletes the head
branch after merge. Delete the corresponding local branch after updating
`main`:

```powershell
git switch main
git pull --ff-only origin main
git branch -d <merged-branch>
```

Merge commits and rebase merges are disabled for pull requests. Force pushes
and deletion of `main` are prohibited.

## Dependabot

Dependabot checks GitHub Actions and NuGet weekly. Version updates are grouped
into one pull request per ecosystem to avoid branch accumulation. Dependabot
pull requests use the same required checks and squash merge policy as human
changes. Automatic merge is disabled.

## Releases

Create release tags only from a validated commit on `main`, using
`vMAJOR.MINOR.PATCH`. Pushing the tag starts the release workflow. Do not create
release tags from feature branches.
