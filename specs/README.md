# SSD Workflow

SSD means "specification-driven development" for this repository. Changes that need durable design and cross-subsystem traceability get a versioned folder that describes the problem, design, tasks, tests, decisions, and changelog.

SSD is required for protocol or persisted-data contracts, architectural ownership changes, coordinated firmware/GUI/Simulink behavior, and multi-stage features with material design decisions. It is not required for localized fixes, routine UI work, documentation, dependencies, or CI maintenance. See [`docs/development-workflow.md`](../docs/development-workflow.md) for the canonical criteria.

## Feature Folder Convention

```text
specs/NNN-kebab-case-feature/
```

- `NNN` is a three-digit sequence.
- `000` is reserved for the repository SSD/CI setup.
- Names use ASCII kebab-case.
- Branches use `feat/NNN-kebab-case-feature`.
- Commits use Conventional Commit subjects and remain focused on completed sub-features.

## Required Files

Every feature folder must include:

- `spec.md`
- `tasks.md`
- `test-plan.md`

Recommended files:

- `design.md`
- `changelog.md`
- `decisions/NNN-short-decision.md`

## States

- `Draft`: problem, objective, scope, and affected subprojects are being written.
- `Accepted`: the feature is approved for implementation.
- `In Progress`: implementation tasks are active.
- `Implemented`: code and docs are complete.
- `Verified`: local validation and CI have passed, and the test plan is updated.
- `Superseded`: another spec replaces this one.

## Traceability

Pull requests for SSD-backed changes should include:

- spec link;
- task IDs completed;
- tests executed;
- affected docs;
- protocol impact statement when firmware/GUI protocol files change.

## Templates

Use the files in [`templates/`](templates/) as starting points for new features.
