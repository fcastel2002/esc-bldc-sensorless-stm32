# SSD Workflow

SSD means "specification-driven development" for this repository. Each non-trivial feature gets a versioned folder that describes the problem, design, tasks, tests, decisions, and changelog.

## Feature Folder Convention

```text
specs/NNN-kebab-case-feature/
```

- `NNN` is a three-digit sequence.
- `000` is reserved for the repository SSD/CI setup.
- Names use ASCII kebab-case.
- Branches should use `feature/NNN-kebab-case-feature`.
- Commits should start with `NNN:` and describe the completed sub-feature.

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

Pull requests should include:

- spec link;
- task IDs completed;
- tests executed;
- affected docs;
- protocol impact statement when firmware/GUI protocol files change.

## Templates

Use the files in [`templates/`](templates/) as starting points for new features.
