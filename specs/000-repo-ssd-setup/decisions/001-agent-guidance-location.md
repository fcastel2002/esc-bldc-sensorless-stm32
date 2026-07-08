# Decision: Agent Guidance Location

Date: 2026-07-08

Status: Accepted

## Context

The repository had a tracked `.codex/` directory with duplicated agent context, including a `CLAUDE.md` mirror and tool-specific configuration. The SSD setup also initially added a root `CLAUDE.md` mirror. This creates duplication, stale-content risk, and vendor-specific coupling.

## Decision

Use `AGENTS.md` files as the canonical active instructions:

- root `AGENTS.md` for repository-wide rules;
- `firmware/AGENTS.md` for firmware-specific rules;
- `gui/EscGui/AGENTS.md` for GUI and bridge-specific rules.

Use `.agents/` only for optional, tool-neutral agent support material. Do not use `.codex/` for versioned project guidance. Do not keep a root `CLAUDE.md` mirror.

## Consequences

- Codex can keep reading standard `AGENTS.md` files directly.
- The repository avoids duplicated Claude/Codex-specific mirrors.
- Local Codex configuration, MCP configuration, or personal skills stay outside Git unless a future spec explicitly justifies versioning them.
- If project-level reusable skills are later needed, add them through a separate spec and prefer a neutral location under `.agents/`.

## References

- `AGENTS.md`
- `.agents/README.md`
- `specs/000-repo-ssd-setup/spec.md`
