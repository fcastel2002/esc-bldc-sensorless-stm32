# 001 Repo Hygiene Cleanup Tasks

## Tasks

- [x] T001 Create `feature/001-repo-hygiene-cleanup` from `feature/000-repo-ssd-setup`.
- [x] T002 Add SSD feature documentation for the cleanup.
- [x] T003 Harden `.gitignore` for generated, IDE, firmware build, Python, and test output files.
- [x] T004 Update agent and repository docs for `.slnx`, `slprj/`, and `out/` conventions.
- [x] T005 Extend `repo-hygiene` CI to reject tracked ignored files.
- [x] T006 Untrack generated and ignored artifacts without deleting local files.
- [x] T007 Run local Git, CI-script-equivalent, firmware, and GUI verification.
- [x] T008 Record verification results.

## Notes

Use `git rm --cached` for index cleanup. Do not delete local generated outputs from disk.
