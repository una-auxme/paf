---
description: "Use when doing repository-wide cleanup, development workflow changes, cross-package fixes, or quality hardening. Covers consolidation-before-expansion, interface discipline, and proof-first validation for PAF."
name: "Repository Consolidation"
---
# Repository Consolidation

- Use `doc/dev_talks/paf25/future_work.md` and `doc/dev_talks/paf25/improvements_assessment.md` as the direction-setting documents for repo-wide changes.
- Prefer consolidation work before new feature breadth: sync active docs, clarify interfaces, strengthen tests and CI, and reduce friction in development workflows.
- When a change touches more than one package or mixes workflow plus runtime behavior, make the controlling contract explicit: name the package boundary, topic/message, task, or doc that defines the expected behavior.
- Keep changes small and reversible. If the best answer is larger than one reviewable slice, implement the smallest slice now and record the rest as follow-up work.
- State proof before implementation. Typical proof in this repo is one of: Ruff lint/format, host smoke tests, ROS-backed unit tests, route-level validation, or docs/link verification.
- If a change updates an interface or development rule, update the canonical documentation in the same change or record the gap in `doc/reasoning/`.
