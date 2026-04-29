---
description: "Use when updating documentation, architecture notes, contributor guidance, or saved reasoning. Covers doc sync with code, canonical docs vs reasoning notes, and the doc/reasoning workflow."
name: "Docs And Reasoning"
applyTo: "doc/**/*.md,README.md,agents.md"
---
# Docs And Reasoning

- Active documentation must match the current implementation. If code and docs disagree, either fix the doc in the same change or call out the gap explicitly.
- Keep canonical behavior and interface docs in their domain folders under `doc/` or the package docs. Use `doc/dev/progress/` for timestamped progress and handoff notes that capture the current chat state, and use `doc/reasoning/` for deeper analysis notes, comparisons, migration thoughts, and development output that should not become the source of truth.
- When saving a progress or handoff note, include the timestamp, motivating task, current chat state, source files or repositories inspected, validation status, remaining blockers, and recommended follow-ups.
- When saving a reasoning note, include the motivating task, the source files or repositories inspected, the main conclusion, and the remaining follow-ups.
- Link new documentation from `doc/README.md` or the most relevant existing index page so it stays discoverable.
- Add documentation hints for new progress notes by updating `doc/dev/README.md`, `doc/README.md`, or the closest relevant module or development doc.
- Prefer short, actionable Markdown over long narrative dumps. Remove or update stale statements instead of piling on contradictory notes.
