---
description: "Use when updating documentation, architecture notes, contributor guidance, or saved reasoning. Covers doc sync with code, canonical docs vs reasoning notes, and the doc/reasoning workflow."
name: "Docs And Reasoning"
applyTo: "doc/**/*.md,README.md,agents.md"
---
# Docs And Reasoning

- Active documentation must match the current implementation. If code and docs disagree, either fix the doc in the same change or call out the gap explicitly.
- Keep canonical behavior and interface docs in their domain folders under `doc/` or the package docs. Use `doc/reasoning/` for analysis notes, comparisons, migration thoughts, and development output that should not become the source of truth.
- When saving a reasoning note, include the motivating task, the source files or repositories inspected, the main conclusion, and the remaining follow-ups.
- Link new documentation from `doc/README.md` or the most relevant existing index page so it stays discoverable.
- Prefer short, actionable Markdown over long narrative dumps. Remove or update stale statements instead of piling on contradictory notes.
