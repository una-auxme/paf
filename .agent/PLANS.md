# Planning Convention

Use this file when the work is large enough that an agent should externalize its plan before or during implementation.

## When To Write A Plan

Write a plan when the task:

- spans multiple packages or mixes code, docs, and workflow changes,
- changes repository behavior, contributor workflow, or validation expectations,
- updates active interfaces, launch flows, or runtime assumptions,
- or is likely to require follow-up work after the current change.

Skip formal planning only for narrow, obviously local edits.

## Plan Template

Keep plans short and operational:

```md
# Goal
- One or two sentences on the desired outcome.

# Boundaries
- What is in scope.
- What is explicitly out of scope.

# Evidence
- Files, docs, tests, configs, or upstream references that define the contract.

# Steps
- Ordered implementation steps.

# Validation
- Commands to run.
- Evidence that will prove the change works in this repository.

# Risks / Follow-ups
- Remaining uncertainty, deferred scope, or issue candidates.
```

## Required Behaviors

- Restate the repository goal in PAF terms, not generic assistant language.
- Separate observed evidence from assumptions.
- Prefer committed scripts, VS Code tasks, and documented workflows over ad-hoc commands.
- For non-trivial development work, state the proof obligation before implementation.
- Prefer the consolidation order from `doc/dev_talks/paf25/future_work.md`: documentation and interfaces first, then tests and CI, then subsystem fixes.
- If scope expands, capture a follow-up in `doc/reasoning/` or the issue tracker instead of silently broadening the change.

## Review Expectations

A good plan makes it easy for a reviewer to answer:

- what changed,
- why that scope is correct,
- how it was validated,
- and what risk remains.
