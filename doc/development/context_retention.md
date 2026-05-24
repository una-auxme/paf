# Context Retention

## Why

Long-lived projects lose intent when design decisions only live in chat threads or ephemeral PR comments.

## Mechanisms

1. Architecture Decision Records (ADRs)
   - Keep ADRs in `doc/adr/`.
   - Record decision, alternatives, and consequences.
2. PR quality fields
   - Capture assumptions, validation, and known gaps in every PR.
3. Test markers and logs
   - Preserve behavior expectations with marker-based tests and structured logs.
4. Progress and handoff notes for non-trivial work
   - Keep timestamped progress and handoff notes in `doc/dev/progress/` when the current chat state, validation state, blockers, or next actions need to survive the session.
   - Add documentation hints in `doc/dev/README.md`, `doc/README.md`, or the closest relevant development or module doc so the latest handoff is easy to find.
5. Reasoning notes for non-trivial improvement work
   - Keep analysis and comparison notes in `doc/reasoning/` when they are worth preserving but are not the canonical source of truth.
   - Promote the stable parts into `doc/development/`, `doc/<module>/`, or ADRs when the behavior or policy is finalized.

## When to write an ADR

Write an ADR when you:

- Change package boundaries or interfaces.
- Introduce a new dependency or framework.
- Make a policy decision (linting, testing, release process).
- Replace an established behavior with a new default.
