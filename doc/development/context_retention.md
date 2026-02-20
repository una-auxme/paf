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

## When to write an ADR

Write an ADR when you:

- Change package boundaries or interfaces.
- Introduce a new dependency or framework.
- Make a policy decision (linting, testing, release process).
- Replace an established behavior with a new default.
