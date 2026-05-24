# Agent Handoff Rules Update

Created: 2026-04-29T10:09:26+02:00
Last updated: 2026-04-29T10:09:26+02:00

## Motivation

Codify a single repository rule for preserving current chat state at handoff time so future agent sessions always leave behind a timestamped progress note plus documentation hints that point to it.

## Sources inspected

- `agents.md`
- `CLAUDE.md`
- `.github/instructions/docs-and-reasoning.instructions.md`
- `doc/development/context_retention.md`
- `doc/dev/README.md`
- `doc/README.md`

## Current conclusion

The repository previously had conflicting guidance:

- `agents.md` and `CLAUDE.md` treated `doc/reasoning/` as the home for handoff notes,
- the newer workflow already introduced `doc/dev/progress/` for timestamped chat-state handoffs,
- `doc/README.md` pointed directly to one specific handoff note instead of the `doc/dev/` index.

The rule set is now aligned so that:

1. timestamped progress and handoff notes live in `doc/dev/progress/`,
2. deeper analysis and comparison notes stay in `doc/reasoning/`,
3. new progress notes must be discoverable through documentation hints such as `doc/dev/README.md`, `doc/README.md`, or the nearest relevant development or module documentation.

## Validation

- Checked the edited instruction and documentation files for editor-reported errors.

## Follow-up

- Future non-trivial chats should either create or update a timestamped note in `doc/dev/progress/` before the agent finishes.
- Keep `doc/README.md` pointing to the `doc/dev/` index rather than rotating a hard-coded link to a single note.
