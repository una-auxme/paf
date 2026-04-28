# Development Hardening Notes

## Task

Improve the repository for better day-to-day development by strengthening agent-facing guidance, preserving debugging scratch space, and capturing reasoning in-repo.

## Sources inspected

- `doc/dev_talks/paf25/future_work.md`
- `doc/dev_talks/paf25/improvements_assessment.md`
- `agents.md`
- `doc/development/context_retention.md`
- `tmp/robot_sf_ll7/AGENTS.md`
- `tmp/robot_sf_ll7/.agent/PLANS.md`
- `tmp/robot_sf_ll7/.github/copilot-instructions.md`

## Main conclusions

1. The next useful repository-wide work is consolidation, not more feature breadth.
2. PAF already has a strong root `agents.md`, but it lacked smaller agent-facing support files for planning, doc sync, and repository-wide cleanup work.
3. A tracked `tmp/` folder should exist for ignored scratch outputs such as cloned reference repositories, debug notes, or local experiments.
4. Reasoning and adaptation notes should live in a dedicated folder so they are preserved without diluting canonical docs.

## Adapted ideas from `robot_sf_ll7`

Safe to port directly:

- a lightweight cross-agent entrypoint (`CLAUDE.md`)
- a planning convention file for non-trivial work (`.agent/PLANS.md`)
- scoped agent instructions instead of expanding the top-level rules endlessly

Used only as inspiration:

- a large skill tree under `.agents/skills/`
- a second canonical instruction file under `.github/copilot-instructions.md`

The second point was intentionally not copied because this repository already uses `agents.md` as its canonical instruction file.

## Recommended next follow-ups

1. Audit stale docs in `doc/perception/`, `doc/planning/`, and `doc/mapping/` against the current ROS2 interfaces.
2. Add more focused tests around the known weak points called out in the PAF25 notes: radar motion quality, cross-traffic logic, and tracking stability.
3. Consider adding a small repo-local skill set later if the team wants repeatable AI workflows beyond the current instruction files.
