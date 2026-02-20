# Logging Standard

## Objectives

- Keep logs machine-parseable and human-readable.
- Preserve request/scenario context across module boundaries.
- Make failures actionable during autonomous runs.

## Rules

- Use Python `logging`; do not use `print` in runtime node logic.
- Use levels consistently:
  - `DEBUG`: detailed diagnostics
  - `INFO`: lifecycle and state transitions
  - `WARNING`: degraded behavior with fallback
  - `ERROR`: failed operation requiring attention
- Include stable context keys where available (for example `route_id`, `scenario_id`, `module`, `node_name`).

## Shared utilities

Common helpers are available in `paf_common`:

- `configure_logging(...)` to configure stream/JSON logging.
- `with_log_context(...)` to bind stable context fields.

Use JSON output for CI and batch processing contexts, and text output for local interactive development.
