# Performance Iteration Guide

## Goal

Reduce local iteration time while preserving correctness.

## Fast local build strategy

Inside the dev container, prefer package-selective builds during development:

```bash
devbuild.pkg <package_name>
```

Use full rebuilds (`devbuild`) before larger integration checks.

## Suggested daily loop

1. Edit one package.
2. Run `devbuild.pkg <package_name>`.
3. Run `ruff check` and relevant unit tests.
4. Run full workspace checks before PR.

## Profiling guidance

Start with reproducible micro/algorithm-level profiling for Python hotspots.

- Lightweight CPU profiling:

```bash
python -m cProfile -o /tmp/profile.out <script_or_module>
python -m pstats /tmp/profile.out
```

- Sampling profiler for running nodes (inside container):

```bash
py-spy top --pid <python_pid>
```

Prioritize fixes only for top hot paths with measurable impact.

## CI/runtime performance considerations

- Keep fast unit tests separate from integration/simulation jobs.
- Avoid enabling heavy simulation or profiling in default PR workflows.
- Track benchmark regressions in dedicated scheduled jobs.
