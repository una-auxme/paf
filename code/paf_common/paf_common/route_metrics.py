"""Helpers for lightweight route-level metrics recorded across ROS nodes."""

from __future__ import annotations

import fcntl
import json
import os
from pathlib import Path
from typing import Any


DEFAULT_ROUTE_METRICS_PATH = Path("/tmp/paf_route_metrics.json")
ROUTE_METRICS_ENV_VAR = "PAF_ROUTE_METRICS_PATH"


def get_route_metrics_path(path: str | os.PathLike[str] | None = None) -> Path:
    """Resolve the route metrics file path."""
    if path is not None:
        return Path(path)

    env_path = os.environ.get(ROUTE_METRICS_ENV_VAR)
    if env_path:
        return Path(env_path)

    return DEFAULT_ROUTE_METRICS_PATH


def reset_route_metrics_file(
    path: str | os.PathLike[str] | None = None,
) -> Path:
    """Delete any stale route metrics file before a fresh run."""
    metrics_path = get_route_metrics_path(path)
    metrics_path.parent.mkdir(parents=True, exist_ok=True)
    metrics_path.unlink(missing_ok=True)
    return metrics_path


def increment_route_metric(
    metric_name: str,
    *,
    amount: int = 1,
    path: str | os.PathLike[str] | None = None,
) -> int:
    """Atomically increment a route metric and return the new counter value."""
    metrics_path = get_route_metrics_path(path)
    metrics_path.parent.mkdir(parents=True, exist_ok=True)

    with metrics_path.open("a+", encoding="utf-8") as metrics_file:
        fcntl.flock(metrics_file.fileno(), fcntl.LOCK_EX)
        metrics_file.seek(0)
        raw_content = metrics_file.read().strip()
        metrics_data: dict[str, Any] = (
            json.loads(raw_content) if raw_content else {"metrics": {}}
        )
        counters = metrics_data.setdefault("metrics", {})
        counters[metric_name] = int(counters.get(metric_name, 0)) + amount

        metrics_file.seek(0)
        metrics_file.truncate()
        json.dump(metrics_data, metrics_file, indent=4, sort_keys=True)
        metrics_file.write("\n")
        metrics_file.flush()
        os.fsync(metrics_file.fileno())
        fcntl.flock(metrics_file.fileno(), fcntl.LOCK_UN)

    return int(counters[metric_name])


def load_route_metrics(
    path: str | os.PathLike[str] | None = None,
) -> dict[str, Any]:
    """Load the current route metrics snapshot from disk."""
    metrics_path = get_route_metrics_path(path)
    if not metrics_path.exists():
        return {"metrics": {}}

    raw_content = metrics_path.read_text(encoding="utf-8").strip()
    if not raw_content:
        return {"metrics": {}}
    return json.loads(raw_content)


def merge_route_metrics_into_checkpoint(
    checkpoint_path: str | os.PathLike[str],
    *,
    metrics_path: str | os.PathLike[str] | None = None,
) -> dict[str, Any]:
    """Merge route metrics into the leaderboard checkpoint json."""
    metrics = load_route_metrics(metrics_path)
    if not metrics.get("metrics"):
        return metrics

    checkpoint_file = Path(checkpoint_path)
    if not checkpoint_file.exists():
        return metrics

    checkpoint = json.loads(checkpoint_file.read_text(encoding="utf-8"))
    checkpoint["paf_metrics"] = metrics
    checkpoint.setdefault("_checkpoint", {}).setdefault("global_record", {})[
        "paf_metrics"
    ] = metrics
    checkpoint_file.write_text(
        json.dumps(checkpoint, indent=4, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return metrics
