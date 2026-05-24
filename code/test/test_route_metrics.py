"""Unit tests for lightweight route metrics helpers."""

from __future__ import annotations

import json
import importlib
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.unit

CODE_ROOT = Path(__file__).resolve().parents[1]
PAF_COMMON_SRC = CODE_ROOT / "paf_common"
if str(PAF_COMMON_SRC) not in sys.path:
    sys.path.insert(0, str(PAF_COMMON_SRC))

route_metrics = importlib.import_module("paf_common.route_metrics")
increment_route_metric = route_metrics.increment_route_metric
load_route_metrics = route_metrics.load_route_metrics
merge_route_metrics_into_checkpoint = route_metrics.merge_route_metrics_into_checkpoint
reset_route_metrics_file = route_metrics.reset_route_metrics_file


def test_increment_route_metric_accumulates_counts(tmp_path: Path) -> None:
    metrics_path = tmp_path / "route_metrics.json"

    assert increment_route_metric("alpha", path=metrics_path) == 1
    assert increment_route_metric("alpha", amount=2, path=metrics_path) == 3
    assert load_route_metrics(metrics_path)["metrics"] == {"alpha": 3}


def test_reset_route_metrics_file_removes_previous_snapshot(tmp_path: Path) -> None:
    metrics_path = tmp_path / "route_metrics.json"
    increment_route_metric("alpha", path=metrics_path)

    reset_route_metrics_file(metrics_path)

    assert load_route_metrics(metrics_path) == {"metrics": {}}


def test_merge_route_metrics_into_checkpoint_adds_summary(tmp_path: Path) -> None:
    checkpoint_path = tmp_path / "simulation_results.json"
    metrics_path = tmp_path / "route_metrics.json"
    checkpoint_path.write_text(
        json.dumps({"_checkpoint": {"global_record": {}}, "labels": [], "values": []}),
        encoding="utf-8",
    )
    increment_route_metric("alpha", amount=2, path=metrics_path)

    merged_metrics = merge_route_metrics_into_checkpoint(
        checkpoint_path,
        metrics_path=metrics_path,
    )
    checkpoint = json.loads(checkpoint_path.read_text(encoding="utf-8"))

    assert merged_metrics["metrics"] == {"alpha": 2}
    assert checkpoint["paf_metrics"]["metrics"] == {"alpha": 2}
    assert checkpoint["_checkpoint"]["global_record"]["paf_metrics"]["metrics"] == {
        "alpha": 2
    }
