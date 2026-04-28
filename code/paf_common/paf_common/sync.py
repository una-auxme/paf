"""Helpers for deterministic startup and frame synchronization."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Optional


def normalize_sync_id(value: str) -> str:
    """Normalize node and stage ids so topic names stay stable."""
    return value.strip().lower().replace(" ", "_").replace("-", "_")


def startup_topic(role_name: str, node_id: str) -> str:
    """Topic on which a node publishes its startup readiness."""
    normalized = normalize_sync_id(node_id)
    return f"/paf/{role_name}/sync/startup/{normalized}"


def frame_complete_topic(role_name: str, stage_id: str) -> str:
    """Topic on which a stage reports its current completed frame id."""
    normalized = normalize_sync_id(stage_id)
    return f"/paf/{role_name}/sync/frame/{normalized}/completed"


def frame_id_from_time_ns(time_ns: int, frame_delta_seconds: float) -> int:
    """Map a simulation time value onto a monotonic frame id."""
    if frame_delta_seconds <= 0.0:
        raise ValueError("frame_delta_seconds must be positive")

    frame_delta_ns = max(1, int(frame_delta_seconds * 1_000_000_000))
    if time_ns <= 0:
        return 0
    return time_ns // frame_delta_ns


@dataclass
class StartupReadinessTracker:
    """Track whether the required startup nodes have reported readiness."""

    required_nodes: Iterable[str]
    ready_by_node: dict[str, bool] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.required_nodes = tuple(
            normalize_sync_id(node) for node in self.required_nodes
        )
        self.ready_by_node = {node: False for node in self.required_nodes}

    def update(self, node_id: str, is_ready: bool) -> None:
        self.ready_by_node[normalize_sync_id(node_id)] = is_ready

    def missing_nodes(self) -> list[str]:
        return [node for node, ready in self.ready_by_node.items() if not ready]

    def all_ready(self) -> bool:
        return all(self.ready_by_node.get(node, False) for node in self.required_nodes)


@dataclass
class FrameBarrier:
    """Track which stages completed for the currently pending frame."""

    required_stages: Iterable[str]
    completed_by_stage: dict[str, int] = field(default_factory=dict)
    pending_frame_id: Optional[int] = None
    pending_started_at: Optional[float] = None

    def __post_init__(self) -> None:
        self.required_stages = tuple(
            normalize_sync_id(stage) for stage in self.required_stages
        )
        self.completed_by_stage = {stage: -1 for stage in self.required_stages}

    def begin_frame(self, frame_id: int, started_at: float) -> None:
        if self.pending_frame_id is None or frame_id > self.pending_frame_id:
            self.pending_frame_id = frame_id
            self.pending_started_at = started_at

    def mark_stage_complete(self, stage_id: str, frame_id: int) -> None:
        normalized = normalize_sync_id(stage_id)
        last_completed = self.completed_by_stage.get(normalized, -1)
        self.completed_by_stage[normalized] = max(last_completed, frame_id)

    def missing_stages(self, frame_id: Optional[int] = None) -> list[str]:
        current_frame = self.pending_frame_id if frame_id is None else frame_id
        if current_frame is None:
            return list(self.required_stages)
        return [
            stage
            for stage in self.required_stages
            if self.completed_by_stage.get(stage, -1) < current_frame
        ]

    def is_ready(self, frame_id: Optional[int] = None) -> bool:
        return not self.missing_stages(frame_id)

    def timed_out(self, now: float, timeout_seconds: float) -> bool:
        if self.pending_frame_id is None or self.pending_started_at is None:
            return False
        return (now - self.pending_started_at) >= timeout_seconds

    def clear_pending(self) -> None:
        self.pending_frame_id = None
        self.pending_started_at = None
