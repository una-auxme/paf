"""Unit tests for deterministic startup and frame synchronization helpers."""

from __future__ import annotations

import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import importlib

import pytest

pytestmark = pytest.mark.unit

CODE_ROOT = Path(__file__).resolve().parents[1]
PAF_COMMON_SRC = CODE_ROOT / "paf_common"
if str(PAF_COMMON_SRC) not in sys.path:
    sys.path.insert(0, str(PAF_COMMON_SRC))


@pytest.fixture(scope="module")
def sync_helpers():
    return importlib.import_module("paf_common.sync")


def test_startup_readiness_tracker_requires_all_nodes(sync_helpers) -> None:
    tracker = sync_helpers.StartupReadinessTracker(["data_management", "mapping"])

    assert not tracker.all_ready()
    assert tracker.missing_nodes() == ["data_management", "mapping"]

    tracker.update("data_management", True)
    assert not tracker.all_ready()
    assert tracker.missing_nodes() == ["mapping"]

    tracker.update("mapping", True)
    assert tracker.all_ready()
    assert tracker.missing_nodes() == []


def test_frame_barrier_requires_current_frame_completion(sync_helpers) -> None:
    barrier = sync_helpers.FrameBarrier(["mapping", "acc", "velocity_controller"])
    barrier.begin_frame(12, started_at=1.0)

    barrier.mark_stage_complete("mapping", 12)
    barrier.mark_stage_complete("acc", 11)
    barrier.mark_stage_complete("velocity_controller", 12)

    assert not barrier.is_ready()
    assert barrier.missing_stages() == ["acc"]

    barrier.mark_stage_complete("acc", 12)
    assert barrier.is_ready()


def test_frame_barrier_timeout_and_clear_pending(sync_helpers) -> None:
    barrier = sync_helpers.FrameBarrier(["mapping"])
    barrier.begin_frame(3, started_at=5.0)

    assert not barrier.timed_out(5.4, 0.5)
    assert barrier.timed_out(5.5, 0.5)

    barrier.clear_pending()
    assert barrier.pending_frame_id is None
    assert barrier.pending_started_at is None


@pytest.mark.parametrize(
    ("time_ns", "expected_frame_id"),
    [
        (0, 0),
        (50_000_000, 1),
        (99_999_999, 1),
        (100_000_000, 2),
    ],
)
def test_frame_id_from_time_ns_uses_fixed_delta(
    sync_helpers, time_ns: int, expected_frame_id: int
) -> None:
    assert sync_helpers.frame_id_from_time_ns(time_ns, 0.05) == expected_frame_id


def test_sync_contract_files_reflect_barrier_design() -> None:
    control_xml = (CODE_ROOT / "control/launch/control.xml").read_text()
    control_yaml = (CODE_ROOT / "control/config/control.yaml").read_text()
    data_management_source = (
        CODE_ROOT / "agent/agent/data_management_node.py"
    ).read_text()
    startup_coordinator_source = (
        CODE_ROOT / "agent/agent/startup_coordinator.py"
    ).read_text()
    paf_agent_source = (
        CODE_ROOT / "leaderboard_launcher/leaderboard_launcher/paf_agent_base.py"
    ).read_text()
    startup_ready_block = data_management_source.split(
        "def _publish_startup_ready_if_available(self) -> None:\n", maxsplit=1
    )[1].split("\n\n    def get_global_plan_service", maxsplit=1)[0]
    startup_required_nodes_block = startup_coordinator_source.split(
        "DEFAULT_REQUIRED_NODES = [\n", maxsplit=1
    )[1].split("]\n\n\nclass StartupCoordinator", maxsplit=1)[0]
    ros_bridge_root = ET.parse(
        CODE_ROOT / "leaderboard_launcher/launch/ros_bridge.dev.xml"
    ).getroot()
    persistent_launch_root = ET.parse(
        CODE_ROOT / "agent/launch/agent.dev.persistent.xml"
    ).getroot()

    assert "loop_sleep_time" not in control_xml
    assert "loop_sleep_time" not in control_yaml
    assert "frame_barrier_timeout" in control_yaml
    assert "sync_frame_delta_seconds" in control_yaml
    assert "self.global_plan is not None" not in startup_ready_block
    assert "self.open_drive_string is not None" in startup_ready_block
    assert '"ego_vehicle_role_name": "\\"[\'hero\']\\""' in paf_agent_source
    assert 'wait_for_message(self.ros_node, "/carla/hero/status"' not in paf_agent_source
    assert '"data_management"' in startup_required_nodes_block
    assert '"motion_planning"' not in startup_required_nodes_block

    wait_for_command_arg = next(
        arg
        for arg in ros_bridge_root.findall("arg")
        if arg.get("name") == "synchronous_mode_wait_for_vehicle_control_command"
    )
    assert wait_for_command_arg.get("default") == "True"

    startup_nodes = {
        node.get("exec", "") for node in persistent_launch_root.findall("node")
    }
    assert "startup_coordinator" in startup_nodes
