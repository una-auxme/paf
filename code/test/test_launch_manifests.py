"""Static smoke tests for ROS launch manifest files."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

pytestmark = pytest.mark.unit

CODE_ROOT = Path(__file__).resolve().parents[1]

LAUNCH_XML_FILES = [
    CODE_ROOT / "acting/launch/acting.xml",
    CODE_ROOT / "agent/launch/agent.dev.xml",
    CODE_ROOT / "control/launch/control.xml",
    CODE_ROOT / "localization/launch/localization.xml",
    CODE_ROOT / "mapping/launch/mapping.xml",
    CODE_ROOT / "perception/launch/perception.xml",
    CODE_ROOT / "planning/launch/planning.dev.xml",
]


@pytest.mark.parametrize("launch_path", LAUNCH_XML_FILES)
def test_launch_xml_is_well_formed(launch_path: Path) -> None:
    """Validate that core launch XML files are present and parseable."""
    assert launch_path.exists()
    root = ET.parse(launch_path).getroot()
    assert root.tag == "launch"


def test_agent_launch_contains_core_subsystems() -> None:
    """Ensure the dev agent launch includes the subsystem launch files."""
    agent_launch = CODE_ROOT / "agent/launch/agent.dev.xml"
    root = ET.parse(agent_launch).getroot()
    included_files = {include.get("file", "") for include in root.findall("include")}

    expected_fragments = {
        "perception.xml",
        "planning.dev.xml",
        "acting.xml",
        "mapping.xml",
    }

    for fragment in expected_fragments:
        assert any(fragment in file_path for file_path in included_files)


def test_agent_persistent_launch_contains_localization() -> None:
    """Ensure persistent launch file keeps localization and persistent planning."""
    persistent_launch = CODE_ROOT / "agent/launch/agent.dev.persistent.xml"
    root = ET.parse(persistent_launch).getroot()
    included_files = {include.get("file", "") for include in root.findall("include")}
    node_execs = {node.get("exec", "") for node in root.findall("node")}

    assert any("localization.xml" in file_path for file_path in included_files)
    assert any(
        "planning.dev.persistent.xml" in file_path for file_path in included_files
    )
    assert "startup_coordinator" in node_execs
