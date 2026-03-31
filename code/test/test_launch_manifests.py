"""Static smoke tests for ROS launch manifest files."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

pytestmark = pytest.mark.unit

LAUNCH_XML_FILES = [
    Path("/workspace/code/acting/launch/acting.xml"),
    Path("/workspace/code/agent/launch/agent.dev.xml"),
    Path("/workspace/code/control/launch/control.xml"),
    Path("/workspace/code/localization/launch/localization.xml"),
    Path("/workspace/code/mapping/launch/mapping.xml"),
    Path("/workspace/code/perception/launch/perception.xml"),
    Path("/workspace/code/planning/launch/planning.dev.xml"),
]


@pytest.mark.parametrize("launch_path", LAUNCH_XML_FILES)
def test_launch_xml_is_well_formed(launch_path: Path) -> None:
    """Validate that core launch XML files are present and parseable."""
    assert launch_path.exists()
    root = ET.parse(launch_path).getroot()
    assert root.tag == "launch"


def test_agent_launch_contains_core_subsystems() -> None:
    """Ensure the dev agent launch includes the subsystem launch files."""
    agent_launch = Path("/workspace/code/agent/launch/agent.dev.xml")
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
    persistent_launch = Path("/workspace/code/agent/launch/agent.dev.persistent.xml")
    root = ET.parse(persistent_launch).getroot()
    included_files = {include.get("file", "") for include in root.findall("include")}

    assert any("localization.xml" in file_path for file_path in included_files)
    assert any(
        "planning.dev.persistent.xml" in file_path for file_path in included_files
    )
