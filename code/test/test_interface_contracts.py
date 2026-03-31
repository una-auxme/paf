"""Contract tests for ROS interface package manifests."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

pytestmark = pytest.mark.unit

INTERFACE_PACKAGE_XML = [
    Path("/workspace/code/mapping_interfaces/package.xml"),
    Path("/workspace/code/perception_interfaces/package.xml"),
    Path("/workspace/code/planning_interfaces/package.xml"),
]

REQUIRED_TEXT_TAGS = {
    "buildtool_depend": {"ament_cmake", "rosidl_default_generators"},
    "exec_depend": {"rosidl_default_runtime"},
    "member_of_group": {"rosidl_interface_packages"},
}


@pytest.mark.parametrize("package_xml_path", INTERFACE_PACKAGE_XML)
def test_interface_package_contract(package_xml_path: Path) -> None:
    """Ensure each interface package.xml contains the expected rosidl contract."""
    assert package_xml_path.exists()

    root = ET.parse(package_xml_path).getroot()
    assert root.tag == "package"

    for tag_name, required_values in REQUIRED_TEXT_TAGS.items():
        values = {
            node.text.strip()
            for node in root.findall(tag_name)
            if node.text and node.text.strip()
        }
        assert required_values.issubset(values), (
            f"Missing {tag_name} entries in {package_xml_path.name}: "
            f"expected {sorted(required_values)}, got {sorted(values)}"
        )
