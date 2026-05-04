"""Host-runnable tests for pure control helper logic."""

from __future__ import annotations

import importlib
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.unit

CODE_ROOT = Path(__file__).resolve().parents[1]
CONTROL_SRC = CODE_ROOT / "control"
if str(CONTROL_SRC) not in sys.path:
    sys.path.insert(0, str(CONTROL_SRC))


@pytest.fixture(scope="module")
def vehicle_control_logic():
    """Import the pure vehicle control helper module."""
    return importlib.import_module("control.vehicle_control_logic")


@pytest.fixture(scope="module")
def velocity_control_logic():
    """Import the pure velocity control helper module."""
    return importlib.import_module("control.velocity_control_logic")


def test_vehicle_control_manual_override_uses_absolute_throttle(
    vehicle_control_logic,
) -> None:
    """Manual override must not leak negative throttle magnitudes."""
    state = vehicle_control_logic.VehicleControlState(
        manual_override_active=True,
        manual_steer=0.25,
        manual_throttle=-0.4,
        emergency=False,
        current_behavior=None,
        reverse=False,
        throttle=0.0,
        brake=0.0,
        pure_pursuit_steer=0.6,
    )

    command = vehicle_control_logic.build_vehicle_control_command(state)

    assert command.reverse is True
    assert command.throttle == pytest.approx(0.4)
    assert command.brake == pytest.approx(0.0)
    assert command.steer == pytest.approx(0.25)
    assert command.hand_brake is False
    assert command.manual_gear_shift is False


def test_vehicle_control_emergency_maps_to_safe_stop(vehicle_control_logic) -> None:
    """Emergency handling should consistently reuse the safe-stop behavior."""
    state = vehicle_control_logic.VehicleControlState(
        manual_override_active=False,
        manual_steer=0.1,
        manual_throttle=0.8,
        emergency=True,
        current_behavior="lane_following",
        reverse=True,
        throttle=0.7,
        brake=0.2,
        pure_pursuit_steer=0.3,
    )

    command = vehicle_control_logic.build_vehicle_control_command(state)
    safe_stop = vehicle_control_logic.build_safe_stop_command()

    assert command == safe_stop


@pytest.mark.parametrize(
    ("current_behavior", "expected_steer"),
    [("us_unstuck", 0.35), ("lane_following", -0.35), (None, -0.35)],
)
def test_vehicle_control_respects_unstuck_steer_sign(
    vehicle_control_logic,
    current_behavior: str | None,
    expected_steer: float,
) -> None:
    """Keep unstuck behavior aligned with the intended steering sign."""
    state = vehicle_control_logic.VehicleControlState(
        manual_override_active=False,
        manual_steer=0.0,
        manual_throttle=0.0,
        emergency=False,
        current_behavior=current_behavior,
        reverse=True,
        throttle=0.55,
        brake=0.15,
        pure_pursuit_steer=0.35,
    )

    command = vehicle_control_logic.build_vehicle_control_command(state)

    assert command.reverse is True
    assert command.throttle == pytest.approx(0.55)
    assert command.brake == pytest.approx(0.15)
    assert command.steer == pytest.approx(expected_steer)


def test_select_target_velocity_prefers_fixed_speed(velocity_control_logic) -> None:
    """Fixed-speed mode should fully override requested target velocities."""
    assert velocity_control_logic.select_target_velocity(
        requested_target_velocity=3.5,
        fixed_speed_active=True,
        fixed_speed=-1.25,
    ) == pytest.approx(-1.25)


@pytest.mark.parametrize(
    ("target_velocity", "current_velocity", "expected"),
    [
        (
            -2.5,
            4.0,
            {"reverse": True, "setpoint": 2.5, "measurement": -4.0, "brake": 0.0},
        ),
        (
            0.05,
            1.5,
            {"reverse": False, "setpoint": None, "measurement": None, "brake": 1.0},
        ),
        (
            6.0,
            1.5,
            {"reverse": False, "setpoint": 6.0, "measurement": 1.5, "brake": 0.0},
        ),
    ],
)
def test_plan_velocity_control_covers_reverse_stop_and_drive_modes(
    velocity_control_logic,
    target_velocity: float,
    current_velocity: float,
    expected: dict[str, float | bool | None],
) -> None:
    """Verify the controller plans the right PID inputs for each mode."""
    plan = velocity_control_logic.plan_velocity_control(
        target_velocity=target_velocity,
        current_velocity=current_velocity,
    )

    assert plan.reverse is expected["reverse"]
    if expected["setpoint"] is None:
        assert plan.setpoint is None
    else:
        assert plan.setpoint == pytest.approx(expected["setpoint"])
    if expected["measurement"] is None:
        assert plan.measurement is None
    else:
        assert plan.measurement == pytest.approx(expected["measurement"])
    assert plan.brake == pytest.approx(expected["brake"])


@pytest.mark.parametrize(
    ("reverse", "pid_output", "expected_throttle", "expected_brake"),
    [(False, 0.42, 0.42, 0.0), (True, -0.3, 0.0, 0.3)],
)
def test_finalize_velocity_control_splits_pid_output_into_actuators(
    velocity_control_logic,
    reverse: bool,
    pid_output: float,
    expected_throttle: float,
    expected_brake: float,
) -> None:
    """Signed PID output should map deterministically to throttle and brake."""
    command = velocity_control_logic.finalize_velocity_control(
        reverse=reverse,
        pid_output=pid_output,
    )

    assert command.reverse is reverse
    assert command.throttle == pytest.approx(expected_throttle)
    assert command.brake == pytest.approx(expected_brake)


def test_control_nodes_delegate_branch_logic_to_pure_helpers() -> None:
    """Keep the ROS nodes wired to the extracted pure helper modules."""
    vehicle_source = (CONTROL_SRC / "control/vehicle_controller.py").read_text(
        encoding="utf-8"
    )
    velocity_source = (CONTROL_SRC / "control/velocity_controller.py").read_text(
        encoding="utf-8"
    )

    assert "build_vehicle_control_command" in vehicle_source
    assert "plan_velocity_control" in velocity_source
    assert "finalize_velocity_control" in velocity_source
