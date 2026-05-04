"""Pure helpers for deriving vehicle control commands."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleControlState:
    """Inputs needed to derive a vehicle control command."""

    manual_override_active: bool
    manual_steer: float
    manual_throttle: float
    emergency: bool
    current_behavior: str | None
    reverse: bool
    throttle: float
    brake: float
    pure_pursuit_steer: float


@dataclass(frozen=True)
class VehicleControlCommand:
    """Pure representation of the outgoing vehicle command."""

    reverse: bool
    throttle: float
    brake: float
    steer: float
    hand_brake: bool
    manual_gear_shift: bool = False


def build_vehicle_control_command(
    state: VehicleControlState,
) -> VehicleControlCommand:
    """Build a vehicle control command from the current controller state."""
    if state.manual_override_active:
        return VehicleControlCommand(
            reverse=state.manual_throttle < 0.0,
            throttle=abs(state.manual_throttle),
            brake=0.0,
            steer=state.manual_steer,
            hand_brake=False,
        )

    if state.emergency:
        return build_safe_stop_command()

    steer = (
        state.pure_pursuit_steer
        if state.current_behavior == "us_unstuck"
        else -state.pure_pursuit_steer
    )
    return VehicleControlCommand(
        reverse=state.reverse,
        throttle=state.throttle,
        brake=state.brake,
        steer=steer,
        hand_brake=False,
    )


def build_safe_stop_command() -> VehicleControlCommand:
    """Return the conservative safe-stop command used on faults/timeouts."""
    return VehicleControlCommand(
        reverse=False,
        throttle=0.0,
        brake=1.0,
        steer=0.0,
        hand_brake=True,
    )
