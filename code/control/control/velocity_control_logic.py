"""Pure helpers for deriving velocity control commands."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class VelocityControlPlan:
    """Describe how the controller should evaluate the next longitudinal step."""

    reverse: bool
    setpoint: float | None
    measurement: float | None
    throttle: float = 0.0
    brake: float = 0.0


@dataclass(frozen=True)
class VelocityControlCommand:
    """Pure representation of the outgoing longitudinal command."""

    reverse: bool
    throttle: float
    brake: float


def select_target_velocity(
    requested_target_velocity: float,
    fixed_speed_active: bool,
    fixed_speed: float,
) -> float:
    """Return the effective target velocity after fixed-speed overrides."""
    return fixed_speed if fixed_speed_active else requested_target_velocity


def plan_velocity_control(
    target_velocity: float,
    current_velocity: float,
) -> VelocityControlPlan:
    """Prepare the PID inputs or a direct standstill command."""
    if target_velocity < 0.0:
        return VelocityControlPlan(
            reverse=True,
            setpoint=abs(target_velocity),
            measurement=-current_velocity,
        )

    if target_velocity < 0.1:
        return VelocityControlPlan(
            reverse=False,
            setpoint=None,
            measurement=None,
            throttle=0.0,
            brake=1.0,
        )

    return VelocityControlPlan(
        reverse=False,
        setpoint=target_velocity,
        measurement=current_velocity,
    )


def finalize_velocity_control(
    reverse: bool,
    pid_output: float,
) -> VelocityControlCommand:
    """Translate the signed PID result into throttle and brake outputs."""
    if pid_output < 0.0:
        return VelocityControlCommand(
            reverse=reverse,
            throttle=0.0,
            brake=abs(pid_output),
        )

    return VelocityControlCommand(
        reverse=reverse,
        throttle=pid_output,
        brake=0.0,
    )
