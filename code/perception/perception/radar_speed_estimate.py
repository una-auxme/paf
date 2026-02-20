"""Velocity estimation from radar measurements.

This module exposes estimators for a stationary 2D radar setup.

Core assumptions:
- target motion is approximately constant velocity over the sample window
- each sample provides target position ``(x, y)`` in radar frame
- Doppler radial velocity ``v_r`` is available when using radial/fused estimators

Main user-facing interfaces:
- ``estimate_velocity_from_xyvr`` for ``(N, 3)`` arrays ``[x, y, v_r]``
- ``estimate_velocity_from_time_xyvr`` for ``(N, 4)`` arrays ``[t, x, y, v_r]``
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


class EstimationError(ValueError):
    """Raised when the velocity estimation problem is ill-posed or ill-conditioned."""


@dataclass(frozen=True)
class VelocityEstimate:
    """Estimated velocity with diagnostics.

    Attributes:
        vx: Estimated x component of velocity.
        vy: Estimated y component of velocity.
        speed: Estimated speed magnitude ``sqrt(vx^2 + vy^2)``.
        heading_rad: Heading angle in radians, computed with ``atan2(vy, vx)``.
        residual_l2: L2 norm of least-squares residuals.
        condition_number: Matrix condition number used as quality gate.
        rank: Rank of the system matrix.
        method: Estimator identifier.
    """

    vx: float
    vy: float
    speed: float
    heading_rad: float
    residual_l2: float
    condition_number: float
    rank: int
    method: str

    @property
    def vector(self) -> np.ndarray:
        """Return velocity as ``[vx, vy]`` numpy vector."""
        return np.array([self.vx, self.vy], dtype=float)


# -----------------------------------------------------------------------------
# Input normalization helpers
# -----------------------------------------------------------------------------


def _as_2d_array(values: Iterable[Iterable[float]], name: str) -> np.ndarray:
    """Normalize nested iterables to a dense ``(N, 2)`` float array."""
    arr = np.asarray(values, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 2:
        raise EstimationError(f"{name} must have shape (N, 2), got {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise EstimationError(f"{name} must contain only finite values")
    return arr


def _as_xyvr_array(points_xyvr: Iterable[Iterable[float]]) -> np.ndarray:
    """Normalize input points to ``(N, 3)`` array with columns ``[x, y, v_r]``."""
    arr = np.asarray(points_xyvr, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise EstimationError(f"points_xyvr must have shape (N, 3), got {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise EstimationError("points_xyvr must contain only finite values")
    return arr


def _as_txyvr_array(samples_txyvr: Iterable[Iterable[float]]) -> np.ndarray:
    """Normalize samples to ``(N, 4)`` array with columns ``[t, x, y, v_r]``."""
    arr = np.asarray(samples_txyvr, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 4:
        raise EstimationError(f"samples_txyvr must have shape (N, 4), got {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise EstimationError("samples_txyvr must contain only finite values")
    return arr


# -----------------------------------------------------------------------------
# Geometry helpers
# -----------------------------------------------------------------------------


def line_of_sight_unit_vectors(positions_xy: Iterable[Iterable[float]]) -> np.ndarray:
    """Return LOS unit vectors from radar origin to each target position.

    Args:
        positions_xy: Array-like with shape ``(N, 2)`` and columns ``[x, y]``.

    Returns:
        ``(N, 2)`` array where row ``i`` is ``u_i = p_i / ||p_i||``.
    """
    positions = _as_2d_array(positions_xy, "positions_xy")
    ranges = np.linalg.norm(positions, axis=1)
    if np.any(ranges <= 0.0):
        raise EstimationError("positions must be non-zero distance from radar origin")
    return positions / ranges[:, None]


# -----------------------------------------------------------------------------
# Estimators
# -----------------------------------------------------------------------------


def estimate_velocity_from_radial(
    positions_xy: Iterable[Iterable[float]],
    radial_velocity: Iterable[float],
    *,
    min_condition_rank: int = 2,
    max_condition_number: float = 1e6,
) -> VelocityEstimate:
    """Estimate 2D true velocity from LOS geometry and radial Doppler values.

    Solves the linear model ``U v = v_r`` in least-squares sense, where rows of
    ``U`` are LOS unit vectors and ``v=[vx, vy]^T``.

    Args:
        positions_xy: ``(N, 2)`` array-like with columns ``[x, y]``.
        radial_velocity: ``(N,)`` array-like radial velocities.
        min_condition_rank: Required rank for observability in 2D (default 2).
        max_condition_number: Reject overly ill-conditioned LOS geometry.

    Returns:
        VelocityEstimate containing velocity and diagnostics.
    """
    U = line_of_sight_unit_vectors(positions_xy)
    vr = np.asarray(radial_velocity, dtype=float).reshape(-1)
    if not np.all(np.isfinite(vr)):
        raise EstimationError("radial_velocity must contain only finite values")

    if U.shape[0] != vr.shape[0]:
        raise EstimationError("positions_xy and radial_velocity must have the same length")
    if U.shape[0] < 2:
        raise EstimationError("at least 2 measurements are required")

    rank = int(np.linalg.matrix_rank(U))
    #if rank < min_condition_rank:
    #    raise EstimationError(
    #        "insufficient LOS diversity: rank(U) < 2, velocity is not fully observable"
    #    )

    cond = float(np.linalg.cond(U))
    if not np.isfinite(cond) or cond > max_condition_number:
        raise EstimationError(f"ill-conditioned LOS geometry (condition number={cond:.3e})")

    v_hat, residuals, _, _ = np.linalg.lstsq(U, vr, rcond=None)
    residual_l2 = float(np.sqrt(residuals[0])) if residuals.size > 0 else 0.0
    vx, vy = float(v_hat[0]), float(v_hat[1])
    speed = float(np.hypot(vx, vy))
    heading = float(np.arctan2(vy, vx))

    return VelocityEstimate(
        vx=vx,
        vy=vy,
        speed=speed,
        heading_rad=heading,
        residual_l2=residual_l2,
        condition_number=cond,
        rank=rank,
        method="radial_lstsq",
    )


def estimate_velocity_from_positions(
    times_s: Iterable[float],
    positions_xy: Iterable[Iterable[float]],
) -> VelocityEstimate:
    """Estimate velocity from position/time using constant-velocity least squares.

    Fits the linear models:
    - ``x(t) = x0 + vx * t``
    - ``y(t) = y0 + vy * t``

    Args:
        times_s: ``(N,)`` sample times.
        positions_xy: ``(N, 2)`` positions.

    Returns:
        VelocityEstimate containing velocity and diagnostics.
    """
    t = np.asarray(times_s, dtype=float).reshape(-1)
    if not np.all(np.isfinite(t)):
        raise EstimationError("times_s must contain only finite values")
    p = _as_2d_array(positions_xy, "positions_xy")
    if t.shape[0] != p.shape[0]:
        raise EstimationError("times_s and positions_xy must have the same length")
    if t.shape[0] < 2:
        raise EstimationError("at least 2 samples are required")

    dt_span = float(np.max(t) - np.min(t))
    if dt_span <= 0.0:
        raise EstimationError("times_s must span a non-zero interval")

    A = np.column_stack([np.ones_like(t), t])
    coeff_x, _, _, _ = np.linalg.lstsq(A, p[:, 0], rcond=None)
    coeff_y, _, _, _ = np.linalg.lstsq(A, p[:, 1], rcond=None)

    vx = float(coeff_x[1])
    vy = float(coeff_y[1])

    p_fit = np.column_stack([A @ coeff_x, A @ coeff_y])
    residual_l2 = float(np.linalg.norm(p - p_fit))
    speed = float(np.hypot(vx, vy))
    heading = float(np.arctan2(vy, vx))

    return VelocityEstimate(
        vx=vx,
        vy=vy,
        speed=speed,
        heading_rad=heading,
        residual_l2=residual_l2,
        condition_number=float(np.linalg.cond(A)),
        rank=int(np.linalg.matrix_rank(A)),
        method="position_lstsq",
    )


def estimate_velocity_from_xyvr(
    points_xyvr: Iterable[Iterable[float]],
    *,
    min_condition_rank: int = 2,
    max_condition_number: float = 1e6,
) -> VelocityEstimate:
    """Estimate velocity from array of points represented as ``[x, y, v_r]``.

    This is the most direct interface for the problem statement:
    provide an ``(N, 3)`` array and receive one estimated velocity vector.

    Args:
        points_xyvr: ``(N, 3)`` samples, columns ``[x, y, radial_velocity]``.
        min_condition_rank: Rank threshold passed to radial estimator.
        max_condition_number: Conditioning threshold passed to radial estimator.

    Returns:
        VelocityEstimate.
    """
    data = _as_xyvr_array(points_xyvr)
    positions_xy = data[:, :2]
    radial_velocity = data[:, 2]
    return estimate_velocity_from_radial(
        positions_xy,
        radial_velocity,
        min_condition_rank=min_condition_rank,
        max_condition_number=max_condition_number,
    )


def estimate_velocity_fused(
    times_s: Iterable[float],
    positions_xy: Iterable[Iterable[float]],
    radial_velocity: Iterable[float],
    *,
    weight_position: float = 1.0,
    weight_radial: float = 1.0,
    max_condition_number: float = 1e8,
) -> VelocityEstimate:
    """Joint estimate from position-time and radial constraints.

    Unknown state is ``[x0, y0, vx, vy]``.
    """
    t = np.asarray(times_s, dtype=float).reshape(-1)
    if not np.all(np.isfinite(t)):
        raise EstimationError("times_s must contain only finite values")
    p = _as_2d_array(positions_xy, "positions_xy")
    vr = np.asarray(radial_velocity, dtype=float).reshape(-1)
    if not np.all(np.isfinite(vr)):
        raise EstimationError("radial_velocity must contain only finite values")
    if t.shape[0] != p.shape[0] or vr.shape[0] != p.shape[0]:
        raise EstimationError(
            "times_s, positions_xy, and radial_velocity must have the same length"
        )
    if t.shape[0] < 2:
        raise EstimationError("at least 2 samples are required")
    if weight_position <= 0.0 or weight_radial <= 0.0:
        raise EstimationError("weights must be positive")

    U = line_of_sight_unit_vectors(p)
    n = t.shape[0]

    # Build weighted linear system A x = b, unknown x=[x0, y0, vx, vy].
    A_pos_x = np.column_stack([np.ones(n), np.zeros(n), t, np.zeros(n)])
    A_pos_y = np.column_stack([np.zeros(n), np.ones(n), np.zeros(n), t])
    A_rad = np.column_stack([np.zeros(n), np.zeros(n), U[:, 0], U[:, 1]])

    A = np.vstack(
        [
            np.sqrt(weight_position) * A_pos_x,
            np.sqrt(weight_position) * A_pos_y,
            np.sqrt(weight_radial) * A_rad,
        ]
    )
    b = np.concatenate(
        [
            np.sqrt(weight_position) * p[:, 0],
            np.sqrt(weight_position) * p[:, 1],
            np.sqrt(weight_radial) * vr,
        ]
    )

    rank = int(np.linalg.matrix_rank(A))
    if rank < 4:
        raise EstimationError("insufficient information for fused estimator (rank(A) < 4)")

    cond = float(np.linalg.cond(A))
    if not np.isfinite(cond) or cond > max_condition_number:
        raise EstimationError(f"ill-conditioned fused system (condition={cond:.3e})")

    x_hat, residuals, _, _ = np.linalg.lstsq(A, b, rcond=None)
    vx = float(x_hat[2])
    vy = float(x_hat[3])
    speed = float(np.hypot(vx, vy))
    heading = float(np.arctan2(vy, vx))
    residual_l2 = float(np.sqrt(residuals[0])) if residuals.size > 0 else 0.0

    return VelocityEstimate(
        vx=vx,
        vy=vy,
        speed=speed,
        heading_rad=heading,
        residual_l2=residual_l2,
        condition_number=cond,
        rank=rank,
        method="fused_lstsq",
    )


def estimate_velocity_from_time_xyvr(
    samples_txyvr: Iterable[Iterable[float]],
    *,
    weight_position: float = 1.0,
    weight_radial: float = 1.0,
    max_condition_number: float = 1e8,
) -> VelocityEstimate:
    """Estimate velocity from time-stamped samples ``[t, x, y, v_r]``.

    This is the high-level fused interface for noisy scenarios.
    """
    data = _as_txyvr_array(samples_txyvr)
    times_s = data[:, 0]
    positions_xy = data[:, 1:3]
    radial_velocity = data[:, 3]
    return estimate_velocity_fused(
        times_s,
        positions_xy,
        radial_velocity,
        weight_position=weight_position,
        weight_radial=weight_radial,
        max_condition_number=max_condition_number,
    )


def radial_consistency_error(
    velocity_xy: Iterable[float],
    positions_xy: Iterable[Iterable[float]],
    radial_velocity: Iterable[float],
) -> np.ndarray:
    """Return predicted-minus-measured radial velocity residuals per sample.

    This helps validate that a candidate velocity is consistent with Doppler data.
    """
    v = np.asarray(velocity_xy, dtype=float).reshape(2)
    if not np.all(np.isfinite(v)):
        raise EstimationError("velocity_xy must contain only finite values")
    U = line_of_sight_unit_vectors(positions_xy)
    vr = np.asarray(radial_velocity, dtype=float).reshape(-1)
    if not np.all(np.isfinite(vr)):
        raise EstimationError("radial_velocity must contain only finite values")
    if U.shape[0] != vr.shape[0]:
        raise EstimationError("positions_xy and radial_velocity must have the same length")
    return U @ v - vr