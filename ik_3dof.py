# ik_3dof.py
# 3DOF leg IK (A0 yaw + A1 hip pitch + A2 knee pitch)
# - Inputs are in leg-local coordinates (mm):
#     x: forward, y: outward, z: up  (so standing on ground => z is negative)
# - Outputs are "logical joint angles" (deg): (a0, a1, a2)
# - Z_OFF is treated as a mechanical height compensation:
#     z_eff = z + Z_OFF
#
# NOTE:
# These are logical angles. To command servos, apply per-channel calibration (offset/dir/center/limits)
# using your existing Calibration.apply_one() BEFORE calling afb.quad.servo()/leg().

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class LegGeometry:
    # mm
    A: float  # hip yaw axis -> hip pitch axis lateral/planar offset
    B: float  # thigh length
    C: float  # shank length
    Z_OFF: float = 0.0  # mechanical z compensation (mm)
    DZ_A0_A1: float = 0.0  # A1이 A0보다 위면 +값


class IKError(ValueError):
    pass


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def ik_leg_xyz(
    x: float,
    y: float,
    z: float,
    geo: LegGeometry,
    *,
    elbow: str = "down",
) -> Tuple[float, float, float]:
    """
    Compute inverse kinematics for a 3DOF leg.

    Args:
        x, y, z: target foot position in leg-local coordinates (mm)
        geo: leg geometry
        elbow: "down" (default) or "up" (alternate knee configuration)

    Returns:
        (a0_deg, a1_deg, a2_deg) in degrees (logical joint angles)

    Conventions:
        - a0 is yaw around +z axis: atan2(y, x)
        - then reduce to 2D plane with r1 = hypot(x, y) - A
        - solve 2-link IK in (r1, z_eff)
    """
    # Mechanical compensation
    z_eff = float(z) + float(geo.Z_OFF) - float(geo.DZ_A0_A1)

    # A0 yaw
    a0 = math.atan2(float(y), float(x))  # rad

    # Planar distance from yaw axis to foot projection
    r = math.hypot(float(x), float(y))
    r1 = r - float(geo.A)

    # 2D distance from hip pitch joint to foot
    D = math.hypot(r1, z_eff)

    B = float(geo.B)
    C = float(geo.C)

    # Workspace check (allow tiny epsilon)
    if D > (B + C) + 1e-6 or D < abs(B - C) - 1e-6:
        raise IKError(f"Target out of reach: D={D:.2f}, range=[{abs(B-C):.2f}, {(B+C):.2f}]")

    # Knee angle a2: using cosine law
    # cos(knee_internal) = (B^2 + C^2 - D^2) / (2BC)
    cos_k = (B * B + C * C - D * D) / (2.0 * B * C)
    cos_k = _clamp(cos_k, -1.0, 1.0)
    knee_internal = math.acos(cos_k)  # rad

    # Many robots define a2 as "knee bend" increasing when folding.
    # We'll return a2 = knee_internal for now (0..pi). You can remap if needed.
    a2 = knee_internal

    # Hip pitch a1:
    # alpha is angle to the target vector
    alpha = math.atan2(z_eff, r1)  # rad
    # beta from cosine law
    cos_b = (B * B + D * D - C * C) / (2.0 * B * D)
    cos_b = _clamp(cos_b, -1.0, 1.0)
    beta = math.acos(cos_b)  # rad

    if elbow == "down":
        a1 = alpha + beta
    elif elbow == "up":
        a1 = alpha - beta
        # also flip knee configuration for consistency (optional)
        a2 = -knee_internal
    else:
        raise ValueError("elbow must be 'down' or 'up'")

    # rad -> deg
    return (math.degrees(a0), math.degrees(a1), math.degrees(a2))


def demo_points(geo: LegGeometry) -> None:
    """Small sanity demo prints a few IK solutions."""
    tests = [
        (120, 60, -80),
        (140, 60, -80),
        (120, 60, -60),
        (120, 80, -80),
    ]
    for x, y, z in tests:
        try:
            a0, a1, a2 = ik_leg_xyz(x, y, z, geo)
            print(f"xyz=({x:>6.1f},{y:>6.1f},{z:>6.1f}) -> a0={a0:>7.2f}, a1={a1:>7.2f}, a2={a2:>7.2f}")
        except IKError as e:
            print(f"xyz=({x},{y},{z}) -> IKError: {e}")


if __name__ == "__main__":
    geo = LegGeometry(A=64.8, B=112.0, C=46.5, Z_OFF=10.0)
    demo_points(geo)