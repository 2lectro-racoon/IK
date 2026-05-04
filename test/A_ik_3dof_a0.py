# ik_3dof_a0.py
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class LegGeometry:
    # Lengths in mm
    A: float                # A0(yaw) -> A1(pitch) planar offset (coxa)
    B: float                # A1 -> knee (thigh)
    C: float                # knee -> foot (shank)
    Z_OFF: float = 0.0      # mechanical foot height offset (+ means foot considered higher)
    DZ_A0_A1: float = 0.0   # A1 is higher than A0 by this many mm (+Z)


class IKError(ValueError):
    pass


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def ik_leg_a0_xyz(
    x: float,
    y: float,
    z: float,
    geo: LegGeometry,
    *,
    elbow: str = "down",
) -> Tuple[float, float, float]:
    """A0-based leg IK.

    Inputs (mm) are relative to A0(yaw) axis:
      x: forward, y: outward, z: up (standing => z negative)

    Geometry:
      A: planar offset from A0 to A1 (coxa)
      DZ_A0_A1: A1 is higher than A0 by +mm

    Output angles (deg):
      a0: yaw
      a1: hip pitch
      a2: knee bend angle (0 = straight, increasing = folding)
    """
    x = float(x)
    y = float(y)
    z = float(z)
    A = float(geo.A)
    B = float(geo.B)
    C = float(geo.C)

    # 1) yaw
    a0 = math.atan2(y, x)  # rad

    # 2) reduce to 2D at A1 (pitch) joint
    r = math.hypot(x, y)  # distance from A0 to foot projection
    r1 = r - A            # distance from A1 to foot projection (planar)
    z1 = (z + geo.Z_OFF) - geo.DZ_A0_A1  # z relative to A1

    # 3) 2-link IK in (r1, z1)
    D = math.hypot(r1, z1)
    if D > (B + C) + 1e-6 or D < abs(B - C) - 1e-6:
        raise IKError(
            f"Target out of reach: D={D:.2f}, range=[{abs(B-C):.2f}, {(B+C):.2f}]"
        )

    cos_k = (B * B + C * C - D * D) / (2.0 * B * C)
    cos_k = _clamp(cos_k, -1.0, 1.0)
    knee_internal = math.acos(cos_k)  # 0..pi
    a2_bend = math.pi - knee_internal

    alpha = math.atan2(z1, r1)
    cos_b = (B * B + D * D - C * C) / (2.0 * B * D)
    cos_b = _clamp(cos_b, -1.0, 1.0)
    beta = math.acos(cos_b)

    if elbow == "down":
        a1 = alpha + beta
    elif elbow == "up":
        a1 = alpha - beta
        a2_bend = -a2_bend
    else:
        raise ValueError("elbow must be 'down' or 'up'")

    return (math.degrees(a0), math.degrees(a1), math.degrees(a2_bend))


def fk_leg_a0(
    a0_deg: float,
    a1_deg: float,
    a2_bend_deg: float,
    geo: LegGeometry,
) -> Tuple[float, float, float]:
    """Forward kinematics validation. Returns (x,y,z) in A0 frame."""
    A = float(geo.A)
    B = float(geo.B)
    C = float(geo.C)

    a0 = math.radians(a0_deg)
    a1 = math.radians(a1_deg)
    a2b = math.radians(a2_bend_deg)

    knee_internal = math.pi - a2b

    xB = B * math.cos(a1)
    zB = B * math.sin(a1)

    shank_angle = a1 - (math.pi - knee_internal)  # = a1 - a2bend
    xC = C * math.cos(shank_angle)
    zC = C * math.sin(shank_angle)

    r1 = xB + xC
    z1 = zB + zC

    r = r1 + A
    x = r * math.cos(a0)
    y = r * math.sin(a0)

    z = z1 - geo.Z_OFF + geo.DZ_A0_A1
    return (x, y, z)