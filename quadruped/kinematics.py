"""Leg and body kinematics for a 12-DOF quadruped (3 joints per leg).

Leg frame (per leg, at the hip-roll axis): x forward, y OUTWARD (away from the body), z up.
Joints:
  hip   (theta1): roll about x. Positive swings the foot outward.
  femur (theta2): pitch. Angle of the femur from straight down; positive swings the knee forward.
  knee  (theta3): flexion between femur and tibia; 0 = straight leg, positive = folded.
All angles in radians inside this module.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

LEGS = ("FL", "FR", "RL", "RR")


class Unreachable(ValueError):
    pass


@dataclass(frozen=True)
class Geometry:
    body_length: float = 186.0
    body_width: float = 78.0
    coxa: float = 55.0
    femur: float = 107.0
    tibia: float = 130.0
    knee_direction: int = -1

    def hip_position(self, leg: str) -> tuple[float, float, float]:
        """Hip-roll axis position in the body frame (x forward, y left, z up)."""
        x = self.body_length / 2 * (1 if leg[0] == "F" else -1)
        y = self.body_width / 2 * (1 if leg[1] == "L" else -1)
        return (x, y, 0.0)

    @staticmethod
    def side(leg: str) -> int:
        """+1 for left legs, -1 for right legs: converts body y to leg-frame outward y."""
        return 1 if leg[1] == "L" else -1


def leg_ik(g: Geometry, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Foot position in the leg frame -> (hip, femur, knee) angles."""
    d2 = y * y + z * z
    if d2 < g.coxa ** 2:
        raise Unreachable(f"foot too close to the hip axis (|yz|={math.sqrt(d2):.1f} < coxa {g.coxa})")
    h = math.sqrt(d2 - g.coxa ** 2)  # vertical reach in the leg plane
    theta1 = math.atan2(y, -z) - math.atan2(g.coxa, h)

    r2 = x * x + h * h
    L2, L3 = g.femur, g.tibia
    c3 = (r2 - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    if c3 > 1 + 1e-9 or c3 < -1 - 1e-9:
        raise Unreachable(f"foot out of reach: distance {math.sqrt(r2):.1f} not in "
                          f"[{abs(L2 - L3):.1f}, {L2 + L3:.1f}]")
    c3 = max(-1.0, min(1.0, c3))
    theta3 = math.acos(c3)  # flexion from straight (the interior knee angle is pi - theta3)
    s = g.knee_direction
    alpha = math.atan2(L3 * math.sin(theta3), L2 + L3 * math.cos(theta3))
    theta2 = math.atan2(x, h) - s * alpha
    return theta1, theta2, theta3


def leg_fk(g: Geometry, theta1: float, theta2: float, theta3: float) -> tuple[float, float, float]:
    """(hip, femur, knee) angles -> foot position in the leg frame."""
    s = g.knee_direction
    x = g.femur * math.sin(theta2) + g.tibia * math.sin(theta2 + s * theta3)
    h = g.femur * math.cos(theta2) + g.tibia * math.cos(theta2 + s * theta3)
    # rotate the leg plane (y'=coxa, z'=-h) about x by theta1
    y = g.coxa * math.cos(theta1) + h * math.sin(theta1)
    z = g.coxa * math.sin(theta1) - h * math.cos(theta1)
    return x, y, z


def knee_position(g: Geometry, theta1: float, theta2: float) -> tuple[float, float, float]:
    x = g.femur * math.sin(theta2)
    h = g.femur * math.cos(theta2)
    return x, g.coxa * math.cos(theta1) + h * math.sin(theta1), g.coxa * math.sin(theta1) - h * math.cos(theta1)


def _rot(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def body_ik(g: Geometry, feet_world: dict[str, tuple[float, float, float]],
            roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
            shift: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> dict[str, tuple[float, float, float]]:
    """Feet positions in the world frame (origin under the body centre at hip height) + body pose
    -> joint angles for every leg."""
    R = _rot(roll, pitch, yaw)
    out = {}
    for leg in LEGS:
        fx, fy, fz = feet_world[leg]
        # world -> body: p_body = R^T (p_world - shift)
        px, py, pz = fx - shift[0], fy - shift[1], fz - shift[2]
        bx = R[0][0] * px + R[1][0] * py + R[2][0] * pz
        by = R[0][1] * px + R[1][1] * py + R[2][1] * pz
        bz = R[0][2] * px + R[1][2] * py + R[2][2] * pz
        hx, hy, hz = g.hip_position(leg)
        out[leg] = leg_ik(g, bx - hx, (by - hy) * g.side(leg), bz - hz)
    return out


def neutral_feet(g: Geometry, height: float, foot_x: float = 0.0, foot_y: float | None = None
                 ) -> dict[str, tuple[float, float, float]]:
    """Standing foot positions in the world frame."""
    fy = g.coxa if foot_y is None else foot_y
    feet = {}
    for leg in LEGS:
        hx, hy, _ = g.hip_position(leg)
        feet[leg] = (hx + foot_x, hy + fy * g.side(leg), -height)
    return feet
