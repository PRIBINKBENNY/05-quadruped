"""Phase-based gait generator.

Each leg runs the same cycle, shifted by a phase offset:
  stance (phase < duty): the foot is on the ground and moves backwards relative to the body,
                         which pushes the body forward.
  swing  (phase >= duty): the foot lifts (sine arc) and moves forwards (smooth cosine profile).

Commands: vx, vy (mm/s, body frame) and yaw_rate (rad/s). Every foot's stance path is the rigid
body motion seen from the robot, so turning and strafing come for free.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .kinematics import LEGS

GAITS = {
    #        phase offsets                                   duty factor (fraction of cycle on the ground)
    "trot":  ({"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}, 0.55),
    "walk":  ({"FL": 0.0, "RR": 0.25, "FR": 0.5, "RL": 0.75}, 0.78),   # one leg in the air at a time
    "pace":  ({"FL": 0.0, "RL": 0.0, "FR": 0.5, "RR": 0.5}, 0.55),
    "stand": ({"FL": 0.0, "RR": 0.0, "FR": 0.0, "RL": 0.0}, 1.0),
}


@dataclass
class Command:
    vx: float = 0.0        # mm/s forward
    vy: float = 0.0        # mm/s left
    yaw_rate: float = 0.0  # rad/s counter-clockwise


class Gait:
    def __init__(self, neutral: dict[str, tuple[float, float, float]], kind: str = "trot",
                 period: float = 0.5, step_height: float = 35.0):
        if kind not in GAITS:
            raise ValueError(f"unknown gait {kind!r}; choose from {sorted(GAITS)}")
        self.neutral = neutral
        self.kind = kind
        self.offsets, self.duty = GAITS[kind]
        self.period = period
        self.step_height = step_height
        self.t = 0.0

    def phase(self, leg: str, t: float | None = None) -> float:
        t = self.t if t is None else t
        return (t / self.period + self.offsets[leg]) % 1.0

    def in_stance(self, leg: str, t: float | None = None) -> bool:
        return self.duty >= 1.0 or self.phase(leg, t) < self.duty

    def _stride(self, leg: str, cmd: Command) -> tuple[float, float]:
        """Distance the foot travels (relative to the body) during one stance phase."""
        px, py, _ = self.neutral[leg]
        vx = cmd.vx - cmd.yaw_rate * py  # rigid-body velocity of the ground point under the foot
        vy = cmd.vy + cmd.yaw_rate * px
        T_stance = self.period * self.duty
        return vx * T_stance, vy * T_stance

    def feet(self, cmd: Command, t: float | None = None) -> dict[str, tuple[float, float, float]]:
        t = self.t if t is None else t
        out = {}
        for leg in LEGS:
            nx, ny, nz = self.neutral[leg]
            if self.duty >= 1.0:
                out[leg] = (nx, ny, nz)
                continue
            sx, sy = self._stride(leg, cmd)
            ph = self.phase(leg, t)
            if ph < self.duty:  # stance: +stride/2 -> -stride/2, linear (constant ground speed)
                u = ph / self.duty
                k = 0.5 - u
                out[leg] = (nx + sx * k, ny + sy * k, nz)
            else:               # swing: -stride/2 -> +stride/2 with a smooth profile, lifted
                u = (ph - self.duty) / (1 - self.duty)
                k = -0.5 + (1 - math.cos(math.pi * u)) / 2
                lift = self.step_height * math.sin(math.pi * u) if (sx or sy) else 0.0
                out[leg] = (nx + sx * k, ny + sy * k, nz + lift)
        return out

    def step(self, dt: float, cmd: Command) -> dict[str, tuple[float, float, float]]:
        self.t += dt
        return self.feet(cmd)
