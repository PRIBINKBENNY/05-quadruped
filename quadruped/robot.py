"""Ties it together: command -> gait -> body IK -> leg IK -> servo pulses -> hardware backend."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path

import yaml

from .gait import Command, Gait
from .kinematics import LEGS, Geometry, Unreachable, body_ik, leg_fk, neutral_feet
from .servo import ServoMap


@dataclass
class Pose:
    roll: float = 0.0   # rad
    pitch: float = 0.0
    yaw: float = 0.0
    x: float = 0.0      # mm body shift
    y: float = 0.0
    z: float = 0.0


class Robot:
    def __init__(self, config: str | Path | dict = "config/robot.yaml", gait: str = "trot", backend=None):
        cfg = config if isinstance(config, dict) else yaml.safe_load(Path(config).read_text())
        self.cfg = cfg
        self.geom = Geometry(**cfg["geometry"])
        st, gt, sv = cfg["stance"], cfg["gait"], cfg["servos"]
        self.height = st["height"]
        self.neutral = neutral_feet(self.geom, st["height"], st.get("foot_x", 0), st.get("foot_y"))
        self.rate_hz = gt.get("rate_hz", 50)
        self.gait = Gait(self.neutral, gait, gt["period"], gt["step_height"])
        self.servos = ServoMap(
            joints={leg: {j: tuple(v) for j, v in joints.items()} for leg, joints in sv["joints"].items()},
            limits_deg={j: tuple(v) for j, v in sv["limits_deg"].items()},
            pulse_min_us=sv.get("pulse_min_us", 500), pulse_max_us=sv.get("pulse_max_us", 2500))
        self.backend = backend
        self.pose = Pose()
        self.last_angles: dict[str, tuple[float, float, float]] = {}
        self.unreachable = 0

    def set_gait(self, kind: str) -> None:
        self.gait = Gait(self.neutral, kind, self.gait.period, self.gait.step_height)

    def solve(self, feet: dict[str, tuple[float, float, float]]) -> dict[str, tuple[float, float, float]]:
        p = self.pose
        try:
            angles = body_ik(self.geom, feet, p.roll, p.pitch, p.yaw, (p.x, p.y, p.z))
        except Unreachable:
            # keep the last good pose rather than sending garbage to the servos
            self.unreachable += 1
            if not self.last_angles:
                raise
            return self.last_angles
        self.last_angles = angles
        return angles

    def tick(self, cmd: Command, dt: float | None = None) -> dict[int, int]:
        dt = dt if dt is not None else 1.0 / self.rate_hz
        feet = self.gait.step(dt, cmd)
        pulses = self.servos.map_all(self.solve(feet))
        if self.backend is not None:
            self.backend.write(pulses)
        return pulses

    def stand(self) -> dict[int, int]:
        pulses = self.servos.map_all(self.solve(self.neutral))
        if self.backend is not None:
            self.backend.write(pulses)
        return pulses

    def run(self, cmd: Command, seconds: float, realtime: bool = True):
        period = 1.0 / self.rate_hz
        n = int(seconds * self.rate_hz)
        next_t = time.perf_counter()
        for _ in range(n):
            yield self.tick(cmd, period)
            if realtime:
                next_t += period
                time.sleep(max(0.0, next_t - time.perf_counter()))

    def trajectory(self, cmd: Command, seconds: float) -> dict:
        """Joint angles and foot positions over time, for the browser simulator / plotting."""
        frames = []
        dt = 1.0 / self.rate_hz
        for i in range(int(seconds * self.rate_hz)):
            feet = self.gait.step(dt, cmd)
            ang = self.solve(feet)
            frames.append({"t": round((i + 1) * dt, 4),
                           "angles": {l: [round(math.degrees(a), 2) for a in ang[l]] for l in LEGS},
                           "feet": {l: [round(v, 2) for v in feet[l]] for l in LEGS},
                           "stance": {l: self.gait.in_stance(l) for l in LEGS}})
        return {"gait": self.gait.kind, "rate_hz": self.rate_hz, "geometry": self.cfg["geometry"],
                "command": cmd.__dict__, "frames": frames}

    def check_fk(self, angles) -> dict[str, tuple[float, float, float]]:
        return {leg: leg_fk(self.geom, *angles[leg]) for leg in LEGS}
