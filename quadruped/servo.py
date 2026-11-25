"""Joint angles -> MG996R servo pulses on a PCA9685, with per-joint calibration and hard limits."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

JOINTS = ("hip", "femur", "knee")


@dataclass
class ServoMap:
    """channel, direction (+1/-1) and offset (deg) for each of the 12 joints."""

    joints: dict[str, dict[str, tuple[int, int, float]]]
    limits_deg: dict[str, tuple[float, float]]
    pulse_min_us: int = 500
    pulse_max_us: int = 2500
    clipped: list[str] = field(default_factory=list)

    def __post_init__(self):
        chans = [c for leg in self.joints.values() for (c, _, _) in leg.values()]
        if len(chans) != len(set(chans)):
            raise ValueError(f"duplicate PCA9685 channels in servo map: {sorted(chans)}")

    def servo_deg(self, leg: str, joint: str, angle_rad: float) -> float:
        """Joint angle -> servo horn angle in [0, 180], clamped to the joint's safe range."""
        lo, hi = self.limits_deg[joint]
        a = math.degrees(angle_rad)
        if a < lo or a > hi:
            self.clipped.append(f"{leg}.{joint}={a:.1f}")
            a = min(max(a, lo), hi)
        ch, direction, offset = self.joints[leg][joint]
        # hip/femur swing both ways around the servo centre. The knee only folds one way (0..140 deg),
        # so its zero sits near one end of the servo's travel, mirrored for reversed servos.
        centre = 90.0 if joint != "knee" else (30.0 if direction > 0 else 150.0)
        return min(max(centre + direction * a + offset, 0.0), 180.0)

    def pulse_us(self, servo_angle_deg: float) -> int:
        return round(self.pulse_min_us + (self.pulse_max_us - self.pulse_min_us) * servo_angle_deg / 180.0)

    def map_all(self, angles: dict[str, tuple[float, float, float]]) -> dict[int, int]:
        """{leg: (hip, femur, knee) rad} -> {channel: pulse_us}"""
        self.clipped = []
        out = {}
        for leg, triple in angles.items():
            for joint, a in zip(JOINTS, triple):
                ch = self.joints[leg][joint][0]
                out[ch] = self.pulse_us(self.servo_deg(leg, joint, a))
        return out
