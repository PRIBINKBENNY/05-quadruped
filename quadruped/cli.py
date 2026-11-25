"""quadruped stand                         stand pose (dry run prints pulses)
quadruped walk --gait trot --vx 120 --seconds 5 --backend serial --port /dev/ttyUSB0
quadruped calibrate --backend pca9685    move one joint at a time to find offsets
quadruped export --gait walk --vx 80     write sim/trajectory.json for plotting
"""
from __future__ import annotations

import argparse
import json
import math

from .backends import DryRunBackend
from .gait import GAITS, Command
from .robot import Robot


def make_backend(args):
    if args.backend == "pca9685":
        from .backends import PCA9685Backend

        return PCA9685Backend()
    if args.backend == "serial":
        from .backends import SerialBackend

        return SerialBackend(args.port)
    return DryRunBackend(echo=True)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="quadruped")
    p.add_argument("--config", default="config/robot.yaml")
    sub = p.add_subparsers(dest="cmd", required=True)
    for name in ("stand", "walk", "calibrate", "export"):
        s = sub.add_parser(name)
        s.add_argument("--backend", choices=["dryrun", "pca9685", "serial"], default="dryrun")
        s.add_argument("--port", default="/dev/ttyUSB0")
        if name in ("walk", "export"):
            s.add_argument("--gait", choices=sorted(GAITS), default="trot")
            s.add_argument("--vx", type=float, default=100.0, help="mm/s forward")
            s.add_argument("--vy", type=float, default=0.0, help="mm/s left")
            s.add_argument("--yaw", type=float, default=0.0, help="deg/s turn (CCW +)")
            s.add_argument("--seconds", type=float, default=4.0)
        if name == "export":
            s.add_argument("--out", default="sim/trajectory.json")
    a = p.parse_args(argv)

    if a.cmd == "export":
        r = Robot(a.config, a.gait)
        traj = r.trajectory(Command(a.vx, a.vy, math.radians(a.yaw)), a.seconds)
        with open(a.out, "w") as f:
            json.dump(traj, f)
        print(f"wrote {len(traj['frames'])} frames to {a.out}")
        return 0

    backend = make_backend(a)
    r = Robot(a.config, getattr(a, "gait", "trot"), backend)
    if a.cmd == "stand":
        r.stand()
    elif a.cmd == "walk":
        r.stand()
        for _ in r.run(Command(a.vx, a.vy, math.radians(a.yaw)), a.seconds):
            pass
        r.stand()
        if r.unreachable:
            print(f"warning: {r.unreachable} frames were out of reach (held last pose). Lower speed or height.")
    elif a.cmd == "calibrate":
        print("Calibration: each joint moves to its 0 deg pose. Adjust the offset until the leg is straight down\n"
              "(hip level, femur vertical, knee straight), then copy the offsets into config/robot.yaml.")
        joints = r.servos.joints
        for leg in joints:
            for joint, (ch, d, off) in joints[leg].items():
                while True:
                    zero = {l: (0.0, 0.0, 0.0) for l in joints}
                    r.servos.joints[leg][joint] = (ch, d, off)
                    pulses = r.servos.map_all(zero)
                    backend.write({ch: pulses[ch]})
                    k = input(f"{leg}.{joint} (ch {ch}) offset={off:+.1f}  [+/- 1deg, ]/[ 5deg, enter=next]: ")
                    if k == "":
                        break
                    off += {"+": 1, "-": -1, "]": 5, "[": -5}.get(k, 0)
                print(f"  {leg}.{joint}: [{ch}, {d}, {off}]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
