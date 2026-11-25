# 05 · Quadruped gait + inverse kinematics

Walking controller for a **12-servo robot dog** (3 joints per leg, MG996R servos on a PCA9685): leg and body **inverse kinematics**, a **phase-based gait generator** (trot, walk, pace) with turning and strafing, **servo calibration**, hardware backends, and a **browser simulator** running the same maths.

**▶ Live simulator:** https://pribinkbenny.github.io/05-quadruped/sim/

```
command (vx, vy, yaw) → gait (foot targets) → body IK (roll/pitch/yaw/shift) → leg IK → servo map → PCA9685
```

## Quick start

```bash
pip install -e ".[dev]"
quadruped stand                                   # dry run: prints the 12 pulse widths
quadruped walk --gait trot --vx 120 --seconds 5   # dry run at 50 Hz
quadruped export --gait walk --vx 80              # sim/trajectory.json for plotting
pytest                                            # 16 tests, incl. Python ↔ JS parity
```

On the robot:

```bash
# Raspberry Pi → PCA9685 over I2C
pip install adafruit-circuitpython-servokit
quadruped calibrate --backend pca9685      # set each joint's zero, copy the offsets into config/robot.yaml
quadruped walk --backend pca9685 --gait trot --vx 100

# or PC/Pi → USB → ESP32/Arduino running firmware/servo_bridge (drives the PCA9685)
quadruped walk --backend serial --port /dev/ttyUSB0 --gait walk --vx 60
```

**Before powering the servos:** measure your leg lengths into `config/robot.yaml`, run `calibrate` with the legs off the ground, and start with `walk` at low speed. 12 MG996Rs can draw well over 10 A at stall, so use a separate 5–6 V supply (a high-current BEC) and join the grounds.

## The maths

**Leg IK** (`quadruped/kinematics.py`), per leg in the hip frame (x forward, y outward, z up):
1. **Hip roll:** `θ1 = atan2(y, −z) − atan2(coxa, h)` with `h = √(y² + z² − coxa²)`
2. **Knee:** law of cosines in the leg plane, `θ3 = acos((r² − L2² − L3²) / 2·L2·L3)`
3. **Femur:** `θ2 = atan2(x, h) − s·atan2(L3·sinθ3, L2 + L3·cosθ3)`, where `s` selects knees-back (dog-like) or knees-forward

Unreachable targets raise `Unreachable`. The controller then holds the last good pose instead of sending garbage to the servos.

**Body IK** maps world foot targets through the inverse body pose (`Rz·Ry·Rx` plus shift), so the body can lean, twist and shift while the feet stay planted.

**Gait** (`quadruped/gait.py`): every leg runs one cycle shifted by a phase offset.

| gait | phase offsets (FL, FR, RL, RR) | duty | feet on the ground |
|---|---|---|---|
| trot | 0, ½, ½, 0 | 0.55 | ≥ 2 (diagonal pairs) |
| walk | 0, ½, ¾, ¼ | 0.78 | ≥ 3 (statically stable) |
| pace | 0, ½, 0, ½ | 0.55 | ≥ 2 (lateral pairs) |

In stance the foot moves backwards at exactly the commanded ground speed, so there's no foot slip. In swing it returns along a cosine profile and lifts on a sine arc. Turning uses rigid-body velocity at each foot: `v = (vx − ω·y, vy + ω·x)`.

**Servos** (`quadruped/servo.py`): joint angle → horn angle using per-joint channel, direction and offset, then **clamped to safe joint limits** (clipping is reported) → 500–2500 µs pulse. Right-side servos are mirrored, and duplicate channels are rejected.

## Tests

| test | what it proves |
|---|---|
| IK∘FK round trip, 3,000 random targets | IK is exact (1e-6 mm) wherever reachable |
| known poses, unreachable targets | sign conventions; failure is explicit |
| body IK | left/right symmetry; roll extends the correct legs; body pose never moves the feet |
| gait support | trot ≥ 2, walk ≥ 3 feet down at every instant |
| foot paths | no jumps at phase boundaries; stance speed equals command; correct step height |
| servo map | mirrored pulses, limits clamp, duplicate channels rejected |
| 50 Hz run, 2 s trot + turn | 100 frames, no unreachable poses, no clipping |
| **JS ↔ Python parity** | the simulator runs identical maths (1e-9) |

## Layout

```
config/robot.yaml                 geometry, stance, gait, servo channels/offsets/limits
quadruped/kinematics.py           leg IK/FK, body IK
quadruped/gait.py                 trot / walk / pace / stand generator
quadruped/servo.py                calibration + limits + pulse mapping
quadruped/robot.py                control loop, trajectory export
quadruped/backends.py             PCA9685 (Pi), serial (ESP32/Arduino), dry run
firmware/servo_bridge/            ESP32/Arduino serial → PCA9685 bridge with failsafe
sim/                              three.js simulator (kinematics.js = JS port)
```

## What I learned

inverse kinematics · gait phase design · static vs dynamic stability · servo calibration · hardware safety · real-time control loops
