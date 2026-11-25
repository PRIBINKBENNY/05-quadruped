import json
import math
import random
import shutil
import subprocess
from pathlib import Path

import pytest

from quadruped import Command, Gait, Geometry, Robot, Unreachable, body_ik, leg_fk, leg_ik
from quadruped.backends import DryRunBackend, encode_serial
from quadruped.kinematics import LEGS, neutral_feet

ROOT = Path(__file__).resolve().parents[1]
G = Geometry()


def close(a, b, tol=1e-6):
    return all(abs(x - y) < tol for x, y in zip(a, b))


# ------------------------------------------------------------------ kinematics
def test_ik_fk_round_trip_over_workspace():
    rng = random.Random(0)
    n = 0
    for _ in range(3000):
        p = (rng.uniform(-120, 120), rng.uniform(-40, 140), rng.uniform(-240, -60))
        try:
            ang = leg_ik(G, *p)
        except Unreachable:
            continue
        assert close(leg_fk(G, *ang), p, 1e-6), p
        n += 1
    assert n > 1500  # most of the sampled box is reachable


def test_known_poses():
    # straight leg, hanging down: all joints zero
    assert close(leg_ik(G, 0, G.coxa, -(G.femur + G.tibia)), (0, 0, 0), 1e-6)
    # standing pose: knee folds backwards (dog-like) => femur leans forward
    t1, t2, t3 = leg_ik(G, 0, G.coxa, -160)
    assert abs(t1) < 1e-9 and t2 > 0 and t3 > 0
    # foot moved outwards => hip rolls outwards (positive)
    assert leg_ik(G, 0, G.coxa + 30, -160)[0] > 0


def test_unreachable_is_reported_not_garbage():
    with pytest.raises(Unreachable):
        leg_ik(G, 0, G.coxa, -(G.femur + G.tibia) - 5)  # too far
    with pytest.raises(Unreachable):
        leg_ik(G, 0, 10, -10)  # inside the coxa radius


def test_body_ik_left_right_symmetry_and_pose():
    feet = neutral_feet(G, 160)
    a = body_ik(G, feet)
    assert close(a["FL"], a["FR"]) and close(a["RL"], a["RR"]) and close(a["FL"], a["RL"])
    # rolling the body right (positive roll lifts the left side): left legs must extend, right legs fold
    r = body_ik(G, feet, roll=math.radians(8))
    assert r["FL"][2] < a["FL"][2] and r["FR"][2] > a["FR"][2]
    # body pose must not move the feet: FK in the body frame, mapped back to world, returns the targets
    from quadruped.kinematics import _rot

    R = _rot(0.1, -0.05, 0.08)
    angles = body_ik(G, feet, 0.1, -0.05, 0.08, (5, -3, 4))
    for leg in LEGS:
        lx, ly, lz = leg_fk(G, *angles[leg])
        hx, hy, hz = G.hip_position(leg)
        b = (hx + lx, hy + ly * G.side(leg), hz + lz)
        w = [sum(R[i][j] * b[j] for j in range(3)) + s for i, s in enumerate((5, -3, 4))]
        assert close(w, feet[leg], 1e-6)


# ------------------------------------------------------------------ gait
def feet_on_ground(gait, t):
    return sum(gait.in_stance(l, t) for l in LEGS)


@pytest.mark.parametrize("kind,min_down", [("trot", 2), ("walk", 3), ("pace", 2)])
def test_support_legs_at_all_times(kind, min_down):
    g = Gait(neutral_feet(G, 160), kind, period=0.5)
    for i in range(1000):
        assert feet_on_ground(g, i * 0.0005) >= min_down


def test_trot_moves_diagonal_pairs_together():
    g = Gait(neutral_feet(G, 160), "trot", period=0.5)
    for i in range(200):
        t = i * 0.0025
        assert g.in_stance("FL", t) == g.in_stance("RR", t)
        assert g.in_stance("FR", t) == g.in_stance("RL", t)


def test_foot_paths_are_continuous_and_planted_in_stance():
    neutral = neutral_feet(G, 160)
    g = Gait(neutral, "trot", period=0.5, step_height=35)
    cmd = Command(vx=150)
    dt = 0.001
    prev = g.feet(cmd, 0.0)
    for i in range(1, 2000):
        t = i * dt
        cur = g.feet(cmd, t)
        for leg in LEGS:
            jump = math.dist(prev[leg], cur[leg])
            assert jump < 2.0, (leg, t, jump)  # no teleporting at phase boundaries
            if g.in_stance(leg, t) and g.in_stance(leg, t - dt):
                assert cur[leg][2] == neutral[leg][2]
                # ground speed in stance matches the command (foot moves backwards relative to body)
                assert abs((cur[leg][0] - prev[leg][0]) / dt + cmd.vx) < 1e-6
        prev = cur
    assert max(g.feet(cmd, t)["FL"][2] for t in [i * 0.005 for i in range(100)]) == pytest.approx(-160 + 35, abs=0.5)


def test_turning_moves_feet_tangentially():
    neutral = neutral_feet(G, 160)
    g = Gait(neutral, "trot", period=0.5)
    w = math.radians(30)
    s = g._stride("FL", Command(yaw_rate=w))
    px, py, _ = neutral["FL"]
    assert s[0] * px + s[1] * py == pytest.approx(0, abs=1e-9)  # perpendicular to the radius
    assert s[1] > 0 and s[0] < 0  # CCW turn: front-left foot sweeps left/back... relative to body


# ------------------------------------------------------------------ servos + robot
def test_servo_mapping_mirrored_and_clamped():
    r = Robot(ROOT / "config/robot.yaml")
    pulses = r.stand()
    assert len(pulses) == 12 and len(set(pulses)) == 12
    assert all(500 <= us <= 2500 for us in pulses.values())
    # right side servos are mounted mirrored: pulses mirror around 1500 for hip/femur
    assert pulses[1] + pulses[5] == 3000 and pulses[0] == pulses[4] == 1500
    r.servos.map_all({l: (math.radians(80), 0.0, 0.0) for l in LEGS})
    assert len(r.servos.clipped) == 4  # hip limit is 35 deg


def test_duplicate_channels_rejected():
    import yaml

    cfg = yaml.safe_load((ROOT / "config/robot.yaml").read_text())
    cfg["servos"]["joints"]["FR"]["hip"] = [0, -1, 0]
    with pytest.raises(ValueError, match="duplicate"):
        Robot(cfg)


def test_walk_produces_50hz_frames_without_unreachable_poses():
    be = DryRunBackend()
    r = Robot(ROOT / "config/robot.yaml", "trot", be)
    frames = list(r.run(Command(vx=150, yaw_rate=math.radians(15)), seconds=2, realtime=False))
    assert len(frames) == 100 == len(be.frames)
    assert r.unreachable == 0 and not r.servos.clipped
    assert len({tuple(sorted(f.items())) for f in frames}) > 20  # it actually moves


def test_serial_protocol():
    assert encode_serial({2: 1600, 0: 1500}) == b"S 0:1500 2:1600\n"


def test_export_trajectory(tmp_path):
    from quadruped.cli import main

    out = tmp_path / "traj.json"
    assert main(["--config", str(ROOT / "config/robot.yaml"), "export", "--gait", "walk", "--vx", "80",
                 "--seconds", "1", "--out", str(out)]) == 0
    traj = json.loads(out.read_text())
    assert traj["gait"] == "walk" and len(traj["frames"]) == 50
    assert all(sum(f["stance"].values()) >= 3 for f in traj["frames"])


# ------------------------------------------------------------------ JS simulator parity
@pytest.mark.skipif(shutil.which("node") is None, reason="node not installed")
def test_js_simulator_matches_python():
    pts = [(0, 55, -160), (40, 70, -150), (-50, 30, -120), (25, 90, -190)]
    neutral = neutral_feet(G, 150)
    script = f"""
const K = require({json.dumps(str(ROOT / 'sim/kinematics.js'))});
const g = {json.dumps(G.__dict__)};
const pts = {json.dumps(pts)};
const out = {{ik: pts.map(p => K.legIK(g, ...p)), fk: pts.map(p => K.legFK(g, ...K.legIK(g, ...p)))}};
const neutral = K.neutralFeet(g, 150);
out.gait = [0.03, 0.21, 0.37, 0.49].map(t => K.gaitFeet(neutral, "trot", 0.5, 35, {{vx: 120, vy: 20, yaw_rate: 0.3}}, t).feet);
out.body = K.bodyIK(g, neutral, {{roll: 0.1, pitch: -0.05, yaw: 0.02, x: 3, y: -2, z: 5}});
console.log(JSON.stringify(out));
"""
    js = json.loads(subprocess.run(["node", "-e", script], capture_output=True, text=True, check=True).stdout)
    for p, a in zip(pts, js["ik"]):
        assert close(leg_ik(G, *p), a, 1e-9)
    gait = Gait(neutral, "trot", 0.5, 35)
    for t, feet in zip([0.03, 0.21, 0.37, 0.49], js["gait"]):
        py = gait.feet(Command(120, 20, 0.3), t)
        for leg in LEGS:
            assert close(py[leg], feet[leg], 1e-9)
    body = body_ik(G, neutral, 0.1, -0.05, 0.02, (3, -2, 5))
    for leg in LEGS:
        assert close(body[leg], js["body"][leg], 1e-9)
