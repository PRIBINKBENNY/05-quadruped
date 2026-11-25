// JavaScript port of quadruped/kinematics.py + gait.py for the browser simulator.
// tests/test_quadruped.py runs this file under Node and checks it matches the Python exactly.
(function (root) {
  const LEGS = ["FL", "FR", "RL", "RR"];
  const GAITS = {
    trot: [{ FL: 0, RR: 0, FR: 0.5, RL: 0.5 }, 0.55],
    walk: [{ FL: 0, RR: 0.25, FR: 0.5, RL: 0.75 }, 0.78],
    pace: [{ FL: 0, RL: 0, FR: 0.5, RR: 0.5 }, 0.55],
    stand: [{ FL: 0, RR: 0, FR: 0, RL: 0 }, 1.0],
  };

  const side = (leg) => (leg[1] === "L" ? 1 : -1);
  function hip(g, leg) {
    return [g.body_length / 2 * (leg[0] === "F" ? 1 : -1), g.body_width / 2 * side(leg), 0];
  }

  function legIK(g, x, y, z) {
    const d2 = y * y + z * z;
    if (d2 < g.coxa * g.coxa) throw new Error("unreachable: too close to hip");
    const h = Math.sqrt(d2 - g.coxa * g.coxa);
    const t1 = Math.atan2(y, -z) - Math.atan2(g.coxa, h);
    const r2 = x * x + h * h, L2 = g.femur, L3 = g.tibia;
    let c3 = (r2 - L2 * L2 - L3 * L3) / (2 * L2 * L3);
    if (c3 > 1 + 1e-9 || c3 < -1 - 1e-9) throw new Error("unreachable: out of reach");
    c3 = Math.max(-1, Math.min(1, c3));
    const t3 = Math.acos(c3);
    const s = g.knee_direction;
    const alpha = Math.atan2(L3 * Math.sin(t3), L2 + L3 * Math.cos(t3));
    const t2 = Math.atan2(x, h) - s * alpha;
    return [t1, t2, t3];
  }

  function legFK(g, t1, t2, t3) {
    const s = g.knee_direction;
    const x = g.femur * Math.sin(t2) + g.tibia * Math.sin(t2 + s * t3);
    const h = g.femur * Math.cos(t2) + g.tibia * Math.cos(t2 + s * t3);
    return [x, g.coxa * Math.cos(t1) + h * Math.sin(t1), g.coxa * Math.sin(t1) - h * Math.cos(t1)];
  }

  function kneePos(g, t1, t2) {
    const x = g.femur * Math.sin(t2), h = g.femur * Math.cos(t2);
    return [x, g.coxa * Math.cos(t1) + h * Math.sin(t1), g.coxa * Math.sin(t1) - h * Math.cos(t1)];
  }

  function rot(roll, pitch, yaw) {
    const cr = Math.cos(roll), sr = Math.sin(roll), cp = Math.cos(pitch), sp = Math.sin(pitch),
      cy = Math.cos(yaw), sy = Math.sin(yaw);
    return [
      [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
      [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
      [-sp, cp * sr, cp * cr],
    ];
  }

  function bodyIK(g, feet, pose) {
    const p = Object.assign({ roll: 0, pitch: 0, yaw: 0, x: 0, y: 0, z: 0 }, pose || {});
    const R = rot(p.roll, p.pitch, p.yaw);
    const out = {};
    for (const leg of LEGS) {
      const [fx, fy, fz] = feet[leg];
      const px = fx - p.x, py = fy - p.y, pz = fz - p.z;
      const bx = R[0][0] * px + R[1][0] * py + R[2][0] * pz;
      const by = R[0][1] * px + R[1][1] * py + R[2][1] * pz;
      const bz = R[0][2] * px + R[1][2] * py + R[2][2] * pz;
      const [hx, hy, hz] = hip(g, leg);
      out[leg] = legIK(g, bx - hx, (by - hy) * side(leg), bz - hz);
    }
    return out;
  }

  function neutralFeet(g, height, footX, footY) {
    const fy = footY == null ? g.coxa : footY;
    const out = {};
    for (const leg of LEGS) {
      const [hx, hy] = hip(g, leg);
      out[leg] = [hx + (footX || 0), hy + fy * side(leg), -height];
    }
    return out;
  }

  function gaitFeet(neutral, kind, period, stepHeight, cmd, t) {
    const [offsets, duty] = GAITS[kind];
    const out = {}, stance = {};
    for (const leg of LEGS) {
      const [nx, ny, nz] = neutral[leg];
      if (duty >= 1) { out[leg] = [nx, ny, nz]; stance[leg] = true; continue; }
      const vx = cmd.vx - cmd.yaw_rate * ny, vy = cmd.vy + cmd.yaw_rate * nx;
      const sx = vx * period * duty, sy = vy * period * duty;
      const ph = ((t / period + offsets[leg]) % 1 + 1) % 1;
      if (ph < duty) {
        const k = 0.5 - ph / duty;
        out[leg] = [nx + sx * k, ny + sy * k, nz];
        stance[leg] = true;
      } else {
        const u = (ph - duty) / (1 - duty);
        const k = -0.5 + (1 - Math.cos(Math.PI * u)) / 2;
        const lift = (sx || sy) ? stepHeight * Math.sin(Math.PI * u) : 0;
        out[leg] = [nx + sx * k, ny + sy * k, nz + lift];
        stance[leg] = false;
      }
    }
    return { feet: out, stance };
  }

  const api = { LEGS, GAITS, side, hip, legIK, legFK, kneePos, rot, bodyIK, neutralFeet, gaitFeet };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.Kin = api;
})(this);
