import * as THREE from "three";

const deg2rad = (d) => (d * Math.PI) / 180;

// UR10e nominal dimensions (meters). Adjust if your URDF differs.
const d1 = 0.1273; // base to shoulder
const a2 = -0.612; // shoulder to elbow
const a3 = -0.5723; // elbow to wrist1
const d4 = 0.163941;
const d5 = 0.1157;
const d6 = 0.0922; // wrist to flange

function dh(alpha, a, d, theta) {
  const ca = Math.cos(alpha);
  const sa = Math.sin(alpha);
  const ct = Math.cos(theta);
  const st = Math.sin(theta);
  const m = new THREE.Matrix4();
  m.set(
    ct,
    -st * ca,
    st * sa,
    a * ct,
    st,
    ct * ca,
    -ct * sa,
    a * st,
    0,
    sa,
    ca,
    d,
    0,
    0,
    0,
    1
  );
  return m;
}

export function fkUR10e(joints) {
  if (!joints || joints.length !== 6) throw new Error("fkUR10e expects 6 joints");
  const [q1, q2, q3, q4, q5, q6] = joints;
  const T1 = dh(Math.PI / 2, 0, d1, q1);
  const T2 = dh(0, a2, 0, q2);
  const T3 = dh(0, a3, 0, q3);
  const T4 = dh(Math.PI / 2, 0, d4, q4);
  const T5 = dh(-Math.PI / 2, 0, d5, q5);
  const T6 = dh(0, 0, d6, q6);
  return new THREE.Matrix4().multiply(T1).multiply(T2).multiply(T3).multiply(T4).multiply(T5).multiply(T6);
}

function normalizeAngle(a) {
  let x = a;
  while (x > Math.PI) x -= 2 * Math.PI;
  while (x < -Math.PI) x += 2 * Math.PI;
  return x;
}

function normalizeSolution(q) {
  return q.map(normalizeAngle);
}

const JOINT_LIMITS = [
  deg2rad(-360), deg2rad(360),
  deg2rad(-360), deg2rad(360),
  deg2rad(-360), deg2rad(360),
  deg2rad(-360), deg2rad(360),
  deg2rad(-360), deg2rad(360),
  deg2rad(-360), deg2rad(360),
];

function isWithinLimits(q) {
  for (let i = 0; i < 6; i++) {
    const lo = JOINT_LIMITS[2 * i];
    const hi = JOINT_LIMITS[2 * i + 1];
    if (q[i] < lo || q[i] > hi) return false;
  }
  return true;
}

function uniqueSolutions(sols, tol = deg2rad(0.1)) {
  const out = [];
  outer: for (const s of sols) {
    for (const t of out) {
      let same = true;
      for (let i = 0; i < 6; i++) {
        if (Math.abs(normalizeAngle(s[i] - t[i])) > tol) {
          same = false;
          break;
        }
      }
      if (same) continue outer;
    }
    out.push(s);
  }
  return out;
}

// Lightweight analytic-style IK scaffold (position-only orientation).
export function ikUR10e(pose) {
  const solutions = [];
  const position = pose.position || pose;
  const rpy = pose.rpy || { rx: 0, ry: 0, rz: 0 };

  const e = new THREE.Euler(rpy.rx, rpy.ry, rpy.rz, "XYZ");
  const q = new THREE.Quaternion().setFromEuler(e);
  const T = new THREE.Matrix4().compose(
    new THREE.Vector3(position.x, position.y, position.z),
    q,
    new THREE.Vector3(1, 1, 1)
  );

  const a = new THREE.Vector3(T.elements[2], T.elements[6], T.elements[10]);
  const p = new THREE.Vector3(T.elements[3], T.elements[7], T.elements[11]);

  // wrist center
  const wc = new THREE.Vector3().copy(p).addScaledVector(a, -d6);
  const wx = wc.x;
  const wy = wc.y;
  const wz = wc.z - d1;

  const r = Math.hypot(wx, wy);
  const shoulderOffset = d4;
  if (r < shoulderOffset) return [];

  const phi = Math.atan2(wy, wx);
  const acosTerm = Math.acos(shoulderOffset / r);
  const theta1Candidates = [phi + acosTerm, phi - acosTerm];

  for (const theta1 of theta1Candidates) {
    const c1 = Math.cos(theta1);
    const s1 = Math.sin(theta1);
    const wx1 = c1 * wx + s1 * wy;
    const wz1 = wz;

    const L2 = Math.abs(a2);
    const L3 = Math.abs(a3);
    const D = (wx1 * wx1 + wz1 * wz1 - L2 * L2 - L3 * L3) / (2 * L2 * L3);
    if (D < -1 || D > 1 || Number.isNaN(D)) continue;

    const theta3Candidates = [Math.acos(D), -Math.acos(D)];
    for (const theta3 of theta3Candidates) {
      const c3 = Math.cos(theta3);
      const s3 = Math.sin(theta3);
      const k1 = L2 + L3 * c3;
      const k2 = L3 * s3;
      const theta2 = Math.atan2(wz1, wx1) - Math.atan2(k2, k1);

      const T1 = dh(Math.PI / 2, 0, d1, theta1);
      const T2 = dh(0, a2, 0, theta2);
      const T3 = dh(0, a3, 0, theta3);
      const T0_3 = new THREE.Matrix4().multiply(T1).multiply(T2).multiply(T3);
      const T3_6 = new THREE.Matrix4().copy(T0_3).invert().multiply(T);

      const rzx = T3_6.elements[8];
      const rzy = T3_6.elements[9];
      const rzz = T3_6.elements[10];
      const r53 = Math.hypot(rzx, rzy);
      if (Math.abs(r53) < 1e-8) continue;

      const theta5Candidates = [Math.atan2(r53, rzz), Math.atan2(-r53, rzz)];
      for (const theta5 of theta5Candidates) {
        const s5 = Math.sin(theta5);
        if (Math.abs(s5) < 1e-6) continue;
        const theta4 = Math.atan2(T3_6.elements[6] / s5, T3_6.elements[2] / s5);
        const theta6 = Math.atan2(-rzy / s5, rzx / s5);
        const sol = normalizeSolution([theta1, theta2, theta3, theta4, theta5, theta6]);
        if (isWithinLimits(sol)) solutions.push(sol);
      }
    }
  }

  return uniqueSolutions(solutions);
}
