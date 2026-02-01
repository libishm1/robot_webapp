/*
 * Analytical inverse kinematics solver for the UR10e robot.
 * Adapted from the DarioArzaba UR10e solver. Enumerates up to eight solutions.
 * Input pose: { position: {x,y,z} meters, rpy: {rx,ry,rz} radians }
 * Output: array of joint solutions in radians.
 */

const EPS = 1e-9;

const deg2rad = (d) => (d * Math.PI) / 180.0;
const rad2deg = (r) => (r * 180.0) / Math.PI;

const sind = (d) => Math.sin(deg2rad(d));
const cosd = (d) => Math.cos(deg2rad(d));

const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));

function wrapPi(a) {
  let x = (a + Math.PI) % (2 * Math.PI);
  if (x <= 0) x += 2 * Math.PI;
  return x - Math.PI;
}

function matMul4(A, B) {
  const C = Array.from({ length: 4 }, () => [0, 0, 0, 0]);
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += A[i][k] * B[k][j];
      C[i][j] = s;
    }
  }
  return C;
}

function invRigid4(T) {
  const R = [
    [T[0][0], T[0][1], T[0][2]],
    [T[1][0], T[1][1], T[1][2]],
    [T[2][0], T[2][1], T[2][2]],
  ];
  const p = [T[0][3], T[1][3], T[2][3]];
  const Rt = [
    [R[0][0], R[1][0], R[2][0]],
    [R[0][1], R[1][1], R[2][1]],
    [R[0][2], R[1][2], R[2][2]],
  ];
  const t = [
    -(Rt[0][0] * p[0] + Rt[0][1] * p[1] + Rt[0][2] * p[2]),
    -(Rt[1][0] * p[0] + Rt[1][1] * p[1] + Rt[1][2] * p[2]),
    -(Rt[2][0] * p[0] + Rt[2][1] * p[1] + Rt[2][2] * p[2]),
  ];
  return [
    [Rt[0][0], Rt[0][1], Rt[0][2], t[0]],
    [Rt[1][0], Rt[1][1], Rt[1][2], t[1]],
    [Rt[2][0], Rt[2][1], Rt[2][2], t[2]],
    [0, 0, 0, 1],
  ];
}

function VTCP_to_T(VTCP) {
  const [PX, PY, PZ, Psi, Theta, phi] = VTCP;
  return [
    [
      cosd(phi) * cosd(Theta),
      sind(Psi) * cosd(phi) * sind(Theta) - cosd(Psi) * sind(phi),
      sind(Psi) * sind(phi) + cosd(Psi) * cosd(phi) * sind(Theta),
      PX,
    ],
    [
      cosd(Theta) * sind(phi),
      cosd(Psi) * cosd(phi) + sind(Psi) * sind(phi) * sind(Theta),
      cosd(Psi) * sind(phi) * sind(Theta) - sind(Psi) * cosd(phi),
      PY,
    ],
    [-sind(Theta), sind(Psi) * cosd(Theta), cosd(Psi) * cosd(Theta), PZ],
    [0, 0, 0, 1],
  ];
}

function dedupeSolutions(sols, tol = 1e-6) {
  const out = [];
  for (const s of sols) {
    const exists = out.some((u) => u.every((ui, i) => Math.abs(ui - s[i]) < tol));
    if (!exists) out.push(s);
  }
  return out;
}

export function ikUR10e_DarioAll(VTCP, opts = {}) {
  const d1 = opts.d1 ?? 180.7;
  const a2 = opts.a2 ?? -612.7;
  const a3 = opts.a3 ?? -571.55;
  const d4 = opts.d4 ?? 174.15;
  const d5 = opts.d5 ?? 119.85;
  const d6 = opts.d6 ?? (116.55 + 211.5);

  const T = VTCP_to_T(VTCP);
  const Z06 = [T[0][2], T[1][2], T[2][2]];
  const P06 = [T[0][3], T[1][3], T[2][3]];

  const P05 = [
    P06[0] - d6 * Z06[0],
    P06[1] - d6 * Z06[1],
    P06[2] - d6 * Z06[2],
  ];
  const P05XY = Math.hypot(P05[0], P05[1]);
  if (P05XY < EPS) return [];

  const phi1 = rad2deg(Math.atan2(P05[1], P05[0]));
  const phi2 = rad2deg(Math.acos(clamp(d4 / P05XY, -1, 1)));

  const theta1_list = [phi1 + phi2 + 90.0, phi1 - phi2 + 90.0];
  const sols = [];

  for (const theta1 of theta1_list) {
    const arg5 =
      (P06[0] * sind(theta1) - P06[1] * cosd(theta1) - d4) / d6;
    if (Math.abs(arg5) > 1 + 1e-6) continue;

    const theta5a = rad2deg(Math.acos(clamp(arg5, -1, 1)));
    const theta5_list = [theta5a, -theta5a];

    for (const theta5 of theta5_list) {
      const s5 = sind(theta5);
      if (Math.abs(s5) < 1e-7) continue;

      const CT6 =
        (-T[1][0] * sind(theta1) + T[1][1] * cosd(theta1)) / s5;
      const ST6 =
        (T[0][0] * sind(theta1) - T[0][1] * cosd(theta1)) / s5;
      const theta6 = rad2deg(Math.atan2(CT6, ST6));

      const T1 = [
        [cosd(theta1), 0, sind(theta1), 0],
        [sind(theta1), 0, -cosd(theta1), 0],
        [0, 1, 0, d1],
        [0, 0, 0, 1],
      ];
      const T5 = [
        [cosd(theta5), 0, -sind(theta5), 0],
        [sind(theta5), 0, cosd(theta5), 0],
        [0, -1, 0, d5],
        [0, 0, 0, 1],
      ];
      const T6 = [
        [cosd(theta6), -sind(theta6), 0, 0],
        [sind(theta6), cosd(theta6), 0, 0],
        [0, 0, 1, d6],
        [0, 0, 0, 1],
      ];
      const P46 = matMul4(T5, T6);
      const P64 = invRigid4(P46);
      const P10 = invRigid4(T1);
      const P16 = matMul4(P10, T);
      const P14 = matMul4(P16, P64);

      const P14XY = Math.hypot(P14[0][3], P14[1][3]);
      if (P14XY < EPS) continue;

      const arg3 =
        (P14XY * P14XY - a2 * a2 - a3 * a3) / (2 * a2 * a3);
      if (Math.abs(arg3) > 1 + 1e-6) continue;
      const theta3a = rad2deg(Math.acos(clamp(arg3, -1, 1)));
      const theta3_list = [theta3a, -theta3a];

      for (const theta3 of theta3_list) {
        const theta2 =
          rad2deg(Math.atan2(-P14[1][3], -P14[0][3])) -
          rad2deg(Math.asin(clamp((-a3 * sind(theta3)) / P14XY, -1, 1)));

        const T2 = [
          [cosd(theta2), -sind(theta2), 0, a2 * cosd(theta2)],
          [sind(theta2), cosd(theta2), 0, a2 * sind(theta2)],
          [0, 0, 1, 0],
          [0, 0, 0, 1],
        ];
        const T3 = [
          [cosd(theta3), -sind(theta3), 0, a3 * cosd(theta3)],
          [sind(theta3), cosd(theta3), 0, a3 * sind(theta3)],
          [0, 0, 1, 0],
          [0, 0, 0, 1],
        ];
        const P13 = matMul4(T2, T3);
        const P31 = invRigid4(P13);
        const P34 = matMul4(P31, P14);
        const theta4 = rad2deg(Math.atan2(P34[1][0], P34[0][0]));

        const q = [
          wrapPi(deg2rad(theta1)),
          wrapPi(deg2rad(theta2)),
          wrapPi(deg2rad(theta3)),
          wrapPi(deg2rad(theta4)),
          wrapPi(deg2rad(theta5)),
          wrapPi(deg2rad(theta6)),
        ];
        sols.push(q);
      }
    }
  }
  return dedupeSolutions(sols);
}

export function ikUR10eDario(pose, opts = {}) {
  if (!pose || !pose.position) return [];
  const p = pose.position || pose;
  const r = pose.rpy || { rx: 0, ry: 0, rz: 0 };
  const VTCP = [
    p.x * 1000,
    p.y * 1000,
    p.z * 1000,
    rad2deg(r.rx),
    rad2deg(r.ry),
    rad2deg(r.rz),
  ];
  return ikUR10e_DarioAll(VTCP, opts);
}
