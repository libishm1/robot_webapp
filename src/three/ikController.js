import * as THREE from "three";
import { ikUR10e } from "../kinematics/UR10eAnalyticalIK.js";

// Approximate UR10e link dimensions (meters) using URDF defaults.
const DH = [
  { a: 0, alpha: Math.PI / 2, d: 0.1273 }, // shoulder pan
  { a: -0.425, alpha: 0, d: 0 }, // shoulder lift
  { a: -0.3922, alpha: 0, d: 0 }, // elbow
  { a: 0, alpha: Math.PI / 2, d: 0.1093 }, // wrist 1
  { a: 0, alpha: -Math.PI / 2, d: 0.09475 }, // wrist 2
  { a: 0, alpha: 0, d: 0.0825 } // wrist 3 / flange
];

const MAX_REACH =
  DH[0].d +
  Math.abs(DH[1].a) +
  Math.abs(DH[2].a) +
  DH[3].d +
  DH[4].d +
  DH[5].d;

const DEFAULT_OPTS = {
  maxIter: 80, // allow deeper refinement for harder targets
  tol: 1e-4,
  damping: 0.01, // lighter damping to move more freely
  stepLimit: 0.5 // larger per-iter step cap to escape shallow minima
};

function dhTransform(a, alpha, d, theta) {
  const cth = Math.cos(theta);
  const sth = Math.sin(theta);
  const ca = Math.cos(alpha);
  const sa = Math.sin(alpha);
  return new THREE.Matrix4().set(
    cth,
    -sth * ca,
    sth * sa,
    a * cth,
    sth,
    cth * ca,
    -cth * sa,
    a * sth,
    0,
    sa,
    ca,
    d,
    0,
    0,
    0,
    1
  );
}

function fkPosition(angles) {
  let tf = new THREE.Matrix4();
  tf.identity();
  DH.forEach((link, i) => {
    tf.multiply(dhTransform(link.a, link.alpha, link.d, angles[i] || 0));
  });
  const pos = new THREE.Vector3();
  pos.setFromMatrixPosition(tf);
  return pos;
}

function numericJacobian(angles, basePos, h = 1e-3) {
  const cols = [];
  for (let i = 0; i < DH.length; i++) {
    const a = [...angles];
    a[i] += h;
    const p = fkPosition(a);
    const col = p.clone().sub(basePos).multiplyScalar(1 / h);
    cols.push(col);
  }
  return cols; // 3 x n (as column vectors)
}

function invert3(m) {
  const a11 = m[0],
    a12 = m[1],
    a13 = m[2];
  const a21 = m[3],
    a22 = m[4],
    a23 = m[5];
  const a31 = m[6],
    a32 = m[7],
    a33 = m[8];
  const det =
    a11 * (a22 * a33 - a23 * a32) -
    a12 * (a21 * a33 - a23 * a31) +
    a13 * (a21 * a32 - a22 * a31);
  if (Math.abs(det) < 1e-9) return null;
  const invDet = 1 / det;
  return [
    (a22 * a33 - a23 * a32) * invDet,
    (a13 * a32 - a12 * a33) * invDet,
    (a12 * a23 - a13 * a22) * invDet,
    (a23 * a31 - a21 * a33) * invDet,
    (a11 * a33 - a13 * a31) * invDet,
    (a13 * a21 - a11 * a23) * invDet,
    (a21 * a32 - a22 * a31) * invDet,
    (a12 * a31 - a11 * a32) * invDet,
    (a11 * a22 - a12 * a21) * invDet
  ];
}

function mul3x3vec(m, v) {
  return [
    m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
    m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
    m[6] * v[0] + m[7] * v[1] + m[8] * v[2]
  ];
}

function dampedLeastSquaresStep(angles, target, opts) {
  const pos = fkPosition(angles);
  const errVec = new THREE.Vector3().subVectors(target, pos);
  const err = errVec.length();
  if (err < opts.tol) return { angles, err, converged: true };

  const Jcols = numericJacobian(angles, pos);

  // JJ^T (3x3)
  const JJt = new Array(9).fill(0);
  for (let c = 0; c < Jcols.length; c++) {
    const col = Jcols[c];
    JJt[0] += col.x * col.x;
    JJt[1] += col.x * col.y;
    JJt[2] += col.x * col.z;
    JJt[3] += col.y * col.x;
    JJt[4] += col.y * col.y;
    JJt[5] += col.y * col.z;
    JJt[6] += col.z * col.x;
    JJt[7] += col.z * col.y;
    JJt[8] += col.z * col.z;
  }
  JJt[0] += opts.damping * opts.damping;
  JJt[4] += opts.damping * opts.damping;
  JJt[8] += opts.damping * opts.damping;

  const JJtInv = invert3(JJt);
  if (!JJtInv) return { angles, err, converged: false, failed: true };

  const e = [errVec.x, errVec.y, errVec.z];
  const y = mul3x3vec(JJtInv, e); // (JJt + λ^2 I)^-1 * e

  // delta = J^T * y
  const delta = new Array(DH.length).fill(0);
  for (let c = 0; c < Jcols.length; c++) {
    const col = Jcols[c];
    delta[c] = col.x * y[0] + col.y * y[1] + col.z * y[2];
  }

  let maxStep = 0;
  delta.forEach((d) => {
    maxStep = Math.max(maxStep, Math.abs(d));
  });
  const scale = maxStep > opts.stepLimit ? opts.stepLimit / maxStep : 1;

  const next = angles.map((a, i) => {
    let v = a + delta[i] * scale;
    if (v > Math.PI) v -= 2 * Math.PI;
    if (v < -Math.PI) v += 2 * Math.PI;
    return v;
  });

  return { angles: next, err, converged: err < opts.tol };
}

// ---------- Simple CCD (about local joint z-axes) -----------------
function fkPointsAndAxes(angles) {
  const frames = [];
  let tf = new THREE.Matrix4();
  tf.identity();
  for (let i = 0; i < DH.length; i++) {
    tf = tf.clone().multiply(dhTransform(DH[i].a, DH[i].alpha, DH[i].d, angles[i] || 0));
    const pos = new THREE.Vector3().setFromMatrixPosition(tf);
    const rot = new THREE.Matrix3().setFromMatrix4(tf);
    const zAxis = new THREE.Vector3(rot.elements[2], rot.elements[5], rot.elements[8]).normalize();
    frames.push({ pos, zAxis });
  }
  return frames;
}

function ccdSolve(angles, target, opts) {
  const current = angles.slice();
  const tgt = new THREE.Vector3(target.x, target.y, target.z);

  for (let iter = 0; iter < opts.maxIter; iter++) {
    const frames = fkPointsAndAxes(current);
    const eePos = frames[frames.length - 1].pos;
    const errVec = new THREE.Vector3().subVectors(tgt, eePos);
    if (errVec.length() < opts.tol) return current;

    for (let i = frames.length - 1; i >= 0; i--) {
      const jointPos = frames[i].pos;
      const axis = frames[i].zAxis;
      const vEE = new THREE.Vector3().subVectors(eePos, jointPos);
      const vTG = new THREE.Vector3().subVectors(tgt, jointPos);
      if (vEE.lengthSq() < 1e-9 || vTG.lengthSq() < 1e-9) continue;

      // Project vectors onto plane orthogonal to axis
      const vEEProj = vEE.clone().sub(axis.clone().multiplyScalar(vEE.dot(axis)));
      const vTGProj = vTG.clone().sub(axis.clone().multiplyScalar(vTG.dot(axis)));
      if (vEEProj.lengthSq() < 1e-9 || vTGProj.lengthSq() < 1e-9) continue;

      vEEProj.normalize();
      vTGProj.normalize();
      const cross = new THREE.Vector3().crossVectors(vEEProj, vTGProj);
      const dot = THREE.MathUtils.clamp(vEEProj.dot(vTGProj), -1, 1);
      let delta = Math.atan2(cross.length(), dot);
      const sign = Math.sign(cross.dot(axis)) || 1;
      delta *= sign;

      // Clamp step
      if (delta > opts.step) delta = opts.step;
      if (delta < -opts.step) delta = -opts.step;

      current[i] = normalizeAngle(current[i] + delta);
      // Update downstream EE by recomputing quickly
      const newFrames = fkPointsAndAxes(current);
      const newEE = newFrames[newFrames.length - 1].pos;
      eePos.copy(newEE);
    }
  }
  return current;
}

// --- Context creation -------------------------------------------------------
export function createIK(mode, joints) {
  if (mode === "ccd") {
    return {
      mode: "ccd",
      data: {
        joints,
        angles: new Array(6).fill(0),
        opts: { maxIter: 60, tol: 1e-4, step: 0.35 }
      }
    };
  }
  if (mode === "analytic") {
    return { mode: "analytic", data: { joints, angles: new Array(6).fill(0) } };
  }
  // default: damped least squares
  return { mode: "damped", data: { joints, angles: new Array(6).fill(0), opts: { ...DEFAULT_OPTS } } };
}

export function solveIK(ctx, target) {
  if (!ctx) return null;

  if (ctx.mode === "analytic") {
    const sols = ikUR10e({
      position: { x: target.x, y: target.y, z: target.z },
      rpy: target.rpy || { rx: 0, ry: 0, rz: 0 }
    });
    if (!sols || sols.length === 0) return null;
    ctx.data.angles = sols[0];
    return ctx.data.angles;
  }

  if (ctx.mode === "ccd") {
    const angles = ccdSolve(ctx.data.angles, target, ctx.data.opts);
    ctx.data.angles = angles.slice();
    return ctx.data.angles;
  }

  const t = new THREE.Vector3(target.x, target.y, target.z);
  const reach = t.length();
  if (reach > MAX_REACH) {
    t.multiplyScalar(MAX_REACH / reach);
  }

  const opts = ctx.mode === "ccd" ? ctx.data.opts : ctx.data.opts;
  let state = { angles: ctx.data.angles.slice(), err: Infinity };
  for (let i = 0; i < opts.maxIter; i++) {
    state = dampedLeastSquaresStep(state.angles, t, opts);
    if (state.converged) break;
    if (state.failed) return null;
  }
  ctx.data.angles = state.angles.slice();
  return ctx.data.angles;
}

export function applyAnglesToJoints(joints, angles) {
  const names = Object.keys(joints);
  names.forEach((name, i) => {
    const joint = joints[name];
    if (joint && typeof joint.setJointValue === "function") {
      joint.setJointValue(angles[i] || 0);
    }
  });
}
