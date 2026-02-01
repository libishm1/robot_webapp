import * as THREE from "three";
import { Solver, Goal, DOF, urdfRobotToIKRoot, setUrdfFromIK, setIKFromUrdf } from "closed-chain-ik";
import { ikUR10eDario } from "../kinematics/UR10eDarioIK.js";

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

const JOINT_KEYS = ["j0", "j1", "j2", "j3", "j4", "j5"];

function getChildren(frame) {
  if (!frame) return [];
  if (Array.isArray(frame.children)) return frame.children;
  if (frame.child) return [frame.child];
  return [];
}

function findFrameByName(root, name) {
  if (!root || !name) return null;
  const stack = [root];
  const visited = new Set();
  while (stack.length) {
    const frame = stack.pop();
    if (!frame || visited.has(frame)) continue;
    visited.add(frame);
    if (frame.name === name) return frame;
    const kids = getChildren(frame);
    for (let i = 0; i < kids.length; i++) stack.push(kids[i]);
  }
  return null;
}


function findLinkByName(root, name) {
  if (!root || !name) return null;
  const stack = [root];
  const visited = new Set();
  while (stack.length) {
    const frame = stack.pop();
    if (!frame || visited.has(frame)) continue;
    visited.add(frame);
    if (frame.isLink && frame.name === name) return frame;
    const kids = getChildren(frame);
    for (let i = 0; i < kids.length; i++) stack.push(kids[i]);
  }
  return null;
}

function findLeafLink(root) {
  if (!root) return null;
  const stack = [root];
  const visited = new Set();
  let last = null;
  while (stack.length) {
    const frame = stack.pop();
    if (!frame || visited.has(frame)) continue;
    visited.add(frame);
    if (frame.isLink) last = frame;
    const kids = getChildren(frame);
    for (let i = 0; i < kids.length; i++) stack.push(kids[i]);
  }
  return last || null;
}


function lockRootDof(root) {
  if (!root || !Array.isArray(root.dof)) return;
  root.dof.forEach((d) => {
    const v = root.getDoFValue(d);
    root.setMinLimit(d, v);
    root.setMaxLimit(d, v);
  });
}

function findLeafFrame(root) {
  if (!root) return null;
  const stack = [root];
  const visited = new Set();
  let last = root;
  while (stack.length) {
    const frame = stack.pop();
    if (!frame || visited.has(frame)) continue;
    visited.add(frame);
    last = frame;
    const kids = getChildren(frame);
    for (let i = 0; i < kids.length; i++) stack.push(kids[i]);
  }
  return last;
}

function readJointValues(joints) {
  return JOINT_KEYS.map((key) => joints?.[key]?.jointValue ?? 0);
}

function setGoalPosition(goal, vec) {
  if (!goal || !vec) return;
  if (typeof goal.setPosition === "function") {
    goal.setPosition(vec.x, vec.y, vec.z);
  } else if (goal.position?.set) {
    goal.position.set(vec.x, vec.y, vec.z);
  } else if (goal.position) {
    goal.position.x = vec.x;
    goal.position.y = vec.y;
    goal.position.z = vec.z;
  }
}


function normalizeAngle(angle) {
  let a = (angle + Math.PI) % (2 * Math.PI);
  if (a <= 0) a += 2 * Math.PI;
  return a - Math.PI;
}

function pickClosestSolution(solutions, current) {
  if (!solutions || solutions.length === 0) return null;
  if (!current || current.length !== 6) return solutions[0];
  let best = solutions[0];
  let bestCost = Infinity;
  solutions.forEach((sol) => {
    let cost = 0;
    for (let i = 0; i < 6; i++) {
      const diff = normalizeAngle((sol[i] || 0) - (current[i] || 0));
      cost += diff * diff;
    }
    if (cost < bestCost) {
      bestCost = cost;
      best = sol;
    }
  });
  return best;
}


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
    frames.push({ pos, zAxis, tf: tf.clone() });
  }
  return frames;
}

function ccdSolve(angles, target, opts) {
  const current = angles.slice();
  const tgtPos = new THREE.Vector3(target.x, target.y, target.z);
  const useOrientation = target.rpy != null;
  const tgtQuat = useOrientation
    ? new THREE.Quaternion().setFromEuler(
        new THREE.Euler(target.rpy.rx, target.rpy.ry, target.rpy.rz, "XYZ")
      )
    : null;

  for (let iter = 0; iter < opts.maxIter; iter++) {
    const frames = fkPointsAndAxes(current);
    const eeFrame = frames[frames.length - 1];
    const eePos = eeFrame.pos;
    const posErr = new THREE.Vector3().subVectors(tgtPos, eePos);
    const posErrLen = posErr.length();

    let oriErrAxis = null;
    let oriErrAngle = 0;
    if (useOrientation) {
      const eeQuat = new THREE.Quaternion().setFromRotationMatrix(eeFrame.tf);
      const qErr = tgtQuat.clone().multiply(eeQuat.clone().invert());
      qErr.normalize();
      oriErrAngle = 2 * Math.acos(THREE.MathUtils.clamp(qErr.w, -1, 1));
      if (oriErrAngle > Math.PI) oriErrAngle = 2 * Math.PI - oriErrAngle;
      const s = Math.sqrt(1 - qErr.w * qErr.w);
      if (s > 1e-6) {
        oriErrAxis = new THREE.Vector3(qErr.x / s, qErr.y / s, qErr.z / s).normalize();
      } else {
        oriErrAxis = new THREE.Vector3(0, 0, 1);
      }
    }

    if (posErrLen < opts.tol && (!useOrientation || oriErrAngle < opts.oriTol)) {
      return current;
    }

    for (let i = frames.length - 1; i >= 0; i--) {
      const jointPos = frames[i].pos;
      const axis = frames[i].zAxis;
      const vEE = new THREE.Vector3().subVectors(eePos, jointPos);
      const vTG = new THREE.Vector3().subVectors(tgtPos, jointPos);
      let delta = 0;

      if (vEE.lengthSq() > 1e-9 && vTG.lengthSq() > 1e-9) {
        // Position correction
        const vEEProj = vEE.clone().sub(axis.clone().multiplyScalar(vEE.dot(axis)));
        const vTGProj = vTG.clone().sub(axis.clone().multiplyScalar(vTG.dot(axis)));
        if (vEEProj.lengthSq() > 1e-9 && vTGProj.lengthSq() > 1e-9) {
          vEEProj.normalize();
          vTGProj.normalize();
          const cross = new THREE.Vector3().crossVectors(vEEProj, vTGProj);
          const dot = THREE.MathUtils.clamp(vEEProj.dot(vTGProj), -1, 1);
          const posDelta = Math.atan2(cross.length(), dot) * Math.sign(cross.dot(axis) || 1);
          delta += posDelta;
        }
      }

      if (useOrientation && oriErrAxis) {
        // Orientation correction: project orientation error onto joint axis
        const oriDelta = (oriErrAngle || 0) * (oriErrAxis.dot(axis));
        delta += oriDelta * opts.oriWeight;
      }

      // Clamp step adaptively
      const step = Math.max(opts.stepMin, Math.min(opts.stepMax, Math.abs(delta))) * Math.sign(delta || 1);
      current[i] = normalizeAngle(current[i] + step);

      // Update downstream EE pose quickly
      const newFrames = fkPointsAndAxes(current);
      const newEE = newFrames[newFrames.length - 1];
      eePos.copy(newEE.pos);
      if (useOrientation) {
        const eeQuatNew = new THREE.Quaternion().setFromRotationMatrix(newEE.tf);
        const qErrNew = tgtQuat.clone().multiply(eeQuatNew.clone().invert());
        const s = Math.sqrt(1 - qErrNew.w * qErrNew.w);
        oriErrAngle = 2 * Math.acos(THREE.MathUtils.clamp(qErrNew.w, -1, 1));
        if (oriErrAngle > Math.PI) oriErrAngle = 2 * Math.PI - oriErrAngle;
        if (s > 1e-6) {
          oriErrAxis = new THREE.Vector3(qErrNew.x / s, qErrNew.y / s, qErrNew.z / s).normalize();
        } else {
          oriErrAxis = new THREE.Vector3(0, 0, 1);
        }
      }
    }
  }
  return current;
}

// --- Context creation -------------------------------------------------------
export function createIK(mode, joints, options = {}) {

  if (mode === "closed") {
    const robot = options.robot;
    const endEffector = options.endEffector || null;
    if (!robot) {
      return { mode: "closed", data: { joints, angles: new Array(6).fill(0) } };
    }
    const ikRoot = urdfRobotToIKRoot(robot, true);
    setIKFromUrdf(ikRoot, robot);
    lockRootDof(ikRoot);
    const endName = options.endEffectorName || endEffector?.name || "wrist_3_link";
    const endLink = findLinkByName(ikRoot, endName) || findLeafLink(ikRoot);
    const goal = new Goal();
    // Translation-only goal keeps the solver focused on position nudges.
    goal.setDoF(DOF.X, DOF.Y, DOF.Z);
    if (endEffector) {
      const initPos = new THREE.Vector3();
      const initQuat = new THREE.Quaternion();
      endEffector.getWorldPosition(initPos);
      endEffector.getWorldQuaternion(initQuat);
      if (typeof goal.setWorldPosition === "function") {
        goal.setWorldPosition(initPos.x, initPos.y, initPos.z);
      } else {
        setGoalPosition(goal, initPos);
      }
      if (typeof goal.setQuaternion === "function") {
        goal.setQuaternion(initQuat.x, initQuat.y, initQuat.z, initQuat.w);
      }
    }
    if (endLink && endLink.isLink && typeof goal.makeClosure === "function") {
      try {
        goal.makeClosure(endLink);
      } catch (err) {
        // Fallback: try a leaf or skip closure to avoid crashing the app.
        const leaf = findLeafFrame(ikRoot);
        if (leaf && leaf !== endLink) {
          try {
            goal.makeClosure(leaf);
          } catch (_) {
            /* ignore */
          }
        }
      }
    }
    const solver = new Solver(ikRoot);
    solver.maxIterations = 20;
    solver.translationStep = 0.01;
    solver.rotationStep = 0.01;
    solver.translationErrorClamp = 0.2;
    solver.rotationErrorClamp = 0.2;
    return {
      mode: "closed",
      data: {
        joints,
        robot,
        endEffector,
        ikRoot,
        goal,
        solver,
        angles: new Array(6).fill(0),
        smoothTarget: null,
        smoothAlpha: 0.25,
      },
    };
  }
  if (mode === "ccd") {
    return {
      mode: "ccd",
      data: {
        joints,
        angles: new Array(6).fill(0),
        opts: {
          maxIter: 120,
          tol: 1e-4,
          oriTol: 1e-2,
          stepMin: 0.01,
          stepMax: 0.35,
          oriWeight: 0.5
        }
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

  if (ctx.mode === "closed") {
    const data = ctx.data || {};
    if (!data.goal || !data.solver || !data.ikRoot || !data.robot) return null;
    // Sync IK tree from current URDF pose before solving.
    setIKFromUrdf(data.ikRoot, data.robot);

    const tgt = new THREE.Vector3(target.x, target.y, target.z);
    if (!data.smoothTarget) {
      data.smoothTarget = tgt.clone();
    } else {
      data.smoothTarget.lerp(tgt, data.smoothAlpha ?? 0.25);
    }
    if (typeof data.goal.setWorldPosition === "function") {
      data.goal.setWorldPosition(data.smoothTarget.x, data.smoothTarget.y, data.smoothTarget.z);
    } else {
      setGoalPosition(data.goal, data.smoothTarget);
    }

    if (data.endEffector && data.goal.rotationDoFCount > 0 && typeof data.goal.setQuaternion === "function") {
      const eeQuat = new THREE.Quaternion();
      data.endEffector.getWorldQuaternion(eeQuat);
      data.goal.setQuaternion(eeQuat.x, eeQuat.y, eeQuat.z, eeQuat.w);
    }

    data.solver.solve();
    setUrdfFromIK(data.robot, data.ikRoot);
    const angles = readJointValues(data.joints);
    data.angles = angles.slice();
    return data.angles;
  }

  if (ctx.mode === "analytic") {
    const sols = ikUR10eDario({
      position: { x: target.x, y: target.y, z: target.z },
      rpy: target.rpy || { rx: 0, ry: 0, rz: 0 }
    });
    if (!sols || sols.length === 0) return null;
    const chosen = pickClosestSolution(sols, ctx.data.angles);
    ctx.data.angles = (chosen || sols[0]).slice();
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
