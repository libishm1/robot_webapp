import { clamp as clampScalar } from "./math.js";

// Workspace envelope (meters) and simple keep-out around the base.
export const workspace = {
  radius: 1.2, // max planar reach
  keepoutRadius: 0.18, // avoid a cylinder around the pedestal
  minZ: 0.05, // keep tool off the ground
  maxZ: 1.6, // ceiling
};

// UR10e joint soft limits (radians). These are conservative defaults.
export const jointLimits = [
  { min: -Math.PI * 2, max: Math.PI * 2 }, // j0
  { min: -Math.PI * 2, max: Math.PI * 2 }, // j1
  { min: -Math.PI * 2, max: Math.PI * 2 }, // j2
  { min: -Math.PI * 2, max: Math.PI * 2 }, // j3
  { min: -Math.PI * 2, max: Math.PI * 2 }, // j4
  { min: -Math.PI * 2, max: Math.PI * 2 }, // j5
];

export function clampTargetToWorkspace(target) {
  const { radius, keepoutRadius, minZ, maxZ } = workspace;
  const safe = { ...target };
  const reasons = [];
  let clamped = false;

  const r = Math.hypot(safe.x, safe.z);
  if (r > radius) {
    const scale = radius / r;
    safe.x *= scale;
    safe.z *= scale;
    clamped = true;
    reasons.push("radial limit");
  }
  if (r < keepoutRadius) {
    const scale = keepoutRadius / Math.max(r, 1e-3);
    safe.x *= scale;
    safe.z *= scale;
    clamped = true;
    reasons.push("base keep-out");
  }

  const yBefore = safe.y;
  safe.y = clampScalar(safe.y, minZ, maxZ);
  if (safe.y !== yBefore) {
    clamped = true;
    reasons.push("vertical limit");
  }

  return { target: safe, clamped, reasons };
}

export function enforceJointLimits(angles) {
  return angles.map((val, idx) => {
    const lim = jointLimits[idx] || { min: -Infinity, max: Infinity };
    return clampScalar(val, lim.min, lim.max);
  });
}

// Very lightweight collision checks (no full physics engine):
// - keep tool above ground plane
// - keep tool outside base keep-out cylinder
export function checkPoseCollisions(pose) {
  if (!pose) return { collides: false, collisions: [] };
  const issues = [];
  const { position } = pose;
  const r = Math.hypot(position.x, position.z);

  if (position.y < workspace.minZ) {
    issues.push("tool under floor");
  }
  if (r < workspace.keepoutRadius - 0.01) {
    issues.push("inside base keep-out");
  }

  return { collides: issues.length > 0, collisions: issues };
}

