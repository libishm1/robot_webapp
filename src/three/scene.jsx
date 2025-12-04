import React, { Suspense, useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { Environment, OrbitControls, PerspectiveCamera, Stats } from "@react-three/drei";
import { applyAnglesToJoints, createIK, solveIK } from "./ikController.js";
import { loadUR10e } from "./robotLoader.js";
import { TargetGizmo } from "./targetGizmo.jsx";
import { poseFromObject } from "../utils/transforms.js";
import { stepPlayback } from "../motion/trajectoryPlanner.js";
import { checkPoseCollisions, enforceJointLimits } from "../utils/safety.js";

function SceneContent({
  target,
  nudgeTick,
  snapTick,
  jointAngles,
  ikEnabled,
  ikMode,
  playbackPlan,
  onAnglesChange,
  onPoseChange,
  onIkError,
  onIkDisable,
  onIkRecovered,
  onIkStatusChange,
  onPlaybackComplete,
  onRobotReady,
  onTargetChange,
  onSafetyChange,
}) {
  const { scene } = useThree();
  const jointsRef = useRef(null);
  const robotRef = useRef(null);
  const eeRef = useRef(null);
  const ikRef = useRef(null);
  const lastStableTarget = useRef(target);
  const playbackRef = useRef(null);
  const manualAngles = useRef(jointAngles);
  const [dragging, setDragging] = useState(false);
  const lastPoseBroadcast = useRef(0);
  const lastAngleBroadcast = useRef(0);

  const syncTargetToEE = () => {
    if (!eeRef.current) return;
    const pose = poseFromObject(eeRef.current);
    if (pose?.position) {
      onTargetChange?.(pose.position, { clamp: false });
      lastPoseBroadcast.current = 0; // force immediate rebroadcast next frame
    }
  };

  const driveJointsToTarget = (pos, { snapTarget = true, force = false } = {}) => {
    if (!ikEnabled && !force) return;
    if (!ikRef.current || !jointsRef.current) return;
    const targetVec = new THREE.Vector3(pos.x, pos.y, pos.z);
    const solverTarget =
      robotRef.current != null
        ? targetVec.clone().applyQuaternion(robotRef.current.quaternion.clone().invert())
        : targetVec;
    const solved = solveIK(ikRef.current, solverTarget);
    if (!solved) {
      if (lastStableTarget.current) {
        onTargetChange?.(lastStableTarget.current, { clamp: false });
      }
      onIkDisable?.();
      onIkStatusChange?.("unreachable");
      return;
    }
    const limited = enforceJointLimits(solved);
    applyAnglesToJoints(jointsRef.current, limited, robotRef.current);
    manualAngles.current = limited;
    onAnglesChange?.(limited);
    onIkRecovered?.();
    onIkStatusChange?.("solved");
    lastStableTarget.current = pos;
    if (snapTarget && eeRef.current) {
      const pose = poseFromObject(eeRef.current);
      if (pose?.position) {
        onTargetChange?.(pose.position, { clamp: false });
        lastStableTarget.current = pose.position;
      }
    }
  };

  useEffect(() => {
    manualAngles.current = jointAngles;
    if (ikRef.current) {
      ikRef.current.data.angles = jointAngles.slice();
    }
    // Always resync target to flange after manual joint edits.
    syncTargetToEE();
  }, [jointAngles]);

  // Apply manual joint overrides immediately when IK is disabled.
  useEffect(() => {
    if (!ikEnabled && jointsRef.current) {
      manualAngles.current = jointAngles;
      applyAnglesToJoints(jointsRef.current, jointAngles, robotRef.current);
    }
  }, [ikEnabled, jointAngles]);

  useEffect(() => {
    playbackRef.current = playbackPlan
      ? { ...playbackPlan, elapsed: 0 }
      : null;
  }, [playbackPlan]);

  // Rebuild IK solver when mode changes, preserving current angles.
  useEffect(() => {
    if (!jointsRef.current) return;
    const angles = manualAngles.current || jointAngles;
    ikRef.current = createIK(ikMode, jointsRef.current);
    if (ikRef.current?.data) {
      ikRef.current.data.angles = angles.slice();
    }
    onIkStatusChange?.("manual");
  }, [ikMode]); // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    if (nudgeTick > 0 && ikEnabled) {
      driveJointsToTarget(target, { snapTarget: true, force: true });
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [nudgeTick]);

  useEffect(() => {
    if (snapTick > 0) {
      driveJointsToTarget(target, { snapTarget: true, force: true });
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [snapTick]);

  // Attempt a one-shot IK solve when the target changes (for nudge buttons),
  // only if IK is enabled.
  useEffect(() => {
    if (ikEnabled) {
      driveJointsToTarget(target, { snapTarget: false });
    }
  }, [target]); // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    let cancelled = false;

    // Remove previous robot if we owned it.
    if (robotRef.current) {
      scene.remove(robotRef.current);
      robotRef.current = null;
      jointsRef.current = null;
      eeRef.current = null;
    }

    loadUR10e(scene)
      .then(({ robot, joints, endEffector }) => {
        if (cancelled) return;
        robotRef.current = robot;
        jointsRef.current = joints;
        eeRef.current = endEffector;
        ikRef.current = createIK(ikMode, joints);
        onIkStatusChange?.("manual");
        if (endEffector) {
          const pose = poseFromObject(endEffector);
          onTargetChange?.(pose?.position || target, { clamp: false });
        }
        onRobotReady?.({ robot, joints, endEffector });
      })
      .catch((err) => {
        console.error("Failed to load UR10e", err);
        onIkError?.("URDF load failed");
      });

    return () => {
      cancelled = true;
      if (robotRef.current) {
        scene.remove(robotRef.current);
        robotRef.current = null;
        jointsRef.current = null;
        eeRef.current = null;
      }
    };
  }, []); // run only once on mount

  useFrame((state, delta) => {
    if (!jointsRef.current) return;

    // Playback path if requested
    if (playbackRef.current) {
      const res = stepPlayback(playbackRef.current, delta);
      if (res.frame) {
        applyAnglesToJoints(jointsRef.current, res.frame, robotRef.current);
        manualAngles.current = res.frame;
        broadcastPose(state);
      }
      if (res.done) {
        playbackRef.current = null;
        onPlaybackComplete?.();
      }
      return;
    }

    if (ikEnabled) {
      const targetVec = new THREE.Vector3(target.x, target.y, target.z);
      const solverTarget =
        robotRef.current != null
          ? targetVec.clone().applyQuaternion(robotRef.current.quaternion.clone().invert())
          : targetVec;
      const solved = solveIK(ikRef.current, solverTarget);
      if (solved) {
        const limited = enforceJointLimits(solved);
        applyAnglesToJoints(jointsRef.current, limited, robotRef.current);
        manualAngles.current = limited;
        if (state.clock.elapsedTime - lastAngleBroadcast.current > 0.04) {
          onAnglesChange?.(limited);
          lastAngleBroadcast.current = state.clock.elapsedTime;
        }
        onIkError?.(null);
        onIkRecovered?.();
      } else {
        onIkError?.("IK did not converge");
        onIkDisable?.();
      }
    } else {
      applyAnglesToJoints(jointsRef.current, manualAngles.current, robotRef.current);
    }

    broadcastPose(state);
  });

  const broadcastPose = (state) => {
    if (!eeRef.current) return;
    if (state.clock.elapsedTime - lastPoseBroadcast.current > 0.05) {
      const pose = poseFromObject(eeRef.current);
      onPoseChange?.(pose);
      // Keep gizmo aligned to the current end-effector pose when not dragging,
      // so the TCP (flange/tool) and target stay coincident while IK is active.
      if (!dragging && pose?.position && ikEnabled) {
        onTargetChange?.(pose.position, { clamp: false });
      }
      if (pose?.position && ikEnabled) {
        lastStableTarget.current = pose.position;
      }
      const collisions = checkPoseCollisions(pose);
      onSafetyChange?.(collisions);
      lastPoseBroadcast.current = state.clock.elapsedTime;
    }
  };

  return (
    <>
      <PerspectiveCamera makeDefault position={[1.6, 1.1, 1.6]} fov={50} />
      <OrbitControls
        enabled={!dragging}
        target={[0, 0.6, 0]}
        maxDistance={4}
        minDistance={0.6}
      />
      <hemisphereLight args={["#9fc9ff", "#0c101d", 0.6]} />
      <directionalLight position={[2, 3, 2]} intensity={1.0} />
      <gridHelper args={[5, 20, "#2a3a65", "#14203a"]} position={[0, -0.001, 0]} />
      <axesHelper args={[0.3]} />
      <TargetGizmo
        position={target}
        onChange={(pos) => {
          onIkError?.(null);
          driveJointsToTarget(pos, { snapTarget: true, force: true });
        }}
        onDragging={(v) => {
          setDragging(v);
          if (!v) {
            // Reattach gizmo to the current TCP after drag ends.
            syncTargetToEE();
          }
        }}
      />
      <Environment preset="city" />
      <Stats />
    </>
  );
}

export default function RobotScene(props) {
  return (
    <Canvas shadows dpr={[1, 2]} gl={{ antialias: true }}>
      <color attach="background" args={["#06080f"]} />
      <Suspense fallback={null}>
        <SceneContent {...props} />
      </Suspense>
    </Canvas>
  );
}
