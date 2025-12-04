import { Euler, Quaternion, Vector3 } from "three";
import { radToDeg, round } from "./math";

export function poseFromObject(object) {
  if (!object) return null;
  const position = new Vector3();
  const quaternion = new Quaternion();
  object.getWorldPosition(position);
  object.getWorldQuaternion(quaternion);
  const euler = new Euler().setFromQuaternion(quaternion, "XYZ");

  return {
    position: { x: position.x, y: position.y, z: position.z },
    rpy: [euler.x, euler.y, euler.z],
  };
}

export function poseForDisplay(pose) {
  if (!pose) return null;
  return {
    position: {
      x: round(pose.position.x, 3),
      y: round(pose.position.y, 3),
      z: round(pose.position.z, 3),
    },
    rpyDeg: pose.rpy.map((r) => round(radToDeg(r), 2)),
  };
}
