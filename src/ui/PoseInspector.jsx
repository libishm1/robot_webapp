import React from "react";
import { radToDeg } from "../utils/math.js";

export default function PoseInspector({ pose, ikError, ikStatus, jointAngles, safetyStatus }) {
  const ready = Boolean(pose);
  const rpy = ready ? pose.rpy : [0, 0, 0];
  const position = ready ? pose.position : { x: 0, y: 0, z: 0 };

  return (
    <div className="panel">
      <div className="row">
        <h2>Pose Inspector</h2>
        <span className={`status-dot ${ready ? "online" : ""}`} />
      </div>
      <div className="grid-two">
        <div className="stack">
          <h3>Position (m)</h3>
          <span className="badge">X: {position.x?.toFixed(3)}</span>
          <span className="badge">Y: {position.y?.toFixed(3)}</span>
          <span className="badge">Z: {position.z?.toFixed(3)}</span>
        </div>
        <div className="stack">
          <h3>RPY (deg)</h3>
          <span className="badge">R: {radToDeg(rpy[0] ?? 0).toFixed(2)}</span>
          <span className="badge">P: {radToDeg(rpy[1] ?? 0).toFixed(2)}</span>
          <span className="badge">Y: {radToDeg(rpy[2] ?? 0).toFixed(2)}</span>
        </div>
      </div>
      <h3>IK Status</h3>
      <div className="row">
        <span>{ikStatus || (ikError ? "Error" : "Solved")}</span>
        <span className={`status-dot ${ikError ? "" : "online"}`} />
      </div>
      {ikError && <div className="badge" style={{ color: "#ff7b95" }}>{ikError}</div>}
      <h3>Safety</h3>
      <div className="row">
        <span>
          {safetyStatus?.collides
            ? "Collision"
            : safetyStatus?.clamped
              ? "Clamped"
              : "Clear"}
        </span>
        <span
          className={`status-dot ${
            safetyStatus?.collides ? "" : safetyStatus?.clamped ? "online" : "online"
          }`}
        />
      </div>
      {safetyStatus?.reasons?.length > 0 && (
        <div className="badge">Workspace: {safetyStatus.reasons.join(", ")}</div>
      )}
      {safetyStatus?.collides && safetyStatus?.collisions?.length > 0 && (
        <div className="badge" style={{ color: "#ff7b95" }}>
          {safetyStatus.collisions.join(", ")}
        </div>
      )}
      <h3>Joints (rad)</h3>
      <div className="grid-two">
        {jointAngles.map((v, idx) => (
          <span className="badge" key={idx}>
            q{idx}: {v.toFixed(3)}
          </span>
        ))}
      </div>
    </div>
  );
}
