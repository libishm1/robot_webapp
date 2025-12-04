import React from "react";

export default function EndEffectorControls({
  target,
  onTargetChange,
  onRecord,
  onPlay,
  onStop,
  onClear,
  onGenerateScript,
  script,
  onCopyScript,
  ip,
  onIpChange,
  onSend,
  pathCount,
  playbackActive,
  safetyStatus,
  advanced,
  ikStatus,
  onHome,
  onSnapTargetToTcp,
  onSnapTcpToTarget,
  step,
  onStepChange,
  ikMode,
  onIkModeChange,
}) {
  const updateField = (key, value) => {
    onTargetChange?.({ ...target, [key]: value });
  };

  return (
    <div className="panel">
      <div className="row">
        <h2>End-Effector</h2>
        <span className="badge">
          IK: {ikStatus || "manual"}
        </span>
        {safetyStatus?.clamped || safetyStatus?.collides ? (
          <span className="badge" style={{ color: "#ff7b95" }}>
            Safety active
          </span>
        ) : (
          <span className="badge">Safe</span>
        )}
      </div>
      <div className="row" style={{ marginTop: 6, gap: 8 }}>
        <button className="secondary" onClick={onHome}>Home joints</button>
        <button className="secondary" onClick={onSnapTargetToTcp}>Snap target to TCP</button>
        <button className="secondary" onClick={onSnapTcpToTarget}>Snap TCP to target</button>
      </div>
      <div className="row" style={{ marginTop: 6 }}>
        <label className="row" style={{ gap: 6, width: "100%" }}>
          <span>Step (m)</span>
          <select value={step} onChange={(e) => onStepChange?.(e.target.value)} style={{ width: "100%" }}>
            {[0.005, 0.01, 0.05, 0.1].map((v) => (
              <option key={v} value={v}>{v}</option>
            ))}
          </select>
        </label>
      </div>
      <div className="row" style={{ marginTop: 6 }}>
        <label className="row" style={{ gap: 6, width: "100%" }}>
          <span>IK Mode</span>
          <select value={ikMode} onChange={(e) => onIkModeChange?.(e.target.value)} style={{ width: "100%" }}>
            <option value="damped">Damped LS (stable)</option>
            <option value="ccd">CCD (basic)</option>
            <option value="analytic">Analytic (UR10e)</option>
          </select>
        </label>
      </div>
      <div className="grid-two">
        {["x", "y", "z"].map((axis) => (
          <label key={axis} className="stack">
            <span>{axis.toUpperCase()} (m)</span>
            <input
              type="number"
              step="0.01"
              value={Number(target[axis]).toFixed(3)}
              onChange={(e) => updateField(axis, Number(e.target.value))}
            />
          </label>
        ))}
      </div>
      <div className="row" style={{ marginTop: 10 }}>
        <button onClick={onRecord}>Add Waypoint</button>
        <button className="secondary" onClick={onClear}>
          Clear
        </button>
      </div>
      {safetyStatus?.reasons?.length > 0 && (
        <div className="muted">
          Clamped: {safetyStatus.reasons.join(", ")}
        </div>
      )}
      {safetyStatus?.collides && safetyStatus?.collisions?.length > 0 && (
        <div className="badge" style={{ color: "#ff7b95", marginTop: 6 }}>
          Collision: {safetyStatus.collisions.join(", ")}
        </div>
      )}
      <div className="row" style={{ marginTop: 6 }}>
        <button onClick={onPlay} disabled={playbackActive || pathCount < 2}>
          {playbackActive ? "Playing..." : "Play Path"}
        </button>
        <button className="secondary" onClick={onStop}>
          Stop
        </button>
      </div>
      <h3>URScript</h3>
      <div className="row">
        <span>Waypoints: {pathCount}</span>
        <button className="secondary" onClick={onGenerateScript}>
          Export
        </button>
      </div>
      <textarea readOnly value={script} placeholder="Generate a URScript to preview it here." />
      <div className="row">
        <button onClick={onCopyScript}>Copy</button>
        <span className="muted">Use the TCP sender for a real UR.</span>
      </div>
      {advanced && (
        <>
          <h3>TCP Sender</h3>
          <div className="grid-two">
            <label className="stack">
              <span>Robot IP</span>
              <input
                type="text"
                value={ip}
                onChange={(e) => onIpChange?.(e.target.value)}
                placeholder="192.168.0.10"
              />
            </label>
            <label className="stack">
              <span>Port</span>
              <input type="text" value="30002" disabled />
            </label>
          </div>
          <div className="row" style={{ marginTop: 6 }}>
            <button onClick={onSend} className="secondary">
              Send URScript (local Node)
            </button>
            <span className="badge">TCP</span>
          </div>
        </>
      )}
    </div>
  );
}
