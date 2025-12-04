import React from "react";

export default function CartesianPad({ onNudge, step = 0.01, onStepChange }) {
  const move = (dx = 0, dy = 0, dz = 0) => {
    onNudge?.({ x: dx, y: dy, z: dz });
  };

  return (
    <div className="panel">
      <h2>Cartesian Nudge</h2>
      <div className="row" style={{ marginBottom: 8 }}>
        <label className="row" style={{ gap: 6, width: "100%" }}>
          <span>Step (m)</span>
          <select value={step} onChange={(e) => onStepChange?.(e.target.value)} style={{ width: "100%" }}>
            {[0.005, 0.01, 0.05, 0.1].map((v) => (
              <option key={v} value={v}>{v}</option>
            ))}
          </select>
        </label>
      </div>
      <div className="grid-two" style={{ alignItems: "center" }}>
        <div className="stack" style={{ alignItems: "center" }}>
          <div className="row" style={{ gap: 8 }}>
            <button className="secondary" style={{ borderColor: "#00ff00", color: "#00ff00" }} onClick={() => move(0, step, 0)}>Y+</button>
          </div>
          <div className="row" style={{ gap: 8 }}>
            <button className="secondary" style={{ borderColor: "#ff0000", color: "#ff0000" }} onClick={() => move(-step, 0, 0)}>X-</button>
            <button className="secondary" style={{ borderColor: "#ff0000", color: "#ff0000" }} onClick={() => move(step, 0, 0)}>X+</button>
          </div>
          <div className="row" style={{ gap: 8 }}>
            <button className="secondary" style={{ borderColor: "#00ff00", color: "#00ff00" }} onClick={() => move(0, -step, 0)}>Y-</button>
          </div>
        </div>
        <div className="stack" style={{ alignItems: "center", gap: 6 }}>
          <button className="secondary" style={{ borderColor: "#0000ff", color: "#0000ff" }} onClick={() => move(0, 0, step)}>Z+</button>
          <button className="secondary" style={{ borderColor: "#0000ff", color: "#0000ff" }} onClick={() => move(0, 0, -step)}>Z-</button>
        </div>
      </div>
    </div>
  );
}
