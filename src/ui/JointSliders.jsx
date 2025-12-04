import React from "react";
import { radToDeg } from "../utils/math.js";

const labels = ["Shoulder Pan", "Shoulder Lift", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"];

export default function JointSliders({ angles, onChange, ikEnabled, onToggleIk, advanced }) {
  return (
    <div className="panel">
      <div className="row">
        <h2>Joint Overrides</h2>
        {advanced && (
          <div className="row" style={{ gap: 6 }}>
            <span className={`status-dot ${ikEnabled ? "online" : ""}`} />
            <button className="secondary" onClick={() => onToggleIk?.(!ikEnabled)}>
              {ikEnabled ? "IK Live" : "Manual"}
            </button>
          </div>
        )}
      </div>
      <div className="stack">
        {labels.map((label, idx) => {
          const deg = radToDeg(angles[idx] ?? 0);
          return (
            <div key={label}>
              <div className="row">
                <span>{label}</span>
                <span className="badge">{deg.toFixed(1)}°</span>
              </div>
              <input
                type="range"
                min={-180}
                max={180}
                step={0.5}
                value={deg}
                onChange={(e) => onChange?.(idx, Number(e.target.value))}
              />
            </div>
          );
        })}
      </div>
    </div>
  );
}
