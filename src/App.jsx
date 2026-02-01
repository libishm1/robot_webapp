import React, { useEffect, useMemo, useState } from "react";
import RobotScene from "./three/scene.jsx";
import JointSliders from "./ui/JointSliders.jsx";
import EndEffectorControls from "./ui/EndEffectorControls.jsx";
import PoseInspector from "./ui/PoseInspector.jsx";
import { addWaypoint, clearPath, getPath } from "./motion/waypointBuffer.js";
import { planTrajectory } from "./motion/trajectoryPlanner.js";
import { generateURScript } from "./post/ur10ePost.js";
import { degToRad } from "./utils/math.js";
import { clampTargetToWorkspace, enforceJointLimits } from "./utils/safety.js";
import CartesianPad from "./ui/CartesianPad.jsx";

const defaultTarget = { x: 0.4, y: 0.2, z: 0.4 };
const defaultAngles = [
  -Math.PI / 2,    // shoulder pan  -90°
  -0.3577925 * Math.PI, // shoulder lift ≈ -102.3° (close to -91.5° shown)
  -Math.PI / 2,    // elbow        -90°
  -Math.PI,        // wrist1      -180°
  -Math.PI / 2,    // wrist2       -90°
  0                // wrist3         0°
];

export default function App() {
  const [target, setTarget] = useState(defaultTarget);
  const [jointAngles, setJointAngles] = useState(defaultAngles);
  const [pose, setPose] = useState(null);
  const [ikError, setIkError] = useState(null);
  const [script, setScript] = useState("");
  const [ikStatus, setIkStatus] = useState("manual");
  const [ikMode, setIkMode] = useState("damped");
  const [ip, setIp] = useState("192.168.0.10");
  const [playbackPlan, setPlaybackPlan] = useState(null);
  const [ikEnabled, setIkEnabled] = useState(false);
  const [pathVersion, setPathVersion] = useState(0);
  const [status, setStatus] = useState("idle");
  const [safetyStatus, setSafetyStatus] = useState({
    clamped: false,
    reasons: [],
    collisions: [],
    collides: false,
  });
  const [advanced, setAdvanced] = useState(false);
  const [nudgeTick, setNudgeTick] = useState(0);
  const [snapTick, setSnapTick] = useState(0);
  const [nudgeStep, setNudgeStep] = useState(0.01);

  useEffect(() => {
    document.title = "robot_webapp";
  }, []);

  const pathCount = useMemo(() => getPath().length, [pathVersion]);

  const handleJointSlider = (index, deg) => {
    const next = [...jointAngles];
    next[index] = degToRad(deg);
    setJointAngles(enforceJointLimits(next));
    setIkEnabled(false);
  };

  const handleTargetChange = (nextTarget, options = { clamp: true }) => {
    if (options.clamp === false) {
      setTarget(nextTarget);
      return;
    }
    const { target: safeTarget, clamped, reasons } = clampTargetToWorkspace(nextTarget);
    setTarget(safeTarget);
    if (clamped) {
      setIkStatus("clamped");
    }
    setSafetyStatus((prev) => ({
      ...prev,
      clamped,
      reasons,
    }));
  };

  const handleAddWaypoint = () => {
    addWaypoint(jointAngles);
    setPathVersion((v) => v + 1);
  };

  const handleHome = () => {
    setJointAngles([0, 0, 0, 0, 0, 0]);
    setTarget(defaultTarget);
    setIkStatus("manual");
  };

  const handleSnapTargetToTcp = () => {
    if (pose?.position) {
      handleTargetChange(pose.position, { clamp: false });
    }
  };

  const handleSnapTcpToTarget = () => {
    setSnapTick((t) => t + 1);
  };

    const handleNudge = (delta) => {
    setTarget((prev) => {
      const next = {
        x: prev.x + (delta.x || 0) * nudgeStep,
        y: prev.y + (delta.y || 0) * nudgeStep,
        z: prev.z + (delta.z || 0) * nudgeStep,
      };
      const { clamped, reasons } = clampTargetToWorkspace(next);
      setSafetyStatus((prevState) => ({ ...prevState, clamped, reasons }));
      return next;
    });
    setNudgeTick((t) => t + 1);
  };

  const handleClear = () => {
    clearPath();
    setPathVersion((v) => v + 1);
    setScript("");
  };

  const handleStepChange = (value) => {
    setNudgeStep(Number(value));
  };

  const handlePlay = () => {
    const path = getPath();
    if (path.length < 2) return;
    const plan = planTrajectory(path, 60);
    setPlaybackPlan(plan);
    setStatus("playing");
  };

  const handleStop = () => {
    setPlaybackPlan(null);
    setStatus("idle");
  };

  const handleGenerateScript = () => {
    const path = getPath();
    setScript(generateURScript(path));
  };

  const handleCopyScript = async () => {
    if (!script) return;
    try {
      await navigator.clipboard.writeText(script);
    } catch (err) {
      console.warn("Clipboard copy failed", err);
    }
  };

  const handleSendScript = () => {
    console.info(
      "Sending from browser requires a local Node bridge. Run:",
      `node src/post/urSender.js ${ip} ./exported.urscript`,
    );
    alert("Run from terminal:\nnode src/post/urSender.js " + ip + " ./exported.urscript");
  };

  return (
    <div className="app">
      <div className="panel" style={{ padding: 0, overflow: "hidden", minHeight: "75vh" }}>
        <RobotScene
          target={target}
          nudgeTick={nudgeTick}
          snapTick={snapTick}
          jointAngles={jointAngles}
          ikEnabled={ikEnabled}
          ikMode={ikMode}
          playbackPlan={playbackPlan}
          onAnglesChange={setJointAngles}
          onPoseChange={setPose}
          onIkError={setIkError}
          onIkDisable={() => {
            setIkEnabled(false);
            setIkError(null);
            setIkStatus("manual");
          }}
          onIkRecovered={() => {
            // No auto-reenable
          }}
          onIkStatusChange={setIkStatus}
          onPlaybackComplete={() => {
            setStatus("idle");
            setPlaybackPlan(null);
          }}
          onRobotReady={() => setStatus("ready")}
          onTargetChange={handleTargetChange}
          onSafetyChange={(collisions) =>
            setSafetyStatus((prev) => ({ ...prev, ...collisions }))
          }
        />
      </div>
      <div className="stack">
        <EndEffectorControls
          target={target}
          onTargetChange={handleTargetChange}
          ikStatus={ikStatus}
          onRecord={handleAddWaypoint}
          onPlay={handlePlay}
          onStop={handleStop}
          onClear={handleClear}
          onHome={handleHome}
          onSnapTargetToTcp={handleSnapTargetToTcp}
          onSnapTcpToTarget={handleSnapTcpToTarget}
          onGenerateScript={handleGenerateScript}
          script={script}
          onCopyScript={handleCopyScript}
          ip={ip}
          onIpChange={setIp}
          onSend={handleSendScript}
          pathCount={pathCount}
          playbackActive={status === "playing"}
          safetyStatus={safetyStatus}
          advanced={advanced}
          step={nudgeStep}
          onStepChange={handleStepChange}
          ikMode={ikMode}
          onIkModeChange={setIkMode}
        />
        <CartesianPad onNudge={handleNudge} step={nudgeStep} onStepChange={handleStepChange} />
        <JointSliders
          angles={jointAngles}
          onChange={handleJointSlider}
          ikEnabled={ikEnabled && advanced}
          onToggleIk={(v) => setIkEnabled(v && advanced)}
          advanced={advanced}
        />
        <PoseInspector
          pose={pose}
          ikError={ikError}
          jointAngles={jointAngles}
          safetyStatus={safetyStatus}
          ikStatus={ikStatus}
        />
      </div>
    </div>
  );
}
