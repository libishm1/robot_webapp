# robot_webapp

Web-based UR10e controller that runs fully in the browser: load the official URDF/meshes with Three.js, solve inverse kinematics in real time, drag the end-effector in a React Three Fiber scene, capture waypoints for playback, and export or stream URScript to a physical UR10e.

## Features
- UR10e model from the official ROS2 description (URDF pre-generated in `public/ur10e.urdf`, meshes under `public/meshes/`).
- Live CCD IK via `inverse-kinematics`, draggable target gizmo, and joint slider overrides.
- Waypoint capture with interpolated playback and trajectory preview.
- URScript exporter (joint or linear moves, blend, TCP/payload options) plus a Node-based TCP sender for port 30002 handoff to a real controller.
- Workspace safety limits (keep-out around the base, floor/ceiling clamp) and lightweight collision checks to flag ground/base intrusions.
- Debug panels for joint angles, FK pose, IK/safety health, and script preview.

## Getting Started
1. Install Node.js 18+ (the workspace is a Vite + React project).
2. Install dependencies: `npm install`.
3. Run the app: `npm run dev` and open the served URL.
4. To transmit URScript to a robot, save the script to a file and run `node src/post/urSender.js <robot_ip> <script_file>` from your terminal on the same network.

## Controls
- Orbit/pan/zoom the scene with the mouse; drag the teal gizmo to reposition the end-effector target.
- Toggle IK/Manual and adjust joint sliders to override poses when IK is paused.
- Use **Add Waypoint** to build a path, **Play Path** to animate it, and **Export** to generate URScript.
- Copy the exported URScript or send it via the provided TCP sender helper for execution on port 30002 (URScript export supports joint/linear moves, blend radius, TCP, and payload settings via `generateURScript` options).

## Safety and workspace limits
- Targets are clamped to a configurable workspace radius with a base keep-out cylinder plus floor/ceiling limits.
- Joint angles are clamped to conservative UR10e soft limits before being applied to the robot model.
- A lightweight collision checker flags when the tool dips below the floor or enters the keep-out zone; status is shown in the UI (not a full physics engine).

 .\.node\node-v24.11.1-win-x64\node.exe .\node_modules\vite\bin\vite.js --host --port 5173                              
