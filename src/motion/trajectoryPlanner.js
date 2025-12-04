import { interpolateJoints } from "./interpolate.js";

export function planTrajectory(path, fps = 60) {
  if (!path || path.length < 2) return { frames: [], fps };
  const frames = [];

  for (let i = 0; i < path.length - 1; i += 1) {
    const a = path[i];
    const b = path[i + 1];
    const duration = Math.max((b.t - a.t) / 1000, 0.4);
    const steps = Math.max(2, Math.round(duration * fps));
    const segment = interpolateJoints(a.q, b.q, steps);
    if (frames.length > 0) {
      segment.shift();
    }
    frames.push(...segment);
  }

  return { frames, fps };
}

export function stepPlayback(state, delta) {
  if (!state || state.frames.length === 0) {
    return { done: true };
  }

  const fps = state.fps ?? 60;
  state.elapsed = (state.elapsed ?? 0) + delta;
  const index = Math.min(
    state.frames.length - 1,
    Math.floor(state.elapsed * fps),
  );
  const frame = state.frames[index];
  const done = index >= state.frames.length - 1;
  return { frame, done, index };
}
