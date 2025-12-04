export function interpolateJoints(a, b, steps) {
  if (!Array.isArray(a) || !Array.isArray(b)) return [];
  const frames = [];
  const count = Math.max(2, steps);
  for (let i = 0; i < count; i += 1) {
    const t = i / (count - 1);
    frames.push(a.map((av, idx) => av + (b[idx] - av) * t));
  }
  return frames;
}
