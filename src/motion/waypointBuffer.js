let buffer = [];

export function addWaypoint(jointAngles) {
  if (!jointAngles) return;
  buffer.push({ t: Date.now(), q: [...jointAngles] });
}

export function getPath() {
  return [...buffer];
}

export function clearPath() {
  buffer = [];
}
