export const degToRad = (deg) => (deg * Math.PI) / 180;
export const radToDeg = (rad) => (rad * 180) / Math.PI;

export const clamp = (value, min, max) => Math.min(Math.max(value, min), max);

export const round = (value, digits = 3) => {
  const factor = 10 ** digits;
  return Math.round(value * factor) / factor;
};

export const toDegreesArray = (rads) => rads.map((r) => radToDeg(r));
export const toRadiansArray = (degs) => degs.map((d) => degToRad(d));
