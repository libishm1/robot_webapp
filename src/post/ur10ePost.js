const defaultOpts = {
  programName: "web_program",
  jointAccel: 1.4,
  jointVel: 1.2,
  linAccel: 1.2,
  linVel: 0.25,
  blend: 0.0,
  tcp: null, // { x, y, z, rx, ry, rz }
  payload: null, // { mass, cog: [x, y, z] }
  digitalOut: [], // [{ pin: 0, value: true }]
};

const fmtNum = (n) => Number(n || 0).toFixed(6);
const fmtList = (arr) => arr.map(fmtNum).join(", ");
const fmtPose = (pose) => `p[${fmtList([pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz])}]`;

/**
 * Generate URScript from a path. Inspired by RoboDK-style post processors
 * (UR10e), but simplified for browser usage.
 *
 * Path entries support either:
 * - Joint: { q: [j0..j5], a?, v?, r? }
 * - Cartesian: { pose: { x,y,z,rx,ry,rz }, a?, v?, r?, linear?: true }
 */
export function generateURScript(path, options = {}) {
  const opts = { ...defaultOpts, ...options };
  const lines = [];
  const safe = Array.isArray(path) ? path : [];

  lines.push(`def ${opts.programName}():`);

  if (opts.tcp) {
    lines.push(`  set_tcp(${fmtPose(opts.tcp)})`);
  }
  if (opts.payload && opts.payload.mass) {
    const cog = opts.payload.cog || [0, 0, 0];
    lines.push(`  set_payload(${fmtNum(opts.payload.mass)}, [${fmtList(cog)}])`);
  }
  (opts.digitalOut || []).forEach((io) => {
    if (io?.pin !== undefined && io?.value !== undefined) {
      lines.push(`  set_standard_digital_out(${io.pin}, ${io.value ? "True" : "False"})`);
    }
  });

  safe.forEach((wp) => {
    // Cartesian linear motion
    if (wp.pose) {
      const a = fmtNum(wp.a ?? opts.linAccel);
      const v = fmtNum(wp.v ?? opts.linVel);
      const r = wp.r ?? opts.blend;
      const blend = r > 0 ? `, r=${fmtNum(r)}` : "";
      lines.push(`  movel(${fmtPose(wp.pose)}, a=${a}, v=${v}${blend})`);
      return;
    }

    // Joint motion (default)
    const joints = (wp.q || []).map(fmtNum).join(", ");
    const a = fmtNum(wp.a ?? opts.jointAccel);
    const v = fmtNum(wp.v ?? opts.jointVel);
    const r = wp.r ?? opts.blend;
    const blend = r > 0 ? `, r=${fmtNum(r)}` : "";
    lines.push(`  movej([${joints}], a=${a}, v=${v}${blend})`);
  });

  lines.push("end");
  return lines.join("\n");
}
