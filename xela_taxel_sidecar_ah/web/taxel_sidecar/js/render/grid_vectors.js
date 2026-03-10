function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

function lerp(a, b, t) {
  return a + (b - a) * t;
}

function getArrowColor(vectorMode, nz, strength01) {
  if (vectorMode !== "xyz") {
    const alpha = clamp(0.78 + 0.18 * strength01, 0.35, 0.98);
    return `rgba(241,245,249,${alpha.toFixed(3)})`;
  }

  const zAbs = Math.abs(clamp(nz, -1.0, 1.0));
  const mix = Math.pow(zAbs, 0.72);
  const base = { r: 241, g: 245, b: 249 };
  const warm = { r: 255, g: 168, b: 88 };   // +Z
  const cool = { r: 120, g: 191, b: 255 };  // -Z
  const target = nz >= 0.0 ? warm : cool;
  const r = Math.round(lerp(base.r, target.r, mix));
  const g = Math.round(lerp(base.g, target.g, mix));
  const b = Math.round(lerp(base.b, target.b, mix));
  const alpha = clamp(0.50 + 0.20 * strength01 + 0.26 * mix, 0.32, 0.98);
  return `rgba(${r},${g},${b},${alpha.toFixed(3)})`;
}

export function drawGridVectors({
  points,
  payload,
  state,
  cellSize,
  toPx,
  drawArrow,
}) {
  const xyRange = Math.max(1e-6, Number(payload?.meta?.xy_force_range) || 0.8);
  const zRange = Math.max(1e-6, Number(payload?.meta?.z_force_range) || 14.0);
  const vectorMode = state.gridVectorMode === "xyz" ? "xyz" : "xy";
  const zProjX = Number.isFinite(state.gridVectorZProjX) ? state.gridVectorZProjX : -0.46;
  const zProjY = Number.isFinite(state.gridVectorZProjY) ? state.gridVectorZProjY : 0.72;
  const arrowDeadzoneNorm = 0.03;
  const arrowMaxNorm = 2.0;
  const emaWeight = 0.45;
  const baseLineWidth = Math.max(1.2, cellSize * 0.085);

  for (const p of points) {
    const key = `${p.module}:${p.sensor_index}`;
    const rawFx = Number(p.fx) || 0;
    const rawFy = Number(p.fy) || 0;
    const rawFz = Number(p.fz) || 0;
    const ema = state.vectorEma.get(key);
    const fx = ema ? (rawFx * (1.0 - emaWeight) + ema.fx * emaWeight) : rawFx;
    const fy = ema ? (rawFy * (1.0 - emaWeight) + ema.fy * emaWeight) : rawFy;
    const fz = ema ? (rawFz * (1.0 - emaWeight) + (ema.fz || 0) * emaWeight) : rawFz;

    const fxDisplay = fx;
    const fyDisplay = p.module === "right" ? -fy : fy;
    let ux = 0.0;
    let uy = 0.0;
    let magNorm = 0.0;
    let nz = 0.0;

    if (vectorMode === "xyz") {
      const nx = fxDisplay / xyRange;
      const ny = fyDisplay / xyRange;
      nz = fz / zRange;
      const vx = nx + nz * zProjX;
      const vy = ny + nz * zProjY;
      const vec2 = Math.hypot(vx, vy);
      if (vec2 < 1e-9) {
        continue;
      }
      ux = vx / vec2;
      uy = vy / vec2;
      magNorm = Math.min(arrowMaxNorm, Math.sqrt(nx * nx + ny * ny + nz * nz));
    } else {
      const mag = Math.hypot(fxDisplay, fyDisplay);
      if (mag < 1e-9) {
        continue;
      }
      ux = fxDisplay / mag;
      uy = fyDisplay / mag;
      magNorm = Math.min(arrowMaxNorm, mag / xyRange);
    }
    if (magNorm < arrowDeadzoneNorm) {
      continue;
    }

    const strength01 = clamp(
      (magNorm - arrowDeadzoneNorm) / (arrowMaxNorm - arrowDeadzoneNorm),
      0.0,
      1.0,
    );
    const easedStrength = Math.pow(strength01, 0.58);
    const [x, y] = toPx(p.x, p.y);
    const length = cellSize * (0.08 + easedStrength * 1.38);
    const arrowColor = getArrowColor(vectorMode, nz, strength01);

    drawArrow(
      x,
      y,
      ux * length,
      -uy * length,
      baseLineWidth,
      arrowColor,
    );
  }
}
