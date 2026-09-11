export function updateSmoothedVectors(state, payload, alpha = 0.22, uiMode = "grid") {
  const gridPoints = (payload.grid && payload.grid.points) || [];
  const urdfPoints = (payload.urdf && payload.urdf.points) || [];
  const preferGrid = uiMode === "grid";
  const points = preferGrid
    ? (gridPoints.length ? gridPoints : urdfPoints)
    : (urdfPoints.length ? urdfPoints : gridPoints);
  const seen = new Set();

  for (const p of points) {
    const key = `${p.module}:${p.sensor_index}`;
    seen.add(key);
    const prev = state.vectorEma.get(key);
    const fx = Number(p.fx) || 0;
    const fy = Number(p.fy) || 0;
    const fz = Number(p.fz) || 0;
    if (!prev) {
      state.vectorEma.set(key, { fx, fy, fz });
      continue;
    }
    state.vectorEma.set(key, {
      fx: prev.fx + (fx - prev.fx) * alpha,
      fy: prev.fy + (fy - prev.fy) * alpha,
      fz: prev.fz + (fz - prev.fz) * alpha,
    });
  }

  for (const key of state.vectorEma.keys()) {
    if (!seen.has(key)) {
      state.vectorEma.delete(key);
    }
  }
}
