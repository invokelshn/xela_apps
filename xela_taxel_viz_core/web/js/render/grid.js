import { drawGridVectors } from "./grid_vectors.js";

export function drawGridMode({
  payload,
  ctx,
  canvas,
  state,
  forceColor,
  drawArrow,
  drawEmpty,
}) {
  const points = (payload.grid && payload.grid.points) || [];
  if (!points.length) {
    drawEmpty("No grid points");
    return;
  }

  const gridCellSize = Math.max(1e-9, Number(payload.grid?.cell_size) || 0.015);
  const separatorColsLeft = Array.isArray(payload.grid?.separator_cols_left)
    ? payload.grid.separator_cols_left
    : [];
  const separatorColsRight = Array.isArray(payload.grid?.separator_cols_right)
    ? payload.grid.separator_cols_right
    : [];

  const pad = Math.max(28, canvas.width * 0.05);
  const minX = Math.min(...points.map((p) => p.x));
  const maxX = Math.max(...points.map((p) => p.x));
  const minY = Math.min(...points.map((p) => p.y));
  const maxY = Math.max(...points.map((p) => p.y));

  const rangeX = Math.max(1e-9, maxX - minX);
  const rangeY = Math.max(1e-9, maxY - minY);

  const sx = (canvas.width - pad * 2) / rangeX;
  const sy = (canvas.height - pad * 2) / rangeY;
  const scale = Math.min(sx, sy);

  const toPx = (x, y) => {
    const px = pad + (x - minX) * scale;
    const py = canvas.height - (pad + (y - minY) * scale);
    return [px, py];
  };

  const cellSize = Math.max(6, Number(payload.grid?.cell_size || 0.015) * scale * 0.94);

  ctx.fillStyle = "#0b1222";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  const modulePtsByName = new Map();
  for (const moduleName of ["left", "right"]) {
    const modulePts = points.filter((p) => p.module === moduleName);
    modulePtsByName.set(moduleName, modulePts);
    if (!modulePts.length) continue;

    const bx0 = Math.min(...modulePts.map((p) => p.x));
    const bx1 = Math.max(...modulePts.map((p) => p.x));
    const by0 = Math.min(...modulePts.map((p) => p.y));
    const by1 = Math.max(...modulePts.map((p) => p.y));
    const a = toPx(bx0, by0);
    const b = toPx(bx1, by1);
    const left = Math.min(a[0], b[0]) - cellSize * 0.8;
    const top = Math.min(a[1], b[1]) - cellSize * 0.8;
    const width = Math.abs(a[0] - b[0]) + cellSize * 1.6;
    const height = Math.abs(a[1] - b[1]) + cellSize * 1.6;

    ctx.fillStyle = moduleName === "left" ? "rgba(59,130,246,0.08)" : "rgba(20,184,166,0.08)";
    ctx.strokeStyle = "rgba(148,163,184,0.35)";
    ctx.lineWidth = 1.0;
    ctx.beginPath();
    ctx.rect(left, top, width, height);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = "#cbd5e1";
    ctx.font = `${Math.max(10, Math.floor(canvas.height * 0.018))}px monospace`;
    ctx.textAlign = "left";
    ctx.fillText(moduleName.toUpperCase(), left + 6, top + 16);
  }

  for (const p of points) {
    const [x, y] = toPx(p.x, p.y);
    const color = forceColor(p.norm || 0);
    ctx.fillStyle = color;
    ctx.strokeStyle = "rgba(8,15,30,0.95)";
    ctx.lineWidth = Math.max(1, cellSize * 0.08);
    ctx.beginPath();
    ctx.rect(x - cellSize / 2, y - cellSize / 2, cellSize, cellSize);
    ctx.fill();
    ctx.stroke();
  }

  for (const moduleName of ["left", "right"]) {
    const modulePts = modulePtsByName.get(moduleName) || [];
    if (!modulePts.length) continue;
    const separators = moduleName === "left" ? separatorColsLeft : separatorColsRight;
    if (!separators.length) continue;

    const moduleMinX = Math.min(...modulePts.map((p) => p.x));
    const moduleMaxY = Math.max(...modulePts.map((p) => p.y));
    const moduleMinY = Math.min(...modulePts.map((p) => p.y));
    const moduleBaseX = moduleMinX - gridCellSize * 0.5;
    const worldTopY = moduleMaxY + gridCellSize * 0.5;
    const worldBottomY = moduleMinY - gridCellSize * 0.5;

    ctx.save();
    ctx.strokeStyle = "rgba(241,245,249,0.95)";
    ctx.lineWidth = Math.max(2.0, cellSize * 0.12);
    for (const colRaw of separators) {
      const col = Number(colRaw);
      if (!Number.isFinite(col)) continue;
      const worldX = moduleBaseX + col * gridCellSize;
      const [x0, y0] = toPx(worldX, worldTopY);
      const [x1, y1] = toPx(worldX, worldBottomY);
      ctx.beginPath();
      ctx.moveTo(x0, y0);
      ctx.lineTo(x1, y1);
      ctx.stroke();
    }
    ctx.restore();
  }

  drawGridVectors({
    points,
    payload,
    state,
    cellSize,
    toPx,
    drawArrow,
  });
}
