function lerp(a, b, t) {
  return a + (b - a) * t;
}

export function forceColorRgb(norm) {
  const t = Math.max(0, Math.min(1, norm));
  const blue = { r: 59, g: 130, b: 246 };
  const orange = { r: 249, g: 115, b: 22 };
  const red = { r: 239, g: 68, b: 68 };

  let c;
  if (t < 0.55) {
    const k = t / 0.55;
    c = {
      r: Math.round(lerp(blue.r, orange.r, k)),
      g: Math.round(lerp(blue.g, orange.g, k)),
      b: Math.round(lerp(blue.b, orange.b, k)),
    };
  } else {
    const k = (t - 0.55) / 0.45;
    c = {
      r: Math.round(lerp(orange.r, red.r, k)),
      g: Math.round(lerp(orange.g, red.g, k)),
      b: Math.round(lerp(orange.b, red.b, k)),
    };
  }
  return c;
}

export function forceColor(norm) {
  const c = forceColorRgb(norm);
  return `rgb(${c.r},${c.g},${c.b})`;
}

export function drawArrowCanvas(ctx, x0, y0, dx, dy, lineWidth, color) {
  const vecLen = Math.hypot(dx, dy);
  if (vecLen < 1e-6) {
    return;
  }

  const x1 = x0 + dx;
  const y1 = y0 + dy;
  const angle = Math.atan2(dy, dx);
  const baseHeadLen = Math.max(2.6, lineWidth * 3.1);
  const headLen = Math.min(baseHeadLen, vecLen * 0.82);
  const shaftX = x1 - headLen * Math.cos(angle);
  const shaftY = y1 - headLen * Math.sin(angle);
  const prevCap = ctx.lineCap;
  const prevJoin = ctx.lineJoin;

  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.lineCap = "butt";
  ctx.lineJoin = "miter";
  ctx.beginPath();
  ctx.moveTo(x0, y0);
  ctx.lineTo(shaftX, shaftY);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(
    x1 - headLen * Math.cos(angle - Math.PI / 7),
    y1 - headLen * Math.sin(angle - Math.PI / 7),
  );
  ctx.lineTo(
    x1 - headLen * Math.cos(angle + Math.PI / 7),
    y1 - headLen * Math.sin(angle + Math.PI / 7),
  );
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.fill();

  ctx.lineCap = prevCap;
  ctx.lineJoin = prevJoin;
}
