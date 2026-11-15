// Shared paint vocabulary for existing map/scene overlays. It never changes
// input points, projections, stage boundaries, or geographic data.
export const MAP_PAINT = {
  background: '#dce3dc', ink: '#34483f', muted: '#586e61',
  paper: '#fbfcf8', edge: '#aebeb0', grid: 'rgba(62, 87, 71, .17)',
};

export function strokeRoute(ctx, points, color, width = 2.6, dash = []) {
  if (!points || points.length < 2) return;
  ctx.save();
  ctx.lineJoin = 'round'; ctx.lineCap = 'round';
  ctx.setLineDash(dash || []);
  ctx.beginPath();
  let started = false;
  for (const point of points) {
    if (!point) { started = false; continue; }
    if (started) ctx.lineTo(point[0], point[1]);
    else { ctx.moveTo(point[0], point[1]); started = true; }
  }
  // A thin light casing replaces expensive neon blur on every path segment.
  if (width >= 1.4 && !dash?.length) { ctx.strokeStyle = 'rgba(251, 252, 248, .88)'; ctx.lineWidth = width + 2.4; ctx.stroke(); }
  ctx.strokeStyle = color || MAP_PAINT.ink; ctx.lineWidth = width; ctx.stroke();
  ctx.restore();
}

export function paintMapLabel(ctx, text, x, y, color) {
  if (!text) return;
  ctx.save();
  ctx.font = '500 11px system-ui, sans-serif';
  const width = ctx.measureText(text).width + 25, height = 25;
  const bounds = ctx.canvas.getBoundingClientRect();
  const left = Math.max(7, Math.min(x, bounds.width - width - 7));
  const top = Math.max(7, Math.min(y, bounds.height - height - 7));
  ctx.fillStyle = MAP_PAINT.paper; ctx.strokeStyle = 'rgba(61, 83, 67, .24)'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.roundRect(left, top, width, height, 5); ctx.fill(); ctx.stroke();
  ctx.fillStyle = color || MAP_PAINT.muted;
  ctx.beginPath(); ctx.arc(left + 8, top + height / 2, 2.5, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = MAP_PAINT.ink; ctx.textBaseline = 'middle';
  ctx.fillText(text, left + 15, top + height / 2);
  ctx.restore();
}
