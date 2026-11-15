// Original, invented markings for the display mesh. This is a decorative
// texture, not an authentic airframe stencil or an engineering reference.
export function buildModelPaint(gl) {
  const canvas = document.createElement('canvas');
  canvas.width = 1024; canvas.height = 512;
  const ctx = canvas.getContext('2d');
  for (const y of [32, 160, 288, 416]) {
    ctx.fillStyle = 'rgba(36, 51, 55, .86)';
    ctx.font = '700 58px Arial, sans-serif';
    ctx.fillText('SIM 01', 430, y + 13);
    ctx.font = '500 12px Arial, sans-serif';
    ctx.fillText('DISPLAY MODEL', 434, y + 33);
    ctx.fillStyle = 'rgba(177, 126, 55, .88)';
    ctx.fillRect(720, y - 8, 70, 9);
    ctx.fillStyle = 'rgba(36, 51, 55, .60)';
    ctx.fillRect(720, y + 8, 46, 3);
    ctx.strokeStyle = 'rgba(36, 51, 55, .38)'; ctx.lineWidth = 1.5;
    ctx.strokeRect(265, y - 26, 116, 54);
  }
  const texture = gl.createTexture();
  gl.bindTexture(gl.TEXTURE_2D, texture);
  gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, true);
  gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, canvas);
  gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, false);
  gl.generateMipmap(gl.TEXTURE_2D);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR_MIPMAP_LINEAR);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.REPEAT);
  return texture;
}
