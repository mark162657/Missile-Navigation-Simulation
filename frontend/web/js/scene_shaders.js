// Presentation-only shaders. These affect light and surface appearance, not
// terrain geometry, geographic data, vehicle dimensions, or simulation state.
export const MESH_VS = `#version 300 es
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
layout(location=2) in vec3 aCol;
uniform mat4 uVP, uModel;
out vec3 vNrm, vCol, vWorld, vLocal;
void main() {
  vec4 world = uModel * vec4(aPos, 1.0);
  vWorld = world.xyz;
  vLocal = aPos;
  vNrm = transpose(inverse(mat3(uModel))) * aNrm;
  vCol = aCol;
  gl_Position = uVP * world;
}`;

export const MESH_FS = `#version 300 es
precision highp float;
uniform vec3 uSun, uCamPos, uFogColor;
uniform float uFogDensity, uEmissive, uAlpha, uAmbient, uSpec;
uniform int uMaterial;
uniform sampler2D uPaint;
in vec3 vNrm, vCol, vWorld, vLocal;
out vec4 frag;
const float PI = 3.14159265;
vec3 toneMap(vec3 color) {
  return pow(clamp((color * (2.51 * color + 0.03)) /
    (color * (2.43 * color + 0.59) + 0.14), 0.0, 1.0), vec3(1.0 / 2.2));
}
float hash(vec2 p) {
  return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
}
float noise(vec2 p) {
  vec2 cell = floor(p), f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  return mix(mix(hash(cell), hash(cell + vec2(1, 0)), f.x),
    mix(hash(cell + vec2(0, 1)), hash(cell + vec2(1, 1)), f.x), f.y);
}
void main() {
  vec3 n = normalize(vNrm);
  if (!gl_FrontFacing) n = -n;
  vec3 view = normalize(uCamPos - vWorld);
  vec3 halfVector = normalize(view + uSun);
  float nv = max(dot(n, view), 0.001);
  float nl = max(dot(n, uSun), 0.0);
  float nh = max(dot(n, halfVector), 0.0);
  float vh = max(dot(view, halfVector), 0.0);
  vec3 base = pow(max(vCol, vec3(0.0)), vec3(2.2));
  float roughness = clamp(0.90 - uSpec * 0.9, 0.25, 0.95);
  if (uMaterial == 1 || uMaterial == 3) {
    // Invented paint seams, not an engineering or manufacturing reference.
    float ring = abs(fract(vLocal.x * 0.82 + 0.2) - 0.5);
    float seam = 1.0 - smoothstep(0.006, 0.017, ring);
    base *= 1.0 - seam * 0.16;
    roughness = mix(roughness, 0.6, seam);
    if (uMaterial == 3) {
      vec2 uv = vec2(clamp((vLocal.x + 3.0) / 6.0, 0.0, 1.0), fract(atan(vLocal.z, vLocal.y) / (2.0 * PI) + 0.5));
      vec4 paint = texture(uPaint, uv);
      base = mix(base, pow(paint.rgb, vec3(2.2)), paint.a);
      roughness = mix(roughness, 0.72, paint.a);
    }
  } else if (uMaterial == 2) {
    // Decorative surface grain. It carries no new elevation or imagery data.
    float broad = noise(vWorld.xy * 0.0008);
    float grain = noise(vWorld.xy * 0.016) * 0.6 + noise(vWorld.xy * 0.08) * 0.4;
    float rock = 1.0 - smoothstep(0.66, 0.94, n.z);
    vec3 vegetation = pow(mix(vec3(0.18, 0.28, 0.12), vec3(0.36, 0.39, 0.22), broad), vec3(2.2));
    vec3 stone = pow(vec3(0.46, 0.43, 0.37), vec3(2.2));
    base = mix(base, vegetation, 0.35);
    base = mix(base, stone, rock * 0.65);
    base *= mix(0.80, 1.12, grain);
    roughness = 0.94;
  }
  float a = roughness * roughness, a2 = a * a;
  float denom = nh * nh * (a2 - 1.0) + 1.0;
  float distribution = a2 / max(PI * denom * denom, 0.0001);
  float k = (roughness + 1.0) * (roughness + 1.0) / 8.0;
  float visibility = (nv / (nv * (1.0 - k) + k)) * (nl / (nl * (1.0 - k) + k));
  vec3 fresnel = vec3(0.04) + vec3(0.96) * pow(1.0 - vh, 5.0);
  vec3 specular = distribution * visibility * fresnel / max(4.0 * nv * nl, 0.001);
  vec3 diffuse = (vec3(1.0) - fresnel) * base / PI;
  vec3 direct = (diffuse + specular) * vec3(3.1, 2.95, 2.65) * nl;
  float hemisphere = n.z * 0.5 + 0.5;
  vec3 ambient = mix(vec3(0.19, 0.18, 0.16), vec3(0.48, 0.61, 0.80), hemisphere);
  vec3 rim = vec3(0.22, 0.30, 0.42) * pow(1.0 - nv, 4.0) * uSpec;
  vec3 color = direct + base * ambient * (0.45 + uAmbient) + rim;
  color = mix(color, vCol * 2.0, uEmissive);
  float distanceToEye = length(uCamPos - vWorld);
  float fog = 1.0 - exp(-pow(distanceToEye * uFogDensity, 1.35));
  color = mix(color, uFogColor, clamp(fog, 0.0, 1.0) * (1.0 - uEmissive));
  frag = vec4(toneMap(color), uAlpha);
}`;

export const SKY_VS = `#version 300 es
layout(location=0) in vec2 aXY;
uniform vec3 uCamF, uCamR, uCamU;
uniform float uTanF, uAspect;
out vec3 vDir;
void main() {
  vDir = uCamF + uCamR * (aXY.x * uTanF * uAspect) + uCamU * (aXY.y * uTanF);
  gl_Position = vec4(aXY, 0.99999, 1.0);
}`;

export const SKY_FS = `#version 300 es
precision highp float;
uniform vec3 uSun;
in vec3 vDir;
out vec4 frag;
float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
float noise(vec2 p) {
  vec2 i = floor(p), f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x),
    mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
}
float clouds(vec2 p) {
  float value = 0.0, weight = 0.5;
  for (int i = 0; i < 4; i++) { value += noise(p) * weight; p = p * 2.03 + 7.6; weight *= 0.5; }
  return value;
}
void main() {
  vec3 direction = normalize(vDir);
  vec3 horizon = vec3(0.60, 0.70, 0.79);
  vec3 zenith = vec3(0.12, 0.28, 0.53);
  vec3 color = direction.z >= 0.0
    ? mix(horizon, zenith, pow(direction.z, 0.48))
    : mix(horizon, vec3(0.10, 0.14, 0.18), clamp(-direction.z * 5.0, 0.0, 1.0));
  if (direction.z > 0.02) {
    vec2 plane = direction.xy / (direction.z + 0.18) * 2.8;
    float cloud = smoothstep(0.47, 0.73, clouds(plane));
    cloud *= smoothstep(0.025, 0.18, direction.z);
    color = mix(color, vec3(0.90, 0.93, 0.96), cloud * 0.65);
  }
  float sun = max(dot(direction, uSun), 0.0);
  color += vec3(1.0, 0.85, 0.65) * pow(sun, 14.0) * 0.16;
  color += vec3(1.0, 0.94, 0.80) * pow(sun, 1800.0) * 4.0;
  color = clamp((color * (2.51 * color + 0.03)) /
    (color * (2.43 * color + 0.59) + 0.14), 0.0, 1.0);
  frag = vec4(pow(color, vec3(1.0 / 2.2)), 1.0);
}`;
