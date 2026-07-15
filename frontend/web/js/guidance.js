// Client-side mirror of the terminal-guidance geometry, so the planner can show
// where the missile will pull up into its terminal dive *before* launch.
//
// Everything here mirrors src/missile/guidance/terminal_guidance.py:
//   terminal_init_range()  →  r = R_SIZE_FACTOR · 2·V_cruise²·|sin θ_impact| / a_max
//   a_max                  =  g · max_g_force   (profile.get_max_lateral_acceleration)
//   V_cruise               =  cruise_speed (km/h) · KMH_TO_MS
// Keep the constants in sync with the backend if the guidance law changes.

const G = 9.80665;                 // m/s²
const KMH_TO_MS = 1000 / 3600;     // matches profile._KMH_TO_MS
const R_SIZE_FACTOR = 3.0;         // TerminalGuidance default terminal_dist_size_factor
const EARTH_R = 6371000;           // m, for the ground-distance walk

// Impact-angle envelope (magnitudes, degrees). MAX/MIN are the hard clamps
// enforced by the backend (terminal_guidance._MAX/_MIN_IMPACT_ANGLE_DEG): too
// steep stalls, too shallow skims/deviates. Beyond RECOMMENDED the dive is
// achievable but energy-marginal, so the UI flags it.
export const MAX_IMPACT_DEG = 55;
export const MIN_IMPACT_DEG = 5;
export const RECOMMENDED_IMPACT_DEG = 45;

// Terminal engage / pull-up range from the target, in meters. Returns null if
// the inputs can't produce a valid range.
export function terminalInitRange(cruiseKmh, maxG, impactDeg) {
  const v = Number(cruiseKmh) * KMH_TO_MS;
  const aMax = G * Number(maxG);
  if (!(v > 0) || !(aMax > 0) || !Number.isFinite(impactDeg)) return null;
  const rMin = (2 * v * v * Math.abs(Math.sin((impactDeg * Math.PI) / 180))) / aMax;
  return R_SIZE_FACTOR * rMin;
}

// Great-circle ground distance in meters. Altitude is ignored: the init range is
// kilometers while missile↔target altitude differs by tens of meters, so the
// horizontal distance tracks the backend's direct_3d_distance to well under a pixel.
function haversine(la1, lo1, la2, lo2) {
  const toRad = Math.PI / 180;
  const dLa = (la2 - la1) * toRad;
  const dLo = (lo2 - lo1) * toRad;
  const a = Math.sin(dLa / 2) ** 2 +
    Math.cos(la1 * toRad) * Math.cos(la2 * toRad) * Math.sin(dLo / 2) ** 2;
  return 2 * EARTH_R * Math.asin(Math.min(1, Math.sqrt(a)));
}

// Where along the planned route terminal guidance engages: the first point,
// flying launch→target, whose distance to the target drops to the init range.
// Returns [lat, lon] or null (missing inputs, or the route never gets that close).
export function terminalPullupGps(cruiseKmh, maxG, impactDeg, trajectory, targetGps) {
  if (!trajectory || trajectory.length < 1 || !targetGps) return null;
  const range = terminalInitRange(cruiseKmh, maxG, impactDeg);
  if (range == null) return null;
  const [tlat, tlon] = targetGps;

  // Returns [lat, lon, elev]: the elevation (trajectory column 2) is interpolated
  // too, so the 3D viewer can place the marker on the terrain. The 2D map reads
  // only [0] and [1], so the extra element is harmless there.
  let prev = null, prevD = 0;
  for (const p of trajectory) {
    const d = haversine(p[0], p[1], tlat, tlon);
    if (d <= range) {
      if (!prev) return [p[0], p[1], p[2] ?? 0];   // route starts already inside the range
      // Linear-interpolate the crossing between prev (outside) and p (inside).
      const t = (prevD - range) / (prevD - d || 1);
      const lerp = (a, b) => a + (b - a) * t;
      return [lerp(prev[0], p[0]), lerp(prev[1], p[1]), lerp(prev[2] ?? 0, p[2] ?? 0)];
    }
    prev = p; prevD = d;
  }
  return null; // never comes within the engage range
}
