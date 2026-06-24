"""
weather.py -- wind field model: mean wind + shear + turbulence (gusts).

WHAT IT DOES
    Produces the local WIND VELOCITY the airframe flies through, in ENU
    (East-North-Up) m/s, as a function of altitude and time. It is the
    "moving air mass" the missile sees. Wind is decomposed -- the way real
    flight simulators do it -- into separable physical layers:

        1. Mean wind     : large-scale, slowly varying with altitude. A
                           log-law boundary-layer profile (deterministic).
        2. Wind shear    : the gradient of the mean profile with altitude
                           (falls straight out of layer 1; no extra model).
        3. Turbulence    : small-scale stochastic gusts. A Dryden model
                           (MIL-F-8785C / MIL-STD-1797): white noise shaped
                           by a filter so the gust is RANDOM but TEMPORALLY
                           CORRELATED -- it cannot jump discontinuously
                           second-to-second, which is exactly the realism we
                           want. Seed-driven, so runs are reproducible.
        4. Discrete gust : optional one-off "1-cosine" gust for scripted
                           worst-case events.

WHAT IT OUTPUTS
    `WindField.step(dt, altitude_m, velocity_enu)` -> WindSample:
        velocity_enu : total wind velocity [east, north, up], m/s
        mean_enu     : the deterministic mean component (layers 1-2)
        gust_enu     : the stochastic turbulence + discrete-gust component
    `WindField.sample_mean(altitude_m)` is a STATELESS query of layers 1-2
    (no RNG advance) for plotting / analysis.

WHO CONSUMES IT
    - dynamics.py (future hookup): aerodynamic force depends on velocity
      RELATIVE TO THE AIR MASS, not ground velocity. The single insertion
      point is the airspeed:

          v_air_enu = velocity_enu - wind.velocity_enu      (all ENU)

      and the aero/Mach/dynamic-pressure calc keys off `v_air_enu` while the
      kinematic integration still uses ground `velocity_enu`. This module
      does NOT reach into dynamics; dynamics pulls a wind sample each step.

DESIGN -- truth-side only (rule 6)
    Wind is part of the simulated REAL WORLD (the "plant"). It is NOT visible
    to navigation/guidance/control except through its effect on the true
    trajectory and hence the IMU. This module never imports those.

Frame: ENU, z positive up -- matches atmosphere.py / dynamics.py / state.py.
Direction convention: `direction_from_deg` is METEOROLOGICAL -- the bearing
(clockwise from north) the wind blows FROM. A "270 deg" (westerly) wind has a
velocity vector pointing EAST.

Turbulence fidelity (rule 7: representative, structure matters)
    Each axis is the EXACT discretization of a first-order Dryden / Ornstein-
    Uhlenbeck shaping filter (the Dryden u-channel form), driven by the
    per-axis intensity and length scale from the low-altitude MIL-F-8785C
    model. The lateral/vertical channels have a second-order Dryden form in
    the full spec; the first-order approximation here is unconditionally
    stable, gives the correct correlation structure, and leaves an obvious
    upgrade hook. Numbers are representative; the layered structure is the
    point.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


# ----------------------------------------------------------------------
# Unit helpers / constants
# ----------------------------------------------------------------------
_FT_PER_M = 3.280839895          # feet per meter
_KNOT_MS = 0.514444             # m/s per knot
_EPS_SPEED = 1e-3               # m/s, below which flow direction is ill-defined
_EPS_ALT = 1e-3                 # m, floor for log-law / Dryden altitude terms

# MIL-F-8785C turbulence severity, parametrised by W20 = the wind speed at
# 20 ft (6.1 m) used as the low-altitude turbulence-intensity driver.
W20_LIGHT = 15.0 * _KNOT_MS     # ~7.7 m/s
W20_MODERATE = 30.0 * _KNOT_MS  # ~15.4 m/s
W20_SEVERE = 45.0 * _KNOT_MS    # ~23.2 m/s

# High-altitude (> ~600 m) Dryden length scale, constant in the spec.
_L_HIGH_M = 1750.0 / _FT_PER_M  # 1750 ft -> ~533 m
# Altitude band over which the low-altitude formulas apply / blend out.
_H_LOW_M = 1000.0 / _FT_PER_M   # 1000 ft -> ~305 m


@dataclass(frozen=True)
class WindSample:
    """Local wind velocity at one point/time (ENU, m/s)."""
    velocity_enu: np.ndarray   # total wind = mean + gust
    mean_enu: np.ndarray       # deterministic mean + shear component
    gust_enu: np.ndarray       # stochastic turbulence + discrete gust


# ======================================================================
# Layer 1-2: mean wind profile (deterministic) + shear
# ======================================================================
class MeanWindProfile:
    """
    Log-law atmospheric boundary-layer mean wind.

        V(h) = V_ref * ln(h / z0) / ln(h_ref / z0)

    Speed grows with altitude from the surface; its altitude derivative IS the
    wind shear (layer 2), so no separate shear model is needed. Direction is
    held constant with altitude here (a veer/backing term could be added later
    without touching callers).
    """

    def __init__(
        self,
        speed_ref: float,
        direction_from_deg: float,
        *,
        ref_height_m: float = 10.0,
        roughness_z0_m: float = 0.03,
        cap_speed: float | None = None,
    ) -> None:
        """
        Args:
            speed_ref: mean wind speed at `ref_height_m`, m/s.
            direction_from_deg: bearing the wind blows FROM (deg, met convention).
            ref_height_m: reference height for `speed_ref`, m (default 10 m).
            roughness_z0_m: surface roughness length, m (0.03 ~ open grassland,
                0.0002 ~ open sea, 0.3 ~ suburban).
            cap_speed: optional ceiling on mean speed, m/s (None = uncapped).
        """
        self.speed_ref = float(speed_ref)
        self.ref_height_m = max(float(ref_height_m), roughness_z0_m * math.e)
        self.roughness_z0_m = max(float(roughness_z0_m), _EPS_ALT)
        self.cap_speed = cap_speed
        # Wind-FROM bearing -> velocity heading is +180 deg.
        theta = math.radians(direction_from_deg)
        self._dir_hat = np.array([-math.sin(theta), -math.cos(theta), 0.0])
        self._log_ref = math.log(self.ref_height_m / self.roughness_z0_m)

    def speed(self, altitude_m: float) -> float:
        """Mean wind speed (m/s) at altitude (m AGL/MSL -- treated as height)."""
        h = max(float(altitude_m), self.roughness_z0_m)
        v = self.speed_ref * math.log(h / self.roughness_z0_m) / self._log_ref
        v = max(v, 0.0)
        if self.cap_speed is not None:
            v = min(v, self.cap_speed)
        return v

    def velocity_enu(self, altitude_m: float) -> np.ndarray:
        """Mean wind velocity vector [east, north, up] (m/s) at altitude."""
        return self.speed(altitude_m) * self._dir_hat


# ======================================================================
# Layer 3: Dryden turbulence (stochastic, stateful, seeded)
# ======================================================================
class DrydenTurbulence:
    """
    Dryden continuous-gust turbulence (MIL-F-8785C), low-altitude model.

    STATEFUL: holds the three gust components (along-wind u, cross-wind v,
    vertical w) between steps. Each is the exact discretization of a
    first-order shaping filter:

        g[k+1] = a * g[k] + sigma * sqrt(1 - a^2) * N(0, 1),   a = exp(-dt / tau)

    with time constant tau = L / V_air (gust length scale over airspeed). As
    airspeed -> 0 the filter freezes (tau -> inf, a -> 1): a parked vehicle
    sees a steady gust, not white noise. Output is rotated into ENU about the
    horizontal flight direction (u along track, v to the right, w up).
    """

    def __init__(
        self,
        w20: float = W20_MODERATE,
        *,
        rng: np.random.Generator | None = None,
        seed: int | None = None,
    ) -> None:
        """
        Args:
            w20: turbulence intensity driver = wind speed at 20 ft, m/s. Use the
                W20_* presets (light/moderate/severe) or any value.
            rng: optional numpy Generator (shared with the rest of the sim for
                one reproducible stream).
            seed: convenience seed if no `rng` is supplied.
        """
        self.w20 = float(w20)
        if rng is not None:
            self._rng = rng
        else:
            self._rng = np.random.default_rng(seed)
        self._gust = np.zeros(3)   # [u, v, w] in wind axes, m/s

    def reset(self) -> None:
        """Zero the gust state (e.g. between Monte-Carlo runs)."""
        self._gust = np.zeros(3)

    # -- MIL-F-8785C low-altitude intensities & length scales ----------
    def _sigma_and_length(self, altitude_m: float) -> tuple[np.ndarray, np.ndarray]:
        """
        (sigma[u,v,w] in m/s, length[u,v,w] in m) at the given altitude.

        Low-altitude formulas (h in feet) blended into the constant high-
        altitude values above ~1000 ft so the model is continuous everywhere.
        """
        h_ft = max(float(altitude_m), _EPS_ALT) * _FT_PER_M
        h_ft_low = min(h_ft, 1000.0)  # low-alt formulas defined up to 1000 ft

        denom = (0.177 + 0.000823 * h_ft_low)
        # Vertical gust intensity scales with W20; horizontal are larger near
        # the ground (anisotropic boundary-layer turbulence).
        sigma_w = 0.1 * self.w20
        sigma_u = sigma_w / (denom ** 0.4)
        sigma_v = sigma_u

        # Length scales (ft) -> meters.
        l_w_ft = h_ft_low
        l_u_ft = h_ft_low / (denom ** 1.2)
        l_v_ft = l_u_ft

        # Blend to the isotropic high-altitude regime above the low band.
        if altitude_m > _H_LOW_M:
            frac = min((altitude_m - _H_LOW_M) / _H_LOW_M, 1.0)
            l_high = _L_HIGH_M
            l_u_ft = l_u_ft * (1 - frac) + l_high * _FT_PER_M * frac
            l_v_ft = l_v_ft * (1 - frac) + l_high * _FT_PER_M * frac
            l_w_ft = l_w_ft * (1 - frac) + l_high * _FT_PER_M * frac
            sig_high = sigma_w  # isotropic at altitude
            sigma_u = sigma_u * (1 - frac) + sig_high * frac
            sigma_v = sigma_v * (1 - frac) + sig_high * frac

        sigma = np.array([sigma_u, sigma_v, sigma_w])
        length = np.array([l_u_ft, l_v_ft, l_w_ft]) / _FT_PER_M
        return sigma, length

    def step(
        self, dt: float, altitude_m: float, velocity_enu: np.ndarray,
        mean_dir_hat: np.ndarray | None = None,
    ) -> np.ndarray:
        """
        Advance the gust filters one step and return the gust in ENU (m/s).

        Args:
            dt: timestep, s.
            altitude_m: current altitude (height), m.
            velocity_enu: airspeed-defining velocity [e, n, u], m/s. Used for
                both the filter time constant (tau = L / |V|) and the axes the
                gust components map onto. Pass ground velocity if airspeed is
                not yet available; the difference is second order.
            mean_dir_hat: optional unit vector to orient the gust along when the
                vehicle is nearly stationary (e.g. the mean-wind direction).
        """
        v_air = float(np.linalg.norm(velocity_enu))
        sigma, length = self._sigma_and_length(altitude_m)

        if v_air < _EPS_SPEED:
            # No relative wind: freeze the filters (no decorrelation), keep gust.
            u_hat, v_hat, w_hat = self._axes(velocity_enu, mean_dir_hat)
            return (self._gust[0] * u_hat + self._gust[1] * v_hat
                    + self._gust[2] * w_hat)

        tau = length / v_air                      # per-axis time constant, s
        a = np.exp(-dt / np.maximum(tau, _EPS_ALT))
        noise = self._rng.standard_normal(3)
        self._gust = a * self._gust + sigma * np.sqrt(1.0 - a * a) * noise

        u_hat, v_hat, w_hat = self._axes(velocity_enu, mean_dir_hat)
        return (self._gust[0] * u_hat + self._gust[1] * v_hat
                + self._gust[2] * w_hat)

    @staticmethod
    def _axes(
        velocity_enu: np.ndarray, mean_dir_hat: np.ndarray | None
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Wind-axis triad in ENU: u_hat along horizontal track, v_hat to the
        right of it, w_hat straight up.
        """
        horiz = np.array([velocity_enu[0], velocity_enu[1], 0.0])
        hn = float(np.linalg.norm(horiz))
        if hn > _EPS_SPEED:
            u_hat = horiz / hn
        elif mean_dir_hat is not None:
            u_hat = np.array([mean_dir_hat[0], mean_dir_hat[1], 0.0])
            n = float(np.linalg.norm(u_hat))
            u_hat = u_hat / n if n > _EPS_SPEED else np.array([0.0, 1.0, 0.0])
        else:
            u_hat = np.array([0.0, 1.0, 0.0])   # default: along north
        w_hat = np.array([0.0, 0.0, 1.0])
        v_hat = np.cross(w_hat, u_hat)          # to the right of track
        return u_hat, v_hat, w_hat


# ======================================================================
# Layer 4: optional discrete "1-cosine" gust
# ======================================================================
@dataclass
class DiscreteGust:
    """
    MIL-spec "1-cosine" discrete gust: a single smooth bump.

        V(t) = (A/2) * (1 - cos(pi * (t - t0) / T))   for t0 <= t <= t0 + 2T

    Adds a clean, scripted, repeatable upset on top of the stochastic field.
    """
    amplitude: float                       # peak gust speed, m/s
    start_time: float                      # t0, s
    duration: float                        # 2T, s (full bump length)
    direction_enu: np.ndarray              # unit vector the gust pushes along

    def velocity_enu(self, t: float) -> np.ndarray:
        """Gust velocity (ENU, m/s) at simulation time `t`."""
        if t < self.start_time or t > self.start_time + self.duration:
            return np.zeros(3)
        phase = math.pi * (t - self.start_time) / (0.5 * self.duration)
        mag = 0.5 * self.amplitude * (1.0 - math.cos(phase))
        return mag * self.direction_enu


# ======================================================================
# Top-level field: combine all layers
# ======================================================================
class WindField:
    """
    The single object dynamics.py talks to. Combines the mean profile, Dryden
    turbulence and any discrete gusts into one ENU wind velocity per step.

    Stateful only through the turbulence filter and the simulation clock; call
    `step` once per integration tick.
    """

    def __init__(
        self,
        mean: MeanWindProfile | None = None,
        turbulence: DrydenTurbulence | None = None,
        gusts: list[DiscreteGust] | None = None,
    ) -> None:
        self.mean = mean
        self.turbulence = turbulence
        self.gusts = list(gusts) if gusts else []
        self._time = 0.0

    # -- construction helper ------------------------------------------
    @classmethod
    def calm(cls) -> "WindField":
        """A no-wind field (all layers zero). Useful as a default / baseline."""
        return cls()

    @classmethod
    def preset(
        cls,
        speed_ref: float,
        direction_from_deg: float,
        *,
        w20: float = W20_MODERATE,
        ref_height_m: float = 10.0,
        roughness_z0_m: float = 0.03,
        rng: np.random.Generator | None = None,
        seed: int | None = None,
    ) -> "WindField":
        """Build a mean-profile + Dryden-turbulence field in one call."""
        mean = MeanWindProfile(
            speed_ref, direction_from_deg,
            ref_height_m=ref_height_m, roughness_z0_m=roughness_z0_m,
        )
        turb = DrydenTurbulence(w20=w20, rng=rng, seed=seed)
        return cls(mean=mean, turbulence=turb)

    # -- stateless query (no RNG advance) -----------------------------
    def sample_mean(self, altitude_m: float) -> np.ndarray:
        """Mean (+ shear) wind velocity at altitude, ENU m/s. No state change."""
        if self.mean is None:
            return np.zeros(3)
        return self.mean.velocity_enu(altitude_m)

    # -- per-step advance ---------------------------------------------
    def step(
        self, dt: float, altitude_m: float, velocity_enu: np.ndarray,
    ) -> WindSample:
        """
        Advance the wind field by `dt` and return the total wind at the
        vehicle's altitude.

        Args:
            dt: timestep, s.
            altitude_m: vehicle altitude (height), m.
            velocity_enu: vehicle velocity [e, n, u], m/s (sets airspeed/axes).
        """
        self._time += dt

        mean = self.sample_mean(altitude_m)
        mean_dir = self.mean._dir_hat if self.mean is not None else None

        gust = np.zeros(3)
        if self.turbulence is not None:
            gust = gust + self.turbulence.step(
                dt, altitude_m, velocity_enu, mean_dir_hat=mean_dir
            )
        for g in self.gusts:
            gust = gust + g.velocity_enu(self._time)

        return WindSample(
            velocity_enu=mean + gust, mean_enu=mean, gust_enu=gust,
        )
