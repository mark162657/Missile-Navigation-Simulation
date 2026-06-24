"""
dynamics.py -- 3-DoF point-mass equations of motion + IMU synthesis.

WHAT IT DOES
    The bottom of the simulation stack. Given the current TRUE missile state, a
    control command, and a timestep, it:
        1. builds the total force (thrust + aerodynamics + gravity) on the
           point mass,
        2. integrates position/velocity forward one step with RK4,
        3. updates the (kinematically derived) attitude and mass,
        4. synthesises the IMU reading the navigation stack consumes.

WHAT IT OUTPUTS
    `step()` returns `(new_state, imu)`:
        - new_state : a NEW MissileState (true_* / vel_* / attitude / time
                      advanced; est_* and nav flags left untouched -- rule 6).
        - imu       : an IMUMeasurement (see below).

WHO CONSUMES IT
    - The simulation loop (Phase 4) calls `step` each tick.
    - navigation/ins.py consumes the IMUMeasurement: `imu.accel_enu` and
      `imu.angular_velocity` feed straight into `INS.predict(...)` with NO
      adapter.

IMPORTANT -- the IMU contract matches the EXISTING INS, not a textbook IMU.
    `INS.predict()` in this project integrates a NAV-FRAME (ENU) KINEMATIC
    acceleration directly into velocity (`self.vel += acc*dt`); it does not
    rotate body->nav and does not add gravity back. So the INS-facing field
    `imu.accel_enu` is the true ENU kinematic acceleration (gravity included),
    NOT gravity-free body-frame specific force. For realism and the eventual
    6-DoF/strapdown upgrade we ALSO expose `imu.specific_force_body` (the
    gravity-free body-frame accelerometer reading), but the current INS does
    not consume it.

    Noise ownership: dynamics emits CLEAN truth by default. The existing INS
    error model (`INS._corrupt_imu`) already adds bias + Gaussian noise, so an
    optional noise knob here defaults to OFF to avoid double-counting.

WIND (weather.py)
    Aerodynamic force depends on velocity RELATIVE TO THE AIR MASS, not ground
    velocity. A WindField supplies the local ENU wind each step; the aero/Mach/
    dynamic-pressure calc and the wind-axis frame key off air-relative velocity
        v_air = vel - v_wind
    while the kinematic (position) integration still uses ground `vel`. The
    wind is sampled ONCE per step and held constant across the RK4 sub-stages
    (so the turbulence filter advances exactly dt per step). Default is
    WindField.calm() -- zero wind, identical to the original behaviour.

Frame: ENU (East-North-Up), z positive up -- matches state.py / ins.py.
Coordinate convention for body axes: x forward, y right, z down.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

import numpy as np

from simulation.physics import atmosphere
from simulation.physics.aerodynamics import S_REF, BOOST_DRAG_CD, Aerodynamics
from simulation.physics.engine import Engine
from simulation.physics.weather import WindField
from missile.profile import MissileProfile
from missile.state import FlightStage, MissileState
from terrain import coordinates

if TYPE_CHECKING:
    from simulation.physics.sequencer import FlightSequencer


_EPS_SPEED = 1e-3  # m/s, below which aero/airflow direction is ill-defined


@dataclass
class ControlInput:
    """
    Autopilot output consumed by the physics (the only thing that drives it).

    Fields:
        throttle : engine throttle, 0..1 (clamped by the engine model)
        elevator : pitch fin deflection, rad (+ -> nose-up incidence)
        rudder   : yaw fin deflection, rad
        aileron  : roll fin deflection, rad (defined for 6-DoF; unused in 3-DoF)
    """
    throttle: float = 0.0
    elevator: float = 0.0
    rudder: float = 0.0
    aileron: float = 0.0


@dataclass
class IMUMeasurement:
    """
    Simulated strapdown IMU output for one step.

    INS-facing fields (consumed directly by INS.predict, no adapter):
        accel_enu        : ENU kinematic acceleration [a_east, a_north, a_up],
                           m/s^2 (gravity INCLUDED -- see module docstring).
        angular_velocity : body rates [roll_rate, pitch_rate, yaw_rate], rad/s.

    Realism / future-6-DoF field (NOT consumed by the current INS):
        specific_force_body : gravity-free accelerometer reading in body axes
                              [fx_fwd, fy_right, fz_down], m/s^2.

        time : simulation time stamp of the sample, s.
    """
    accel_enu: np.ndarray
    angular_velocity: np.ndarray
    specific_force_body: np.ndarray
    time: float = 0.0


class MissileDynamics:
    """
    3-DoF point-mass flight dynamics for one missile.

    Structured so the 6-DoF upgrade touches only this file and aerodynamics.py
    (rule 8): the force model, attitude bookkeeping and integrator live here;
    everything else (atmosphere, engine, state, INS) is unchanged.
    """

    def __init__(
        self,
        profile: MissileProfile,
        aerodynamics: Aerodynamics | None = None,
        engine: Engine | None = None,
        sequencer: "FlightSequencer | None" = None,
        wind: WindField | None = None,
        *,
        imu_accel_noise_std: float = 0.0,
        imu_gyro_noise_std: float = 0.0,
        rng: np.random.Generator | None = None,
    ) -> None:
        """
        Args:
            profile: missile profile (mass, fuel, etc.)
            aerodynamics / engine: optional injected models (defaults built
                from the profile).
            sequencer: optional FlightSequencer. When attached, it governs the
                boost stage (booster thrust + mass + programmed pitch-over) and
                the boost->cruise transition. When None, the missile is modelled
                purely in cruise (turbofan only) -- the original behaviour.
            wind: optional WindField (mean wind + shear + turbulence). When
                None, a calm (zero-wind) field is used and behaviour is
                identical to the original. Aero keys off air-relative velocity.
            imu_accel_noise_std: optional white noise on the IMU accel output,
                m/s^2. Default 0 -- the INS already adds its own noise.
            imu_gyro_noise_std: optional white noise on the IMU rate output,
                rad/s. Default 0.
            rng: optional numpy Generator for reproducible IMU noise.
        """
        self.profile = profile
        self.aero = aerodynamics if aerodynamics is not None else Aerodynamics()
        self.engine = engine if engine is not None else Engine(profile)
        self.sequencer = sequencer
        self.wind = wind if wind is not None else WindField.calm()

        # Mass model: profile mass_kg is the fully-fuelled launch mass.
        self.dry_mass_kg = float(profile.detailed.mass_kg) - self.engine.fuel_capacity_kg
        if self.dry_mass_kg <= 0.0:
            # Defensive: never let total mass go non-positive.
            self.dry_mass_kg = float(profile.detailed.mass_kg)

        self._accel_noise = float(imu_accel_noise_std)
        self._gyro_noise = float(imu_gyro_noise_std)
        self._rng = rng if rng is not None else np.random.default_rng()

    # ------------------------------------------------------------------
    @property
    def current_mass_kg(self) -> float:
        """Total mass = dry airframe + remaining fuel, kg."""
        return self.dry_mass_kg + self.engine.fuel_remaining_kg

    # ------------------------------------------------------------------
    # The single integration step called by the simulation loop (rule 4).
    # ------------------------------------------------------------------
    def step(
        self,
        state: MissileState,
        control: ControlInput,
        dt: float,
    ) -> tuple[MissileState, IMUMeasurement]:
        """
        Advance the TRUE state by one timestep and produce the IMU reading.

        Args:
            state:   current true MissileState
            control: autopilot command (fins + throttle)
            dt:      timestep, s

        Returns:
            (new_state, imu_measurement)
        """
        seq = self.sequencer
        boosting = seq is not None and seq.is_boosting

        # Vehicle mass (held constant across the RK4 sub-stages): the cruise
        # vehicle plus any still-attached booster.
        mass = self.current_mass_kg
        if seq is not None:
            mass += seq.attached_booster_mass()

        # State vector y = [lat, lon, alt, v_east, v_north, v_up].
        y0 = np.array([
            state.true_lat, state.true_lon, state.true_alt,
            state.vel_east, state.vel_north, state.vel_up,
        ], dtype=float)

        # Sample the wind field ONCE per step (held constant across the RK4
        # sub-stages); this advances the turbulence filter by exactly dt.
        wind_vel = self.wind.step(dt, state.true_alt, y0[3:6]).velocity_enu

        if boosting:
            # BOOST: thrust along the PROGRAMMED body axis, drag only (wings
            # folded), velocity follows; attitude is commanded, not derived.
            cmd_att = seq.commanded_attitude()            # (roll, pitch, yaw)
            body_x = _heading_pitch_to_unit(cmd_att[2], cmd_att[1])
            thrust = seq.booster_thrust()

            def deriv(y: np.ndarray) -> np.ndarray:
                lat, alt = y[0], y[2]
                vel = y[3:6]
                accel = self._accel_enu_boost(alt, vel, mass, thrust, body_x,
                                              wind_vel)
                m_lat = coordinates.meter_per_deg_lat(lat)
                m_lon = coordinates.meter_per_deg_lon_at(lat)
                return np.array([vel[1] / m_lat, vel[0] / m_lon, vel[2],
                                 accel[0], accel[1], accel[2]], dtype=float)

            accel0, sf_body0 = self._imu_boost(
                state.true_alt, y0[3:6], mass, thrust, body_x, wind_vel)
        else:
            # CRUISE: quasi-steady aero trim, velocity-derived attitude, turbofan.
            alpha = self.aero.trim_alpha(control.elevator)
            beta = self.aero.trim_beta(control.rudder)
            fallback_hat = _heading_pitch_to_unit(state.yaw, state.pitch)

            def deriv(y: np.ndarray) -> np.ndarray:
                lat, alt = y[0], y[2]
                vel = y[3:6]
                accel = self._accel_enu(alt, vel, mass, alpha, beta,
                                        control, fallback_hat, wind_vel)
                m_lat = coordinates.meter_per_deg_lat(lat)
                m_lon = coordinates.meter_per_deg_lon_at(lat)
                return np.array([vel[1] / m_lat, vel[0] / m_lon, vel[2],
                                 accel[0], accel[1], accel[2]], dtype=float)

            accel0, sf_body0 = self._imu_accelerations(
                state.true_alt, y0[3:6], mass, alpha, beta, control,
                fallback_hat, wind_vel)

        # --- RK4 integration (rule 7) ---
        k1 = deriv(y0)
        k2 = deriv(y0 + 0.5 * dt * k1)
        k3 = deriv(y0 + 0.5 * dt * k2)
        k4 = deriv(y0 + dt * k3)
        y_new = y0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        new_lat, new_lon, new_alt = y_new[0], y_new[1], y_new[2]
        new_vel = y_new[3:6]

        # --- attitude bookkeeping ---
        if boosting:
            new_roll, new_pitch, new_yaw = cmd_att  # commanded (programmed)
        else:
            # Airframe points along the AIR-relative velocity (+ AoA), not the
            # ground track: in a crosswind the nose is yawed into the wind.
            new_vel_air = new_vel - wind_vel
            new_yaw = _heading_from_velocity(new_vel_air)
            new_pitch = _gamma_from_velocity(new_vel_air) + alpha
            new_roll = 0.0  # 3-DoF point mass; 6-DoF will integrate roll.

        roll_rate = _wrap_pi(new_roll - state.roll) / dt
        pitch_rate = _wrap_pi(new_pitch - state.pitch) / dt
        yaw_rate = _wrap_pi(new_yaw - state.yaw) / dt
        ang_vel = np.array([roll_rate, pitch_rate, yaw_rate], dtype=float)

        # --- advance stage / fuel exactly once (outside the RK4 stages) ---
        if seq is not None:
            seq.advance(dt)  # burn booster, jettison + switch to CRUISE at burnout
        if not boosting:
            self.engine.consume_fuel(control.throttle, dt)

        # --- optional sensor noise (default off; INS adds its own) ---
        if self._accel_noise > 0.0:
            accel0 = accel0 + self._rng.normal(0.0, self._accel_noise, size=3)
            sf_body0 = sf_body0 + self._rng.normal(0.0, self._accel_noise, size=3)
        if self._gyro_noise > 0.0:
            ang_vel = ang_vel + self._rng.normal(0.0, self._gyro_noise, size=3)

        new_speed = float(np.linalg.norm(new_vel))
        new_state = replace(
            state,
            true_lat=float(new_lat),
            true_lon=float(new_lon),
            true_alt=float(new_alt),
            vel_east=float(new_vel[0]),
            vel_north=float(new_vel[1]),
            vel_up=float(new_vel[2]),
            roll=float(new_roll),
            pitch=float(new_pitch),
            yaw=float(new_yaw % (2.0 * math.pi)),
            time=state.time + dt,
            distance_traveled=state.distance_traveled + new_speed * dt,
        )

        imu = IMUMeasurement(
            accel_enu=accel0,
            angular_velocity=ang_vel,
            specific_force_body=sf_body0,
            time=new_state.time,
        )
        return new_state, imu

    # ------------------------------------------------------------------
    # Force model
    # ------------------------------------------------------------------
    def _wind_axes(
        self, vel: np.ndarray, alpha: float, beta: float, fallback_hat: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
        """
        Build the airflow unit vectors in ENU and the body x-axis.

        Returns (v_hat, lift_hat, side_hat, body_x_hat, speed). For x-forward /
        y-right / z-down body axes: body_y = side_hat, body_z = -lift_hat.
        """
        speed = float(np.linalg.norm(vel))
        if speed < _EPS_SPEED:
            v_hat = fallback_hat
        else:
            v_hat = vel / speed

        up = np.array([0.0, 0.0, 1.0])
        side = np.cross(v_hat, up)            # points to the right of velocity
        side_norm = float(np.linalg.norm(side))
        if side_norm < _EPS_SPEED:
            # Velocity nearly vertical: pick an arbitrary horizontal "right".
            side_hat = np.array([1.0, 0.0, 0.0])
        else:
            side_hat = side / side_norm
        lift_hat = np.cross(side_hat, v_hat)  # perpendicular to V, up-ish
        lift_hat /= float(np.linalg.norm(lift_hat))

        # Body x-axis = velocity rotated by incidence (small-angle decomposition).
        body_x = (math.cos(alpha) * math.cos(beta) * v_hat
                  + math.sin(alpha) * lift_hat
                  + math.sin(beta) * side_hat)
        body_x /= float(np.linalg.norm(body_x))
        return v_hat, lift_hat, side_hat, body_x, speed

    def _force_enu(
        self, alt: float, vel: np.ndarray, mass: float,
        alpha: float, beta: float, control: ControlInput,
        fallback_hat: np.ndarray, v_wind: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Total ENU force on the missile and the wind-axis frame.

        Returns (force_no_gravity_enu, gravity_enu, body_axes) where
        body_axes = (body_x_hat, side_hat, lift_hat). Aerodynamics key off the
        AIR-relative velocity v_air = vel - v_wind; thrust still acts along the
        body x-axis and the force is applied to the ground-frame momentum.
        """
        v_air = vel - v_wind
        v_hat, lift_hat, side_hat, body_x, speed = self._wind_axes(
            v_air, alpha, beta, fallback_hat
        )

        atm = atmosphere.sample(alt)
        mach = speed / atm.speed_of_sound if atm.speed_of_sound > 0 else 0.0
        q_dyn = 0.5 * atm.density * speed * speed

        aero = self.aero.compute(mach, alpha, beta, q_dyn, control.elevator)
        thrust = self.engine.thrust(control.throttle, alt)

        # Aerodynamic + propulsive force (everything except gravity).
        force_no_g = (thrust * body_x
                      + aero.lift * lift_hat
                      + aero.side_force * side_hat
                      - aero.drag * v_hat)
        gravity = np.array([0.0, 0.0, -mass * atmosphere.G0])
        return force_no_g, gravity, (body_x, side_hat, lift_hat)

    def _accel_enu(
        self, alt: float, vel: np.ndarray, mass: float,
        alpha: float, beta: float, control: ControlInput,
        fallback_hat: np.ndarray, v_wind: np.ndarray,
    ) -> np.ndarray:
        """Total ENU kinematic acceleration (gravity included), for RK4."""
        force_no_g, gravity, _ = self._force_enu(
            alt, vel, mass, alpha, beta, control, fallback_hat, v_wind
        )
        return (force_no_g + gravity) / mass

    def _imu_accelerations(
        self, alt: float, vel: np.ndarray, mass: float,
        alpha: float, beta: float, control: ControlInput,
        fallback_hat: np.ndarray, v_wind: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Compute both IMU accelerations for one sample:
          - accel_enu : ENU kinematic acceleration (gravity included) -> INS.
          - specific_force_body : gravity-free body-frame accelerometer reading.
        """
        force_no_g, gravity, (body_x, side_hat, lift_hat) = self._force_enu(
            alt, vel, mass, alpha, beta, control, fallback_hat, v_wind
        )
        accel_enu = (force_no_g + gravity) / mass

        # Specific force = sensed (gravity-free) acceleration, in ENU, then
        # projected onto body axes (x fwd, y right, z down = -lift).
        sf_enu = force_no_g / mass
        sf_body = np.array([
            float(np.dot(sf_enu, body_x)),
            float(np.dot(sf_enu, side_hat)),
            float(np.dot(sf_enu, -lift_hat)),
        ], dtype=float)
        return accel_enu, sf_body

    # ------------------------------------------------------------------
    # Boost force model (programmed attitude, booster thrust, drag only)
    # ------------------------------------------------------------------
    def _boost_force_no_g(
        self, alt: float, vel: np.ndarray, thrust: float, body_x: np.ndarray,
        v_wind: np.ndarray,
    ) -> np.ndarray:
        """Thrust (along the commanded body x) minus drag; ENU, gravity-free.

        Drag opposes the AIR-relative velocity v_air = vel - v_wind."""
        v_air = vel - v_wind
        speed = float(np.linalg.norm(v_air))
        v_hat = v_air / speed if speed > _EPS_SPEED else body_x
        atm = atmosphere.sample(alt)
        q_dyn = 0.5 * atm.density * speed * speed
        drag = q_dyn * S_REF * BOOST_DRAG_CD  # wings folded -> parasite drag only
        return thrust * body_x - drag * v_hat

    def _accel_enu_boost(
        self, alt: float, vel: np.ndarray, mass: float,
        thrust: float, body_x: np.ndarray, v_wind: np.ndarray,
    ) -> np.ndarray:
        """Total ENU kinematic acceleration during boost (gravity included)."""
        force_no_g = self._boost_force_no_g(alt, vel, thrust, body_x, v_wind)
        gravity = np.array([0.0, 0.0, -mass * atmosphere.G0])
        return (force_no_g + gravity) / mass

    def _imu_boost(
        self, alt: float, vel: np.ndarray, mass: float,
        thrust: float, body_x: np.ndarray, v_wind: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Boost IMU sample: (ENU kinematic accel, gravity-free body specific force)."""
        force_no_g = self._boost_force_no_g(alt, vel, thrust, body_x, v_wind)
        gravity = np.array([0.0, 0.0, -mass * atmosphere.G0])
        accel_enu = (force_no_g + gravity) / mass

        # Body axes built around the commanded forward direction.
        up = np.array([0.0, 0.0, 1.0])
        right = np.cross(body_x, up)
        rn = float(np.linalg.norm(right))
        right = right / rn if rn > _EPS_SPEED else np.array([1.0, 0.0, 0.0])
        lift = np.cross(right, body_x)
        lift /= float(np.linalg.norm(lift))

        sf_enu = force_no_g / mass
        sf_body = np.array([
            float(np.dot(sf_enu, body_x)),
            float(np.dot(sf_enu, right)),
            float(np.dot(sf_enu, -lift)),
        ], dtype=float)
        return accel_enu, sf_body


# ----------------------------------------------------------------------
# Small geometry helpers (ENU velocity <-> heading / flight-path angle)
# ----------------------------------------------------------------------
def _heading_from_velocity(vel: np.ndarray) -> float:
    """Heading (rad, clockwise from north) from ENU velocity [e, n, u]."""
    horiz = math.hypot(vel[0], vel[1])
    if horiz < _EPS_SPEED:
        return 0.0
    return math.atan2(vel[0], vel[1]) % (2.0 * math.pi)


def _gamma_from_velocity(vel: np.ndarray) -> float:
    """Flight-path angle (rad, + climbing) from ENU velocity [e, n, u]."""
    horiz = math.hypot(vel[0], vel[1])
    return math.atan2(vel[2], horiz)


def _heading_pitch_to_unit(yaw: float, pitch: float) -> np.ndarray:
    """ENU unit vector for a given heading/pitch (used as the V=0 fallback)."""
    return np.array([
        math.cos(pitch) * math.sin(yaw),   # east
        math.cos(pitch) * math.cos(yaw),   # north
        math.sin(pitch),                   # up
    ], dtype=float)


def _wrap_pi(angle: float) -> float:
    """Wrap an angle difference to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
