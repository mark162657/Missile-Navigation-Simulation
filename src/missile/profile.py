import math
from dataclasses import dataclass, field, asdict

import numpy as np

from missile.navigation.ins import INS


_KMH_TO_MS = 1.0 / 3.6
_G = 9.80665


@dataclass
class BasicSpec:
    """
    SECTION 1 — basic performance envelope.

    These are the fields the user is expected to fill in. They are the
    "publicly findable" specs of a missile (open-source figures, datasheets,
    news reporting). Every field is required; there are no defaults.

    Units (chosen to match how the public usually reports them):
        speeds        : km/h
        accelerations : m/s^2
        altitudes     : m (min/max are MSL ceiling/floor)
        cruise AGL    : m (height ABOVE GROUND, terrain-relative)
        g-forces      : g (multiples of 9.80665 m/s^2)
        turn rates    : deg/s
        range         : km

    Altitude vs. AGL:
        min_altitude / max_altitude  -> absolute envelope (MSL): the lowest and
            highest the missile may ever be, regardless of terrain.
        cruise_agl_min / cruise_agl_max -> the PREFERRED terrain-following band,
            measured as relative height above the ground directly below (e.g. a
            cruise missile likes to hug the terrain at ~20-50 m AGL). The
            absolute altitude this implies changes constantly as terrain rises
            and falls; only the height-above-ground stays in this band.
    """
    cruise_speed: float          # km/h
    min_speed: float             # km/h
    max_speed: float             # km/h
    max_acceleration: float      # m/s^2
    min_altitude: float          # m MSL (absolute floor)
    max_altitude: float          # m MSL (absolute ceiling)
    max_g_force: float           # g
    sustained_turn_rate: float   # deg/s
    sustained_g_force: float     # g
    evasive_turn_rate: float     # deg/s
    max_range: float             # km (max operational range)
    cruise_agl_min: float        # m AGL (preferred terrain-following floor)
    cruise_agl_max: float        # m AGL (preferred terrain-following ceiling)

    # Added after the initial schema -> carries a default so older configs and
    # hand-built specs still construct. Boost-phase AXIAL (longitudinal) g limit:
    # the booster accelerates the missile far harder than the cruise g-envelope.
    max_longitudinal_g_boost: float = 20.0   # g (boost-phase axial accel limit)

    # --- SI conversions used by the physics helpers ---
    @property
    def cruise_speed_ms(self) -> float:
        return self.cruise_speed * _KMH_TO_MS

    @property
    def min_speed_ms(self) -> float:
        return self.min_speed * _KMH_TO_MS

    @property
    def max_speed_ms(self) -> float:
        return self.max_speed * _KMH_TO_MS

    @property
    def sustained_turn_rate_rads(self) -> float:
        return math.radians(self.sustained_turn_rate)

    @property
    def max_longitudinal_accel_boost(self) -> float:
        """Boost-phase axial acceleration limit, m/s^2 (from the g value)."""
        return self.max_longitudinal_g_boost * _G

    @property
    def evasive_turn_rate_rads(self) -> float:
        return math.radians(self.evasive_turn_rate)


@dataclass
class DetailedSpec:
    """
    SECTION 2 — detailed / internal specs.

    These are harder to source publicly, so every field carries a sensible
    default (representative tactical-grade values). This section is closest to
    what the INS needs: it describes the inertial measurement unit's error
    behaviour, plus the navigation update rates.

    Units are kept human-readable (datasheet style) and converted to SI inside
    `create_ins()`:
        accel bias / noise / walk : m/s^2  (and m/s^2/sqrt(s) for walk)
        gyro bias                 : deg/hr
        gyro noise                : deg/s
        gyro bias walk            : deg/min/sqrt(s)
        update rates              : Hz
        mass                      : kg
        fuel capacity             : kg
        fuel burn rate            : kg/s (at cruise)
    """
    mass_kg: float = 1300.0
    imu_grade: str = "tactical"

    # Propulsion / endurance (used to model fuel-limited range)
    fuel_capacity_kg: float = 450.0
    fuel_burn_rate_kgps: float = 0.12

    # IMU error model (1-sigma magnitudes / spectral densities)
    accel_bias_sigma: float = 0.02            # m/s^2   (~2 mg turn-on bias)
    gyro_bias_sigma_dph: float = 5.0          # deg/hr  (turn-on bias)
    accel_noise_std: float = 0.05             # m/s^2   (white noise)
    gyro_noise_std_dps: float = 0.1           # deg/s   (white noise)
    accel_bias_walk_std: float = 1e-3         # m/s^2/sqrt(s) (in-run instability)
    gyro_bias_walk_std_dpm: float = 0.01      # deg/min/sqrt(s)

    # Navigation update rates
    ins_update_rate_hz: float = 500.0
    gps_update_rate_hz: float = 5.0
    tercom_update_rate_hz: float = 1.0


@dataclass
class BoosterSpec:
    """
    SECTION 3 — jettisonable solid-rocket boost motor.

    These describe the launch booster that accelerates the missile off the
    launcher up to turbofan-ignition speed, then separates. Like SECTION 2,
    every field has a representative (Tomahawk-class) default, so older config
    files that predate this section still load unchanged.

    This is the data half of the same pattern as DetailedSpec/create_ins(): the
    simulation's physics layer turns it into behaviour
    (simulation.physics.booster.SolidBooster, built by the FlightSequencer from
    this spec). Kept as plain data here so the dependency points one way
    (simulation -> missile), never the reverse.

    Units:
        booster_thrust_N   : N    (constant thrust while burning; solid motors
                                    are ~altitude/airspeed independent)
        burn_time_s        : s    (nominal burn duration)
        propellant_mass_kg : kg   (solid propellant burned over burn_time_s)
        casing_mass_kg     : kg   (inert structure jettisoned at burnout)
        launch_mode        : str  (default launch platform / boost attitude:
                                    "GROUND" | "SURFACE_VLS" | "SUBMARINE")
    """
    booster_thrust_N: float = 26690.0  # Mk 135 approx. 6,000 lbf
    burn_time_s: float = 12.0          # Mk 135 reported burn, below 15 s max
    propellant_mass_kg: float = 145.0
    casing_mass_kg: float = 150.0
    launch_mode: str = "SURFACE_VLS"

    @property
    def total_mass_kg(self) -> float:
        """Full booster mass at ignition (casing + propellant), kg."""
        return self.casing_mass_kg + self.propellant_mass_kg

    @property
    def burn_rate_kgps(self) -> float:
        """Steady propellant mass flow, kg/s."""
        if self.burn_time_s <= 0.0:
            return 0.0
        return self.propellant_mass_kg / self.burn_time_s


@dataclass
class WarheadSpec:
    """
    SECTION 4 — warhead / terminal effects.

    Consumed by the impact/detonation layer (simulation.impact + the result
    scoring) to decide the lethal outcome of an impact. Like the other non-basic
    sections it carries defaults, so older config files still load; the defaults
    describe the Tomahawk's WDU-36/B unitary warhead.

    Units:
        blast_radius_m : m  (effective lethal blast/frag radius; a target inside
                             this ground distance of the impact counts as killed)
    """
    warhead_name: str = "WDU-36/B"
    blast_radius_m: float = 40.0


@dataclass
class MissileProfile:
    """
    Full missile profile = SECTION 1 (basic) + SECTION 2 (detailed).

    The user only has to provide `basic`. `detailed` defaults to representative
    tactical-grade values and is mainly consumed by the INS. `booster` defaults
    to a representative boost motor and is consumed by the physics layer.
    """
    name: str
    basic: BasicSpec
    detailed: DetailedSpec = field(default_factory=DetailedSpec)
    booster: BoosterSpec = field(default_factory=BoosterSpec)
    warhead: WarheadSpec = field(default_factory=WarheadSpec)

    # ------------------------------------------------------------------
    # Construction from / to plain dicts (config_store / JSON bridge)
    # ------------------------------------------------------------------
    @classmethod
    def from_config(cls, config: dict) -> "MissileProfile":
        """
        Build a profile from a stored configuration dict.

        Accepts the layout {"basic": {...}, "detailed": {...}, "booster": {...}}.
        The detailed and booster sections are optional — missing keys fall back
        to defaults.

        NOTE: profiles loaded through config_store currently arrive without a
        booster section (config_store.validate_configuration keeps only
        name/basic/detailed), so the booster defaults apply on that path until
        config_store + the missile JSON files are extended.
        """
        basic = BasicSpec(**config["basic"])
        detailed_data = config.get("detailed") or {}
        detailed = DetailedSpec(**detailed_data)
        booster_data = config.get("booster") or {}
        booster = BoosterSpec(**booster_data)
        warhead_data = config.get("warhead") or {}
        warhead = WarheadSpec(**warhead_data)
        return cls(name=config["name"], basic=basic, detailed=detailed,
                   booster=booster, warhead=warhead)

    def to_config(self) -> dict:
        """Serialize back to a nested dict suitable for JSON storage."""
        return {
            "name": self.name,
            "basic": asdict(self.basic),
            "detailed": asdict(self.detailed),
            "booster": asdict(self.booster),
            "warhead": asdict(self.warhead),
        }

    # ------------------------------------------------------------------
    # INS factory — this is where SECTION 2 meets the navigation stack
    # ------------------------------------------------------------------
    def create_ins(
        self,
        init_pos: np.ndarray | list[float],
        init_vel: np.ndarray | list[float],
        init_att: np.ndarray | list[float] | None = None,
        rng: np.random.Generator | None = None,
    ) -> INS:
        """
        Build an INS configured from this profile's detailed IMU specs.

        Turn-on biases are sampled once from the 1-sigma magnitudes, so each
        constructed unit drifts differently (like INS.tactical_grade).

        Args:
            init_pos: initial [lat, lon, alt]
            init_vel: initial [vx east, vy north, vz up] in m/s
            init_att: initial [roll, pitch, yaw] in radians
            rng: optional Generator for reproducible error sampling
        """
        rng = rng if rng is not None else np.random.default_rng()
        d = self.detailed

        accel_bias = rng.normal(0.0, d.accel_bias_sigma, size=3)
        gyro_bias = rng.normal(0.0, math.radians(d.gyro_bias_sigma_dph) / 3600.0, size=3)

        return INS(
            init_pos,
            init_vel,
            init_att,
            accel_bias=accel_bias,
            gyro_bias=gyro_bias,
            accel_noise_std=d.accel_noise_std,
            gyro_noise_std=math.radians(d.gyro_noise_std_dps),
            accel_bias_walk_std=d.accel_bias_walk_std,
            gyro_bias_walk_std=math.radians(d.gyro_bias_walk_std_dpm) / 60.0,
            rng=rng,
        )

    # ------------------------------------------------------------------
    # Maneuver / performance helpers (operate in SI internally)
    # ------------------------------------------------------------------
    def calculate_turning_radius(self, speed_ms: float, turn_rate_rads: float) -> float:
        """
        Convert turn rate to turning radius: r = v / omega.

        Args:
            speed_ms: current speed in m/s
            turn_rate_rads: current turn rate in rad/s

        Return:
            turning radius in meters (inf if turn rate ~ 0)
        """
        if abs(turn_rate_rads) < 1e-6:
            return float("inf")
        return speed_ms / turn_rate_rads

    def min_turn_radius(self) -> float:
        """Tightest sustained turn radius at cruise speed, in meters."""
        return self.calculate_turning_radius(
            self.basic.cruise_speed_ms, self.basic.sustained_turn_rate_rads
        )

    def get_max_lateral_acceleration(self) -> float:
        """Max lateral acceleration from max g-force, in m/s^2."""
        return _G * self.basic.max_g_force

    # ------------------------------------------------------------------
    # Terrain-following helpers (operate on height ABOVE GROUND)
    # ------------------------------------------------------------------
    def preferred_agl(self) -> float:
        """Midpoint of the preferred terrain-following band, in m AGL."""
        return 0.5 * (self.basic.cruise_agl_min + self.basic.cruise_agl_max)

    def is_within_cruise_band(self, agl_m: float) -> bool:
        """True if a height-above-ground (m) is inside the preferred band."""
        return self.basic.cruise_agl_min <= agl_m <= self.basic.cruise_agl_max

    def clamp_to_cruise_band(self, agl_m: float) -> float:
        """Clamp a desired height-above-ground (m) into the preferred band."""
        return max(self.basic.cruise_agl_min, min(agl_m, self.basic.cruise_agl_max))

    def target_msl_altitude(self, ground_elevation_m: float) -> float:
        """
        Convert the preferred AGL band to an absolute (MSL) altitude target for
        the given ground elevation, then clamp to the absolute envelope.

        Args:
            ground_elevation_m: terrain height (MSL) directly below the missile
        """
        target = ground_elevation_m + self.preferred_agl()
        return max(self.basic.min_altitude, min(target, self.basic.max_altitude))

    def estimate_endurance_s(self) -> float:
        """Estimated powered flight time at cruise burn, in seconds."""
        if self.detailed.fuel_burn_rate_kgps <= 0:
            return float("inf")
        return self.detailed.fuel_capacity_kg / self.detailed.fuel_burn_rate_kgps

    def estimate_fuel_range_km(self) -> float:
        """
        Fuel-limited range estimate at cruise speed, in km.

        This is the range implied by Section 2 (fuel) and may differ from the
        publicly stated `basic.max_range`; the smaller of the two is the real
        constraint in a mission.
        """
        return self.basic.cruise_speed * (self.estimate_endurance_s() / 3600.0)

    def validate_maneuver(
        self, current_speed_ms: float, desired_speed_ms: float, turn_rate_rads: float
    ) -> bool:
        """Check a requested maneuver against the basic performance envelope (SI inputs)."""
        b = self.basic

        if not (b.min_speed_ms <= current_speed_ms <= b.max_speed_ms):
            return False

        if turn_rate_rads > b.evasive_turn_rate_rads:
            return False

        acceleration_required = abs(desired_speed_ms - current_speed_ms)
        if acceleration_required > b.max_acceleration:
            return False

        lateral_acceleration = desired_speed_ms * turn_rate_rads
        if lateral_acceleration > self.get_max_lateral_acceleration():
            return False

        return True

    def get_turn_rate_for_maneuver(self, maneuver_type: str) -> float:
        """Return the turn rate (rad/s) for a named maneuver type."""
        kind = maneuver_type.lower()
        if kind == "manual":
            return self.basic.sustained_turn_rate_rads
        elif kind == "evasive":
            return self.basic.evasive_turn_rate_rads
        else:
            raise ValueError("Unknown maneuver type")
