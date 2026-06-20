"""
engine.py -- turbofan cruise propulsion model.

WHAT IT DOES
    Models the sustainer turbofan of a cruise missile:
        - thrust as a function of throttle (0..1) and altitude (density lapse)
        - fuel burn rate as a function of throttle
        - remaining-fuel bookkeeping (cuts thrust to zero when dry)

WHAT IT OUTPUTS
    - thrust(...)            -> instantaneous thrust in N (pure, no state change)
    - consume_fuel(...)      -> fuel burned this step in kg (mutates remaining)
    - fuel_remaining_kg      -> current fuel mass, kg
    These let dynamics.py keep thrust evaluation pure inside the RK4 stages
    while advancing the fuel state exactly once per integration step.

WHO CONSUMES IT
    - dynamics.py : adds thrust along the body x-axis and updates total mass
                    (dry airframe + remaining fuel) as fuel burns.

Mass/fuel convention (chosen here; MissileState has no mass field by design):
    The profile's `mass_kg` is treated as the FULLY FUELLED launch mass, so
        dry_mass = mass_kg - fuel_capacity_kg
    and current mass = dry_mass + fuel_remaining_kg. The dynamics layer owns
    this running mass; nothing is written back into MissileState.

Numbers below are representative of an F107-class small turbofan. Exact values
are not critical (rule 7).
"""

from __future__ import annotations

from missile.physics.atmosphere import RHO0_ISA, density
from missile.profile import MissileProfile


# ----------------------------------------------------------------------
# Engine constants (representative small turbofan, e.g. F107)
# ----------------------------------------------------------------------
SEA_LEVEL_MAX_THRUST = 3100.0   # max thrust at sea level, full throttle, N
THRUST_LAPSE_EXP = 0.7          # thrust ~ (rho/rho0) ** exp  (altitude lapse)

# Throttle at which the profile's quoted cruise fuel burn applies. Used to
# scale the burn law so throttle=CRUISE_THROTTLE reproduces fuel_burn_rate_kgps.
CRUISE_THROTTLE = 0.6


class Engine:
    """
    Turbofan sustainer for one missile, built from its MissileProfile.

    Tracks remaining fuel internally. Thrust evaluation is a pure function of
    (throttle, altitude); fuel is advanced separately via `consume_fuel` so the
    RK4 integrator can call `thrust` at several sub-stages without side effects.
    """

    def __init__(self, profile: MissileProfile) -> None:
        d = profile.detailed
        self.fuel_capacity_kg = float(d.fuel_capacity_kg)
        self.fuel_remaining_kg = float(d.fuel_capacity_kg)
        # Cruise burn rate (kg/s) quoted at CRUISE_THROTTLE; scaled linearly.
        self._cruise_burn_kgps = float(d.fuel_burn_rate_kgps)

    # --- thrust (pure) ---------------------------------------------------
    def thrust(self, throttle: float, altitude_m: float) -> float:
        """
        Thrust (N) for a throttle setting (0..1) at the given altitude.

        Returns 0 when out of fuel. Altitude reduces thrust through the air
        density ratio raised to THRUST_LAPSE_EXP.
        """
        if self.fuel_remaining_kg <= 0.0:
            return 0.0
        throttle = max(0.0, min(throttle, 1.0))
        lapse = (density(altitude_m) / RHO0_ISA) ** THRUST_LAPSE_EXP
        return SEA_LEVEL_MAX_THRUST * throttle * lapse

    # --- fuel bookkeeping (mutates) -------------------------------------
    def fuel_burn_rate(self, throttle: float) -> float:
        """Fuel mass flow (kg/s) for a throttle setting (0 if dry)."""
        if self.fuel_remaining_kg <= 0.0:
            return 0.0
        throttle = max(0.0, min(throttle, 1.0))
        return self._cruise_burn_kgps * (throttle / CRUISE_THROTTLE)

    def consume_fuel(self, throttle: float, dt: float) -> float:
        """
        Advance the fuel state by one step and return the fuel burned (kg).

        Call exactly once per integration step (not inside RK4 sub-stages).
        """
        burned = min(self.fuel_burn_rate(throttle) * dt, self.fuel_remaining_kg)
        self.fuel_remaining_kg -= burned
        return burned

    @property
    def is_out_of_fuel(self) -> bool:
        return self.fuel_remaining_kg <= 0.0
