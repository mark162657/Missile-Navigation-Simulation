"""
booster.py -- solid-rocket boost motor model.

WHAT IT DOES
    Models the jettisonable solid-rocket booster that accelerates a cruise
    missile off the launcher and up to turbofan-ignition speed (~0.5 Mach).
    Unlike the airbreathing turbofan in engine.py, a solid motor:
        - produces high thrust that is ~independent of altitude and airspeed,
        - burns for a short, fixed duration,
        - carries its own propellant (burned) plus a casing (jettisoned at
          burnout -> a step decrease in vehicle mass).

WHAT IT OUTPUTS
    - thrust()            -> current booster thrust, N (0 once burnt out/separated)
    - consume(dt)         -> propellant burned this step, kg (mutates remaining)
    - total_mass          -> casing + remaining propellant, kg (0 after separate())
    - is_burnt_out / separate()

WHO CONSUMES IT
    - sequencer.py : the FlightSequencer owns one SolidBooster, advances its
                     burn, and jettisons it at burnout.
    - dynamics.py  : (indirectly, via the sequencer) adds booster thrust along
                     the commanded body axis and includes the booster mass.

The booster PARAMETERS live in missile.profile.BoosterSpec (the data half of
the same pattern as DetailedSpec/INS); this module is the behaviour half.
Build one from a profile's spec: `SolidBooster(profile.booster)` (the
FlightSequencer does this for you).
"""

from __future__ import annotations

# BoosterSpec lives in the profile (data); re-exported here for convenience so
# `from simulation.physics.booster import BoosterSpec` keeps working.
from missile.profile import BoosterSpec


class SolidBooster:
    """One jettisonable solid-rocket booster. Stateful (propellant + attach)."""

    def __init__(self, spec: BoosterSpec | None = None) -> None:
        self.spec = spec if spec is not None else BoosterSpec()
        self.propellant_remaining_kg = float(self.spec.propellant_mass_kg)
        self.separated = False

    # --- thrust (pure) ---------------------------------------------------
    def thrust(self) -> float:
        """Current thrust, N. Zero once separated or out of propellant.

        Solid-motor thrust is modelled as constant and altitude-independent
        (no density lapse, unlike the turbofan).
        """
        if self.separated or self.propellant_remaining_kg <= 0.0:
            return 0.0 # 0 once separated or out of propellant
        return self.spec.booster_thrust_N

    # --- propellant / mass ----------------------------------------------
    def burn_rate_kgps(self) -> float:
        """Steady propellant mass flow, kg/s (propellant / burn time)."""
        return self.spec.burn_rate_kgps

    def consume(self, dt: float) -> float:
        """Burn propellant for one step; return mass burned (kg)."""
        if self.separated:
            return 0.0
        burned = min(self.burn_rate_kgps() * dt, self.propellant_remaining_kg)
        self.propellant_remaining_kg -= burned
        return burned

    @property
    def total_mass(self) -> float:
        """Mass the booster still adds to the vehicle (0 after separation)."""
        if self.separated:
            return 0.0
        return self.spec.casing_mass_kg + self.propellant_remaining_kg

    @property
    def is_burnt_out(self) -> bool:
        return self.propellant_remaining_kg <= 0.0

    def separate(self) -> None:
        """Jettison the booster casing (irreversible)."""
        self.separated = True
