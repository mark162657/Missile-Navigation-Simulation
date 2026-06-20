"""
atmosphere.py -- International Standard Atmosphere (ISA) model.

WHAT IT DOES
    Given a geometric altitude (meters MSL) it returns the local air
    temperature, pressure, density and speed of sound, using the ISA layers
    (troposphere lapse 0-11 km, isothermal lower stratosphere 11-20 km).

WHAT IT OUTPUTS
    Scalar floats, or an `AtmosphereSample` bundling all four. SI units:
        temperature      : K
        pressure         : Pa
        density          : kg/m^3
        speed_of_sound   : m/s

WHO CONSUMES IT
    - aerodynamics.py : needs density (dynamic pressure) and speed of sound
                        (Mach number).
    - engine.py       : needs density to model thrust lapse with altitude.
    - dynamics.py     : imports the shared physical constants (G0 gravity,
                        etc.) defined here.

This file is the single home for the universal physical constants (rule 9).
Airframe-specific numbers (reference area, aero coefficients) live with the
module that owns them (aerodynamics.py), not here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# ----------------------------------------------------------------------
# Universal physical constants (single source of truth -- rule 9)
# ----------------------------------------------------------------------
G0 = 9.80665          # standard gravity, m/s^2 (matches profile._G)
R_AIR = 287.0528      # specific gas constant for dry air, J/(kg*K)
GAMMA_AIR = 1.4       # ratio of specific heats for air (dimensionless)

# ISA sea-level reference values
T0_ISA = 288.15       # sea-level standard temperature, K
P0_ISA = 101325.0     # sea-level standard pressure, Pa
RHO0_ISA = 1.225      # sea-level standard density, kg/m^3

# ISA troposphere / lower-stratosphere layer parameters
LAPSE_RATE = 0.0065   # temperature lapse rate in the troposphere, K/m
H_TROPOPAUSE = 11000.0  # base of the isothermal layer, m
T_TROPOPAUSE = T0_ISA - LAPSE_RATE * H_TROPOPAUSE  # 216.65 K

# Pressure at the tropopause (closed-form from the lapse layer below it).
_P_TROPOPAUSE = P0_ISA * (T_TROPOPAUSE / T0_ISA) ** (G0 / (R_AIR * LAPSE_RATE))


@dataclass(frozen=True)
class AtmosphereSample:
    """Bundle of local atmospheric properties at one altitude (SI units)."""
    temperature: float     # K
    pressure: float        # Pa
    density: float         # kg/m^3
    speed_of_sound: float  # m/s


def temperature(altitude_m: float) -> float:
    """Air temperature (K) at geometric altitude (m MSL), ISA model."""
    if altitude_m <= H_TROPOPAUSE:
        return T0_ISA - LAPSE_RATE * altitude_m
    # Isothermal lower stratosphere (11-20 km).
    return T_TROPOPAUSE


def pressure(altitude_m: float) -> float:
    """Static air pressure (Pa) at geometric altitude (m MSL), ISA model."""
    if altitude_m <= H_TROPOPAUSE:
        temp = T0_ISA - LAPSE_RATE * altitude_m
        return P0_ISA * (temp / T0_ISA) ** (G0 / (R_AIR * LAPSE_RATE))
    # Exponential decay in the isothermal layer.
    return _P_TROPOPAUSE * math.exp(
        -G0 * (altitude_m - H_TROPOPAUSE) / (R_AIR * T_TROPOPAUSE)
    )


def density(altitude_m: float) -> float:
    """Air density (kg/m^3) at geometric altitude (m MSL), ISA model."""
    return pressure(altitude_m) / (R_AIR * temperature(altitude_m))


def speed_of_sound(altitude_m: float) -> float:
    """Speed of sound (m/s) at geometric altitude (m MSL), ISA model."""
    return math.sqrt(GAMMA_AIR * R_AIR * temperature(altitude_m))


def sample(altitude_m: float) -> AtmosphereSample:
    """Return all four atmospheric properties at once (one altitude eval)."""
    temp = temperature(altitude_m)
    pres = P0_ISA * (temp / T0_ISA) ** (G0 / (R_AIR * LAPSE_RATE)) \
        if altitude_m <= H_TROPOPAUSE \
        else _P_TROPOPAUSE * math.exp(
            -G0 * (altitude_m - H_TROPOPAUSE) / (R_AIR * T_TROPOPAUSE)
        )
    rho = pres / (R_AIR * temp)
    a = math.sqrt(GAMMA_AIR * R_AIR * temp)
    return AtmosphereSample(temperature=temp, pressure=pres, density=rho,
                            speed_of_sound=a)
