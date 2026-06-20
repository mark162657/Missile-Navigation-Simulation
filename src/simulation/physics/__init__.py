"""
simulation.physics -- bottom-of-stack flight physics for the cruise-missile sim.

Layers (each file documents its own inputs/outputs/consumers):
    atmosphere.py    : ISA air properties + the shared physical constants.
    aerodynamics.py  : lift/drag/side force + pitching moment from (Mach, AoA,
                       sideslip, controls).
    engine.py        : turbofan thrust, fuel burn, remaining-fuel bookkeeping.
    dynamics.py      : 3-DoF point-mass equations of motion (RK4) + IMU output.

Data flows ONE way (rule 6):  control -> physics -> (new_state, IMU).
Physics never calls guidance, navigation or autopilot.

Public API (so the simulation loop can do
`from simulation.physics import MissileDynamics, ControlInput, IMUMeasurement`):
"""

from simulation.physics.dynamics import (
    ControlInput,
    IMUMeasurement,
    MissileDynamics,
)
from simulation.physics.booster import BoosterSpec, SolidBooster
from simulation.physics.sequencer import FlightSequencer, LaunchMode

__all__ = [
    "MissileDynamics", "ControlInput", "IMUMeasurement",
    "FlightSequencer", "LaunchMode", "SolidBooster", "BoosterSpec",
]
