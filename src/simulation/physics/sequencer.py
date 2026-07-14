"""
sequencer.py -- flight-stage sequencer (vehicle configuration over time).

WHAT IT DOES
    Owns the missile's PHYSICAL staging, which is distinct from the guidance
    "mode": which motor is firing, the current vehicle mass, and the discrete
    events (booster burnout -> casing jettison -> turbofan takeover). It also
    supplies the programmed pitch-over attitude during boost, because at low
    boost speeds the airframe cannot pitch over aerodynamically -- a real
    booster does it by thrust/attitude programming, not by angle of attack.

    Stage machine (uses the existing FlightStage enum from missile.state):
        BOOST  --(booster burnt out)-->  CRUISE  (-> TERMINAL/IMPACT later)

WHAT IT OUTPUTS (queried by dynamics.py each step)
    - stage                 : current FlightStage
    - booster_thrust()      : booster thrust this step, N (0 outside BOOST)
    - attached_booster_mass(): casing + remaining propellant, kg (0 after sep)
    - commanded_attitude(state, dt) : (roll, pitch, yaw) rad during BOOST, else
                              None. Delegated to closed-loop BoostGuidance
                              (missile.guidance.boost_guidance).
    - advance(dt)           : burn propellant, step the schedule, and on burnout
                              jettison the booster and switch to CRUISE.

WHO CONSUMES IT
    - dynamics.py : if a sequencer is attached, MissileDynamics queries it for
                    the active thrust source, the vehicle mass, and (during
                    boost) the commanded attitude -- then integrates the SAME
                    equations of motion. No change to dynamics' public API.

DESIGN NOTE -- two kinds of "stage"
    This object handles the PHYSICAL stage (propulsion + mass). The guidance
    layer (later) keeps its own notion of phase (BOOST/MIDCOURSE/TERMINAL) for
    choosing guidance laws. They share the FlightStage enum and coincide at the
    boost->cruise instant, but physics never needs to know about guidance.

SIMPLIFICATIONS (kept deliberately light -- the project's focus is nav/guidance)
    - Submarine launch abstracts the underwater/surface-breach phase: it simply
      starts near-vertical at the launch altitude, like surface VLS.
    - The pitch-over and heading are now closed-loop (see boost_guidance.py): an
      altitude-scheduled flight-path-angle track plus pure-pursuit route capture.

Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
"""

from __future__ import annotations

import math
from enum import Enum, auto
from typing import TYPE_CHECKING

import numpy as np

from simulation.physics.booster import SolidBooster
from missile.state import FlightStage, MissileState
from missile.guidance.boost_guidance import BoostGuidance, BoostGuidanceSpecs

if TYPE_CHECKING:
    from missile.profile import MissileProfile
    from terrain.coordinates import CoordinateSystem


class LaunchMode(Enum):
    """How/where the missile is launched -- sets the initial boost attitude."""
    GROUND = auto()        # TEL / ground launch: shallow climb
    SURFACE_VLS = auto()   # ship vertical launch: near-vertical, pitch over
    SUBMARINE = auto()     # sub launch: abstracted to near-vertical from surface


# Launch (initial) pitch angle by mode, radians.
_LAUNCH_PITCH = {
    LaunchMode.GROUND: math.radians(45.0),
    LaunchMode.SURFACE_VLS: math.radians(88.0),
    LaunchMode.SUBMARINE: math.radians(88.0),
}


def _resolve_launch_mode(profile: "MissileProfile | None") -> LaunchMode:
    """Default launch mode: from the profile's booster spec, else SURFACE_VLS."""
    if profile is None:
        return LaunchMode.SURFACE_VLS
    try:
        return LaunchMode[profile.booster.launch_mode]
    except (KeyError, AttributeError):
        return LaunchMode.SURFACE_VLS


class FlightSequencer:
    """
    Sequences BOOST -> CRUISE for one missile.

    Construct with the cruise turbofan (so the sequencer can hand off to it) and
    a launch mode. Attach to MissileDynamics via its `sequencer=` argument.
    """

    def __init__(
        self,
        launch_mode: LaunchMode | None = None,
        cruise_heading_rad: float = 0.0,
        booster: SolidBooster | None = None,
        profile: "MissileProfile | None" = None,
        handoff_pitch_rad: float = math.radians(4.0),
        coordinate: "CoordinateSystem | None" = None,
        trajectory: np.ndarray | None = None,
    ) -> None:
        """
        Args:
            launch_mode: GROUND / SURFACE_VLS / SUBMARINE (sets initial pitch).
                If None, taken from `profile.booster.launch_mode` when a profile
                is given, else SURFACE_VLS.
            cruise_heading_rad: azimuth (rad, clockwise from north) the boost
                points toward; the missile leaves boost on this heading.
            booster: optional SolidBooster. If None, built from the profile's
                booster spec (SolidBooster(profile.booster)) when given, else a
                default booster.
            profile: optional MissileProfile to source the booster spec and the
                default launch mode from (the data-in-profile pattern).
            handoff_pitch_rad: pitch the schedule eases to by burnout, where the
                cruise dynamics (velocity-derived attitude) takes over.
        """
        if booster is None:
            booster = (SolidBooster(profile.booster) if profile is not None
                       else SolidBooster())
        if launch_mode is None:
            launch_mode = _resolve_launch_mode(profile)

        self.launch_mode = launch_mode
        self.cruise_heading = float(cruise_heading_rad)
        self.booster = booster
        self.handoff_pitch = float(handoff_pitch_rad)

        self.stage = FlightStage.BOOST
        self._launch_pitch = _LAUNCH_PITCH[launch_mode]
        self._elapsed = 0.0  # time since boost ignition, s

        # The pitch-over target is limited by what the booster can actually hold
        # up (see _thrust_limited_pitchover): commanding a shallower attitude
        # than thrust/weight can support makes the missile sink while still
        # boosting. Floor the handoff pitch and widen the pitch-over altitude
        # band accordingly so boost hands off to cruise climbing, never sinking.
        eff_handoff, eff_h_pitch = self._thrust_limited_pitchover(profile)

        # Closed-loop boost guidance (missile.guidance): vertical pitch-over +
        # lateral trajectory intercept. Replaces the old open-loop smoothstep.
        self.boost_guidance = BoostGuidance(
            specs=BoostGuidanceSpecs(
                gamma_launch=self._launch_pitch,
                gamma_handoff=eff_handoff,
                H_pitch=eff_h_pitch,
            ),
            coordinate=coordinate,
            trajectory=trajectory,
            cruise_heading_rad=self.cruise_heading,
        )

    # ------------------------------------------------------------------
    # Thrust-limited pitch-over sizing
    # ------------------------------------------------------------------
    def _thrust_limited_pitchover(
        self, profile: "MissileProfile | None"
    ) -> tuple[float, float]:
        """
        Size the boost pitch-over to the booster's actual thrust/weight.

        Steady climb needs the vertical thrust component to at least carry the
        weight, i.e. pitch >= asin(1 / (T/W)). A solid booster in this class runs
        a fairly low T/W (~1.7-1.9), so a near-level handoff (a few degrees) is
        physically unsupportable: as soon as the schedule pitches below the
        weight-support angle the vertical thrust drops under gravity, climb rate
        bleeds off, and the missile can sink into the ground before burnout.

        Returns (gamma_handoff, H_pitch):
            gamma_handoff : the requested handoff pitch, floored at the
                            weight-support angle plus a small climb margin so
                            boost hands off to cruise still climbing.
            H_pitch       : the altitude band over which the pitch-over
                            completes, modestly widened so the (steeper)
                            pitch-over eases out rather than snapping to the
                            handoff angle.

        With no profile (booster mass known, airframe mass unknown) the requested
        handoff and the default band are returned unchanged.
        """
        requested = self.handoff_pitch
        default_band = BoostGuidanceSpecs.H_pitch
        if profile is None:
            return requested, default_band

        g0 = 9.80665
        spec = self.booster.spec
        # Mean vehicle mass over the burn: airframe (+ cruise fuel, unburned
        # during boost) plus the booster casing and, on average, half its
        # propellant (propellant burns roughly linearly to zero).
        airframe = float(profile.detailed.mass_kg)
        mean_mass = airframe + spec.casing_mass_kg + 0.5 * spec.propellant_mass_kg
        weight = mean_mass * g0
        thrust = float(spec.booster_thrust_N)

        if thrust <= weight:
            # T/W <= 1: the booster cannot even hold weight at any pitch. Keep the
            # schedule near-vertical (best effort) and let the widened band apply.
            support_pitch = math.radians(80.0)
        else:
            support_pitch = math.asin(weight / thrust)

        # Small positive reserve so the handoff climbs gently rather than sitting
        # exactly neutral.
        floor = support_pitch + math.radians(3.0)
        gamma_handoff = max(requested, floor)

        # Modestly widen the band so the (now steeper) pitch-over eases out
        # toward the handoff instead of snapping to it; capped so the missile
        # still pitches over well within the burn rather than climbing vertically.
        band = default_band * 1.5
        return gamma_handoff, band

    # ------------------------------------------------------------------
    # Queries (read-only; called by dynamics at the start of a step)
    # ------------------------------------------------------------------
    @property
    def is_boosting(self) -> bool:
        return self.stage == FlightStage.BOOST

    def booster_thrust(self) -> float:
        """Booster thrust this step (N); 0 outside the BOOST stage."""
        return self.booster.thrust() if self.is_boosting else 0.0

    def attached_booster_mass(self) -> float:
        """Mass the booster still adds to the vehicle (kg); 0 after separation."""
        return self.booster.total_mass

    def commanded_attitude(self, state: MissileState, dt: float) -> tuple[float, float, float] | None:
        """
        Commanded (roll, pitch, yaw) during BOOST, else None.

        Delegates to closed-loop BoostGuidance: an altitude-scheduled pitch-over
        (vertical) plus pure-pursuit intercept of the planned route (lateral).
        """
        if not self.is_boosting:
            return None
        return self.boost_guidance.commanded_attitude(state, dt)

    # ------------------------------------------------------------------
    # Advance (mutating; called once per step after integration)
    # ------------------------------------------------------------------
    def advance(self, dt: float) -> None:
        """
        Step the boost: burn propellant and the schedule clock; on burnout,
        jettison the booster and transition to CRUISE.
        """
        if self.stage != FlightStage.BOOST:
            return

        self._elapsed += dt
        self.booster.consume(dt)

        if self.booster.is_burnt_out or self._elapsed >= self.booster.spec.burn_time_s:
            self.booster.separate()
            self.stage = FlightStage.CRUISE
