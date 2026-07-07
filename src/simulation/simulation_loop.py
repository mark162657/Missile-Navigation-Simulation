"""
simulation_loop.py -- top-level integrated driver (physics plant + navigator).

This is the real replacement for ``NavigationComputer.run_navigation_loop``'s
use of ``MissileState.update_physics``. Instead of propagating ground truth from
a hand-fed constant acceleration (an Euler stand-in), the loop owns the full
physics package (``simulation.physics.MissileDynamics``) and advances truth from
the actual force model (thrust / drag / gravity / aero, RK4). Each tick the plant
emits a CLEAN ``IMUMeasurement`` that is, by construction, consistent with the
truth motion; that sample is handed straight to ``NavigationComputer.update``.

Data flows ONE way (matches simulation.physics' rule 6):
    control -> dynamics.step -> (truth_state, imu) -> nav.update

WHY A SEPARATE TRUTH STATE
    ``MissileState`` keeps separate ``true_*`` / ``est_*`` fields only for
    POSITION. Velocity, attitude, time and distance are shared, and the
    navigator's ``apply_ins_estimate`` overwrites them with the INS estimate.
    ``MissileDynamics.step`` reads ``vel_*`` as TRUTH, so if the plant and the
    navigator shared one ``MissileState`` the estimate would corrupt the plant's
    truth velocity on the next tick. The loop therefore keeps its own ``truth``
    state for the plant and only publishes truth POSITION (+ flight stage) into
    the navigator's state, which is all the sensors (GPS / TERCOM / baro) read.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
"""

from __future__ import annotations

from dataclasses import replace
from typing import TYPE_CHECKING

from missile.controls.control_input import ControlInput

if TYPE_CHECKING:
    from missile.navigation.navigation_computer import NavigationComputer
    from missile.state import MissileState
    from simulation.physics.dynamics import IMUMeasurement, MissileDynamics


class SimulationLoop:
    """Drives the ground-truth plant and the navigator together.

    The physics tick equals the navigator's INS tick (``nav.ins_period``): the
    plant is the highest-rate loop, and every step advances both truth and the
    navigator by exactly one ``ins_period`` so their clocks stay aligned.
    """

    def __init__(
        self,
        nav: "NavigationComputer",
        dynamics: "MissileDynamics",
        control: ControlInput | None = None,
    ) -> None:
        """
        Args:
            nav: the navigation computer to drive (its ``state`` seeds truth and
                receives the published truth position each tick).
            dynamics: the physics plant (owns profile / engine / aero / booster).
            control: the autopilot command applied each tick when none is passed
                to :meth:`step` / :meth:`run`. Defaults to a null command.
        """
        self.nav = nav
        self.dynamics = dynamics
        self.control = control if control is not None else ControlInput()

        # Independent ground-truth state for the plant, seeded from the nav's
        # launch state. Kept SEPARATE from nav.state (see module docstring).
        self.truth: "MissileState" = replace(nav.state)

    @property
    def dt(self) -> float:
        """Physics/navigation tick, in seconds (== the navigator's INS period)."""
        return self.nav.ins_period

    def step(self, control: ControlInput | None = None) -> "IMUMeasurement":
        """Advance truth one tick with the physics, then navigate that tick.

        Returns the clean ``IMUMeasurement`` the plant emitted (useful for
        logging / tests). Truth POSITION and flight stage are published into the
        navigator's state so the sensors see the current ground truth.
        """
        ctrl = self.control if control is None else control
        self.truth, imu = self.dynamics.step(self.truth, ctrl, self.dt)

        # Publish truth position + stage into the nav state for the sensors.
        # The estimate side (est_*, vel_*, attitude, time) is owned by the nav.
        self.nav.state.true_lat = self.truth.true_lat
        self.nav.state.true_lon = self.truth.true_lon
        self.nav.state.true_alt = self.truth.true_alt
        self.nav.state.missile_stage = self.truth.missile_stage

        self.nav.update(imu)
        return imu

    def run(
        self,
        run_seconds: float,
        control: ControlInput | None = None,
    ) -> None:
        """Run the integrated loop for ``run_seconds`` of simulated time.

        Resets the navigator's schedule, then steps at ``self.dt`` until the
        elapsed time is reached.
        """
        self.nav.reset_schedule()
        steps = int(round(run_seconds / self.dt))
        for _ in range(steps):
            self.step(control)
