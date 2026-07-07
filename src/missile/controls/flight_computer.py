from guidance.target_geometry import TargetGeometry
from missile.controls.autopilot import AutoPilot
from missile.state import MissileState, FlightStage
from missile.profile import MissileProfile
from missile.planning.trajectory import TrajectoryGenerator
from guidance.path_follower import PathFollower
from guidance.terminal_guidance import TerminalGuidance


class FlightComputer:
    def __init__(
            self,
            trajectory: TrajectoryGenerator,
            profile: MissileProfile,
            target: TargetGeometry,
            impact_angle_deg: float,
            approach_azimuth_rad: float
    ):
        self.path_follower = PathFollower(trajectory, profile)
        self.terminal = TerminalGuidance(target, profile, impact_angle_deg, approach_azimuth_rad)
        self.autopilot = AutoPilot(profile)
        self._terminal_latched = False

    def _resolve_stage(self, state: MissileState) -> FlightStage:
        stage = state.missile_stage
        if self.latched:
            return FlightStage.TERMINAL
        elif stage is state.FlightStage.CRUISE and self.terminal.should_engage:
            self._terminal_latched = True
            self.stage = FlightStage.TERMINAL
            self._reset()
            return FlightStage.TERMINAL

        else:
            return stage

    def _step_cruise(self, state: MissileState, dt: float) -> None:
        lateral_accel, target_alt, target_spd = self.path_follower.update(state)
        self.autopilot.update(state, target_alt, target_spd, lateral_accel, dt)

    def _reset(self):
        """Reset the autopilot during transition of guidance stage"""
        self.autopilot.reset()



