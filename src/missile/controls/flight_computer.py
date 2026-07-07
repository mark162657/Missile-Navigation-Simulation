from controls.control_input import ControlInput
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
            approach_azimuth_rad: float,
            lookahead_dist: float=300.0
    ):
        self.path_follower = PathFollower(trajectory, profile, lookahead_dist=lookahead_dist)
        self.terminal = TerminalGuidance(target, profile, impact_angle_deg, approach_azimuth_rad)
        self.autopilot = AutoPilot(profile)
        self._terminal_latched = False

    def step(self, state: MissileState, dt: float) -> ControlInput:
        stage = self._resolve_stage(state)

        # these stages are handled by flight sequencer
        if stage in (FlightStage.PRE_LAUNCHED, FlightStage.BOOST, FlightStage.IMPACT):
            return ControlInput()
        if stage == FlightStage.TERMINAL:
            return self._step_terminal(state, dt)
        return self._step_cruise(state, dt)

    def _resolve_stage(self, state: MissileState) -> FlightStage:
        if self._terminal_latched:
            return FlightStage.TERMINAL
        if state.missile_stage == FlightStage.CRUISE and self.terminal.should_engage(state):
            self._terminal_latched = True
            self._reset()
            return FlightStage.TERMINAL
        return state.missile_stage

    def _step_cruise(self, state: MissileState, dt: float) -> ControlInput:
        lateral_accel, target_alt, target_spd = self.path_follower.update(state)
        self.autopilot.update(state, target_alt, target_spd, lateral_accel, dt)
        return self.autopilot.update(state, target_alt, target_spd, lateral_accel, dt)

    def _step_terminal(self, state: MissileState, dt: float) -> ControlInput:
        cmd = self.terminal.update(state, dt)
        throttle = self._speed_throttle(state, cmd.target_spd, dt)
        return ControlInput(throttle=throttle, accel_turn=cmd.accel_turn, accel_climb=cmd.accel_climb)


    def _speed_throttle(self, state: MissileState, target_spd: float, dt: float) -> float:
        curr_spd = self.state.get_ground_speed()
        spd_error = self.target_spd - curr_spd
        return self.autopilot.spd_pid.update(spd_error, curr_spd, dt)

    def _reset(self):
        """Reset the autopilot during transition of guidance stage"""
        self.autopilot.reset()



