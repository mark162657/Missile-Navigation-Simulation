from missile.controls.autopilot import AutoPilot
from missile.state import MissileState
from missile.profile import MissileProfile
from missile.planning.trajectory import TrajectoryGenerator
from guidance.path_follower import PathFollower

class FlightComputer:
    def __init__(self, trajectory: TrajectoryGenerator, profile: MissileProfile,):
        self.path_follower = PathFollower(trajectory, profile)
        self.autopilot = AutoPilot(profile)

    def step(self, state: MissileState, dt: float):
        lateral_accel, target_alt, target_spd = self.path_follower.update(state)
        return self.autopilot.update(state, target_alt, target_spd, lateral_accel, dt)

