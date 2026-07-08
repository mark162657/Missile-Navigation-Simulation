import math
import numpy as np
from dataclasses import dataclass, replace

from missile.profile import MissileProfile
from missile.state import MissileState, FlightStage
from missile.controls.control_input import ControlInput
from missile.controls.flight_computer import FlightComputer
from missile.guidance.target_geometry import TargetGeometry
from missile.planning.pathfinding_backend import Pathfinding
from missile.planning.trajectory import TrajectoryGenerator
from simulation.physics.dynamics import MissileDynamics
from simulation.physics.sequencer import FlightSequencer
from terrain.coordinates import CoordinateSystem

@dataclass
class SimulationConfig:
    # Geographic setup
    dem_name: str
    start_gps: tuple(float, float, float) # 3d location of starting location (lat, lon, elev)
    target_gps: tuple(float, float, float)

    # Planning
    heuristic_weight: float = 2.0

    # Midcourse guidance
    lookahead_dist: float = 300.0
    dt: float = 0.01 # sim tick, 100Hz
    max_flight_time_s: float = 7200 # hard guard for max flight time to prevent burning your pc (default 2hr)
    impact_radius_m: float = 10 # horizontal miss in meter that still counts as a hit

    # Terminal guidance
    approach_azimuth_radius: float | None = None
    impact_angle_deg: float = -30.0 # desired dive angle at impact (negative for a dive)


class Simulation:
    def __init__(self, profile: MissileProfile, config: SimulationConfig) -> None:
        self.profile = profile
        self.config = config

        self.coord = CoordinateSystem(config.start_gps[0], config.start_gps[1])
        self.pathfinding = Pathfinding(config.dem_name)
        self.trajectory_gen = TrajectoryGenerator(
            self.pathfinding.engine, self.pathfinding.dem_loader
        )
        self.target = TargetGeometry(config.target_gps[:2], self.coord, config.target_gps[2])

        # Setting up control related
        self.trajectory: np.ndarray | None = None
        self.flight_computer: FlightComputer | None = None
        self.dynamics: MissileDynamics | None = None
        self.sequencer: FlightSequencer | None = None
        self.nav = None # navigation computer

        self.state: MissileState | None = None
        self.sim_time: float = 0.0
        self._result: dict | None = None


    @classmethod
    def from_config(cls, profile: MissileProfile, config: SimulationConfig) -> "Simulation":
        """Build a simulation from a profile and a configuration. Mirroring the (profile, config) pair."""
        return cls(profile, config)

    def plan_mission(self) -> np.ndarray:
        """
        Run Pathifnding algorithm start -> target and receive the returned trajectory.

        Return:
            the (N, 3) [lat, lon, ground_elev] array
        """

        # run the pathfinder and get the raw pixel/row coordinate
        raw_pixel_path = self.run_patfinding()

        # turn the raw path to trajectory in gps and ground_elev
        self.trajectory = self.trajectory_gen.get_trajectory(raw_pixel_path)

        if self.trajectory is None or len(self.trajectory) < 3:
            raise RuntimeError("Empty trajectory returned.")
        return self.trajectory

    def run_pathfinding(self) -> list[tuple[int, int]]:
        """
        A* pathfinding over DEM in pixel coordinates (convert gps -> row/col coordinate)

        Return:
            path in pixel coordinate
        """
        start_rc = self.pathfinding.dem_loader.lat_lon_to_pixel(self.config.start_gps[0], self.config.start_gps[1])
        target_rc = self.pathfinding.dem_loader.lat_lon_to_pixel(self.config.target_gps[0], self.config.target_gps[1])

        path = self.pathfinding.find_path(
            tuple(start_rc), tuple(target_rc),
            heuristic_weight=self.config.heuristic_weight
        )

        if not path:
            raise RuntimeError("Pathfinding failed: no route start -> target.")

        return path


    # Pre-launched
    def _build_initial_state(self, true_start_gps: tuple[float, float, float]) -> MissileState:
        """
        Build and initialise the initial state of a missile in state.py
        This serves as the only point which initial state.py is built.

        Args:
            true_start_gps: (lat, lon, alt) of the gps position of the origin position

        Return:
            MissileState object with initial state
        """
        lat, lon, alt = true_start_gps
        return MissileState(
            true_lat=lat,
            true_lon=lon,
            true_alt=alt,
            est_lat=lat,
            est_lon=lon,
            est_alt=alt,
            vel_east=0.0,
            vel_north=0.0,
            vel_up=0.0,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            time=0.0,
            distance_traveled=0.0,
            distance_to_target=0.0,
            gps_valid=True,
            tercom_active=False,
            ins_calibrated=True,
            missile_stage=FlightStage.PRE_LAUNCHED
        )







