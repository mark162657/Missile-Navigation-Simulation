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
from missile.navigation.navigation_computer import NavigationComputer
from missile.profile_selector import choose_profile
from paths import PROJECT_ROOT
from simulation.clock import RealtimePacer
from simulation.physics.dynamics import MissileDynamics, IMUMeasurement
from simulation.physics.sequencer import FlightSequencer
from simulation.result import MissionResult, FlightLogger
from terrain.coordinates import CoordinateSystem
from terrain.dem_loader import DEMLoader


@dataclass
class SimulationConfig:
    # Geographic setup
    dem_name: str
    start_gps: tuple[float, float, float] # 3d location of starting location (lat, lon, elev)
    target_gps: tuple[float, float, float]

    # Planning
    heuristic_weight: float = 2.0

    # Midcourse guidance
    lookahead_dist: float = 300.0
    dt: float = 0.002 # sim tick, 500Hz
    max_flight_time_s: float = 7200 # hard guard for max flight time to prevent burning your pc (default 2hr)
    impact_radius_m: float = 10 # horizontal miss in meter that still counts as a hit

    # Wall-clock pacing: 1.0 = 1 sim s == 1 real s; 0.0 = as-fast-as-possible
    realtime_factor: float = 1.0

    # Terminal guidance
    approach_azimuth_rad: float | None = None
    impact_angle_deg: float = -30.0 # desired dive angle at impact (negative for a dive)

    detonation_radius_m: float = 25.0 # detonate if missile is within this radius m of target

    # TODO Missile-identifier hashed
    missile_id: str = ""
    command_centre_id: str = ""


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
        self.flight_computer: FlightComputer
        self.dynamics: MissileDynamics
        self.sequencer: FlightSequencer
        self.nav = None # navigation computer

        self.state: MissileState
        self.sim_time: float = 0.0
        self._result: dict | None = None

        self._SEA_LEVEL_M = 0.0

    @classmethod
    def from_config(cls, profile: MissileProfile, config: SimulationConfig) -> "Simulation":
        """
        Build a simulation from a profile and a configuration. Mirroring the (profile, config) pair.

        Args:
            profile: MissileProfile object
            config: SimulationConfig object

        Return:
            Simulation class object
        """
        return cls(profile, config)

    def plan_mission(self) -> np.ndarray:
        """
        Run Pathifnding algorithm start -> target and receive the returned trajectory.

        Return:
            the (N, 3) [lat, lon, ground_elev] array
        """

        # run the pathfinder and get the raw pixel/row coordinate
        raw_pixel_path = self.run_pathfinding()

        # turn the raw path to trajectory in gps and ground_elev
        self.trajectory = self.trajectory_gen.get_trajectory(raw_pixel_path)

        if self.trajectory is None or len(self.trajectory) < 3:
            raise RuntimeError("Empty trajectory returned.")
        return self.trajectory

    def _ask_yes_no(self, prompt: str, *, default_no: bool = True) -> bool:
        """Ask a y/n question. Empty input follows default_no."""
        hint = "[y/N]" if default_no else "[Y/n]"
        while True:
            raw = input(f"{prompt} {hint}: ").strip().lower()
            if raw in ("y", "yes"):
                return True
            if raw in ("n", "no"):
                return False
            if raw == "":
                return not default_no
            print("  Enter y or n.")

    def _confirm_pathfinding(self) -> bool:
        """
        Before pathfinding, print mission endpoints and ask whether to plan
        the route. Returns True to run A*, False to abort.
        """
        start = self.config.start_gps
        target = self.config.target_gps
        ground_km = self.target.direct_ground_distance(
            self._build_initial_state(start)
        ) / 1000.0

        print("\n--- Ready to plan route ---")
        print(f"  DEM    : {self.config.dem_name}")
        print(f"  Launch : {start[0]:.5f}°N, {start[1]:.5f}°E  alt {start[2]:.1f} m")
        print(f"  Target : {target[0]:.5f}°N, {target[1]:.5f}°E  alt {target[2]:.1f} m")
        print(f"  Direct : {ground_km:.1f} km")
        print(f"  Missile: {self.profile.name}")

        return self._ask_yes_no("\nRun pathfinding?")

    def _confirm_launch(self) -> bool:
        """
        After pathfinding, print a short route summary and ask the operator
        whether to ignite. Returns True to launch, False to abort.
        """
        traj = self.trajectory
        n_pts = 0 if traj is None else len(traj)
        start = self.config.start_gps
        target = self.config.target_gps
        ground_km = self.target.direct_ground_distance(
            self._build_initial_state(start)
        ) / 1000.0

        print("\n--- Pathfinding complete ---")
        print(f"  Launch : {start[0]:.5f}°N, {start[1]:.5f}°E  alt {start[2]:.1f} m")
        print(f"  Target : {target[0]:.5f}°N, {target[1]:.5f}°E  alt {target[2]:.1f} m")
        print(f"  Direct : {ground_km:.1f} km")
        print(f"  Route  : {n_pts:,} trajectory points")
        print(f"  Missile: {self.profile.name}")

        return self._ask_yes_no("\nConfirm launch?")

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

    def _approach_azimuth(self) -> float:
        """Terminal approach bearing (rad, CW from north)."""
        if self.config.approach_azimuth_rad is not None:
            return self.config.approach_azimuth_rad
        heading_deg = self.coord.get_heading(
            self.config.start_gps[0], self.config.start_gps[1],
            self.config.target_gps[0], self.config.target_gps[1]
        )
        return math.radians(heading_deg)

    def _pre_ignition_setup(self) -> None:
        """Start navigation and flight computer before the ignition during pre-launched."""
        self.flight_computer = FlightComputer(
            trajectory=self.trajectory,
            profile=self.profile,
            target=self.target,
            coordinate=self.coord,
            impact_angle_deg=self.config.impact_angle_deg,
            approach_azimuth_rad=self._approach_azimuth(),
            lookahead_dist=self.config.lookahead_dist
        )

        self.nav = NavigationComputer(
            true_start_gps=self.config.start_gps,
            dem_name=self.config.dem_name,
        )

    # Ignition phase
    def _ignite(self) -> None:
        """Initialise the boost sequencer, transition from PRE_LAUNCHED -> BOOST."""
        self.sequencer = FlightSequencer(
            cruise_heading_rad=self._approach_azimuth(),
            profile=self.profile,
            coordinate=self.coord,
            trajectory=self.trajectory,
        )
        self.dynamics = MissileDynamics(self.profile, sequencer=self.sequencer)

        # mirror BOOST into state
        self.state = replace(self.state, missile_stage=FlightStage.BOOST)

    # Simulation loop post-ignition
    def step(self, dt: float | None = None) -> None:
        """
        Step the simulation forward by dt seconds.
        Step guidance, physics, navigation, and check impact every tick.

        Args:
            dt: timestamp / tick
        """
        dt = self.config.dt if dt is None else dt

        control = self._step_guidance(dt) # missile control
        imu = self._step_physics(control, dt) # physics and imu
        self._step_navigation(imu, dt) # navigation modules
        self._update_stage()

        # check impact every tick
        self._check_impact()

        # driving navigation's GPS/TERCOM
        self.sim_time += dt

    def _step_guidance(self, dt: float) -> ControlInput:
        """
        Produce a control command using FlightComputer for this tick.
        Returns ControlInput for PRE_LAUNCHED / BOOST / IMPACT.
        These stages are handled by FlightSequencer, not FlightComputer.

        Args:
            dt: timestamp / tick

        Return:
            ControlInput for the missile.
        """
        if self.state.missile_stage in (FlightStage.PRE_LAUNCHED, FlightStage.BOOST, FlightStage.IMPACT):
            return ControlInput()
        return self.flight_computer.step(self.state, dt)


    def _step_physics(self, control: ControlInput, dt: float) -> IMUMeasurement:
        """
        Update the physical state of the missile using dynamics.py for every tick.
        Dynamics apply the control input and return the state and IMU measurement.

        Args:
            control: control input calculated from FlightComputer
            dt: timestamp / tick

        Returns:
            IMU measurement of the missile.
        """
        self.state, imu = self.dynamics.step(self.state, control, dt)
        return imu

    def _step_navigation(self, imu: IMUMeasurement, dt: float) -> None:
        """
        Update the navigation estimate from this tick's IMU measurement.
        Truths are advanced and handled in MissileDynamics.

        Args:
            imu: IMU measurement of the missile (from physics)
            dt: timestamp / tick

        """
        self.nav.step(imu, self.state, self.sim_time, dt)

    def _update_stage(self) -> None:
        """
        Check if terminal guidance has started, if so, switch the FlightStage in missile state.
        Done in FlightComputer, if terminal guidance is engaged (terminal_latched=True), and the
        current FlightStage is CRUISE, then switch to TERMINAL.
        """
        if self.flight_computer is None:
            return

        if self.flight_computer.terminal_latched and self.state.missile_stage==FlightStage.CRUISE:
            self.state = replace(self.state, missile_stage=FlightStage.TERMINAL)

    def _ground_elevation_msl(self) -> float:
        _SEA_LEVEL_M = 0.0

        ground = self.pathfinding.dem_loader.get_elevation(
            self.state.true_lat, self.state.true_lon
        )

        if ground is None or not math.isfinite(ground):
            ground = self._SEA_LEVEL_M
        return float(ground)

    def _at_ground(self) -> bool:
        """
        True the tick the missile reaches/at the ground elevation (terrain or target)

        Return:
            True if the missile is at the ground elevation, False otherwise.
        """
        return self.state.true_alt <= self._ground_elevation_msl()

    def _impact_angle_deg(self) -> float:
        """
        Actual missile flight-path (dive) gained from velocity vector.

        Return:
             degrees of the impact angle, negative for a dive.
        """
        return math.degrees(math.atan2(self.state.vel_up, math.hypot(self.state.vel_east, self.state.vel_north)))

    def _within_target_radius(
            self,
            target_gps: tuple[float, float],
            radius_m: float | int
    ) -> bool:
        """
        Return True if the missile is within radius_m of target_gps.

        Args:
            target_gps: (lat, lon) of the target
            radius_m: radius around target that is valid for the missile to be detonated
        """
        target_lat, target_lon = target_gps
        dx = (self.state.true_lon - target_lon) * self.coord.meter_per_deg_lon_at(target_lat)
        dy = (self.state.true_lat - target_lat) * self.coord.meter_per_deg_lat(target_lat)
        return math.hypot(dx, dy) <= radius_m

    def _detonate(self) -> bool:
        """
        Detonate the missile if it is within the detonation radius around the target.

        Return:
            True if the missile is within the detonation radius, False otherwise.
        """
        if self.state.missile_stage != FlightStage.TERMINAL:
            return False
        return self.target.direct_ground_distance(self.state) <= self.config.detonation_radius_m

    def _check_impact(self) -> None:
        """
        Check impact by: if stage is not impact already. Check if the missile hit the terrain obstacle or not.
        If not impacted already, and not hit an obstacle accidentally.
        """
        if self.state.missile_stage == FlightStage.IMPACT:
            return
        if not self._at_ground():
            return  # still in the air

        warhead = self.profile.warhead

        miss_distance_m = self.target.direct_ground_distance(self.state)
        detonated = self._detonate()
        hit_terrain = self.state.missile_stage != FlightStage.TERMINAL

        self.state = replace(self.state, missile_stage=FlightStage.IMPACT)
        self._result = MissionResult(
            outcome = MissionResult.classify(
            miss_distance_m, warhead.blast_radius_m, hit_terrain=hit_terrain, detonated=detonated
            ),
            miss_distance_m=miss_distance_m,
            impact_angle_deg=self._impact_angle_deg(),
            impact_speed_ms=self.state.get_ground_speed(),
            impact_gps=(self.state.true_lat, self.state.true_lon, self.state.true_alt),
            flight_time_s=self.state.time,
            distance_flown_m=self.state.distance_traveled,
            start_gps=self.config.start_gps,
            target_gps=self.config.target_gps,
            detonated=detonated,
            missile_id=self.config.missile_id,
            command_centre_id=self.config.command_centre_id,
        )
        self.state = replace(self.state, missile_stage=FlightStage.IMPACT)


    def _check_mission_complete(self) -> bool:
        """
        Check if the mission ends on impact or timeout.

        Return:
            True if the mission is complete, False otherwise.
        """
        return (self.state.missile_stage == FlightStage.IMPACT
               or self.state.time >= self.config.max_flight_time_s)

    # ----------------------------------------------------------------
    # Main Driver Loop
    # ----------------------------------------------------------------

    def run(self, max_duration_s: float | None = None) -> MissionResult | None:
        """
        Confirm plan -> pathfind -> confirm launch -> ignite -> fly -> log -> save.

        Args:
            max_duration_s: optional override of config.max_flight_time_s.

        Return:
            the MissionResult (also saved to data/results/; a per-flight CSV is
            written to data/logs/), or None if the operator aborted at a confirm.
        """
        if max_duration_s is not None:
            self.config.max_flight_time_s = max_duration_s

        if self.trajectory is None:
            if not self._confirm_pathfinding():
                print("Pathfinding aborted.")
                return None
            self.plan_mission()

        if not self._confirm_launch():
            print("Launch aborted.")
            return None

        self.state = self._build_initial_state(self.config.start_gps)
        self._pre_ignition_setup()
        self._ignite()

        # Per-flight telemetry: one CSV in data/logs/, sampled at 10 Hz.
        self.logger = FlightLogger(interval_s=0.1, missile_id=self.config.missile_id).open()
        pacer = RealtimePacer(self.config.realtime_factor)
        pacer.start()
        try:
            # step: guidance / physics and navigation update per tick
            while self._alive():
                self.step()
                # Hold until wall clock catches sim_time (no-op if realtime_factor <= 0).
                pacer.wait_until(self.sim_time)
                self._log_flight()
            # always capture the final (impact) tick, ignoring the sample interval
            self._log_flight(force=True)
        finally:
            self.logger.close()

        return self._finalise_result()

    def _log_flight(self, force: bool = False) -> None:
        """Record one telemetry row for the current tick (ground alt + target range)."""
        self.logger.record(
            self.state, self.sim_time,
            distance_to_target_m=self.target.direct_ground_distance(self.state),
            ground_alt_m=self.pathfinding.dem_loader.get_elevation(
                self.state.true_lat, self.state.true_lon
            ),
            force=force,
        )

    def _alive(self) -> bool:
        """
        Check if the simulation / missile is still alive.

        Return:
            True if the simulation is still alive, False otherwise.
        """
        return not self._check_mission_complete()



    def _finalise_result(self) -> MissionResult:
        """
        Fill in a MissionResult for timeout condition, happens if ended without impact.
        Lastly, save and report.
        """
        if self._result is None:
            self._result = MissionResult.timeout(
                flight_time_s=self.state.time,
                distance_flown_m=self.state.distance_traveled,
                start_gps=self.config.start_gps,
                target_gps=self.config.target_gps,
                missile_id=self.config.missile_id,
                command_centre_id=self.config.command_centre_id,
            )
        self._result.save()  # -> data/results/<id>_<timestamp>.json
        print(self._result.summary())
        return self._result

def plain_mission_only(profile: MissileProfile, config: SimulationConfig) -> None:
    sim = Simulation(profile, config)
    sim.plan_mission()
    return sim


# ----------------------------------------------------------------
# Interactive setup (profile + config)
# Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
# ----------------------------------------------------------------
def _ask(msg: str, default: str | None = None) -> str:
    hint = f" [{default}]" if default not in (None, "") else ""
    raw = input(f"{msg}{hint}: ").strip()
    return raw or (default or "")


def _ask_float(msg: str, default: float | None = None) -> float:
    hint = f" [{default}]" if default is not None else ""
    while True:
        raw = input(f"{msg}{hint}: ").strip()
        if not raw and default is not None:
            return float(default)
        try:
            return float(raw)
        except ValueError:
            print("  Please enter a number.")


def _ask_gps(label: str, dem_name: str) -> tuple[float, float, float]:
    dem = DEMLoader(dem_name)

    lat = _ask_float(f"{label} latitude (deg)")
    lon = _ask_float(f"{label} longitude (deg)")

    default_alt = dem.get_elevation(lat, lon)
    if default_alt is None:
        print(
            f"Warning: Could not read DEM elevation for {label} at "
            f"({lat}, {lon}). Defaulting altitude to 0.0 m."
        )
        default_alt = 0.0

    alt = _ask_float(
        f"{label} altitude (m MSL) [Pixel_elev]",
        default=default_alt,
    )

    return (lat, lon, alt)


def _choose_dem() -> str:
    dem_dir = PROJECT_ROOT / "data" / "dem"
    dems = sorted(p.name for p in dem_dir.glob("*.tif"))
    if not dems:
        return _ask("DEM filename (in data/dem/)")
    print("\nAvailable DEMs:")
    for i, name in enumerate(dems, 1):
        print(f"  {i}. {name}")
    while True:
        raw = input(f"Choice [1-{len(dems)}]: ").strip()
        if raw.isdigit() and 1 <= int(raw) <= len(dems):
            return dems[int(raw) - 1]
        print("  Invalid choice.")


# Preset scenarios for quick, repeatable runs. Coordinates fall inside the
# named DEM; altitudes are resolved from it at setup time.
TEST_MISSIONS = {
    "1": {
        "name": "Simple north-south separation",
        "dem_name": "merged_dem_sib_N54_N59_E090_E100.tif",
        "start": (56.000, 95.000),
        "target": (56.903, 95.000),
        "desc": "Point A: 56.000°N, 95.000°E -> Point B: 56.903°N, 95.000°E  (~100.3 km)",
    },
}


def _choose_mission_type() -> str:
    """Ask whether to build a custom mission or run a preset test mission."""
    print("\nSelect mission type:")
    print("  1. Custom mission")
    print("  2. Test mission")
    while True:
        raw = input("Choice [1-2]: ").strip()
        if raw in ("1", "2"):
            return raw
        print("  Invalid choice.")


def _choose_test_mission() -> dict:
    print("\nAvailable test missions:")
    for key, m in TEST_MISSIONS.items():
        print(f"  {key}. {m['name']}")
        print(f"       {m['desc']}")
    while True:
        raw = input(f"Choice [1-{len(TEST_MISSIONS)}]: ").strip()
        if raw in TEST_MISSIONS:
            return TEST_MISSIONS[raw]
        print("  Invalid choice.")


def _auto_gps(dem: DEMLoader, lat: float, lon: float) -> tuple[float, float, float]:
    """Resolve a (lat, lon, alt) triple, reading altitude from the DEM."""
    alt = dem.get_elevation(lat, lon)
    if alt is None:
        print(f"Warning: no DEM elevation at ({lat}, {lon}); defaulting altitude to 0.0 m.")
        alt = 0.0
    return (lat, lon, alt)


def _setup_test_mission(profile: MissileProfile) -> Simulation:
    """Build a Simulation from a preset scenario, everything else defaulted."""
    mission = _choose_test_mission()
    dem_name = mission["dem_name"]
    dem = DEMLoader(dem_name)

    start_gps = _auto_gps(dem, *mission["start"])
    target_gps = _auto_gps(dem, *mission["target"])

    print(f"\n--- Test mission: {mission['name']} ---")
    print(f"  Launch: {start_gps}")
    print(f"  Target: {target_gps}")

    config = SimulationConfig(
        dem_name=dem_name,
        start_gps=start_gps,
        target_gps=target_gps,
    )
    return Simulation(profile, config)


def setup_mission() -> Simulation:
    """Interactively build a profile + config, return a ready Simulation."""
    mission_type = _choose_mission_type()

    profile = choose_profile()  # pick existing / create new

    if mission_type == "2":
        return _setup_test_mission(profile)

    print("\n--- Mission setup ---")
    dem_name = _choose_dem()
    start_gps = _ask_gps("Launch", dem_name)
    target_gps = _ask_gps("Target", dem_name)

    # Essentials above; everything else keeps its SimulationConfig default unless
    # the user overrides it here.
    config = SimulationConfig(
        dem_name=dem_name,
        start_gps=start_gps,
        target_gps=target_gps,
        impact_angle_deg=_ask_float("Impact/dive angle (deg, negative = dive)", default=-30.0),
        detonation_radius_m=_ask_float("Detonation radius (m)", default=25.0),
        missile_id=_ask("Missile ID (optional)", default=""),
        command_centre_id=_ask("Command centre ID (optional)", default=""),
    )
    return Simulation(profile, config)


def main() -> None:
    sim = setup_mission()
    sim.run(20)


if __name__ == "__main__":
    main()














