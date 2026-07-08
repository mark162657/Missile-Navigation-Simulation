"""
Extensive test suite for the controls + guidance packages.

Covers, with the real modules wherever they import cleanly:
  * controls: PIDController, AutoPilot, ControlInput
  * guidance: TargetGeometry, TerminalGuidance (unit + closed-loop vs. the real
    physics plant), PathFollower (heavy DEM/scipy deps stubbed for import).

Run:  PYTHONPATH=src pytest tests/test_controls_guidance.py -v
"""

import math
import sys
import types
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from missile.controls.control_input import ControlInput
from missile.controls.pid_controller import PIDController
from missile.controls.autopilot import AutoPilot
from missile.profile import MissileProfile, BasicSpec, DetailedSpec
from missile.state import MissileState, FlightStage
from missile.guidance.target_geometry import TargetGeometry
from missile.guidance.terminal_guidance import TerminalGuidance, TerminalCommand
from terrain.coordinates import CoordinateSystem

_G = 9.80665


# ----------------------------------------------------------------------
# Fixtures
# ----------------------------------------------------------------------
def make_profile() -> MissileProfile:
    """Tomahawk-like fallback profile (matches the other integration tests)."""
    basic = BasicSpec(
        cruise_speed=880.0, min_speed=400.0, max_speed=920.0,
        max_acceleration=9.8, min_altitude=30.0, max_altitude=1500.0,
        max_g_force=6.0, sustained_turn_rate=8.0, sustained_g_force=2.0,
        evasive_turn_rate=25.0, max_range=1600.0,
        cruise_agl_min=20.0, cruise_agl_max=70.0,
    )
    return MissileProfile(name="Test", basic=basic, detailed=DetailedSpec())


def make_state(lat, lon, alt, ve, vn, vu, *, yaw=0.0) -> MissileState:
    """Minimal state with est == truth (perfect nav for guidance tests)."""
    return MissileState(
        true_lat=lat, true_lon=lon, true_alt=alt,
        est_lat=lat, est_lon=lon, est_alt=alt,
        vel_east=ve, vel_north=vn, vel_up=vu,
        roll=0.0, pitch=0.0, yaw=yaw,
        time=0.0, distance_traveled=0.0, distance_to_target=0.0,
        gps_valid=True, tercom_active=False, ins_calibrated=True,
    )


@pytest.fixture
def profile():
    return make_profile()


# ======================================================================
# PIDController
# ======================================================================
class TestPIDController:
    def test_proportional_only(self):
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0, out_max=100, out_min=-100)
        assert pid.update(error=5.0, measurement=0.0, dt=0.1) == pytest.approx(10.0)

    def test_clamp_high_and_low(self):
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0, out_max=3.0, out_min=-3.0)
        assert pid.update(10.0, 0.0, 0.1) == 3.0
        assert pid.update(-10.0, 0.0, 0.1) == -3.0

    def test_integral_accumulates(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, out_max=100, out_min=-100)
        pid.update(2.0, 0.0, 0.5)      # integral = 1.0
        out = pid.update(2.0, 0.0, 0.5)  # integral = 2.0
        assert out == pytest.approx(2.0)

    def test_anti_windup_freezes_integral(self):
        # Saturating high with positive error must not let the integral grow.
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, out_max=1.0, out_min=-1.0)
        for _ in range(20):
            pid.update(5.0, 0.0, 0.1)
        # Conditional anti-windup: integral saturates where ki*integral = out_max
        # (=1.0 here) and is rolled back on every further step -> never grows past it.
        assert pid.integral == pytest.approx(1.0, abs=1e-9)

    def test_derivative_on_measurement_brakes(self):
        # Rising measurement with zero error -> derivative pushes negative.
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0, out_max=100, out_min=-100)
        pid.update(0.0, 0.0, 0.1)          # seeds prev_mea, d=0
        out = pid.update(0.0, 1.0, 0.1)    # measurement jumped +1 over dt=0.1
        assert out == pytest.approx(-10.0)

    def test_no_setpoint_kick(self):
        # Derivative keys off measurement, so a setpoint (error) jump does NOT spike D.
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0, out_max=100, out_min=-100)
        pid.update(0.0, 0.0, 0.1)
        out = pid.update(50.0, 0.0, 0.1)   # big error, measurement unchanged
        assert out == pytest.approx(0.0)

    def test_bad_dt_raises(self):
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0, out_max=1, out_min=-1)
        with pytest.raises(ValueError):
            pid.update(1.0, 0.0, 0.0)
        with pytest.raises(ValueError):
            pid.update(1.0, 0.0, -0.1)

    def test_bad_limits_raise(self):
        with pytest.raises(ValueError):
            PIDController(kp=1, ki=0, kd=0, out_max=1.0, out_min=1.0)

    def test_reset(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=1.0, out_max=100, out_min=-100)
        pid.update(2.0, 1.0, 0.1)
        pid.reset()
        assert pid.integral == 0.0 and pid.prev_mea is None


# ======================================================================
# AutoPilot
# ======================================================================
class TestAutoPilot:
    def test_level_flight_holds_gravity(self, profile):
        ap = AutoPilot(profile)
        spd = profile.basic.cruise_speed_ms
        st = make_state(0, 0, 100.0, spd, 0, 0)
        # On target altitude & speed -> alt PID ~0 -> accel_climb ~ _G.
        cmd = ap.update(st, target_alt=100.0, target_spd=spd, lateral_accel_cmd=0.0, dt=0.1)
        assert cmd.accel_climb == pytest.approx(_G, abs=1e-6)
        assert 0.0 <= cmd.throttle <= 1.0
        assert cmd.accel_turn == 0.0

    def test_below_target_commands_climb(self, profile):
        ap = AutoPilot(profile)
        spd = profile.basic.cruise_speed_ms
        st = make_state(0, 0, 50.0, spd, 0, 0)
        cmd = ap.update(st, target_alt=200.0, target_spd=spd, lateral_accel_cmd=0.0, dt=0.1)
        assert cmd.accel_climb > _G       # extra climb accel on top of gravity hold

    def test_slow_commands_more_throttle(self, profile):
        ap = AutoPilot(profile)
        spd = profile.basic.cruise_speed_ms
        slow = make_state(0, 0, 100.0, 0.5 * spd, 0, 0)
        cmd = ap.update(slow, target_alt=100.0, target_spd=spd, lateral_accel_cmd=0.0, dt=0.1)
        assert cmd.throttle > 0.0

    def test_lateral_passthrough_and_reset(self, profile):
        ap = AutoPilot(profile)
        spd = profile.basic.cruise_speed_ms
        st = make_state(0, 0, 100.0, spd, 0, 0)
        cmd = ap.update(st, 100.0, spd, lateral_accel_cmd=12.3, dt=0.1)
        assert cmd.accel_turn == 12.3
        ap.reset()
        assert ap.alt_pid.integral == 0.0 and ap.spd_pid.integral == 0.0


# ======================================================================
# ControlInput
# ======================================================================
def test_control_input_defaults():
    ci = ControlInput()
    assert (ci.throttle, ci.accel_turn, ci.accel_climb) == (0.0, 0.0, 0.0)
    ci2 = ControlInput(throttle=0.5, accel_turn=1.0, accel_climb=9.8)
    assert ci2.accel_climb == 9.8


# ======================================================================
# TargetGeometry
# ======================================================================
class TestTargetGeometry:
    def setup_method(self):
        self.coord = CoordinateSystem(0.0, 0.0)
        self.tg = TargetGeometry((0.0, 0.0), self.coord, target_alt=0.0)

    def test_zero_distance_at_target(self):
        st = make_state(0.0, 0.0, 0.0, 0, 0, 0)
        assert self.tg.direct_ground_distance(st) == pytest.approx(0.0, abs=1e-6)

    def test_ground_distance_north(self):
        # 0.01 deg latitude ~ 1111.9 m (spherical, R=6371 km).
        st = make_state(0.01, 0.0, 0.0, 0, 0, 0)
        d = self.tg.direct_ground_distance(st)
        assert d == pytest.approx(math.radians(0.01) * 6371000.0, rel=1e-3)

    def test_3d_is_hypot_of_ground_and_alt(self):
        st = make_state(0.01, 0.0, 1000.0, 0, 0, 0)
        ground = self.tg.direct_ground_distance(st)
        slant = self.tg.direct_3d_distance(st)
        assert slant == pytest.approx(math.hypot(ground, 1000.0), rel=1e-6)

    def test_km_scaling(self):
        st = make_state(0.05, 0.0, 0.0, 0, 0, 0)
        assert self.tg.direct_ground_distance(st, meter=False) == pytest.approx(
            self.tg.direct_ground_distance(st, meter=True) / 1000.0, rel=1e-9)


# ======================================================================
# TerminalGuidance -- unit
# ======================================================================
class TestTerminalGuidanceUnit:
    def setup_method(self):
        self.coord = CoordinateSystem(0.0, 0.0)
        self.tg = TargetGeometry((0.0, 0.0), self.coord, target_alt=0.0)
        self.profile = make_profile()

    def _guid(self, impact_deg=-60.0, az=math.pi):
        return TerminalGuidance(self.tg, self.profile, impact_deg, az)

    def test_flight_path_angle(self):
        g = self._guid()
        st = make_state(0.02, 0, 1000.0, 0.0, -200.0, -200.0)  # 45 deg dive
        assert g._flight_path_angle(st) == pytest.approx(math.radians(-45.0), abs=1e-9)

    def test_los_angle_below_is_negative(self):
        g = self._guid()
        st = make_state(0.02, 0, 1000.0, 0, -200.0, 0)  # north of target, up high
        assert g._los_angle(st) < 0.0

    def test_tgo_method2_exceeds_range_over_speed_on_curved_path(self):
        g = self._guid(impact_deg=-70.0)
        st = make_state(0.02, 0, 1200.0, 0, -244.0, 0)
        V = st.get_ground_speed()
        gm = g._flight_path_angle(st)
        lam = g._los_angle(st)
        R = self.tg.direct_3d_distance(st)
        tgo = g._time_to_go(st, V, gm, lam)
        assert tgo > R / V           # curvature correction lengthens t_go

    def test_guidance_law_signs(self):
        g = self._guid(impact_deg=-90.0)
        # a positive command == pitch DOWN (paper EOM V*gamma_dot = -a_n).
        V, tgo = 200.0, 10.0
        # level entry, target 45 deg below, vertical impact -> start diving (a_n>0)
        a = g._guidance_accel(V, tgo, gamma_m=0.0,
                              lam=math.radians(-45), )
        assert a > 0
        # already on a steep collision course -> ~0
        a0 = g._guidance_accel(V, tgo, gamma_m=math.radians(-90),
                               lam=math.radians(-90))
        assert abs(a0) < 1e-6

    def test_guidance_law_lofts_for_steep_impact(self):
        g = self._guid(impact_deg=-80.0)
        # shallow LOS but steep desired impact -> command pitch UP first (a_n<0)
        a = g._guidance_accel(200.0, 10.0, gamma_m=0.0, lam=math.radians(-17))
        assert a < 0

    def test_reference_invariance(self):
        # Coeffs (-6,4,2) sum to 0 => rotating all three angles leaves a_n unchanged.
        g = self._guid(impact_deg=-60.0)
        base = g._guidance_accel(200.0, 10.0, gamma_m=0.1, lam=-0.3)
        g2 = self._guid(impact_deg=math.degrees(math.radians(-60.0) + 0.2))
        shifted = g2._guidance_accel(200.0, 10.0, gamma_m=0.1 + 0.2, lam=-0.3 + 0.2)
        assert base == pytest.approx(shifted, abs=1e-9)

    def test_command_clamped_to_envelope(self):
        g = self._guid(impact_deg=-89.0)
        a = g._guidance_accel(300.0, 0.05, gamma_m=0.0, lam=math.radians(-80))
        assert abs(a) <= self.profile.get_max_lateral_acceleration() + 1e-9

    @pytest.mark.parametrize("gm_deg", [0.0, -20.0, -45.0, -60.0, -85.0])
    def test_decompose_delivers_minus_a_n(self, gm_deg):
        # THE gravity/cos-gamma fix: plant net-perp accel must equal -a_n exactly.
        g = self._guid()
        gm = math.radians(gm_deg)
        a_n = 25.0
        st = make_state(0.02, 0, 1000.0, 0, -200.0 * math.cos(gm), 200.0 * math.sin(gm))
        accel_climb, _ = g._decompose(a_n, gm, 200.0, st)
        net_perp = accel_climb - _G * math.cos(gm)   # what dynamics._force_enu delivers
        assert net_perp == pytest.approx(-a_n, abs=1e-9)

    def test_engage_range_formula(self):
        g = self._guid(impact_deg=-60.0)
        V = self.profile.basic.cruise_speed_ms
        a_max = self.profile.get_max_lateral_acceleration()
        expect = 3.0 * 2.0 * V * V * abs(math.sin(math.radians(-60.0))) / a_max
        assert g.terminal_init_range() == pytest.approx(expect, rel=1e-9)

    def test_should_engage_boundary(self):
        g = self._guid(impact_deg=-60.0)
        m_lat = 111195.0  # ~m per deg lat near equator
        far = make_state((g.d_init + 500.0) / m_lat, 0, 1000.0, 0, -200.0, 0)
        near = make_state((g.d_init - 500.0) / m_lat, 0, 1000.0, 0, -200.0, 0)
        assert g.should_engage(far) is False
        assert g.should_engage(near) is True

    def test_no_blowup_near_impact(self):
        g = self._guid(impact_deg=-60.0)
        st = make_state(1e-7, 0, 0.5, 0, -200.0, -150.0)  # basically on top of target
        cmd = g.update(st, dt=0.02)
        for v in (cmd.accel_turn, cmd.accel_climb, cmd.target_spd):
            assert math.isfinite(v)


# ======================================================================
# TerminalGuidance -- closed loop against the REAL physics plant
# ======================================================================
def test_closed_loop_dive_engagement():
    from simulation.physics.dynamics import MissileDynamics
    from terrain import coordinates as coords

    profile = make_profile()
    coord = CoordinateSystem(0.0, 0.0)
    target = TargetGeometry((0.0, 0.0), coord, target_alt=0.0)

    impact_deg = -55.0
    guid = TerminalGuidance(target, profile, impact_deg, approach_azimuth_rad=math.pi)

    # Start NORTH of the target, flying due south (yaw=pi), level, at cruise speed.
    V = profile.basic.cruise_speed_ms
    m_lat = coords.meter_per_deg_lat(0.0)
    start_ground = 0.75 * guid.d_init            # comfortably inside the engage range
    start_alt = 2200.0
    st = make_state(start_ground / m_lat, 0.0, start_alt, 0.0, -V, 0.0, yaw=math.pi)
    st.missile_stage = FlightStage.TERMINAL

    assert guid.should_engage(st)

    plant = MissileDynamics(profile)  # calm wind, no booster
    dt = 0.02
    a_max = profile.get_max_lateral_acceleration()

    prev_alt = st.true_alt
    impact_gamma = None
    impact_miss = None

    for _ in range(4000):  # <= 80 s guard
        st.est_lat, st.est_lon, st.est_alt = st.true_lat, st.true_lon, st.true_alt
        cmd = guid.update(st, dt)

        # sanity every tick
        assert math.isfinite(cmd.accel_climb) and math.isfinite(cmd.accel_turn)

        # simple speed hold on throttle so the geometry test isn't confounded
        thr = float(np.clip(0.55 + 0.01 * (V - st.get_ground_speed()), 0.0, 1.0))
        control = ControlInput(throttle=thr, accel_turn=cmd.accel_turn,
                               accel_climb=cmd.accel_climb)
        st, _imu = plant.step(st, control, dt)

        assert math.isfinite(st.true_alt) and math.isfinite(st.true_lat)

        if st.true_alt <= 0.0:  # crossed the ground plane -> "impact"
            # flight-path angle and horizontal miss at impact
            vh = math.hypot(st.vel_east, st.vel_north)
            impact_gamma = math.degrees(math.atan2(st.vel_up, vh))
            impact_miss = target.direct_ground_distance(st)
            break

        assert st.true_alt < prev_alt + 5.0   # never climbs wildly; overall descent
        prev_alt = st.true_alt

    assert impact_gamma is not None, "missile never reached the ground"
    # Impact-angle constraint: steep dive, clearly beyond the initial LOS (~ -36 deg).
    assert impact_gamma < -45.0, f"impact too shallow: {impact_gamma:.1f} deg"
    assert impact_gamma > -80.0, f"impact overshot: {impact_gamma:.1f} deg"
    # Miss distance small.
    assert impact_miss < 300.0, f"miss too large: {impact_miss:.1f} m"


# ======================================================================
# PathFollower (stub heavy DEM/scipy import deps, then test the L1 math)
# ======================================================================
def _install_import_stubs():
    """Fake matplotlib/rasterio/scipy so path_follower's import chain resolves."""
    for name in ("matplotlib", "matplotlib.pyplot", "matplotlib.colors",
                 "rasterio", "rasterio.transform", "rasterio.windows",
                 "scipy", "scipy.interpolate"):
        if name not in sys.modules:
            mod = types.ModuleType(name)
            sys.modules[name] = mod
    # attributes referenced at import time
    sys.modules["matplotlib.colors"].LinearSegmentedColormap = object
    sys.modules["matplotlib.colors"].LightSource = object
    sys.modules["rasterio.transform"].rowcol = lambda *a, **k: (0, 0)
    sys.modules["rasterio.transform"].xy = lambda *a, **k: (0.0, 0.0)
    sys.modules["rasterio.windows"].Window = object
    sys.modules["scipy.interpolate"].splprep = lambda *a, **k: None
    sys.modules["scipy.interpolate"].splev = lambda *a, **k: None


class TestPathFollower:
    def setup_method(self):
        _install_import_stubs()
        from missile.guidance.path_follower import PathFollower
        self.PathFollower = PathFollower
        self.coord = CoordinateSystem(0.0, 0.0)
        self.profile = make_profile()
        # A straight north-bound path (lat increasing), 200 points, elev 0.
        lats = np.linspace(0.0, 0.02, 200)
        self.path = np.column_stack([lats, np.zeros(200), np.zeros(200)])

    def _pf(self):
        return self.PathFollower(self.path, self.profile, self.coord, lookahead_dist=300.0)

    def test_builds_enu_and_length(self):
        pf = self._pf()
        assert pf.traj_enu.shape == (200, 2)
        assert pf.traj_length == 200

    def test_bad_shape_raises(self):
        with pytest.raises(ValueError):
            self.PathFollower(np.zeros((5,)), self.profile, self.coord)

    def test_l1_zero_when_aligned(self):
        pf = self._pf()
        pos = np.array([0.0, 0.0])
        aim = np.array([0.0, 500.0])       # straight ahead (north)
        a = pf._l1_lateral_accel(pos, enu_bearing=0.0, enu_ground_speed=200.0,
                                 aim_pt_enu=aim, kl=2.0)
        assert a == pytest.approx(0.0, abs=1e-9)

    def test_l1_sign_right_turn(self):
        pf = self._pf()
        pos = np.array([0.0, 0.0])
        aim = np.array([100.0, 400.0])     # aim point to the East (right) of due-north
        a = pf._l1_lateral_accel(pos, enu_bearing=0.0, enu_ground_speed=200.0,
                                 aim_pt_enu=aim, kl=2.0)
        assert a > 0.0                     # + = turn right (toward the aim point)

    def test_l1_clamped(self):
        pf = self._pf()
        pos = np.array([0.0, 0.0])
        aim = np.array([300.0, 5.0])       # nearly abeam -> huge demanded accel
        a = pf._l1_lateral_accel(pos, 0.0, 300.0, aim, kl=2.0)
        assert abs(a) <= self.profile.get_max_lateral_acceleration() + 1e-9

    def test_closest_and_lookahead_advance(self):
        # _find_closest is an intentional FORWARD local-window search (50 pts from
        # last_idx), tracking monotonic progress -- so it advances gradually.
        pf = self._pf()
        ci1 = pf._find_closest(pf.traj_enu[30])   # within the first window
        assert 25 <= ci1 <= 35
        ci2 = pf._find_closest(pf.traj_enu[60])   # window has moved forward
        assert ci2 >= ci1 and 55 <= ci2 <= 65
        ai = pf._lookahead(ci2, pf.l1)
        assert ai >= ci2
