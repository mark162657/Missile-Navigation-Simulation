"""
test_weather.py -- wind field model sanity checks.

PLAIN SCRIPT (not pytest). Run:

    python tests/test_weather.py

What it checks:
    1. Mean log-law profile increases with altitude (wind shear) and points
       in the right ENU direction for a given met "wind-from" bearing.
    2. Dryden turbulence is RANDOM but TEMPORALLY CORRELATED -- the lag-1
       autocorrelation is high (not white noise), and the running std is in
       the right ballpark for the chosen severity.
    3. Reproducibility: same seed -> identical gust stream; different seed ->
       different stream.
    4. A discrete 1-cosine gust rises and falls smoothly within its window.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import numpy as np

from simulation.physics.weather import (
    DiscreteGust, DrydenTurbulence, MeanWindProfile, WindField,
    W20_MODERATE, W20_SEVERE,
)


def check_mean_profile() -> None:
    print("== 1. mean log-law profile + direction ==")
    # Westerly wind (from 270 deg) -> velocity points EAST (+e, ~0 n).
    mean = MeanWindProfile(speed_ref=10.0, direction_from_deg=270.0)
    for h in (10.0, 100.0, 1000.0, 5000.0):
        v = mean.velocity_enu(h)
        print(f"  h={h:6.0f} m  |V|={np.linalg.norm(v):5.2f}  "
              f"ENU=[{v[0]:6.2f},{v[1]:6.2f},{v[2]:5.2f}]")
    assert mean.speed(1000.0) > mean.speed(10.0), "shear: speed must grow w/ alt"
    v = mean.velocity_enu(100.0)
    assert v[0] > 0 and abs(v[1]) < 1e-6, "westerly wind must blow toward +east"
    print("  OK: increases with altitude, blows east.\n")


def check_turbulence_correlated() -> None:
    print("== 2. Dryden turbulence: correlated, not white ==")
    turb = DrydenTurbulence(w20=W20_MODERATE, seed=42)
    dt = 0.01
    vel = np.array([220.0, 0.0, 0.0])     # ~Mach 0.65 along east
    series = []
    for _ in range(20000):
        g = turb.step(dt, altitude_m=500.0, velocity_enu=vel)
        series.append(g[0])               # along-track (east) gust
    s = np.array(series)
    lag1 = np.corrcoef(s[:-1], s[1:])[0, 1]
    print(f"  gust std = {s.std():5.2f} m/s   lag-1 autocorr = {lag1:5.3f}")
    assert lag1 > 0.9, "turbulence must be strongly correlated step-to-step"
    assert 0.5 < s.std() < 15.0, "gust magnitude out of plausible range"
    print("  OK: smooth (high autocorr) yet stochastic.\n")


def check_reproducible() -> None:
    print("== 3. reproducibility by seed ==")
    dt, vel = 0.02, np.array([200.0, 0.0, 0.0])

    def run(seed: int) -> np.ndarray:
        t = DrydenTurbulence(w20=W20_SEVERE, seed=seed)
        return np.array([t.step(dt, 300.0, vel)[2] for _ in range(500)])

    a, b, c = run(7), run(7), run(8)
    assert np.allclose(a, b), "same seed must reproduce the gust stream"
    assert not np.allclose(a, c), "different seed must differ"
    print("  OK: seed 7 == seed 7, seed 7 != seed 8.\n")


def check_discrete_gust() -> None:
    print("== 4. discrete 1-cosine gust window ==")
    gust = DiscreteGust(amplitude=12.0, start_time=2.0, duration=1.0,
                        direction_enu=np.array([0.0, 0.0, 1.0]))
    before = np.linalg.norm(gust.velocity_enu(1.9))
    peak = np.linalg.norm(gust.velocity_enu(2.5))    # mid-window
    after = np.linalg.norm(gust.velocity_enu(3.1))
    print(f"  before={before:.2f}  peak={peak:.2f}  after={after:.2f}")
    assert before == 0.0 and after == 0.0, "gust must be zero outside window"
    assert abs(peak - 12.0) < 1e-6, "1-cosine peak must equal amplitude"
    print("  OK: smooth bump confined to its window.\n")


def check_field_integration() -> None:
    print("== 5. WindField.step combines layers ==")
    field = WindField.preset(speed_ref=8.0, direction_from_deg=315.0,
                             w20=W20_MODERATE, seed=1)
    vel = np.array([180.0, 60.0, 5.0])
    sample = field.step(0.01, altitude_m=250.0, velocity_enu=vel)
    print(f"  mean ENU = {np.round(sample.mean_enu, 2)}")
    print(f"  gust ENU = {np.round(sample.gust_enu, 2)}")
    print(f"  total    = {np.round(sample.velocity_enu, 2)}")
    assert np.allclose(sample.velocity_enu, sample.mean_enu + sample.gust_enu)
    print("  OK: total == mean + gust.\n")


if __name__ == "__main__":
    check_mean_profile()
    check_turbulence_correlated()
    check_reproducible()
    check_discrete_gust()
    check_field_integration()
    print("All weather-model checks passed.")
