"""End-to-end: drive the headless simulator and check it follows the rules."""
import pytest

from robotaxi.sim import DT, World


@pytest.fixture(scope="module")
def world():
    w = World(seed=11)
    for _ in range(int(4 * 60 / DT)):        # four simulated minutes
        w.step(DT)
    return w


def test_taxi_drives(world):
    assert world.stats["dist"] > 300


def test_no_collisions(world):
    assert world.stats["collisions"] == 0


def test_no_red_lights_or_rolling_stops(world):
    assert world.rules.violations == 0


def test_stays_under_the_limit(world):
    assert world.rules.max_over_kmh < 2.5
    assert world.rules.over_t == 0


def test_it_actually_met_traffic_controls(world):
    r = world.rules
    assert r.red_stops + r.sign_stops >= 2


def test_telemetry_has_rule_columns(world):
    tel = world.dataframes()["telemetry"]
    for col in ("vlim", "ctl", "ctl_key", "sig", "hold"):
        assert col in tel.columns


def test_benchmark_trial_brakes_for_a_sudden_stop():
    from robotaxi.sim.benchmark import run_trial
    from robotaxi.sim.scenarios import SCENARIOS
    k = [n for n, _ in SCENARIOS].index("Lead car slams brakes")
    row = next(r for r in (run_trial(k, s) for s in range(40, 50)) if r is not None)
    assert not row["collided"] and row["reaction_s"] < 2 and row["min_gap_m"] > 0


def test_sweep_restores_the_setting(tmp_path):
    from robotaxi.sim import ride
    from robotaxi.sim.sweep import run_sweep
    before = ride.GPS_SIGMA
    table, _ = run_sweep("gps_sigma", minutes=.3, seeds=1, values=[3.0], out_dir=str(tmp_path), verbose=False)
    assert ride.GPS_SIGMA == before and len(table) == 1
