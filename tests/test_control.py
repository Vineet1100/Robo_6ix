"""Speed control, Ziegler-Nichols and routing."""
import math

import numpy as np

from robotaxi.sim.control import RULES, SpeedPID, rule_gains, zn_at
from robotaxi.sim.routing import Dest, dest_point, plan_route
from robotaxi.sim.geometry import P, lane_coord
from robotaxi.zn import route_profile


class _Ego:
    def __init__(self, v):
        self.x, self.y, self.th, self.v, self.acc = 100.0, lane_coord(0, 2 * P), 0.0, v, 0.0


def test_zn_ultimate_gain_is_positive_and_finite():
    for v in (2, 6, 11):
        for mode in ("traction", "brake"):
            z = zn_at(v, 1900, .9, mode, .02)
            assert 0 < z["Ku"] < 100 and 0 < z["Tu"] < 10


def test_every_tuning_rule_gives_sane_gains():
    z = zn_at(8, 1900, .9, "traction", .02)
    for rule in RULES:
        g = rule_gains(z, rule)
        assert g["Kp"] > 0 and g["Ki"] >= 0 and g["Kd"] >= 0


def test_pid_tracks_a_speed_step():
    pid, ego, dt = SpeedPID(1 / 30), _Ego(0.0), 1 / 30
    pid.reset(0.0)
    t = 0.0
    for _ in range(int(12 / dt)):              # accelerate gently towards ~10 m/s
        t += dt
        pid.step(ego, 1900, 1.0 if ego.v < 10 else 0.0, t, dt)
    assert 8.5 < ego.v < 11.5


def test_route_profile_respects_each_points_limit():
    s = np.arange(0, 200, 5.0)
    vlim = np.where(s < 100, 13.9, 8.3)
    out = route_profile(s, np.zeros_like(s), np.full_like(s, .9), 1900, vlim)
    v = np.array(out["v"])
    assert (v <= vlim + 1e-6).all()


def test_dijkstra_route_ends_at_destination_without_u_turns():
    dest = Dest(a=(4, 1), dd=1, frac=.4)
    r = plan_route(20, lane_coord(0, 2 * P), 0, dest)
    dx, dy = dest_point(dest)
    assert math.hypot(r.px[-1] - dx, r.py[-1] - dy) < 1.0
    heading = np.unwrap(np.arctan2(r.ty, r.tx))
    assert np.abs(np.diff(heading)).max() < .3            # smooth, no reversal
