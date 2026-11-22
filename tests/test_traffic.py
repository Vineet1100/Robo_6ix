"""Signal timing and right-of-way rules (robotaxi/sim/traffic.py, the twin of static/js/13-traffic.js)."""
import numpy as np
import pytest

from robotaxi.sim import traffic as T
from robotaxi.sim.routing import Dest, plan_route
from robotaxi.sim.geometry import lane_coord, P


def a_signal():
    return next(c for c in T.CTRL.values() if c.type == "signal")


def test_signal_cycle_never_green_both_ways():
    c = a_signal()
    for t in np.arange(0, T.CYCLE * 3, .25):
        h, v = T.sig_state(c, "h", t)[0], T.sig_state(c, "v", t)[0]
        assert not (h in "GY" and v in "GY"), f"conflicting greens at t={t}"


@pytest.mark.parametrize("axis", ["h", "v"])
def test_signal_phase_lengths(axis):
    c = a_signal()
    ts = np.arange(0, T.CYCLE, .01)
    states = [T.sig_state(c, axis, t)[0] for t in ts]
    assert states.count("G") * .01 == pytest.approx(T.SIG_G, abs=.02)
    assert states.count("Y") * .01 == pytest.approx(T.SIG_Y, abs=.02)
    assert states.count("R") * .01 == pytest.approx(T.CYCLE - T.SIG_G - T.SIG_Y, abs=.02)


def test_red_countdown_reaches_green():
    c = a_signal()
    for t in np.arange(0, T.CYCLE, .5):
        s, rem = T.sig_state(c, "h", t)
        if s == "R":
            assert T.sig_state(c, "h", t + rem + .01)[0] == "G"


def test_approach_rules():
    allstop = next(c for c in T.CTRL.values() if c.type == "allstop")
    assert T.approach_rule(allstop, "h") == T.approach_rule(allstop, "v") == "stop"
    two_way = next(c for c in T.CTRL.values() if c.type == "stop")
    rules = {T.approach_rule(two_way, "h"), T.approach_rule(two_way, "v")}
    assert rules == {"stop", "free"}
    yld = next(c for c in T.CTRL.values() if c.type == "yield")
    assert {T.approach_rule(yld, "h"), T.approach_rule(yld, "v")} == {"yield", "free"}


def test_route_gets_speed_limits_and_stop_lines():
    # start on Adelaide eastbound, go to York St between King and Wellington
    r = plan_route(20, lane_coord(0, 2 * P), 0, Dest(a=(3, 3), dd=1, frac=.5))
    assert r is not None and r.n > 100
    assert set(np.round(r.vl * 3.6).astype(int)) <= {30, 40, 50}
    for nd in r.nodes:
        assert nd["ctrl"] is not None
        assert 0 <= nd["qs"] <= nd["q"]
        # the stop line index sits just outside the intersection box
        assert max(abs(r.px[nd["qs"]] - nd["i"] * P), abs(r.py[nd["qs"]] - nd["j"] * P)) >= T.STOPL - .6
    assert T.ctrl_count(r) >= 1


def test_walk_need_only_for_street_crossings():
    assert T.walk_need((1, 1, -1, -1), (1, 1, 1, -1)) is not None      # across the avenue
    assert T.walk_need((1, 1, -1, -1), (0, 1, 1, -1)) is None          # along the block


def test_route_that_loops_a_block_sees_the_same_intersection_twice():
    # southbound on John, destination back at John & Wellington after going round the block
    r = plan_route(58, 189, 1, Dest(a=(1, 4), dd=3, frac=.6))
    passes = [nd for nd in r.nodes if (nd["i"], nd["j"]) == (1, 4)]
    assert len(passes) == 2
    assert passes[0]["q"] < 150 < passes[1]["q"]          # the first pass is matched to the first time through
    qs = [nd["q"] for nd in r.nodes]
    assert qs == sorted(qs)
