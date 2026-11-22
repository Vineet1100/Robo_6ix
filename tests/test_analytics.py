"""pandas / scikit-learn analytics used by the web app and the simulator report."""
import numpy as np
import pandas as pd

from robotaxi.analytics import eta_model, hotspots, rf_calibration, rules_check


def test_rf_calibration_recovers_the_radio_model():
    rng = np.random.default_rng(1)
    d = rng.uniform(1, 80, 600)
    rssi = -41 - 10 * 2.3 * np.log10(d) + rng.normal(0, 3.5, d.size)
    fit = rf_calibration(pd.DataFrame(dict(d=d, rssi=rssi)))
    assert abs(fit["P0"] + 41) < 1 and abs(fit["n"] - 2.3) < .1


def test_eta_model_learns_time_per_metre():
    rng = np.random.default_rng(2)
    L = rng.uniform(150, 600, 20)
    turns = rng.integers(0, 5, 20)
    ctrls = rng.integers(1, 8, 20)
    dur = 20 + L / 8 + 5 * turns + 7 * ctrls + rng.normal(0, 3, 20)
    m = eta_model(pd.DataFrame(dict(length=L, turns=turns, controls=ctrls, duration=dur)))
    assert abs(m["per_m"] - 1 / 8) < .02 and m["per_ctrl"] > 3


def test_rules_check_counts_each_stop_once():
    rows = []
    for k, (hold, key) in enumerate([("", ""), ("red", "1,2"), ("red", "1,2"), ("", "1,2"), ("stop", "2,2"), ("gap", "2,2")]):
        rows += [dict(t=k + i * .2, v=0.0 if hold else 8.0, vlim=40, hold=hold, ctl_key=key) for i in range(10)]
    out = rules_check(pd.DataFrame(rows))
    assert out["red_stops"] == 1 and out["sign_stops"] == 1 and out["pct_over_limit"] == 0


def test_hotspots_find_a_cluster():
    rng = np.random.default_rng(3)
    pts = np.vstack([rng.normal([120, 180], 2, (30, 2)), rng.uniform(0, 300, (40, 2))])
    tel = pd.DataFrame(dict(x=pts[:, 0], y=pts[:, 1], max_risk=np.r_[np.full(30, .8), np.full(40, .1)],
                            action=["BRAKE"] * 30 + ["CRUISE"] * 40, crit=["Raccoon"] * 30 + [""] * 40))
    hs = hotspots(tel)
    assert hs and abs(hs[0]["x"] - 120) < 6 and abs(hs[0]["y"] - 180) < 6
