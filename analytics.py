"""Data analysis of a running simulation with pandas and scikit-learn.

* Summary statistics (pandas)
* Radio-model calibration: fit RSSI = P0 - 10·n·log10(d) with LinearRegression (the browser
  then uses the fitted model in its particle filter)
* Trip-time (ETA) model: LinearRegression on route length + number of turns
* Risk hotspots: DBSCAN clustering of high-risk moments on the map
"""
from __future__ import annotations

import math

import numpy as np
import pandas as pd
from sklearn.cluster import DBSCAN
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_absolute_error, r2_score

from .citymap import nearest_street

TRUE_RF = dict(P0=-41.0, n=2.3, shadow_db=3.5)     # what the simulator uses as ground truth


def _f(x, nd=3):
    return None if x is None or (isinstance(x, float) and math.isnan(x)) else round(float(x), nd)


def summary(fr: dict) -> dict:
    tel, pk, tr, ev = fr["telemetry"], fr["pickups"], fr["trips"], fr["events"]
    out = {}
    if len(tel):
        moving = tel[tel.v > 1]
        out.update(sim_time_s=_f(tel.t.max(), 1), mean_speed_kmh=_f(tel.v.mean() * 3.6, 1),
                   speed_rms_error_ms=_f(np.sqrt(((moving.vref - moving.v) ** 2).mean())) if len(moving) else None,
                   planner_ms_mean=_f(tel.plan_ms.mean()), planner_ms_p95=_f(tel.plan_ms.quantile(.95)),
                   time_share=(tel.action.value_counts(normalize=True) * 100).round(1).to_dict(),
                   odo_raw_drift_max_m=_f(tel.odo_raw.max(), 2), odo_fused_err_max_m=_f(tel.odo_fus.max(), 2),
                   samples=int(len(tel)))
    if len(pk):
        out.update(pickups=int(len(pk)), door_rf_mean_m=_f(pk.door_rf.mean(), 2), door_gps_mean_m=_f(pk.door_gps.mean(), 2),
                   door_rf_p90_m=_f(pk.door_rf.quantile(.9), 2), rider_wait_mean_s=_f(pk.wait_s.mean(), 1))
    if len(tr):
        out.update(legs_completed=int(len(tr)), mean_leg_m=_f(tr.length.mean(), 1), mean_leg_s=_f(tr.duration.mean(), 1))
    if len(ev):
        out["events"] = ev.level.value_counts().to_dict()
    return out


def rf_calibration(rf: pd.DataFrame) -> dict | None:
    if len(rf) < 40:
        return None
    d = rf[(rf.d > 1) & (rf.d < 90)]
    if len(d) < 40:
        return None
    X = np.log10(d[["d"]].to_numpy())
    y = d.rssi.to_numpy()
    reg = LinearRegression().fit(X, y)
    pred = reg.predict(X)
    return dict(P0=_f(reg.intercept_, 2), n=_f(-reg.coef_[0] / 10, 3), r2=_f(r2_score(y, pred)),
                shadow_db=_f(np.std(y - pred), 2), samples=int(len(d)), truth=TRUE_RF)


def eta_model(tr: pd.DataFrame) -> dict | None:
    if len(tr) < 3:
        return None
    X, y = tr[["length", "turns"]].to_numpy(float), tr.duration.to_numpy(float)
    reg = LinearRegression().fit(X, y)
    pred = reg.predict(X)
    return dict(per_m=_f(reg.coef_[0], 4), per_turn=_f(reg.coef_[1], 2), base_s=_f(reg.intercept_, 2),
                mae_s=_f(mean_absolute_error(y, pred), 1), r2=_f(r2_score(y, pred)) if len(tr) >= 4 else None,
                samples=int(len(tr)))


def predict_eta(model: dict | None, length: float, turns: int) -> float:
    if not model:
        return length / 6.5 + 20 + 4 * turns          # heuristic until enough trips are logged
    return max(5.0, model["base_s"] + model["per_m"] * length + model["per_turn"] * turns)


def hotspots(tel: pd.DataFrame, top=6) -> list:
    if not len(tel):
        return []
    risky = tel[(tel.max_risk > .35) | tel.action.isin(["BRAKE", "EMERGENCY"])]
    if len(risky) < 4:
        return []
    labels = DBSCAN(eps=10, min_samples=4).fit_predict(risky[["x", "y"]].to_numpy())
    risky = risky.assign(cluster=labels)
    out = []
    for cid, g in risky[risky.cluster >= 0].groupby("cluster"):
        cause = g.crit.dropna()
        cause = cause[cause != ""]
        x, y = float(g.x.mean()), float(g.y.mean())
        out.append(dict(x=round(x, 1), y=round(y, 1), count=int(len(g)), radius=round(min(18.0, float(np.hypot(g.x - x, g.y - y).quantile(.8)) + 4), 1),
                        cause=(cause.mode().iloc[0] if len(cause) else "mixed"), where=nearest_street(x, y)))
    return sorted(out, key=lambda h: -h["count"])[:top]


def zn_by_condition(zn: pd.DataFrame) -> list:
    if not len(zn):
        return []
    z = zn.assign(surface=np.where(zn.mu < .6, "wet", "dry"))
    g = z.groupby(["mode", "surface"]).agg(points=("Ku", "size"), Ku_mean=("Ku", "mean"), Tu_mean=("Tu", "mean"),
                                           Kp_mean=("Kp", "mean")).reset_index()
    return g.round(3).to_dict(orient="records")


def analyze(fr: dict) -> dict:
    return dict(summary=summary(fr), rf_fit=rf_calibration(fr["rf"]), eta=eta_model(fr["trips"]),
                hotspots=hotspots(fr["telemetry"]), zn=zn_by_condition(fr["zn_log"]),
                counts={k: int(len(v)) for k, v in fr.items()})
