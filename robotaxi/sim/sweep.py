"""Parameter sweeps: change one setting, rerun the simulator on several seeds, and see what it does.

Answers "what if" questions with data instead of guesses, for example:

    gps_sigma     How bad can the phone's GPS get before pickups suffer? (the radio fix should keep the
                  door close to the rider even when GPS alone is 15 m off)
    amber_decel   How hard is the car willing to brake for an amber light? (lower = runs more ambers,
                  higher = more stops)
    walk_min      How much green a pedestrian needs before starting to cross (affects how long cars wait)

    python simulate.py --sweep gps_sigma --minutes 15 --seeds 3

Every combination of value and seed is a separate World, so results are comparable. The table and a
chart are written to runs/sweep-<name>-<time>/.
"""
from __future__ import annotations

import os
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from . import ride, traffic  # noqa: E402
from .world import DT, World  # noqa: E402

# name -> (module, attribute, values, label)
PARAMS = {
    "gps_sigma": (ride, "GPS_SIGMA", [2.0, 5.0, 8.0, 12.0, 16.0], "phone GPS error σ (m)"),
    "amber_decel": (traffic, "AMBER_DECEL", [2.0, 2.6, 3.2, 3.8, 4.5], "max braking accepted for an amber (m/s²)"),
    "walk_min": (traffic, "WALK_MIN", [4.0, 7.0, 9.0, 12.0, 15.0], "green time a pedestrian needs to start (s)"),
}


def _measure(w: World, minutes: float) -> dict:
    for _ in range(int(minutes * 60 / DT)):
        w.step(DT)
    rides = pd.DataFrame(w.ride.records)
    tel = pd.DataFrame(w.telemetry)
    hours = w.t / 3600
    return dict(
        trips_per_hour=w.stats["trips"] / hours,
        km_per_hour=w.stats["dist"] / 1000 / hours,
        mean_speed_kmh=float(tel.v.mean() * 3.6) if len(tel) else np.nan,
        door_err_rf_m=float(rides.door_err_rf_m.mean()) if len(rides) else np.nan,
        door_err_gps_only_m=float(rides.door_err_gps_only_m.mean()) if len(rides) else np.nan,
        red_stops_per_hour=w.rules.red_stops / hours,
        emergency_per_hour=w.stats["emergency"] / hours,
        collisions=w.stats["collisions"],
        violations=w.rules.violations,
    )


def run_sweep(name: str, minutes: float = 15, seeds: int = 3, seed0: int = 300, out_dir: str | None = None,
              values: list | None = None, verbose: bool = True):
    if name not in PARAMS:
        raise SystemExit(f"unknown parameter '{name}'. Choose from: {', '.join(PARAMS)}")
    mod, attr, default_values, label = PARAMS[name]
    values = values or default_values
    out_dir = out_dir or os.path.join("runs", f"sweep-{name}-" + time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    original = getattr(mod, attr)
    rows = []
    try:
        for v in values:
            setattr(mod, attr, v)
            for k in range(seeds):
                t0 = time.time()
                m = _measure(World(seed=seed0 + k), minutes)
                rows.append(dict(value=v, seed=seed0 + k, **m))
                if verbose:
                    print(f"  {name}={v:<6g} seed {seed0 + k}  trips/h {m['trips_per_hour']:5.1f}  door {m['door_err_rf_m']:.2f} m"
                          f"  (GPS only {m['door_err_gps_only_m']:.1f} m)  red stops/h {m['red_stops_per_hour']:5.1f}"
                          f"  collisions {m['collisions']}  [{time.time() - t0:.0f} s]")
    finally:
        setattr(mod, attr, original)
    df = pd.DataFrame(rows)
    table = df.drop(columns="seed").groupby("value").mean().round(2)
    df.to_csv(os.path.join(out_dir, "runs.csv"), index=False)
    table.to_csv(os.path.join(out_dir, "by_value.csv"))
    _plot(df, table, name, label, os.path.join(out_dir, "sweep.png"))
    return table, out_dir


def _plot(df, table, name, label, path):
    panels = [("trips_per_hour", "Trips per hour"), ("door_err_rf_m", "Door to rider (m)"),
              ("red_stops_per_hour", "Red-light stops per hour"), ("emergency_per_hour", "Emergency stops per hour")]
    fig, axes = plt.subplots(1, len(panels), figsize=(18, 4.6), dpi=110)
    fig.suptitle(f"Parameter sweep: {name}", fontsize=14, fontweight="bold")
    for ax, (col, title) in zip(axes, panels):
        ax.scatter(df.value, df[col], color="#a8dadc", s=22, zorder=2, label="each run")
        ax.plot(table.index, table[col], color="#1d3557", lw=2, marker="o", zorder=3, label="mean")
        if col == "door_err_rf_m":
            ax.plot(table.index, table.door_err_gps_only_m, color="#ff8a3d", lw=2, ls="--", marker="s", label="GPS only")
        ax.set_title(title)
        ax.set_xlabel(label, fontsize=8)
        ax.grid(alpha=.3)
        ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
