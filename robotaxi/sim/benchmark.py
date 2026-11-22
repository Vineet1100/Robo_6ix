"""Scenario benchmark: run every crunch-time test many times in isolation and measure how the robotaxi copes.

This is how autonomous-driving teams usually judge a planner: not by one long drive, but by replaying the
same dangerous situation over and over with different random traffic and checking the numbers every time.

For each trial a fresh World is created (its own seed), the taxi drives until it is cruising, the hazard is
injected a few seconds ahead, and the next 12 seconds are measured:

    reaction_s      time from the hazard appearing until the taxi brakes harder than 1 m/s^2
    min_gap_m       closest distance between the taxi's body and the hazard (0 = contact)
    peak_decel      hardest braking used (m/s^2)
    emergency       whether the planner had to use EMERGENCY BRAKE
    collided        whether the taxi touched the hazard while moving
    speed_kmh       the taxi's speed when the hazard appeared

    python simulate.py --benchmark --trials 10
"""
from __future__ import annotations

import math
import os
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from .agents import SIZE_R  # noqa: E402
from .geometry import DIR  # noqa: E402
from .scenarios import SCENARIOS, frame_at  # noqa: E402
from .world import DT, World  # noqa: E402

WATCH_S = 12.0          # seconds measured after each injection
WARMUP_MAX_S = 90.0     # give up on a trial if the taxi never gets a clear straight to test on


def clearance(ego, o) -> float:
    """Distance between the taxi's body (three circles) and an object's body (capsule or circle)."""
    c, s = math.cos(ego.th), math.sin(ego.th)
    ex = ego.x + c * np.array([-1.2, 0, 1.2])
    ey = ego.y + s * np.array([-1.2, 0, 1.2])
    if o.kind == "car":
        hl = max(1.4, o.L / 2 - .95)
        offs = np.linspace(-hl, hl, 7)
        ox, oy, r = o.x + DIR[o.d][0] * offs, o.y + DIR[o.d][1] * offs, .95
    else:
        ox, oy, r = np.array([o.x]), np.array([o.y]), SIZE_R[o.kind]
    d = np.sqrt((ex[:, None] - ox[None, :]) ** 2 + (ey[:, None] - oy[None, :]) ** 2).min()
    return float(max(0.0, d - 1.0 - r))


def _warm_up(w: World) -> bool:
    """Drive until the taxi is cruising with a clear straight ahead (hazards need room to be placed)."""
    t_end = w.t + WARMUP_MAX_S
    while w.t < t_end:
        w.step(DT)
        e = w.ego
        if w.t > 8 and e.v > max(5.5, .8 * e.vlim) and w.decision.act[0] == "CRUISE" and frame_at(w, e.v * 2.4 + 6):
            return True
    return False


def run_trial(scenario: int, seed: int) -> dict | None:
    name, fn = SCENARIOS[scenario]
    w = World(seed=seed)
    w.auto_chaos = False
    if not _warm_up(w):
        return None
    before = {id(o) for o in w.actors}
    v0, col0 = w.ego.v, w.stats["collisions"]
    if not fn(w):
        return None
    hazards = [o for o in w.actors if id(o) not in before]
    if not hazards:
        return None
    w.log(f"BENCHMARK > {name}", "info")
    t0, reaction, min_gap, peak, emerg = w.t, None, 1e9, 0.0, False
    while w.t - t0 < WATCH_S:
        w.step(DT)
        e = w.ego
        if reaction is None and e.acc < -1.0:
            reaction = w.t - t0
        peak = max(peak, -e.acc)
        emerg = emerg or w.decision.act[0] == "EMERGENCY"
        live = [o for o in hazards if o in w.actors]
        if live:
            min_gap = min(min_gap, min(clearance(e, o) for o in live))
    return dict(scenario=name, seed=seed, speed_kmh=round(v0 * 3.6, 1),
                reaction_s=round(reaction, 2) if reaction is not None else np.nan,
                min_gap_m=round(min_gap, 2) if min_gap < 1e8 else np.nan, peak_decel=round(peak, 2),
                emergency=emerg, collided=w.stats["collisions"] > col0)


def run_benchmark(trials: int = 10, seed: int = 100, out_dir: str | None = None, verbose: bool = True):
    """Run every scenario `trials` times. Returns (per-trial DataFrame, summary DataFrame, output folder)."""
    out_dir = out_dir or os.path.join("runs", "benchmark-" + time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    rows = []
    for k, (name, _) in enumerate(SCENARIOS):
        got, s = 0, seed + 1000 * k
        while got < trials and s < seed + 1000 * k + trials * 4:      # skip seeds where the hazard can't be placed
            row = run_trial(k, s)
            s += 1
            if row is None:
                continue
            rows.append(row)
            got += 1
            if verbose:
                flag = "CONTACT" if row["collided"] else "ok"
                print(f"  {name:28s} trial {got:2d}  {row['speed_kmh']:4.0f} km/h  reaction {row['reaction_s']:.2f} s"
                      f"  closest {row['min_gap_m']:.2f} m  peak {row['peak_decel']:.1f} m/s²  {flag}")
    trials_df = pd.DataFrame(rows)
    summary = summarize(trials_df)
    trials_df.to_csv(os.path.join(out_dir, "trials.csv"), index=False)
    summary.to_csv(os.path.join(out_dir, "summary.csv"))
    plot(trials_df, summary, os.path.join(out_dir, "benchmark.png"))
    return trials_df, summary, out_dir


def summarize(df: pd.DataFrame) -> pd.DataFrame:
    g = df.groupby("scenario", sort=False)
    out = pd.DataFrame({
        "trials": g.size(),
        "pass_rate_%": (100 * (1 - g.collided.mean())).round(1),
        "median_reaction_s": g.reaction_s.median().round(2),
        "worst_closest_m": g.min_gap_m.min().round(2),
        "median_closest_m": g.min_gap_m.median().round(2),
        "mean_peak_decel": g.peak_decel.mean().round(2),
        "emergency_%": (100 * g.emergency.mean()).round(0),
    })
    total = pd.DataFrame({"trials": [len(df)], "pass_rate_%": [round(100 * (1 - df.collided.mean()), 1)],
                          "median_reaction_s": [round(df.reaction_s.median(), 2)],
                          "worst_closest_m": [round(df.min_gap_m.min(), 2)],
                          "median_closest_m": [round(df.min_gap_m.median(), 2)],
                          "mean_peak_decel": [round(df.peak_decel.mean(), 2)],
                          "emergency_%": [round(100 * df.emergency.mean())]}, index=["ALL"])
    return pd.concat([out, total])


def plot(df: pd.DataFrame, summary: pd.DataFrame, path: str):
    names = [n for n in summary.index if n != "ALL"]
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.2), dpi=110)
    fig.suptitle(f"Crunch-time scenario benchmark — {len(df)} trials, pass rate {summary.loc['ALL', 'pass_rate_%']:.0f}%",
                 fontsize=14, fontweight="bold")
    pos = np.arange(len(names))
    for ax, col, title, unit in ((axes[0], "reaction_s", "Reaction time", "s"),
                                 (axes[1], "min_gap_m", "Closest approach", "m"),
                                 (axes[2], "peak_decel", "Hardest braking", "m/s²")):
        data = [df.loc[df.scenario == n, col].dropna().to_numpy() for n in names]
        ax.boxplot(data, vert=False, widths=.55, patch_artist=True,
                   boxprops=dict(facecolor="#a8dadc", color="#1d3557"), medianprops=dict(color="#e63946", lw=2))
        ax.set_yticks(pos + 1, names if ax is axes[0] else [""] * len(names), fontsize=9)
        ax.set_title(f"{title} ({unit})")
        ax.grid(axis="x", alpha=.3)
        if col == "min_gap_m":
            ax.axvline(0, color="#e63946", lw=1.2, ls="--")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
