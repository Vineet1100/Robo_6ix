"""Export a headless simulator run with pandas (CSV) and draw an analysis report with matplotlib.
The traffic-rule check reuses robotaxi.analytics.rules_check, the same pandas code the web app uses."""
from __future__ import annotations

import os
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from ..analytics import rules_check  # noqa: E402
from .geometry import CITY, HNAME, M, N, P, VNAME, W  # noqa: E402


def summarize(dfs) -> pd.DataFrame:
    tel, rides, zn = dfs["telemetry"], dfs["rides"], dfs["ziegler_nichols"]
    rows = {}
    if len(tel):
        rows["sim time (s)"] = tel.t.iloc[-1]
        rows["mean speed (km/h)"] = tel.v.mean() * 3.6
        moving = tel[tel.v > 1]
        rows["speed tracking RMS error (m/s)"] = float(np.sqrt(((moving.v_ref - moving.v) ** 2).mean())) if len(moving) else np.nan
        rows["mean planner latency (ms)"] = tel.plan_ms.mean()
        rows["time in each manoeuvre (%)"] = ", ".join(f"{k} {v:.0f}" for k, v in (tel.action.value_counts(normalize=True) * 100).items())
        rows["max odometry drift, raw (m)"] = tel.odo_raw_err.max()
        rows["max odometry error, fused (m)"] = tel.odo_fused_err.max()
    if len(rides):
        rows["pickups"] = len(rides)
        rows["mean door distance with RF (m)"] = rides.door_err_rf_m.mean()
        rows["mean door distance GPS-only (m)"] = rides.door_err_gps_only_m.mean()
        rows["mean rider wait (s)"] = rides.wait_s.mean()
    rules = rules_check(tel) if len(tel) else None
    if rules:
        rows["stops at red / amber lights"] = rules["red_stops"]
        rows["stops at stop signs"] = rules["sign_stops"]
        rows["yields (yield signs, left turns)"] = rules["yield_holds"]
        rows["moving time over the speed limit (%)"] = rules["pct_over_limit"]
        rows["mean speed by limit (km/h)"] = ", ".join(f"{k} zone {v}" for k, v in rules["speed_by_limit"].items())
    if len(zn):
        rows["Ku range along roads"] = f"{zn.Ku.min():.2f} – {zn.Ku.max():.2f}"
        rows["Tu range along roads (s)"] = f"{zn.Tu.min():.2f} – {zn.Tu.max():.2f}"
    return pd.DataFrame({"value": rows})


def export_run(world, out_dir=None):
    """Write telemetry / Ziegler-Nichols / rides / events CSVs, a summary and report.png; returns (folder, summary)."""
    out_dir = out_dir or os.path.join("runs", time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    dfs = world.dataframes()
    for name, df in dfs.items():
        df.to_csv(os.path.join(out_dir, f"{name}.csv"), index=False)
    summ = summarize(dfs)
    for k, v in world.rule_summary().items():          # the simulator's own live counters, incl. violations
        summ.loc[f"rules: {k.replace('_', ' ')}"] = v
    summ.loc["collisions (taxi moving)"] = world.stats["collisions"]
    summ.loc["trips completed"] = world.stats["trips"]
    summ.to_csv(os.path.join(out_dir, "summary.csv"))
    plot_report(world, dfs, os.path.join(out_dir, "report.png"))
    return out_dir, summ


def plot_report(world, dfs, path):
    tel, zn, rides = dfs["telemetry"], dfs["ziegler_nichols"], dfs["rides"]
    fig = plt.figure(figsize=(16, 10), dpi=110)
    fig.suptitle(f"Robotaxi run report — {CITY.name}", fontsize=15, fontweight="bold")
    gs = fig.add_gridspec(3, 3, hspace=.45, wspace=.28)

    ax = fig.add_subplot(gs[:, 0])
    ax.set_title("Driven path vs odometry (last trip)", pad=48)
    for k in range(N + 1):
        ax.axhline(k * P, color="#bbb", lw=6, zorder=0)
        ax.axvline(k * P, color="#bbb", lw=6, zorder=0)
    for j, nm in enumerate(HNAME):
        ax.text(W + 4, j * P, nm, fontsize=7, va="center")
    for i, nm in enumerate(VNAME):
        ax.text(i * P, -6, nm, fontsize=7, ha="center", rotation=90, va="bottom")
    if len(tel):
        ax.plot(tel.x, tel.y, color="#2a9d8f", lw=1, alpha=.6, label="all driving")
    od = world.odo
    if od.trail_true:
        for arr, col, lab in ((od.trail_true, "k", "ground truth"), (od.trail_raw, "#e83e8c", "wheel+gyro dead reckoning"),
                              (od.trail_fus, "#17becf", "fused estimate")):
            a = np.array(arr)
            ax.plot(a[:, 0], a[:, 1], color=col, lw=1.6, label=lab)
    if len(rides):
        ax.scatter(rides.rider_x, rides.rider_y, marker="*", s=90, color="#7b2cbf", label="pickups", zorder=5)
    ax.set_xlim(-M / 2, W + 45)
    ax.set_ylim(W + M / 2, -M / 2)
    ax.set_aspect("equal")
    ax.legend(fontsize=7, loc="lower left")

    def last(df, sec=60):
        return df[df.t > df.t.iloc[-1] - sec] if len(df) else df

    ax = fig.add_subplot(gs[0, 1])
    ax.set_title("PID speed tracking (last 60 s)")
    t = last(tel)
    if len(t):
        ax.plot(t.t, t.v_ref * 3.6, "k--", lw=1, label="v_ref")
        ax.plot(t.t, t.v * 3.6, color="#2a9d8f", lw=1.6, label="speed")
        ax2 = ax.twinx()
        ax2.fill_between(t.t, 0, t.u, where=t.u >= 0, color="#3ddc84", alpha=.3, step="mid")
        ax2.fill_between(t.t, 0, t.u, where=t.u < 0, color="#ff4d5e", alpha=.3, step="mid")
        ax2.set_ylim(-1.05, 1.05)
        ax2.set_ylabel("throttle(+)/brake(−)")
    ax.set_ylabel("km/h")
    ax.legend(fontsize=7, loc="upper left")

    ax = fig.add_subplot(gs[1, 1])
    ax.set_title("PID terms + feed-forward (last 60 s)")
    if len(t):
        for col, c in (("P", "#e9c46a"), ("I", "#7b2cbf"), ("D", "#00b4d8"), ("ff", "#888"), ("u", "k")):
            ax.plot(t.t, t[col].clip(-1.5, 1.5), color=c, lw=1.2 if col != "u" else 1.6, label=col)
    ax.legend(fontsize=7, ncol=5)

    ax = fig.add_subplot(gs[2, 1])
    ax.set_title("Ziegler–Nichols ultimate gain Ku at each 5 m coordinate")
    if len(zn):
        for mode, c in (("traction", "#4f8dff"), ("brake", "#ff8a3d")):
            z = zn[zn["mode"] == mode]
            ax.scatter(z.t, z.Ku, s=8, color=c, label=f"{mode}")
        wet = zn[zn.mu < .6]
        ax.scatter(wet.t, wet.Ku, s=26, facecolors="none", edgecolors="#1d3557", label="wet road (μ 0.55)")
        ax.set_xlabel("sim time (s)")
        ax.legend(fontsize=7)

    ax = fig.add_subplot(gs[0, 2])
    ax.set_title("Collision risk & manoeuvres")
    if len(tel):
        ax.plot(tel.t, tel.max_risk, color="#e63946", lw=.8)
        ax.set_ylim(0, 1.02)
        ax.set_ylabel("max P(collision | no action)")
        counts = tel.action.value_counts()
        ins = ax.inset_axes([.55, .5, .43, .45])
        labels = [str(k) for k in counts.index.tolist()]
        ins.barh(np.arange(len(labels)), counts.to_numpy() / counts.to_numpy().sum() * 100, color="#457b9d")
        ins.set_yticks(np.arange(len(labels)), labels)
        ins.tick_params(labelsize=6)
        ins.set_xlabel("% time", fontsize=6)

    ax = fig.add_subplot(gs[1, 2])
    ax.set_title("Pickup accuracy: walk to the door")
    if len(rides):
        x = np.arange(len(rides))
        ax.bar(x - .2, rides.door_err_gps_only_m, .4, color="#ff8a3d", label="GPS only")
        ax.bar(x + .2, rides.door_err_rf_m, .4, color="#2a9d8f", label="GPS + RF (BLE/UWB)")
        ax.set_xticks(x, [str(i) for i in rides.ride_id], fontsize=7, rotation=45)
        ax.set_ylabel("m")
        ax.legend(fontsize=7)

    ax = fig.add_subplot(gs[2, 2])
    ax.axis("off")
    ax.set_title("Summary (pandas)")
    summ = summarize(dfs)
    txt = "\n".join(f"{k}: {v:.2f}" if isinstance(v, (float, np.floating)) else f"{k}: {v}" for k, v in summ["value"].items())
    ax.text(0, 1, txt, va="top", fontsize=8, family="monospace", wrap=True)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
