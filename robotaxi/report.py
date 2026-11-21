"""matplotlib report + zipped CSV export of everything logged so far."""
from __future__ import annotations

import io
import json
import zipfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .analytics import analyze  # noqa: E402
from .citymap import load_map  # noqa: E402


def report_png(fr: dict) -> bytes:
    a = analyze(fr)
    tel, zn, rf, tr, pk = fr["telemetry"], fr["zn_log"], fr["rf"], fr["trips"], fr["pickups"]
    m = load_map()
    P, N = m["block_pitch"], m["blocks_per_side"]
    fig, ax = plt.subplots(2, 3, figsize=(17, 10), dpi=100)
    fig.suptitle("Robotaxi run report — computed in Python (pandas · NumPy · scikit-learn · matplotlib)", fontsize=14, fontweight="bold")

    g = ax[0, 0]
    g.set_title("Path driven + risk hotspots (DBSCAN)")
    for k in range(N + 1):
        g.axhline(k * P, color="#ccc", lw=5, zorder=0)
        g.axvline(k * P, color="#ccc", lw=5, zorder=0)
    if len(tel):
        g.plot(tel.x, tel.y, color="#2a9d8f", lw=.8)
    for h in a["hotspots"]:
        g.add_patch(plt.Circle((h["x"], h["y"]), h["radius"], color="#e63946", alpha=.35))
        g.text(h["x"], h["y"], f'{h["count"]}', ha="center", va="center", fontsize=8, fontweight="bold")
    if len(pk):
        g.scatter(pk.x, pk.y, marker="*", s=90, color="#7b2cbf", label="pickups", zorder=5)
        g.legend(fontsize=7)
    for j, nm in enumerate(m["hnames"]):
        g.text(N * P + 3, j * P, nm, fontsize=6, va="center")
    g.set_xlim(-10, N * P + 55)
    g.set_ylim(N * P + 10, -10)
    g.set_aspect("equal")

    g = ax[0, 1]
    g.set_title("PID speed tracking (last 60 s)")
    if len(tel):
        t = tel[tel.t > tel.t.max() - 60]
        g.plot(t.t, t.vref * 3.6, "k--", lw=1, label="v_ref")
        g.plot(t.t, t.v * 3.6, color="#2a9d8f", lw=1.6, label="speed")
        g2 = g.twinx()
        g2.fill_between(t.t, 0, t.u, where=t.u >= 0, color="#3ddc84", alpha=.3, step="mid")
        g2.fill_between(t.t, 0, t.u, where=t.u < 0, color="#ff4d5e", alpha=.3, step="mid")
        g2.set_ylim(-1.05, 1.05)
        g.legend(fontsize=7)
    g.set_ylabel("km/h")

    g = ax[0, 2]
    g.set_title("Ziegler–Nichols Ku at each 5 m coordinate")
    if len(zn):
        for mode, c in (("traction", "#4f8dff"), ("brake", "#ff8a3d")):
            z = zn[zn["mode"] == mode]
            g.scatter(z.t, z.Ku, s=7, color=c, label=mode)
        w = zn[zn.mu < .6]
        g.scatter(w.t, w.Ku, s=24, facecolors="none", edgecolors="#1d3557", label="wet road")
        g.legend(fontsize=7)
        g.set_xlabel("sim time (s)")

    g = ax[1, 0]
    g.set_title("Radio model calibration (LinearRegression)")
    if len(rf):
        s = rf.sample(min(len(rf), 1500), random_state=0)
        g.scatter(s.d, s.rssi, s=4, alpha=.35, color="#7b2cbf", label="BLE samples")
        dd = np.linspace(1, 90, 100)
        if a["rf_fit"]:
            f = a["rf_fit"]
            g.plot(dd, f["P0"] - 10 * f["n"] * np.log10(dd), "k", lw=2, label=f'fit: P0={f["P0"]}, n={f["n"]}, R²={f["r2"]}')
        g.plot(dd, -41 - 23 * np.log10(dd), "--", color="#2a9d8f", label="simulator truth")
        g.set_xscale("log")
        g.legend(fontsize=7)
    g.set_xlabel("distance (m)")
    g.set_ylabel("RSSI (dBm)")

    g = ax[1, 1]
    g.set_title("Trip-time model (LinearRegression)")
    if len(tr):
        g.scatter(tr.length, tr.duration, color="#457b9d", s=25, label="completed legs")
        if a["eta"]:
            e = a["eta"]
            pred = e["base_s"] + e["per_m"] * tr.length + e["per_turn"] * tr.turns
            if e.get("per_ctrl") is not None and "controls" in tr:
                pred = pred + e["per_ctrl"] * tr.controls.fillna(0)
            g.scatter(tr.length, pred, marker="x", color="#e63946", label=f'model (MAE {e["mae_s"]} s)')
        g.legend(fontsize=7)
    g.set_xlabel("route length (m)")
    g.set_ylabel("duration (s)")

    g = ax[1, 2]
    g.axis("off")
    g.set_title("Summary (pandas)")
    s = a["summary"]
    lines = [f"{k}: {v}" for k, v in s.items() if not isinstance(v, dict)]
    if "time_share" in s:
        lines.append("time share %: " + ", ".join(f"{k} {v}" for k, v in s["time_share"].items()))
    g.text(0, 1, "\n".join(lines), va="top", family="monospace", fontsize=8, wrap=True)
    fig.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format="png")
    plt.close(fig)
    return buf.getvalue()


def export_zip(fr: dict) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        for name, df in fr.items():
            z.writestr(f"{name}.csv", df.to_csv(index=False))
        z.writestr("analysis.json", json.dumps(analyze(fr), indent=2, default=str))
        z.writestr("report.png", report_png(fr))
    return buf.getvalue()
