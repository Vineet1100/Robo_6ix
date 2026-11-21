"""Ziegler–Nichols ultimate gain/period along a whole route, vectorised with NumPy.

For every 5 m coordinate the speed plant is linearised as
    G(s) = K e^{-theta s} / ((tau s + 1)(tau_a s + 1))
and the phase-crossover frequency (angle = -180 deg) is found by bisection on all points at once.
"""
from __future__ import annotations

import numpy as np

from .vehicle import VEHICLE as V

G = 9.81


def traction_force(v):
    return np.minimum(V["Fmax"], V["Pmax"] / np.maximum(v, .5))


def ultimate(v, m, mu, mode):
    v0 = np.maximum(np.asarray(v, float), 1.5)
    mu = np.asarray(mu, float)
    b = 2 * V["c"] * v0
    if mode == "traction":
        u0 = np.clip((V["c"] * v0 ** 2 + V["crr"] * m * G) / traction_force(v0), 0, 1)
        b = b + np.where(v0 > V["Pmax"] / V["Fmax"], V["Pmax"] / v0 ** 2 * u0, 0)
        kf = traction_force(v0)
    else:
        kf = mu * G * m
    K, tau, ta, th = kf / b, m / b, V["ta"], V["th"] + V["Ts"] / 2
    lo, hi = np.full_like(v0, 1e-3), np.full_like(v0, 200.0)
    for _ in range(60):
        w = (lo + hi) / 2
        below = np.arctan(w * tau) + np.arctan(w * ta) + w * th < np.pi
        lo, hi = np.where(below, w, lo), np.where(below, hi, w)
    w = (lo + hi) / 2
    Ku = np.sqrt(1 + (w * tau) ** 2) * np.sqrt(1 + (w * ta) ** 2) / K
    return Ku, 2 * np.pi / w


def route_profile(s, k, mu, mass, vlim, decel=1.5, lat_acc=2.2):
    """s: distance along route, k: |curvature| (max in a ±10 m window), mu: grip, at 5 m spacing.
    vlim is the speed limit in m/s, either one number or one value per point (each street has its own limit)."""
    s, k, mu = (np.asarray(a, float) for a in (s, k, mu))
    vlim = np.broadcast_to(np.asarray(vlim, float), s.shape)
    v = np.where(k > .03, np.sqrt(lat_acc / np.maximum(k, 1e-6)), vlim)
    v = np.minimum(np.minimum(v, vlim), np.sqrt(2 * decel * np.maximum(s[-1] - s, 0)) + .5)
    kt, tu = ultimate(v, mass, mu, "traction")
    kb, _ = ultimate(v, mass, mu, "brake")
    return dict(s=s.round(1).tolist(), v=v.round(2).tolist(), mu=mu.round(2).tolist(),
                kt=kt.round(3).tolist(), kb=kb.round(3).tolist(), tu=tu.round(3).tolist())
