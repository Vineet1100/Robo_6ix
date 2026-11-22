"""Odometry (wheel encoder + gyro dead reckoning, fused with periodic map fixes) and the
RF particle filter that locates a rider's phone from BLE RSSI + UWB angle-of-arrival."""
from __future__ import annotations

import math
import random

import numpy as np

from .geometry import NODE, P, W, ang_diff


class Odometry:
    def __init__(self):
        self.total = self.trip = 0.0
        self.raw = [0.0, 0.0, 0.0]
        self.fus = [0.0, 0.0, 0.0]
        self.start = (0.0, 0.0, 0.0)
        self.bias = 0.0
        self.scale = 1.0
        self.fix_t = -9.0
        self.last_trail = 0.0
        self.trail_true, self.trail_raw, self.trail_fus = [], [], []
        self.err_raw = self.err_fus = 0.0

    def reset(self, ego):
        self.raw = [ego.x, ego.y, ego.th]
        self.fus = [ego.x, ego.y, ego.th]
        self.start = (ego.x, ego.y, ego.th)
        self.trip = self.last_trail = 0.0
        self.trail_true, self.trail_raw, self.trail_fus = [], [], []
        self.bias = random.uniform(.0025, .005) * random.choice((-1, 1))      # gyro bias (rad/s)
        self.scale = 1 + random.uniform(.01, .015) * random.choice((-1, 1))  # wheel-radius error

    def step(self, ego, t, dt):
        vm = max(0.0, ego.v * self.scale + (random.gauss(0, .04) if ego.v > .05 else 0))
        wm = ego.omega + self.bias + random.gauss(0, .008)
        for p in (self.raw, self.fus):
            p[2] += wm * dt
            p[0] += vm * math.cos(p[2]) * dt
            p[1] += vm * math.sin(p[2]) * dt
        self.total += vm * dt
        self.trip += vm * dt
        if t - self.fix_t > 1:      # map-matched position fix, complementary-filter gain 0.55
            self.fix_t = t
            fx, fy, ft = ego.x + random.gauss(0, .25), ego.y + random.gauss(0, .25), ego.th + random.gauss(0, .01)
            self.fus[0] += .55 * (fx - self.fus[0])
            self.fus[1] += .55 * (fy - self.fus[1])
            self.fus[2] += .55 * ang_diff(self.fus[2], ft)
        self.err_raw = math.hypot(self.raw[0] - ego.x, self.raw[1] - ego.y)
        self.err_fus = math.hypot(self.fus[0] - ego.x, self.fus[1] - ego.y)
        if self.trip - self.last_trail > .8:
            self.last_trail = self.trip
            self.trail_true.append((ego.x, ego.y))
            self.trail_raw.append((self.raw[0], self.raw[1]))
            self.trail_fus.append((self.fus[0], self.fus[1]))
            for tr in (self.trail_true, self.trail_raw, self.trail_fus):
                if len(tr) > 800:
                    del tr[0]


# ------------------------------------------------------------------ RF localisation of the rider
class RF:
    P0 = -41.0         # RSSI at 1 m (dBm)
    n = 2.3            # path-loss exponent
    shadow = 3.5       # dB
    ble_range = 85.0
    uwb_range = 35.0
    aoa_sigma = math.radians(8)


def rssi_at(d):
    return RF.P0 - 10 * RF.n * np.log10(np.maximum(d, .5))


def snap_sidewalk_np(x, y):
    """Vectorised: project points onto the nearest sidewalk walking line."""
    vi, hj = np.round(x / P), np.round(y / P)
    dv, dh = np.abs(x - vi * P), np.abs(y - hj * P)
    hor = dh <= dv
    y2 = np.where(hor, hj * P + np.where(y >= hj * P, NODE, -NODE), y)
    x2 = np.where(hor, x, vi * P + np.where(x >= vi * P, NODE, -NODE))
    return np.clip(x2, -NODE, W + NODE), np.clip(y2, -NODE, W + NODE)


class ParticleFilter:
    def __init__(self, cx, cy, sigma, n=500):
        self.n = n
        x, y = snap_sidewalk_np(cx + np.random.randn(n) * sigma, cy + np.random.randn(n) * sigma)
        self.px, self.py = x, y
        self.w = np.full(n, 1 / n)

    def update(self, ax, ay, rssi, aoa):
        dx, dy = self.px - ax, self.py - ay
        d = np.hypot(dx, dy)
        lik = np.exp(-((rssi - rssi_at(d)) ** 2) / (2 * (RF.shadow + 1.5) ** 2))
        if aoa is not None:
            da = (np.arctan2(dy, dx) - aoa + np.pi) % (2 * np.pi) - np.pi
            lik *= np.exp(-da * da / (2 * (RF.aoa_sigma * 1.6) ** 2)) + .002
        self.w *= lik + 1e-9
        s = self.w.sum()
        if not s > 0:
            self.w[:] = 1 / self.n
            return
        self.w /= s
        if 1 / np.sum(self.w ** 2) < self.n / 2:      # systematic resampling
            pos = (np.random.rand() + np.arange(self.n)) / self.n
            idx = np.minimum(np.searchsorted(np.cumsum(self.w), pos), self.n - 1)
            self.px, self.py = snap_sidewalk_np(self.px[idx] + np.random.randn(self.n) * .35,
                                                self.py[idx] + np.random.randn(self.n) * .35)
            self.w[:] = 1 / self.n

    def estimate(self):
        mx, my = float(np.sum(self.w * self.px)), float(np.sum(self.w * self.py))
        sd = float(math.sqrt(np.sum(self.w * ((self.px - mx) ** 2 + (self.py - my) ** 2))))
        sx, sy = snap_sidewalk_np(np.array([mx]), np.array([my]))
        return float(sx[0]), float(sy[0]), sd
