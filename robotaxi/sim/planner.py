"""Perception (tracks + confidence), prediction, probabilistic collision risk and the cost-based
manoeuvre selector. Risk evaluation is vectorised over candidate actions and time with NumPy."""
from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass, field

import numpy as np

from .agents import CLSNAME, CONF0, SIG, SIZE_R
from .geometry import DIR, RANGE, VLIM, clamp, on_road
from .routing import DS

HORIZON, PDT = 4.0, .1
NS = int(round(HORIZON / PDT))
T = np.arange(NS + 1) * PDT

ACTIONS = [  # key, label, accel (None = nominal), lateral offset, comfort cost
    ("CRUISE", "Cruise / follow", None, 0.0, 0.0),
    ("SLOW", "Ease off", -1.5, 0.0, 1.2),
    ("BRAKE", "Firm brake", -4.0, 0.0, 4.0),
    ("EMERGENCY", "Emergency brake", -8.0, 0.0, 11.0),
    ("NUDGE_L", "Nudge left", -1.0, -1.1, 3.5),
    ("NUDGE_R", "Nudge right", -1.0, .9, 3.0),
]
EGO_OFF = np.array([-1.2, 0.0, 1.2])   # ego modelled as 3 circles of radius 1.0


@dataclass
class Track:
    o: object
    dist: float
    pts: np.ndarray            # (NS+1, 2) predicted centre
    offs: np.ndarray           # circle offsets along heading
    hx: float = 0.0
    hy: float = 0.0
    r: float = .35
    s0: float = .3
    k: float = .1
    fwd: float = 0.0
    lat: float = 0.0
    closing: float = 0.0
    p_no: float = 0.0
    p_plan: float = 0.0
    ttc: float | None = None


@dataclass
class Decision:
    act: tuple = ACTIONS[0]
    results: list = field(default_factory=list)   # [(action, Rmax, cost, traj)]
    tracks: list = field(default_factory=list)
    max_no: float = 0.0
    crit: Track | None = None
    latency_ms: float = 0.0
    traj: np.ndarray | None = None


def object_model(o):
    veh = o.kind == "car"
    if veh:
        hx, hy = DIR[o.d]
        vx, vy = hx * o.v, hy * o.v
        ax = ay = 0.0
        t_stop = 1e9
        if o.acc < 0:
            ax, ay, t_stop = hx * o.acc, hy * o.acc, o.v / -o.acc
        tt = np.minimum(T, t_stop)
        pts = np.stack([o.x + vx * tt + .5 * ax * tt * tt, o.y + vy * tt + .5 * ay * tt * tt], axis=1)
        hl = max(1.4, o.L / 2 - .95)
        nc = max(3, int(math.ceil(2 * hl / 1.2)) + 1)
        return pts, np.linspace(-hl, hl, nc), hx, hy, .95, SIG["car"][0], SIG["car"][1]
    s0, kr, ks = SIG[o.kind]
    road = on_road(o.x, o.y)
    still = math.hypot(o.vx, o.vy) < .05 and not road
    pts = np.stack([o.x + o.vx * T, o.y + o.vy * T], axis=1)
    return pts, np.zeros(1), 0.0, 0.0, SIZE_R[o.kind], s0, kr if road else (.04 if still else ks)


def ego_traj(ego, route, act, nominal, mu):
    """Predicted ego centres (NS+1, 2) and headings (NS+1, 2) for one candidate action."""
    _, _, a_set, lat_t, _ = act
    pl = ego.pull
    v, lat, s = ego.v, ego.lat + pl, ego.pi * DS
    c0, s0 = math.cos(ego.th), math.sin(ego.th)
    pts = np.zeros((NS + 1, 2)); hs = np.zeros((NS + 1, 2))
    ox = oy = 0.0
    if route is not None:
        q = ego.pi
        ox = ego.x - (route.px[q] - route.ty[q] * lat)
        oy = ego.y - (route.py[q] + route.tx[q] * lat)
    vs = 0.0
    s_start = s
    for i in range(NS + 1):
        t = i * PDT
        if route is not None:
            q = min(route.n - 1, int(round(s / DS)))
            f = max(0.0, 1 - t / 1.5)
            pts[i] = (route.px[q] - route.ty[q] * lat + ox * f, route.py[q] + route.tx[q] * lat + oy * f)
            hs[i] = (c0, s0) if i == 0 else (route.tx[q], route.ty[q])
        else:
            dd = s - s_start
            pts[i] = (ego.x + c0 * dd, ego.y + s0 * dd); hs[i] = (c0, s0)
        a = ego.acc if t < .3 else max(nominal if a_set is None else min(a_set, nominal), -mu * 9.81 * .95)
        v = clamp(v + a * PDT, 0, VLIM)
        s += v * PDT
        vs += v
        lat += clamp(lat_t + pl - lat, -1.2 * PDT, 1.2 * PDT)
    return pts, hs, vs / (NS + 1)


def plan(world):
    t0 = time.perf_counter()
    ego = world.ego
    tracks = []
    c, s = math.cos(ego.th), math.sin(ego.th)
    evx, evy = c * ego.v, s * ego.v
    for o in world.actors:
        if o.boarding:
            continue
        d = math.hypot(o.x - ego.x, o.y - ego.y)
        if d > RANGE:
            continue
        tgt = CONF0[o.kind] - .28 * (d / RANGE) ** 1.6
        o.conf = tgt if o.conf is None else clamp(o.conf + (tgt + (random.random() - .5) * .04 - o.conf) * .3, .3, .995)
        pts, offs, hx, hy, r, s0, k = object_model(o)
        tk = Track(o, d, pts, offs, hx, hy, r, s0, k)
        dx, dy = o.x - ego.x, o.y - ego.y
        tk.fwd, tk.lat = dx * c + dy * s, -dx * s + dy * c
        ovx, ovy = (DIR[o.d][0] * o.v, DIR[o.d][1] * o.v) if o.kind == "car" else (o.vx, o.vy)
        tk.closing = -(dx * (ovx - evx) + dy * (ovy - evy)) / max(d, .1)
        tracks.append(tk)

    nominal = ego.nominal
    mu = world.pid.mu
    trajs = [ego_traj(ego, world.route, a, nominal, mu) for a in ACTIONS]
    E = np.stack([tr[0] for tr in trajs])           # (A, T, 2)
    H = np.stack([tr[1] for tr in trajs])           # (A, T, 2)
    EC = E[:, :, None, :] + H[:, :, None, :] * EGO_OFF[None, None, :, None]    # (A, T, 3, 2)
    A = len(ACTIONS)
    risk = np.zeros((A, len(tracks)))
    d0 = np.zeros((A, len(tracks)))
    ttc = [None] * len(tracks)
    w = 1 - .11 * T
    for j, tk in enumerate(tracks):
        OC = tk.pts[:, None, :] + np.array([tk.hx, tk.hy])[None, None, :] * tk.offs[None, :, None]  # (T, c, 2)
        diff = EC[:, :, :, None, :] - OC[None, :, None, :, :]
        dist = np.sqrt((diff ** 2).sum(-1)).min(axis=(2, 3)) - 1.0 - tk.r     # (A, T)
        sig = tk.s0 + tk.k * T
        p = np.where(dist <= 0, 1.0, np.exp(-np.maximum(dist, 0) ** 2 / (2 * sig ** 2)))
        risk[:, j] = (p * w).max(axis=1)
        d0[:, j] = dist[:, 0]
        hit = np.nonzero(dist[0] <= .2)[0]
        ttc[j] = float(T[hit[0]]) if len(hit) else None

    mine = world.ride.req.rider if (world.ride.req and world.ride.phase == "to_pickup") else None
    cost_mask = np.array([not (tk.o is mine and tk.o.mode == "wait") for tk in tracks], dtype=bool)
    results = []
    for a_i, act in enumerate(ACTIONS):
        rr = risk[a_i][cost_mask] if len(tracks) else np.zeros(0)
        Rmax = float(rr.max()) if len(rr) else 0.0
        Rsum = float(rr.sum()) if len(rr) else 0.0
        vref = max(getattr(ego, "vlim", VLIM) or VLIM, 3.0)       # progress measured against this street's limit
        cost = 160 * Rmax + 25 * Rsum + act[4] + 16 * (1 - min(trajs[a_i][2] / vref, 1.0)) + (2 if act[3] else 0)
        if act is not world.decision.act and act[0] not in ("BRAKE", "EMERGENCY"):
            cost += 3.0   # switching penalty: stops dithering between manoeuvres
        results.append((act, Rmax, cost, trajs[a_i][0]))
    best = min(results, key=lambda r: r[2])
    cur = next((r for r in results if r[0] is world.decision.act), None)
    if cur is not None and best is not cur and cur[1] < .25 and best[2] > cur[2] - 1.5 and ego.v > .5:   # no stickiness at a standstill
        best = cur
    bi = results.index(best)
    for j, tk in enumerate(tracks):
        tk.p_no = 1 - (1 - risk[0, j]) * (1 - .15 * math.exp(-max(d0[0, j], 0) / 2))
        tk.p_plan = 1 - (1 - risk[bi, j]) * (1 - .15 * math.exp(-max(d0[bi, j], 0) / 2))
        tk.ttc = ttc[j]
    tracks.sort(key=lambda tk: (-tk.p_no, tk.dist))
    crit = tracks[0] if tracks and tracks[0].p_no > .35 else None
    prev = world.decision.act
    dec = Decision(best[0], results, tracks, tracks[0].p_no if tracks else 0.0, crit,
                   (time.perf_counter() - t0) * 1000, best[3])
    if best[0] is not prev:
        if best[0][0] == "EMERGENCY":
            world.stats["emergency"] += 1
        if best[0][0] not in ("CRUISE", "SLOW") and tracks:
            ctk = tracks[0]
            extra = f", TTC {ctk.ttc:.1f} s" if ctk.ttc is not None else ""
            world.log(f"{best[0][1].upper()} — {CLSNAME[ctk.o.kind]} #{ctk.o.id} @ {ctk.dist:.1f} m, "
                      f"P={ctk.p_no * 100:.0f}%{extra} ({dec.latency_ms:.2f} ms)",
                      "bad" if best[0][0] == "EMERGENCY" else "warn")
    return dec
