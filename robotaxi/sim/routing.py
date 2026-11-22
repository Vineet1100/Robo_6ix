"""Street-grid routing (Dijkstra over intersection/heading states) and lane-centred paths."""
from __future__ import annotations

import heapq
import math
import random
from dataclasses import dataclass, field

import numpy as np

from .geometry import (DIR, HNAME, LO, N, NODE, P, VNAME, W, ang_diff, clamp, horiz, in_grid,
                      lane_coord, sgn_d)

DS = 0.5   # path resampling step (m)


@dataclass
class Dest:
    a: tuple      # (i, j) node the destination edge starts from
    dd: int       # travel direction along that edge
    frac: float   # position along the edge (0..1)


@dataclass
class Route:
    px: np.ndarray
    py: np.ndarray
    tx: np.ndarray
    ty: np.ndarray
    k: np.ndarray            # signed curvature
    n: int
    length: float
    nodes: list              # [{i, j, q, turn, onto}]
    dest: Dest
    name: str
    kind: str = "Destination"
    zn: dict = field(default_factory=dict)


def node_ahead(x, y, d):
    hx = horiz(d)
    s = sgn_d(d)
    al, perp = (x, y) if hx else (y, x)
    k = math.ceil((al + 3) / P) if s > 0 else math.floor((al - 3) / P)
    r = round(perp / P)
    return (k, r) if hx else (r, k)


def dest_point(dest: Dest):
    (ai, aj), dd = dest.a, dest.dd
    hx = horiz(dd)
    al = (ai * P if hx else aj * P) + sgn_d(dd) * P * dest.frac
    lc = lane_coord(dd, aj * P if hx else ai * P)
    return (al, lc) if hx else (lc, al)


def dest_name(dest: Dest):
    (ai, aj), dd = dest.a, dest.dd
    if horiz(dd):
        i2 = ai + DIR[dd][0]
        return f"{HNAME[aj]}, between {VNAME[min(ai, i2)]} & {VNAME[max(ai, i2)]}"
    j2 = aj + DIR[dd][1]
    return f"{VNAME[ai]}, between {HNAME[min(aj, j2)]} & {HNAME[max(aj, j2)]}"


def dest_from_point(x, y) -> Dest:
    """Snap an arbitrary point to the nearest curb lane (right-hand side faces the point)."""
    vi, hj = round(x / P), round(y / P)
    dv, dh = abs(x - vi * P), abs(y - hj * P)
    if dh <= dv:
        k = int(clamp(math.floor(x / P), 0, N - 1))
        f = clamp((x - k * P) / P, .2, .8)
        j = int(clamp(hj, 0, N))
        return Dest((k, j), 0, f) if y >= j * P else Dest((k + 1, j), 2, 1 - f)
    k = int(clamp(math.floor(y / P), 0, N - 1))
    f = clamp((y - k * P) / P, .2, .8)
    i = int(clamp(vi, 0, N))
    return Dest((i, k), 1, f) if x <= i * P else Dest((i, k + 1), 3, 1 - f)


def random_dest() -> Dest:
    a = (random.randint(0, N), random.randint(0, N))
    ds = [dd for dd in range(4) if in_grid(a[0] + DIR[dd][0], a[1] + DIR[dd][1])]
    return Dest(a, random.choice(ds), random.uniform(.3, .7))


def snap_sidewalk(x, y):
    vi, hj = round(x / P), round(y / P)
    if abs(y - hj * P) <= abs(x - vi * P):
        y = hj * P + (NODE if y >= hj * P else -NODE)
    else:
        x = vi * P + (NODE if x >= vi * P else -NODE)
    return clamp(x, -NODE, W + NODE), clamp(y, -NODE, W + NODE)


def snap_pickup(x, y):
    """Snap to a mid-block sidewalk point inside the service area, or None."""
    vi, hj = round(x / P), round(y / P)
    if abs(y - hj * P) <= abs(x - vi * P):
        k = clamp(math.floor(x / P), 0, N - 1)
        p = (clamp(x, k * P + 9, k * P + P - 9), clamp(hj, 0, N) * P + (NODE if y >= hj * P else -NODE))
    else:
        k = clamp(math.floor(y / P), 0, N - 1)
        p = (clamp(vi, 0, N) * P + (NODE if x >= vi * P else -NODE), clamp(y, k * P + 9, k * P + P - 9))
    if p[0] < 0 or p[0] > W or p[1] < 0 or p[1] > W:
        return None
    return p


def plan_route(x, y, d, dest: Dest):
    start = node_ahead(x, y, d)
    if not in_grid(*start):
        return None
    prev = (start[0] - DIR[d][0], start[1] - DIR[d][1])
    hx = horiz(d)
    al = x if hx else y
    dx, dy = dest_point(dest)
    dal = dx if hx else dy
    if prev == tuple(dest.a) and dest.dd == d and sgn_d(d) * (dal - al) > 3:
        return build_path(x, y, d, [], dest)
    s0 = (start[0], start[1], d)
    dist = {s0: 0.0}
    prv = {}
    pq = [(0.0, s0)]
    goal = None
    while pq:
        c, st = heapq.heappop(pq)
        if c > dist.get(st, 1e18):
            continue
        i, j, din = st
        if (i, j) == tuple(dest.a) and dest.dd != (din + 2) % 4 and in_grid(i + DIR[dest.dd][0], j + DIR[dest.dd][1]):
            goal = st
            break
        for dout in range(4):
            if dout == (din + 2) % 4:
                continue
            ni, nj = i + DIR[dout][0], j + DIR[dout][1]
            if not in_grid(ni, nj):
                continue
            pen = 0 if dout == din else (4 if dout == (din + 1) % 4 else 12)
            ns, nc = (ni, nj, dout), c + P + pen
            if nc < dist.get(ns, 1e18):
                dist[ns] = nc
                prv[ns] = st
                heapq.heappush(pq, (nc, ns))
    if goal is None:
        return None
    seq = []
    st = goal
    while st is not None:
        seq.append(st)
        st = prv.get(st)
    seq.reverse()
    return build_path(x, y, d, seq, dest)


def build_path(x, y, d, seq, dest: Dest) -> Route:
    hx = horiz(d)
    lc = lane_coord(d, round((y if hx else x) / P) * P)
    ctrl = [(x, lc) if hx else (lc, y)]
    turn_at = []
    for k, (ni, nj, din) in enumerate(seq):
        dout = seq[k + 1][2] if k + 1 < len(seq) else dest.dd
        if dout == din:
            turn_at.append((ni, nj, None, None))
            continue
        if horiz(din):
            cy, cx = lane_coord(din, nj * P), lane_coord(dout, ni * P)
        else:
            cx, cy = lane_coord(din, ni * P), lane_coord(dout, nj * P)
        right = dout == (din + 1) % 4
        R = 4.8 if right else 7.5
        b1 = (cx - DIR[din][0] * R, cy - DIR[din][1] * R)
        b2 = (cx + DIR[dout][0] * R, cy + DIR[dout][1] * R)
        ctrl.append(b1)
        for t in range(1, 12):
            u = t / 12
            ctrl.append(((1 - u) ** 2 * b1[0] + 2 * (1 - u) * u * cx + u * u * b2[0],
                         (1 - u) ** 2 * b1[1] + 2 * (1 - u) * u * cy + u * u * b2[1]))
        ctrl.append(b2)
        turn_at.append((ni, nj, "right" if right else "left", HNAME[nj] if horiz(dout) else VNAME[ni]))
    ctrl.append(dest_point(dest))
    c = np.array(ctrl, dtype=float)
    seg = np.hypot(np.diff(c[:, 0]), np.diff(c[:, 1]))
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    L = float(cum[-1])
    n = max(3, int(L / DS) + 1)
    s = np.minimum(np.arange(n) * DS, L)
    px = np.interp(s, cum, c[:, 0])
    py = np.interp(s, cum, c[:, 1])
    ax = np.gradient(px)
    ay = np.gradient(py)
    nrm = np.hypot(ax, ay)
    nrm[nrm == 0] = 1
    tx, ty = ax / nrm, ay / nrm
    th = np.arctan2(ty, tx)
    kk = np.zeros(n)
    dth = (th[2:] - th[:-2] + np.pi) % (2 * np.pi) - np.pi
    kk[1:-1] = dth / (2 * DS)
    # match each intersection to the path IN ORDER: a route that loops round a block passes the same one twice
    nodes, q0 = [], 0
    for (ni, nj, turn, onto) in turn_at:
        d2 = (px - ni * P) ** 2 + (py - nj * P) ** 2
        q = q0
        while q < n - 1 and d2[q] > 81:          # first point within 9 m of the intersection centre...
            q += 1
        while q < n - 1 and d2[q + 1] <= d2[q]:  # ...then its closest point on this pass
            q += 1
        q0 = min(n - 1, q + 1)
        nodes.append(dict(i=ni, j=nj, q=int(q), turn=turn, onto=onto))
    r = Route(px, py, tx, ty, kk, n, L, nodes, dest, dest_name(dest))
    from .traffic import annotate_route
    annotate_route(r)          # per-point speed limits + stop lines of every intersection on the route
    return r
