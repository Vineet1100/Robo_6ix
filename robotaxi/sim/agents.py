"""Traffic agents: cars, TTC streetcars, pedestrians, children, dogs, cats, raccoons and scripted hazards.

NPC vehicles use the Intelligent Driver Model (IDM); the obstacle search is vectorised with NumPy.
"""
from __future__ import annotations

import math
import random

import numpy as np

from .traffic import STOPL, limit_kmh, npc_stop_gap, uturn_blocked, walk_need, walk_ok
from .geometry import (DIR, LO, N, NODE, P, VLIM, W, ang_diff, clamp, horiz, lane_coord, right_v,
                      sgn_d)

CAR_COLORS = [(216, 216, 216), (29, 31, 34), (138, 143, 150), (161, 42, 42), (36, 70, 122), (231, 227, 218),
              (59, 79, 58), (199, 164, 58), (90, 46, 79), (240, 240, 240), (74, 77, 82)]
SHIRTS = [(217, 83, 79), (59, 125, 216), (242, 242, 242), (45, 45, 45), (226, 177, 60), (92, 184, 92),
          (155, 89, 182), (230, 126, 34), (26, 188, 156)]
FURS = [(139, 90, 43), (217, 195, 160), (43, 43, 43), (160, 116, 63), (232, 226, 214)]

CLS = dict(car="VEH", ped="PED", child="CHILD", dog="DOG", cat="CAT", raccoon="RACC", ball="BALL")
CLSNAME = dict(car="Vehicle", ped="Pedestrian", child="Child", dog="Dog", cat="Cat", raccoon="Raccoon", ball="Ball / debris")
SIZE_R = dict(car=.95, ped=.35, child=.3, dog=.45, cat=.25, raccoon=.35, ball=.15)
BOX = dict(ped=.95, child=.75, dog=1.15, cat=.65, raccoon=1.0, ball=.5)
SIG = dict(car=(.25, .12, .12), ped=(.3, .35, .1), child=(.3, .55, .18), dog=(.35, .7, .2),
           cat=(.3, .6, .2), raccoon=(.35, .7, .25), ball=(.15, .3, .3))   # (sigma0, k_road, k_sidewalk)
CONF0 = dict(car=.985, ped=.955, child=.91, dog=.9, cat=.82, raccoon=.88, ball=.76)


class Actor:
    _next = 1

    def __init__(self, kind, x, y, **kw):
        self.id = Actor._next
        Actor._next += 1
        self.kind, self.x, self.y = kind, float(x), float(y)
        self.vx = self.vy = 0.0
        self.hd = 0.0
        self.conf = None
        self.scripted = self.done = self.boarding = self.is_rider = self.user = False
        self.born = 0.0
        self.hit_t = None
        self.__dict__.update(kw)


# ------------------------------------------------------------------ vehicles
def make_car(x, y, d, **kw):
    a = Actor("car", x, y, d=d, hd=math.atan2(DIR[d][1], DIR[d][0]), v=random.uniform(6, 10), acc=0.0,
              L=random.uniform(4.3, 5.0), Wd=random.uniform(1.8, 2.0), color=random.choice(CAR_COLORS),
              plan=None, v0=random.uniform(8.5, VLIM), vf=random.uniform(.88, 1.05), fix_v0=False, cs=None,
              stop_t=0.0, ignore_until=0.0, tram=False,
              no_yield=False, no_yield_clear=False, force_straight_at=None, script=None, brake_at=0.0, held=None)
    a.__dict__.update(kw)
    return a


def choose_plan(a):
    s, hx = sgn_d(a.d), horiz(a.d)
    al, perp = (a.x, a.y) if hx else (a.y, a.x)
    k = math.ceil((al + 2.6) / P) if s > 0 else math.floor((al - 2.6) / P)
    cross, rc = k * P, round(perp / P) * P
    straight_ok = 0 <= cross + s * P <= W and 0 <= cross <= W
    if a.tram:
        # streetcars run the full line, then re-enter at the far end (loop to the depot)
        return dict(cross=cross, d2=a.d, at=cross + s * 3) if straight_ok else dict(cross=cross, d2=a.d, at=cross + s * 10, wrap=True)
    opts = []
    if straight_ok:
        opts.append((a.d, .5))
    if 0 <= cross <= W:
        for d2 in ((a.d + 1) % 4, (a.d + 3) % 4):
            if 0 <= rc + sgn_d(d2) * P <= W:
                opts.append((d2, .25))
    if not opts:
        return dict(cross=al, d2=(a.d + 2) % 4, at=al)
    if a.force_straight_at is not None and abs(a.force_straight_at - cross) < 1 and straight_ok:
        d2 = a.d
        a.force_straight_at = None
    else:
        d2 = random.choices([o[0] for o in opts], [o[1] for o in opts])[0]
    at = cross + s * 3 if d2 == a.d else cross + LO * right_v(d2)[0 if hx else 1]
    return dict(cross=cross, d2=d2, at=at)


def step_vehicle(a, dt, world=None):
    if a.plan is None:
        a.plan = choose_plan(a)
    a.v = max(0.0, a.v + a.acc * dt)
    s, hx = sgn_d(a.d), horiz(a.d)
    if hx:
        a.x += s * a.v * dt
    else:
        a.y += s * a.v * dt
    al = a.x if hx else a.y
    if s * (al - a.plan["at"]) >= 0:
        if world is not None and a.plan["d2"] == (a.d + 2) % 4 and uturn_blocked(world, a):
            if hx:                                  # wait until the other lane is clear
                a.x -= s * a.v * dt
            else:
                a.y -= s * a.v * dt
            a.v = 0.0
            return
        if a.plan.get("wrap"):
            start = -14.0 if s > 0 else W + 14.0     # re-enter from beyond the map edge
            if hx:
                a.x = start
            else:
                a.y = start
            a.plan = None
            return
        d2 = a.plan["d2"]
        if d2 != a.d:
            if hx:
                a.x = a.plan["at"]
            else:
                a.y = a.plan["at"]
            if d2 == (a.d + 2) % 4:       # U-turn at the edge of the map (streetcars)
                rc = round((a.y if hx else a.x) / P) * P
                if hx:
                    a.y = lane_coord(d2, rc)
                else:
                    a.x = lane_coord(d2, rc)
            a.d = d2
        a.plan = None
        if a.no_yield and a.no_yield_clear:
            a.no_yield = False
    th = math.atan2(DIR[a.d][1], DIR[a.d][0])
    a.hd += ang_diff(a.hd, th) * min(1.0, dt * 7)


DX = np.array([d[0] for d in DIR], float)
DY = np.array([d[1] for d in DIR], float)


def npc_accels(cars, walkers, ego, t, dt, world=None):
    """Vectorised IDM obstacle search for all NPC vehicles; writes car.acc.
    With a world, every car also obeys the traffic lights, stop / yield signs and street speed limits."""
    if not cars:
        return
    if world is not None:
        for c in cars:
            if not c.fix_v0:
                lim = limit_kmh(c.d, c.x, c.y) / 3.6
                c.v0 = min(9.0, lim) if c.tram else lim * c.vf
    C = len(cars)
    X = np.array([c.x for c in cars]); Y = np.array([c.y for c in cars])
    D = np.array([c.d for c in cars]); V = np.array([c.v for c in cars]); L = np.array([c.L for c in cars])
    V0 = np.array([c.v0 for c in cars])
    noy = np.array([c.no_yield for c in cars]); ign = np.array([t < c.ignore_until for c in cars])
    hx, hy = DX[D], DY[D]
    rx, ry = DX[(D + 1) % 4], DY[(D + 1) % 4]
    INF = 1e9
    gaps, leads = [], []

    # other vehicles
    dx = X[None, :] - X[:, None]; dy = Y[None, :] - Y[:, None]
    al = dx * hx[:, None] + dy * hy[:, None]; lat = dx * rx[:, None] + dy * ry[:, None]
    same = D[None, :] == D[:, None]; opp = ((D[None, :] + 2) % 4) == D[:, None]
    np.fill_diagonal(al, -1)
    m_same = same & (np.abs(lat) < 1.8) & (al > 0) & (al < 45)
    gaps.append(np.where(m_same, al - L[:, None] / 2 - L[None, :] / 2, INF)); leads.append(np.broadcast_to(V[None, :], (C, C)))
    m_perp = ~same & ~opp & (np.abs(lat) < 2.6) & (al > 0) & (al < 14) & ~noy[:, None] & ~ign[:, None]
    gaps.append(np.where(m_perp, al - L[:, None] / 2 - 1.2, INF)); leads.append(np.zeros((C, C)))

    # pedestrians and animals
    if walkers:
        WX = np.array([w.x for w in walkers]); WY = np.array([w.y for w in walkers])
        dx = WX[None, :] - X[:, None]; dy = WY[None, :] - Y[:, None]
        al = dx * hx[:, None] + dy * hy[:, None]; lat = dx * rx[:, None] + dy * ry[:, None]
        m = (al > 0) & (al < 25) & (np.abs(lat) < 1.9)
        gaps.append(np.where(m, al - L[:, None] / 2 - .4, INF)); leads.append(np.zeros_like(al))

    # the robotaxi: three points along its body, any orientation
    c, s = math.cos(ego.th), math.sin(ego.th)
    pts = np.array([[ego.x + c * k, ego.y + s * k] for k in (-1.25, 0, 1.25)])
    dx = pts[None, :, 0] - X[:, None]; dy = pts[None, :, 1] - Y[:, None]
    al = dx * hx[:, None] + dy * hy[:, None]; lat = dx * rx[:, None] + dy * ry[:, None]
    # only points in front of the car's front half count as "ahead" (one beside it is not a leader)
    m = (al > L[:, None] / 2 - .3) & (al < 45) & (np.abs(lat) < 2.05) & ~noy[:, None]
    ev = np.maximum(0, ego.v * (c * hx + s * hy))
    gaps.append(np.where(m, al - L[:, None] / 2 - 1.05, INF)); leads.append(np.broadcast_to(ev[:, None], (C, 3)))

    # yield at the next intersection if the robotaxi is in / about to enter it
    hz = (D % 2 == 0)
    al0 = np.where(hz, X, Y); perp = np.where(hz, Y, X); sg = np.where((D == 0) | (D == 1), 1, -1)
    cr = np.where(sg > 0, np.ceil(al0 / P) * P, np.floor(al0 / P) * P)
    dI = sg * (cr - al0); rc = np.round(perp / P) * P
    cx = np.where(hz, cr, rc); cy = np.where(hz, rc, cr)
    de = np.hypot(ego.x - cx, ego.y - cy); toward = (cx - ego.x) * c + (cy - ego.y) * s > 0
    dF = dI - L / 2                      # distance from the vehicle's FRONT to the intersection centre
    if world is None:
        m = ~noy & (dF > .3) & (dF < 20) & ((de < 9) | ((de < 18) & toward & (ego.v > .3)))
        gaps.append(np.where(m, np.maximum(dF - 4.7, .1), INF)[:, None]); leads.append(np.zeros((C, 1)))
    else:
        # with traffic controls the rules decide; only a robotaxi already in the box is waited for
        m = ~noy & (dF > STOPL - 1.5) & (dF < 20) & (de < 9)
        gaps.append(np.where(m, np.maximum(dF - STOPL, .1), INF)[:, None]); leads.append(np.zeros((C, 1)))
        gaps.append(np.array([[npc_stop_gap(world, car)] for car in cars])); leads.append(np.zeros((C, 1)))

    G = np.concatenate(gaps, axis=1); LV = np.concatenate(leads, axis=1)
    idx = np.argmin(G, axis=1); gap = G[np.arange(C), idx]; lv = LV[np.arange(C), idx]
    a_max, b = 1.6, 2.5
    free = a_max * (1 - (V / V0) ** 4)
    ss = 2 + V * 1.3 + V * (V - lv) / (2 * math.sqrt(a_max * b))
    acc = np.where(gap < 1e8, a_max * (1 - (V / V0) ** 4 - (np.maximum(ss, 0) / np.maximum(gap, .1)) ** 2), free)
    for i, car in enumerate(cars):
        a = acc[i]
        if car.plan is not None and car.plan["d2"] != car.d:
            dist = sgn_d(car.d) * (car.plan["at"] - (car.x if horiz(car.d) else car.y))
            if dist < 22 and car.v > 4.5:
                a = min(a, (4.5 ** 2 - car.v ** 2) / (2 * max(dist, 1)))
        if car.script == "brake":          # scripted hard stop (crunch-time test)
            if t < car.brake_at:
                a = 0.0
            elif car.held is None:
                a = -7.5
                if car.v <= 0:
                    car.held = t
            elif t - car.held < 4:
                a = -1.0
            else:
                car.script = None
        a = clamp(a, -9, 2)
        if car.v < .1 and a < .1:
            car.stop_t += dt
            if car.stop_t > 4:
                car.ignore_until, car.stop_t = t + 3, 0.0
        else:
            car.stop_t = 0.0
        car.acc = a


# ------------------------------------------------------------------ walkers
def node_pos(n):
    i, j, qx, qy = n
    return i * P + qx * NODE, j * P + qy * NODE


def node_ok(n):
    x, y = node_pos(n)
    return 0 <= n[0] <= N and 0 <= n[1] <= N and 0 <= x <= W and 0 <= y <= W


def neighbors(n):
    i, j, qx, qy = n
    c = [((i, j, -qx, qy), .13), ((i, j, qx, -qy), .13), ((i + qx, j, -qx, qy), .37), ((i, j + qy, qx, -qy), .37)]
    return [o for o in c if node_ok(o[0])]


def rand_node():
    while True:
        n = (random.randint(0, N), random.randint(0, N), random.choice((-1, 1)), random.choice((-1, 1)))
        if node_ok(n):
            return n


SPD = dict(ped=(1.1, 1.6), child=(1.0, 1.4), dog=(1.4, 2.0), cat=(.6, 1.0))


def make_walker(kind):
    n = rand_node()
    t = random.choice(neighbors(n))[0]
    (x0, y0), (x1, y1) = node_pos(n), node_pos(t)
    f = random.random()
    return Actor(kind, x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, sp=random.uniform(*SPD[kind]), node=n, target=t,
                 wait=0.0, mode="graph", col=random.choice(SHIRTS), fur=random.choice(FURS), phase=random.random() * 6)


def make_script(kind, pts, t, **kw):
    a = Actor(kind, pts[0]["x"], pts[0]["y"], mode="script", path=pts, pi=1, wait=pts[0].get("pause", 0),
              born=t, scripted=True, col=random.choice(SHIRTS), fur=(139, 90, 43), phase=random.random() * 6)
    a.hd = math.atan2(pts[1]["y"] - pts[0]["y"], pts[1]["x"] - pts[0]["x"])
    a.__dict__.update(kw)
    return a


def _move_to(a, tx, ty, sp, dt):
    dx, dy = tx - a.x, ty - a.y
    dd = math.hypot(dx, dy)
    if dd <= sp * dt + 1e-6:
        a.x, a.y, a.vx, a.vy = tx, ty, 0.0, 0.0
        return True
    a.vx, a.vy = dx / dd * sp, dy / dd * sp
    a.x += a.vx * dt
    a.y += a.vy * dt
    a.hd = math.atan2(dy, dx)
    return False


def step_walker(a, t, dt, world=None):
    m = a.mode
    if m == "wait":
        a.vx = a.vy = 0.0
        return
    if m == "walkto":
        if math.hypot(a.goto[0] - a.x, a.goto[1] - a.y) < .25:
            a.arrived, a.vx, a.vy = True, 0.0, 0.0
            return
        a.arrived = False
        _move_to(a, a.goto[0], a.goto[1], getattr(a, "sp", 1.3), dt)
        return
    if a.wait > 0:
        a.wait -= dt
        a.vx = a.vy = 0.0
        return
    if m == "script":
        if a.pi >= len(a.path):
            a.vx = a.vy = 0.0
            a.done = True
            return
        p = a.path[a.pi]
        if _move_to(a, p["x"], p["y"], p["sp"], dt):
            a.pi += 1
            a.wait = p.get("pause", 0)
        return
    if m == "graze":
        if getattr(a, "gt", None) is None:
            a.gt = (random.uniform(a.bx0, a.bx1), random.uniform(a.by0, a.by1))
        if _move_to(a, a.gt[0], a.gt[1], .6, dt):
            a.gt, a.wait = None, random.uniform(2, 7)
        return
    # sidewalk graph walking
    if a.target is None:
        nb = neighbors(a.node)
        a.target = random.choices([o[0] for o in nb], [o[1] for o in nb])[0]
        a.xok = False
    if world is not None and not getattr(a, "xok", True):
        w = walk_need(a.node, a.target)            # crossing a street: walk signal + look both ways
        if w and not walk_ok(world, w):
            a.vx = a.vy = 0.0
            a.walk_wait = True
            return
        a.xok, a.walk_wait = True, False
    sp = a.sp * (1 + .3 * math.sin(t * 3 + a.phase)) if a.kind == "dog" else a.sp
    tx, ty = node_pos(a.target)
    if _move_to(a, tx, ty, sp, dt):
        a.node, a.target = a.target, None
        a.wait = random.uniform(.5, 3) if random.random() < .25 else 0.0
