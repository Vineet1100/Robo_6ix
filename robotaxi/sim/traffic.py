"""Traffic lights, stop signs, yield signs and speed limits for the headless simulator.

This is the Python twin of static/js/13-traffic.js: the same layout (read from the map file), the same
46 s signal cycle and the same right-of-way rules, so a policy tested here behaves the same in the browser.

Every vehicle (NPC cars, streetcars and the robotaxi) runs ctrl_decide(): it looks up the control for its
approach, measures the distance from its front bumper to the stop line, and either holds there or goes.
Holding is done by putting a virtual stopped car on the stop line, so the normal car-following model
(IDM) brakes for it smoothly.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .geometry import CITY, DIR, HNAME, N, NODE, P, VNAME, W, clamp, horiz, in_grid, sgn_d

STOPL = 7.6                       # stop line distance from the centre of an intersection (m)
SIG_G, SIG_Y, SIG_AR = 18.0, 3.0, 2.0
CYCLE = 2 * (SIG_G + SIG_Y + SIG_AR)   # 46 s
WALK_MIN = 9.0                    # a pedestrian only starts crossing with this much green left
AMBER_DECEL = 3.2                 # stop on amber only if it needs less than this (m/s^2)
NAMES = dict(signal="Traffic light", allstop="All-way stop", stop="Stop sign", yield_="Yield sign")

SPEED_H = list(CITY.maxspeed_h) or [40.0] * (N + 1)     # km/h, Queen -> Front
SPEED_V = list(CITY.maxspeed_v) or [40.0] * (N + 1)     # km/h, Spadina -> Yonge


@dataclass
class Control:
    i: int
    j: int
    type: str                     # signal | allstop | stop | yield
    approach: str | None = None   # for stop / yield: which road stops ("h" = east-west traffic)
    offset: float = 0.0           # signal phase offset (s)
    name: str = ""
    queue: list = field(default_factory=list)   # all-way / two-way stop arrival order

    @property
    def key(self):
        return (self.i, self.j)


CTRL: dict[tuple, Control] = {}
for (ci, cj), c in CITY.controls.items():
    CTRL[(ci, cj)] = Control(ci, cj, c["type"], c.get("approach"), float(c.get("offset", 0)),
                             CITY.intersections.get((ci, cj)) or f"{HNAME[cj]} & {VNAME[ci]}")


def axis_of(d):
    return "h" if horiz(d) else "v"


def limit_kmh(d, x, y):
    if horiz(d):
        return SPEED_H[clamp(round(y / P), 0, N)]
    return SPEED_V[clamp(round(x / P), 0, N)]


def approach_rule(c: Control, axis: str) -> str:
    if c.type == "signal":
        return "signal"
    if c.type == "allstop":
        return "stop"
    return c.type if c.approach == axis else "free"


def sig_state(c: Control, axis: str, t: float):
    """(state, seconds left) for one direction of a signal: state is G, Y or R."""
    u = (t + c.offset) % CYCLE
    half = SIG_G + SIG_Y + SIG_AR
    w = u if axis == "h" else (u - half) % CYCLE
    if w < SIG_G:
        return "G", SIG_G - w
    if w < SIG_G + SIG_Y:
        return "Y", SIG_G + SIG_Y - w
    return "R", CYCLE - w


# ------------------------------------------------------------------ geometry helpers
def in_box(o, c, m=STOPL):
    return abs(o.x - c.i * P) < m and abs(o.y - c.j * P) < m


def approach_info(o, c):
    """(axis, perpendicular offset, distance to the centre along the heading: + = approaching)."""
    d = o.d
    if horiz(d):
        return "h", o.y - c.j * P, sgn_d(d) * (c.i * P - o.x)
    return "v", o.x - c.i * P, sgn_d(d) * (c.j * P - o.y)


def _vehicles(world):
    yield world.ego
    for a in world.actors:
        if a.kind == "car":
            yield a


def _near(o, c, r=65):
    return abs(o.x - c.i * P) < r and abs(o.y - c.j * P) < r


def cross_clear(world, c, axis, me, horizon):
    """Safe to enter c from `axis`? Checks the box itself and cross traffic that does not have to stop."""
    for o in _vehicles(world):
        if o is me or not _near(o, c):
            continue
        cs = getattr(o, "cs", None)
        if in_box(o, c, STOPL - .6):
            if o.v > .2 or not (cs and cs["key"] == c.key and not cs["go"]):
                return False
            continue
        ax, perp, dI = approach_info(o, c)
        if ax == axis or abs(perp) > 4.5 or dI < 0:
            continue
        rule = approach_rule(c, ax)
        if rule in ("free", "signal"):
            if o.v > .5 and dI - STOPL < max(o.v, 2.5) * horizon + 5:
                return False
        elif cs and cs["key"] == c.key and cs["go"] and dI < 18:
            return False
    return True


def turn_at(world, o, c):
    if o is world.ego:
        r = world.route
        if r is None:
            return None
        for nd in r.nodes:
            if nd["i"] == c.i and nd["j"] == c.j and nd["q"] >= world.ego.pi - 30:
                return nd["turn"]
        return None
    p = o.plan
    if not p:
        return None
    _, _, dI = approach_info(o, c)
    cr = c.i * P if horiz(o.d) else c.j * P
    if abs(p["cross"] - cr) > 1 or dI < -1:
        return None
    return "left" if p["d2"] == (o.d + 3) % 4 else "right" if p["d2"] == (o.d + 1) % 4 else None


def oncoming_clear(world, c, me):
    """Left turn: wait for oncoming traffic going straight or turning right."""
    for o in _vehicles(world):
        if o is me or not _near(o, c) or o.d != (me.d + 2) % 4:
            continue
        _, perp, dI = approach_info(o, c)
        if abs(perp) > 4.5 or dI < -2 or turn_at(world, o, c) == "left":
            continue
        if in_box(o, c) and o.v > .2:
            return False
        if o.v > .5 and dI - STOPL < max(o.v, 3) * 4 + 4:
            return False
        cs = getattr(o, "cs", None)
        if dI - STOPL < 5 and ((cs and cs["key"] == c.key and cs["go"]) or
                               (c.type == "signal" and sig_state(c, axis_of(o.d), world.t)[0] == "G")):
            return False                  # waiting right at its line with a green / its turn: may go any moment
    return True


def oncoming_left_committed(world, c, me):
    """An oncoming vehicle that has already been given its turn to go left across our path."""
    for o in _vehicles(world):
        if o is me or not _near(o, c) or o.d != (me.d + 2) % 4:
            continue
        cs = getattr(o, "cs", None)
        _, perp, dI = approach_info(o, c)
        if cs and cs["key"] == c.key and cs["go"] and abs(perp) < 4.5 and -2 < dI < 18 and turn_at(world, o, c) == "left":
            return True
    return False


# ------------------------------------------------------------------ the rule engine
def ctrl_decide(world, a, c: Control, axis: str, g: float, turn, visit=None):
    """Shared by every vehicle. g = distance from the front bumper to the stop line.
    `visit` tells repeat visits to the same intersection apart (a route can loop round a block).
    Returns dict(hold, rule, state, why)."""
    t, v = world.t, a.v
    rule = approach_rule(c, axis)
    if rule == "free" and turn != "left":
        return dict(hold=False, rule=rule)
    cs = getattr(a, "cs", None)
    if not cs or cs["key"] != c.key or cs.get("visit") != visit or t - cs.get("t_seen", t) > 5:
        cs = a.cs = dict(key=c.key, visit=visit, t0=-1.0, done=False, go=False, ygo=None, t_done=0.0, lw=False,
                         counted=False, passed=False)
    cs["t_seen"] = t
    if rule == "free":                                   # main road, turning left
        if cs["go"]:
            return dict(hold=False, rule=rule)
        if oncoming_clear(world, c, a):
            cs["go"] = g < 3
            return dict(hold=False, rule=rule)
        return dict(hold=True, rule=rule, why="left")
    if rule == "signal":
        st, rem = sig_state(c, axis, t)
        out = dict(rule=rule, state=st, rem=rem)
        if st == "G":
            cs["ygo"] = None
            if turn == "left" and not cs["go"]:
                if oncoming_clear(world, c, a):
                    cs["go"] = g < 3
                    return dict(out, hold=False)
                cs["lw"] = True
                return dict(out, hold=True, why="left")
            return dict(out, hold=False)
        if cs["go"]:
            return dict(out, hold=False)
        if st == "Y":
            if cs["ygo"] is None:
                cs["ygo"] = g < v * v / (2 * AMBER_DECEL) + .8   # can't stop comfortably: carry on
            if turn == "left" and cs["lw"] and v < .5 and g < 4 and oncoming_clear(world, c, a):
                cs["go"] = True
                return dict(out, hold=False)
            return dict(out, hold=not cs["ygo"], why="amber")
        if cs["ygo"]:
            return dict(out, hold=False)
        return dict(out, hold=True, why="red")
    if rule == "stop":
        if not cs["done"]:
            if v < .3 and g < 4.5:
                if cs["t0"] < 0:
                    cs["t0"] = t
                if t - cs["t0"] >= 1.0:                   # a full stop of at least one second
                    cs["done"], cs["t_done"] = True, t
                    c.queue.append(a)
            else:
                cs["t0"] = -1.0
            return dict(hold=True, rule=rule, why="stop")
        if cs["go"]:
            return dict(hold=False, rule=rule)
        c.queue = [o for o in c.queue if getattr(o, "cs", None) and o.cs["key"] == c.key and not o.cs["go"]]
        q_ok = c.type != "allstop" or (c.queue and c.queue[0] is a) or t - cs["t_done"] > 6
        clear = (cross_clear(world, c, axis, a, 2.3 if t - cs["t_done"] > 8 else 3.5)
                 and (turn != "left" or oncoming_clear(world, c, a)) and not oncoming_left_committed(world, c, a))
        if q_ok and clear:
            cs["go"] = True
            c.queue = [o for o in c.queue if o is not a]
            return dict(hold=False, rule=rule)
        return dict(hold=True, rule=rule, why="gap" if q_ok else "queue")
    # yield
    if cs["go"]:
        return dict(hold=False, rule=rule)
    if cross_clear(world, c, axis, a, 4.0) and (turn != "left" or oncoming_clear(world, c, a)):
        cs["go"] = g < 6
        return dict(hold=False, rule=rule)
    return dict(hold=True, rule=rule, why="yield")


def npc_stop_gap(world, a):
    """Distance from an NPC's front bumper to a stop line it must hold at (1e9 if none)."""
    if a.no_yield:                          # the scripted red-light runner ignores everything
        return 1e9
    s, hx = sgn_d(a.d), horiz(a.d)
    al, perp = (a.x, a.y) if hx else (a.y, a.x)
    cr = math.ceil(al / P) * P if s > 0 else math.floor(al / P) * P
    rc = round(perp / P) * P
    dI = s * (cr - al)
    if dI > 70 or cr < 0 or cr > W:
        return 1e9
    c = CTRL.get((round(cr / P), round(rc / P)) if hx else (round(rc / P), round(cr / P)))
    if c is None:
        return 1e9
    g = dI - STOPL - a.L / 2
    if g < -.6:
        return 1e9                          # past the line: committed
    return max(g, .05) if ctrl_decide(world, a, c, axis_of(a.d), g, turn_at(world, a, c))["hold"] else 1e9


# ------------------------------------------------------------------ the robotaxi
def annotate_route(r):
    """Per-point speed limit (m/s) plus, for each intersection on the route, its control, the index of
    the stop line (qs) and the approach heading."""
    import numpy as np
    jh = np.clip(np.round(r.py / P), 0, N).astype(int)
    iv = np.clip(np.round(r.px / P), 0, N).astype(int)
    kh, kv = np.array(SPEED_H)[jh], np.array(SPEED_V)[iv]
    r.vl = np.where(np.abs(r.tx) > .92, kh, np.where(np.abs(r.ty) > .92, kv, np.minimum(kh, kv))) / 3.6
    for nd in r.nodes:
        cx, cy = nd["i"] * P, nd["j"] * P
        q = nd["q"]
        while q > 0 and max(abs(r.px[q] - cx), abs(r.py[q] - cy)) < STOPL:
            q -= 1
        tx, ty = r.tx[max(0, q - 1)], r.ty[max(0, q - 1)]
        nd["qs"] = q
        nd["ad"] = (0 if tx > 0 else 2) if abs(tx) > abs(ty) else (1 if ty > 0 else 3)
        nd["ctrl"] = CTRL.get((nd["i"], nd["j"]))


def ctrl_count(r, start=0):
    """Controlled intersections still ahead where the taxi may have to stop (an ETA feature)."""
    return sum(1 for nd in r.nodes if nd.get("ctrl") and nd["qs"] >= start and approach_rule(nd["ctrl"], axis_of(nd["ad"])) != "free")


@dataclass
class RuleStats:
    red_stops: int = 0
    sign_stops: int = 0
    yields: int = 0
    violations: int = 0
    over_t: float = 0.0
    drive_t: float = 0.0
    max_over_kmh: float = 0.0


def ego_rules(world, dt):
    """Acceleration limit the traffic rules put on the robotaxi this step; also keeps the rule stats."""
    e, r, st = world.ego, world.route, world.rules
    e.ctl = None
    if r is None or getattr(r, "vl", None) is None:
        e.vlim = limit_kmh(e.d, e.x, e.y) / 3.6
        return 1e9
    q0 = min(e.pi, r.n - 1)
    e.vlim = float(r.vl[q0])
    s_off = (e.x - r.px[q0]) * r.tx[q0] + (e.y - r.py[q0]) * r.ty[q0]
    acc, v = 1e9, e.v
    for nd in r.nodes:
        c = nd.get("ctrl")
        if c is None:
            continue
        g = (nd["qs"] - e.pi) * world.DS - s_off - e.L / 2
        if g < -.6:                                         # just crossed this stop line: check it was legal
            cs = getattr(e, "cs", None)
            if cs and cs["key"] == c.key and cs.get("visit") == nd["q"] and not cs["passed"]:
                cs["passed"] = True
                rl = approach_rule(c, axis_of(nd["ad"]))
                if rl == "signal" and sig_state(c, axis_of(nd["ad"]), world.t)[0] == "R" and not (cs["ygo"] or cs["go"]):
                    st.violations += 1
                    world.log(f"Entered {c.name} on a red light", "bad")
                if rl == "stop" and not cs["done"]:
                    st.violations += 1
                    world.log(f"Rolled through the stop sign at {c.name}", "bad")
            continue
        if g > 75:
            break
        res = ctrl_decide(world, e, c, axis_of(nd["ad"]), g, nd["turn"], visit=nd["q"])
        e.ctl = dict(c=c, g=g, **res)
        cs = getattr(e, "cs", None)
        if res["hold"]:
            gs = g - 1.2                                    # aim to stop 1.2 m before the line
            acc = min(acc, (-3.5 if v > .05 else -1.0) if gs <= 0 else clamp((math.sqrt(2 * 2.2 * gs) - v) * 1.8, -6.5, 1.2))
            if v < .3 and cs and cs["key"] == c.key and not cs["counted"] and res.get("why") in ("red", "amber", "left", "yield"):
                cs["counted"] = True
                if res["why"] in ("red", "amber"):
                    st.red_stops += 1
                    world.log(f"Stopped for a {res['why']} light at {c.name}", "info")
                else:
                    st.yields += 1
                    world.log(f"Yielding at {c.name}", "info")
        if res["rule"] == "stop" and cs and cs["key"] == c.key and cs["done"] and not cs["counted"]:
            cs["counted"] = True
            st.sign_stops += 1
            world.log(f"Full stop at {c.name}", "info")
        if res["rule"] != "free" or nd["turn"] == "left":
            break                                           # only the first stop line that can hold the car
    # slow down in time for a lower limit ahead
    for q in range(e.pi, min(r.n, e.pi + 120), 4):
        vl = float(r.vl[q])
        if vl < v:
            dist = max((q - e.pi) * world.DS - 2, .5)
            acc = min(acc, (vl * vl - v * v) / (2 * dist))
    if e.v > 1:
        st.drive_t += dt
    over = e.v - e.vlim
    if over > .6:
        st.over_t += dt
    st.max_over_kmh = max(st.max_over_kmh, over * 3.6)
    return acc


# ------------------------------------------------------------------ pedestrians
def walk_need(n, t):
    """If walking from sidewalk node n to t crosses a street, describe the crossing (else None)."""
    if t is None or t[0] != n[0] or t[1] != n[1]:
        return None
    c = CTRL.get((n[0], n[1]))
    ax, ay = n[0] * P + n[2] * NODE, n[1] * P + n[3] * NODE
    bx, by = t[0] * P + t[2] * NODE, t[1] * P + t[3] * NODE
    return dict(c=c, axis="h" if t[2] != n[2] else "v", mx=(ax + bx) / 2, my=(ay + by) / 2)


def walk_ok(world, w):
    """Walk signal at lights, and everywhere: don't step out in front of a car that is close and moving."""
    c = w["c"]
    if c is not None and c.type == "signal":
        st, rem = sig_state(c, w["axis"], world.t)
        if st != "G" or rem <= WALK_MIN:
            return False
    for o in _vehicles(world):
        dd = math.hypot(o.x - w["mx"], o.y - w["my"])
        if dd < 3.0 or (dd < 5.5 and o.v > .3):      # a stopped car at the crossing: walk in front of it
            return False
        if o.v > 1 and dd < max(14, o.v * 3.2):
            h = (math.cos(o.th), math.sin(o.th)) if o is world.ego else DIR[o.d]
            if (w["mx"] - o.x) * h[0] + (w["my"] - o.y) * h[1] > -2:
                return False
    return True


def uturn_blocked(world, a):
    """A U-turn swaps lanes in one step, so first make sure nothing is in the way."""
    from .geometry import lane_coord
    d2, hx = (a.d + 2) % 4, horiz(a.d)
    rc = round((a.y if hx else a.x) / P) * P
    lc, al = lane_coord(d2, rc), (a.x if hx else a.y)
    for o in _vehicles(world):
        if o is a:
            continue
        oal, operp = (o.x, o.y) if hx else (o.y, o.x)
        if abs(operp - lc) < 3.6 and abs(oal - al) < a.L / 2 + getattr(o, "L", 4.4) / 2 + 5:
            return True
    return False


def nearest(world, k=6):
    """The k controls closest to the robotaxi, with their live state (for the CLI / reports)."""
    e = world.ego
    rows = []
    for c in sorted(CTRL.values(), key=lambda c: math.hypot(c.i * P - e.x, c.j * P - e.y))[:k]:
        state = ""
        if c.type == "signal":
            h, v = sig_state(c, "h", world.t), sig_state(c, "v", world.t)
            state = f"E-W {h[0]} {h[1]:.0f}s, N-S {v[0]} {v[1]:.0f}s"
        rows.append(dict(name=c.name, control=NAMES.get(c.type, NAMES["yield_"]), state=state))
    return rows


def all_controls_ok():
    """Sanity check used by the tests: every intersection has a control and every approach exists."""
    return all(in_grid(i, j) for (i, j) in CTRL) and len(CTRL) == (N + 1) ** 2
