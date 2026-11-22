"""Crunch-time tests: hazards injected on the robotaxi's route a few seconds ahead."""
from __future__ import annotations

import math
import random

from .agents import make_car, make_script, choose_plan
from .geometry import DIR, P, W, clamp, horiz, lane_coord, right_v, sgn_d
from .routing import DS
from .traffic import CTRL, CYCLE


def frame_at(world, D):
    """A straight, mid-block point on the route ~D metres ahead, in road-aligned coordinates."""
    r, e = world.route, world.ego
    if r is None:
        return None
    q = min(r.n - 1, e.pi + int(round(D / DS)))
    for _ in range(30):
        if q >= r.n - 4:
            return None
        tx, ty = r.tx[q], r.ty[q]
        if abs(tx) < .995 and abs(ty) < .995:
            q += 4
            continue
        d = (0 if tx > 0 else 2) if abs(tx) > .9 else (1 if ty > 0 else 3)
        hx = horiz(d)
        px, py = r.px[q], r.py[q]
        al, perp = (px, py) if hx else (py, px)
        if abs(al - round(al / P) * P) < 8:
            q += 4
            continue
        return dict(hx=hx, s=sgn_d(d), al=al, rc=round(perp / P) * P, rr=right_v(d)[1 if hx else 0], d=d, q=q)
    return None


def PT(F, al, pp, sp=None, pause=None):
    o = dict(x=al, y=pp) if F["hx"] else dict(x=pp, y=al)
    if sp:
        o["sp"] = sp
    if pause:
        o["pause"] = pause
    return o


def _clear_lane(world, x, y, r):
    world.actors = [o for o in world.actors if o.kind != "car" or getattr(o, "tram", False) or math.hypot(o.x - x, o.y - y) > r]


def jaywalker(w):
    F = frame_at(w, w.ego.v * 2.4 + 6)
    if not F:
        return False
    a, s, rc, rr = F["al"], F["s"], F["rc"], F["rr"]
    w.actors.append(make_script("ped", [PT(F, a, rc + rr * 5.6), PT(F, a, rc - rr * 5.4, 1.5),
                                        PT(F, clamp(a + random.uniform(-25, 25), 2, W - 2), rc - rr * 5.4, 1.3)], w.t))
    return True


def child_ball(w):
    F = frame_at(w, w.ego.v * 3 + 6)
    if not F:
        return False
    a, s, rc, rr = F["al"], F["s"], F["rc"], F["rr"]
    w.actors.append(make_script("ball", [PT(F, a, rc - rr * 6), PT(F, a + s * .5, rc + rr * 9, 4.5)], w.t))
    w.actors.append(make_script("child", [PT(F, a - s, rc - rr * 6.5, None, 1.0), PT(F, a, rc + rr * 5.8, 2.8),
                                          PT(F, a + s * .5, rc + rr * 8, 1)], w.t))
    return True


def raccoon(w):
    F = frame_at(w, w.ego.v * 3 + 8)
    if not F:
        return False
    a, s, rc, rr = F["al"], F["s"], F["rc"], F["rr"]
    w.actors.append(make_script("raccoon", [PT(F, a, rc + rr * 7), PT(F, a, rc + rr * 2, 4.5, 1.8),
                                            PT(F, a - s * 2, rc - rr * 9, 4.5)], w.t, fur=(111, 106, 100)))
    return True


def dog_dart(w):
    F = frame_at(w, w.ego.v * 2.5 + 7)
    if not F:
        return False
    a, s, rc, rr = F["al"], F["s"], F["rc"], F["rr"]
    w.actors.append(make_script("dog", [PT(F, a, rc - rr * 5.5), PT(F, a + s * 1.5, rc - rr * 1.5, 4.5),
                                        PT(F, a - s, rc + rr, 4, .6), PT(F, a + s * 2, rc + rr * 2.6, 4),
                                        PT(F, a, rc + rr * 6, 5)], w.t))
    return True


def lead_brake(w):
    F = frame_at(w, max(w.ego.v * 1.4, 9) + 4.5)
    if not F:
        return False
    p = PT(F, F["al"], lane_coord(F["d"], F["rc"]))
    _clear_lane(w, p["x"], p["y"], 10)
    c = make_car(p["x"], p["y"], F["d"], v=w.ego.v, script="brake", brake_at=w.t + .9, color=(200, 200, 200),
                 scripted=True, born=w.t)
    c.plan = choose_plan(c)
    if c.plan["d2"] != c.d:
        cr = c.plan["cross"]
        if 0 <= cr + F["s"] * P <= W:
            c.plan = dict(cross=cr, d2=c.d, at=cr + F["s"] * 3)
    w.actors.append(c)
    return True


def red_light_runner(w):
    r, e = w.route, w.ego
    if r is None:
        return False
    nd = next((n for n in r.nodes if 14 < (n["q"] - e.pi) * DS < 58), None)
    if nd is None:
        return False
    q0 = max(e.pi, nd["q"] - 16)
    tx, ty = r.tx[q0], r.ty[q0]
    if abs(tx) < .99 and abs(ty) < .99:
        return False
    d = (0 if tx > 0 else 2) if abs(tx) > .9 else (1 if ty > 0 else 3)
    hx = horiz(d)
    rc = nd["j"] * P if hx else nd["i"] * P
    rr = right_v(d)[1 if hx else 0]
    cross = nd["i"] * P if hx else nd["j"] * P
    dist = (nd["q"] - e.pi) * DS
    off = 11 * dist / max(e.v, 4)
    side, d2 = 1, (d + 3) % 4
    if not 2 <= rc + rr * off <= W - 2:
        side, d2 = -1, (d + 1) % 4
        if not 2 <= rc - rr * off <= W - 2:
            return False
    lc, pp = lane_coord(d2, cross), rc + side * rr * off
    x, y = (lc, pp) if hx else (pp, lc)
    _clear_lane(w, x, y, 12)
    ct = CTRL.get((nd["i"], nd["j"]))
    if ct is not None and ct.type == "signal":       # the taxi gets the green, the runner has a red to run
        ct.offset = ((1 if hx else 24) - w.t) % CYCLE
    c = make_car(x, y, d2, v=11.0, v0=11.1, fix_v0=True, no_yield=True, no_yield_clear=True, color=(184, 50, 44), scripted=True, born=w.t)
    c.force_straight_at = rc
    c.plan = choose_plan(c)
    w.actors.append(c)
    return True


SCENARIOS = [("Jaywalker", jaywalker), ("Child chasing ball", child_ball), ("Raccoon freezes in lane", raccoon),
             ("Dog darts across", dog_dart), ("Lead car slams brakes", lead_brake), ("Cross-traffic runs light", red_light_runner)]


def run_scenario(w, i):
    name, fn = SCENARIOS[i]
    ok = fn(w)
    w.log(f"TEST > {name} injected ahead" if ok else f"Not enough road ahead for '{name}' — try again in a moment", "info")
    return ok
