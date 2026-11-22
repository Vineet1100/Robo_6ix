"""Vehicle dynamics, pure-pursuit steering, PID speed control and Ziegler–Nichols tuning."""
from __future__ import annotations

import math
import random
from collections import deque

import numpy as np

from .geometry import DIR, N, P, VLIM, ang_diff, clamp, heading_to_dir, wet_zones
from .routing import DS

G = 9.81
WHEELBASE = 2.8


class VehicleParams:
    m0 = 1850.0     # kg, empty
    c_aero = .527   # 0.5 * rho * Cd * A
    crr = .012
    f_max = 5600.0  # N traction limit
    p_max = 60000.0  # W motor power
    tau_act = .25   # actuator lag (s)
    dead = .08      # actuator dead time (s)


VEH = VehicleParams()

# Wet patches (puddles) reduce tyre grip. Same seeded layout as the browser (wetZones in 05-dynamics-pid.js).
WET = wet_zones()


def mu_at(x, y):
    for (wx, wy, r) in WET:
        if (x - wx) ** 2 + (y - wy) ** 2 < r * r:
            return .55
    return .9


def grade_at(x, y, th):
    """Road grade (rise/run) along heading th from a smooth synthetic elevation field."""
    gx = 3 / 80 * math.cos(x / 80) * math.cos(y / 95) + 1.5 / 50 * math.cos((x + y) / 50)
    gy = -3 / 95 * math.sin(x / 80) * math.sin(y / 95) + 1.5 / 50 * math.cos((x + y) / 50)
    return gx * math.cos(th) + gy * math.sin(th)


def traction_force(v):
    return min(VEH.f_max, VEH.p_max / max(v, .5))


def resist(v, gr, m):
    return VEH.c_aero * v * v + (VEH.crr * m * G if v > .05 else 0) + m * G * gr


# ------------------------------------------------------------------ Ziegler–Nichols
RULES = {
    "classic": ("Classic Ziegler–Nichols", .6, .5, .125),
    "pessen": ("Pessen integral rule", .7, .4, .15),
    "some": ("ZN — some overshoot", .33, .5, .333),
    "none": ("ZN — no overshoot", .2, .5, .333),
}


def zn_at(v, m, mu, mode, Ts):
    """Linearise the speed plant at (v, m, mu, mode) and return ultimate gain/period.

    G(s) = K e^{-theta s} / ((tau s + 1)(tau_a s + 1)); solve phase(G(jw)) = -180 deg by bisection.
    """
    v0 = max(v, 1.5)
    b = 2 * VEH.c_aero * v0
    if mode == "traction" and v0 > VEH.p_max / VEH.f_max:
        u0 = clamp(resist(v0, 0, m) / traction_force(v0), 0, 1)
        b += VEH.p_max / (v0 * v0) * u0
    kf = traction_force(v0) if mode == "traction" else mu * G * m
    K, tp, ta, th = kf / b, m / b, VEH.tau_act, VEH.dead + Ts / 2
    lo, hi = 1e-3, 200.0
    for _ in range(60):
        w = (lo + hi) / 2
        if math.atan(w * tp) + math.atan(w * ta) + w * th < math.pi:
            lo = w
        else:
            hi = w
    w = (lo + hi) / 2
    Ku = math.sqrt(1 + (w * tp) ** 2) * math.sqrt(1 + (w * ta) ** 2) / K
    return dict(K=K, tau=tp, tau_a=ta, theta=th, Ku=Ku, Tu=2 * math.pi / w, w=w, mode=mode, v=v0, m=m, mu=mu)


def rule_gains(z, rule):
    _, kp, ti, td = RULES[rule]
    Kp, Ti, Td = kp * z["Ku"], ti * z["Tu"], td * z["Tu"]
    return dict(Kp=Kp, Ki=Kp / Ti, Kd=Kp * Td, Ti=Ti, Td=Td)


class SpeedPID:
    """Reference-speed PID with model feed-forward, filtered derivative-on-measurement,
    conditional-integration anti-windup and gain scheduling from Ziegler–Nichols."""

    def __init__(self, Ts):
        self.Ts = Ts
        self.vref = self.I = self.dF = self.v_prev = 0.0
        self.u = self.u_del = self.u_act = 0.0
        self.q = deque()
        self.P = self.D = self.ff = self.e = 0.0
        self.sat = self.clamped = False
        self.rule = "classic"
        self.ff_on = True
        self.zn = None
        self.g = dict(Kp=1, Ki=1, Kd=0, Ti=1, Td=0)
        self.zn_t = -1.0
        self.mu, self.grade, self.mode, self.a_cmd = .9, 0.0, "traction", 0.0

    def reset(self, v):
        self.vref = self.v_prev = v

    def step(self, ego, mass, a_cmd, t, dt):
        m = mass
        mu, gr = mu_at(ego.x, ego.y), grade_at(ego.x, ego.y, ego.th)
        self.mu, self.grade = mu, gr
        a_cmd = max(a_cmd, -mu * G * .95)
        self.a_cmd = a_cmd
        self.vref = clamp(self.vref + a_cmd * dt, max(0.0, ego.v - 2.5), min(VLIM, ego.v + 2.5))
        f_req = m * a_cmd + resist(ego.v, gr, m)
        ff = (f_req / traction_force(ego.v) if f_req >= 0 else f_req / (mu * G * m)) if self.ff_on else 0.0
        self.mode = "traction" if f_req >= 0 else "brake"
        if t - self.zn_t >= .2 or self.zn is None:
            self.zn_t = t
            self.zn = zn_at(ego.v, m, mu, self.mode, self.Ts)
            self.g = rule_gains(self.zn, self.rule)
        Kp, Ki, Kd = self.g["Kp"], self.g["Ki"], self.g["Kd"]
        e = self.vref - ego.v
        dv = (ego.v - self.v_prev) / dt
        self.v_prev = ego.v
        tf = max(.03, self.g["Td"] / 8)
        self.dF += (dv - self.dF) * min(1.0, dt / tf)
        P, D = Kp * e, -Kd * self.dF
        u_raw = ff + P + self.I + D
        u_sat = clamp(u_raw, -1, 1)
        self.sat = u_sat != u_raw
        self.clamped = self.sat and (e > 0) == (u_raw > 0)
        if not self.clamped:
            self.I = clamp(self.I + Ki * e * dt, -.6, .6)
        uc = u_sat
        if self.vref < .05 and ego.v < .2:     # standstill brake hold
            uc = min(uc, -.35)
            self.I *= .97
        self.u, self.P, self.D, self.ff, self.e = uc, P, D, ff, e
        # actuator: dead time + first-order lag
        self.q.append((t, uc))
        while self.q and self.q[0][0] <= t - VEH.dead:
            self.u_del = self.q.popleft()[1]
        self.u_act += (self.u_del - self.u_act) * min(1.0, dt / VEH.tau_act)
        ua = self.u_act
        F = ua * traction_force(ego.v) if ua >= 0 else ua * mu * G * m
        a = (F - resist(ego.v, gr, m)) / m
        if ego.v < .05 and ua <= 0:
            net = -resist(0, gr, m)
            a = max(0.0, (net + F) / m) if net > -F else 0.0
            if a <= 0:                     # brakes on at a standstill: held, no creeping over the line
                ego.v = 0.0
        ego.v = clamp(ego.v + a * dt, 0, VLIM + .5)
        ego.acc = a


# ------------------------------------------------------------------ pure pursuit + bicycle model
def pure_pursuit(ego, route, dt):
    r = route
    if r is None:
        return
    lo, hi = max(0, ego.pi - 10), min(r.n - 1, ego.pi + 60)
    d2 = (r.px[lo:hi + 1] - ego.x) ** 2 + (r.py[lo:hi + 1] - ego.y) ** 2
    best = lo + int(np.argmin(d2))
    ego.pi = best
    ego.cte = (ego.x - r.px[best]) * (-r.ty[best]) + (ego.y - r.py[best]) * r.tx[best]
    ego.head_err = ang_diff(math.atan2(r.ty[best], r.tx[best]), ego.th)
    Ld = clamp(2.5 + .5 * ego.v, 3, 10)
    c, s = math.cos(ego.th), math.sin(ego.th)
    rx, ry = ego.x - c * WHEELBASE / 2, ego.y - s * WHEELBASE / 2
    lat = ego.lat + ego.pull
    q = np.arange(best, r.n)
    tx = r.px[q] - r.ty[q] * lat
    ty = r.py[q] + r.tx[q] * lat
    far = np.nonzero(np.hypot(tx - rx, ty - ry) >= Ld)[0]
    k = far[0] if len(far) else len(q) - 1
    tx, ty = float(tx[k]), float(ty[k])
    dx, dy = tx - rx, ty - ry
    lx, ly = dx * c + dy * s, -dx * s + dy * c
    ld = max(math.hypot(lx, ly), .5)
    kap = 2 * ly / (ld * ld)
    tgt = clamp(math.atan(WHEELBASE * kap), -.6, .6)
    ego.steer += clamp(tgt - ego.steer, -1.6 * dt, 1.6 * dt)
    ego.pp = dict(Ld=ld, alpha=math.atan2(ly, lx), kappa=kap, lx=tx, ly=ty, rx=rx, ry=ry)


def bicycle_step(ego, dt):
    c, s = math.cos(ego.th), math.sin(ego.th)
    rx, ry = ego.x - c * WHEELBASE / 2 + ego.v * c * dt, ego.y - s * WHEELBASE / 2 + ego.v * s * dt
    ego.omega = ego.v / WHEELBASE * math.tan(ego.steer)
    ego.th = (ego.th + ego.omega * dt + math.pi) % (2 * math.pi) - math.pi
    ego.x, ego.y = rx + math.cos(ego.th) * WHEELBASE / 2, ry + math.sin(ego.th) * WHEELBASE / 2
    ego.hd = ego.th
    ego.d = heading_to_dir(ego.th)


def ego_nominal(ego, route, cars):
    """Nominal longitudinal command: IDM car-following along the path, curve speed, stop at the target,
    the street's speed limit and any traffic light / stop sign / yield holding the car (ego.rule_acc)."""
    v = ego.v
    vd = getattr(ego, "vlim", None) or VLIM
    acc = 1.6 * (1 - (v / vd) ** 4)
    r = route
    if r is None:
        return -2.0
    i0 = ego.pi
    seg = slice(i0, min(r.n, i0 + 100), 2)
    PX, PY = r.px[seg], r.py[seg]
    gap, lv = 1e9, 0.0
    for o in cars:
        if abs(o.x - ego.x) > 50 or abs(o.y - ego.y) > 50 or len(PX) == 0:
            continue
        oh = DIR[o.d]
        e = (PX - o.x) ** 2 + (PY - o.y) ** 2
        b = int(np.argmin(e))
        q = i0 + 2 * b
        dot = oh[0] * r.tx[q] + oh[1] * r.ty[q]
        if e[b] <= 1.8 ** 2 and dot >= .5:              # a car ahead going our way: follow it
            g = (q - i0) * DS - ego.L / 2 - o.L / 2
            if g < gap:
                gap, lv = g, o.v * dot
            continue
        # any other car whose body sits on our path (e.g. one still turning off it): stop short of it
        hl = o.L / 2 - .6
        for k in (-hl, 0.0, hl):
            e = (PX - (o.x + oh[0] * k)) ** 2 + (PY - (o.y + oh[1] * k)) ** 2
            b = int(np.argmin(e))
            if e[b] <= 1.7 ** 2:
                g = 2 * b * DS - ego.L / 2 - .6
                if g < gap:
                    gap, lv = g, max(0.0, o.v * dot)
    if gap < 1e8:
        ss = 2 + v * 1.3 + v * (v - lv) / (2 * math.sqrt(1.6 * 2.5))
        acc = 1.6 * (1 - (v / vd) ** 4 - (max(ss, 0) / max(gap, .1)) ** 2)
    rule_acc = getattr(ego, "rule_acc", None)
    if rule_acc is not None:
        acc = min(acc, rule_acc)
    ks = np.abs(r.k[i0:min(r.n, i0 + 70):2])
    for j, k in enumerate(ks):
        if k < .03:
            continue
        vc = math.sqrt(2.2 / k)
        dist = max(2 * j * DS - 1, .5)
        if v > vc:
            acc = min(acc, (vc * vc - v * v) / (2 * dist))
    rem = (r.n - 1 - i0) * DS
    if rem < v * v / 3 + 10:
        vt = math.sqrt(max(0.0, 3 * (rem - .4)))
        acc = min(acc, clamp((vt - v) * 1.8, -6, 1.2))
    return clamp(acc, -6, 2)
