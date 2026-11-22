"""The World: owns every agent and subsystem, advances the simulation and records data for pandas.

This is the headless Python simulator. It drives the same map, traffic controls and decision logic as the
browser version, but without graphics, so it can run thousands of simulated minutes for testing and
analysis (see simulate.py and tests/)."""
from __future__ import annotations

import math
import random

import numpy as np
import pandas as pd

from . import planner
from .agents import (CLSNAME, SIZE_R, Actor, choose_plan, make_car, make_script, make_walker, npc_accels,
                     step_vehicle, step_walker)
from .geometry import CITY, DIR, LO, N, NODE, P, SW, VLIM, W, clamp, horiz, lane_coord, right_v, sgn_d
from .control import (RULES, SpeedPID, bicycle_step, ego_nominal, mu_at, pure_pursuit, rule_gains, zn_at)
from .localization import Odometry
from .ride import RideManager
from .routing import DS
from . import traffic

DT = 1 / 30


class Ego:
    def __init__(self):
        self.kind = "ego"
        self.x, self.y = 20.0, lane_coord(0, 2 * P)
        self.th = self.hd = 0.0
        self.d = 0
        self.v, self.acc = 8.0, 0.0
        self.lat = self.pull = self.steer = self.omega = 0.0
        self.L, self.Wd = 4.4, 2.05
        self.pi = 0
        self.pp = None
        self.cte = self.head_err = self.nominal = 0.0
        self.vlim, self.rule_acc, self.cs, self.ctl = VLIM, None, None, None


class World:
    def __init__(self, seed=None):
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
        self.t = 0.0
        self.ego = Ego()
        self.actors: list[Actor] = []
        self.route = None
        self.events = []
        self.event_count = 0
        self.route_ver = 0
        self.stats = dict(emergency=0, collisions=0, contacts_while_stopped=0, trips=0, dist=0.0)
        self.rules = traffic.RuleStats()
        self.DS = DS
        for c in traffic.CTRL.values():      # traffic controls are module-level: start each world fresh
            c.queue.clear()
        self.pid = SpeedPID(DT)
        self.pid.reset(self.ego.v)
        self.odo = Odometry()
        self.ride = RideManager(self)
        self.decision = planner.Decision()
        self.plan_timer = 0.0
        self.auto_chaos, self.next_chaos = True, 8.0
        self.telemetry, self.zn_log = [], []
        self._tele_t, self._zn_s = -1.0, -99.0
        self.hist = []          # short rolling history for live charts
        self._spawn()
        self.odo.reset(self.ego)
        self.ride.request()

    # ------------------------------------------------------------- setup
    def _spawn(self):
        e = self.ego
        n = 0
        while n < 22:
            vert = random.random() < .5
            r, seg = random.randint(0, N), random.randint(0, N - 1)
            al = seg * P + random.uniform(10, P - 10)
            d = random.choice((1, 3)) if vert else random.choice((0, 2))
            lc = lane_coord(d, r * P)
            x, y = (lc, al) if vert else (al, lc)
            if math.hypot(x - e.x, y - e.y) < 35 or any(o.kind == "car" and math.hypot(o.x - x, o.y - y) < 12 for o in self.actors):
                continue
            self.actors.append(make_car(x, y, d))
            n += 1
        for axis, idx, route in CITY.tram_roads:       # TTC streetcars on Queen, King and Spadina
            for k in range(3 if "King" in route else 2):
                d = (2 if k % 2 else 0) if axis == "h" else (3 if k % 2 else 1)
                al = random.uniform(20, W - 20)
                lc = lane_coord(d, idx * P)
                x, y = (al, lc) if axis == "h" else (lc, al)
                if math.hypot(x - e.x, y - e.y) < 40:
                    continue
                self.actors.append(make_car(x, y, d, tram=True, L=15.0, Wd=2.55, v0=9.0, v=6.0,
                                            color=(218, 41, 28), route_name=route))
        for kind, cnt in (("ped", 46), ("child", 5), ("dog", 8), ("cat", 4)):
            for _ in range(cnt):
                self.actors.append(make_walker(kind))
        for (bi, bj), (style, _) in CITY.blocks.items():
            if style == "park" and 0 <= bi < N and 0 <= bj < N:
                x0, y0 = bi * P + SW + 4, bj * P + SW + 4
                x1, y1 = (bi + 1) * P - SW - 4, (bj + 1) * P - SW - 4
                for _ in range(3):
                    self.actors.append(Actor("raccoon", random.uniform(x0, x1), random.uniform(y0, y1), mode="graze",
                                             bx0=x0, bx1=x1, by0=y0, by1=y1, wait=random.uniform(0, 4),
                                             fur=(111, 106, 100), phase=0.0, hd=random.uniform(0, 6)))

    # ------------------------------------------------------------- helpers
    def log(self, msg, level="info"):
        self.event_count += 1
        self.events.append((self.t, msg, level))
        if len(self.events) > 2000:
            del self.events[:500]

    def mass(self):
        return 1850 + 80 * self.ride.pax

    def set_route(self, r, why, kind):
        r.kind = kind
        self.route = r
        self.route_ver += 1
        self.ego.pi = 0
        self.odo.reset(self.ego)
        self.trip_t0 = self.t
        self._zn_profile(r)
        turns = sum(1 for nd in r.nodes if nd["turn"])
        self.log(f"{why} -> {r.name} · route {r.length:.0f} m, {turns} turn{'s' if turns != 1 else ''}", "info")

    def replace_route(self, r):
        r.kind = self.route.kind if self.route else "Pickup"
        self.route = r
        self.route_ver += 1
        self.ego.pi = 0
        self._zn_profile(r)

    def _zn_profile(self, r):
        """Predicted Ziegler–Nichols Ku along the route (every 5 m)."""
        rows = []
        m = self.mass()
        for q in range(0, r.n, 10):
            km = float(np.abs(r.k[max(0, q - 20):min(r.n, q + 20)]).max())
            vl = float(r.vl[q]) if getattr(r, "vl", None) is not None else VLIM   # that street's limit
            v = min(vl, math.sqrt(2.2 / km) if km > .03 else vl)
            v = min(v, math.sqrt(3 * max((r.n - 1 - q) * DS, 0)) + .5)
            mu = mu_at(r.px[q], r.py[q])
            rows.append(dict(s=q * DS, x=r.px[q], y=r.py[q], v=v, mu=mu,
                             Ku_traction=zn_at(v, m, mu, "traction", DT)["Ku"],
                             Ku_brake=zn_at(v, m, mu, "brake", DT)["Ku"]))
        r.zn = pd.DataFrame(rows)

    # ------------------------------------------------------------- step
    def step(self, dt=DT):
        self.t += dt
        e = self.ego
        self.plan_timer -= dt
        if self.plan_timer <= 0:
            self.decision = planner.plan(self)
            self.plan_timer = .1
        key, label, a_set, lat_t, _ = self.decision.act
        cars = [a for a in self.actors if a.kind == "car"]
        walkers = [a for a in self.actors if a.kind != "car"]
        e.rule_acc = traffic.ego_rules(self, dt)
        e.nominal = ego_nominal(e, self.route, cars)
        a_cmd = e.nominal if a_set is None else min(a_set, e.nominal)
        e.lat += clamp(lat_t - e.lat, -1.2 * dt, 1.2 * dt)
        self.ride.step(dt)
        self.pid.step(e, self.mass(), a_cmd, self.t, dt)
        pure_pursuit(e, self.route, dt)
        bicycle_step(e, dt)
        self.odo.step(e, self.t, dt)
        npc_accels(cars, walkers, e, self.t, dt, self)
        for c in cars:
            step_vehicle(c, dt, self)
        for wk in walkers:
            step_walker(wk, self.t, dt, self)
        self.stats["dist"] += e.v * dt
        self._collisions()
        self._cleanup()
        self._record()
        if self.auto_chaos and self.t > self.next_chaos:
            if e.v > 4:
                from .scenarios import SCENARIOS, run_scenario
                ok = run_scenario(self, random.randrange(len(SCENARIOS)))
                self.next_chaos = self.t + (random.uniform(9, 15) if ok else 2)
            else:
                self.next_chaos = self.t + 2

    def _collisions(self):
        e = self.ego
        c, s = math.cos(e.th), math.sin(e.th)
        for o in self.actors:
            if o.boarding or abs(o.x - e.x) > 9 or abs(o.y - e.y) > 9:
                continue
            if o.kind == "car":
                hl = max(1.4, o.L / 2 - .95)
                offs = np.linspace(-hl, hl, 7)
                ox, oy = o.x + DIR[o.d][0] * offs, o.y + DIR[o.d][1] * offs
                rr = .95
            else:
                ox, oy, rr = np.array([o.x]), np.array([o.y]), SIZE_R[o.kind]
            ex = e.x + c * np.array([-1.2, 0, 1.2])
            ey = e.y + s * np.array([-1.2, 0, 1.2])
            d = np.sqrt((ex[:, None] - ox[None, :]) ** 2 + (ey[:, None] - oy[None, :]) ** 2).min() - 1.0 - rr
            if d <= 0 and (o.hit_t is None or self.t - o.hit_t > 4):
                o.hit_t = self.t
                who = "TTC streetcar" if getattr(o, "tram", False) else CLSNAME[o.kind]
                if e.v > .5:      # the robotaxi was moving: counts against it
                    self.stats["collisions"] += 1
                    self.log(f"CONTACT with {who} #{o.id} at {e.v * 3.6:.0f} km/h", "bad")
                else:
                    self.stats["contacts_while_stopped"] += 1
                    self.log(f"{who} #{o.id} walked/drove into the stopped robotaxi", "warn")

    def _cleanup(self):
        e = self.ego
        keep = []
        for o in self.actors:
            if o.scripted:
                dd = math.hypot(o.x - e.x, o.y - e.y)
                if (self.t - o.born > 40 and dd > 40) or (o.done and dd > 45) or self.t - o.born > 120:
                    continue
            keep.append(o)
        self.actors = keep

    def _record(self):
        e, p = self.ego, self.pid
        if self.t - self._tele_t >= .1:
            self._tele_t = self.t
            row = dict(t=self.t, x=e.x, y=e.y, heading_deg=math.degrees(e.th), v=e.v, v_ref=p.vref, accel=e.acc,
                       u=p.u, P=p.P, I=p.I, D=p.D, ff=p.ff, action=self.decision.act[0], max_risk=self.decision.max_no,
                       mu=p.mu, grade=p.grade, steer_deg=math.degrees(e.steer), cte=e.cte,
                       odo_raw_err=self.odo.err_raw, odo_fused_err=self.odo.err_fus, phase=self.ride.phase,
                       plan_ms=self.decision.latency_ms, vlim=round(e.vlim * 3.6),
                       ctl=e.ctl["rule"] if e.ctl else "", ctl_key=f"{e.ctl['c'].i},{e.ctl['c'].j}" if e.ctl else "",
                       sig=e.ctl.get("state", "") if e.ctl else "", hold=(e.ctl.get("why") or "") if e.ctl and e.ctl["hold"] else "")
            self.telemetry.append(row)
            self.hist.append(row)
            if len(self.hist) > 220:
                del self.hist[0]
        if self.odo.total - self._zn_s >= 5:
            self._zn_s = self.odo.total
            z = zn_at(e.v, self.mass(), p.mu, p.mode, DT)
            g = rule_gains(z, p.rule)
            self.zn_log.append(dict(t=self.t, x=e.x, y=e.y, v=e.v, mu=p.mu, grade=p.grade, mode=p.mode,
                                    mass=self.mass(), K=z["K"], tau=z["tau"], theta=z["theta"], Ku=z["Ku"],
                                    Tu=z["Tu"], rule=p.rule, Kp=g["Kp"], Ki=g["Ki"], Kd=g["Kd"]))

    # ------------------------------------------------------------- pandas exports
    def dataframes(self):
        return dict(
            telemetry=pd.DataFrame(self.telemetry),
            ziegler_nichols=pd.DataFrame(self.zn_log),
            rides=pd.DataFrame(self.ride.records),
            events=pd.DataFrame(self.events, columns=["t", "message", "level"]),
        )

    def rule_summary(self):
        r = self.rules
        return dict(red_light_stops=r.red_stops, stop_sign_stops=r.sign_stops, yields=r.yields, violations=r.violations,
                    pct_time_over_limit=round(100 * r.over_t / r.drive_t, 2) if r.drive_t else 0.0,
                    worst_overspeed_kmh=round(max(0.0, r.max_over_kmh), 1))
