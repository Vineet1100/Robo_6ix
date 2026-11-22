"""Ride-hailing: requests, dispatch, GPS -> RF rider localisation, curb pull-over, boarding, drop-off."""
from __future__ import annotations

import math
import random

from .agents import SHIRTS, Actor, neighbors, node_pos, rand_node
from .geometry import W, clamp, right_v
from .localization import RF, ParticleFilter, rssi_at
from .routing import (DS, dest_from_point, dest_name, dest_point, plan_route, random_dest, snap_pickup,
                      snap_sidewalk)

GPS_SIGMA = 8.0


class Request:
    _next = 1001

    def __init__(self, rider, gps, t, dest=None, user=False):
        self.id = Request._next
        Request._next += 1
        self.rider, self.gps, self.t, self.dest, self.user = rider, gps, t, dest, user
        rider.req_id = self.id
        self.where = dest_name(dest_from_point(rider.x, rider.y))
        gd = dest_from_point(*gps)
        gx, gy = dest_point(gd)
        rv = right_v(gd.dd)
        self.gps_door_err = math.hypot(gx + rv[0] * 2.5 - rider.x, gy + rv[1] * 2.5 - rider.y)


class RideManager:
    def __init__(self, world):
        self.w = world
        self.phase = "idle"
        self.req = None
        self.queue = []
        self.pax = 0
        self.pf = None
        self.est = None
        self.meas = None
        self.last_meas = self.last_replan = self.idle_t = self.t_board = 0.0
        self.dest = None
        self.auto = True
        self.records = []

    # ------------------------------------------------------------- requests
    def request(self, pickup=None, dest=None, user=False):
        w = self.w
        if pickup is not None:
            p = snap_pickup(*pickup)
            if p is None:
                w.log("That spot is outside the service area — pick an inner street", "warn")
                return None
        else:
            p = None
            for _ in range(80):
                n = rand_node()
                nb = [o for o in neighbors(n) if o[0][0] != n[0] or o[0][1] != n[1]]
                if not nb:
                    continue
                (x0, y0), (x1, y1) = node_pos(n), node_pos(random.choice(nb)[0])
                f = random.uniform(.3, .7)
                c = (x0 + (x1 - x0) * f, y0 + (y1 - y0) * f)
                if 70 < math.hypot(c[0] - w.ego.x, c[1] - w.ego.y) < 230:
                    p = c
                    break
            if p is None:
                return None
        rider = Actor("ped", p[0], p[1], mode="wait", col=(56, 225, 198) if user else (179, 136, 255),
                      phase=random.random() * 6, is_rider=True, user=user, sp=1.3, node=None, target=None, wait=0.0)
        w.actors.append(rider)
        gps = (p[0] + random.gauss(0, GPS_SIGMA), p[1] + random.gauss(0, GPS_SIGMA))
        req = Request(rider, gps, w.t, dest, user)
        if user:
            self.queue.insert(0, req)
            if self.phase == "to_pickup" and self.req and not self.req.user:
                old = self.req
                self._release(old.rider)
                self.req, self.phase = None, "idle"
                w.log(f"Auto request #{old.id} handed to another taxi — your booking takes priority", "info")
            w.log(f"YOUR ride #{req.id} — pickup on {req.where} -> drop-off on {dest_name(dest)}"
                  + ("" if self.phase == "idle" else " (queued after current trip)"), "info")
        else:
            self.queue.append(req)
            w.log(f"Ride request #{req.id} — rider on {req.where}; phone GPS ±{GPS_SIGMA:.0f} m", "info")
        return req

    def _release(self, rider):
        rider.is_rider = False
        rider.mode, rider.path, rider.pi = "script", [dict(x=rider.x, y=rider.y)], 1
        rider.scripted, rider.done, rider.born = True, True, self.w.t
        rider.col = random.choice(SHIRTS)

    def door(self):
        e = self.w.ego
        c, s = math.cos(e.th), math.sin(e.th)
        return e.x - s * 1.6, e.y + c * 1.6

    # ------------------------------------------------------------- state machine
    def step(self, dt):
        w, e = self.w, self.w.ego
        rem = (w.route.n - 1 - e.pi) * DS if w.route is not None else 1e9
        pull_t = .6 if ((self.phase in ("to_pickup", "to_dropoff") and rem < 24) or self.phase in ("boarding", "alighting")) else 0.0
        e.pull += clamp(pull_t - e.pull, -.6 * dt, .6 * dt)

        if self.phase == "idle":
            if not self.queue:
                self.idle_t += dt
                if self.auto and self.idle_t > 2.5:
                    self.idle_t = 0.0
                    self.request()
                return
            req = self.queue.pop(0)
            self.req, self.pf, self.est, self.meas = req, None, None, None
            dest = dest_from_point(*req.gps)
            r = plan_route(e.x, e.y, e.d, dest)
            if r is None:
                w.log(f"Could not route to rider #{req.id} — request cancelled", "warn")
                w.actors.remove(req.rider)
                self.req = None
                return
            self.dest, self.phase = dest, "to_pickup"
            w.set_route(r, f"Dispatched to rider #{req.id} (heading for the phone's GPS fix)", "Pickup")
            return

        req = self.req
        rd = req.rider
        if self.phase == "to_pickup":
            d = math.hypot(rd.x - e.x, rd.y - e.y)
            if d < RF.ble_range and w.t - self.last_meas >= .1:
                self.last_meas = w.t
                rssi = float(rssi_at(d)) + random.gauss(0, RF.shadow)
                aoa = math.atan2(rd.y - e.y, rd.x - e.x) + random.gauss(0, RF.aoa_sigma) if d < RF.uwb_range else None
                self.meas = dict(rssi=rssi, aoa=aoa, rng=10 ** ((RF.P0 - rssi) / (10 * RF.n)))
                if self.pf is None:
                    self.pf = ParticleFilter(req.gps[0], req.gps[1], GPS_SIGMA)
                    w.log(f"RF lock on rider #{req.id}'s phone at {d:.0f} m — BLE RSSI {rssi:.0f} dBm", "info")
                self.pf.update(e.x, e.y, rssi, aoa)
                self.est = self.pf.estimate()
            near_x = abs(e.x - round(e.x / 60) * 60) < 9 and abs(e.y - round(e.y / 60) * 60) < 9
            if self.est and self.est[2] < 6 and w.t - self.last_replan > .5 and not near_x and rem > 4:
                self.last_replan = w.t
                nd = dest_from_point(self.est[0], self.est[1])
                same = nd.a == self.dest.a and nd.dd == self.dest.dd
                if same and abs(nd.frac - self.dest.frac) * 60 > 1.2:
                    r = plan_route(e.x, e.y, e.d, nd)
                    if r is not None and r.length < rem + 25:
                        self.dest = nd
                        w.replace_route(r)
                elif not same and self.est[2] < 3.5:
                    r = plan_route(e.x, e.y, e.d, nd)
                    if r is not None:
                        self.dest = nd
                        w.replace_route(r)
                        w.log(f"RF correction — rider is really on {dest_name(nd)}; re-routing ({r.length:.0f} m)", "warn")
            if rem < 1.5 and e.v < .3:
                dx, dy = self.door()
                err = math.hypot(dx - rd.x, dy - rd.y)
                gps_err = math.hypot(req.gps[0] - rd.x, req.gps[1] - rd.y)
                rf_err = math.hypot(self.est[0] - rd.x, self.est[1] - rd.y) if self.est else float("nan")
                self.records.append(dict(t=w.t, ride_id=req.id, user=req.user, rider_x=rd.x, rider_y=rd.y,
                                         phone_gps_err_m=gps_err, rf_fix_err_m=rf_err, door_err_rf_m=err,
                                         door_err_gps_only_m=req.gps_door_err, wait_s=w.t - req.t))
                w.log(f"Pulled over for rider #{req.id} — door {err:.1f} m from rider (GPS-only stop: {req.gps_door_err:.1f} m)", "info")
                self.phase = "boarding"
                rd.mode, rd.goto, rd.boarding, rd.arrived = "walkto", (dx, dy), True, False
                self.t_board = w.t
        elif self.phase == "boarding":
            rd.goto = self.door()
            if getattr(rd, "arrived", False) or w.t - self.t_board > 30:
                r = plan_route(e.x, e.y, e.d, req.dest) if req.dest else None
                tries = 0
                while r is None and tries < 40:
                    c = plan_route(e.x, e.y, e.d, random_dest())
                    if c is not None and 150 < c.length < 480:
                        r = c
                    tries += 1
                if r is None:
                    return
                if rd in w.actors:
                    w.actors.remove(rd)
                self.pax, self.phase = 1, "to_dropoff"
                w.set_route(r, f"Rider #{req.id} on board", "Drop-off")
        elif self.phase == "to_dropoff":
            if rem < 1.5 and e.v < .3:
                self.phase, self.pax = "alighting", 0
                dx, dy = self.door()
                rd.x, rd.y = dx, dy
                sx, sy = snap_sidewalk(dx + (dx - e.x) * 1.5, dy + (dy - e.y) * 1.5)
                rd.mode, rd.goto, rd.boarding, rd.arrived = "walkto", (sx, sy), True, False
                w.actors.append(rd)
                self.t_board = w.t
                w.stats["trips"] += 1
                if self.records and self.records[-1]["ride_id"] == req.id:
                    self.records[-1].update(trip_m=w.odo.trip, dropoff=w.route.name, done_t=w.t)
                w.log(f"Trip {w.stats['trips']} complete — rider #{req.id} dropped off on {w.route.name} ({w.odo.trip:.0f} m)", "info")
        elif self.phase == "alighting":
            if getattr(rd, "arrived", False) or w.t - self.t_board > 10:
                rd.boarding = False
                self._release(rd)
                self.phase, self.req, self.idle_t = "idle", None, 0.0
