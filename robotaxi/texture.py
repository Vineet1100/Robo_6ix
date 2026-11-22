"""Satellite-style aerial imagery of downtown Toronto, drawn in Python with Pillow + NumPy.

Flask renders this once when it starts and serves it at /api/texture.jpg; the browser lays it under the
live simulation. Roads, lane markings, crosswalks, TTC tracks, trees, roofs, shadows, wet patches, the
traffic-control markings and every landmark (TD Centre, Roy Thomson Hall, CN Tower...) are drawn here.
Block styles come from the map file, so changing a block there changes the picture.

Everything is procedural and drawn from scratch: landmark footprints are stylised top-down shapes,
not copied imagery.
"""
from __future__ import annotations

import math
import random

import numpy as np
from PIL import Image, ImageDraw

import io
import threading

from .sim.geometry import CITY, DIR, M, N, NODE, P, RH, SW, TS, W, horiz, in_grid, right_v, sgn_d, wet_zones
from .sim.traffic import CTRL, STOPL, approach_rule, axis_of

WET = wet_zones()
SIZE = int((W + 2 * M) * TS)        # 8 px per metre, same as the browser canvas


class Canvas:
    """Draw in world metres onto an RGBA Pillow image (alpha-blended)."""

    def __init__(self):
        self.img = Image.new("RGB", (SIZE, SIZE), (143, 138, 127))   # RGB base + RGBA draw = alpha blending
        self.d = ImageDraw.Draw(self.img, "RGBA")
        self.rng = random.Random(20261112)

    def p(self, x, y):
        return ((x + M) * TS, (y + M) * TS)

    def rect(self, x, y, w, h, fill, outline=None, width=1):
        (x0, y0), (x1, y1) = self.p(x, y), self.p(x + w, y + h)
        self.d.rectangle([x0, y0, x1, y1], fill=fill, outline=outline, width=width)

    def poly(self, pts, fill, outline=None, width=1):
        self.d.polygon([self.p(*q) for q in pts], fill=fill, outline=outline, width=width)

    def circle(self, x, y, r, fill=None, outline=None, width=1):
        (x0, y0), (x1, y1) = self.p(x - r, y - r), self.p(x + r, y + r)
        self.d.ellipse([x0, y0, x1, y1], fill=fill, outline=outline, width=width)

    def ellipse(self, x, y, rx, ry, fill):
        (x0, y0), (x1, y1) = self.p(x - rx, y - ry), self.p(x + rx, y + ry)
        self.d.ellipse([x0, y0, x1, y1], fill=fill)

    def line(self, pts, fill, width_m):
        self.d.line([self.p(*q) for q in pts], fill=fill, width=max(1, int(width_m * TS)), joint="curve")

    def r(self, a, b):
        return self.rng.uniform(a, b)

    def pick(self, seq):
        return self.rng.choice(seq)


def shade(c, f):
    if f > 0:
        return tuple(int(v + (255 - v) * f) for v in c[:3]) + ((c[3],) if len(c) == 4 else ())
    return tuple(int(v * (1 + f)) for v in c[:3]) + ((c[3],) if len(c) == 4 else ())


SHADOW = (12, 14, 10, 105)


def shadow_rect(cv, x, y, w, h, ht):
    sh = min(ht * .13, 16)
    cv.poly([(x + w, y), (x + w + sh, y + sh), (x + w + sh, y + h + sh), (x + sh, y + h + sh), (x, y + h), (x, y)], SHADOW)


def shadow_poly(cv, pts, ht):
    sh = min(ht * .13, 16)
    cv.poly([(px + sh, py + sh) for px, py in pts], SHADOW)
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        cv.poly([a, b, (b[0] + sh, b[1] + sh), (a[0] + sh, a[1] + sh)], SHADOW)


def tree(cv, x, y, r):
    cv.ellipse(x + r * .5, y + r * .5, r, r * .92, (8, 12, 6, 85))
    base = cv.pick([(47, 74, 34), (54, 82, 42), (44, 69, 32), (61, 90, 44), (65, 85, 42)])
    for _ in range(3):
        ox, oy, r2 = cv.r(-r * .3, r * .3), cv.r(-r * .3, r * .3), r * cv.r(.55, .82)
        cv.circle(x + ox, y + oy, r2, shade(base, -.25) + (255,))
        cv.circle(x + ox - r2 * .2, y + oy - r2 * .2, r2 * .65, shade(base, .1) + (255,))
        cv.circle(x + ox - r2 * .35, y + oy - r2 * .35, r2 * .3, shade(base, .3) + (200,))


def roof(cv, x, y, w, h, col):
    cv.rect(x, y, w, h, col + (255,))
    cv.rect(x + .6, y + .6, w - 1.2, h - 1.2, shade(col, .07) + (255,))
    cv.rect(x + .2, y + .2, w - .4, h - .4, None, outline=shade(col, -.28) + (255,), width=2)
    n = int(cv.r(1, 4 + w * h / 160))
    for _ in range(n):
        bw, bh = cv.r(1, 3.2), cv.r(1, 2.4)
        if w - bw - 2.4 < 0 or h - bh - 2.4 < 0:
            continue
        bx, by = x + cv.r(1.2, w - bw - 1.2), y + cv.r(1.2, h - bh - 1.2)
        cv.rect(bx + .35, by + .35, bw, bh, (0, 0, 0, 80))
        cv.rect(bx, by, bw, bh, cv.pick([(201, 203, 204), (179, 182, 184), (216, 216, 210), (158, 163, 166)]) + (255,))


def glass_roof(cv, x, y, w, h, col):
    roof(cv, x, y, w, h, col)
    k = 1.6
    while k < w:
        cv.rect(x + k, y + .6, .08, h - 1.2, (255, 255, 255, 22))
        k += 1.6


def plaza(cv, x, y, s, col=(168, 163, 151)):
    cv.rect(x, y, s, s, col + (255,))
    k = 0.0
    while k < s:
        cv.rect(x + k, y, .05, s, (0, 0, 0, 15))
        cv.rect(x, y + k, s, .05, (0, 0, 0, 15))
        k += 1.5


def rect_pts(x, y, w, h):
    return [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]


# ------------------------------------------------------------------ block styles
def b_fin(cv, x, y, s):
    plaza(cv, x, y, s)
    L = cv.rng.random()
    if L < .4:
        ps = [(x + 3, y + 3, s - 6, s - 6)]
    elif L < .7:
        m = cv.r(.4, .6) * s
        ps = [(x + 2, y + 2, m - 3, s - 4), (x + m + 1, y + 2, s - m - 3, s - 4)]
    else:
        m = s / 2
        ps = [(x + 2, y + 2, m - 3, m - 3), (x + m + 1, y + 2, m - 3, m - 3), (x + 2, y + m + 1, m - 3, m - 3), (x + m + 1, y + m + 1, m - 3, m - 3)]
    bs = []
    for p in ps:
        i = cv.r(.5, 2.5)
        bs.append((p[0] + i, p[1] + i, p[2] - 2 * i, p[3] - 2 * i, cv.r(60, 180),
                   cv.pick([(111, 122, 132), (89, 99, 109), (138, 148, 156), (184, 188, 191), (79, 90, 99), (154, 164, 171)])))
    for b in bs:
        shadow_rect(cv, *b[:4], b[4])
    for b in bs:
        glass_roof(cv, *b[:4], b[5])


def b_condo(cv, x, y, s):
    plaza(cv, x, y, s, (157, 150, 138))
    for _ in range(int(cv.r(3, 5))):
        w, h = cv.r(10, 18), cv.r(10, 20)
        bx, by = x + cv.r(1, s - w - 1), y + cv.r(1, s - h - 1)
        shadow_rect(cv, bx, by, w, h, cv.r(12, 30))
        roof(cv, bx, by, w, h, cv.pick([(140, 74, 58), (154, 91, 69), (122, 70, 54), (160, 106, 82), (110, 62, 49), (139, 125, 107)]))
    if cv.rng.random() < .8:
        w, h = cv.r(12, 17), cv.r(12, 17)
        bx, by = x + cv.r(2, s - w - 2), y + cv.r(2, s - h - 2)
        shadow_rect(cv, bx, by, w, h, cv.r(90, 150))
        glass_roof(cv, bx, by, w, h, cv.pick([(109, 131, 146), (143, 163, 176), (123, 141, 153), (95, 115, 128)]))
    for _ in range(3):
        tree(cv, x + cv.r(2, s - 2), y + cv.r(2, s - 2), cv.r(1.6, 2.4))


def b_heritage(cv, x, y, s):
    plaza(cv, x, y, s, (141, 135, 123))
    cx = x
    while cx < x + s - 3:
        w = min(cv.r(4.5, 8), x + s - cx)
        d = cv.r(13, 18)
        shadow_rect(cv, cx, y, w, d, cv.r(8, 14))
        roof(cv, cx, y, w - .15, d, cv.pick([(140, 74, 58), (160, 83, 63), (125, 106, 90), (93, 74, 62), (154, 123, 91), (107, 90, 76), (176, 122, 82)]))
        cx += w
    cv.rect(x, y + 19, s, 3, (125, 120, 109, 255))
    shadow_rect(cv, x + 2, y + 24, s - 4, s - 26, 24)
    roof(cv, x + 2, y + 24, s - 4, s - 26, cv.pick([(154, 91, 69), (142, 147, 153), (167, 160, 151)]))


def b_park(cv, x, y, s):
    cv.rect(x, y, s, s, (91, 127, 58, 255))
    k = 0.0
    while k < s:
        cv.rect(x + k, y, 1.2, s, (255, 255, 255, 10))
        k += 2.4
    for _ in range(140):
        cv.circle(x + cv.r(0, s), y + cv.r(0, s), cv.r(.5, 2.6), (40, 60, 20, 33) if cv.rng.random() < .5 else (150, 150, 80, 30))
    pts1 = [(x + s * u, y + s * (.2 + .55 * u * u)) for u in np.linspace(0, 1, 20)]
    pts2 = [(x + s * (.15 + .65 * u), y + s * (1 - u) + s * .1 * math.sin(u * 3)) for u in np.linspace(0, 1, 20)]
    cv.line(pts1, (200, 185, 148, 255), 1.6)
    cv.line(pts2, (200, 185, 148, 255), 1.6)
    px, py, prx, pry = x + s * cv.r(.35, .65), y + s * cv.r(.35, .65), cv.r(5, 8), cv.r(3.5, 6)
    cv.ellipse(px, py, prx + .9, pry + .9, (185, 167, 125, 255))
    cv.ellipse(px, py, prx, pry, (43, 89, 102, 255))
    cv.ellipse(px - 1, py - 1, prx * .6, pry * .5, (120, 180, 190, 55))
    gx, gy = x + 3, y + s - 11
    cv.rect(gx, gy, 9, 7, (200, 173, 115, 255))
    for k, c in enumerate([(214, 69, 69), (63, 127, 214), (224, 195, 65)]):
        cv.rect(gx + 1.5 + k * 2.5, gy + 1.5 + (k % 2) * 2, 1.6, 2.2, c + (255,))
    for _ in range(24):
        tx, ty = x + cv.r(2, s - 2), y + cv.r(2, s - 2)
        if math.hypot((tx - px) / (prx + 2.5), (ty - py) / (pry + 2.5)) < 1 or (tx < gx + 11 and ty > gy - 2):
            continue
        tree(cv, tx, ty, cv.r(1.8, 3.4))


def b_lot(cv, x, y, s):
    cv.rect(x, y, s, s, (85, 88, 92, 255))
    cols = [(216, 216, 216), (29, 31, 34), (138, 143, 150), (161, 42, 42), (36, 70, 122), (231, 227, 218), (199, 164, 58)]
    for ry in (1.5, 12.5, 17.5, 28.5, 33.5):
        xx = x + 3
        while xx < x + s - 5.6:
            cv.rect(xx, y + ry, .1, 5, (235, 235, 230, 180))
            if cv.rng.random() < .62:
                c = cv.pick(cols)
                cv.rect(xx + .55, y + ry + .5, 1.8, 4.3, (0, 0, 0, 80))
                cv.rect(xx + .4, y + ry + .3, 1.8, 4.3, c + (255,))
                cv.rect(xx + .55, y + ry + 1.3, 1.5, 1.0, (24, 33, 42, 255))
            xx += 2.6
    shadow_rect(cv, x + s - 10, y + s - 9, 7, 6, 10)
    roof(cv, x + s - 10, y + s - 9, 7, 6, (165, 157, 144))


def b_td(cv, x, y, s):
    plaza(cv, x, y, s, (179, 173, 159))
    t = [(x + 2, y + 3, 16, 30, 220), (x + s - 17, y + s - 25, 15, 22, 180), (x + s - 15, y + 2, 12, 12, 40)]
    for b in t:
        shadow_rect(cv, *b[:4], b[4])
    for b in t:
        cv.rect(*b[:4], (27, 30, 33, 255))
        k = 1.5
        while k < b[2]:
            cv.rect(b[0] + k, b[1], .06, b[3], (255, 255, 255, 25))
            k += 1.5
        cv.rect(b[0] + .3, b[1] + .3, b[2] - .6, b[3] - .6, None, outline=(44, 49, 54, 255), width=3)
    for _ in range(4):
        tree(cv, x + cv.r(20, s - 18), y + cv.r(5, 18), 1.6)


def b_rbp(cv, x, y, s):
    plaza(cv, x, y, s, (170, 164, 150))
    t1 = [(x + 2, y + 2), (x + 30, y + 2), (x + 2, y + 30)]
    t2 = [(x + s - 2, y + s - 2), (x + s - 30, y + s - 2), (x + s - 2, y + s - 30)]
    shadow_poly(cv, t1, 180)
    shadow_poly(cv, t2, 130)
    cv.poly([(x + 20, y + 10), (x + s - 10, y + s - 20), (x + s - 20, y + s - 10), (x + 10, y + 20)], (168, 196, 212, 255))
    for t in (t1, t2):
        cv.poly(t, (184, 146, 42, 255), outline=(90, 64, 10, 255), width=3)
        a, b, c = t
        mid = [(a[0] * .55 + b[0] * .45, a[1] * .55 + b[1] * .45), (a[0] * .55 + c[0] * .45, a[1] * .55 + c[1] * .45)]
        cv.poly([a, mid[0], mid[1]], (230, 198, 92, 255))
        cv.poly([b, c, mid[1], mid[0]], (141, 108, 24, 140))


def b_fcp(cv, x, y, s):
    plaza(cv, x, y, s, (162, 157, 145))
    roof(cv, x + 1, y + 1, s - 2, s - 2, (138, 143, 147))
    cx, cy, h, ch = x + s / 2, y + s / 2, 13, 3.5
    octo = [(cx - h + ch, cy - h), (cx + h - ch, cy - h), (cx + h, cy - h + ch), (cx + h, cy + h - ch), (cx + h - ch, cy + h),
            (cx - h + ch, cy + h), (cx - h, cy + h - ch), (cx - h, cy - h + ch)]
    shadow_poly(cv, octo, 300)
    cv.poly(octo, (236, 235, 230, 255))
    k = -h + 1
    while k < h:
        cv.rect(cx + k, cy - h + .5, .08, 2 * h - 1, (0, 0, 0, 30))
        k += 1.2
    cv.rect(cx - 4, cy - 4, 8, 8, (201, 201, 196, 255))


def b_scotia(cv, x, y, s):
    plaza(cv, x, y, s, (165, 159, 147))
    x0, y0, w, h, n = x + 8, y + 6, 26, 26, 3
    p = [(x0 + n, y0), (x0 + w - n, y0), (x0 + w - n, y0 + n), (x0 + w, y0 + n), (x0 + w, y0 + h - n), (x0 + w - n, y0 + h - n),
         (x0 + w - n, y0 + h), (x0 + n, y0 + h), (x0 + n, y0 + h - n), (x0, y0 + h - n), (x0, y0 + n), (x0 + n, y0 + n)]
    shadow_poly(cv, p, 275)
    cv.poly(p, (124, 51, 41, 255))
    cv.rect(x0 + n + 1, y0 + n + 1, w - 2 * n - 2, h - 2 * n - 2, (142, 61, 49, 255))
    shadow_rect(cv, x + 2, y + s - 10, s - 4, 8, 20)
    roof(cv, x + 2, y + s - 10, s - 4, 8, (156, 143, 128))


def b_brook(cv, x, y, s):
    plaza(cv, x, y, s, (167, 162, 150))
    a, b = (x + 2, y + 2, 17, 19, 260), (x + s - 19, y + s - 22, 17, 20, 200)
    for t in (a, b):
        shadow_rect(cv, *t[:4], t[4])
    glass_roof(cv, *a[:4], (76, 88, 102))
    glass_roof(cv, *b[:4], (107, 122, 134))
    gx, gy, gw, gh = x + 21, y + 3, 7, s - 6
    cv.rect(gx, gy, gw, gh, (214, 226, 232, 255))
    k = 0.0
    while k < gh:
        cv.line([(gx, gy + k), (gx + gw / 2, gy + k - .5), (gx + gw, gy + k)], (255, 255, 255, 220), .12)
        k += 1.2


def b_rth(cv, x, y, s):
    plaza(cv, x, y, s, (171, 166, 153))
    cx, cy, R = x + s * .45, y + s * .55, 17
    cv.circle(cx + 3, cy + 3, R, (12, 14, 10, 90))
    for i, rr in enumerate(np.linspace(R, 2, 9)):
        f = i / 8
        cv.circle(cx - 3 * f, cy - 3 * f, rr, (int(95 + 106 * f), int(127 + 92 * f), int(147 + 81 * f), 255))
    for k in range(32):
        a = k / 32 * 2 * math.pi
        cv.line([(cx + math.cos(a) * 3, cy + math.sin(a) * 3), (cx + math.cos(a) * R, cy + math.sin(a) * R)], (255, 255, 255, 80), .1)
    for r in (6, 11, 15):
        cv.circle(cx, cy, r, None, outline=(255, 255, 255, 80), width=1)
    cv.circle(cx, cy, 3, (217, 212, 196, 255))
    shadow_rect(cv, x + s - 15, y + 2, 13, 16, 110)
    glass_roof(cv, x + s - 15, y + 2, 13, 16, (125, 138, 147))
    for k in range(7):
        a = k / 7 * 2 * math.pi + .4
        tree(cv, cx + math.cos(a) * (R + 3.5), cy + math.sin(a) * (R + 3.5), 1.5)


def b_cityhall(cv, x, y, s):
    plaza(cv, x, y, s, (194, 188, 174))
    px, py, pw, ph = x + 6, y + s - 17, s - 12, 8
    cv.rect(px, py, pw, ph, (47, 111, 138, 255))
    for k in range(3):
        ax = px + pw * (k + .5) / 3
        (x0, y0), (x1, y1) = cv.p(ax - pw / 6 + .6, py - .6), cv.p(ax + pw / 6 - .6, py + ph + .6)
        cv.d.ellipse([x0, y0, x1, y1], outline=(232, 230, 223, 255), width=5)
    cv.circle(x + s / 2 + 3, y + 5, 10, (12, 14, 10, 90))
    cv.circle(x + s / 2, y + 2, 9, (228, 226, 220, 255))
    cv.circle(x + s / 2, y + 2, 6, None, outline=(189, 185, 174, 255), width=2)
    for side in (-1, 1):
        pts = [(x + s / 2 + side * (24 * math.sin(a)), y - 14 + 24 * math.cos(a)) for a in np.linspace(.35, 1.2, 12)]
        cv.line(pts, (215, 213, 206, 255), 5)
    for k in range(4):
        tree(cv, x + 3 + k * 3, y + s - 4, 1.4)
        tree(cv, x + s - 3 - k * 3, y + s - 4, 1.4)


def b_osgoode(cv, x, y, s):
    cv.rect(x, y, s, s, (95, 138, 60, 255))
    shadow_rect(cv, x + 6, y + 2, s - 12, 12, 16)
    roof(cv, x + 6, y + 2, s - 12, 12, (207, 198, 176))
    cv.rect(x + .6, y + .6, s - 1.2, s - 1.2, None, outline=(42, 42, 42, 255), width=1)
    cv.line([(x + s / 2, y + s), (x + s / 2, y + 14)], (217, 207, 178, 255), 1.4)
    for _ in range(8):
        tree(cv, x + cv.r(3, s - 3), y + cv.r(17, s - 3), cv.r(1.8, 2.8))


def b_eaton(cv, x, y, s):
    plaza(cv, x, y, s, (159, 153, 141))
    shadow_rect(cv, x + 1, y + 1, s - 12, s - 2, 35)
    roof(cv, x + 1, y + 1, s - 12, s - 2, (142, 147, 153))
    gx, gw = x + s - 11, 8
    cv.rect(gx, y, gw, s, (216, 228, 234, 255))
    k = 0.0
    while k < s:
        cv.line([(gx, y + k), (gx + gw / 2, y + k - .4), (gx + gw, y + k)], (255, 255, 255, 230), .12)
        k += 1.1
    shadow_rect(cv, x + s - 14, y + s - 14, 13, 13, 40)
    roof(cv, x + s - 14, y + s - 14, 13, 13, (203, 191, 164))


def b_union(cv, x, y, s):
    cv.rect(x, y, s, 12, (154, 149, 138, 255))
    shadow_rect(cv, x + 1, y + 1, s - 2, 10, 25)
    roof(cv, x + 1, y + 1, s - 2, 10, (203, 191, 164))
    cv.rect(x + s / 2 - 6, y + 2, 12, 8, (184, 171, 143, 255))


def b_southlow(cv, x, y, s):
    cv.rect(x, y, s, 11, (141, 135, 123, 255))
    n = int(cv.r(2, 4))
    w = (s - 2) / n - 1
    cx = x + 1
    for _ in range(n):
        shadow_rect(cv, cx, y + 1, w, 8, 15)
        roof(cv, cx, y + 1, w, 8, cv.pick([(156, 143, 128), (142, 147, 153), (167, 160, 151)]))
        cx += w + 1


STYLES = dict(osgoodepark=b_park, fin=b_fin, condo=b_condo, heritage=b_heritage, park=b_park, lot=b_lot, td=b_td, rbp=b_rbp, fcp=b_fcp,
              scotia=b_scotia, brook=b_brook, rth=b_rth, cityhall=b_cityhall, osgoode=b_osgoode, eaton=b_eaton,
              union=b_union, southlow=b_southlow)


def south_strip(cv):
    yr, yr2 = W + 18, W + 29
    cv.rect(-M, yr, W + 2 * M, yr2 - yr, (107, 97, 82, 255))
    for t in range(5):
        ty = yr + 1.4 + t * 2.1
        x = -M
        while x < W + M:
            cv.rect(x, ty - .35, .22, 1.6, (60, 45, 30, 150))
            x += .6
        cv.rect(-M, ty, W + 2 * M, .09, (164, 166, 168, 255))
        cv.rect(-M, ty + .9, W + 2 * M, .09, (164, 166, 168, 255))
    cv.rect(-M, yr2 + .6, W + 2 * M, M, (125, 138, 92, 255))
    sx, sw = 3 * P - 2, P + 4
    cv.rect(sx, yr, sw, yr2 - yr, (169, 188, 198, 255))
    k = 0.0
    while k < sw:
        cv.rect(sx + k, yr, .12, yr2 - yr, (255, 255, 255, 170))
        k += 1.3
    # CN Tower (top-down: tripod base, pod rings) with its long shadow
    cx, cy = 75, W + 36
    cv.poly([(cx - 1.5, cy - 1.5), (cx + 60, cy + 60), (cx + 56, cy + 64), (cx + 1.5, cy + 1.5)], (10, 12, 10, 100))
    for k in range(3):
        a = k * 2.094 - 1.57
        cv.line([(cx, cy), (cx + math.cos(a) * 11, cy + math.sin(a) * 11)], (189, 184, 173, 255), 2.4)
    for r, c in ((8, (74, 79, 85)), (6, (217, 212, 200)), (3.2, (140, 147, 154)), (1.4, (232, 228, 218))):
        cv.circle(cx, cy, r, c + (255,))
    # Rogers Centre dome edge
    rx, ry, R = 10, W + 62, 30
    cv.circle(rx, ry, R, (200, 203, 205, 255))
    for r in (R - 4, R - 9, R - 15):
        cv.circle(rx, ry, r, None, outline=(80, 85, 90, 160), width=2)
    for k in range(16):
        a = k / 16 * 2 * math.pi
        cv.line([(rx, ry), (rx + math.cos(a) * R, ry + math.sin(a) * R)], (80, 85, 90, 140), .3)


def build_texture() -> Image.Image:
    cv = Canvas()
    for i in range(N + 1):     # sidewalks
        cv.rect(i * P - SW, -M, 2 * SW, W + 2 * M, (170, 167, 158, 255))
        cv.rect(-M, i * P - SW, W + 2 * M, 2 * SW, (170, 167, 158, 255))
    for i in range(N + 1):     # asphalt
        cv.rect(i * P - RH, -M, 2 * RH, W + 2 * M, (59, 62, 66, 255))
        cv.rect(-M, i * P - RH, W + 2 * M, 2 * RH, (59, 62, 66, 255))
        for o in (-2.75, -1.25, 1.25, 2.75):   # tyre wear
            cv.rect(i * P + o - .25, -M, .5, W + 2 * M, (0, 0, 0, 22))
            cv.rect(-M, i * P + o - .25, W + 2 * M, .5, (0, 0, 0, 22))
    for _ in range(90):
        i, t, o = cv.rng.randint(0, N), cv.r(-M, W + M), cv.r(-RH + 1, RH - 2)
        w, h = cv.r(1, 4), cv.r(.8, 2.5)
        if cv.rng.random() < .5:
            cv.rect(i * P + o, t, h, w, (72, 75, 79, 180))
        else:
            cv.rect(t, i * P + o, w, h, (72, 75, 79, 180))
    yel, wht = (216, 177, 60, 255), (235, 235, 230, 215)
    for j in range(N + 1):     # lane markings + stop lines
        for i in range(-1, N + 1):
            a, b = max(-M, i * P + 7.6), min(W + M, (i + 1) * P - 7.6)
            L = b - a
            if L <= 0:
                continue
            for o in (-.25, .12):
                cv.rect(a, j * P + o, L, .13, yel)
                cv.rect(j * P + o, a, .13, L, yel)
            for o in (-3.75, 3.63):
                cv.rect(a, j * P + o, L, .12, wht)
                cv.rect(j * P + o, a, .12, L, wht)
            if i < N:
                cv.rect(b - .45, j * P + .3, .45, 3.4, wht)
                cv.rect(j * P - 3.7, b - .45, 3.4, .45, wht)
            if i >= 0:
                cv.rect(a, j * P - 3.7, .45, 3.4, wht)
                cv.rect(j * P + .3, a, 3.4, .45, wht)
    for i in range(N + 1):     # crosswalks
        for j in range(N + 1):
            for sg in (-1, 1):
                t = -3.5
                while t < 3.5:
                    cv.rect(i * P + t, j * P + sg * NODE - .9, .5, 1.8, wht)
                    cv.rect(i * P + sg * NODE - .9, j * P + t, 1.8, .5, wht)
                    t += .9
    for axis, idx, _ in CITY.tram_roads:     # TTC streetcar rails
        for lo in (-2, 2):
            for rg in (-.72, .72):
                c = idx * P + lo + rg
                if axis == "h":
                    cv.rect(-M, c - .07, W + 2 * M, .14, (20, 22, 24, 190))
                    cv.rect(-M, c - .02, W + 2 * M, .04, (170, 175, 180, 140))
                else:
                    cv.rect(c - .07, -M, .14, W + 2 * M, (20, 22, 24, 190))
                    cv.rect(c - .02, -M, .04, W + 2 * M, (170, 175, 180, 140))
    for (bi, bj), (style, _) in sorted(CITY.blocks.items(), key=lambda kv: (kv[0][1], kv[0][0])):
        x0, y0, s = bi * P + SW, bj * P + SW, P - 2 * SW
        cv.rect(x0, y0, s, s, (138, 133, 122, 255))
        STYLES.get(style, b_condo)(cv, x0 + 1, y0 + 1, s - 2)
        pr = .75 if style == "park" else .45 if style in ("heritage", "condo") else .3
        if bj < N:
            for side in range(4):
                u = 4.0
                while u < s - 4:
                    if cv.rng.random() <= pr:
                        tx = x0 + u if side in (0, 2) else (x0 + s - .6 if side == 1 else x0 + .6)
                        ty = y0 + .6 if side == 0 else (y0 + s - .6 if side == 2 else y0 + u)
                        tree(cv, tx, ty, cv.r(1.7, 2.5))
                    u += cv.r(7, 10)
    south_strip(cv)
    for (wx, wy, r) in WET:      # puddles / wet patches
        for k, rr in enumerate(np.linspace(r, r * .3, 6)):
            cv.circle(wx, wy, rr, (18, 32, 52, 22 + k * 6))
        for _ in range(6):
            cv.ellipse(wx + cv.r(-r * .5, r * .5), wy + cv.r(-r * .5, r * .5), cv.r(.6, 2), cv.r(.15, .4), (170, 200, 230, 30))
    paint_controls(cv)
    # photographic grain with NumPy
    arr = np.asarray(cv.img.convert("RGB"), dtype=np.int16)
    rng = np.random.default_rng(7)
    noise = rng.normal(0, 6, arr.shape[:2]).astype(np.int16)
    arr = np.clip(arr + noise[:, :, None], 0, 255).astype(np.uint8)
    return Image.fromarray(arr, "RGB")


# ------------------------------------------------------------------ traffic-control markings
def _app_pt(c, d, back, right):
    """World point `back` metres upstream of the intersection centre, `right` metres right of heading d."""
    hx, s = horiz(d), sgn_d(d)
    rr = right_v(d)[1 if hx else 0]
    al = (c.i if hx else c.j) * P - s * back
    pp = (c.j if hx else c.i) * P + rr * right
    return (al, pp) if hx else (pp, al)


def _approaches(c):
    return [d for d in range(4) if in_grid(c.i - DIR[d][0], c.j - DIR[d][1])]


def _fill_app(cv, c, d, b1, b2, r1, r2, col):
    (x1, y1), (x2, y2) = _app_pt(c, d, b1, r1), _app_pt(c, d, b2, r2)
    cv.rect(min(x1, x2), min(y1, y2), abs(x2 - x1), abs(y2 - y1), col)


def paint_controls(cv):
    """No stop line where traffic has right of way, shark teeth at yields, STOP painted on the road."""
    asphalt, paint = (59, 62, 66, 255), (238, 238, 232, 230)
    for c in CTRL.values():
        for d in _approaches(c):
            rule = approach_rule(c, axis_of(d))
            if rule in ("free", "yield"):
                _fill_app(cv, c, d, STOPL - .05, STOPL + .5, .25, 3.75, asphalt)
            if rule == "yield":
                for k in range(5):
                    a = _app_pt(c, d, STOPL + .05, .45 + k * .68)
                    b = _app_pt(c, d, STOPL + .05, .45 + k * .68 + .5)
                    t = _app_pt(c, d, STOPL + .85, .45 + k * .68 + .25)
                    cv.poly([a, b, t], paint)
            if rule == "stop":
                _stop_text(cv, _app_pt(c, d, STOPL + 3.2, 2), d)


_FONT = None


def _stop_text(cv, at, d):
    """'STOP' lettering readable by the approaching driver (rotated to face them)."""
    from PIL import ImageFont
    global _FONT
    if _FONT is None:
        try:
            _FONT = ImageFont.truetype("DejaVuSans-Bold.ttf", int(1.5 * TS))
        except OSError:
            _FONT = ImageFont.load_default()
    w, h = int(4.6 * TS), int(2.2 * TS)
    tile = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    ImageDraw.Draw(tile).text((w / 2, h / 2), "STOP", font=_FONT, fill=(238, 238, 232, 215), anchor="mm")
    # tile text reads left-to-right with "up" = -y; rotate so "up" points along the travel direction
    ang = {0: -90, 1: 180, 2: 90, 3: 0}[d]   # Pillow rotates counter-clockwise
    tile = tile.rotate(ang, expand=True, resample=Image.BICUBIC)
    x, y = cv.p(*at)
    cv.img.paste(tile, (int(x - tile.width / 2), int(y - tile.height / 2)), tile)


# ------------------------------------------------------------------ cached JPEG for Flask
_cache: dict = {}
_lock = threading.Lock()


def texture_jpeg(quality: int = 88) -> bytes:
    """The aerial image as JPEG bytes, rendered once and then served from memory."""
    with _lock:
        if "jpg" not in _cache:
            buf = io.BytesIO()
            build_texture().save(buf, "JPEG", quality=quality, optimize=True)
            _cache["jpg"] = buf.getvalue()
        return _cache["jpg"]
