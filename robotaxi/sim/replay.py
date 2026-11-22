"""Turn a headless run into an animated GIF, drawn in Python with Pillow on top of the Python-drawn city.

    python simulate.py --gif --minutes 1          a one minute clip, camera following the robotaxi

Each frame shows the city, every car / streetcar / pedestrian / animal, the live traffic lights on their
stop lines, the taxi's planned path, and a small panel with time, speed, speed limit, the next traffic
control and the decision the planner just made.
"""
from __future__ import annotations

import math
import os
import time

from PIL import Image, ImageDraw, ImageFilter, ImageFont

from ..texture import build_texture
from .geometry import DIR, M, P, TS, W
from .traffic import CTRL, STOPL, approach_rule, axis_of, sig_state
from .world import DT, World

PX = 4                                   # pixels per metre in the GIF
VIEW_W, VIEW_H = 120, 84                 # metres shown around the taxi
SIGCOL = dict(G=(43, 217, 107), Y=(255, 191, 31), R=(255, 59, 74))
KIND_COL = dict(ped=(255, 214, 64), child=(255, 160, 60), dog=(255, 140, 50), cat=(255, 140, 50),
                raccoon=(255, 120, 40), ball=(180, 120, 255))


def _font(size):
    try:
        return ImageFont.truetype("DejaVuSans-Bold.ttf", size)
    except OSError:
        return ImageFont.load_default()


def _rect(cx, cy, L, Wd, th):
    c, s = math.cos(th), math.sin(th)
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy) for dx, dy in ((L / 2, Wd / 2), (L / 2, -Wd / 2), (-L / 2, -Wd / 2), (-L / 2, Wd / 2))]


class Recorder:
    """Draws frames while a World runs."""

    def __init__(self, world: World):
        self.w = world
        base = build_texture()                                   # same image Flask serves to the browser
        # softened a little: the photographic grain is lovely on screen but makes GIFs huge
        self.bg = base.resize((int((W + 2 * M) * PX), int((W + 2 * M) * PX)), Image.LANCZOS).filter(ImageFilter.GaussianBlur(.8))
        self.frames: list[Image.Image] = []
        self.f_small, self.f_big = _font(11), _font(15)

    def _to_px(self, x, y, ox, oy):
        return ((x - ox) * PX, (y - oy) * PX)

    def frame(self):
        w, e = self.w, self.w.ego
        ox = min(max(e.x - VIEW_W / 2, -M), W + M - VIEW_W)
        oy = min(max(e.y - VIEW_H / 2, -M), W + M - VIEW_H)
        x0, y0 = int((ox + M) * PX), int((oy + M) * PX)
        img = self.bg.crop((x0, y0, x0 + VIEW_W * PX, y0 + VIEW_H * PX)).convert("RGB")
        d = ImageDraw.Draw(img, "RGBA")
        P_ = lambda x, y: self._to_px(x, y, ox, oy)  # noqa: E731

        for c in CTRL.values():                                  # live signal bars on the stop lines
            if c.type != "signal" or abs(c.i * P - e.x) > VIEW_W or abs(c.j * P - e.y) > VIEW_H:
                continue
            for dd in range(4):
                if approach_rule(c, axis_of(dd)) != "signal":
                    continue
                st = sig_state(c, axis_of(dd), w.t)[0]
                hx = dd % 2 == 0
                s = 1 if dd in (0, 1) else -1
                rr = DIR[(dd + 1) % 4][1 if hx else 0]
                al = (c.i if hx else c.j) * P - s * (STOPL + .2)
                a, b = (c.j if hx else c.i) * P + rr * .3, (c.j if hx else c.i) * P + rr * 3.7
                p1, p2 = ((al, a), (al, b)) if hx else ((a, al), (b, al))
                d.line([P_(*p1), P_(*p2)], fill=SIGCOL[st] + (240,), width=3)

        r = w.route
        if r is not None:                                        # planned path ahead
            pts = [P_(r.px[q], r.py[q]) for q in range(e.pi, min(r.n, e.pi + 260), 6)]
            if len(pts) > 1:
                d.line(pts, fill=(70, 200, 255, 170), width=3)

        for o in w.actors:
            if abs(o.x - e.x) > VIEW_W or abs(o.y - e.y) > VIEW_H:
                continue
            if o.kind == "car":
                th = math.atan2(DIR[o.d][1], DIR[o.d][0])
                col = (218, 41, 28) if o.tram else tuple(o.color)
                d.polygon([P_(*p) for p in _rect(o.x, o.y, o.L, o.Wd, th)], fill=col + (255,), outline=(20, 20, 20, 255))
            else:
                rr_ = {"ball": .25, "cat": .35, "dog": .5, "raccoon": .45}.get(o.kind, .45) * PX
                cx, cy = P_(o.x, o.y)
                d.ellipse([cx - rr_, cy - rr_, cx + rr_, cy + rr_], fill=KIND_COL.get(o.kind, (255, 255, 255)) + (255,),
                          outline=(20, 20, 20, 255))

        d.polygon([P_(*p) for p in _rect(e.x, e.y, e.L, e.Wd, e.th)], fill=(245, 245, 245, 255), outline=(40, 210, 255, 255), width=2)

        # panel
        ctl = e.ctl
        lines = [f"t {w.t / 60:4.1f} min   {e.v * 3.6:3.0f} km/h   limit {e.vlim * 3.6:.0f}",
                 f"decision: {w.decision.act[1]}"]
        if ctl:
            what = {"signal": "light", "stop": "stop sign", "yield": "yield", "free": "right of way"}[ctl["rule"]]
            state = f" {ctl['state']}" if ctl.get("state") else ""
            hold = f", holding ({ctl['why']})" if ctl["hold"] else ""
            lines.append(f"next: {what}{state} at {ctl['c'].name.replace(' St W', '').replace(' Ave', '')} {max(0, ctl['g']):.0f} m{hold}")
        lines.append(f"trips {w.stats['trips']}   collisions {w.stats['collisions']}   violations {w.rules.violations}")
        d.rectangle([6, 6, 6 + 330, 10 + 17 * len(lines)], fill=(10, 14, 20, 200))
        for k, ln in enumerate(lines):
            d.text((12, 9 + 17 * k), ln, fill=(230, 238, 245, 255), font=self.f_small)
        self.frames.append(img)


def record_gif(minutes: float = 1.0, seed: int = 1, fps: int = 8, out: str | None = None, warmup_s: float = 5.0) -> str:
    """Run a World and save a GIF of `minutes` of driving (after a short warm-up)."""
    out = out or os.path.join("runs", f"replay-{time.strftime('%Y%m%d-%H%M%S')}.gif")
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    w = World(seed=seed)
    rec = Recorder(w)
    for _ in range(int(warmup_s / DT)):
        w.step(DT)
    every = max(1, round(1 / (fps * DT)))
    for i in range(int(minutes * 60 / DT)):
        w.step(DT)
        if i % every == 0:
            rec.frame()
    pal = rec.frames[len(rec.frames) // 2].quantize(colors=128, method=Image.MEDIANCUT)    # one shared palette
    frames = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in rec.frames]
    frames[0].save(out, save_all=True, append_images=frames[1:], duration=int(1000 / fps), loop=0, optimize=True)
    return out
