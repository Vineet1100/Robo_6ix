"""City geometry for the headless simulator.

Everything comes from the same map file the web app uses (data/toronto_downtown.osm, read with
BeautifulSoup in robotaxi.citymap), so the Python simulator and the browser drive the same streets,
speed limits and intersection controls.

Coordinates are local metres: x grows east, y grows south (screen convention).
Roads run along x = i*P (north-south avenues) and y = j*P (east-west streets).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..citymap import load_map


@dataclass
class CityMap:
    name: str
    P: float                 # block pitch (m)
    N: int                   # blocks per side
    vlim: float              # default speed limit (m/s)
    margin: float
    sensor_range: float
    hnames: list = field(default_factory=list)      # east-west streets, north -> south
    vnames: list = field(default_factory=list)      # north-south avenues, west -> east
    tram_roads: list = field(default_factory=list)  # (axis, index, route)
    blocks: dict = field(default_factory=dict)      # (i, j) -> (style, name)
    pois: list = field(default_factory=list)        # (name, x, y)
    intersections: dict = field(default_factory=dict)  # (i, j) -> name
    controls: dict = field(default_factory=dict)    # (i, j) -> dict(type, approach, offset)
    maxspeed_h: list = field(default_factory=list)  # km/h per east-west street
    maxspeed_v: list = field(default_factory=list)  # km/h per north-south avenue


def city_from_map(m: dict) -> CityMap:
    key = lambda s: tuple(int(v) for v in s.split(","))  # noqa: E731
    return CityMap(
        name=m["name"], P=m["block_pitch"], N=m["blocks_per_side"], vlim=m["speed_limit_ms"],
        margin=m.get("margin", 40.0), sensor_range=m["sensor_range"], hnames=m["hnames"], vnames=m["vnames"],
        tram_roads=[(t["axis"], t["index"], t["route"]) for t in m["trams"]],
        blocks={key(k): (b["style"], b["name"]) for k, b in m["blocks"].items()},
        pois=[(p["name"], p["x"], p["y"]) for p in m["pois"]],
        intersections={key(k): v for k, v in m["intersections"].items()},
        controls={key(k): c for k, c in m["controls"].items() if c},
        maxspeed_h=list(m["maxspeed_h"]), maxspeed_v=list(m["maxspeed_v"]),
    )


CITY = city_from_map(load_map())

# ---------------------------------------------------------------- constants (same as static/js/01-config.js)
P = CITY.P
N = CITY.N
W = P * N
M = CITY.margin
RH = 4.0                    # half road width (one lane each way)
LO = 2.0                    # lane-centre offset from road centre
SW = 6.5                    # road centre to back of sidewalk
NODE = 5.2                  # where pedestrians walk / wait
TS = 8                      # aerial texture pixels per metre
RANGE = CITY.sensor_range
VLIM = max(CITY.maxspeed_h + CITY.maxspeed_v) / 3.6 if CITY.maxspeed_h else CITY.vlim   # highest limit anywhere
HNAME = CITY.hnames
VNAME = CITY.vnames
DNAME = ["eastbound", "southbound", "westbound", "northbound"]

DIR = [(1, 0), (0, 1), (-1, 0), (0, -1)]   # E, S, W, N


def right_v(d):
    return DIR[(d + 1) % 4]


def sgn_d(d):
    return 1 if d in (0, 1) else -1


def horiz(d):
    return d % 2 == 0


def clamp(v, a, b):
    return a if v < a else b if v > b else v


def ang_diff(a, b):
    """Signed smallest angle from a to b."""
    return (b - a + math.pi) % (2 * math.pi) - math.pi


def lane_coord(d, rc):
    """Perpendicular coordinate of the lane centre for travel direction d on road centre rc."""
    return rc + LO * right_v(d)[1 if horiz(d) else 0]


def on_road(x, y):
    if x < -RH or y < -RH or x > W + RH or y > W + RH:
        return False
    return abs(x - round(x / P) * P) < RH or abs(y - round(y / P) * P) < RH


def in_grid(i, j):
    return 0 <= i <= N and 0 <= j <= N


def heading_to_dir(th):
    return int(round(th / (math.pi / 2))) % 4


def mulberry32(seed: int):
    """Same small seeded generator as mulberry() in static/js/01-config.js, so Python and the browser
    produce identical "random" layouts (wet patches) from the same seed."""
    state = [seed & 0xFFFFFFFF]

    def nxt() -> float:
        state[0] = (state[0] + 0x6D2B79F5) & 0xFFFFFFFF
        s = state[0]
        t = ((s ^ (s >> 15)) * (1 | s)) & 0xFFFFFFFF                       # Math.imul(s ^ s>>>15, 1|s)
        t = ((t + (((t ^ (t >> 7)) * (61 | t)) & 0xFFFFFFFF)) & 0xFFFFFFFF) ^ t   # (t + imul(t ^ t>>>7, 61|t)) ^ t
        return ((t ^ (t >> 14)) & 0xFFFFFFFF) / 4294967296
    return nxt


def wet_zones():
    """The 16 wet patches; identical to wetZones in static/js/05-dynamics-pid.js."""
    r = mulberry32(77)
    out = []
    for _ in range(16):
        vert = r() < .5
        ri = math.floor(r() * (N + 1))
        al = math.floor(r() * N) * P + 12 + r() * (P - 24)
        off = (r() - .5) * 4
        rad = 5 + r() * 7
        out.append((ri * P + off, al, rad) if vert else (al, ri * P + off, rad))
    return out
