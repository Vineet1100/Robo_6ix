"""City model loaded from an OSM-style XML file with BeautifulSoup.

The browser asks for /api/map at start-up and builds the streets, block styles, landmarks,
streetcar routes, speed limits, traffic signals, stop and yield signs and vehicle parameters from this data.
"""
from __future__ import annotations

import os
import warnings
from functools import lru_cache

from bs4 import BeautifulSoup, XMLParsedAsHTMLWarning

from .vehicle import VEHICLE

warnings.filterwarnings("ignore", category=XMLParsedAsHTMLWarning)
DATA = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", "toronto_downtown.osm")


@lru_cache(maxsize=1)
def load_map(path: str = DATA) -> dict:
    with open(path, encoding="utf-8") as f:
        soup = BeautifulSoup(f.read(), "html.parser")
    meta = soup.find("meta")
    h, v, trams, maxh, maxv = {}, {}, [], {}, {}
    for way in soup.find_all("way"):
        tags = {t["k"]: t["v"] for t in way.find_all("tag")}
        idx = int(way["index"])
        (h if way["axis"] == "h" else v)[idx] = tags["name"]
        (maxh if way["axis"] == "h" else maxv)[idx] = float(tags.get("maxspeed", meta["speed_limit_kmh"]))
        if tags.get("railway") == "tram":
            trams.append(dict(axis=way["axis"], index=idx, route=tags.get("route", "")))
    blocks = {f'{b["i"]},{b["j"]}': dict(style=b["style"], name=b.get("name", "")) for b in soup.find_all("block")}
    intersections, controls = {}, {}
    for node in soup.find_all("node"):
        tags = {t["k"]: t["v"] for t in node.find_all("tag")}
        key = f'{node["i"]},{node["j"]}'
        intersections[key] = tags.get("name", "")
        controls[key] = _control(tags)
    return dict(
        name=meta["name"], block_pitch=float(meta["block_pitch"]), blocks_per_side=int(meta["blocks"]),
        speed_limit_ms=float(meta["speed_limit_kmh"]) / 3.6, sensor_range=float(meta["sensor_range"]),
        hnames=[h[k] for k in sorted(h)], vnames=[v[k] for k in sorted(v)], trams=trams, blocks=blocks,
        intersections=intersections, controls=controls,
        maxspeed_h=[maxh[k] for k in sorted(maxh)], maxspeed_v=[maxv[k] for k in sorted(maxv)],
        pois=[dict(name=p["name"], x=float(p["x"]), y=float(p["y"])) for p in soup.find_all("poi")],
        vehicle=VEHICLE, source=os.path.basename(path),
    )


def _control(tags: dict) -> dict | None:
    """Turn OSM-style tags into the simulator's intersection control.

    type: signal, allstop, stop (one road stops) or yield. approach says which road has to
    stop or yield: h for east-west traffic, v for north-south traffic.
    """
    hw = tags.get("highway")
    if hw == "traffic_signals":
        return dict(type="signal", offset=float(tags.get("signal:offset", 0)))
    if hw == "stop":
        if tags.get("stop") == "all":
            return dict(type="allstop")
        return dict(type="stop", approach=tags.get("control:approach", "v"))
    if hw == "give_way":
        return dict(type="yield", approach=tags.get("control:approach", "v"))
    return None


def nearest_street(x: float, y: float) -> str:
    """Human-readable location for a point, e.g. 'King St W near Bay St'."""
    m = load_map()
    P, N = m["block_pitch"], m["blocks_per_side"]
    vi, hj = min(max(round(x / P), 0), N), min(max(round(y / P), 0), N)
    if abs(y - hj * P) <= abs(x - vi * P):
        return f'{m["hnames"][hj]} near {m["vnames"][vi]}'
    return f'{m["vnames"][vi]} near {m["hnames"][hj]}'
