"""The map file is the single source of truth for both the browser and the Python simulator."""
from robotaxi.citymap import load_map, nearest_street
from robotaxi.sim.geometry import CITY, N, W, mulberry32, wet_zones


def test_map_has_every_block_and_street():
    m = load_map()
    assert m["blocks_per_side"] == 5
    assert len(m["hnames"]) == len(m["vnames"]) == 6
    assert len(m["blocks"]) == 7 * 7            # 5x5 inner blocks plus the ring around them
    assert {t["route"].split()[1] for t in m["trams"]} == {"501", "504", "510"}


def test_every_intersection_has_a_control():
    m = load_map()
    kinds = [c["type"] for c in m["controls"].values()]
    assert len(kinds) == (N + 1) ** 2
    assert kinds.count("signal") == 20
    assert kinds.count("allstop") == 6
    assert kinds.count("stop") == 6
    assert kinds.count("yield") == 4


def test_speed_limits_per_street():
    m = load_map()
    assert m["maxspeed_h"] == [40, 40, 40, 40, 40, 50]     # Front St is 50
    assert m["maxspeed_v"] == [50, 30, 50, 30, 40, 40]     # Spadina, University 50; John, York 30
    assert CITY.maxspeed_h == m["maxspeed_h"]


def test_nearest_street_names():
    assert "King" in nearest_street(130, 181)
    assert "Bay" in nearest_street(241, 70)


def test_mulberry_matches_the_browser():
    # first three values of mulberry(77) from static/js/01-config.js, computed in node
    r = mulberry32(77)
    assert [round(r(), 12) for _ in range(3)] == [0.425601355731, 0.042219757801, 0.696006729733]


def test_wet_zones_are_on_roads():
    for x, y, r in wet_zones():
        assert 5 <= r <= 12
        assert -5 < x < W + 5 and -5 < y < W + 5
