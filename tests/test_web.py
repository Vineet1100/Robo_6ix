"""The Flask app serves the page, the map, the Python-drawn city and the analytics."""
import pytest

from app import app


@pytest.fixture()
def client():
    app.config.update(TESTING=True)
    return app.test_client()


def test_page_and_map(client):
    assert client.get("/").status_code == 200
    m = client.get("/api/map").get_json()
    assert len(m["controls"]) == 36 and m["maxspeed_h"][-1] == 50


def test_aerial_image_is_drawn_in_python(client):
    r = client.get("/api/texture.jpg")
    assert r.status_code == 200 and r.mimetype == "image/jpeg" and len(r.data) > 200_000


def test_zn_profile_takes_a_limit_per_point(client):
    body = dict(s=[0, 5, 10, 15], k=[0, 0, 0, 0], mu=[.9] * 4, mass=1900, vlim=[13.9, 13.9, 8.3, 8.3])
    out = client.post("/api/zn_profile", json=body).get_json()
    assert len(out["v"]) == 4 and out["v"][2] <= 8.3


def test_eta_endpoint(client):
    r = client.get("/api/eta?length=300&turns=2&controls=3").get_json()
    assert r["seconds"] > 0
