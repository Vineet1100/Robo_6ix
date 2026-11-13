"""Flask server for the downtown-Toronto robotaxi prototype.

    python app.py            -> http://127.0.0.1:5000

The browser runs the real-time simulation and drawing (60 fps needs to stay client-side).
Python does the data work: the map (BeautifulSoup), Ziegler–Nichols route profiles (NumPy),
logging + statistics (pandas), radio-model calibration, trip-time model and hotspot
clustering (scikit-learn), and the downloadable report (matplotlib).
"""
import argparse
import os
import threading
import time
import webbrowser

from flask import Flask, Response, jsonify, request, send_file

from robotaxi.analytics import analyze, predict_eta
from robotaxi.citymap import load_map
from robotaxi.report import export_zip, report_png
from robotaxi.store import STORE
from robotaxi.zn import route_profile

ROOT = os.path.dirname(os.path.abspath(__file__))
app = Flask(__name__, static_folder="static", static_url_path="")
_cache = {"t": 0.0, "data": None}


def frames():
    return STORE.frames()


def cached_analysis(max_age=2.0):
    if _cache["data"] is None or time.time() - _cache["t"] > max_age:
        _cache.update(t=time.time(), data=analyze(frames()))
    return _cache["data"]


@app.get("/")
def index():
    return send_file(os.path.join(ROOT, "static", "index.html"))


@app.get("/api/map")
def api_map():
    return jsonify(load_map())


@app.post("/api/ingest")
def api_ingest():
    return jsonify(ok=True, counts=STORE.ingest(request.get_json(force=True) or {}))


@app.get("/api/analytics")
def api_analytics():
    return jsonify(cached_analysis())


@app.post("/api/zn_profile")
def api_zn_profile():
    d = request.get_json(force=True)
    return jsonify(route_profile(d["s"], d["k"], d["mu"], float(d["mass"]), float(d["vlim"])))


@app.get("/api/eta")
def api_eta():
    length, turns = float(request.args.get("length", 0)), int(request.args.get("turns", 0))
    model = cached_analysis(5)["eta"]
    return jsonify(seconds=round(predict_eta(model, length, turns), 1), model="scikit-learn LinearRegression" if model else "heuristic (needs 3+ trips)")


@app.get("/api/report.png")
def api_report():
    return Response(report_png(frames()), mimetype="image/png")


@app.get("/api/export.zip")
def api_export():
    data = export_zip(frames())
    return Response(data, mimetype="application/zip",
                    headers={"Content-Disposition": f'attachment; filename="robotaxi_run_{time.strftime("%Y%m%d-%H%M%S")}.zip"'})


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()
    url = f"http://127.0.0.1:{args.port}/"
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    print(f"Robotaxi prototype: {url}")
    app.run(host="127.0.0.1", port=args.port, debug=False, threaded=True)
