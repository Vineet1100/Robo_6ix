# Robotaxi Prototype — Downtown Toronto (Flask + Python data stack)

## Run it

**Mac:** double-click **`Start Robotaxi.command`**. If macOS warns that it's from an unidentified developer, right-click it, choose **Open**, then **Open** again (only needed once).
It installs the Python packages and opens **http://127.0.0.1:5000** in your browser. Leave the Terminal window open while you use it.

**Windows:** double-click **`start_windows.bat`**.

**Terminal (any OS):**
```bash
pip3 install -r requirements.txt
python3 app.py              # opens http://127.0.0.1:5000
```

Opening `static/index.html` directly (double-clicking it) also works, but in **offline mode**: the simulation runs and
the page says "python offline", with no Python analytics. For the full version use the launcher, which serves
the page at http://127.0.0.1:5000.

## What runs in Python, and why

| Job | Where | Library |
|---|---|---|
| Web server + API | `app.py` | **Flask** |
| City map: street names, block styles, landmarks, TTC routes, speed limits, traffic lights, stop and yield signs | `robotaxi/citymap.py` + `data/toronto_downtown.osm` | **BeautifulSoup** |
| Vehicle parameters (mass, power, delays) | `robotaxi/vehicle.py` | — (single source of truth, sent to the browser) |
| Ziegler–Nichols Ku/Tu profile for every new route | `robotaxi/zn.py` | **NumPy** (vectorised bisection) |
| Logging telemetry, events, pickups, trips, radio samples, ZN log | `robotaxi/store.py` | **pandas** |
| Summary statistics | `robotaxi/analytics.py` → `summary` | **pandas** |
| Radio path-loss calibration (fed back into the particle filter) | `analytics.py` → `rf_calibration` | **scikit-learn** LinearRegression |
| Trip-time / ETA model (used for every ETA on the page) | `analytics.py` → `eta_model` | **scikit-learn** LinearRegression |
| Risk hotspots (red dashed circles on the map) | `analytics.py` → `hotspots` | **scikit-learn** DBSCAN |
| Ziegler–Nichols by surface/mode | `analytics.py` → `zn_by_condition` | **pandas** groupby |
| Traffic-rule compliance: time over each street's limit, stops at lights, stop and yield signs | `analytics.py` → `rules_check` | **pandas** |
| Downloadable report + CSV zip | `robotaxi/report.py` | **matplotlib**, pandas |

**Kept in JavaScript on purpose:** the real-time simulation (traffic, perception, collision risk, decisions, PID loop,
pure pursuit, rider particle filter) and all drawing. These run 30–60 times a second, and a round-trip to Python for
every frame would make the car react late and the map stutter. Routing also stays in JS, because the car re-plans
mid-drive many times a second as the radio fix improves.

## How the two halves talk

```
browser (static/js)                         Flask (app.py + robotaxi/)
 start-up ── GET  /api/map ───────────────► BeautifulSoup map + vehicle params
 new route ─ POST /api/zn_profile ────────► NumPy Ziegler–Nichols along the route
 every 2 s ─ POST /api/ingest ────────────► pandas store (telemetry, events, pickups, trips, rf, zn_log)
 every 4 s ─ GET  /api/analytics ◄──────── pandas summary + scikit-learn fits + DBSCAN hotspots
 buttons ─── GET  /api/export.zip, /api/report.png (matplotlib)
```

## Project layout / what to change where

```
app.py                     Flask routes (add a new API endpoint here)
requirements.txt
data/toronto_downtown.osm  THE MAP: street names, block style per block, landmarks, streetcar routes,
                           speed limit per street, and the control at every intersection
robotaxi/
  citymap.py               XML → JSON for the browser
  vehicle.py               car mass, motor power, actuator delays
  zn.py                    Ziegler–Nichols maths (NumPy)
  store.py                 what gets logged + row limits
  analytics.py             statistics, ML models, hotspot settings (DBSCAN eps / min_samples)
  report.py                matplotlib report layout, export contents
static/index.html          page layout: left navigation, map, and one panel page per concept (served at / by Flask)
static/css/styles.css      colours / styling
static/js/01-config.js     constants, object sizes, risk-model numbers, Python bridge (PY / OUT / api)
static/js/02-texture.js    drawing of buildings and landmarks
static/js/03-agents.js     cars, streetcars, people, animals
static/js/04-navigation.js routing, pure pursuit, odometry
static/js/05-dynamics-pid.js  PID + live Ziegler–Nichols (asks Python for route profiles)
static/js/06-ride-hailing.js  ride flow, radio model (RF truth vs RF_MODEL fitted in Python)
static/js/07-planner.js    collision risk + decision cost
static/js/08-scenarios.js  crunch-time tests 1–6
static/js/09-simulation.js world setup, step, telemetry sampling
static/js/10-render.js     drawing the map view (+ hotspots)
static/js/11-booking.js    A → B booking map (ETAs from the Python model)
static/js/12-ui.js         panels, controls, start-up, Python sync + analytics panel
static/js/13-traffic.js    traffic lights, stop / yield signs, speed limits, walk signals, signs on the map
```

| I want to change… | Edit |
|---|---|
| Street names, which landmark is on which block, streetcar routes, speed limits | `data/toronto_downtown.osm` |
| Which intersection has a light, stop sign or yield sign | `data/toronto_downtown.osm` (`highway=traffic_signals`, `stop`, `give_way`) |
| Light timing, how cars behave at stop signs, amber decision | `static/js/13-traffic.js` → `SIGT`, `ctrlDecide` |
| Car mass / motor power / braking delay | `robotaxi/vehicle.py` |
| What the analytics compute, hotspot sensitivity | `robotaxi/analytics.py` (`DBSCAN(eps=10, min_samples=4)`) |
| Charts in the downloadable report | `robotaxi/report.py` |
| The phone's true radio behaviour vs. the taxi's starting guess | `static/js/06-ride-hailing.js` → `RF` (truth) and `RF_MODEL` (initial guess) |
| How often data is sent / analysed | `static/js/12-ui.js` → `setInterval(pushToPython,2000)`, `setInterval(pullAnalytics,4000)` |
| Everything else about the simulation | see the `static/js` table above |

## Concept pages (left sidebar)

The left sidebar switches the right-hand panel between pages: **Overview, Perception & Decisions, Ride-hailing,
Navigation & Steering, Traffic Rules, PID & Ziegler–Nichols, Localization, Python Analytics, Event Log, How It Works**.
Each page has a short intro explaining the concept. Pages have their own links (e.g. `http://127.0.0.1:5000/#control`).

To move a panel to a different page, change its `data-page="…"` in `static/index.html`. To add a page, add a link
in `<nav id="pnav">` and give its sections the same `data-page` id.
