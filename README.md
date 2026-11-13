# Phantom6ix

### A digital prototype of a self driving robotaxi for downtown Toronto

Phantom6ix drives itself through a simulated downtown core, from Spadina Avenue to Yonge Street and from Queen Street down to Front Street. It keeps an eye on every pedestrian, car, TTC streetcar and the occasional raccoon, predicts what each one will do next, and decides in under a millisecond whether to cruise, ease off, brake or steer around them.

You can book a ride on a map of downtown, watch the taxi find you using your phone's signal, and see it pull up with the door right beside you. Behind the scenes, a Python back end studies every trip, so the taxi gets better at finding riders and estimating arrival times the longer it drives.

<br>

## Highlights

**It sees and decides**

- Tracks everything within 60 metres and rates how confident it is in each detection
- Predicts where every object will be over the next four seconds and works out the chance of a collision
- Compares six possible moves on safety, comfort and progress, then picks the best one
- Handles crunch time moments on demand, like a jaywalker, a child chasing a ball, a raccoon frozen in the lane, a dog darting across, a sudden stop ahead and a driver running a red light

**It drives**

- Plans routes across the street grid with Dijkstra's algorithm
- Steers with pure pursuit, a path tracking method used on real autonomous vehicles
- Controls speed with a PID controller tuned by Ziegler Nichols at every point on the road, since speed, passenger weight and wet pavement all change how the car responds
- Keeps track of its own position with wheel and gyro odometry and corrects the drift with regular map fixes

**It picks you up**

- Book a ride by tapping a pickup spot and a destination on a full map of downtown
- The taxi heads to your phone's GPS location, which is usually off by several metres
- Once it is close, it reads your phone's Bluetooth signal and direction and runs a particle filter to pinpoint exactly where you are standing
- It then lines its rear door up with you, even if that means circling the block because GPS put you on the wrong side of the street

**It learns from data**

- The browser sends driving data to a Flask server every two seconds
- pandas summarises each run
- scikit-learn learns how your phone's signal fades with distance and feeds that straight back into the taxi
- A second model learns real trip times and powers every ETA in the app
- Clustering finds the intersections where risky moments keep happening and marks them on the map
- One click downloads all the data as CSV files along with a full report

<br>

## Results

| What I measured | Result |
| --- | --- |
| Door to rider distance using GPS alone | about 6 to 12 metres |
| Door to rider distance using GPS plus the phone's signal | about 1 to 1.5 metres |
| Phone signal model learned by scikit-learn | within 0.2 percent of the hidden true values |
| Average speed tracking error | about 0.3 m/s |
| Time to make each driving decision | well under 1 millisecond |
| Collisions in 30 minute runs with constant hazards | zero in most runs |

Before the signal model was trained, one pickup stopped 20 metres away from the rider. After training, every pickup landed within about a metre and a half.

<br>

## Getting started

You will need Python 3.10 or newer.

**On a Mac**

1. Download or clone this repository
2. Double click **Start Robotaxi.command**
3. If macOS warns you about an unidentified developer, right click the file, choose Open, then click Open again
4. Your browser opens at http://127.0.0.1:5000 and the top bar shows **python connected**

**On Windows**

1. Download or clone this repository
2. Double click **start_windows.bat**

**From the terminal**

```bash
git clone https://github.com/Vineet1100/Phantom6ix.git
cd Phantom6ix
pip install -r requirements.txt
python app.py
```

You can also open `static/index.html` by itself. The simulation runs fully, just without the Python analytics.

<br>

## Controls

| Action | What it does |
| --- | --- |
| Request a ride, the B key or clicking the minimap | Opens the booking map |
| Keys 1 to 6 | Triggers a crunch time scenario |
| Space | Pauses the simulation |
| H | Switches between heading up and north up |
| T | Tilts the map into 3D |
| Scroll | Zooms in and out |

The sidebar on the left has a page for each part of the system: Overview, Perception and Decisions, Ride Hailing, Navigation and Steering, PID and Ziegler Nichols, Localization, Python Analytics, Event Log and How It Works.

<br>

## Headless simulator

The whole simulation also runs in Python with no graphics: the same streets, traffic, hazards, planner, PID controller and particle filter as the browser. It runs about 15 times faster than real time, so you can do long runs, many seeds and before and after comparisons from the terminal.

```bash
python -m robotaxi.sim --minutes 30 --seed 7              # one 30 minute run
python -m robotaxi.sim --runs 6 --minutes 10 --calibrate  # retrain the phone signal model after every run
python -m robotaxi.sim --minutes 10 --export run.zip      # same CSVs and report as the web app's download
python -m robotaxi.sim --help                             # all options
```

Every run records the same tables the browser sends to Flask, so the pandas and scikit-learn analytics and the report work on it unchanged. With `--calibrate`, the radio model learned from earlier runs is fed into the particle filter for the next run, the same way the browser picks it up from the server.

It is a line for line port of the browser code, and the tests check it against the JavaScript: the seeded wet patches, hills and Ziegler Nichols numbers match exactly, and runs of the two versions give the same distances, trip counts and pickup accuracy.

Six 10 minute runs with a crunch time test every 9 to 15 seconds and the phone signal model retrained after each run (`python -m robotaxi.sim --runs 6 --minutes 10 --calibrate`):

| What I measured | Result |
| --- | --- |
| Door to rider distance using GPS plus the phone's signal | 1.7 m on average; 1.0 to 1.9 m per run once the model is trained |
| Door to rider distance if the taxi had trusted GPS alone | 9.0 m on average |
| Phone signal model learned by scikit-learn | P0 -40.9 dBm and n 2.30, against true values of -41 and 2.3 |
| Average speed tracking error | 0.23 to 0.27 m/s |
| Crunch time tests survived | 153 tests across 60 minutes; 4 of 6 runs had no contact at all |

<br>

## Tests

```bash
pip install pytest
pytest
```

The tests cover routing to every block, path smoothness, pure pursuit tracking, the PID controller on wet and dry roads, braking for a child in the lane, the particle filter finding a rider, every crunch time test without a collision, learning the phone signal model, and the Flask API fed with a headless run. GitHub Actions runs them on every push.

<br>

## How it is built

The live simulation and graphics run in the browser at 60 frames per second, while everything to do with data runs in Python. Sending every frame to Python and back would make the car react late, so each side does what it is best at. The headless simulator in `robotaxi/sim` runs the same simulation in Python when no one needs to watch it.

| Tool | What it does here |
| --- | --- |
| Flask | Runs the web server and API |
| BeautifulSoup | Reads the downtown Toronto map file with streets, landmarks, streetcar routes and the speed limit |
| NumPy | Calculates Ziegler Nichols tuning along every new route |
| pandas | Logs every run, builds summaries and exports CSV files |
| scikit-learn | Learns the phone signal model and trip times, and finds risk hotspots |
| matplotlib | Creates the downloadable run report |
| JavaScript and HTML Canvas | Runs the real time simulation, decisions, control and graphics |
| Python headless simulator | Runs the same simulation with no graphics, for long batch runs, tests and before and after comparisons |
| pytest | Checks the routing, physics, planner, particle filter, server and whole runs |

<br>

## Project layout

```text
app.py                     Flask server
requirements.txt           Python packages
pyproject.toml             Test settings
Start Robotaxi.command     Mac launcher
start_windows.bat          Windows launcher

data/
  toronto_downtown.osm     Streets, landmarks, streetcar routes and speed limit

robotaxi/
  citymap.py               Reads the map with BeautifulSoup
  vehicle.py               Car weight, motor power and braking delay
  zn.py                    Ziegler Nichols maths with NumPy
  store.py                 Holds incoming data for pandas
  analytics.py             Statistics, learned models and hotspots
  report.py                Report and CSV export
  sim/                     Headless simulator (python -m robotaxi.sim)
    common.py              Grid, lanes, geometry and object classes
    agents.py              Cars, streetcars, people and animals
    navigation.py          Routing, pure pursuit and odometry
    dynamics.py            Vehicle physics, PID and Ziegler Nichols
    ride_hailing.py        Ride requests, phone signal and particle filter
    planner.py             Prediction, collision risk and decisions
    scenarios.py           Crunch time tests
    world.py               World setup, each step and the recorded tables
    cli.py                 Batch runs from the terminal

tests/                     pytest suite

static/
  index.html               Page layout and sidebar
  css/styles.css           Colours and styling
  js/01-config.js          Settings and the link to Python
  js/02-texture.js         Draws the city and its landmarks
  js/03-agents.js          Cars, streetcars, people and animals
  js/04-navigation.js      Routing, steering and odometry
  js/05-dynamics-pid.js    Vehicle physics, PID and Ziegler Nichols
  js/06-ride-hailing.js    Ride requests and finding the rider
  js/07-planner.js         Collision risk and decision making
  js/08-scenarios.js       Crunch time tests
  js/09-simulation.js      World setup and each simulation step
  js/10-render.js          Drawing the map
  js/11-booking.js         Booking map
  js/12-ui.js              Panels, controls and sidebar pages
```

<br>

## Want to tweak it?

| To change | Look in |
| --- | --- |
| Street names, landmarks, streetcar routes or the speed limit | `data/toronto_downtown.osm` |
| Car weight, motor power or braking delay | `robotaxi/vehicle.py` |
| How cautious the car drives | `static/js/07-planner.js` and `robotaxi/sim/planner.py` |
| How accurate the phone's GPS is | `static/js/06-ride-hailing.js` and `robotaxi/sim/ride_hailing.py` |
| How many cars, people and animals appear | `static/js/09-simulation.js`, or `Population` in `robotaxi/sim/world.py` |
| What the analytics calculate | `robotaxi/analytics.py` |
| The charts in the report | `robotaxi/report.py` |
| Colours and styling | `static/css/styles.css` |

<br>

## Good to know

This is a prototype built to explore how an autonomous taxi thinks. It is not software for a real vehicle.

- The map is a simplified grid that uses real downtown Toronto street names, and the landmarks are stylised rather than exact
- The sensors are simulated, so detections come from the simulator with added noise and buildings do not block the view
- There are no traffic lights, and other cars yield to the taxi at intersections except for the red light runner test
- Real autonomous vehicles usually run C++ and ROS on board, with Python used for tools and machine learning
- The driving logic lives in two places, the browser and the headless simulator, so a change to one should be made in the other too. The tests that compare them against the JavaScript will catch some differences, not all

<br>
