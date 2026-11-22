"""Headless Python simulator: the robotaxi, traffic, pedestrians, animals and traffic rules without graphics.

    from robotaxi.sim import World, DT
    w = World(seed=1)
    for _ in range(int(60 / DT)):
        w.step(DT)
    print(w.stats, w.rule_summary())

Run it from the command line with simulate.py. Modules:
    geometry    city constants and helpers, built from data/toronto_downtown.osm
    traffic     traffic lights, stop / yield signs, speed limits, walk signals (twin of 13-traffic.js)
    routing     Dijkstra over (intersection, heading) states and smooth bezier paths
    agents      cars and streetcars (vectorised IDM in NumPy), pedestrians and animals
    control     vehicle physics, PID speed control, Ziegler-Nichols, pure pursuit, bicycle model
    localization  wheel / gyro odometry with map fixes, phone radio model, particle filter
    planner     collision-risk prediction and the six-action decision maker
    ride        ride requests, finding the rider by radio signal, pickups and drop-offs
    scenarios   crunch-time tests (jaywalker, child and ball, raccoon, dog, hard braking, red-light runner)
    world       the World class that owns everything and records data for pandas
    export      CSVs, summary table and matplotlib report for a run
"""
from .world import DT, World

__all__ = ["DT", "World"]
