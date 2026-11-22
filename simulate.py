"""Run the robotaxi simulation headless in Python and get a full report.

    python simulate.py                      10 simulated minutes, seed 1
    python simulate.py --minutes 30 --seed 7
    python simulate.py --runs 5             five seeds back to back, results compared in one table
    python simulate.py --no-hazards         turn off the auto-spawned crunch-time tests
    python simulate.py --benchmark          replay every crunch-time test 10 times and score the planner
    python simulate.py --sweep gps_sigma    rerun with different values of one setting and compare
    python simulate.py --gif --minutes 1    save an animated GIF of the drive

Same map, traffic lights, stop / yield signs, speed limits and decision logic as the browser version,
just without graphics, so it runs about 30 times faster than real time. Each run writes CSVs, a summary
and a matplotlib report to runs/<time>/.
"""
import argparse
import os
import time

import pandas as pd

from robotaxi.sim import DT, World
from robotaxi.sim.benchmark import run_benchmark
from robotaxi.sim.export import export_run
from robotaxi.sim.replay import record_gif
from robotaxi.sim.sweep import PARAMS, run_sweep


def run_one(minutes, seed, hazards, out):
    w = World(seed=seed)
    w.auto_chaos = hazards
    steps = int(minutes * 60 / DT)
    t0 = time.time()
    for i in range(steps):
        w.step(DT)
        if i and i % int(5 * 60 / DT) == 0:
            r = w.rules
            print(f"  seed {seed}  t={w.t / 60:4.0f} min  trips={w.stats['trips']:2d}  collisions={w.stats['collisions']}"
                  f"  red stops={r.red_stops}  stop signs={r.sign_stops}  violations={r.violations}")
    folder, summary = export_run(w, out)
    row = dict(seed=seed, minutes=minutes, wall_s=round(time.time() - t0), trips=w.stats["trips"],
               km=round(w.stats["dist"] / 1000, 2), collisions=w.stats["collisions"],
               emergency_stops=w.stats["emergency"], **w.rule_summary())
    return folder, summary, row


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--minutes", type=float, default=10, help="simulated minutes per run")
    ap.add_argument("--seed", type=int, default=1, help="random seed of the first run")
    ap.add_argument("--runs", type=int, default=1, help="number of runs with consecutive seeds")
    ap.add_argument("--out", default=None, help="output folder (default runs/<time>)")
    ap.add_argument("--no-hazards", action="store_true", help="disable the auto-spawned crunch-time tests")
    ap.add_argument("--benchmark", action="store_true", help="score the planner on every crunch-time scenario")
    ap.add_argument("--trials", type=int, default=10, help="trials per scenario for --benchmark")
    ap.add_argument("--sweep", choices=sorted(PARAMS), help="parameter to sweep")
    ap.add_argument("--seeds", type=int, default=3, help="seeds per value for --sweep")
    ap.add_argument("--gif", action="store_true", help="save an animated GIF of --minutes of driving")
    args = ap.parse_args()

    if args.gif:
        path = record_gif(args.minutes, args.seed, out=args.out)
        print(f"GIF written to {path}")
        return
    if args.sweep:
        table, folder = run_sweep(args.sweep, args.minutes, args.seeds, out_dir=args.out)
        print("\n" + table.to_string())
        print(f"\nFiles written to {folder}/: runs.csv, by_value.csv, sweep.png")
        return

    if args.benchmark:
        _, summary, folder = run_benchmark(args.trials, args.seed, args.out)
        print("\n" + summary.to_string())
        print(f"\nFiles written to {folder}/: trials.csv, summary.csv, benchmark.png")
        return

    base = args.out or os.path.join("runs", time.strftime("%Y%m%d-%H%M%S"))
    rows = []
    for k in range(args.runs):
        seed = args.seed + k
        out = base if args.runs == 1 else os.path.join(base, f"seed{seed}")
        folder, summary, row = run_one(args.minutes, seed, not args.no_hazards, out)
        rows.append(row)
        if args.runs == 1:
            pd.set_option("display.max_colwidth", 90)
            print("\n" + summary.to_string())
        print(f"\nFiles written to {folder}/")
    if args.runs > 1:
        table = pd.DataFrame(rows).set_index("seed")
        table.to_csv(os.path.join(base, "comparison.csv"))
        print("\n" + table.to_string())
        print(f"\nTotals: {table.trips.sum()} trips, {table.km.sum():.1f} km, {table.collisions.sum()} collisions, "
              f"{table.violations.sum()} rule violations")


if __name__ == "__main__":
    main()
