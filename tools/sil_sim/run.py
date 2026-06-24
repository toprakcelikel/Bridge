#!/usr/bin/env python3
"""
CLI runner / smoke test for the Elcano SIL simulator.

Examples
--------
    python run.py                         # square course, default knobs
    python run.py --course dogleg
    python run.py --kp-steering 8 --cruise 120
    python run.py --csv out.csv           # dump trajectory
    python run.py --plot                  # matplotlib path (needs matplotlib)

Prints the run summary and the scorecard value J (lower is better).
"""

import argparse
import csv
import os
import sys

# Allow running as a plain script from anywhere.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sim import run_sim, score_run          # noqa: E402
from courses import COURSES                  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--course", default="square", choices=sorted(COURSES))
    ap.add_argument("--max-seconds", type=float, default=120)
    ap.add_argument("--cruise", type=int, dest="cruise_cmPs")
    ap.add_argument("--kp-steering", type=int, dest="kp_steering")
    ap.add_argument("--waypoint-radius", type=int, dest="waypoint_radius_cm")
    ap.add_argument("--max-steer", type=int, dest="max_steer_deg")
    ap.add_argument("--deadband", type=int, dest="steer_deadband")
    ap.add_argument("--csv", help="write per-tick trajectory to this CSV file")
    ap.add_argument("--plot", action="store_true", help="plot path (matplotlib)")
    args = ap.parse_args()

    knobs = {}
    for k in ("cruise_cmPs", "kp_steering", "waypoint_radius_cm",
              "max_steer_deg", "steer_deadband"):
        v = getattr(args, k)
        if v is not None:
            knobs[k] = v

    samples, result, waypoints_cm = run_sim(
        args.course, knobs=knobs, max_seconds=args.max_seconds)
    j = score_run(samples, result, waypoints_cm)

    print("Course        : {0}".format(args.course))
    print("Knobs         : {0}".format(knobs or "(defaults)"))
    print("Finished      : {0}".format(result["finished"]))
    print("Finish time   : {0} s".format(result["finish_time_s"]))
    print("Waypoints     : {0}/{1}".format(
        result["waypoints_reached"], result["n_waypoints"]))
    print("Ticks         : {0}".format(result["ticks"]))
    print("Score J       : {0:.2f}  (lower is better)".format(j))

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(samples[0].keys()))
            writer.writeheader()
            writer.writerows(samples)
        print("Trajectory    : wrote {0} rows to {1}".format(len(samples), args.csv))

    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not installed; skipping plot "
                  "(pip install matplotlib)")
            return 0
        xs = [s["east_cm"] / 100.0 for s in samples]
        ys = [s["north_cm"] / 100.0 for s in samples]
        wx = [e / 100.0 for e, n in waypoints_cm]
        wy = [n / 100.0 for e, n in waypoints_cm]
        plt.figure(figsize=(6, 6))
        plt.plot(xs, ys, "-b", label="path")
        plt.plot(wx, wy, "ro--", label="waypoints")
        plt.gca().set_aspect("equal", "box")
        plt.xlabel("east (m)")
        plt.ylabel("north (m)")
        plt.title("SIL trike path — {0}  (J={1:.1f})".format(args.course, j))
        plt.legend()
        plt.grid(True)
        plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
