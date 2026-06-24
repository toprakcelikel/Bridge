#!/usr/bin/env python3
"""
Phase 3 auto-tuner: use Optuna (Bayesian / TPE search) to find knob values
that minimize the SIL scorecard J, then show a before/after comparison.

Optuna is the "clever detective": instead of brute-forcing every combination,
it learns from each trial which regions of the knob space look promising and
spends its budget there.

Examples
--------
    python tune.py                              # tune kp_steering, 60 trials
    python tune.py --params kp_steering max_steer_deg --trials 100
    python tune.py --courses square dogleg      # tune across two courses
    python tune.py --plot                        # before/after path plot

Nothing here touches hardware — every trial is a fast pure-Python SIL run.
"""

import argparse
import sys

import optuna

from sim import run_sim, score_run, evaluate
from courses import COURSES


# ===========================================================================
# Search space — (low, high, kind). Add a knob here to make it tunable.
# Floats use a log scale where the natural range spans orders of magnitude.
# ===========================================================================
SEARCH_SPACE = {
    "kp_steering":        (1,      20,    "int"),
    "max_steer_deg":      (10,     35,    "int"),
    "waypoint_radius_cm": (150,    500,   "int"),
    "cruise_cmPs":        (60,     180,   "int"),
    "steer_deadband":     (1,      15,    "int"),
    "throttle_kp":        (1e-3,   1e-1,  "logfloat"),
    "throttle_ki":        (1e-2,   1.0,   "logfloat"),
    "throttle_kd":        (1e-6,   1e-3,  "logfloat"),
}

# Firmware defaults — the "before" baseline.
DEFAULT_KNOBS = {
    "kp_steering": 5,
    "max_steer_deg": 30,
    "waypoint_radius_cm": 300,
    "cruise_cmPs": 100,
    "steer_deadband": 5,
    "throttle_kp": 0.0175,
    "throttle_ki": 0.2,
    "throttle_kd": 0.00001,
}


def _suggest(trial, params):
    knobs = {}
    for name in params:
        low, high, kind = SEARCH_SPACE[name]
        if kind == "int":
            knobs[name] = trial.suggest_int(name, low, high)
        elif kind == "logfloat":
            knobs[name] = trial.suggest_float(name, low, high, log=True)
        else:
            knobs[name] = trial.suggest_float(name, low, high)
    return knobs


def make_objective(params, courses, max_seconds):
    def objective(trial):
        knobs = _suggest(trial, params)
        return evaluate(knobs, courses=courses, max_seconds=max_seconds)
    return objective


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", nargs="+", default=["kp_steering"],
                    choices=sorted(SEARCH_SPACE),
                    help="knobs to tune (default: kp_steering)")
    ap.add_argument("--courses", nargs="+", default=["square"],
                    choices=sorted(COURSES))
    ap.add_argument("--trials", type=int, default=60)
    ap.add_argument("--max-seconds", type=float, default=120)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--plot", action="store_true",
                    help="before/after path plot on the first course")
    ap.add_argument("--save", metavar="PNG",
                    help="save the before/after plot to this file instead of "
                         "(or as well as) showing it")
    args = ap.parse_args()

    courses = tuple(args.courses)

    # ---- Baseline (firmware defaults) ----
    baseline_j = evaluate(DEFAULT_KNOBS, courses=courses,
                          max_seconds=args.max_seconds)

    print("Tuning      : {0}".format(", ".join(args.params)))
    print("Courses     : {0}".format(", ".join(courses)))
    print("Trials      : {0}".format(args.trials))
    print("Baseline J  : {0:.2f}  (firmware defaults: {1})".format(
        baseline_j, {k: DEFAULT_KNOBS[k] for k in args.params}))
    print("-" * 60)

    # ---- Optuna study ----
    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = optuna.create_study(
        direction="minimize",
        sampler=optuna.samplers.TPESampler(seed=args.seed))
    study.optimize(make_objective(args.params, courses, args.max_seconds),
                   n_trials=args.trials, show_progress_bar=False)

    best = study.best_params
    best_j = study.best_value

    # Full tuned knob set = defaults overridden by the tuned params.
    tuned_knobs = dict(DEFAULT_KNOBS)
    tuned_knobs.update(best)

    improvement = (baseline_j - best_j) / baseline_j * 100.0 if baseline_j else 0.0
    print("Best J      : {0:.2f}".format(best_j))
    print("Best params : {0}".format(
        {k: best[k] for k in args.params}))
    print("Improvement : {0:.1f}%  lower cost vs baseline".format(improvement))

    if args.plot or args.save:
        _plot_before_after(courses[0], DEFAULT_KNOBS, tuned_knobs,
                           baseline_j, best_j, args.max_seconds, args.params,
                           save=args.save, show=args.plot)

    return 0


def _plot_before_after(course, baseline_knobs, tuned_knobs,
                       baseline_j, best_j, max_seconds, params,
                       save=None, show=True):
    try:
        import matplotlib
        if save and not show:
            matplotlib.use("Agg")   # headless-safe when only saving
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping plot.")
        return

    b_samples, _, wp = run_sim(course, knobs=baseline_knobs,
                               max_seconds=max_seconds)
    t_samples, _, _ = run_sim(course, knobs=tuned_knobs,
                              max_seconds=max_seconds)

    def path(samples):
        return ([s["east_cm"] / 100.0 for s in samples],
                [s["north_cm"] / 100.0 for s in samples])

    bx, by = path(b_samples)
    tx, ty = path(t_samples)
    wx = [e / 100.0 for e, n in wp]
    wy = [n / 100.0 for e, n in wp]

    plt.figure(figsize=(7, 7))
    plt.plot(bx, by, "-", color="tab:red", alpha=0.8,
             label="before (J={0:.0f})".format(baseline_j))
    plt.plot(tx, ty, "-", color="tab:green", alpha=0.9,
             label="after (J={0:.0f})".format(best_j))
    plt.plot(wx, wy, "ko--", label="waypoints")
    plt.gca().set_aspect("equal", "box")
    plt.xlabel("east (m)")
    plt.ylabel("north (m)")
    plt.title("Auto-tune {0} on '{1}'".format(", ".join(params), course))
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    if save:
        plt.savefig(save, dpi=120)
        print("Plot        : saved to {0}".format(save))
    if show:
        plt.show()


if __name__ == "__main__":
    sys.exit(main())
