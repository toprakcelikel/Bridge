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
                    choices=sorted(COURSES),
                    help="tune AND report on these (no held-out check). "
                         "Ignored if --train is given.")
    ap.add_argument("--train", nargs="+", choices=sorted(COURSES),
                    help="courses the tuner is allowed to optimize on")
    ap.add_argument("--validate", nargs="+", choices=sorted(COURSES),
                    help="held-out courses the tuner never sees; the winner "
                         "is only scored on them at the end")
    ap.add_argument("--sampler", choices=["tpe", "cmaes"], default="tpe",
                    help="search strategy: tpe (robust for mixed int/float, "
                         "default) or cmaes (continuous evolution strategy)")
    ap.add_argument("--trials", type=int, default=60)
    ap.add_argument("--max-seconds", type=float, default=120)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--plot", action="store_true",
                    help="before/after path plot on a held-out (or first) course")
    ap.add_argument("--save", metavar="PNG",
                    help="save the before/after plot to this file instead of "
                         "(or as well as) showing it")
    args = ap.parse_args()

    # Resolve the train / validation split.
    if args.train:
        train_courses = tuple(args.train)
        val_courses = tuple(args.validate) if args.validate else ()
    else:
        # Back-compat: tune and report on --courses, no held-out check.
        train_courses = tuple(args.courses)
        val_courses = tuple(args.validate) if args.validate else ()

    max_s = args.max_seconds

    # ---- Baselines (firmware defaults) ----
    base_train = evaluate(DEFAULT_KNOBS, courses=train_courses, max_seconds=max_s)
    base_val = (evaluate(DEFAULT_KNOBS, courses=val_courses, max_seconds=max_s)
                if val_courses else None)

    print("Tuning      : {0}".format(", ".join(args.params)))
    print("Sampler     : {0}".format(args.sampler))
    print("Train       : {0}".format(", ".join(train_courses)))
    print("Validate    : {0}".format(", ".join(val_courses) if val_courses
                                     else "(none — no held-out check)"))
    print("Trials      : {0}".format(args.trials))
    print("Baseline    : train J={0:.2f}{1}  (defaults: {2})".format(
        base_train,
        "  val J={0:.2f}".format(base_val) if base_val is not None else "",
        {k: DEFAULT_KNOBS[k] for k in args.params}))
    print("-" * 64)

    # ---- Optuna study ----
    optuna.logging.set_verbosity(optuna.logging.WARNING)
    if args.sampler == "cmaes":
        sampler = optuna.samplers.CmaEsSampler(seed=args.seed)
    else:
        sampler = optuna.samplers.TPESampler(seed=args.seed)
    study = optuna.create_study(direction="minimize", sampler=sampler)
    study.optimize(make_objective(args.params, train_courses, max_s),
                   n_trials=args.trials, show_progress_bar=False)

    best = study.best_params
    best_train = study.best_value

    # Full tuned knob set = defaults overridden by the tuned params.
    tuned_knobs = dict(DEFAULT_KNOBS)
    tuned_knobs.update(best)

    def pct(base, tuned):
        return (base - tuned) / base * 100.0 if base else 0.0

    print("Best params : {0}".format({k: best[k] for k in args.params}))
    print("Train  J    : {0:.2f} -> {1:.2f}   ({2:+.1f}% better)".format(
        base_train, best_train, pct(base_train, best_train)))

    best_val = None
    if val_courses:
        best_val = evaluate(tuned_knobs, courses=val_courses, max_seconds=max_s)
        verdict = "generalizes" if best_val < base_val else "OVERFIT — worse on held-out"
        print("Valid. J    : {0:.2f} -> {1:.2f}   ({2:+.1f}% better)  [{3}]".format(
            base_val, best_val, pct(base_val, best_val), verdict))

    if args.plot or args.save:
        # Prefer plotting a held-out course (the honest test) when available.
        plot_course = val_courses[0] if val_courses else train_courses[0]
        plot_base = base_val if val_courses else base_train
        plot_best = best_val if val_courses else best_train
        _plot_before_after(plot_course, DEFAULT_KNOBS, tuned_knobs,
                           plot_base, plot_best, max_s, args.params,
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
