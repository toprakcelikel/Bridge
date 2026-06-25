#!/usr/bin/env python3
"""Render each test course as a waypoint diagram (for the demo doc)."""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from courses import COURSES

ORDER = ["square", "straight", "dogleg", "loop"]
TITLES = {
    "square":   "square  (real UW-Bothell mission, held-out validation)",
    "straight": "straight  (training)",
    "dogleg":   "dogleg  (training)",
    "loop":     "loop  (training)",
}

fig, axes = plt.subplots(2, 2, figsize=(10, 10))
for ax, name in zip(axes.flat, ORDER):
    wp = COURSES[name]
    xs = [p[0] / 100.0 for p in wp]   # cm -> m
    ys = [p[1] / 100.0 for p in wp]
    ax.plot(xs, ys, "-o", color="#1f77b4", lw=2, ms=7, zorder=2)
    for i, (x, y) in enumerate(zip(xs, ys)):
        ax.annotate("WP{0}".format(i), (x, y),
                    textcoords="offset points", xytext=(8, 6), fontsize=9)
    ax.plot(xs[0], ys[0], "s", color="green", ms=12, zorder=3, label="start")
    ax.set_title(TITLES[name], fontsize=11)
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_aspect("equal", "box")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)

fig.suptitle("SIL test courses  (waypoints in metres)", fontsize=13)
fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig("courses.png", dpi=110)
print("saved courses.png")
