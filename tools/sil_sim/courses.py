"""
Test courses for the SIL simulator. Each course is a list of waypoints in the
local Euclidean cm frame (east_cm, north_cm), with waypoint 0 as the start.

The default 'square' course mirrors jetson_navigate.py's UW-Bothell loop,
projected to cm via the same flat-earth math. Extra synthetic courses are
provided so the auto-tuner can be evaluated across multiple geometries and
avoid overfitting to a single track.
"""

from nav import Origin, latlon_to_cm


# Same lat/lon loop jetson_navigate.py drives (~20 m square).
_SQUARE_LATLON = [
    (47.7600, -122.1917),   # WP0 — origin / start
    (47.7602, -122.1917),   # WP1 — ~22 m north
    (47.7602, -122.1914),   # WP2 — ~22 m east of WP1
    (47.7600, -122.1914),   # WP3 — ~22 m south
]


def _square_cm():
    origin = Origin(*_SQUARE_LATLON[0])
    return [latlon_to_cm(origin, lat, lon) for lat, lon in _SQUARE_LATLON]


# Synthetic courses defined directly in cm (east, north).
_STRAIGHT = [
    (0, 0),
    (0, 3000),       # 30 m straight north
]

_DOGLEG = [
    (0, 0),
    (0, 2000),       # 20 m north
    (1500, 2000),    # 15 m east
    (1500, 4000),    # 20 m north again
]

_LONG_LOOP = [
    (0, 0),
    (0, 3000),
    (3000, 3000),
    (3000, 0),
    (0, 0),          # back to start
]


COURSES = {
    "square": _square_cm(),
    "straight": _STRAIGHT,
    "dogleg": _DOGLEG,
    "loop": _LONG_LOOP,
}


def get_course(name):
    if name not in COURSES:
        raise KeyError("unknown course {0!r}; choose from {1}".format(
            name, sorted(COURSES)))
    return list(COURSES[name])
