"""
Port of the Nav control law (tools/jetson_navigate.py / Navigate.ino) with the
CAN I/O stripped out so it can run inside the pure-Python SIL loop.

Nav consumes the trike pose (east_cm, north_cm, heading_centiDeg) and emits a
drive command (speed_cmPs, brake, steer_DegX10) toward the current waypoint,
advancing through the mission as each waypoint is captured.

The tunable knobs (CRUISE_CMPS, KP_STEERING, WAYPOINT_RADIUS_CM, MAX_STEER_DEG)
are constructor arguments — these are exactly the parameters the auto-tuner
will search over.
"""

import math

EARTH_RADIUS_CM = 637100000
TO_RADIANS = math.pi / 180.0

# Defaults — match Sensor Hub Settings.h / jetson_navigate.py
CRUISE_CMPS = 100
KP_STEERING = 5
WAYPOINT_RADIUS_CM = 300
MAX_STEER_DEG = 30
NAV_LOOP_HZ = 10


class Origin(object):
    def __init__(self, lat_deg, lon_deg):
        self.lat = lat_deg
        self.lon = lon_deg
        self.cos_lat = math.cos(lat_deg * TO_RADIANS)


def latlon_to_cm(origin, lat_deg, lon_deg):
    d_lat = lat_deg - origin.lat
    d_lon = lon_deg - origin.lon
    north_cm = int(d_lat * TO_RADIANS * EARTH_RADIUS_CM)
    east_cm = int(d_lon * TO_RADIANS * EARTH_RADIUS_CM * origin.cos_lat)
    return east_cm, north_cm


def distance_cm(e1, n1, e2, n2):
    de = e1 - e2
    dn = n1 - n2
    return int(math.sqrt(de * de + dn * dn))


def bearing_centiDeg(e1, n1, e2, n2):
    de = float(e2 - e1)
    dn = float(n2 - n1)
    rad = math.atan2(de, dn)
    deg = rad / TO_RADIANS
    if deg < 0:
        deg += 360.0
    return int(deg * 100.0)


def wrap_centiDeg_signed(x):
    while x > 18000:
        x -= 36000
    while x <= -18000:
        x += 36000
    return x


class Nav(object):
    """Stateful Nav node. Feed it pose, get back a drive command."""

    def __init__(self, waypoints_cm,
                 cruise_cmPs=CRUISE_CMPS,
                 kp_steering=KP_STEERING,
                 waypoint_radius_cm=WAYPOINT_RADIUS_CM,
                 max_steer_deg=MAX_STEER_DEG):
        self.waypoints_cm = list(waypoints_cm)
        self.cruise_cmPs = cruise_cmPs
        self.kp_steering = kp_steering
        self.waypoint_radius_cm = waypoint_radius_cm
        self.max_steer_deg = max_steer_deg

        self.waypoint_idx = 0
        self.mission_complete = False
        # Last-computed diagnostics (used by the scorecard).
        self.last_dist_cm = 0
        self.last_bearing_cD = 0
        self.last_err_cD = 0

    def _advance_waypoint(self, east_cm, north_cm):
        if self.mission_complete:
            return
        tgt_e, tgt_n = self.waypoints_cm[self.waypoint_idx]
        if distance_cm(east_cm, north_cm, tgt_e, tgt_n) >= self.waypoint_radius_cm:
            return
        self.waypoint_idx += 1
        if self.waypoint_idx >= len(self.waypoints_cm):
            self.mission_complete = True

    def step(self, east_cm, north_cm, heading_centiDeg):
        """Returns (speed_cmPs, brake, steer_DegX10)."""
        self._advance_waypoint(east_cm, north_cm)

        if self.mission_complete:
            self.last_dist_cm = 0
            self.last_err_cD = 0
            return 0, 100, 0

        tgt_e, tgt_n = self.waypoints_cm[self.waypoint_idx]
        dist_cm = distance_cm(east_cm, north_cm, tgt_e, tgt_n)
        bearing_cD = bearing_centiDeg(east_cm, north_cm, tgt_e, tgt_n)
        err_cD = wrap_centiDeg_signed(bearing_cD - heading_centiDeg)

        steer = (self.kp_steering * err_cD) // 100
        cap = self.max_steer_deg * 10
        if steer > cap:
            steer = cap
        if steer < -cap:
            steer = -cap

        self.last_dist_cm = dist_cm
        self.last_bearing_cD = bearing_cD
        self.last_err_cD = err_cD
        return self.cruise_cmPs, 0, steer
