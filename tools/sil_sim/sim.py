"""
Closed-loop software-in-the-loop (SIL) simulator for the Elcano trike.

Ties together the three nodes that run on separate Arduino Dues on the real
trike, but here as a single fast pure-Python loop at LOOP_TIME_MS ticks:

    Nav (nav.Nav)            -> drive command (speed, brake, steer)
        |  0x350
    DBW (dbw.SpeedController + dbw.SteeringController)
        |  throttle / brake / L_TURN / R_TURN pin levels
    Router (physics.RouterPhysics)
        |  pose (X_mm, Y_mm, heading, speed) + actual wheel angle (0x430)
        +--> fed back to Nav and DBW on the next tick (one-tick CAN delay)

The public entry point for the auto-tuner is `evaluate(knobs, ...) -> score`
(lower is better). `run_sim(...)` returns the full trajectory for plotting.
"""

import math

from nav import Nav, NAV_LOOP_HZ
from dbw import SpeedController, SteeringController
from physics import RouterPhysics, LOOP_TIME_MS
from courses import get_course


# ===========================================================================
# Scorecard weights — J = Σ w_i * term_i  (lower is better)
# ===========================================================================
DEFAULT_WEIGHTS = {
    "cross_track": 1.0,      # per cm of mean cross-track error
    "heading": 0.5,          # per degree of mean heading error
    "time": 2.0,             # per second to finish the mission
    "jerk": 0.05,            # per DegX10 of total steering change
    "violation": 50.0,       # per off-course sample
    "did_not_finish": 5000.0,  # flat penalty if the mission never completes
}

# A position this far off the corridor counts as a violation.
VIOLATION_CT_CM = 600


def _point_to_segment_cm(px, py, ax, ay, bx, by):
    """Perpendicular distance (cm) from P to segment AB, clamped to the segment."""
    abx = bx - ax
    aby = by - ay
    seg_len2 = abx * abx + aby * aby
    if seg_len2 == 0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * abx + (py - ay) * aby) / seg_len2
    t = max(0.0, min(1.0, t))
    cx = ax + t * abx
    cy = ay + t * aby
    return math.hypot(px - cx, py - cy)


def run_sim(course="square", knobs=None, max_seconds=120, start_heading_tenths=0):
    """Run one closed-loop mission.

    knobs is an optional dict overriding any of:
        cruise_cmPs, kp_steering, waypoint_radius_cm, max_steer_deg,
        steer_deadband, throttle_kp, throttle_ki, throttle_kd

    Returns (samples, result) where samples is a list of per-tick dicts and
    result summarizes the run.
    """
    knobs = dict(knobs or {})
    waypoints_cm = get_course(course) if isinstance(course, str) else list(course)

    nav = Nav(
        waypoints_cm,
        cruise_cmPs=knobs.get("cruise_cmPs", 100),
        kp_steering=knobs.get("kp_steering", 5),
        waypoint_radius_cm=knobs.get("waypoint_radius_cm", 300),
        max_steer_deg=knobs.get("max_steer_deg", 30),
    )
    speed_ctrl = SpeedController(
        kp=knobs.get("throttle_kp", 0.0175),
        ki=knobs.get("throttle_ki", 0.2),
        kd=knobs.get("throttle_kd", 0.00001),
    )
    steer_ctrl = SteeringController(deadband=knobs.get("steer_deadband", 5))
    router = RouterPhysics()
    router.heading_tenths = start_heading_tenths

    dt_ms = LOOP_TIME_MS
    max_ticks = int(max_seconds * 1000 / dt_ms)

    samples = []
    prev_steer_cmd = 0

    for tick in range(max_ticks):
        t_s = tick * dt_ms / 1000.0

        # ---- pose from the Router (previous tick) -> CAN-frame units ----
        east_cm = router.X_mm // 10
        north_cm = router.Y_mm // 10
        heading_centiDeg = router.heading_tenths * 10
        measured_speed_cmPs = router.speed_mmPs // 10
        measured_angle_DegX10 = router.angle_tenths

        # ---- Nav ----
        speed_cmd, brake_cmd, steer_cmd = nav.step(
            east_cm, north_cm, heading_centiDeg)

        # ---- DBW ----
        throttle, brake_on = speed_ctrl.update(speed_cmd, measured_speed_cmPs)
        l_turn, r_turn = steer_ctrl.update(steer_cmd, measured_angle_DegX10)

        # ---- Router physics ----
        router.step(throttle, brake_on, l_turn, r_turn)

        samples.append({
            "t_s": t_s,
            "east_cm": east_cm,
            "north_cm": north_cm,
            "heading_centiDeg": heading_centiDeg,
            "speed_cmPs": measured_speed_cmPs,
            "angle_tenths": measured_angle_DegX10,
            "steer_cmd": steer_cmd,
            "throttle": throttle,
            "brake_on": brake_on,
            "waypoint_idx": nav.waypoint_idx,
            "dist_cm": nav.last_dist_cm,
            "err_cD": nav.last_err_cD,
        })
        prev_steer_cmd = steer_cmd

        if nav.mission_complete:
            break

    result = {
        "finished": nav.mission_complete,
        "finish_time_s": samples[-1]["t_s"] if nav.mission_complete else None,
        "ticks": len(samples),
        "waypoints_reached": nav.waypoint_idx,
        "n_waypoints": len(waypoints_cm),
    }
    return samples, result, waypoints_cm


def score_run(samples, result, waypoints_cm, weights=None):
    """Compute the scalar cost J for a run (lower is better)."""
    w = dict(DEFAULT_WEIGHTS)
    if weights:
        w.update(weights)

    n = len(samples)
    if n == 0:
        return w["did_not_finish"]

    sum_ct = 0.0
    sum_head = 0.0
    sum_jerk = 0.0
    violations = 0
    prev_steer = samples[0]["steer_cmd"]

    for s in samples:
        idx = s["waypoint_idx"]
        # Corridor segment: previous waypoint -> current target.
        bx, by = waypoints_cm[min(idx, len(waypoints_cm) - 1)]
        ax, ay = waypoints_cm[idx - 1] if idx > 0 else waypoints_cm[0]
        ct = _point_to_segment_cm(s["east_cm"], s["north_cm"], ax, ay, bx, by)
        sum_ct += ct
        if ct > VIOLATION_CT_CM:
            violations += 1

        sum_head += abs(s["err_cD"]) / 100.0
        sum_jerk += abs(s["steer_cmd"] - prev_steer)
        prev_steer = s["steer_cmd"]

    mean_ct = sum_ct / n
    mean_head = sum_head / n

    finish_time = result["finish_time_s"]
    if finish_time is None:
        # Penalize by elapsed time + flat DNF penalty, scaled by how many
        # waypoints were missed (partial credit for partial progress).
        finish_time = samples[-1]["t_s"]
        missed = result["n_waypoints"] - result["waypoints_reached"]
        dnf = w["did_not_finish"] * (missed / max(1, result["n_waypoints"]))
    else:
        dnf = 0.0

    j = (w["cross_track"] * mean_ct +
         w["heading"] * mean_head +
         w["time"] * finish_time +
         w["jerk"] * (sum_jerk / n) +
         w["violation"] * violations +
         dnf)
    return j


def evaluate(knobs=None, courses=("square",), max_seconds=120, weights=None):
    """Auto-tuner entry point: run knobs over one or more courses, return mean J.

    Averaging across multiple courses guards against overfitting to a single
    track geometry.
    """
    if isinstance(courses, str):
        courses = (courses,)
    total = 0.0
    for course in courses:
        samples, result, wp = run_sim(course, knobs=knobs, max_seconds=max_seconds)
        total += score_run(samples, result, wp, weights=weights)
    return total / len(courses)
