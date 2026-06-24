"""
Port of the DBW (Drive_By_Wire) control law: the throttle PID
(SpeedController) and the two-wire bang-bang steering (SteeringController).

The DBW receives Nav's command on CAN 0x350 (speed_cmPs, brake, angle_DegX10),
reads back the actual wheel angle on 0x430 and the actual speed from wheel
ticks, and drives:
  * DAC0      -> throttle level 0..255  (Router reads on A0)
  * BRAKE_ON  -> brake engaged          (Router reads on D48)
  * L_TURN / R_TURN digital wires       (Router reads on D4 / D2)

Constants are from Drive_By_Wire/Settings.h, SpeedController.h/.cpp,
SteeringController.cpp and DBW_Pins.h.
"""

from pid import PID


# ===========================================================================
# Throttle / PID constants (DBW_Pins.h, Settings.h)
# ===========================================================================
PROPORTIONAL_THROTTLE = 0.0175
INTEGRAL_THROTTLE = 0.2
DERIVATIVE_THROTTLE = 0.00001
PID_SAMPLE_TIME = 100

MIN_PID_TH = -255
PID_BRAKE = -100
PID_COAST = 10
MAX_PID_TH = 255
MIN_THROTTLE = 75
MAX_THROTTLE = 175

# Steering bang-bang
DEADBAND_DegX10 = 5


def _map(val, dlo, dhi, nlo, nhi):
    """MAP() macro from DBW_Pins.h (kept as float; firmware truncates to int)."""
    return nlo + ((val - dlo) * (nhi - nlo)) / (dhi - dlo)


class SpeedController:
    """Throttle PID + brake logic — port of SpeedController.

    Feedback is the measured speed in cm/s. On the real DBW this comes from
    the wheel-tick cyclometer; in the SIL sim we feed the Router's actual
    speed directly (wheel-tick quantization is ignored as a documented
    simplification).
    """

    def __init__(self, kp=PROPORTIONAL_THROTTLE, ki=INTEGRAL_THROTTLE,
                 kd=DERIVATIVE_THROTTLE):
        self.pid = PID(kp, ki, kd, sample_time_ms=PID_SAMPLE_TIME,
                       out_min=MIN_PID_TH, out_max=MAX_PID_TH)
        self.current_throttle = 0

    def update(self, desired_speed_cmPs, measured_speed_cmPs):
        """Returns (throttle 0..255, brake_on bool)."""
        if desired_speed_cmPs <= 0:
            # Stop(): brake on, throttle zero. Keep PID integral from winding.
            self.current_throttle = 0
            self.pid.reset(measured_speed_cmPs, 0.0)
            return 0, True

        pid_throttle = self.pid.compute(measured_speed_cmPs, desired_speed_cmPs)

        if pid_throttle < PID_BRAKE:
            self.current_throttle = 0
            return 0, True
        elif pid_throttle < PID_COAST:
            self.current_throttle = 0
            return 0, False
        else:
            throttle = int(_map(pid_throttle, MIN_THROTTLE, MAX_THROTTLE,
                                PID_COAST, MAX_PID_TH))
            # DAC0 (12-bit) -> Router A0 (rawThrottle/4) logical round-trip.
            # Modeled as identity clamped to the 0..255 byte the Router sees.
            if throttle < 0:
                throttle = 0
            if throttle > 255:
                throttle = 255
            self.current_throttle = throttle
            return throttle, False


class SteeringController:
    """Two-wire bang-bang steering with deadband — port of SteeringController.

    desired_angle_DegX10 comes from Nav (0x350); measured_angle_DegX10 is the
    Router's actual wheel angle (0x430). Returns the L_TURN / R_TURN levels.
    """

    def __init__(self, deadband=DEADBAND_DegX10):
        self.deadband = deadband

    def update(self, desired_angle_DegX10, measured_angle_DegX10):
        """Returns (l_turn bool, r_turn bool)."""
        err = desired_angle_DegX10 - measured_angle_DegX10
        l_turn = err < -self.deadband
        r_turn = err > self.deadband
        return l_turn, r_turn
