"""
Port of the Router-side integer physics engine
(Non_grapic_simulator/libraries/simulator_physics/src/simulator_physics.h)
plus the glue from simulator_closed_loop.ino that wires DBW pin levels into it.

All arithmetic is kept integer (// and int()) to match the embedded behavior
per the professor's integer-only requirement. Constants are taken verbatim
from simulator_closed_loop.ino.
"""

# ===========================================================================
# Constants (from simulator_closed_loop.ino)
# ===========================================================================
FRICTION_NUM = 9296
FRICTION_DEN = 10000
MIN_EFFECTIVE_THROTTLE = 65
MAX_EFFECTIVE_THROTTLE = 227
MAX_SPEED_mmPs = 2000
THROTTLE_HISTORY = 10
THROTTLE_DELAY_START = 3
THROTTLE_DELAY_END = 10
WHEEL_CIRCUM_MM = 1555
LOOP_TIME_MS = 100
MAX_ANGLE_TENTHS = 250
ANGLE_CHANGE_TENTHS = 20

# Brake decay: speed *= 5000/10000 each loop while braking (computeSpeed()).
BRAKE_DECAY_NUM = 5000
BRAKE_DECAY_DEN = 10000


# ===========================================================================
# Integer sine/cosine (returns value * 1000) — port of sin1000()/cos1000()
# ===========================================================================
_SIN_TABLE = [
    0, 17, 35, 52, 70, 87, 105, 122, 139, 156,
    174, 191, 208, 225, 242, 259, 276, 292, 309, 326,
    342, 358, 375, 391, 407, 423, 438, 454, 469, 485,
    500, 515, 530, 545, 559, 574, 588, 602, 616, 629,
    643, 656, 669, 682, 695, 707, 719, 731, 743, 755,
    766, 777, 788, 799, 809, 819, 829, 839, 848, 857,
    866, 875, 883, 891, 899, 906, 914, 921, 927, 934,
    940, 946, 951, 956, 961, 966, 970, 974, 978, 982,
    985, 988, 990, 993, 995, 996, 998, 999, 999, 1000,
    1000,
]


def sin1000(angle_tenths):
    angle_tenths = ((angle_tenths % 3600) + 3600) % 3600
    deg = angle_tenths // 10
    if angle_tenths < 900:
        return _SIN_TABLE[deg]
    elif angle_tenths < 1800:
        return _SIN_TABLE[180 - deg]
    elif angle_tenths < 2700:
        return -_SIN_TABLE[deg - 180]
    else:
        return -_SIN_TABLE[360 - deg]


def cos1000(angle_tenths):
    return sin1000(angle_tenths + 900)


class RouterPhysics:
    """Stateful integer physics model of the trike, mirroring the Router Due.

    Inputs each tick are DBW pin levels (throttle 0-255, brakeOn, lTurn,
    rTurn) exactly as the Router reads them off the bridge harness; outputs
    are the pose (X_mm, Y_mm, heading_tenths, speed_mmPs) and the actual
    wheel angle (angle_tenths) that the Router publishes back on CAN.
    """

    def __init__(self):
        self.throttle_history = [0] * THROTTLE_HISTORY
        self.history_index = 0
        self.speed_mmPs = 0
        self.prev_speed_mmPs = 0
        self.heading_tenths = 0
        self.angle_tenths = 0
        self.X_mm = 0
        self.Y_mm = 0

    # ---- computeSpeed(throttle, brakeOn) -------------------------------
    def compute_speed(self, throttle, brake_on):
        if brake_on:
            self.prev_speed_mmPs = self.prev_speed_mmPs * BRAKE_DECAY_NUM // BRAKE_DECAY_DEN
            return self.prev_speed_mmPs

        total = 0
        for i in range(THROTTLE_DELAY_START, THROTTLE_DELAY_END + 1):
            idx = (self.history_index - i + THROTTLE_HISTORY) % THROTTLE_HISTORY
            total += self.throttle_history[idx]
        mean_throttle = total // 8

        estimated_speed = 0
        if mean_throttle > MIN_EFFECTIVE_THROTTLE:
            estimated_speed = (MAX_SPEED_mmPs *
                               (mean_throttle - MIN_EFFECTIVE_THROTTLE) //
                               (MAX_EFFECTIVE_THROTTLE - MIN_EFFECTIVE_THROTTLE))
        if estimated_speed < 0:
            estimated_speed = 0

        momentum = self.prev_speed_mmPs * FRICTION_NUM // FRICTION_DEN
        new_speed = momentum if momentum > estimated_speed else estimated_speed
        if new_speed > MAX_SPEED_mmPs:
            new_speed = MAX_SPEED_mmPs
        self.prev_speed_mmPs = new_speed
        return new_speed

    # ---- updateAngle(lTurn, rTurn) -------------------------------------
    def update_angle(self, l_turn, r_turn):
        if l_turn and not r_turn:
            self.angle_tenths -= ANGLE_CHANGE_TENTHS
        elif not l_turn and r_turn:
            self.angle_tenths += ANGLE_CHANGE_TENTHS
        if self.angle_tenths > MAX_ANGLE_TENTHS:
            self.angle_tenths = MAX_ANGLE_TENTHS
        if self.angle_tenths < -MAX_ANGLE_TENTHS:
            self.angle_tenths = -MAX_ANGLE_TENTHS
        return self.angle_tenths

    # ---- updatePosition(speed) -----------------------------------------
    def update_position(self, speed):
        if speed > 0:
            self.heading_tenths += self.angle_tenths // 10
        if self.heading_tenths >= 3600:
            self.heading_tenths -= 3600
        if self.heading_tenths < 0:
            self.heading_tenths += 3600
        distance_mm = speed * LOOP_TIME_MS // 1000
        self.X_mm += distance_mm * sin1000(self.heading_tenths) // 1000
        self.Y_mm += distance_mm * cos1000(self.heading_tenths) // 1000

    # ---- one Router loop ------------------------------------------------
    def step(self, throttle, brake_on, l_turn, r_turn):
        """Advance physics one LOOP_TIME_MS tick from raw DBW pin levels."""
        self.update_angle(l_turn, r_turn)
        self.throttle_history[self.history_index] = throttle
        self.history_index = (self.history_index + 1) % THROTTLE_HISTORY
        self.speed_mmPs = self.compute_speed(throttle, brake_on)
        self.update_position(self.speed_mmPs)
        return self.pose()

    def pose(self):
        return {
            "X_mm": self.X_mm,
            "Y_mm": self.Y_mm,
            "heading_tenths": self.heading_tenths,
            "speed_mmPs": self.speed_mmPs,
            "angle_tenths": self.angle_tenths,
        }
