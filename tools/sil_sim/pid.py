"""
Port of the Arduino PID library (Brett Beauregard v1.2.2) as used by the DBW
firmware (Drive_By_Wire/PID.cpp).

Only the pieces the DBW actually exercises are reproduced:
  * P_ON_E (proportional on error), DIRECT acting
  * fixed sample time (the DBW calls Compute() once per 100 ms loop, so in the
    SIL sim every call advances exactly one sample)
  * control clamping to [controlMin, controlMax]

The firmware pre-scales the gains in SetTunings():
    kp = Kp
    ki = Ki * SampleTimeInSec
    kd = Kd / SampleTimeInSec
We do the same here so the numbers match the embedded controller exactly.
"""


class PID:
    def __init__(self, kp, ki, kd, sample_time_ms=100,
                 out_min=-255.0, out_max=255.0):
        sample_time_s = sample_time_ms / 1000.0
        # Pre-scaled gains, exactly as Arduino PID::SetTunings does.
        self.kp = float(kp)
        self.ki = float(ki) * sample_time_s
        self.kd = float(kd) / sample_time_s
        self.out_min = float(out_min)
        self.out_max = float(out_max)

        self._control_sum = 0.0
        self._last_feedback = 0.0
        self._initialized = False

    def reset(self, feedback=0.0, control=0.0):
        """Bumpless (re)initialization — mirrors PID::Initialize()."""
        self._control_sum = min(max(control, self.out_min), self.out_max)
        self._last_feedback = feedback
        self._initialized = True

    def compute(self, feedback, setpoint):
        """One PID step. Returns the control output (float)."""
        if not self._initialized:
            self.reset(feedback, 0.0)

        error = setpoint - feedback
        d_feedback = feedback - self._last_feedback

        # Integral term (P_ON_E => no proportional-on-measurement subtraction).
        self._control_sum += self.ki * error
        self._control_sum = min(max(self._control_sum, self.out_min), self.out_max)

        control = self.kp * error
        control += self._control_sum - self.kd * d_feedback
        control = min(max(control, self.out_min), self.out_max)

        self._last_feedback = feedback
        return control
