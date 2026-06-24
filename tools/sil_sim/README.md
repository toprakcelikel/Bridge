# Elcano SIL Simulator (Phase 2)

A pure-Python **software-in-the-loop** model of the trike's closed control
loop. It needs **no hardware** — it runs the same Nav → DBW → Router signal
chain that lives on the three Arduino Dues, but as one fast Python loop. This
is the engine the auto-tuner (Phases 3+) calls thousands of times.

## Files

| File | Mirrors firmware | Purpose |
|------|------------------|---------|
| `pid.py` | `Drive_By_Wire/PID.cpp` | Brett Beauregard PID (P_ON_E, pre-scaled gains) |
| `physics.py` | `simulator_physics.h` + `simulator_closed_loop.ino` | Integer Router physics (speed/angle/position) |
| `dbw.py` | `SpeedController.cpp`, `SteeringController.cpp` | Throttle PID + two-wire bang-bang steering |
| `nav.py` | `jetson_navigate.py` / `Navigate.ino` | Waypoint nav + steering control law (the knobs) |
| `courses.py` | — | Waypoint test courses (square, straight, dogleg, loop) |
| `sim.py` | the whole CAN bus | Closed-loop orchestrator, scorecard, `evaluate(knobs)` |
| `run.py` | — | CLI smoke test / CSV dump / plot |

## Run it

```sh
cd tools/sil_sim
python run.py --course square            # default knobs
python run.py --course dogleg --kp-steering 8
python run.py --course square --csv out.csv
python run.py --course square --plot     # needs matplotlib
```

Prints a summary plus the scorecard **J** (lower is better).

## Knobs (the tuner's search space)

`cruise_cmPs`, `kp_steering`, `waypoint_radius_cm`, `max_steer_deg`,
`steer_deadband`, `throttle_kp`, `throttle_ki`, `throttle_kd`.

## Scorecard

`sim.score_run()` computes
`J = w·(mean cross-track) + w·(mean heading err) + w·(finish time) +
w·(mean steering jerk) + w·(off-course violations) + DNF penalty`.
Weights live in `sim.DEFAULT_WEIGHTS`.

## Auto-tuner entry point

```python
from sim import evaluate
J = evaluate({"kp_steering": 8}, courses=("square", "dogleg"))
```

`evaluate()` averages J across the listed courses to avoid overfitting one
track. Phase 3 wraps this with Optuna; Phase 4 with CMA-ES.

## Fidelity notes / simplifications

- Integer arithmetic is preserved in the physics and control ports to match
  the embedded behavior.
- The DBW reads its measured speed from the Router's actual speed directly
  (wheel-tick quantization is not modeled).
- The DAC0→A0 throttle round-trip is modeled as identity clamped to 0–255;
  absolute speed calibration can be refined later via system-ID (Phase 6).
- One-tick (100 ms) feedback delay between Router pose and Nav/DBW, matching
  the asynchronous CAN loops on the real bus.
