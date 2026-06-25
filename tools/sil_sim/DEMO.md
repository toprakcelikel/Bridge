# Auto‑Tuning the Elcano Trike — SIL Demo

**Toprak Celikel** · Elcano Autonomous Trike · Prof. Tyler Folsom
_Software‑in‑the‑loop (SIL) simulator + automatic gain tuning. No hardware required._

---

## TL;DR

We replaced **hand‑tuning the trike's control gains on real hardware** (slow, risky, only a
handful of trials) with a **pure‑Python copy of the control loop** that runs thousands of
simulated drives in seconds, plus an **optimizer** that searches the gains automatically.

On the real UW‑Bothell square mission (held out from training), the tuned gains track the
route **~21 % better** than the firmware defaults — and that number is *honest*, because we
caught and closed the loopholes the optimizer first tried to exploit.

---

## 1. What we are tuning, and where it lives

The real trike runs three Arduino Dues on one 500 kbps CAN bus. The SIL reproduces all three
as one fast Python loop:

```
Nav (Sensor Hub Due)        --0x350-->   DBW Due                -->   Router (physics)
"where to aim + how fast"                "how to drive that"          dead-reckoned pose
   kp_steering                              throttle PID (kp/ki/kd)      feeds back next tick
   max_steer_deg                            steer_deadband               (one-tick CAN delay)
   cruise_cmPs
   (waypoint_radius — fixed)
```

The eight knobs split cleanly across the **CAN boundary** between the two control boards:

### Navigation knobs — Sensor Hub Due (`jetson_navigate.py` / `Navigate.ino`)
These decide the *intent*: which way to point and how fast to ask for.

| Knob | Firmware name | Default | What it does |
|---|---|---|---|
| `kp_steering` | `KP_STEERING` | 5 | Steering aggressiveness. `steer = (kp_steering × heading_error) // 100`. Low = lazy, rounds corners; high = snappy, can overshoot. |
| `max_steer_deg` | `MAX_STEER_DEG` | 30 | Cap on commanded steer angle. **Physically limited to 25°** (the wheel clamps there), so we cap the search at 25. |
| `cruise_cmPs` | `CRUISE_CMPS` | 100 | Target cruise speed (cm/s). 100 = 1 m/s. |
| `waypoint_radius_cm` | `WAYPOINT_RADIUS_CM` | 300 | How close counts as "arrived." **A mission tolerance, not a control gain — removed from tuning** (see §4). |

### DBW knobs — Drive‑By‑Wire Due (`Drive_By_Wire/`)
These decide the *execution*: how to faithfully follow the command Nav sent.

| Knob | Firmware name (file) | Default | What it does |
|---|---|---|---|
| `throttle_kp` | `proportional_throttle` (`SettingsTemplate.h`) | 0.0175 | Throttle PID **P** — reacts to current speed error. |
| `throttle_ki` | `integral_throttle` (`SettingsTemplate.h`) | 0.2 | Throttle PID **I** — kills steady‑state offset (e.g. uphill). |
| `throttle_kd` | `derivative_throttle` (`SettingsTemplate.h`) | 0.00001 | Throttle PID **D** — damps overshoot. |
| `steer_deadband` | `DEADBAND_DegX10` (`SteeringController.cpp`) | 5 | "Close enough" zone for the bang‑bang steering; stops left/right chatter. |

> **Hardware note:** the nav knobs require a reflash of the **Sensor Hub**; the throttle/deadband
> knobs require a reflash of the **DBW board** — different boards, different upload targets.

---

## 2. The scorecard `J` (how we measure "good driving")

Every simulated drive is graded by a single scalar cost **`J` (lower = better)**. It is a
weighted sum of eight terms averaged over the drive:

$$
J = w_{ct}\,\overline{e_{ct}}
  + w_{h}\,\overline{e_{h}}
  + w_{t}\,T_{finish}
  + w_{j}\,\overline{\lvert \Delta steer \rvert}
  + w_{s}\,\overline{(v - v_{safe})^{+}}
  + w_{wp}\,\overline{(d^{\min}_{i} - tol)^{+}}
  + w_{v}\,N_{viol}
  + w_{dnf}\,\frac{missed}{N_{wp}}
$$

| Term | Symbol | Weight | Meaning |
|---|---|---|---|
| Cross‑track error | $\overline{e_{ct}}$ | 1.0 | Mean distance (cm) from the **whole intended path** — tight tracking. |
| Heading error | $\overline{e_{h}}$ | 0.5 | Mean pointing error toward the target (deg). |
| Finish time | $T_{finish}$ | 2.0 | Seconds to complete the mission — rewards making progress. |
| Steering jerk | $\overline{\lvert \Delta steer\rvert}$ | 0.05 | Mean change in steer command — rewards smoothness. |
| Excess speed | $\overline{(v - v_{safe})^{+}}$ | 0.20 | Mean speed **above** the safe cap (150 cm/s) — penalizes flooring it. |
| **Waypoint miss** | $\overline{(d^{\min}_{i} - tol)^{+}}$ | 2.0 | Per waypoint, how far the **closest pass** was *outside* the 2 m tolerance — enforces actually visiting each waypoint. |
| Violations | $N_{viol}$ | 50.0 | Count of samples more than 6 m off the path. |
| Did‑not‑finish | $missed/N_{wp}$ | 5000.0 | Flat penalty (pro‑rated by waypoints missed) if the mission never completes. |

The $(\cdot)^{+}$ means "positive part" — speed at or below the safe cap, or a waypoint pass
*inside* the tolerance, costs nothing. Source of truth: `DEFAULT_WEIGHTS` in `sim.py`.

> **This is the key collaboration point:** the *weights* encode what we value (tracking vs.
> speed vs. smoothness). Right now they are my best guess — **Folsom's priorities should set them.**

---

## 3. The ML technique — Bayesian optimization (Optuna)

We do **not** brute‑force every gain combination (8 knobs × wide ranges = astronomically many).
Instead we use **Optuna** with a **TPE (Tree‑structured Parzen Estimator) sampler** — a form of
**Bayesian optimization**:

1. Try a set of gains → run the SIL → get `J`.
2. Build a probabilistic model of *which regions of gain‑space produce low `J`*.
3. Spend the next trial where the model thinks the payoff is highest (balancing
   "exploit known‑good" vs. "explore the unknown").
4. Repeat for N trials (we use 300).

It is a "clever detective" that learns from each trial instead of guessing blindly.
A **CMA‑ES** sampler is also available (`--sampler cmaes`) for the continuous gains.

---

## 4. How we stop the optimizer from cheating

An optimizer minimizes the *number* you give it — it will happily find loopholes that lower `J`
without actually driving better. We found three and closed them:

| Loophole the optimizer found | Fix |
|---|---|
| **Cut corners.** Measuring error only against the current leg let it slice across turns. | Cross‑track is now measured against the **entire intended path** (`_dist_to_path_cm`) — corner‑cutting always shows up as distance from the route. |
| **Floor the throttle** to crush the finish‑time term. | Added an **excess‑speed penalty** above a 150 cm/s safe cap — fast is no longer free. |
| **Enlarge the "arrived" radius** to finish early and cut corners. | **Removed `waypoint_radius` from the tunables** — it's a mission tolerance, not a control gain. |
| Tuning a steer angle the hardware can't reach (>25°). | **Capped `max_steer_deg` at 25°**, the physical wheel limit. |
| **Penalizing physically‑necessary corner rounding.** A trike *cannot* trace a sharp 90° vertex (min turning radius ≈ 2.6 m), so charging cross‑track at corners punished the unavoidable. | Added a **waypoint pass‑through term**: the path between waypoints is free, but the trike must come within **2 m** of each waypoint. Forgiving about *how* it rounds, strict about *whether* it visits. |

**Result:** the headline improvement is an honest **~21 %** on the held‑out mission. Earlier,
before these fixes, the optimizer reported an inflated **30 %** by flooring the throttle and
cutting corners. The cruise speed now settles at a sensible **131 cm/s on its own**, the
steering deadband relaxed to a precise **9** (it had railed at the maximum under the old
scorecard), and the tuned path threads each corner waypoint instead of bowing past it.
_The framework is self‑skeptical._

> **Why the 2 m waypoint tolerance?** It sits above the **corner turning‑radius floor**: with
> wheelbase ≈ 1.2 m and max steer 25°, $R_{min} \approx 2.6$ m, so the closest a min‑radius arc
> can get to a 90° vertex is $R_{min}(\sqrt{2}-1) \approx 1.1$ m. A 2 m tolerance clears that with
> margin. On hardware it should be raised to match real **GPS accuracy** (~2–3 m) — both the
> turning radius and the GPS accuracy are things we pin down from one **real logged drive**.

---

## 5. Methodology — train / validation split (anti‑overfitting)

Tuning on one track risks **overfitting** — gains that ace that track but fail elsewhere. We
guard against it the same way ML models are validated: **train on some courses, validate on a
held‑out course the optimizer never saw.**

![SIL test courses](courses.png)

| Course | Role | Why |
|---|---|---|
| **straight** | train | Pure speed/throttle behavior, no turns. |
| **dogleg** | train | Two opposite 90° turns (a Z) — stresses steering both ways. |
| **loop** | train | Closed 30 m square returning to start — sustained turning. |
| **square** | **validation (held out)** | The **real UW‑Bothell mission** — the only one that matters operationally; never used during tuning. |

If the tuned gains improve the **held‑out square** too, the result *generalizes* — it learned to
drive, not to memorize a track.

---

## 6. Results

**Phase 3 — single knob (`kp_steering`):** swept 5 → 14, ~11 % better. Proves the optimizer finds
a real minimum, not noise.

![Kp before/after](kp_before_after.png)

**Phase 4 — full vector (7 knobs), honest scorecard:**

| | Train `J` | Validation `J` (held‑out square) |
|---|---|---|
| Firmware defaults | 209.6 | 214.5 |
| **Auto‑tuned** | **163.3 (+22.1 %)** | **169.4 (+21.0 %)** |

The validation improvement (on a course never seen in training) confirms the gains **generalize**.
Tuned knobs: `kp_steering=34, max_steer_deg=25, cruise_cmPs=131, steer_deadband=9,
throttle_kp=0.0095, throttle_ki=0.997, throttle_kd=2.2e-05`.

![Phase 4 before/after — honest scorecard](phase4b_before_after.png)

> On the long straight edges the tuned path (green) hugs the line tightly, and it now **threads
> each corner waypoint** within tolerance instead of bowing past it. A faint overshoot remains at
> one corner (the higher steering gain) — a real, visible trade‑off the **weights** let us dial in.

---

## 7. The two asks for Folsom

1. **What should we optimize for?** Tight tracking vs. smooth/comfortable vs. fast — your
   priorities set the scorecard weights (§2). Today they're my guess.
2. **One real logged drive.** Everything here is simulated. A single CAN log from the trike lets
   us measure the **sim‑to‑real gap** before any of these gains touch hardware (Phase 5/6).

---

## Appendix — run it yourself (no hardware)

```powershell
cd c:\projects\elcano-Bridge\tools\sil_sim

# Drive the real mission with default gains, plot the path
python run.py --course square --plot

# Full auto-tune: train on straight/dogleg/loop, validate on held-out square
python tune.py --params kp_steering max_steer_deg cruise_cmPs steer_deadband `
               throttle_kp throttle_ki throttle_kd `
               --train straight dogleg loop --validate square `
               --trials 300 --max-seconds 200 --save phase4b_before_after.png
```

| File | Role |
|---|---|
| `nav.py` | Nav control law (steering + waypoint logic) |
| `dbw.py` | DBW throttle PID + bang‑bang steering |
| `physics.py` | Router physics (port of `simulator_closed_loop.ino`) |
| `pid.py` | Port of the Arduino PID library |
| `courses.py` | Test courses (square / straight / dogleg / loop) |
| `sim.py` | Closed‑loop orchestrator + scorecard `J` |
| `tune.py` | Optuna auto‑tuner + before/after plot |
| `run.py` | CLI smoke test for a single run |
