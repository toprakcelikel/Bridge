#!/usr/bin/env python3
"""
Elcano test runner — PC-driven CSV test for the closed-loop simulator.

Reads a test CSV (Stage 2 command rows + `# assert` lines), pushes
commands to Sensor_Hub over USB serial at the scheduled times, captures
the LOG telemetry Sensor_Hub prints back, evaluates the assertions, and
prints a pass/fail summary.

Usage:
    python test_runner.py <test.csv> <serial_port>

Examples:
    python test_runner.py ../tests/test_steering_response.csv COM22
    python test_runner.py ../tests/test_steering_response.csv /dev/ttyACM0

CSV format (extends the existing Stage 2 CSVs — backwards compatible):
    Command row:    time_ms, CANID, nbytes, speed_cmPs, brake, mode, angle_tenths
    Assertion line: # assert t=<ms>: <python expression using LOG field names>
    Scored line:    time_ms, field_name, A, B, C, D   (trapezoidal, A<=B<=C<=D)
    Comment:        # any other line starting with #

Trapezoidal scoring (per scored line): the LOG field is sampled at time_ms and
scored in [0, 100] — full marks on the [B, C] plateau, linear ramps up on
[A, B] and down on [C, D], zero outside [A, D]. A scored test passes when its
score >= 60. The overall figure of merit is the average score across all
scored lines. Example:
    2800, actual_angle_tenths, 130, 240, 250, 260

Sensor_Hub serial protocol (matching the NavigateTestRunner sketch in
NavigateTestRunner/NavigateTestRunner.ino):
    Inbound  (PC -> SH):  CMD,<id>,<a>,<b>,<c>,<d>\\n
    Outbound (SH -> PC):  LOG,<t_ms>,<key1>=<v1>,<key2>=<v2>,...\\n
                          plus ACK,<t_ms>\\n per accepted CMD

Dependencies:
    pip install pyserial
"""

from __future__ import annotations

import re
import sys
import time
import threading
from dataclasses import dataclass

try:
    import serial
except ImportError:
    sys.stderr.write("Missing pyserial — install with: pip install pyserial\n")
    sys.exit(2)


# -- Patterns ----------------------------------------------------------------

CMD_PATTERN = re.compile(
    r"^\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,"
    r"\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*$"
)
ASSERT_PATTERN = re.compile(r"^\s*#\s*assert\s+t=(\d+)\s*:\s*(.+?)\s*$")
LOG_PATTERN = re.compile(r"^LOG,(\d+),(.+)$")
# Trapezoidal scored test line: time_ms, field_name, A, B, C, D  (A<=B<=C<=D).
# The non-numeric second field distinguishes it from a numeric command row.
SCORE_PATTERN = re.compile(
    r"^\s*(\d+)\s*,\s*([A-Za-z_]\w*)\s*,"
    r"\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*$"
)


def parse_log_fields(line: str) -> dict[str, int] | None:
    """Return the key=val fields of a LOG line, or None if not a LOG line."""
    m = LOG_PATTERN.match(line)
    if not m:
        return None
    fields: dict[str, int] = {}
    for kv in m.group(2).split(","):
        if "=" in kv:
            k, v = kv.split("=", 1)
            try:
                fields[k.strip()] = int(v.strip())
            except ValueError:
                pass
    return fields


def settle_center(ser: "serial.Serial", timeout_s: float = 6.0,
                  verbose: bool = False) -> bool:
    """Bring the rig to a known state before a timed test starts.

    The Router + DBW run continuously between tests, so without this a test
    inherits the previous test's wheel angle (the vehicle drives in circles at
    full lock). Hold center + full brake until the sim reports the wheel
    centered and stopped AND the DBW's own reported angle (dbw_angle_tenths,
    via CAN 0x400) agrees with it. The DBW agreement is a liveness proof: a
    stalled DBW freezes dbw_angle_tenths at a stale value, so it won't line up
    with a freshly centered wheel. Returns True once a good state holds for a
    few consecutive samples, False on timeout.
    """
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    t0 = time.monotonic()
    last_send = 0.0
    good_streak = 0
    while time.monotonic() - t0 < timeout_s:
        now = time.monotonic()
        if now - last_send >= 0.1:
            ser.write(b"CMD,350,0,2,1,0\n")  # speed 0, brake 2 (full), center
            ser.flush()
            last_send = now
        raw = ser.readline()
        if not raw:
            continue
        fields = parse_log_fields(raw.decode("ascii", errors="ignore").rstrip("\r\n"))
        if not fields:
            continue
        angle = fields.get("actual_angle_tenths")
        dbw = fields.get("dbw_angle_tenths")
        speed = fields.get("sim_speed_cmPs")
        if angle is None or dbw is None or speed is None:
            continue
        good = abs(angle) <= 20 and abs(dbw) <= 20 and speed <= 20
        good_streak = good_streak + 1 if good else 0
        if verbose:
            print(f"  [settle] angle={angle} dbw={dbw} speed={speed} "
                  f"streak={good_streak}")
        if good_streak >= 3:
            return True
    return False


# -- Data --------------------------------------------------------------------

@dataclass
class Command:
    time_ms: int
    can_id: int
    speed: int
    brake: int
    mode: int
    angle: int


@dataclass
class Assertion:
    time_ms: int
    expression: str


@dataclass
class ScoredAssertion:
    time_ms: int
    field: str
    a: int
    b: int
    c: int
    d: int


# -- CSV parsing -------------------------------------------------------------

def parse_csv(path: str) -> tuple[list[Command], list[Assertion], list[ScoredAssertion]]:
    commands: list[Command] = []
    asserts: list[Assertion] = []
    scored: list[ScoredAssertion] = []
    with open(path) as f:
        for raw in f:
            line = raw.rstrip("\r\n")
            stripped = line.strip()
            if not stripped:
                continue
            m = ASSERT_PATTERN.match(line)
            if m:
                asserts.append(Assertion(int(m.group(1)), m.group(2)))
                continue
            if stripped.startswith("#"):
                continue
            m = SCORE_PATTERN.match(line)
            if m:
                a, b, c, d = (int(m.group(3)), int(m.group(4)),
                              int(m.group(5)), int(m.group(6)))
                if not (a <= b <= c <= d):
                    sys.stderr.write(
                        f"  [warn] scored line not A<=B<=C<=D: {line!r}\n")
                scored.append(
                    ScoredAssertion(
                        time_ms=int(m.group(1)),
                        field=m.group(2),
                        a=a, b=b, c=c, d=d,
                    )
                )
                continue
            m = CMD_PATTERN.match(line)
            if m:
                commands.append(
                    Command(
                        time_ms=int(m.group(1)),
                        can_id=int(m.group(2)),
                        speed=int(m.group(4)),
                        brake=int(m.group(5)),
                        mode=int(m.group(6)),
                        angle=int(m.group(7)),
                    )
                )
            else:
                sys.stderr.write(f"  [warn] unparseable line: {line!r}\n")
    commands.sort(key=lambda c: c.time_ms)
    asserts.sort(key=lambda a: a.time_ms)
    scored.sort(key=lambda s: s.time_ms)
    return commands, asserts, scored


# -- Serial reader thread ----------------------------------------------------

class LogReader(threading.Thread):
    """Background thread: drain serial, capture LOG lines, echo others."""

    def __init__(self, ser: "serial.Serial", t0: float, verbose: bool = False):
        super().__init__(daemon=True)
        self.ser = ser
        self.t0 = t0
        self.verbose = verbose
        self.log: list[tuple[int, dict[str, int]]] = []
        self._stop_flag = threading.Event()

    def stop(self) -> None:
        self._stop_flag.set()

    def run(self) -> None:
        while not self._stop_flag.is_set():
            try:
                raw = self.ser.readline()
            except Exception:
                continue
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore").rstrip("\r\n")
            if not line:
                continue
            elapsed_ms = int((time.monotonic() - self.t0) * 1000)
            m = LOG_PATTERN.match(line)
            if m:
                # Index LOG entries by PC-side elapsed time, not the device's
                # millis() — the device has been running long before the test
                # started, so its millis values won't match our assertion times.
                try:
                    fields: dict[str, int] = {}
                    for kv in m.group(2).split(","):
                        if "=" in kv:
                            k, v = kv.split("=", 1)
                            fields[k.strip()] = int(v.strip())
                    self.log.append((elapsed_ms, fields))
                    if self.verbose:
                        print(f"  [{elapsed_ms:6d} ms] {line}")
                except Exception:
                    pass
            else:
                # Echo human-readable lines as they happen
                print(f"  [{elapsed_ms:6d} ms] {line}")


# -- Assertion evaluation ----------------------------------------------------

def evaluate(assertion: Assertion, log: list[tuple[int, dict[str, int]]]) -> tuple[bool, str]:
    if not log:
        return False, "no LOG lines captured"
    nearest = min(log, key=lambda e: abs(e[0] - assertion.time_ms))
    drift_ms = nearest[0] - assertion.time_ms
    if abs(drift_ms) > 500:
        return False, f"no LOG within 500 ms of t={assertion.time_ms} (nearest: t={nearest[0]})"
    fields = nearest[1]
    # Substitute field names with values in the expression.
    expr = assertion.expression
    for k in sorted(fields.keys(), key=len, reverse=True):
        expr = re.sub(rf"\b{re.escape(k)}\b", str(fields[k]), expr)
    try:
        result = eval(expr, {"__builtins__": {}}, {})  # noqa: S307 — restricted builtins
    except Exception as e:
        return False, f"eval error: {e!r}  (expanded expr: {expr!r})"
    return bool(result), f"@ t={nearest[0]} (Δ {drift_ms:+d} ms) → {expr}"


def trapezoid_score(result: float, a: int, b: int, c: int, d: int) -> float:
    """Trapezoidal score in [0, 100] for a value against ramp points A<=B<=C<=D.

    Full marks on the [B, C] plateau, linear ramp up on [A, B] and down on
    [C, D], and zero outside [A, D].
    """
    if result < a or result > d:
        return 0.0
    if b <= result <= c:
        return 100.0
    if a != b and a <= result < b:
        return 100.0 * (result - a) / (b - a)
    if c != d and c < result <= d:
        return 100.0 * (d - result) / (d - c)
    return 0.0


def evaluate_scored(sa: "ScoredAssertion",
                    log: list[tuple[int, dict[str, int]]]) -> tuple[float | None, str]:
    """Sample sa.field near sa.time_ms and return (score, detail).

    score is None when the value could not be measured (no LOG in range or the
    field is absent) — callers treat that as 0 for the figure of merit.
    """
    if not log:
        return None, "no LOG lines captured"
    nearest = min(log, key=lambda e: abs(e[0] - sa.time_ms))
    drift_ms = nearest[0] - sa.time_ms
    if abs(drift_ms) > 500:
        return None, f"no LOG within 500 ms of t={sa.time_ms} (nearest: t={nearest[0]})"
    fields = nearest[1]
    if sa.field not in fields:
        return None, (f"field {sa.field!r} not in LOG "
                      f"(have: {', '.join(sorted(fields))})")
    val = fields[sa.field]
    score = trapezoid_score(val, sa.a, sa.b, sa.c, sa.d)
    return score, f"@ t={nearest[0]} (Δ {drift_ms:+d} ms) → {sa.field}={val}"


# -- Main --------------------------------------------------------------------

def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    verbose = any(a in ("--verbose", "-v") for a in sys.argv[1:])
    if len(args) != 2:
        sys.stderr.write(f"Usage: {sys.argv[0]} [--verbose] <test.csv> <serial_port>\n")
        return 2
    csv_path, port = args[0], args[1]

    commands, asserts, scored = parse_csv(csv_path)
    print(f"Loaded {len(commands)} commands, {len(asserts)} assertions, "
          f"{len(scored)} scored tests from {csv_path}")

    ser = serial.Serial(port, 115200, timeout=0.1)
    # Let the Due settle (Native USB CDC takes a moment to come up after open).
    time.sleep(0.5)

    # Deterministic start: re-center and stop, and confirm the DBW loop is
    # alive, before the test clock starts. Without this each run inherits the
    # previous run's wheel angle and results become a coin flip.
    print("Settling to a centered/stopped state before test...")
    if settle_center(ser, verbose=verbose):
        print("  Settled: wheel centered, DBW tracking.")
    else:
        print("  WARNING: could not reach a centered state in time. The DBW is"
              " likely stalled or dropping CAN frames (0x350/0x430 lost in the"
              " 0x701-0x70A Logger flood). Reset the DBW board. Continuing anyway.")

    t0 = time.monotonic()
    reader = LogReader(ser, t0, verbose=verbose)
    reader.start()

    # Schedule and send commands.
    for cmd in commands:
        target = t0 + cmd.time_ms / 1000.0
        delay = target - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        line = f"CMD,{cmd.can_id},{cmd.speed},{cmd.brake},{cmd.mode},{cmd.angle}\n"
        ser.write(line.encode("ascii"))
        ser.flush()
        elapsed_ms = int((time.monotonic() - t0) * 1000)
        print(f"  [{elapsed_ms:6d} ms] -> {line.strip()}")

    # Wait until the last assertion/scored time + slack so all LOG lines arrive.
    end_times = [a.time_ms for a in asserts] + [s.time_ms for s in scored]
    if end_times:
        slack_s = max(end_times) / 1000.0 + 1.0
        target = t0 + slack_s
        delay = target - time.monotonic()
        if delay > 0:
            time.sleep(delay)

    reader.stop()
    reader.join(timeout=1)
    ser.close()

    # Evaluate.
    print()
    print("=" * 60)
    passed = failed = 0

    if asserts:
        print(f"Assertions ({len(asserts)}):")
        for a in asserts:
            ok, detail = evaluate(a, reader.log)
            status = "PASS" if ok else "FAIL"
            print(f"  [{status}] t={a.time_ms:>5d} ms: {a.expression}")
            print(f"           {detail}")
            if ok:
                passed += 1
            else:
                failed += 1

    fom: float | None = None
    if scored:
        print(f"Scored tests ({len(scored)}), trapezoidal (A,B,C,D), pass \u2265 60:")
        total = 0.0
        for sa in scored:
            score, detail = evaluate_scored(sa, reader.log)
            s = score if score is not None else 0.0
            ok = score is not None and s >= 60.0
            status = "PASS" if ok else "FAIL"
            print(f"  [{status}] t={sa.time_ms:>5d} ms: {sa.field} "
                  f"({sa.a},{sa.b},{sa.c},{sa.d})  score={s:6.1f}")
            print(f"           {detail}")
            total += s
            if ok:
                passed += 1
            else:
                failed += 1
        fom = total / len(scored)

    print()
    print(f"Summary: {passed} passed, {failed} failed, "
          f"{len(reader.log)} LOG samples captured")
    if fom is not None:
        print(f"Figure of merit (avg score): {fom:.1f} / 100")

    # Diagnose a stalled DBW: if its reported angle never changed across the
    # whole run, the closed loop was broken (frozen/dropped CAN), so any
    # steering failures above are a DBW dropout, not a steering-logic fault.
    if failed:
        dbw_vals = [f["dbw_angle_tenths"] for _, f in reader.log
                    if "dbw_angle_tenths" in f]
        if len(dbw_vals) >= 5 and len(set(dbw_vals)) == 1:
            print(f"NOTE: dbw_angle_tenths was frozen at {dbw_vals[0]} for the entire"
                  " run \u2014 the DBW stalled or dropped CAN frames (not a steering-logic"
                  " failure). Reset the DBW board and re-run.")

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
