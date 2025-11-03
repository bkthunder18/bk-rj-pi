# angle_test.py
# Works with Stepper(shifter, lock, outputs) where:
#   outputs = multiprocessing.Value('i', 0)  # shared 8-bit shift register
# And where Stepper.angle is multiprocessing.Value('d', ...)

import time
import csv
import multiprocessing as mp
from shifter import Shifter
from stepper_class_shiftregister_multiprocessing import Stepper

# ---------------- Hardware pins for 74HC595 ----------------
DATA_PIN, LATCH_PIN, CLOCK_PIN = 16, 20, 21

# ---------------- Helpers ----------------
def wait_degrees(deg, safety=1.35):
    """Sleep long enough for 'deg' at the class per-step delay (+ margin)."""
    steps = int(abs(deg) * Stepper.steps_per_degree)   # class var
    time_needed = steps * (Stepper.delay / 1e6)        # µs -> s
    time.sleep(max(0.01, time_needed * safety))

def norm180(a):
    a = (a + 180.0) % 360.0 - 180.0
    return a

def err_to_target(curr_angle_deg, target_deg):
    """Signed shortest error (deg) from current to target."""
    return norm180((curr_angle_deg % 360.0) - (target_deg % 360.0))

def near0(x, tol=2.0):
    return abs(x) <= tol

# --------------- Main test ---------------
if __name__ == "__main__":
    # Shift register driver
    s = Shifter(data=DATA_PIN, clock=CLOCK_PIN, latch=LATCH_PIN)

    # One lock (the class might use it to serialize stepping)
    lock = mp.Lock()

    # One shared 8-bit register for ALL motors
    reg = mp.Value('i', 0)  # <- THIS is what your Stepper wants as "outputs"

    # Instantiate motors (NO bit-map list here!)
    m1 = Stepper(s, lock, reg)
    m2 = Stepper(s, lock, reg)

    # Zero logical angles
    m1.zero()
    m2.zero()

    print("Starting alternating 90° accuracy test…")

    # How many cycles (each cycle does m1: 0→90→0, then m2: 0→90→0)
    CYCLES = 2
    TOL = 2.0

    rows = []
    header = ["pattern", "cycle", "motor", "step_idx", "target_deg",
              "final_angle_deg", "error_deg", "pass"]

    # Pattern: alternating 90°
    for c in range(1, CYCLES + 1):
        # m1 to +90
        tgt = 90
        m1.goAngle(tgt)            # non-blocking per your class
        wait_degrees(90)
        a = m1.angle.value         # <-- read .value
        e = err_to_target(a, tgt)
        rows.append(["alternating90", c, "m1", 0, tgt, a, e, near0(e, TOL)])

        # m1 back to 0
        tgt = 0
        m1.goAngle(tgt)
        wait_degrees(90)
        a = m1.angle.value
        e = err_to_target(a, tgt)
        rows.append(["alternating90", c, "m1", 1, tgt, a, e, near0(e, TOL)])

        # m2 to +90
        tgt = 90
        m2.goAngle(tgt)
        wait_degrees(90)
        a = m2.angle.value
        e = err_to_target(a, tgt)
        rows.append(["alternating90", c, "m2", 0, tgt, a, e, near0(e, TOL)])

        # m2 back to 0
        tgt = 0
        m2.goAngle(tgt)
        wait_degrees(90)
        a = m2.angle.value
        e = err_to_target(a, tgt)
        rows.append(["alternating90", c, "m2", 1, tgt, a, e, near0(e, TOL)])

    # Print summary
    e1 = err_to_target(m1.angle.value, 0)
    e2 = err_to_target(m2.angle.value, 0)
    ok1, ok2 = near0(e1, TOL), near0(e2, TOL)
    status = "✅ PASS" if (ok1 and ok2) else "⚠️ CHECK"
    print(f"{status}  Final: m1={m1.angle.value:.2f}° (err {e1:+.2f}), "
          f"m2={m2.angle.value:.2f}° (err {e2:+.2f})")

    # Optional: write CSV log
    with open("angle_test_log.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    print("Wrote angle_test_log.csv")
