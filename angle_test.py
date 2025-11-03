# angle_test_selfcheck.py
import csv, time, multiprocessing
from datetime import datetime
from shifter import Shifter
from stepper_class_shiftregister_multiprocessing import Stepper

# ---------- Hardware (edit if needed) ----------
DATA_PIN, LATCH_PIN, CLOCK_PIN = 16, 20, 21
M1_BITS = [4, 5, 6, 7]   # QE..QH
M2_BITS = [0, 1, 2, 3]   # QA..QD

# ---------- Helpers ----------
def wait_degrees(deg, safety=1.35):
    """Sleep long enough for a motion of 'deg' based on class timing + margin."""
    steps = int(abs(deg) * Stepper.steps_per_degree)
    per_step_s = Stepper.delay / 1e6
    time.sleep(max(0.01, steps * per_step_s * safety))

def norm180(a):
    """Wrap degrees to [-180, 180)."""
    a = (a + 180.0) % 360.0 - 180.0
    return a

def err_to_target(curr_angle_deg, target_deg):
    """Signed shortest error (deg) from current to target."""
    return norm180((curr_angle_deg % 360.0) - (target_deg % 360.0))

def near(value, tol=2.0):
    return abs(value) <= tol

# ---------- Patterns ----------
def pattern_alternating_90(m1, m2, cycles, writer, tol):
    """
    m1: 0 -> +90 -> 0; then m2: 0 -> +90 -> 0; repeat
    """
    for c in range(1, cycles+1):
        # m1 +90
        target = 90
        m1.goAngle(target); wait_degrees(90)
        e = err_to_target(m1.angle, target)
        writer.writerow(["alternating90", c, "m1", 0, target, m1.angle, e, near(e, tol)])
        # m1 back to 0
        target = 0
        m1.goAngle(target); wait_degrees(90)
        e = err_to_target(m1.angle, target)
        writer.writerow(["alternating90", c, "m1", 1, target, m1.angle, e, near(e, tol)])
        # m2 +90
        target = 90
        m2.goAngle(target); wait_degrees(90)
        e = err_to_target(m2.angle, target)
        writer.writerow(["alternating90", c, "m2", 0, target, m2.angle, e, near(e, tol)])
        # m2 back to 0
        target = 0
        m2.goAngle(target); wait_degrees(90)
        e = err_to_target(m2.angle, target)
        writer.writerow(["alternating90", c, "m2", 1, target, m2.angle, e, near(e, tol)])

def pattern_figure_eight(m1, m2, cycles, writer, tol):
    """
    Do +/-90 sequences to check cumulative wrap & shortest path.
    m1: 0 -> +90 -> -90 -> +90 -> 0
    m2: 0 -> +90 -> -90 -> +90 -> 0
    """
    seq = [90, -90, 90, 0]
    for c in range(1, cycles+1):
        for i, tgt in enumerate(seq):
            m1.goAngle(tgt); wait_degrees(90)
            e = err_to_target(m1.angle, tgt)
            writer.writerow(["figure8", c, "m1", i, tgt, m1.angle, e, near(e, tol)])
        for i, tgt in enumerate(seq):
            m2.goAngle(tgt); wait_degrees(90)
            e = err_to_target(m2.angle, tgt)
            writer.writerow(["figure8", c, "m2", i, tgt, m2.angle, e, near(e, tol)])

def pattern_interleaved(m1, m2, cycles, writer, tol):
    """
    Interleave commands so controller must compose both motors' bits.
    m1: 0->90  while m2: 0->90, then both back to 0.
    """
    for c in range(1, cycles+1):
        # both to +90 (commands issued back-to-back)
        m1.goAngle(90)
        m2.goAngle(90)
        wait_degrees(90)  # same move size for both

        e1 = err_to_target(m1.angle, 90)
        e2 = err_to_target(m2.angle, 90)
        writer.writerow(["interleaved", c, "m1", 0, 90, m1.angle, e1, near(e1, tol)])
        writer.writerow(["interleaved", c, "m2", 0, 90, m2.angle, e2, near(e2, tol)])

        # both back to 0
        m1.goAngle(0)
        m2.goAngle(0)
        wait_degrees(90)

        e1 = err_to_target(m1.angle, 0)
        e2 = err_to_target(m2.angle, 0)
        writer.writerow(["interleaved", c, "m1", 1, 0, m1.angle, e1, near(e1, tol)])
        writer.writerow(["interleaved", c, "m2", 1, 0, m2.angle, e2, near(e2, tol)])

# ---------- Main ----------
if __name__ == "__main__":
    s = Shifter(data=DATA_PIN, latch=LATCH_PIN, clock=CLOCK_PIN)
    lock = multiprocessing.Lock()
    m1 = Stepper(s, lock, M1_BITS)
    m2 = Stepper(s, lock, M2_BITS)

    # Reset logical angles
    m1.zero(); m2.zero()

    # Config
    TOL = 2.0          # degrees for PASS/FAIL
    CYCLES_SHORT = 2   # number of loops per pattern

    log_name = "angle_test_log.csv"
    with open(log_name, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", datetime.now().isoformat()])
        writer.writerow(["Stepper.delay_us", Stepper.delay])
        writer.writerow(["steps_per_degree", Stepper.steps_per_degree])
        writer.writerow([])

        # header
        writer.writerow(["pattern", "cycle", "motor", "step_index",
                         "target_deg", "final_angle_deg", "error_deg", "pass"])

        # Run patterns
        pattern_alternating_90(m1, m2, CYCLES_SHORT, writer, TOL)
        pattern_figure_eight(m1, m2, CYCLES_SHORT, writer, TOL)
        pattern_interleaved(m1, m2, CYCLES_SHORT, writer, TOL)

    # Summarize results
    # Quick pass/fail by checking final positions (should be ~0)
    e1 = err_to_target(m1.angle, 0)
    e2 = err_to_target(m2.angle, 0)
    ok1, ok2 = near(e1, TOL), near(e2, TOL)
    status = "✅ PASS" if (ok1 and ok2) else "⚠️ CHECK"
    print(f"{status}  Final (deg): m1={m1.angle:.2f} (err {e1:+.2f}), "
          f"m2={m2.angle:.2f} (err {e2:+.2f}). "
          f"Log written to {log_name}")

    if not (ok1 and ok2):
        print("Hints: increase Stepper.delay (µs), verify coil order A-B-C-D, "
              "ensure 5 V supply is stable, and grounds are tied.")
