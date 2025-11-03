# angle_accuracy_test.py
# Tests 90° alternating moves on m1 and m2

import time
import multiprocessing
from shifter import Shifter
from stepper_class_shiftregister_multiprocessing import Stepper

# --- Initialize hardware ---
s = Shifter(data=16, latch=20, clock=21)
lock = multiprocessing.Lock()

m1 = Stepper(s, lock)
m2 = Stepper(s, lock)

# Zero logical positions
m1.zero()
m2.zero()

print("Starting 90° alternating move test...")
time.sleep(1)

# Define how many cycles you want (each cycle returns to start)
cycles = 2  # change to 4 or 8 for a longer test

for i in range(cycles):
    print(f"\nCycle {i+1} -----------------------------")

    # Move motor 1 +90°, then back
    print("m1 → +90°")
    m1.goAngle(90)
    time.sleep(4)   # wait long enough to finish

    print("m1 → 0°")
    m1.goAngle(0)
    time.sleep(4)

    # Move motor 2 +90°, then back
    print("m2 → +90°")
    m2.goAngle(90)
    time.sleep(4)

    print("m2 → 0°")
    m2.goAngle(0)
    time.sleep(4)

# After final return
print("\n✅ Test complete. Both motors should now be at ~0° (original positions).")

# Optional: display final angles from stored values
print(f"m1 angle: {m1.angle:.2f}°, m2 angle: {m2.angle:.2f}°")
