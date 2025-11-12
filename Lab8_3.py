# stepper_class_shiftregister_multiprocessing.py
#
# ENME441 Lab 8 — Dual 28BYJ-48 via 74HC595 shift register + multiprocessing
# - Shortest-path absolute moves (goAngle)
# - Simultaneous motion (each motor in its own Process)
# - Shared angle across processes (multiprocessing.Value)
#
# Wiring assumptions:
#   - One 74HC595 drives up to two 28BYJ-48 steppers via an H-bridge/ULN2003.
#   - Lower nibble QA..QD = Motor 1 coils, Upper nibble QE..QH = Motor 2 coils.
#   - Shifter.shiftByte(byte) latches QA..QH on each call.
#
# Notes:
#   - Steps per rev = 4096 (28BYJ-48 with gearbox)
#   - steps_per_degree = 4096/360
#   - delay is in microseconds (us)

import time
import multiprocessing
from shifter import Shifter   # your provided Shifter class (data, latch, clock)

class Stepper:
    """
    Drive multiple 28BYJ-48 steppers through one or more 74HC595 shift registers.

    Instance layout:
      - Each Stepper claims a 4-bit nibble (QA..QD, then QE..QH, then next chip…)
      - Nibble offset = 4*Stepper.num_steppers at construction time.

    Concurrency model:
      - rotate()/goAngle() start a new Process running __rotate().
      - __step() writes to the *shared* shift-register byte with a Lock so that
        two motors never stomp on each other’s nibble during the latch.
      - Angle is stored in multiprocessing.Value('d', 0.0) so increments done in
        child processes persist and remain visible to the parent.
    """

    # Class attributes (shared across instances):
    num_steppers = 0
    shifter_outputs = multiprocessing.Value('i', 0)   # 8-bit QA..QH image shared by all motors
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001]  # half-step sequence (CCW)
    delay = 1200                                      # us between steps
    steps_per_degree = 4096/360.0                     # 4096 steps per 360°

    def __init__(self, shifter: Shifter, lock: multiprocessing.Lock):
        self.s = shifter
        # Use a shared double so child processes update the *same* angle
        self.angle = multiprocessing.Value('d', 0.0)  # degrees
        self.step_state = 0                           # index in seq (0..7)
        self.shifter_bit_start = 4*Stepper.num_steppers  # which nibble this motor owns
        self.lock = lock                              # lock to serialize shift-register writes

        Stepper.num_steppers += 1

    # sign function returning -1, 0, +1
    def __sgn(self, x: float) -> int:
        if x == 0: return 0
        return 1 if x > 0 else -1

    # Take a single step in +1 / -1 direction
    def __step(self, dir_sign: int) -> None:
        # advance sequence state (wrap 0..7)
        self.step_state = (self.step_state + dir_sign) % 8

        # Safely update the shared QA..QH byte and latch it
        with self.lock:
            sep = Stepper.shifter_outputs.value
            # Clear this motor’s 4-bit nibble, then OR in the new 4-bit pattern
            sep &= ~(0b1111 << self.shifter_bit_start)
            sep |= (Stepper.seq[self.step_state] << self.shifter_bit_start)
            Stepper.shifter_outputs.value = sep
            self.s.shiftByte(sep)

        # Update the shared angle atomically (child processes persist the change)
        with self.angle.get_lock():
            self.angle.value = (self.angle.value + dir_sign / Stepper.steps_per_degree) % 360.0

    # Worker that runs in a child process to make a relative move
    def __rotate(self, delta_deg: float) -> None:
        num_steps = int(abs(delta_deg) * Stepper.steps_per_degree)
        dir_sign = self.__sgn(delta_deg)
        for _ in range(num_steps):
            self.__step(dir_sign)
            time.sleep(Stepper.delay/1e6)

    # Non-blocking relative move from current position — returns the Process so you can .join()
    def rotate(self, delta_deg: float) -> multiprocessing.Process:
        p = multiprocessing.Process(target=self.__rotate, args=(delta_deg,))
        p.start()
        return p

    # Blocking relative move (optional helper)
    def rotate_sync(self, delta_deg: float) -> None:
        p = self.rotate(delta_deg)
        p.join()

    # Absolute move to |angle| in degrees, taking the shortest possible path
    def goAngle(self, angle: float) -> multiprocessing.Process:
        angle = angle % 360.0
        current = self.angle.value
        delta = angle - current
        # Wrap into [-180, +180] for shortest path
        if   delta >  180.0: delta -= 360.0
        elif delta < -180.0: delta += 360.0
        return self.rotate(delta)

    # Reset stored angle to 0° (does NOT physically home the motor)
    def zero(self) -> None:
        self.angle.value = 0.0


# -------------------------
# Example usage / smoke test
# -------------------------
if __name__ == '__main__':
    # Set up shifter — match your Pi wiring (GPIO numbers here are BCM)
    s = Shifter(data=16, latch=20, clock=21)

    # A single lock protects the shared shift-register byte for all motors
    lock = multiprocessing.Lock()

    # Two motors share one 74HC595 (lower nibble = m1, upper nibble = m2)
    m1 = Stepper(s, lock)
    m2 = Stepper(s, lock)

    # Zero the reported angles
    m1.zero()
    m2.zero()

    # --- Demo 1: simultaneous non-blocking motion ---
    # Both should start together and step interleaved via the shared lock
    p1 = m1.goAngle(90)     # shortest path from 0 -> +90
    p2 = m2.goAngle(270)    # shortest path from 0 -> -90
    p1.join(); p2.join()
    print(f"m1 angle ≈ {m1.angle.value:.1f}°  |  m2 angle ≈ {m2.angle.value:.1f}°")

    # --- Demo 2: more movements ---
    # Move both to 350°, observe shortest paths
    p1 = m1.goAngle(350)
    p2 = m2.goAngle(350)
    p1.join(); p2.join()
    print(f"m1 angle ≈ {m1.angle.value:.1f}°  |  m2 angle ≈ {m2.angle.value:.1f}°")

    # Keep main alive if you want to scope outputs or extend tests
    try:
        while False:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nend")
