# Notes from lecture:
#   - Steps per rev = 4096 (28BYJ-48 with gearbox)
#   - steps_per_degree = 4096/360
#   - delay in microseconds

import time
import multiprocessing
from shifter import Shifter

class Stepper:
    num_steppers = 0
    shifter_outputs = multiprocessing.Value('i', 0)
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001]
    delay = 1200
    steps_per_degree = 4096/360.0

    def __init__(self, shifter: Shifter, lock: multiprocessing.Lock):
        self.s = shifter
        self.angle = multiprocessing.Value('d', 0.0)
        self.step_state = 0
        self.shifter_bit_start = 4 * Stepper.num_steppers
        self.lock = lock
        Stepper.num_steppers += 1

    def __sgn(self, x: float) -> int:
        if x == 0: return 0
        return 1 if x > 0 else -1

    def __step(self, dir_sign: int) -> None:
        self.step_state = (self.step_state + dir_sign) % 8
        with self.lock:
            sep = Stepper.shifter_outputs.value
            sep &= ~(0b1111 << self.shifter_bit_start)
            sep |= (Stepper.seq[self.step_state] << self.shifter_bit_start)
            Stepper.shifter_outputs.value = sep
            self.s.shiftByte(sep)
        with self.angle.get_lock():
            self.angle.value = (self.angle.value + dir_sign / Stepper.steps_per_degree) % 360.0

    def __rotate(self, delta_deg: float) -> None:
        num_steps = int(abs(delta_deg) * Stepper.steps_per_degree)
        dir_sign = self.__sgn(delta_deg)
        for _ in range(num_steps):
            self.__step(dir_sign)
            time.sleep(Stepper.delay / 1e6)

    def rotate(self, delta_deg: float) -> multiprocessing.Process:
        p = multiprocessing.Process(target=self.__rotate, args=(delta_deg,))
        p.start()
        return p

    def rotate_sync(self, delta_deg: float) -> None:
        p = self.rotate(delta_deg)
        p.join()

    def goAngle(self, angle: float) -> multiprocessing.Process:
        angle = angle % 360.0
        current = self.angle.value
        delta = angle - current
        if delta > 180.0:
            delta -= 360.0
        elif delta < -180.0:
            delta += 360.0
        return self.rotate(delta)

    def zero(self) -> None:
        self.angle.value = 0.0


if __name__ == '__main__':
    s = Shifter(data=16, latch=20, clock=21)
    lock = multiprocessing.Lock()

    m1 = Stepper(s, lock)
    m2 = Stepper(s, lock)

    m1.zero()
    m2.zero()
    print(f"Zeroed: m1={m1.angle.value:.1f}°, m2={m2.angle.value:.1f}°")

    # Step 4: run sequences, both at same time
    p1 = m1.goAngle(90)
    p2 = m2.goAngle(-90)
    p1.join()
    p2.join()

    p1 = m1.goAngle(-45)
    p2 = m2.goAngle(45)
    p1.join()
    p2.join()

    p1 = m1.goAngle(-135)
    p1.join()

    p1 = m1.goAngle(135)
    p1.join()

    p1 = m1.goAngle(0)
    p1.join()

    print(f"Final angles: m1={m1.angle.value:.1f}°, m2={m2.angle.value:.1f}°")
