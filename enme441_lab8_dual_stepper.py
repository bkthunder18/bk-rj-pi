
# enme441_lab8_dual_stepper.py
# Dual 28BYJ-48 steppers via 74HC595 + L293D/ULN2003 drivers (Pi Zero)
# Implements multiprocessing so m1 and m2 can run "simultaneously" even if called sequentially.
#
# Wiring assumption:
#   74HC595 QA..QD drive Motor 1 coils (lower nibble)
#   74HC595 QE..QH drive Motor 2 coils (upper nibble)
#
# Change SHIFTER_* pins if needed.

import time
import math
from multiprocessing import Process, Value, Lock
from ctypes import c_ubyte, c_double

try:
    import RPi.GPIO as GPIO
except Exception as e:
    # Allow import on non-Pi machines for static checks, but will fail at runtime on Pi if GPIO missing.
    class _DummyGPIO:
        BCM=BOARD=OUT=LOW=HIGH=None
        def setmode(self,*a,**k): pass
        def setup(self,*a,**k): pass
        def output(self,*a,**k): pass
        def cleanup(self): pass
    GPIO=_DummyGPIO()

# -------------------- 74HC595 DRIVER --------------------
SHIFTER_DATA  = 16   # BCM pins (edit if needed)
SHIFTER_CLOCK = 20
SHIFTER_LATCH = 21

GPIO.setmode(GPIO.BCM)
GPIO.setup(SHIFTER_DATA,  GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(SHIFTER_CLOCK, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(SHIFTER_LATCH, GPIO.OUT, initial=GPIO.LOW)

def _pulse(pin):
    GPIO.output(pin, True)
    GPIO.output(pin, False)

def shift_out_byte(databyte: int):
    \"\"\"Shift 8 bits (LSB first) into the 74HC595, then latch.\"\"\"
    databyte &= 0xFF
    for i in range(8):
        bit = (databyte >> i) & 0x1
        GPIO.output(SHIFTER_DATA, bool(bit))
        _pulse(SHIFTER_CLOCK)
    _pulse(SHIFTER_LATCH)

# -------------------- SHARED BUS --------------------
class Bus595:
    \"\"\"Shared 8-bit output image for the 74HC595 with atomic updates via a Lock.

    Each motor owns a 4-bit nibble:
      - Motor 1 uses bits [3:0]
      - Motor 2 uses bits [7:4]
    \"\"\"
    def __init__(self):
        self.byte = Value(c_ubyte, 0)  # last latched state
        self.lock = Lock()

    def write_nibble(self, which: int, value: int):
        \"\"\"Write 4-bit 'value' into nibble 'which' (0=low, 1=high) without clobbering the other nibble.\"\"\"
        value &= 0xF
        with self.lock:
            cur = self.byte.value
            if which == 0:
                cur = (cur & 0xF0) | value
            else:
                cur = (cur & 0x0F) | (value << 4)
            self.byte.value = cur
            shift_out_byte(cur)

    def all_off(self):
        with self.lock:
            self.byte.value = 0
            shift_out_byte(0)

# -------------------- STEPPER --------------------
class Stepper:
    \"\"\"28BYJ-48 in half-step mode. 4096 half-steps per output rev (1:64 gearbox).

    Half-step coil pattern (one-hot and adjacent-hot):
       0x1, 0x3, 0x2, 0x6, 0x4, 0xC, 0x8, 0x9
    \"\"\"
    HALFSTEP_SEQ = (0x1, 0x3, 0x2, 0x6, 0x4, 0xC, 0x8, 0x9)
    STEPS_PER_REV = 4096  # 2 x 4 x 8 x 64

    def __init__(self, bus: Bus595, nibble_index: int, step_hz: float = 700.0):
        assert nibble_index in (0,1), \"nibble_index must be 0 (low) or 1 (high)\"
        self.bus = bus
        self.nibble = nibble_index
        # Shared angle in degrees, accessible across processes
        self.angle_deg = Value(c_double, 0.0)
        # Current half-step index [0..7]
        self.seq_index = Value(c_ubyte, 0)
        # Timing
        # step_hz is the electrical half-step rate; limit to conservative speed
        step_hz = max(20.0, min(step_hz, 1000.0))
        self.step_delay = 1.0 / step_hz

    @staticmethod
    def _deg_to_steps(deg: float) -> int:
        return int(round(deg * Stepper.STEPS_PER_REV / 360.0))

    @staticmethod
    def _steps_to_deg(steps: int) -> float:
        return steps * 360.0 / Stepper.STEPS_PER_REV

    def zero(self):
        \"\"\"Logical zero: don't move, just define current position as 0°.\"\"\"
        with self.angle_deg.get_lock():
            self.angle_deg.value = 0.0

    def rotate(self, deg: float):
        \"\"\"Relative rotation: positive = CCW, negative = CW (convention).\"\"\"
        steps = self._deg_to_steps(deg)
        self._start_motion(steps)

    def goAngle(self, target_deg: float):
        \"\"\"Absolute move to target angle in degrees using the shortest path modulo 360.\"\"\"
        # Normalize target into [0,360)
        t = ((target_deg % 360.0) + 360.0) % 360.0
        with self.angle_deg.get_lock():
            cur = ((self.angle_deg.value % 360.0) + 360.0) % 360.0
        # Compute shortest signed delta in degrees: result in (-180, 180]
        delta = t - cur
        if delta > 180.0:
            delta -= 360.0
        elif delta <= -180.0:
            delta += 360.0
        self.rotate(delta)

    def _start_motion(self, steps: int):
        # Launch a process so calls on different motors can overlap.
        p = Process(target=self._run_steps, args=(steps,))
        p.daemon = True
        p.start()

    def _run_steps(self, steps: int):
        direction = 1 if steps >= 0 else -1
        steps = abs(steps)
        for _ in range(steps):
            # advance sequence index
            with self.seq_index.get_lock():
                self.seq_index.value = (self.seq_index.value + (1 if direction>0 else -1)) % 8
                seq_val = Stepper.HALFSTEP_SEQ[self.seq_index.value]
            # write nibble without clobbering the other motor
            self.bus.write_nibble(self.nibble, seq_val)
            time.sleep(self.step_delay)
        # update angle
        with self.angle_deg.get_lock():
            self.angle_deg.value = (self.angle_deg.value + Stepper._steps_to_deg(direction*steps)) % 360.0
        # release coils at the end (optional; comment out to hold torque)
        self.bus.write_nibble(self.nibble, 0x0)

# -------------------- DEMO (Lab sequence) --------------------
def main():
    bus = Bus595()
    m1 = Stepper(bus, nibble_index=0, step_hz=700.0)  # lower nibble
    m2 = Stepper(bus, nibble_index=1, step_hz=700.0)  # upper nibble

    print(\"Zero both motors...\")
    m1.zero()
    m2.zero()
    time.sleep(0.2)

    # Lab sequence (both motors should operate simultaneously where calls are back-to-back)
    print(\"m1.goAngle(90)\")
    m1.goAngle(90)
    print(\"m1.goAngle(-45)\")
    m1.goAngle(-45)

    print(\"m2.goAngle(-90)\")
    m2.goAngle(-90)
    print(\"m2.goAngle(45)\")
    m2.goAngle(45)

    print(\"m1.goAngle(-135)\")
    m1.goAngle(-135)
    print(\"m1.goAngle(135)\")
    m1.goAngle(135)
    print(\"m1.goAngle(0)\")
    m1.goAngle(0)

    # Keep main alive while child processes finish.
    try:
        while True:
            time.sleep(0.05)
    except KeyboardInterrupt:
        print(\"\\nStopping and releasing coils...\")
        bus.all_off()

if __name__ == \"__main__\":
    main()
