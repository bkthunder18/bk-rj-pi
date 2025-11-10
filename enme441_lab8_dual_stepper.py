
# enme441_lab8_dual_stepper.py
# Dual 28BYJ-48 steppers via 74HC595 + L293D/ULN2003 drivers (Pi Zero)
# Half-step mode (4096 half-steps/rev). Two motors share one 74HC595:
#   - Motor 1 on QA..QD (lower nibble)
#   - Motor 2 on QE..QH (upper nibble)
#
# Default pins (BCM): DATA=16, CLOCK=20, LATCH=21  -> change if wired differently.
#
# Run:  python3 enme441_lab8_dual_stepper.py
#
# Tip: If a motor runs backwards, swap any two adjacent coil wires on the driver header
#      or reverse the sign of your requested angles.

import time
from multiprocessing import Process, Value, Lock
from ctypes import c_ubyte, c_double

try:
    import RPi.GPIO as GPIO
except Exception:
    class _DummyGPIO:
        BCM=BOARD=OUT=LOW=HIGH=None
        def setmode(self,*a,**k): pass
        def setup(self,*a,**k): pass
        def output(self,*a,**k): pass
        def cleanup(self): pass
    GPIO=_DummyGPIO()

# ------------- 74HC595 low-level -------------
SHIFTER_DATA  = 16   # BCM pins
SHIFTER_CLOCK = 20
SHIFTER_LATCH = 21

GPIO.setmode(GPIO.BCM)
GPIO.setup(SHIFTER_DATA,  GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(SHIFTER_CLOCK, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(SHIFTER_LATCH, GPIO.OUT, initial=GPIO.LOW)

def _pulse(pin: int) -> None:
    GPIO.output(pin, True)
    GPIO.output(pin, False)

def shift_out_byte(databyte: int) -> None:
    """Shift 8 bits (LSB-first) into the 74HC595, then latch."""
    databyte &= 0xFF
    for i in range(8):
        bit = (databyte >> i) & 0x1
        GPIO.output(SHIFTER_DATA, bool(bit))
        _pulse(SHIFTER_CLOCK)
    _pulse(SHIFTER_LATCH)

# ------------- Shared bus abstraction -------------
class Bus595:
    """8-bit shadow register with locking. Motor1 uses low nibble, Motor2 uses high nibble."""
    def __init__(self):
        self.byte = Value(c_ubyte, 0)
        self.lock = Lock()
    def write_nibble(self, which: int, value: int) -> None:
        value &= 0xF
        with self.lock:
            cur = self.byte.value
            if which == 0:        # low nibble -> QA..QD
                cur = (cur & 0xF0) | value
            else:                  # high nibble -> QE..QH
                cur = (cur & 0x0F) | (value << 4)
            self.byte.value = cur
            shift_out_byte(cur)
    def all_off(self) -> None:
        with self.lock:
            self.byte.value = 0
            shift_out_byte(0)

# ------------- Stepper driver (28BYJ-48 half-step) -------------
class Stepper:
    # One-hot and adjacent-hot sequence (half-step)
    HALFSTEP_SEQ = (0x1, 0x3, 0x2, 0x6, 0x4, 0xC, 0x8, 0x9)
    STEPS_PER_REV = 4096  # gearbox accounted for

    def __init__(self, bus: Bus595, nibble_index: int, step_hz: float = 600.0):
        assert nibble_index in (0, 1), "nibble_index must be 0 (low) or 1 (high)"
        self.bus = bus
        self.nibble = nibble_index
        self.angle_deg = Value(c_double, 0.0)  # shared state across processes
        self.seq_index = Value(c_ubyte, 0)     # [0..7]
        # Guard speed to something most 28BYJ-48s will tolerate
        step_hz = max(20.0, min(step_hz, 800.0))
        self.step_delay = 1.0 / step_hz

    @staticmethod
    def _deg_to_steps(deg: float) -> int:
        return int(round(deg * Stepper.STEPS_PER_REV / 360.0))
    @staticmethod
    def _steps_to_deg(steps: int) -> float:
        return steps * 360.0 / Stepper.STEPS_PER_REV

    def zero(self) -> None:
        with self.angle_deg.get_lock():
            self.angle_deg.value = 0.0

    def rotate(self, deg: float) -> None:
        self._start_motion(self._deg_to_steps(deg))

    def goAngle(self, target_deg: float) -> None:
        # Normalize target to [0, 360)
        t = (target_deg % 360.0 + 360.0) % 360.0
        with self.angle_deg.get_lock():
            cur = (self.angle_deg.value % 360.0 + 360.0) % 360.0
        delta = t - cur
        if   delta > 180.0: delta -= 360.0
        elif delta <= -180.0: delta += 360.0
        self.rotate(delta)

    def _start_motion(self, steps: int) -> None:
        p = Process(target=self._run_steps, args=(steps,))
        p.daemon = True
        p.start()

    def _run_steps(self, steps: int) -> None:
        direction = 1 if steps >= 0 else -1
        steps = abs(steps)
        for _ in range(steps):
            with self.seq_index.get_lock():
                self.seq_index.value = (self.seq_index.value + (1 if direction > 0 else -1)) % 8
                seq_val = Stepper.HALFSTEP_SEQ[self.seq_index.value]
            self.bus.write_nibble(self.nibble, seq_val)
            time.sleep(self.step_delay)
        with self.angle_deg.get_lock():
            self.angle_deg.value = (self.angle_deg.value + Stepper._steps_to_deg(direction * steps)) % 360.0
        # De-energize coils to keep drivers cool (comment out to hold torque)
        self.bus.write_nibble(self.nibble, 0x0)

# ------------- Lab 8 demo sequence -------------
def main():
    bus = Bus595()
    m1 = Stepper(bus, nibble_index=0, step_hz=600.0)  # QA..QD
    m2 = Stepper(bus, nibble_index=1, step_hz=600.0)  # QE..QH

    print("Zero both motors...")
    m1.zero(); m2.zero()
    time.sleep(0.2)

    # Sequence (back-to-back calls overlap via multiprocessing)
    print("m1.goAngle(90)")
    m1.goAngle(90)
    print("m1.goAngle(-45)")
    m1.goAngle(-45)

    print("m2.goAngle(-90)")
    m2.goAngle(-90)
    print("m2.goAngle(45)")
    m2.goAngle(45)

    print("m1.goAngle(-135)")
    m1.goAngle(-135)
    print("m1.goAngle(135)")
    m1.goAngle(135)
    print("m1.goAngle(0)")
    m1.goAngle(0)

    # Keep main alive so child processes can finish
    try:
        while True:
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopping and releasing coils...")
        bus.all_off()

if __name__ == "__main__":
    try:
        main()
    finally:
        try:
            GPIO.cleanup()
        except Exception:
            pass
