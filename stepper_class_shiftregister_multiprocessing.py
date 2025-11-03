# stepper_class_shiftregister_multiprocessing.py
#
# Stepper class
#
# Because only one motor action is allowed at a time, multithreading could be
# used instead of multiprocessing. However, the GIL makes the motor process run 
# too slowly on the Pi Zero, so multiprocessing is needed.

import time
import multiprocessing
from shifter import Shifter   # our custom Shifter class

class Stepper:
    """
    Supports operation of an arbitrary number of stepper motors using
    one or more shift registers.
  
    A class attribute (shifter_outputs) keeps track of all
    shift register output values for all motors.  In addition to
    simplifying sequential control of multiple motors, this schema also
    makes simultaneous operation of multiple motors possible.
   
    Motor instantiation sequence is inverted from the shift register outputs.
    For example, in the case of 2 motors, the 2nd motor must be connected
    with the first set of shift register outputs (Qa-Qd), and the 1st motor
    with the second set of outputs (Qe-Qh). This is because the MSB of
    the register is associated with Qa, and the LSB with Qh (look at the code
    to see why this makes sense).
 
    An instance attribute (shifter_bit_start) tracks the bit position
    in the shift register where the 4 control bits for each motor
    begin.
    """

    # Class attributes:
    num_steppers = 0      # track number of Steppers instantiated
    shifter_outputs = 0   # track shift register outputs for all motors
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001] # CCW sequence
    delay = 1200          # delay between motor steps [us]
    steps_per_degree = 4096/360    # 4096 steps/rev * 1/360 rev/deg

    def __init__(self, shifter, lock, outputs):  #**accept shared outputs for concurrency**
        self.s = shifter           # shift register
        self.step_state = 0        # track position in sequence
        self.shifter_bit_start = 4*Stepper.num_steppers  # starting bit position
        self.lock = lock           # multiprocessing lock
        self.outputs = outputs     #**store shared shift-register byte across processes**
        self.angle = multiprocessing.Value('d', 0.0)  #**shared angle so child process updates are visible**
        self.proc = None  #**track the currently-running process for this motor to serialize its commands smoothly**

        Stepper.num_steppers += 1   # increment the instance count

    # Signum function:
    def __sgn(self, x):
        if x == 0: return(0)
        else: return(int(abs(x)/x))

    # Move a single +/-1 step in the motor sequence:
    def __step(self, dir):
        self.step_state = (self.step_state + dir) % 8  #**stable wrap logic for next microstep**

        mask = (0b1111 << self.shifter_bit_start)  #**mask for just this motor’s 4 output bits**
        new_nibble = (Stepper.seq[self.step_state] << self.shifter_bit_start) & mask  #**prepare nibble for only this motor**

        with self.lock:  #**tiny critical section: merge my nibble and clock it out**
            current = self.outputs.value  #**read shared 8-bit register**
            current = (current & ~mask) | new_nibble  #**preserve other motors; update only my 4 bits**
            self.outputs.value = current  #**write back merged byte**
            self.s.shiftByte(current)  #**send to the 74HC595**

        self.angle.value = (self.angle.value + dir/Stepper.steps_per_degree) % 360  #**maintain shared absolute angle**

    # Move relative angle from current position (runs inside a process):
    def __rotate(self, delta):
        numSteps = int(Stepper.steps_per_degree * abs(delta))
        dir = self.__sgn(delta)
        for _ in range(numSteps):
            self.__step(dir)
            time.sleep(Stepper.delay/1e6)

    # Public relative move: serialize per-motor by waiting for prior process to finish
    def rotate(self, delta):
        if self.proc is not None and self.proc.is_alive():  #**wait for previous command on THIS motor for smoothness**
            self.proc.join()  #**serialize same-motor commands; different motors still run concurrently**
        self.proc = multiprocessing.Process(target=self.__rotate, args=(delta,))  #**start fresh process for this move**
        self.proc.start()  #**launch the move**

    # Move to an absolute angle taking the shortest possible path:
    def goAngle(self, angle):
        curr = self.angle.value  #**read current shared angle**
        delta = ((angle - curr + 180) % 360) - 180  #**compute shortest-path delta in degrees**
        self.rotate(delta)  #**queue as a serialized relative move for smooth stepping**

    # Set the motor zero point
    def zero(self):
        self.angle.value = 0.0  #**reset shared angle to zero**

# Example use:

if __name__ == '__main__':

    s = Shifter(data=16,latch=20,clock=21)   # set up Shifter

    lock = multiprocessing.Lock()  #**single shared lock to protect SR byte updates**

    shared_outputs = multiprocessing.Value('i', 0)  #**shared 8-bit shift-register value**

    m1 = Stepper(s, lock, shared_outputs)  #**pass shared outputs**
    m2 = Stepper(s, lock, shared_outputs)  #**pass shared outputs**

    m1.zero()  #**initialize**
    m2.zero()  #**initialize**

    # Demo sequence: now smooth (same-motor commands run one-after-another);
    # different motors still run at the same time.
    m1.goAngle(90)     #**absolute move; m1 starts**
    m2.goAngle(-90)    #**absolute move; m2 starts concurrently**
    m1.goAngle(-45)    #**will begin AFTER m1 reaches 90, keeping m1 smooth**
    m2.goAngle(45)     #**will begin AFTER m2 reaches -90, keeping m2 smooth**
    m1.goAngle(-135)   #**queued for m1**
    m1.goAngle(135)    #**queued for m1**
    m1.goAngle(0)      #**queued for m1 (final)**
    # m2 has no further commands in this demo and will hold at 45°

    try:
        while True:
            time.sleep(0.1)
    except:
        print('\nend')
