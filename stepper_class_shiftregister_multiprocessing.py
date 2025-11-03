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

    def __init__(self, shifter, lock, outputs):  #**added outputs param to accept shared SR byte for concurrency**
        self.s = shifter           # shift register
        self.step_state = 0        # track position in sequence
        self.shifter_bit_start = 4*Stepper.num_steppers  # starting bit position
        self.lock = lock           # multiprocessing lock
        self.outputs = outputs     #**store the shared SR byte so all processes read/write the same value**
        self.angle = multiprocessing.Value('d', 0.0)  #**make angle a shared double so child processes update true position**

        Stepper.num_steppers += 1   # increment the instance count

    # Signum function:
    def __sgn(self, x):
        if x == 0: return(0)
        else: return(int(abs(x)/x))

    # Move a single +/-1 step in the motor sequence:
    def __step(self, dir):
        self.step_state = (self.step_state + dir) % 8  #**fix wrap logic; compute next state once per step**

        mask = (0b1111 << self.shifter_bit_start)  #**mask for just this motor’s 4 bits**
        new_nibble = (Stepper.seq[self.step_state] << self.shifter_bit_start) & mask  #**prepare nibble for this motor only**

        with self.lock:  #**narrow critical section to only the byte merge + shift (enables interleaving)**
            current = self.outputs.value  #**read shared SR byte**
            current = (current & ~mask) | new_nibble  #**clear my 4 bits, set new nibble; preserve other motors**
            self.outputs.value = current  #**write back shared SR byte**
            self.s.shiftByte(current)  #**clock out merged value to the 74HC595**

        self.angle.value = (self.angle.value + dir/Stepper.steps_per_degree) % 360  #**update shared angle so goAngle has correct state**

    # Move relative angle from current position:
    def __rotate(self, delta):
        numSteps = int(Stepper.steps_per_degree * abs(delta))
        dir = self.__sgn(delta)
        for _ in range(numSteps):  #**iterate without holding a big lock (lock is inside __step)**
            self.__step(dir)  #**step uses fine-grained locking for the SR update**
            time.sleep(Stepper.delay/1e6)

    # Move relative angle from current position:
    def rotate(self, delta):
        time.sleep(0.1)
        p = multiprocessing.Process(target=self.__rotate, args=(delta,))
        p.start()

    # Move to an absolute angle taking the shortest possible path:
    def goAngle(self, angle):
        curr = self.angle.value  #**read current shared angle**
        delta = ((angle - curr + 180) % 360) - 180  #**compute shortest-path delta in [-180,180)**
        self.rotate(delta)  #**reuse rotate for motion**

    # Set the motor zero point
    def zero(self):
        self.angle.value = 0.0  #**reset shared angle to zero**

# Example use:

if __name__ == '__main__':

    s = Shifter(data=16,latch=20,clock=21)   # set up Shifter

    lock = multiprocessing.Lock()  #**single shared lock for SR critical sections**

    shared_outputs = multiprocessing.Value('i', 0)  #**shared 8-bit SR value across all Stepper processes**

    m1 = Stepper(s, lock, shared_outputs)  #**pass shared_outputs to each Stepper**
    m2 = Stepper(s, lock, shared_outputs)  #**pass shared_outputs to each Stepper**

    m1.zero()  #**initialize shared angle for repeatable absolute moves**
    m2.zero()  #**initialize shared angle for repeatable absolute moves**

    # Demo sequence matching lab intent (simultaneous moves happen because locks are fine-grained)
    m1.goAngle(90)    #**use absolute positioning per lab remainder**
    m1.goAngle(-45)   #**use shortest-path absolute move**

    m2.goAngle(-90)   #**independent absolute command can run concurrently**
    m2.goAngle(45)    #**independent absolute command can run concurrently**

    m1.goAngle(-135)  #**more absolute moves**
    m1.goAngle(135)   #**more absolute moves**
    m1.goAngle(0)     #**return to zero**

    try:
        while True:
            pass
    except:
        print('\nend')
