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
        self.cmd_q = multiprocessing.Queue()  #**per-motor command queue to avoid spawning a process per move**
        self.stop_evt = multiprocessing.Event()  #**signal to cleanly stop the worker**
        self.worker = multiprocessing.Process(target=self.__worker, args=())  #**dedicated worker to execute steps at steady timing**
        self.worker.start()  #**start the per-motor worker for smooth motion**

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

    def __worker(self):  #**new: steady-timing execution loop improves smoothness**
        step_period = Stepper.delay / 1e6  # seconds per microstep  #**use configured delay as base period**
        pending = 0  # signed steps remaining                        #**track remaining steps smoothly**
        last_time = time.perf_counter()  #**monotonic clock for precise pacing**
        while not self.stop_evt.is_set():
            # Non-blocking drain of latest commands; keep only most recent target/delta burst
            try:
                # Pull all queued items quickly; coalesce into 'pending'
                while True:
                    cmd, val = self.cmd_q.get_nowait()  #**read any queued motion without blocking**
                    if cmd == 'delta':
                        pending += int(Stepper.steps_per_degree * abs(val)) * self.__sgn(val)  #**convert delta degrees to signed steps**
                    elif cmd == 'abs':
                        curr = self.angle.value
                        delta = ((val - curr + 180) % 360) - 180
                        pending = int(Stepper.steps_per_degree * abs(delta)) * self.__sgn(delta)  #**replace with shortest-path absolute move**
                    elif cmd == 'clear':
                        pending = 0  #**cancel pending for this motor**
            except Exception:
                pass

            now = time.perf_counter()
            # Only step when period elapsed; keeps speed constant and smooth
            if pending != 0 and (now - last_time) >= step_period:  #**enforce consistent step interval**
                dir = 1 if pending > 0 else -1
                self.__step(dir)
                pending -= dir
                last_time = now
            else:
                # Sleep a short time to reduce CPU but keep responsiveness
                time.sleep(0.0005)  #**short sleep to avoid busy-wait while staying smooth**

    # Move relative angle from current position:
    def __rotate(self, delta):
        # Replaced by queueing to worker for smooth, serialized per-motor motion
        pass  #**no-op: worker handles motion; kept for interface compatibility**

    # Move relative angle from current position:
    def rotate(self, delta):
        self.cmd_q.put(('delta', float(delta)))  #**enqueue a relative move instead of spawning a new process**

    # Move to an absolute angle taking the shortest possible path:
    def goAngle(self, angle):
        # Clear any queued older commands, then set latest absolute target
        self._clear_queue()  #**drop stale commands to avoid direction thrashing**
        self.cmd_q.put(('abs', float(angle)))  #**enqueue absolute target for smooth shortest-path motion**

    def _clear_queue(self):  #**helper to purge stale commands when a new absolute target arrives**
        try:
            while True:
                self.cmd_q.get_nowait()
        except Exception:
            pass
        self.cmd_q.put(('clear', 0))  #**ensure pending in worker is reset**

    # Set the motor zero point
    def zero(self):
        self.angle.value = 0.0  #**reset shared angle to zero**

    def close(self):  #**allow clean shutdown of the worker**
        self.stop_evt.set()
        try:
            self.worker.join(timeout=1.0)
        except Exception:
            pass

# Example use:

if __name__ == '__main__':

    s = Shifter(data=16,latch=20,clock=21)   # set up Shifter

    lock = multiprocessing.Lock()  #**single shared lock for SR critical sections**

    shared_outputs = multiprocessing.Value('i', 0)  #**shared 8-bit SR value across all Stepper processes**

    m1 = Stepper(s, lock, shared_outputs)  #**pass shared_outputs to each Stepper**
    m2 = Stepper(s, lock, shared_outputs)  #**pass shared_outputs to each Stepper**

    m1.zero()  #**initialize shared angle for repeatable absolute moves**
    m2.zero()  #**initialize shared angle for repeatable absolute moves**

    # Demo sequence — now smooth: each motor’s worker paces steps uniformly,
    # and absolute commands replace prior ones to avoid "tug-of-war."
    m1.goAngle(90)    #**absolute move handled smoothly by the worker**
    m1.goAngle(-45)   #**replaces previous pending for m1 with shortest-path to -45**

    m2.goAngle(-90)   #**absolute move on independent worker**
    m2.goAngle(45)    #**replaces previous pending for m2 with shortest-path to 45**

    m1.goAngle(-135)  #**subsequent absolute commands stay smooth**
    m1.goAngle(135)   #**subsequent absolute commands stay smooth**
    m1.goAngle(0)     #**final absolute target**

    try:
        while True:
            time.sleep(0.1)
    except:
        print('\nend')
        m1.close()  #**cleanly stop worker**
        m2.close()  #**cleanly stop worker**
