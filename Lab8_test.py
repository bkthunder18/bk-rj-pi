# ENME 441 – Lab 8: Stepper Motor Control (two motors, simultaneous)
# Raspberry Pi Zero  -> 74HC595 -> L293D -> 28BYJ-48 (x2)
# GPIO: SER=GPIO16, SRCLK=GPIO21, RCLK=GPIO20
#
# Features:
# - Two independent 28BYJ-48 stepper objects (m1, m2)
# - Simultaneous operation via one controller process that composes 8 output bits each tick
# - multiprocessing.Value used for .angle (degrees) and .step_idx shared state
# - goAngle(a): absolute-angle command with shortest-path math
#
# Wiring bit map (choose which 4 bits in the 595 drive each motor's coils A,B,C,D):
#   Here: motor1 uses QE..QH (bits 4..7), motor2 uses QA..QD (bits 0..3)
#
# Author: you
# Date: 2025-11-03

import time
import math
import multiprocessing as mp

import RPi.GPIO as GPIO

# ----------------------------- 595 SHIFTER -----------------------------
class Shifter595:
    """
    Minimal 74HC595 driver using 3 GPIOs.
    data -> SER (DS), clock -> SH_CP, latch -> ST_CP
    """
    def __init__(self, data_pin=16, latch_pin=20, clock_pin=21):
        self.data = data_pin
        self.latch = latch_pin
        self.clock = clock_pin
        GPIO.setup(self.data,  GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.latch, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.clock, GPIO.OUT, initial=GPIO.LOW)

    def write_byte(self, value: int):
        """Shift out MSB->LSB or LSB->MSB; we’ll use MSB-first here."""
        val = value & 0xFF
        # latch low while shifting
        GPIO.output(self.latch, GPIO.LOW)
        for bitpos in range(7, -1, -1):   # MSB first
            bit = (val >> bitpos) & 1
            GPIO.output(self.data, GPIO.HIGH if bit else GPIO.LOW)
            # pulse clock
            GPIO.output(self.clock, GPIO.HIGH)
            GPIO.output(self.clock, GPIO.LOW)
        # latch to outputs
        GPIO.output(self.latch, GPIO.HIGH)
        GPIO.output(self.latch, GPIO.LOW)

# ---------------------------- STEPPER MODEL ----------------------------
# 28BYJ-48 half-step sequence (A,B,C,D)
HALFSTEP = (
    (1,0,0,0),
    (1,1,0,0),
    (0,1,0,0),
    (0,1,1,0),
    (0,0,1,0),
    (0,0,1,1),
    (0,0,0,1),
    (1,0,0,1),
)
STEPS_PER_REV = 4096  # 28BYJ-48, half-step with gearbox (~1:64)

class Stepper:
    """
    Logical stepper motor object:
      - maintains shared angle (deg) and step index (0..7)
      - exposes goAngle(a_deg) to enqueue absolute target
      - uses a per-motor command Queue consumed by the controller
    """
    def __init__(self, name, bit_map, shared_state, cmd_queue):
        """
        bit_map: (bA, bB, bC, bD) → which 595 bit drives A,B,C,D coils (0..7)
        shared_state: dict-like with:
          - angle: multiprocessing.Value('d')
          - step_idx: multiprocessing.Value('i')
        """
        self.name = name
        self.bits = bit_map
        self.angle = shared_state['angle']     # mp.Value('d')
        self.step_idx = shared_state['step_idx']  # mp.Value('i')
        self.cmd_q = cmd_queue

    def zero(self):
        """Set current logical angle to 0° (no motion)."""
        with self.angle.get_lock():
            self.angle.value = 0.0

    def goAngle(self, target_deg: float, rpm: float = 10.0):
        """
        Enqueue an absolute-angle move at roughly the given RPM.
        Non-blocking so multiple motors can move simultaneously.
        """
        # compute shortest-path delta at enqueue time based on current *shared* angle
        with self.angle.get_lock():
            curr = self.angle.value
        delta = shortest_path_deg(curr, target_deg)
        steps = angle_to_steps(delta)
        direction = 1 if steps > 0 else -1 if steps < 0 else 0
        steps = abs(steps)

        # convert RPM to per-half-step delay (approx)
        # rev/min -> steps/sec: (rpm * STEPS_PER_REV) / 60
        # delay = 1 / steps_per_sec
        steps_per_sec = max(1.0, (rpm * STEPS_PER_REV) / 60.0)
        delay_s = 1.0 / steps_per_sec

        # Put a move plan on our queue
        self.cmd_q.put({
            'type': 'move',
            'steps': steps,
            'dir': direction,      # +1 forward, -1 backward
            'delay': delay_s
        })

# ------------------------- CONTROLLER PROCESS --------------------------
def controller_process(pin_cfg, motor_defs):
    """
    Runs in its own process:
      - consumes queued commands for each motor
      - advances active motors step-by-step
      - composes one 8-bit pattern (for 74HC595) per tick (so both motors step "at the same time")
    pin_cfg: dict with 'data','latch','clock'
    motor_defs: list of dicts:
        {
          'name': str,
          'bits': (bA,bB,bC,bD),
          'angle': mp.Value('d'),
          'step_idx': mp.Value('i'),
          'queue': mp.Queue()
        }
    """
    GPIO.setmode(GPIO.BCM)
    sh = Shifter595(pin_cfg['data'], pin_cfg['latch'], pin_cfg['clock'])

    # active motion state per motor
    active = []
    for md in motor_defs:
        active.append({
            'steps_left': 0,
            'dir': 0,
            'delay': 0.003,
            'last_step_time': 0.0
        })

    try:
        while True:
            now = time.time()

            # Pull new commands (non-blocking) for each motor if idle
            for i, md in enumerate(motor_defs):
                if active[i]['steps_left'] == 0:
                    try:
                        cmd = md['queue'].get_nowait()
                        if cmd.get('type') == 'move' and cmd['steps'] > 0 and cmd['dir'] != 0:
                            active[i]['steps_left'] = int(cmd['steps'])
                            active[i]['dir'] = int(cmd['dir'])
                            active[i]['delay'] = float(cmd['delay'])
                            # last_step_time <= now so we can step immediately
                            active[i]['last_step_time'] = 0.0
                    except Exception:
                        pass  # nothing queued

            # Decide if any motor needs to step this tick
            # Build coil states per motor based on (potential) next step index
            out_bits = 0

            for i, md in enumerate(motor_defs):
                step_idx = md['step_idx']
                with step_idx.get_lock():
                    idx_val = step_idx.value

                move_now = False
                if active[i]['steps_left'] > 0:
                    if (now - active[i]['last_step_time']) >= active[i]['delay']:
                        move_now = True

                # Determine which HALFSTEP tuple to output *this* tick
                next_idx = idx_val
                if move_now:
                    next_idx = (idx_val + active[i]['dir']) % 8
                coils = HALFSTEP[next_idx]

                # Place this motor's coils into the combined 8-bit
                bA, bB, bC, bD = md['bits']
                for bit, coil_on in zip((bA,bB,bC,bD), coils):
                    if coil_on:
                        out_bits |= (1 << bit)

            # Write combined pattern
            sh.write_byte(out_bits)

            # Now commit any steps we actually took, and update angles
            for i, md in enumerate(motor_defs):
                if active[i]['steps_left'] > 0 and (now - active[i]['last_step_time']) >= active[i]['delay']:
                    # commit one half-step
                    with md['step_idx'].get_lock():
                        md['step_idx'].value = (md['step_idx'].value + active[i]['dir']) % 8
                    with md['angle'].get_lock():
                        # angle delta per half-step:
                        md['angle'].value = wrap_deg(md['angle'].value + steps_to_angle(active[i]['dir']))
                    active[i]['steps_left'] -= 1
                    active[i]['last_step_time'] = now

            # Small idle
            time.sleep(0.0005)

    finally:
        # de-energize coils
        sh.write_byte(0x00)
        GPIO.cleanup()

# ---------------------------- MATH HELPERS -----------------------------
def wrap_deg(a):
    """Wrap angle to [-180, 180)."""
    a = (a + 180.0) % 360.0 - 180.0
    return a

def shortest_path_deg(curr, target):
    """Return delta (target - curr) along shortest path in [-180, 180)."""
    curr_w = wrap_deg(curr)
    tgt_w = wrap_deg(target)
    d = tgt_w - curr_w
    if d > 180.0:  d -= 360.0
    if d <= -180.0: d += 360.0
    return d

def angle_to_steps(delta_deg):
    """Convert degrees to half-steps (integer)."""
    steps = int(round((delta_deg / 360.0) * STEPS_PER_REV))
    return steps

def steps_to_angle(direction):
    """Angle delta for a single half-step with given dir (+1/-1)."""
    return (direction * 360.0) / STEPS_PER_REV

# ------------------------------- MAIN ---------------------------------
if __name__ == "__main__":
    # GPIO init in controller process
    pin_cfg = {'data': 16, 'latch': 20, 'clock': 21}

    # Shared state for two motors
    # Motor 1 uses QE..QH (bits 4..7). Motor 2 uses QA..QD (bits 0..3).
    BIT_MAP_M1 = (4,5,6,7)   # A,B,C,D -> QE,QF,QG,QH
    BIT_MAP_M2 = (0,1,2,3)   # A,B,C,D -> QA,QB,QC,QD

    # Build shared structures
    mgr = mp.Manager()

    m1_state = {'angle': mp.Value('d', 0.0),
                'step_idx': mp.Value('i', 0)}
    m2_state = {'angle': mp.Value('d', 0.0),
                'step_idx': mp.Value('i', 0)}

    q1 = mp.Queue()
    q2 = mp.Queue()

    motor_defs = [
        {'name':'m1', 'bits': BIT_MAP_M1, 'angle': m1_state['angle'], 'step_idx': m1_state['step_idx'], 'queue': q1},
        {'name':'m2', 'bits': BIT_MAP_M2, 'angle': m2_state['angle'], 'step_idx': m2_state['step_idx'], 'queue': q2},
    ]

    # Start controller process
    ctrl = mp.Process(target=controller_process, args=(pin_cfg, motor_defs), daemon=True)
    ctrl.start()

    # Create user-facing Stepper objects
    # (They just enqueue moves; the controller actually generates steps for both simultaneously.)
    m1 = Stepper('m1', BIT_MAP_M1, m1_state, q1)
    m2 = Stepper('m2', BIT_MAP_M2, m2_state, q2)

    try:
        # --- DEMO SEQUENCE FROM LAB (both motors operate simultaneously) ---
        # 1) zero both (angle bookkeeping)
        m1.zero()
        m2.zero()

        # 2) issue commands (non-blocking, serviced together by controller)
        #   m1.goAngle(90), m1.goAngle(-45)
        m1.goAngle(90)
        m1.goAngle(-45)

        #   m2.goAngle(-90), m2.goAngle(45)
        m2.goAngle(-90)
        m2.goAngle(45)

        #   m1.goAngle(-135), m1.goAngle(135), m1.goAngle(0)
        m1.goAngle(-135)
        m1.goAngle(135)
        m1.goAngle(0)

        # Let controller run until all queues drain and motors finish.
        # Simple “done” detector: wait until both queues are empty and no steps_left for a quiet window.
        # We don’t have direct access to steps_left here, so just wait until angle changes stop for a bit.
        def snapshot_angles():
            with m1_state['angle'].get_lock():
                a1 = m1_state['angle'].value
            with m2_state['angle'].get_lock():
                a2 = m2_state['angle'].value
            return (a1, a2)

        stable_start = None
        last_a1, last_a2 = snapshot_angles()
        while True:
            time.sleep(0.2)
            a1, a2 = snapshot_angles()
            # queues empty?
            empty = q1.empty() and q2.empty()
            # angles stable?
            if abs(a1 - last_a1) < 0.01 and abs(a2 - last_a2) < 0.01 and empty:
                if stable_start is None:
                    stable_start = time.time()
                elif time.time() - stable_start > 0.8:
                    break
            else:
                stable_start = None
            last_a1, last_a2 = a1, a2

        # De-energize coils at end
        # (Controller keeps outputs, but you can just exit main; the controller is daemonized.)
        print("Done. Final angles (deg): m1={:.1f}, m2={:.1f}".format(a1, a2))

    finally:
        # exit main; controller is daemon and will terminate with process
        pass
