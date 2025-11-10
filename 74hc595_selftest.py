
# 74hc595_selftest.py
# Blink QA..QH one-by-one to verify your 74HC595 wiring to the drivers.
import time
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

SHIFTER_DATA  = 16
SHIFTER_CLOCK = 20
SHIFTER_LATCH = 21

GPIO.setmode(GPIO.BCM)
GPIO.setup(SHIFTER_DATA,  GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(SHIFTER_CLOCK, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(SHIFTER_LATCH, GPIO.OUT, initial=GPIO.LOW)

def _pulse(pin):
    GPIO.output(pin, True)
    GPIO.output(pin, False)

def shift_out_byte(b):
    b &= 0xFF
    for i in range(8):
        GPIO.output(SHIFTER_DATA, (b>>i) & 1)
        _pulse(SHIFTER_CLOCK)
    _pulse(SHIFTER_LATCH)

try:
    print("Sweeping QA..QH (LSB->MSB). Watch LEDs/inputs on your driver board.")
    while True:
        for i in range(8):
            shift_out_byte(1<<i)
            time.sleep(0.25)
        # all off
        shift_out_byte(0x00)
        time.sleep(0.25)
except KeyboardInterrupt:
    shift_out_byte(0x00)
    try: GPIO.cleanup()
    except: pass
