# turret_motors.py
#
# Wrapper around the working Stepper class from Lab8_4.py
# Provides a high-level TurretMotors class with separate
# pan (azimuth) and tilt (elevation) motors, plus gear ratios
# to account for belt/gear reductions.
#
# Pan and tilt angles passed into TurretMotors are in *turret* degrees.
# Internally we convert to stepper shaft degrees using:
#   stepper_angle = turret_angle * gear_ratio
#
# So if your belt drive is 2:1 (stepper turns 2x turret),
# set pan_gear_ratio = 2.0, etc.

import multiprocessing
from shifter import Shifter
from Lab8_4 import Stepper  # use your known-good Stepper class


class TurretMotors:
    def __init__(
        self,
        data_pin: int = 16,
        latch_pin: int = 20,
        clock_pin: int = 21,
        pan_gear_ratio: float = 1.0,
        tilt_gear_ratio: float = 1.0,
    ):
        """
        data_pin, latch_pin, clock_pin:
            GPIO pins for the SN74HC595 shift register.

        pan_gear_ratio:
            stepper_deg / turret_deg for the pan axis
        tilt_gear_ratio:
            stepper_deg / turret_deg for the tilt axis
        """
        self.shifter = Shifter(data=data_pin, latch=latch_pin, clock=clock_pin)
        self.lock = multiprocessing.Lock()

        # NOTE: Stepper.num_steppers controls which 4 bits each motor uses.
        # The first Stepper created uses bits 0–3, the second uses bits 4–7.
        # We'll treat the first as PAN (azimuth) and the second as TILT.
        self.pan = Stepper(self.shifter, self.lock)
        self.tilt = Stepper(self.shifter, self.lock)

        # Gear ratios (stepper_deg / turret_deg).
        # You will tune these later as you test the belt drive.
        self.pan_gear_ratio = pan_gear_ratio
        self.tilt_gear_ratio = tilt_gear_ratio

        # Start with logical 0° for both axes
        self.zero()

    # ---------- calibration / configuration ----------

    def set_gear_ratios(self, pan_ratio: float | None = None, tilt_ratio: float | None = None) -> None:
        """
        Update gear ratios. These are the values you'll eventually
        change from the web interface.
        """
        if pan_ratio is not None:
            self.pan_gear_ratio = pan_ratio
        if tilt_ratio is not None:
            self.tilt_gear_ratio = tilt_ratio

    def zero(self) -> None:
        """
        Set the current stepper angles to 0° (logical turret zero).
        This does NOT move the motors; it just resets the angle variables.
        """
        self.pan.zero()
        self.tilt.zero()

    # ---------- angle getters (handy for debugging) ----------

    @property
    def pan_stepper_angle(self) -> float:
        """Current pan stepper shaft angle in degrees."""
        return self.pan.angle.value

    @property
    def tilt_stepper_angle(self) -> float:
        """Current tilt stepper shaft angle in degrees."""
        return self.tilt.angle.value

    @property
    def pan_turret_angle(self) -> float:
        """Approximate turret pan angle in degrees (stepper angle / gear ratio)."""
        if self.pan_gear_ratio == 0:
            return 0.0
        return self.pan_stepper_angle / self.pan_gear_ratio

    @property
    def tilt_turret_angle(self) -> float:
        """Approximate turret tilt angle in degrees (stepper angle / gear ratio)."""
        if self.tilt_gear_ratio == 0:
            return 0.0
        return self.tilt_stepper_angle / self.tilt_gear_ratio

    # ---------- main motion API ----------

    def goto(self, pan_deg: float, tilt_deg: float, sync: bool = True):
        """
        Move both axes to the requested *turret* angles (in degrees).

        Internally converts to stepper shaft angles using gear ratios:
            stepper_target = turret_target * gear_ratio

        If sync=True (default), this call blocks until both moves finish.
        If sync=False, it returns the two Process objects.
        """
        pan_stepper_target = pan_deg * self.pan_gear_ratio
        tilt_stepper_target = tilt_deg * self.tilt_gear_ratio

        p_pan = self.pan.goAngle(pan_stepper_target)
        p_tilt = self.tilt.goAngle(tilt_stepper_target)

        if sync:
            p_pan.join()
            p_tilt.join()
            return None
        else:
            return p_pan, p_tilt

    def goto_pan(self, pan_deg: float, sync: bool = True):
        """
        Move only the pan axis to a given turret angle.
        """
        pan_stepper_target = pan_deg * self.pan_gear_ratio
        p_pan = self.pan.goAngle(pan_stepper_target)
        if sync:
            p_pan.join()
            return None
        else:
            return p_pan

    def goto_tilt(self, tilt_deg: float, sync: bool = True):
        """
        Move only the tilt axis to a given turret angle.
        """
        tilt_stepper_target = tilt_deg * self.tilt_gear_ratio
        p_tilt = self.tilt.goAngle(tilt_stepper_target)
        if sync:
            p_tilt.join()
            return None
        else:
            return p_tilt
