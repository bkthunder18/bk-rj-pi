# test_turret_motors.py
#
# Simple test harness for TurretMotors.
# Moves through a few hard-coded turret angles so you can
# check motion smoothness and verify the gear ratios.

import time
from turret_motors import TurretMotors


def main():
    # You can tweak these as you learn the real belt ratio.
    # Example: if stepper turns 2° for every 1° of turret motion,
    # set pan_gear_ratio = 2.0, etc.
    turret = TurretMotors(
        data_pin=16,
        latch_pin=20,
        clock_pin=21,
        pan_gear_ratio=1.0,   # stepper_deg / turret_deg (pan)
        tilt_gear_ratio=1.0,  # stepper_deg / turret_deg (tilt)
    )

    print("Zeroing logical turret angles...")
    turret.zero()
    print(
        f"Initial stepper angles: pan={turret.pan_stepper_angle:.1f}°, "
        f"tilt={turret.tilt_stepper_angle:.1f}°"
    )

    try:
        # --- Move 1: small pan to the right ---
        print("\nMove 1: pan=+45°, tilt=0° (turret coordinates)")
        turret.goto(pan_deg=45.0, tilt_deg=0.0)
        print(
            f"After Move 1 (stepper): pan={turret.pan_stepper_angle:.1f}°, "
            f"tilt={turret.tilt_stepper_angle:.1f}°"
        )
        time.sleep(1.0)

        # --- Move 2: small pan to the left, slight tilt up ---
        print("\nMove 2: pan=-45°, tilt=10°")
        turret.goto(pan_deg=-45.0, tilt_deg=10.0)
        print(
            f"After Move 2 (stepper): pan={turret.pan_stepper_angle:.1f}°, "
            f"tilt={turret.tilt_stepper_angle:.1f}°"
        )
        time.sleep(1.0)

        # --- Move 3: center pan, tilt up more ---
        print("\nMove 3: pan=0°, tilt=25°")
        turret.goto(pan_deg=0.0, tilt_deg=25.0)
        print(
            f"After Move 3 (stepper): pan={turret.pan_stepper_angle:.1f}°, "
            f"tilt={turret.tilt_stepper_angle:.1f}°"
        )
        time.sleep(1.0)

        # --- Move 4: return to zero ---
        print("\nMove 4: pan=0°, tilt=0° (back to logical zero)")
        turret.goto(pan_deg=0.0, tilt_deg=0.0)
        print(
            f"After Move 4 (stepper): pan={turret.pan_stepper_angle:.1f}°, "
            f"tilt={turret.tilt_stepper_angle:.1f}°"
        )

        print("\nTest sequence complete.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    # No explicit GPIO cleanup here – that’s handled in your
    # underlying Stepper/Shifter code as needed.


if __name__ == "__main__":
    main()
