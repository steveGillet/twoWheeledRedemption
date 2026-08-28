#!/usr/bin/env python3
"""Safely shut off both drive motors on the two-wheeled Raspberry Pi robot.

Standalone copy of the pin map and stop behavior from controller.py.
Does not import the balance controller.

BCM pins (gpiozero):
  right: Motor(13, 6), PWM 20
  left:  Motor(19, 26), PWM 21

On the Pi, run with the robot venv:

  ~/Desktop/twoWheeledRedemption/venv/bin/python3 stop_motors.py

If robot-controller.service is holding the pins, stop that first:

  sudo systemctl stop robot-controller.service
"""

from __future__ import annotations

import sys

# Same BCM mapping as controller.py setup_motors().
RIGHT_FORWARD = 13
RIGHT_BACKWARD = 6
LEFT_FORWARD = 19
LEFT_BACKWARD = 26
RIGHT_PWM = 20
LEFT_PWM = 21
PWM_HZ = 1000


def stop_all() -> None:
    from gpiozero import Motor, PWMOutputDevice

    right_motor = Motor(RIGHT_FORWARD, RIGHT_BACKWARD)
    left_motor = Motor(LEFT_FORWARD, LEFT_BACKWARD)
    right_pwm = PWMOutputDevice(RIGHT_PWM, frequency=PWM_HZ)
    left_pwm = PWMOutputDevice(LEFT_PWM, frequency=PWM_HZ)
    devices = (right_motor, left_motor, right_pwm, left_pwm)
    try:
        right_motor.stop()
        left_motor.stop()
        right_pwm.value = 0.0
        left_pwm.value = 0.0
        right_pwm.off()
        left_pwm.off()
    finally:
        for device in devices:
            device.close()


def main() -> int:
    try:
        stop_all()
    except Exception as exc:
        name = type(exc).__name__
        print(f"Failed to stop motors ({name}): {exc}", file=sys.stderr)
        if "pin" in str(exc).lower() or "busy" in str(exc).lower() or name.endswith("InUse"):
            print(
                "GPIO is likely held by robot-controller.service. "
                "Run: sudo systemctl stop robot-controller.service",
                file=sys.stderr,
            )
        return 1
    print("Motors stopped. Direction drivers off, PWM set to 0.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
