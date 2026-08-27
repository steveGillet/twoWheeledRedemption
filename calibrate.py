#!/usr/bin/env python3
"""Interactive MPU9250 calibration for the two-wheeled balancer.

This is the only script that runs live calibration. Keep the robot still
for gyro bias, then move it in a figure-eight for magnetometer hard/soft
iron. Results are written to calib.json.

gyro.py never calibrates. It only optionally loads this file for a
non-calibrating read/test loop.

imusensor's MPU9250.begin() also calls caliberateGyro() at the end. This
script prints a still prompt first, then begin(), then a separate
figure-eight magnetometer pass.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Callable

from gyro import DEFAULT_CALIB, I2C_BUS, IMU_ADDRESS, calib_path


def open_imu_for_calibration(
    bus: Any | None = None,
    address: int = IMU_ADDRESS,
    bus_id: int = I2C_BUS,
    imu: Any | None = None,
) -> Any:
    """Open the MPU9250 and run begin(), including the library gyro bias pass."""
    if imu is None:
        import smbus
        from imusensor.MPU9250 import MPU9250

        if bus is None:
            bus = smbus.SMBus(bus_id)
        imu = MPU9250.MPU9250(bus, address)
    imu.begin()
    return imu


def run_calibration(
    imu: Any,
    path: Path,
    *,
    printer: Callable[[str], None] = print,
    prompt: Callable[[str], str] | None = input,
    run_gyro: bool = False,
    run_mag: bool = True,
) -> Path:
    """Run interactive calib and save biases.

    begin() already ran caliberateGyro(). Set run_gyro=True to repeat it.
    run_mag=True runs caliberateMagApprox() (figure-eight, ~20 seconds).
    """
    path = Path(path)
    if run_gyro:
        printer("Keep the IMU still. Measuring gyro bias...")
        imu.caliberateGyro()
        printer("Gyro bias done.")
    else:
        printer("Gyro bias was measured during IMU startup (keep-still pass).")

    if run_mag:
        msg = (
            "Press Enter, then move the IMU in a figure-eight for about 20 seconds, "
            "covering pitch and roll."
        )
        if prompt is not None:
            prompt(msg)
        else:
            printer(msg)
        printer("Sampling magnetometer...")
        imu.caliberateMagApprox()
        printer("Magnetometer figure-eight pass done.")

    path.parent.mkdir(parents=True, exist_ok=True)
    imu.saveCalibDataToFile(str(path))
    printer(f"Saved calibration to {path}")
    return path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Interactive gyro + magnetometer calibration. "
            "Writes calib.json for gyro.py --load-calib."
        )
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Where to write calib.json (default: project calib.json or GYRO_CALIB).",
    )
    parser.add_argument("--bus", type=int, default=I2C_BUS)
    parser.add_argument("--address", type=lambda s: int(s, 0), default=IMU_ADDRESS)
    parser.add_argument(
        "--repeat-gyro",
        action="store_true",
        help="Run caliberateGyro() again after begin() (second still pass).",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Do not wait for Enter before the figure-eight mag pass.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    path = args.output or calib_path()
    print("Calibration script (not the IMU test).")
    print("Keep the robot still. Starting IMU and gyro bias...")
    imu = open_imu_for_calibration(bus_id=args.bus, address=args.address)
    prompt = None if args.yes else input
    run_calibration(
        imu,
        path,
        run_gyro=args.repeat_gyro,
        run_mag=True,
        prompt=prompt,
    )
    print("Done. Use: python gyro.py --load-calib")
    return 0


if __name__ == "__main__":
    sys.exit(main())
