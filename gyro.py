#!/usr/bin/env python3
"""Non-calibrating MPU9250 read/test for the two-wheeled balancer.

This script never runs live gyro, accel, or magnetometer calibration and
never asks for a figure-eight. Interactive calibration lives in
calibrate.py, which writes calib.json.

Loading:
  - Default: if calib.json exists (or GYRO_CALIB), load it; else IMU defaults.
  - --load-calib [PATH]: load that file; error if it is missing.
  - --no-calib: ignore any calib file and use IMU default scale/bias.

Note: imusensor's MPU9250.begin() normally calls caliberateGyro() at the
end (a still-bias estimate). This test mode replaces that with a no-op
before begin() so register setup still happens without a live calib pass.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Any, Protocol

I2C_BUS = 1
IMU_ADDRESS = 0x68
SAMPLE_PERIOD_S = 0.01
# Passed to Madgwick(b=...). Current imusensor ignores `b` and sets
# beta from GyroMeasError = pi * (40/180), so beta is about 0.60.
MADGWICK_B = 0.5
DEFAULT_CALIB = Path("/home/steve/Desktop/twoWheeledRedemption/calib.json")


class IMULike(Protocol):
    AccelVals: Any
    GyroVals: Any
    MagVals: Any

    def begin(self) -> Any: ...
    def readSensor(self) -> Any: ...
    def loadCalibDataFromFile(self, path: str) -> Any: ...


class FusionLike(Protocol):
    roll: float
    pitch: float
    yaw: float

    def updateRollPitchYaw(
        self,
        ax: float,
        ay: float,
        az: float,
        gx: float,
        gy: float,
        gz: float,
        mx: float,
        my: float,
        mz: float,
        dt: float,
    ) -> Any: ...


def calib_path() -> Path:
    return Path(os.environ.get("GYRO_CALIB", str(DEFAULT_CALIB)))


def skip_live_calibration(imu: Any) -> None:
    """Disable imusensor live calib so begin() does not estimate gyro bias."""
    imu.caliberateGyro = lambda *args, **kwargs: None
    imu.caliberateMagApprox = lambda *args, **kwargs: None
    imu.caliberateMagPrecise = lambda *args, **kwargs: None
    imu.caliberateAccelerometer = lambda *args, **kwargs: None


def load_saved_calib(
    imu: Any,
    path: Path | None,
    *,
    required: bool = False,
    printer=print,
) -> bool:
    """Load a calib.json written by calibrate.py.

    If path is None, skip (IMU defaults). If the file is missing and
    required is False, skip. If required is True, raise FileNotFoundError.
    """
    if path is None:
        printer("No calib path given; using IMU default scale/bias.")
        return False
    if not path.is_file():
        msg = f"No calib file at {path}; using IMU default scale/bias."
        if required:
            raise FileNotFoundError(
                f"No calib file at {path}. Run: python calibrate.py"
            )
        printer(msg)
        return False
    imu.loadCalibDataFromFile(str(path))
    printer(f"Loaded saved calib from {path}")
    return True


def make_fusion(beta_arg: float = MADGWICK_B):
    """Build a Madgwick filter.

    `beta_arg` is the constructor argument. imusensor currently comments
    out `self.beta = b`, so the live gain is the library default (~0.60).
    """
    from imusensor.filters import madgwick

    return madgwick.Madgwick(beta_arg)


def open_imu(
    bus: Any | None = None,
    address: int = IMU_ADDRESS,
    bus_id: int = I2C_BUS,
    calib: Path | None = DEFAULT_CALIB,
    imu: IMULike | None = None,
    *,
    require_calib: bool = False,
) -> IMULike:
    """Create and initialize the MPU9250 without live calibration.

    Live figure-eight / gyro calib is calibrate.py. This only loads a
    saved file when calib points at an existing json (or require_calib).
    """
    if imu is None:
        import smbus
        from imusensor.MPU9250 import MPU9250

        if bus is None:
            bus = smbus.SMBus(bus_id)
        imu = MPU9250.MPU9250(bus, address)
    skip_live_calibration(imu)
    imu.begin()
    load_saved_calib(imu, calib, required=require_calib)
    return imu


def fuse_sample(imu: IMULike, fusion: FusionLike, dt: float) -> dict[str, Any]:
    """Read the IMU once and apply one Madgwick update.

    dt is the time since the previous sample in seconds. A single update
    is used (the old test loop applied the same reading ten times).
    """
    if dt <= 0:
        dt = SAMPLE_PERIOD_S
    imu.readSensor()
    fusion.updateRollPitchYaw(
        imu.AccelVals[0],
        imu.AccelVals[1],
        imu.AccelVals[2],
        imu.GyroVals[0],
        imu.GyroVals[1],
        imu.GyroVals[2],
        imu.MagVals[0],
        imu.MagVals[1],
        imu.MagVals[2],
        dt,
    )
    return {
        "accel": tuple(float(v) for v in imu.AccelVals),
        "gyro": tuple(float(v) for v in imu.GyroVals),
        "mag": tuple(float(v) for v in imu.MagVals),
        "roll": float(fusion.roll),
        "pitch": float(fusion.pitch),
        "yaw": float(fusion.yaw),
        "dt": float(dt),
    }


def format_sample(sample: dict[str, Any]) -> str:
    return (
        f"Roll: {sample['roll']:.2f}  Pitch: {sample['pitch']:.2f}  "
        f"Yaw: {sample['yaw']:.2f}"
    )


def run_loop(
    imu: IMULike,
    fusion: FusionLike,
    *,
    period: float = SAMPLE_PERIOD_S,
    samples: int = 0,
    printer=print,
) -> int:
    """Print fused angles. samples=0 runs until Ctrl-C."""
    printer("IMU test mode: no live calibration (use calibrate.py for that).")
    printer("Ctrl-C to stop.")
    count = 0
    now = time.time()
    try:
        while samples <= 0 or count < samples:
            start = time.time()
            new = time.time()
            dt = new - now
            now = new
            sample = fuse_sample(imu, fusion, dt)
            printer(format_sample(sample))
            count += 1
            elapsed = time.time() - start
            if elapsed < period:
                time.sleep(period - elapsed)
    except KeyboardInterrupt:
        printer("stopped")
    return count


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read MPU9250 + Madgwick without live calibration. "
            "Optionally load calib.json from calibrate.py."
        )
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=0,
        help="Number of prints then exit. 0 (default) runs until Ctrl-C.",
    )
    parser.add_argument(
        "--period",
        type=float,
        default=SAMPLE_PERIOD_S,
        help="Loop period in seconds (default 0.01).",
    )
    parser.add_argument(
        "--load-calib",
        nargs="?",
        const=str(DEFAULT_CALIB),
        default=None,
        metavar="PATH",
        help=(
            "Load a calib.json. If PATH is omitted, use the project file. "
            "Fails if the file is missing. "
            "Without this flag, the project file is still loaded when it exists."
        ),
    )
    parser.add_argument(
        "--calib",
        type=Path,
        default=None,
        help="Same as --load-calib PATH (kept for older command lines).",
    )
    parser.add_argument(
        "--no-calib",
        action="store_true",
        help="Do not load calib.json; use IMU default scale/bias.",
    )
    parser.add_argument("--bus", type=int, default=I2C_BUS)
    parser.add_argument("--address", type=lambda s: int(s, 0), default=IMU_ADDRESS)
    return parser.parse_args(argv)


def resolve_calib_args(args: argparse.Namespace) -> tuple[Path | None, bool]:
    """Return (path or None, required). None path means do not load."""
    if args.no_calib:
        return None, False
    if args.load_calib is not None:
        return Path(args.load_calib), True
    if args.calib is not None:
        return Path(args.calib), True
    return calib_path(), False


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    path, required = resolve_calib_args(args)
    imu = open_imu(
        bus_id=args.bus,
        address=args.address,
        calib=path,
        require_calib=required,
    )
    fusion = make_fusion()
    run_loop(imu, fusion, period=args.period, samples=args.samples)
    return 0


if __name__ == "__main__":
    sys.exit(main())
