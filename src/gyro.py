#!/usr/bin/env python3
"""Non-calibrating MPU9250 read/test for the two-wheeled balancer.

This script never runs live gyro, accel, or magnetometer calibration and
never asks for a figure-eight. Interactive calibration lives in
calibrate.py, which writes calib.json.

Loading:
  - Default: if calib.json exists (or GYRO_CALIB), load it; else IMU defaults.
  - --load-calib [PATH]: load that file; error if it is missing.
  - --no-calib: ignore any calib file and use IMU default scale/bias.

Madgwick beta lives here (MADGWICK_B). imusensor ignores the constructor
argument, so make_fusion() assigns fusion.beta after construction.

Pitch zero: rest the chassis in the upright pose with wheels off, then
run --calibrate-pitch. That stores an offset subtracted from fused pitch.

Prints go through logger.log() and are off by default. Pass --verbose to
enable, optionally --print-period SEC to throttle.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Protocol

from logger import add_print_args, apply_print_args, log

I2C_BUS = 1
IMU_ADDRESS = 0x68
SAMPLE_PERIOD_S = 0.01
# Live Madgwick gain. Applied in make_fusion() because imusensor ignores b.
MADGWICK_B = 0.9
PROJECT_ROOT = Path(__file__).resolve().parent.parent
CONFIG_DIR = PROJECT_ROOT / "config"
DEFAULT_CALIB = CONFIG_DIR / "calib.json"
DEFAULT_PITCH_OFFSET = CONFIG_DIR / "pitch_offset.json"
PITCH_CALIB_SETTLE_S = 2.0
PITCH_CALIB_MEASURE_S = 3.0


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
    beta: float

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


def pitch_offset_path() -> Path:
    return Path(os.environ.get("GYRO_PITCH_OFFSET", str(DEFAULT_PITCH_OFFSET)))


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
    printer=log,
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


def load_pitch_offset(
    path: Path | None = None,
    *,
    printer=log,
) -> float:
    """Load stored pitch offset in degrees. Missing file means 0.0."""
    if path is None:
        path = pitch_offset_path()
    if not path.is_file():
        printer("No pitch offset file; using 0.0 deg.")
        return 0.0
    data = json.loads(path.read_text(encoding="utf-8"))
    offset = float(data.get("pitch_offset_deg", 0.0))
    printer(f"Loaded pitch offset {offset:.3f} deg from {path}")
    return offset


def save_pitch_offset(
    offset: float,
    path: Path | None = None,
    *,
    printer=log,
) -> Path:
    """Write pitch offset in degrees."""
    if path is None:
        path = pitch_offset_path()
    payload = {"pitch_offset_deg": float(offset)}
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    printer(f"Saved pitch offset {offset:.3f} deg to {path}")
    return path


def make_fusion(beta_arg: float = MADGWICK_B):
    """Build a Madgwick filter and apply MADGWICK_B.

    imusensor comments out `self.beta = b` in the constructor, so the
    live gain is set on the instance after creation.
    """
    from imusensor.filters import madgwick

    fusion = madgwick.Madgwick(beta_arg)
    fusion.beta = float(beta_arg)
    return fusion


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


def fuse_sample(
    imu: IMULike,
    fusion: FusionLike,
    dt: float,
    pitch_offset: float = 0.0,
) -> dict[str, Any]:
    """Read the IMU once and apply one Madgwick update.

    dt is the time since the previous sample in seconds. A single update
    is used (the old test loop applied the same reading ten times).
    pitch is fused pitch minus pitch_offset (degrees).
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
    raw_pitch = float(fusion.pitch)
    offset = float(pitch_offset)
    return {
        "accel": tuple(float(v) for v in imu.AccelVals),
        "gyro": tuple(float(v) for v in imu.GyroVals),
        "mag": tuple(float(v) for v in imu.MagVals),
        "roll": float(fusion.roll),
        "pitch_raw": raw_pitch,
        "pitch": raw_pitch - offset,
        "pitch_offset": offset,
        "yaw": float(fusion.yaw),
        "dt": float(dt),
    }


def format_sample(sample: dict[str, Any]) -> str:
    return (
        f"Roll: {sample['roll']:.2f}  Pitch: {sample['pitch']:.2f}  "
        f"Yaw: {sample['yaw']:.2f}"
    )


def calibrate_pitch_offset(
    imu: IMULike,
    fusion: FusionLike,
    *,
    settle_s: float = PITCH_CALIB_SETTLE_S,
    measure_s: float = PITCH_CALIB_MEASURE_S,
    period: float = SAMPLE_PERIOD_S,
    path: Path | None = None,
    printer=log,
) -> float:
    """Average fused pitch while still and store it as the zero offset.

    Rest the robot in the pose that should read as standing-straight
    (wheels off so it cannot roll). After this, that pose reports ~0.
    """
    printer(
        "Pitch calibration: rest the robot still in the upright/zero-tilt "
        "pose with wheels off so it cannot roll."
    )
    settle_n = max(1, int(settle_s / period))
    measure_n = max(1, int(measure_s / period))
    now = time.time()
    printer(f"Settling filter for {settle_s:.1f}s...")
    for _ in range(settle_n):
        start = time.time()
        new = time.time()
        dt = new - now
        now = new
        fuse_sample(imu, fusion, dt, pitch_offset=0.0)
        elapsed = time.time() - start
        if elapsed < period:
            time.sleep(period - elapsed)
    printer(f"Averaging pitch for {measure_s:.1f}s...")
    pitches: list[float] = []
    for _ in range(measure_n):
        start = time.time()
        new = time.time()
        dt = new - now
        now = new
        sample = fuse_sample(imu, fusion, dt, pitch_offset=0.0)
        pitches.append(float(sample["pitch_raw"]))
        elapsed = time.time() - start
        if elapsed < period:
            time.sleep(period - elapsed)
    offset = float(sum(pitches) / len(pitches))
    save_pitch_offset(offset, path=path, printer=printer)
    printer(
        f"Average pitch {offset:.3f} deg stored. "
        "This pose should now read near zero."
    )
    return offset


def run_loop(
    imu: IMULike,
    fusion: FusionLike,
    *,
    period: float = SAMPLE_PERIOD_S,
    samples: int = 0,
    pitch_offset: float = 0.0,
    printer=log,
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
            sample = fuse_sample(imu, fusion, dt, pitch_offset=pitch_offset)
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
    parser.add_argument(
        "--calibrate-pitch",
        action="store_true",
        help=(
            "Average fused pitch while still and save pitch_offset.json. "
            "Rest the robot in the upright pose with wheels off. No motors."
        ),
    )
    parser.add_argument(
        "--no-pitch-offset",
        action="store_true",
        help="Do not load or apply pitch_offset.json.",
    )
    parser.add_argument("--bus", type=int, default=I2C_BUS)
    parser.add_argument("--address", type=lambda s: int(s, 0), default=IMU_ADDRESS)
    add_print_args(parser)
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
    apply_print_args(args)
    path, required = resolve_calib_args(args)
    imu = open_imu(
        bus_id=args.bus,
        address=args.address,
        calib=path,
        require_calib=required,
    )
    fusion = make_fusion()
    if args.calibrate_pitch:
        calibrate_pitch_offset(imu, fusion, period=args.period)
        return 0
    offset = 0.0 if args.no_pitch_offset else load_pitch_offset()
    run_loop(
        imu,
        fusion,
        period=args.period,
        samples=args.samples,
        pitch_offset=offset,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
