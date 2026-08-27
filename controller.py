#!/usr/bin/env python3
"""H-infinity balance controller for the two-wheeled robot.

IMU setup matches gyro.py: no live calibration here. Interactive gyro and
figure-eight magnetometer calibration is calibrate.py, which writes
calib.json.

Loading:
  - Default: load calib.json if it exists; otherwise IMU default scale/bias.
  - --load-calib [PATH]: load that file; error if missing.
  - --no-calib: never load a file.

imusensor Madgwick(b=...) currently ignores `b` and uses beta ~0.60 from
GyroMeasError. The 0.8 argument is kept for compatibility and does not
change the filter gain.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
from scipy.io import loadmat
from control import ss, c2d

import gyro

Ts = 0.01  # Sampling time (s)
MAX_U = 2.65  # stall torque Nm
WARM_UP_SECONDS = 3.0
CONTROLLER_MAT = Path("hInfSynController.mat")
MADGWICK_B = 0.8  # constructor arg only; imusensor does not apply this to beta

rightMotor = None
leftMotor = None
rightMotorpwm = None
leftMotorpwm = None
rightEncoder = None


class Encoder:
    def __init__(self, pinNumberA, pinNumberB):
        from gpiozero import DigitalInputDevice

        self.counter = 0
        self.encoderInputA = DigitalInputDevice(pinNumberA)
        self.encoderInputA.when_activated = self.cycleCount
        self.encoderInputB = DigitalInputDevice(pinNumberB)
        self.direction = None

    def cycleCount(self):
        self.counter += 1
        if self.encoderInputB.value == 0:
            self.direction = "backward"
        else:
            self.direction = "forward"
        if self.counter >= 1120:
            print("One cycle completed")
            self.counter = 0
            print("I am moving ", self.direction)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Balance controller. Loads calib.json like gyro.py; "
            "does not run live IMU calibration."
        )
    )
    parser.add_argument(
        "--load-calib",
        nargs="?",
        const=str(gyro.DEFAULT_CALIB),
        default=None,
        metavar="PATH",
        help=(
            "Load a calib.json. If PATH is omitted, use the project file. "
            "Fails if the file is missing. Without this flag, the project "
            "file is still loaded when it exists."
        ),
    )
    parser.add_argument(
        "--calib",
        type=Path,
        default=None,
        help="Same as --load-calib PATH.",
    )
    parser.add_argument(
        "--no-calib",
        action="store_true",
        help="Do not load calib.json; use IMU default scale/bias.",
    )
    parser.add_argument("--bus", type=int, default=gyro.I2C_BUS)
    parser.add_argument(
        "--address", type=lambda s: int(s, 0), default=gyro.IMU_ADDRESS
    )
    parser.add_argument(
        "--mat",
        type=Path,
        default=CONTROLLER_MAT,
        help="Discrete H-infinity controller .mat (KA, KB, KC, KD).",
    )
    return parser.parse_args(argv)


def setup_imu(
    calib: Path | None,
    *,
    require_calib: bool = False,
    bus_id: int = gyro.I2C_BUS,
    address: int = gyro.IMU_ADDRESS,
    imu=None,
):
    """Open MPU9250 without live calib, optionally load calib.json."""
    return gyro.open_imu(
        bus_id=bus_id,
        address=address,
        calib=calib,
        imu=imu,
        require_calib=require_calib,
    )


def setup_fusion():
    """Madgwick filter for tilt (theta) and yaw (psi)."""
    return gyro.make_fusion(MADGWICK_B)


def load_hinf(mat_path: Path = CONTROLLER_MAT, ts: float = Ts):
    """Load continuous H-infinity matrices and discretize with Tustin."""
    data = loadmat(str(mat_path))
    KA = np.squeeze(data["KA"])
    KB = np.squeeze(data["KB"])
    KC = np.squeeze(data["KC"])
    KD = np.squeeze(data["KD"])
    K = ss(KA, KB, KC, KD)
    Kd = c2d(K, ts, method="tustin")
    return Kd.A, Kd.B, Kd.C, Kd.D


def setup_motors():
    """GPIO motors and one wheel encoder. Called only from main()."""
    global rightMotor, leftMotor, rightMotorpwm, leftMotorpwm, rightEncoder
    from gpiozero import Motor, PWMOutputDevice

    rightMotor = Motor(13, 6)
    leftMotor = Motor(19, 26)
    rightMotorpwm = PWMOutputDevice(20, frequency=1000)  # 20000 cap
    leftMotorpwm = PWMOutputDevice(21, frequency=1000)
    rightEncoder = Encoder(14, 15)


def apply_control(u):
    # u is (2,1) array: u[0] for common mode (forward/tilt control), u[1] for diff mode (yaw control)
    # Scale u to motor speeds: Assume u in [-max_u, max_u] maps to [-1,1] speed
    print("raw u", u)
    u_scaled = u / MAX_U
    print("scaled u", u_scaled)
    left_speed = np.clip(u_scaled[0] + u_scaled[1], -1.0, 1.0).item()  # Common + diff
    right_speed = np.clip(u_scaled[0] - u_scaled[1], -1.0, 1.0).item()  # Common - diff (adjust sign if yaw direction wrong)

    # Set directions and PWM values, COMMENT THIS OUT FOR TEST
    if left_speed > 0:
        leftMotor.forward()
        leftMotorpwm.value = left_speed
    elif left_speed < 0:
        leftMotor.backward()
        leftMotorpwm.value = -left_speed  # abs for PWM
    else:
        leftMotor.stop()
        leftMotorpwm.value = 0.0

    if right_speed > 0:
        rightMotor.forward()
        rightMotorpwm.value = right_speed
    elif right_speed < 0:
        rightMotor.backward()
        rightMotorpwm.value = -right_speed  # abs for PWM
    else:
        rightMotor.stop()
        rightMotorpwm.value = 0.0

    print(f"Control: left_speed={left_speed:.2f}, right_speed={right_speed:.2f}")
    return left_speed, right_speed


def warm_up(imu, sensorfusion, ts: float = Ts, seconds: float = WARM_UP_SECONDS):
    """Prime Madgwick while the robot is still. Does not calibrate the IMU."""
    print("Warming up sensor fusion (keep robot still)...")
    warm_up_steps = int(seconds / ts)
    pitch_samples = []
    currTime = time.time()
    for _ in range(warm_up_steps):
        newTime = time.time()
        dt = newTime - currTime
        currTime = newTime
        sample = gyro.fuse_sample(imu, sensorfusion, dt)
        pitch_samples.append(sample["pitch"])
        time.sleep(ts)
    print("Warm-up complete")
    return pitch_samples


def control_step(sample, yaw_initial, r, Ad, Bd, Cd, Dd, x_k, first_run):
    """One balance iteration from a fused IMU sample. Returns updated state."""
    pitch = sample["pitch"]  # Theta (tilt), degrees
    yaw = sample["yaw"]
    if first_run:
        yaw_initial = yaw
        first_run = False
    yaw_relative = yaw - yaw_initial  # Psi (relative yaw)
    print(f"Pitch: {pitch:.2f}, Yaw: {yaw_relative:.2f}")

    theta_rad = np.deg2rad(pitch)
    psi_rad = np.deg2rad(yaw_relative)
    y = np.array([[theta_rad], [psi_rad]])
    e = r - y
    u = Cd @ x_k + Dd @ e
    x_k = Ad @ x_k + Bd @ e
    return u, x_k, yaw_initial, first_run, yaw_relative


def stop_motors():
    if rightMotor is not None:
        rightMotor.stop()
    if leftMotor is not None:
        leftMotor.stop()
    if rightMotorpwm is not None:
        rightMotorpwm.value = 0.0
    if leftMotorpwm is not None:
        leftMotorpwm.value = 0.0


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    path, required = gyro.resolve_calib_args(args)
    imu = setup_imu(
        path,
        require_calib=required,
        bus_id=args.bus,
        address=args.address,
    )
    sensorfusion = setup_fusion()

    Ad, Bd, Cd, Dd = load_hinf(args.mat, Ts)
    n_states = Ad.shape[0]
    x_k = np.zeros((n_states, 1))
    yaw_initial = 0.0
    r = np.array([[0.0], [0.0]])  # theta=0 (upright), psi=0 (desired yaw)

    setup_motors()
    warm_up(imu, sensorfusion, Ts, WARM_UP_SECONDS)

    currTime = time.time()
    first_run = True
    try:
        while True:
            start_time = time.time()
            newTime = time.time()
            dt = newTime - currTime
            currTime = newTime
            sample = gyro.fuse_sample(imu, sensorfusion, dt)
            u, x_k, yaw_initial, first_run, _yaw_relative = control_step(
                sample, yaw_initial, r, Ad, Bd, Cd, Dd, x_k, first_run
            )
            apply_control(u)
            elapsed = time.time() - start_time
            if elapsed < Ts:
                time.sleep(Ts - elapsed)
    except KeyboardInterrupt:
        print("Terminated")
        stop_motors()
        return 0


if __name__ == "__main__":
    sys.exit(main())
