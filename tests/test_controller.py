#!/usr/bin/env python3
"""Unit tests for controller.py. No GPIO or I2C required."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import gyro  # noqa: E402
import controller  # noqa: E402
from tests.test_gyro import FakeIMU, FakeFusion  # noqa: E402

MAT_PATH = ROOT / "hInfSynController.mat"


def _mock_motors():
    left = MagicMock(name="leftMotor")
    right = MagicMock(name="rightMotor")
    left_pwm = MagicMock(name="leftMotorpwm")
    right_pwm = MagicMock(name="rightMotorpwm")
    controller.leftMotor = left
    controller.rightMotor = right
    controller.leftMotorpwm = left_pwm
    controller.rightMotorpwm = right_pwm
    return left, right, left_pwm, right_pwm


class TestControllerCalibFlags(unittest.TestCase):
    def test_default_optional_project_file(self):
        args = controller.parse_args([])
        path, required = gyro.resolve_calib_args(args)
        self.assertEqual(path, gyro.calib_path())
        self.assertFalse(required)

    def test_load_calib_flag_requires_file(self):
        args = controller.parse_args(["--load-calib"])
        path, required = gyro.resolve_calib_args(args)
        self.assertTrue(required)
        self.assertEqual(path, gyro.DEFAULT_CALIB)

    def test_load_calib_custom_path(self):
        args = controller.parse_args(["--load-calib", "/tmp/ctrl-calib.json"])
        path, required = gyro.resolve_calib_args(args)
        self.assertTrue(required)
        self.assertEqual(path, Path("/tmp/ctrl-calib.json"))

    def test_no_calib(self):
        args = controller.parse_args(["--no-calib"])
        path, required = gyro.resolve_calib_args(args)
        self.assertIsNone(path)
        self.assertFalse(required)

    def test_no_calib_wins_over_load(self):
        args = controller.parse_args(["--no-calib", "--load-calib"])
        path, required = gyro.resolve_calib_args(args)
        self.assertIsNone(path)
        self.assertFalse(required)


class TestControllerSetupImu(unittest.TestCase):
    def test_loads_when_file_present(self):
        imu = FakeIMU()
        calib = Path("/tmp/controller_test_calib.json")
        calib.write_text(json.dumps({"GyroBias": [0, 0, 0]}), encoding="utf-8")
        try:
            out = controller.setup_imu(calib, require_calib=True, imu=imu)
            self.assertIs(out, imu)
            self.assertEqual(imu.loaded, str(calib))
            self.assertEqual(imu.begin_calls, 1)
        finally:
            calib.unlink(missing_ok=True)

    def test_defaults_when_file_missing_and_not_required(self):
        imu = FakeIMU()
        out = controller.setup_imu(
            Path("/tmp/controller-missing-calib.json"),
            require_calib=False,
            imu=imu,
        )
        self.assertIs(out, imu)
        self.assertIsNone(imu.loaded)

    def test_required_missing_raises(self):
        imu = FakeIMU()
        with self.assertRaises(FileNotFoundError):
            controller.setup_imu(
                Path("/tmp/controller-missing-calib.json"),
                require_calib=True,
                imu=imu,
            )

    def test_no_path_uses_defaults(self):
        imu = FakeIMU()
        out = controller.setup_imu(None, require_calib=False, imu=imu)
        self.assertIs(out, imu)
        self.assertIsNone(imu.loaded)
        self.assertEqual(imu.begin_calls, 1)


class TestLoadHinf(unittest.TestCase):
    def test_load_hinf_shapes_and_tilt_u(self):
        if not MAT_PATH.is_file():
            self.skipTest(f"missing {MAT_PATH}")
        Ad, Bd, Cd, Dd = controller.load_hinf(MAT_PATH, ts=0.01)
        self.assertEqual(Ad.shape[0], Ad.shape[1])
        self.assertEqual(Ad.shape[0], Bd.shape[0])
        self.assertEqual(Cd.shape[0], 2)
        self.assertEqual(Dd.shape, (2, 2))
        self.assertEqual(Bd.shape[1], 2)
        self.assertEqual(Cd.shape[1], Ad.shape[0])

        x_k = np.zeros((Ad.shape[0], 1))
        r = np.array([[0.0], [0.0]])
        sample = {
            "pitch": 3.0,
            "yaw": 0.0,
            "roll": 0.0,
            "dt": 0.01,
            "accel": (0, 0, 9.81),
            "gyro": (0, 0, 0),
            "mag": (1, 0, 0),
        }
        u, x_next, _yaw0, first, yaw_rel = controller.control_step(
            sample, 0.0, r, Ad, Bd, Cd, Dd, x_k, True
        )
        self.assertFalse(first)
        self.assertAlmostEqual(yaw_rel, 0.0)
        theta = np.deg2rad(3.0)
        e = r - np.array([[theta], [0.0]])
        u_expected = Dd @ e
        np.testing.assert_allclose(u, u_expected, rtol=1e-9, atol=1e-9)
        self.assertGreater(abs(float(u[0, 0])), abs(float(u[1, 0])))
        self.assertEqual(x_next.shape, (Ad.shape[0], 1))


class TestApplyControl(unittest.TestCase):
    def setUp(self):
        self.left, self.right, self.left_pwm, self.right_pwm = _mock_motors()
        self.addCleanup(self._clear)

    def _clear(self):
        controller.leftMotor = None
        controller.rightMotor = None
        controller.leftMotorpwm = None
        controller.rightMotorpwm = None

    def test_forward_common_mode(self):
        u = np.array([[1.0], [0.0]])
        left_speed, right_speed = controller.apply_control(u)
        expected = 1.0 / controller.MAX_U
        self.assertAlmostEqual(left_speed, expected)
        self.assertAlmostEqual(right_speed, expected)
        self.left.forward.assert_called_once()
        self.right.forward.assert_called_once()
        self.left.backward.assert_not_called()
        self.assertAlmostEqual(self.left_pwm.value, expected)
        self.assertAlmostEqual(self.right_pwm.value, expected)

    def test_reverse_common_mode(self):
        u = np.array([[-1.0], [0.0]])
        left_speed, right_speed = controller.apply_control(u)
        expected = -1.0 / controller.MAX_U
        self.assertAlmostEqual(left_speed, expected)
        self.assertAlmostEqual(right_speed, expected)
        self.left.backward.assert_called_once()
        self.right.backward.assert_called_once()
        self.left.forward.assert_not_called()
        self.assertAlmostEqual(self.left_pwm.value, abs(expected))
        self.assertAlmostEqual(self.right_pwm.value, abs(expected))

    def test_mix_diff_mode(self):
        u = np.array([[0.0], [1.0]])
        left_speed, right_speed = controller.apply_control(u)
        mag = 1.0 / controller.MAX_U
        self.assertAlmostEqual(left_speed, mag)
        self.assertAlmostEqual(right_speed, -mag)
        self.left.forward.assert_called_once()
        self.right.backward.assert_called_once()
        self.assertAlmostEqual(self.left_pwm.value, mag)
        self.assertAlmostEqual(self.right_pwm.value, mag)

    def test_clips_past_stall_torque(self):
        u = np.array([[10.0], [0.0]])
        left_speed, right_speed = controller.apply_control(u)
        self.assertAlmostEqual(left_speed, 1.0)
        self.assertAlmostEqual(right_speed, 1.0)
        self.left.forward.assert_called()
        self.right.forward.assert_called()
        self.assertAlmostEqual(self.left_pwm.value, 1.0)
        self.assertAlmostEqual(self.right_pwm.value, 1.0)


class TestWarmUp(unittest.TestCase):
    def test_warm_up_short_duration_fake_imu(self):
        imu = FakeIMU()
        fusion = FakeFusion()
        with patch("controller.time.sleep", return_value=None):
            pitches = controller.warm_up(imu, fusion, ts=0.01, seconds=0.02)
        self.assertEqual(len(pitches), 2)
        self.assertEqual(imu.read_calls, 2)
        self.assertEqual(len(fusion.updates), 2)
        self.assertTrue(all(p == fusion.pitch for p in pitches))


class TestControlStep(unittest.TestCase):
    def test_first_run_zeros_relative_yaw(self):
        sample = {
            "pitch": 3.0,
            "yaw": 40.0,
            "roll": 0.0,
            "dt": 0.01,
            "accel": (0, 0, 9.81),
            "gyro": (0, 0, 0),
            "mag": (1, 0, 0),
        }
        Ad = np.array([[0.9]])
        Bd = np.array([[0.1, 0.0]])
        Cd = np.array([[1.0], [0.0]])
        Dd = np.array([[2.0, 0.0], [0.0, 1.0]])
        x_k = np.zeros((1, 1))
        r = np.array([[0.0], [0.0]])
        u, x_next, yaw0, first, yaw_rel = controller.control_step(
            sample, 0.0, r, Ad, Bd, Cd, Dd, x_k, True
        )
        self.assertFalse(first)
        self.assertAlmostEqual(yaw0, 40.0)
        self.assertAlmostEqual(yaw_rel, 0.0)
        self.assertEqual(u.shape[0], 2)
        self.assertEqual(x_next.shape, (1, 1))

    def test_yaw_difference_and_pitch_radians(self):
        sample = {
            "pitch": 3.0,
            "yaw": 50.0,
            "roll": 0.0,
            "dt": 0.01,
            "accel": (0, 0, 9.81),
            "gyro": (0, 0, 0),
            "mag": (1, 0, 0),
        }
        Ad = np.array([[1.0]])
        Bd = np.zeros((1, 2))
        Cd = np.zeros((2, 1))
        Dd = np.eye(2)
        x_k = np.zeros((1, 1))
        r = np.array([[0.0], [0.0]])
        u, _x, yaw0, first, yaw_rel = controller.control_step(
            sample, 40.0, r, Ad, Bd, Cd, Dd, x_k, False
        )
        self.assertTrue(first is False)
        self.assertAlmostEqual(yaw0, 40.0)
        self.assertAlmostEqual(yaw_rel, 10.0)
        self.assertAlmostEqual(float(u[0, 0]), -np.deg2rad(3.0), places=6)
        self.assertAlmostEqual(abs(float(u[0, 0])), 0.052359877, places=5)
        self.assertAlmostEqual(float(u[1, 0]), -np.deg2rad(10.0), places=6)


class TestStopMotors(unittest.TestCase):
    def test_stop_motors_on_mocks(self):
        left, right, left_pwm, right_pwm = _mock_motors()
        controller.stop_motors()
        left.stop.assert_called_once()
        right.stop.assert_called_once()
        self.assertEqual(left_pwm.value, 0.0)
        self.assertEqual(right_pwm.value, 0.0)
        controller.leftMotor = None
        controller.rightMotor = None
        controller.leftMotorpwm = None
        controller.rightMotorpwm = None

    def test_stop_motors_none_is_safe(self):
        controller.leftMotor = None
        controller.rightMotor = None
        controller.leftMotorpwm = None
        controller.rightMotorpwm = None
        controller.stop_motors()


if __name__ == "__main__":
    unittest.main()
