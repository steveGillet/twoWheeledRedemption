#!/usr/bin/env python3
"""Unit tests for gyro.py. No I2C hardware required."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import gyro  # noqa: E402


class FakeIMU:
    def __init__(self):
        self.AccelVals = [0.0, 0.0, 9.81]
        self.GyroVals = [0.0, 0.0, 0.0]
        self.MagVals = [20.0, 0.0, 0.0]
        self.begin_calls = 0
        self.read_calls = 0
        self.loaded = None
        self.caliberateGyro = MagicMock(name="caliberateGyro")
        self.caliberateMagApprox = MagicMock(name="caliberateMagApprox")
        self.caliberateMagPrecise = MagicMock(name="caliberateMagPrecise")
        self.caliberateAccelerometer = MagicMock(name="caliberateAccelerometer")

    def begin(self):
        self.begin_calls += 1
        self.caliberateGyro()
        return 1

    def readSensor(self):
        self.read_calls += 1

    def loadCalibDataFromFile(self, path: str):
        self.loaded = path

    def saveCalibDataToFile(self, path: str):
        self.saved = path


class FakeFusion:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.updates = []

    def updateRollPitchYaw(self, ax, ay, az, gx, gy, gz, mx, my, mz, dt):
        self.updates.append((ax, ay, az, gx, gy, gz, mx, my, mz, dt))
        self.roll = 1.5
        self.pitch = -2.25
        self.yaw = 10.0


class TestSkipLiveCalibration(unittest.TestCase):
    def test_begin_does_not_run_library_gyro_calib(self):
        imu = FakeIMU()
        gyro_calib = imu.caliberateGyro
        mag_calib = imu.caliberateMagApprox
        accel_calib = imu.caliberateAccelerometer
        gyro.skip_live_calibration(imu)
        imu.begin()
        self.assertEqual(imu.begin_calls, 1)
        gyro_calib.assert_not_called()
        mag_calib.assert_not_called()
        accel_calib.assert_not_called()


class TestCalibLoad(unittest.TestCase):
    def test_missing_file_returns_false(self):
        imu = FakeIMU()
        ok = gyro.load_saved_calib(imu, Path("/no/such/calib.json"))
        self.assertFalse(ok)
        self.assertIsNone(imu.loaded)

    def test_none_path_skips(self):
        imu = FakeIMU()
        self.assertFalse(gyro.load_saved_calib(imu, None))
        self.assertIsNone(imu.loaded)

    def test_required_missing_file_raises(self):
        imu = FakeIMU()
        with self.assertRaises(FileNotFoundError):
            gyro.load_saved_calib(
                imu, Path("/no/such/calib.json"), required=True
            )
        self.assertIsNone(imu.loaded)

    def test_existing_json_is_applied(self):
        imu = FakeIMU()
        path = Path(self.id().replace(".", "_") + ".json")
        try:
            # file must exist; contents unused by FakeIMU
            path.write_text("{}", encoding="utf-8")
            ok = gyro.load_saved_calib(imu, path)
            self.assertTrue(ok)
            self.assertEqual(imu.loaded, str(path))
        finally:
            if path.exists():
                path.unlink()


class TestOpenImu(unittest.TestCase):
    def test_open_imu_inits_without_live_calib_and_loads_file(self, tmp=None):
        imu = FakeIMU()
        calib = Path("/tmp/gyro_test_calib.json")
        calib.write_text(json.dumps({"GyroBias": [0, 0, 0]}), encoding="utf-8")
        try:
            mag_calib = imu.caliberateMagApprox
            out = gyro.open_imu(imu=imu, calib=calib)
            self.assertIs(out, imu)
            self.assertEqual(imu.begin_calls, 1)
            self.assertEqual(imu.loaded, str(calib))
            mag_calib.assert_not_called()
        finally:
            calib.unlink(missing_ok=True)

    def test_open_imu_no_calib_file(self):
        imu = FakeIMU()
        out = gyro.open_imu(imu=imu, calib=Path("/tmp/does-not-exist-calib.json"))
        self.assertIs(out, imu)
        self.assertIsNone(imu.loaded)
        self.assertEqual(imu.begin_calls, 1)


class TestFuseSample(unittest.TestCase):
    def test_reads_once_and_updates_once(self):
        imu = FakeIMU()
        fusion = FakeFusion()
        sample = gyro.fuse_sample(imu, fusion, 0.01)
        self.assertEqual(imu.read_calls, 1)
        self.assertEqual(len(fusion.updates), 1)
        self.assertAlmostEqual(sample["dt"], 0.01)
        self.assertEqual(sample["roll"], 1.5)
        self.assertEqual(sample["pitch"], -2.25)
        self.assertEqual(sample["yaw"], 10.0)
        self.assertEqual(sample["accel"], (0.0, 0.0, 9.81))

    def test_nonpositive_dt_falls_back_to_sample_period(self):
        imu = FakeIMU()
        fusion = FakeFusion()
        gyro.fuse_sample(imu, fusion, 0.0)
        self.assertAlmostEqual(fusion.updates[0][-1], gyro.SAMPLE_PERIOD_S)
        gyro.fuse_sample(imu, fusion, -1.0)
        self.assertAlmostEqual(fusion.updates[1][-1], gyro.SAMPLE_PERIOD_S)

    def test_format_sample_has_angles(self):
        text = gyro.format_sample(
            {"roll": 1.234, "pitch": -5.6, "yaw": 90.0}
        )
        self.assertIn("Roll: 1.23", text)
        self.assertIn("Pitch: -5.60", text)
        self.assertIn("Yaw: 90.00", text)


class TestRunLoop(unittest.TestCase):
    def test_finite_samples(self):
        imu = FakeIMU()
        fusion = FakeFusion()
        lines: list[str] = []
        n = gyro.run_loop(
            imu, fusion, period=0.0, samples=3, printer=lines.append
        )
        self.assertEqual(n, 3)
        self.assertEqual(imu.read_calls, 3)
        self.assertTrue(any("IMU test mode" in line for line in lines))
        self.assertEqual(sum("Roll:" in line for line in lines), 3)


class TestParseArgs(unittest.TestCase):
    def test_defaults(self):
        args = gyro.parse_args([])
        self.assertEqual(args.samples, 0)
        self.assertFalse(args.no_calib)

    def test_no_calib_and_samples(self):
        args = gyro.parse_args(["--no-calib", "--samples", "5"])
        self.assertTrue(args.no_calib)
        self.assertEqual(args.samples, 5)

    def test_load_calib_flag_without_path(self):
        args = gyro.parse_args(["--load-calib"])
        path, required = gyro.resolve_calib_args(args)
        self.assertTrue(required)
        self.assertEqual(path, gyro.DEFAULT_CALIB)

    def test_load_calib_flag_with_path(self):
        args = gyro.parse_args(["--load-calib", "/tmp/mine.json"])
        path, required = gyro.resolve_calib_args(args)
        self.assertTrue(required)
        self.assertEqual(path, Path("/tmp/mine.json"))

    def test_no_calib_wins(self):
        args = gyro.parse_args(["--no-calib", "--load-calib"])
        path, required = gyro.resolve_calib_args(args)
        self.assertIsNone(path)
        self.assertFalse(required)

    def test_default_is_optional_project_file(self):
        args = gyro.parse_args([])
        path, required = gyro.resolve_calib_args(args)
        self.assertEqual(path, gyro.calib_path())
        self.assertFalse(required)


class TestMadgwickLibrary(unittest.TestCase):
    """Exercise the real Madgwick class when imusensor is installed."""

    @classmethod
    def setUpClass(cls):
        try:
            from imusensor.filters import madgwick
        except ImportError:
            raise unittest.SkipTest("imusensor not installed")
        cls.madgwick = madgwick

    def test_constructor_ignores_b_like_current_library(self):
        a = self.madgwick.Madgwick(0.1)
        b = self.madgwick.Madgwick(0.8)
        # Library comments out `self.beta = b`; both should share default beta.
        self.assertAlmostEqual(float(a.beta), float(b.beta), places=6)
        self.assertGreater(float(a.beta), 0.0)

    def test_level_accel_stays_near_zero_tilt(self):
        filt = self.madgwick.Madgwick()
        # Slightly off-axis mag avoids a quaternion-length 0 in this library.
        for _ in range(80):
            filt.updateRollPitchYaw(
                0.05, 0.05, 9.81,
                0.0, 0.0, 0.0,
                20.0, 1.0, 2.0,
                0.01,
            )
        self.assertEqual(filt.roll, filt.roll)
        self.assertLess(abs(filt.roll), 25.0)
        self.assertLess(abs(filt.pitch), 25.0)

    def test_plus_x_gravity_produces_nonzero_pitch(self):
        filt = self.madgwick.Madgwick()
        for _ in range(80):
            filt.updateRollPitchYaw(
                9.81, 0.05, 0.05,
                0.0, 0.0, 0.0,
                20.0, 1.0, 2.0,
                0.01,
            )
        self.assertGreater(abs(filt.pitch), abs(filt.roll))
        self.assertGreater(abs(filt.pitch), 20.0)

    def test_output_is_finite(self):
        filt = self.madgwick.Madgwick()
        filt.updateRollPitchYaw(0.05, 0.05, 9.81, 0, 0, 0, 20, 1, 2, 0.01)
        for value in (filt.roll, filt.pitch, filt.yaw):
            self.assertEqual(value, value)  # not NaN
            self.assertLess(abs(value), 180.1)


class TestMakeFusion(unittest.TestCase):
    def test_make_fusion_returns_object_with_update(self):
        try:
            fusion = gyro.make_fusion()
        except ImportError:
            self.skipTest("imusensor not installed")
        self.assertTrue(hasattr(fusion, "updateRollPitchYaw"))
        self.assertTrue(hasattr(fusion, "roll"))


if __name__ == "__main__":
    unittest.main()
