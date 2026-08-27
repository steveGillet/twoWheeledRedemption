#!/usr/bin/env python3
"""Unit tests for calibrate.py. No I2C hardware required."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import calibrate  # noqa: E402


class FakeCalibIMU:
    def __init__(self):
        self.begin_calls = 0
        self.caliberateGyro = MagicMock()
        self.caliberateMagApprox = MagicMock()
        self.saveCalibDataToFile = MagicMock()

    def begin(self):
        self.begin_calls += 1
        self.caliberateGyro()
        return 1


class TestCalibrate(unittest.TestCase):
    def test_open_runs_begin_including_library_gyro(self):
        imu = FakeCalibIMU()
        out = calibrate.open_imu_for_calibration(imu=imu)
        self.assertIs(out, imu)
        self.assertEqual(imu.begin_calls, 1)
        imu.caliberateGyro.assert_called()

    def test_run_calibration_saves_after_figure_eight(self):
        imu = FakeCalibIMU()
        prompts: list[str] = []
        logs: list[str] = []
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "calib.json"
            out = calibrate.run_calibration(
                imu,
                path,
                printer=logs.append,
                prompt=lambda m: prompts.append(m) or "",
                run_gyro=False,
                run_mag=True,
            )
            self.assertEqual(out, path)
        imu.caliberateMagApprox.assert_called_once()
        imu.saveCalibDataToFile.assert_called_once()
        saved = imu.saveCalibDataToFile.call_args[0][0]
        self.assertTrue(saved.endswith("calib.json"))
        self.assertTrue(any("figure-eight" in p.lower() for p in prompts))
        self.assertTrue(any("Saved calibration" in line for line in logs))

    def test_repeat_gyro_calls_caliberate_gyro(self):
        imu = FakeCalibIMU()
        imu.caliberateGyro.reset_mock()
        with TemporaryDirectory() as tmp:
            calibrate.run_calibration(
                imu,
                Path(tmp) / "calib.json",
                printer=lambda *_: None,
                prompt=lambda *_: "",
                run_gyro=True,
                run_mag=False,
            )
        imu.caliberateGyro.assert_called_once()
        imu.caliberateMagApprox.assert_not_called()

    def test_parse_output_and_yes(self):
        args = calibrate.parse_args(["--output", "/tmp/c.json", "--yes"])
        self.assertEqual(args.output, Path("/tmp/c.json"))
        self.assertTrue(args.yes)


if __name__ == "__main__":
    unittest.main()
