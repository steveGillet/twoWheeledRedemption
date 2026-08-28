# twoWheeledRedemption

Self-balancing two-wheeled robot control on a Raspberry Pi (Ubuntu).

## Layout

- `src/` — Python modules (`gyro.py`, `controller.py`, `logger.py`, `calibrate.py`, `motorDriver.py`, `stop_motors.py`)
- `tests/` — unit tests (no I2C or GPIO required)
- `scripts/` — command-line entry points
- `config/` — IMU calib, pitch offset, controller `.mat` files, systemd unit
- `docs/` — notes and TODOs
- `venv/` — local virtualenv (not committed)

## Run

From the repo root, with `src` on `PYTHONPATH`:

```bash
cd ~/Desktop/twoWheeledRedemption
source venv/bin/activate
export PYTHONPATH=src

python scripts/run_controller.py
python scripts/run_gyro.py --verbose
python scripts/calibrate.py
python scripts/stop_motors.py
```

Prints are off by default. Pass `--verbose`, optionally `--print-period SEC`.

Pitch zero (wheels off, rest upright):

```bash
PYTHONPATH=src python scripts/run_gyro.py --calibrate-pitch
```

## Tests

```bash
cd ~/Desktop/twoWheeledRedemption
PYTHONPATH=src ./venv/bin/python3 -m unittest tests.test_gyro tests.test_controller tests.test_calibrate
```

## systemd

The installed unit is `/etc/systemd/system/robot-controller.service`.
After pulling layout changes, copy `config/robot-controller.service` into place
and run `sudo systemctl daemon-reload`. That needs a password on this Pi.

## Next

See `docs/TODO.md`: add pitch-rate and yaw-rate feedback in addition to pitch and yaw.
