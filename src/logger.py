"""Shared print helper for the two-wheeled robot scripts.

Prints are off by default so the 100 Hz control loop is not stalled by I/O.
Pass --verbose to enable, optionally --print-period SEC to throttle.
"""

from __future__ import annotations

import argparse
import time

PRINTS_ENABLED = False
PRINT_PERIOD_S = 0.0
_last_print_mono = 0.0


def configure_prints(
    *, enabled: bool | None = None, period_s: float | None = None
) -> None:
    """Set print enable and optional minimum interval between prints."""
    global PRINTS_ENABLED, PRINT_PERIOD_S, _last_print_mono
    if enabled is not None:
        PRINTS_ENABLED = bool(enabled)
    if period_s is not None:
        PRINT_PERIOD_S = max(0.0, float(period_s))
    _last_print_mono = 0.0


def log(*args, **kwargs) -> None:
    """Universal print. No-op when disabled. Throttles if PRINT_PERIOD_S > 0."""
    global _last_print_mono
    if not PRINTS_ENABLED:
        return
    if PRINT_PERIOD_S > 0.0:
        now = time.monotonic()
        if (now - _last_print_mono) < PRINT_PERIOD_S:
            return
        _last_print_mono = now
    print(*args, **kwargs)


def add_print_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable prints (disabled by default).",
    )
    parser.add_argument(
        "--print-period",
        type=float,
        default=0.0,
        metavar="SEC",
        help="Minimum seconds between prints when verbose. 0 prints every call.",
    )


def apply_print_args(args: argparse.Namespace) -> None:
    configure_prints(
        enabled=bool(getattr(args, "verbose", False)),
        period_s=float(getattr(args, "print_period", 0.0) or 0.0),
    )
