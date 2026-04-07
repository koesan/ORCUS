"""Runtime timing helpers shared across modules."""

from __future__ import annotations

import time
from dataclasses import dataclass


@dataclass(frozen=True)
class RuntimeStamp:
    """Snapshot of wall and monotonic time for one observation."""

    wall_time: float
    monotonic_time: float


def now_stamp() -> RuntimeStamp:
    """Capture a consistent timing snapshot."""
    return RuntimeStamp(wall_time=time.time(), monotonic_time=time.monotonic())


def monotonic_now() -> float:
    """Single monotonic time source for local freshness checks."""
    return time.monotonic()
