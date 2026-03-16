"""Backward compatibility shim — old import paths are preserved.

Usage:
    from modules.swarm.swarm_coordinator import SwarmManager, TrackedTarget

This file redirects to the new modules coordinator.py and target.py.
"""

from modules.swarm.coordinator import SwarmManager
from modules.swarm.target import TrackedTarget

__all__ = ["SwarmManager", "TrackedTarget"]
