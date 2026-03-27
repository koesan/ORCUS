"""ORCUS core exports."""


def __getattr__(name):
    """Lazy import to avoid heavyweight optional dependencies at package import time."""
    if name == "DroneManager":
        from modules.core.fleet_manager import DroneManager
        return DroneManager
    if name == "GeoMath":
        from modules.core.geo_math import GeoMath
        return GeoMath
    if name == "SwarmLogger":
        from modules.core.logger import SwarmLogger
        return SwarmLogger
    if name == "LinkDiscipline":
        from modules.core.comm import LinkDiscipline
        return LinkDiscipline
    if name == "PID":
        from modules.core.pid_controller import PID
        return PID
    if name == "FilteredPID":
        from modules.core.pid_controller import FilteredPID
        return FilteredPID
    raise AttributeError(f"module 'modules.core' has no attribute {name!r}")


__all__ = [
    'DroneManager',
    'GeoMath',
    'SwarmLogger',
    'LinkDiscipline',
    'PID',
    'FilteredPID',
]
