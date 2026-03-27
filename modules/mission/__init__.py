"""ORCUS mission exports."""

_EXPORTS = {
    "MissionController": ("modules.mission.mission_controller", "MissionController"),
    "FlightController": ("modules.mission.flight_controller", "FlightController"),
    "MotionAuthority": ("modules.mission.flight_controller", "MotionAuthority"),
    "MotionProfile": ("modules.mission.flight_controller", "MotionProfile"),
    "MotionCommand": ("modules.mission.flight_controller", "MotionCommand"),
    "Navigation": ("modules.mission.navigation", "Navigation"),
    "Scanner": ("modules.mission.navigation", "Scanner"),
    "AttackController": ("modules.mission.attack_controller", "AttackController"),
    "AttackFSM": ("modules.mission.attack_controller", "AttackFSM"),
    "LowPassFilter": ("modules.core.pid_controller", "LowPassFilter"),
    "VelocitySmoother": ("modules.core.pid_controller", "VelocitySmoother"),
    "FollowerLink": ("modules.mission.follower_link", "FollowerLink"),
    "MissionState": ("modules.core.logger", "MissionState"),
    "MissionPhase": ("modules.core.logger", "MissionPhase"),
}


def __getattr__(name):
    target = _EXPORTS.get(name)
    if target is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module_name, attr_name = target
    module = __import__(module_name, fromlist=[attr_name])
    return getattr(module, attr_name)


def __dir__():
    return sorted(list(globals().keys()) + list(_EXPORTS.keys()))


__all__ = list(_EXPORTS.keys())
