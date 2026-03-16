"""ORCUS Mission Package — lazy exported mission-layer API."""

_EXPORTS = {
    "MissionController": ("modules.mission.mission_controller", "MissionController"),
    "CollisionMissionController": ("modules.mission.mission_controller", "MissionController"),
    "FlightController": ("modules.mission.flight_controller", "FlightController"),
    "Navigation": ("modules.mission.navigation", "Navigation"),
    "Scanner": ("modules.mission.scanner", "Scanner"),
    "TrackingController": ("modules.mission.tracking_controller", "TrackingController"),
    "IBVSGuidance": ("modules.mission.ibvs_guidance", "IBVSGuidance"),
    "AttackFSM": ("modules.mission.attack_fsm", "AttackFSM"),
    "SwarmBridge": ("modules.mission.swarm_bridge", "SwarmBridge"),
    "DetectionProcessor": ("modules.mission.detection_processor", "DetectionProcessor"),
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

__all__ = [
    'MissionController',
    'CollisionMissionController',
    'FlightController',
    'Navigation',
    'Scanner',
    'TrackingController',
    'IBVSGuidance',
    'AttackFSM',
    'SwarmBridge',
    'DetectionProcessor',
]
