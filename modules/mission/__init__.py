"""ORCUS Mission Package - Flight control, navigation, scanning, tracking, IBVS guidance."""

# Lazy imports for test mocking
def __getattr__(name):
    if name == 'MissionController':
        from modules.mission.mission_controller import MissionController
        return MissionController
    if name == 'FlightController':
        from modules.mission.flight_controller import FlightController
        return FlightController
    if name == 'Navigation':
        from modules.mission.navigation import Navigation
        return Navigation
    if name == 'Scanner':
        from modules.mission.scanner import Scanner
        return Scanner
    if name == 'TrackingController':
        from modules.mission.tracking_controller import TrackingController
        return TrackingController
    if name == 'CollisionMissionController':
        from modules.mission.mission_controller import MissionController
        return MissionController
    if name == 'IBVSGuidance':
        from modules.mission.ibvs_guidance import IBVSGuidance
        return IBVSGuidance
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = [
    'MissionController',
    'CollisionMissionController',
    'FlightController',
    'Navigation',
    'Scanner',
    'TrackingController',
    'IBVSGuidance',
]
