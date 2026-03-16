"""ORCUS Vision Package - YOLO detection, BoT-SORT tracking, camera handling."""


def __getattr__(name):
    """Lazy import — YOLO/torch/rospy bağımlılığı olmadan modül erişimi."""
    if name == "HumanTracker":
        from modules.vision.detector import HumanTracker
        return HumanTracker
    if name == "CameraAIHandler":
        from modules.vision.camera_handler import CameraAIHandler
        return CameraAIHandler
    if name == "GroupClusterEngine":
        from modules.vision.group_tracker import GroupClusterEngine
        return GroupClusterEngine
    if name == "GroupResult":
        from modules.vision.group_tracker import GroupResult
        return GroupResult
    raise AttributeError(f"module 'modules.vision' has no attribute {name!r}")


__all__ = [
    'HumanTracker',
    'CameraAIHandler',
    'GroupClusterEngine',
    'GroupResult',
]
