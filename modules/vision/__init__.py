"""Vision package exports."""


def __getattr__(name):
    """Lazy import without heavy optional dependencies."""
    _mapping = {
        "HumanTracker": ("modules.vision.detector", "HumanTracker"),
        "CameraAIHandler": ("modules.vision.camera_handler", "CameraAIHandler"),
        "GroupClusterEngine": ("modules.vision.group_tracker", "GroupClusterEngine"),
        "GroupResult": ("modules.vision.group_tracker", "GroupResult"),
        "TargetGroup": ("modules.vision.group_tracker", "TargetGroup"),
        "DetectionProcessor": ("modules.vision.detection_processor", "DetectionProcessor"),
        "DetectionResult": ("modules.vision.detection_processor", "DetectionResult"),
    }
    if name in _mapping:
        mod_path, attr_name = _mapping[name]
        import importlib
        mod = importlib.import_module(mod_path)
        return getattr(mod, attr_name)
    raise AttributeError(f"module 'modules.vision' has no attribute {name!r}")


__all__ = [
    'HumanTracker',
    'CameraAIHandler',
    'GroupClusterEngine',
    'GroupResult',
    'DetectionProcessor',
    'DetectionResult',
    'TargetGroup',
]
