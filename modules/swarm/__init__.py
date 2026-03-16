"""ORCUS Swarm Package — lazy exported swarm-layer API."""

_EXPORTS = {
    "SwarmManager": ("modules.swarm.coordinator", "SwarmManager"),
    "TrackedTarget": ("modules.swarm.target", "TrackedTarget"),
    "TargetFusionManager": ("modules.swarm.target_fusion", "TargetFusionManager"),
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


__all__ = ["SwarmManager", "TrackedTarget", "TargetFusionManager"]
