"""ORCUS swarm exports."""

_EXPORTS = {
    "SwarmManager": ("modules.swarm.coordinator", "SwarmManager"),
    "LeaderLink": ("modules.swarm.leader_link", "LeaderLink"),
    "TrackedTarget": ("modules.swarm.target", "TrackedTarget"),
    "TargetRegistry": ("modules.swarm.target", "TargetRegistry"),
    "TargetFusionManager": ("modules.swarm.target_fusion", "TargetFusionManager"),
    "FusionEngine": ("modules.swarm.target_fusion", "FusionEngine"),
    "OwnershipManager": ("modules.swarm.assignment", "OwnershipManager"),
    "AssignmentEngine": ("modules.swarm.assignment", "AssignmentEngine"),
    "BattlespaceView": ("modules.swarm.battlespace", "BattlespaceView"),
    "TargetLifecycle": ("modules.swarm.target", "TargetLifecycle"),
    "LocalIdentityIndex": ("modules.swarm.target", "LocalIdentityIndex"),
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
