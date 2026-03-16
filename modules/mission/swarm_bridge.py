"""Swarm communication layer — Secure, normalized messaging with Swarm Manager.

Always returns SwarmResponse; tuple/dict/str distinction is made here,
upper layer never deals with raw parsing.
"""

import time
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any

from modules.core.logger import SwarmLogger
from modules.core.geo_math import GeoMath
from config import SWARM_GEO_VALIDATION_THRESHOLD_M


# ======================================================================
# Normalized response
# ======================================================================

@dataclass
class SwarmResponse:
    """Normalized form of response from swarm manager."""
    action: str = "SEARCH"
    target_id: Optional[str] = None
    tracker_id: Optional[int] = None
    local_id: Optional[int] = None  # Drone's local ID for this target
    lat: Optional[float] = None
    lon: Optional[float] = None
    extra: Dict[str, Any] = field(default_factory=dict)


def parse_response(raw) -> SwarmResponse:
    """Convert any swarm response (dict, tuple, str, None) to SwarmResponse."""
    if raw is None:
        return SwarmResponse()

    if isinstance(raw, dict):
        return SwarmResponse(
            action=raw.get("action", "SEARCH"),
            target_id=raw.get("target_id"),
            tracker_id=raw.get("tracker_id"),
            local_id=raw.get("local_id"),
            lat=raw.get("lat"),
            lon=raw.get("lon"),
            extra={k: v for k, v in raw.items()
                   if k not in ("action", "target_id", "tracker_id", "local_id", "lat", "lon")},
        )

    if isinstance(raw, (tuple, list)):
        n = len(raw)
        return SwarmResponse(
            action=raw[0] if n > 0 and isinstance(raw[0], str) else "SEARCH",
            target_id=raw[1] if n > 1 else None,
            tracker_id=raw[2] if n > 2 else None,
            lat=raw[3] if n > 3 else None,
            lon=raw[4] if n > 4 else None,
        )

    if isinstance(raw, str):
        return SwarmResponse(action=raw)

    return SwarmResponse()


# ======================================================================
# Swarm Bridge
# ======================================================================

class SwarmBridge:
    """Manages all communication with Swarm Manager.

    All message sending and response parsing goes through this layer;
    upper layer only works with SwarmResponse.
    """

    def __init__(self, swarm_manager, drone_manager, port: int):
        self._sm = swarm_manager
        self._dm = drone_manager
        self.port = port

    # ------------------------------------------------------------------
    # Basic sending
    # ------------------------------------------------------------------

    def send_mission_packet(self, packet: dict) -> SwarmResponse:
        """Send packet, normalize response."""
        if not self._sm:
            return SwarmResponse()
        try:
            raw = self._sm.check_mission_updates(self.port, packet)
            return parse_response(raw)
        except Exception as e:
            SwarmLogger.log("ERROR", f"DRONE_{self.port}",
                            f"SwarmBridge send error: {e}", "BRIDGE")
            return SwarmResponse()

    def send_action(self, action: str, target_id: Optional[str] = None,
                    **kwargs) -> SwarmResponse:
        """Send simple action packet."""
        pkt = {"action": action}
        if target_id is not None:
            pkt["target_id"] = target_id
        pkt.update(kwargs)
        return self.send_mission_packet(pkt)

    # ------------------------------------------------------------------
    # Special packets
    # ------------------------------------------------------------------

    def send_heartbeat(self, target_id: str, tracker_id: Optional[int],
                       drone_gps: tuple) -> SwarmResponse:
        """Send HEARTBEAT."""
        return self.send_action(
            "HEARTBEAT", target_id=target_id,
            local_tracker_id=tracker_id,
            drone_gps=drone_gps,
        )

    def send_echo(self, target_id: str, drone_gps: tuple,
                  tracker_id: Optional[int] = None,
                  bbox=None) -> SwarmResponse:
        """Send ECHO_TARGET."""
        return self.send_action(
            "ECHO_TARGET", target_id=target_id,
            drone_gps=drone_gps,
            local_tracker_id=tracker_id,
            bbox=bbox,
        )

    def send_locked_data(self, target_id: str, drone_gps: tuple,
                         target_gps: Optional[tuple] = None,
                         bbox=None) -> SwarmResponse:
        """Send LOCKED_DATA (during CENTERING phase)."""
        return self.send_action(
            "LOCKED_DATA", target_id=target_id,
            drone_gps=drone_gps,
            target_gps=target_gps,
            bbox=bbox,
        )

    def send_attacking(self, target_id: str) -> SwarmResponse:
        """Send ATTACKING notification."""
        return self.send_action("ATTACKING", target_id=target_id)

    def send_impact(self, target_id: str, coverage: float,
                    altitude: float) -> SwarmResponse:
        """Send IMPACT_CONFIRMED."""
        return self.send_action(
            "IMPACT_CONFIRMED", target_id=target_id,
            coverage=coverage, altitude=altitude,
        )

    def send_mismatch(self, target_id: str) -> SwarmResponse:
        """Send GEO MISMATCH."""
        return self.send_action("MISMATCH", target_id=target_id)

    def send_detection_packet(self, detected_targets: list,
                              locked_target_id: Optional[str] = None,
                              state_label: str = "") -> SwarmResponse:
        """Send standard detection packet."""
        packet = {
            "drone_id": self.port,
            "timestamp": time.time(),
            "locked_target_id": locked_target_id,
            "detected_targets": detected_targets,
            "state": state_label,
        }
        return self.send_mission_packet(packet)

    # ------------------------------------------------------------------
    # Geo-validation
    # ------------------------------------------------------------------

    def geo_validate(self, resp: SwarmResponse,
                     reports: list, threshold_m: float = None) -> SwarmResponse:
        """Compare GPS in swarm response with local detection.

        If threshold is exceeded, send MISMATCH and modify response.
        """
        threshold = threshold_m or SWARM_GEO_VALIDATION_THRESHOLD_M

        if resp.action not in ("ATTACK", "CENTER", "ATTACK_ASSIGN",
                                "REASSIGN", "ATTACK_CONFIRMED"):
            return resp

        if resp.lat is None or resp.lon is None:
            return resp

        # Find matching local detection.
        # For group targets, representative tracker_id can change; actionable local_id and
        # member_ids are more reliable. Last resort: pick geographically closest report.
        matched = None
        candidate_ids = []
        if resp.local_id is not None:
            candidate_ids.append(resp.local_id)
        if resp.tracker_id is not None and resp.tracker_id not in candidate_ids:
            candidate_ids.append(resp.tracker_id)

        for candidate_id in candidate_ids:
            matched = next((r for r in reports if r.get("track_id") == candidate_id), None)
            if matched:
                break

        if matched is None and resp.local_id is not None:
            matched = next(
                (r for r in reports if resp.local_id in set(r.get("member_ids") or [])),
                None,
            )

        if matched is None and reports:
            def _geo_cost(report):
                world_xyz = report.get("world_xyz", (None, None, None))
                if world_xyz[0] is None or world_xyz[1] is None:
                    return float("inf")
                return GeoMath.haversine_distance(world_xyz[0], world_xyz[1], resp.lat, resp.lon)

            matched = min(reports, key=_geo_cost)
            if _geo_cost(matched) == float("inf"):
                matched = None

        if not matched:
            return resp

        world_xyz = matched.get("world_xyz", (None, None, None))
        t_lat, t_lon = world_xyz[0], world_xyz[1]
        if t_lat is None or t_lon is None:
            return resp

        dist = GeoMath.haversine_distance(t_lat, t_lon, resp.lat, resp.lon)
        if dist <= threshold:
            return resp

        SwarmLogger.log("MISMATCH", f"DRONE_{self.port}",
                        f"Geo-Validation FAILED! Target {resp.target_id} is {dist:.1f}m away. Dropping lock.")
        self.send_mismatch(resp.target_id)
        return SwarmResponse(action="MISMATCH", target_id=resp.target_id)

    # ------------------------------------------------------------------
    # report_target wrapper (check_human_detection compatibility)
    # ------------------------------------------------------------------

    def report_target(self, lat, lon, conf, tracker_id=None,
                      raw_data=None, covariance=None) -> SwarmResponse:
        """swarm_manager.report_target wrapper."""
        if not self._sm:
            return SwarmResponse()
        try:
            raw = self._sm.report_target(
                self.port, lat, lon, conf,
                tracker_id=tracker_id, raw_data=raw_data,
                covariance=covariance,
            )
            return parse_response(raw)
        except Exception as e:
            import traceback
            SwarmLogger.log("ERROR", f"DRONE_{self.port}",
                            f"report_target error: {e}\n{traceback.format_exc()}", "BRIDGE")
            return SwarmResponse()
