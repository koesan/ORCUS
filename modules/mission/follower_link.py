"""Drone-side link to the swarm leader."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from config import BRIDGE_LOG_INTERVAL_S
from modules.core.comm import LinkDiscipline
from modules.core.logger import SwarmLogger

_TELEMETRY_MIN_INTERVAL_MS = 120
_TELEMETRY_MAX_SILENCE_MS = 600


@dataclass
class SwarmResponse:
    action: str = "SEARCH"
    target_id: Optional[str] = None
    tracker_id: Optional[int] = None
    local_id: Optional[int] = None
    lat: Optional[float] = None
    lon: Optional[float] = None
    command_id: Optional[str] = None
    epoch: Optional[int] = None
    issued_at: Optional[float] = None
    stale_after_s: Optional[float] = None
    extra: Dict[str, Any] = field(default_factory=dict)


def parse_response(raw) -> SwarmResponse:
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
            command_id=raw.get("command_id"),
            epoch=raw.get("epoch"),
            issued_at=raw.get("issued_at"),
            stale_after_s=raw.get("stale_after_s"),
            extra={k: v for k, v in raw.items() if k not in {"action", "target_id", "tracker_id", "local_id", "lat", "lon", "command_id", "epoch", "issued_at", "stale_after_s"}},
        )
    if isinstance(raw, (tuple, list)):
        tracker_or_local = raw[2] if len(raw) > 2 else None
        return SwarmResponse(
            action=raw[0] if len(raw) > 0 and isinstance(raw[0], str) else "SEARCH",
            target_id=raw[1] if len(raw) > 1 else None,
            tracker_id=tracker_or_local,
            local_id=tracker_or_local,
            lat=raw[3] if len(raw) > 3 else None,
            lon=raw[4] if len(raw) > 4 else None,
        )
    if isinstance(raw, str):
        return SwarmResponse(action=raw)
    return SwarmResponse()


class FollowerLink:
    """Single drone-side communication surface to swarm leader.

    Drone publishes already grouped, locally merged, geolocated, and filtered
    target reports. Leader receives only the compact target summary it needs
    for cross-drone fusion, target creation, assignment, and commanding.
    """

    def __init__(self, swarm_manager, drone_manager, port: int):
        self._sm = swarm_manager
        self.port = port
        self._discipline = LinkDiscipline(node_id=f"drone:{port}")

    @staticmethod
    def _sanitize_detection(item: dict) -> dict:
        if not isinstance(item, dict):
            return {}
        allowed = {
            "track_id",
            "confidence",
            "world_xyz",
            "covariance",
            "observation_quality",
            "observed_at",
            "frame_seq",
            "is_edge",
            "is_group",
            "group_member_count",
            "group_id",
            "member_ids",
        }
        return {k: item[k] for k in allowed if k in item}

    @staticmethod
    def _dropped_detection_fields(item: dict) -> List[str]:
        if not isinstance(item, dict):
            return []
        allowed = {
            "track_id",
            "confidence",
            "world_xyz",
            "covariance",
            "observation_quality",
            "observed_at",
            "frame_seq",
            "is_edge",
            "is_group",
            "group_member_count",
            "group_id",
            "member_ids",
        }
        return sorted(str(k) for k in item.keys() if k not in allowed)

    def _log_sanitization(self, context: str, dropped_fields: List[str]) -> None:
        if not dropped_fields:
            return
        SwarmLogger.log_link_activity(
            module=f"DRONE_{self.port}",
            direction="SANITIZE",
            action=context,
            detail="dropped=" + ",".join(dropped_fields[:6]) + (",..." if len(dropped_fields) > 6 else ""),
            level="SECURITY",
            interval_s=max(5.0, BRIDGE_LOG_INTERVAL_S),
            key=f"sanitize:{self.port}:{context}",
        )

    def send_mission_packet(self, packet: dict) -> SwarmResponse:
        if not self._sm:
            return SwarmResponse()
        try:
            raw = self._sm.check_mission_updates(self.port, packet)
            resp = parse_response(raw)
            self._log_response(packet, resp)
            return resp
        except Exception as exc:
            SwarmLogger.log("ERROR", f"DRONE_{self.port}", f"FollowerLink send error: {exc}", "LINK")
            return SwarmResponse()

    def send_action(self, action: str, target_id: Optional[str] = None, **kwargs) -> SwarmResponse:
        packet = {"action": action}
        if target_id is not None:
            packet["target_id"] = target_id
        packet.update(kwargs)
        self._log_packet(packet)
        return self.send_mission_packet(packet)

    def send_detection_packet(self, detected_targets: list, locked_target_id: Optional[str] = None, state_label: str = "") -> SwarmResponse:
        sanitized_targets = []
        dropped = set()
        for item in (detected_targets or []):
            dropped.update(self._dropped_detection_fields(item))
            sanitized_targets.append(self._sanitize_detection(item))
        self._log_sanitization("DETECTION_PACKET", sorted(dropped))
        packet = {
            "drone_id": self.port,
            "timestamp": time.time(),
            "locked_target_id": locked_target_id,
            "detected_targets": sanitized_targets,
            "state": state_label,
        }
        self._log_detection_packet(packet)
        return self.send_mission_packet(packet)

    def report_target(self, lat, lon, conf, tracker_id=None, raw_data=None, covariance=None) -> SwarmResponse:
        if not self._sm:
            return SwarmResponse()
        try:
            safe_raw = raw_data
            if isinstance(raw_data, dict) and "action" not in raw_data:
                self._log_sanitization("REPORT_TARGET", self._dropped_detection_fields(raw_data))
                safe_raw = self._sanitize_detection(raw_data)
            raw = self._sm.report_target(self.port, lat, lon, conf, tracker_id=tracker_id, raw_data=safe_raw, covariance=covariance)
            return parse_response(raw)
        except Exception as exc:
            SwarmLogger.log("ERROR", f"DRONE_{self.port}", f"report_target error: {exc}", "LINK")
            return SwarmResponse()

    def publish_telemetry(self, detected_targets: list, locked_target_id: Optional[str] = None, state_label: str = "", force: bool = False) -> SwarmResponse:
        signature = (
            str(locked_target_id or ""),
            str(state_label or ""),
            len(detected_targets or []),
            tuple(sorted(int(item.get("track_id")) for item in (detected_targets or []) if item.get("track_id") is not None)[:4]),
        )
        window = self._discipline.window("telemetry", min_interval_ms=_TELEMETRY_MIN_INTERVAL_MS, max_silence_ms=_TELEMETRY_MAX_SILENCE_MS)
        if not force and not window.should_emit(signature=signature):
            return SwarmResponse(action="HOLD", target_id=locked_target_id)
        window.mark_emitted(signature=signature)
        return self.send_detection_packet(detected_targets, locked_target_id, state_label)

    def publish_autonomous_update(self, target_id: str, drone_gps: Tuple[float, float, float], target_gps: Optional[Tuple[float, float, float]] = None, session_id: Optional[str] = None) -> SwarmResponse:
        kwargs = {"target_gps": target_gps}
        if session_id is not None:
            kwargs["session_id"] = session_id
        return self.send_action("EXECUTION_UPDATE", target_id=target_id, **kwargs)

    def publish_terminal_event(self, target_id: str, coverage: float, altitude: float) -> SwarmResponse:
        return self.send_action("IMPACT_CONFIRMED", target_id=target_id, coverage=coverage, altitude=altitude)

    def _log_packet(self, packet: dict) -> None:
        action = str(packet.get("action") or "UNKNOWN")
        summary = [f"action={action}"]
        if packet.get("target_id") is not None:
            summary.append(f"target={packet.get('target_id')}")
        if packet.get("local_tracker_id") is not None:
            summary.append(f"local_id={packet.get('local_tracker_id')}")
        SwarmLogger.log_link_activity(
            module=f"DRONE_{self.port}",
            direction="TX",
            action=action,
            detail=" ".join(summary[1:]) if len(summary) > 1 else "",
            level="BRIDGE",
            interval_s=BRIDGE_LOG_INTERVAL_S,
            key=f"link:tx:{self.port}:{action}",
        )

    def _log_detection_packet(self, packet: dict) -> None:
        targets = packet.get("detected_targets") or []
        group_count = sum(1 for item in targets if item.get("is_group"))
        local_ids = [str(item.get("track_id")) for item in targets if item.get("track_id") is not None]
        preview = ",".join(local_ids[:4]) if local_ids else "-"
        if len(local_ids) > 4:
            preview += ",..."
        SwarmLogger.log_link_activity(
            module=f"DRONE_{self.port}",
            direction="TX",
            action="DETECTIONS",
            detail=(
                f"count={len(targets)} groups={group_count} "
                f"locked={packet.get('locked_target_id')} state={packet.get('state') or '-'} ids={preview}"
            ),
            level="BRIDGE",
            interval_s=max(1.0, BRIDGE_LOG_INTERVAL_S),
            key=f"link:tx:{self.port}:detections",
        )

    def _log_response(self, packet: dict, resp: SwarmResponse) -> None:
        packet_action = packet.get("action") or ("DETECTIONS" if "detected_targets" in packet else "UNKNOWN")
        extras = []
        if resp.target_id is not None:
            extras.append(f"target={resp.target_id}")
        if resp.local_id is not None:
            extras.append(f"local_id={resp.local_id}")
        if resp.command_id is not None:
            extras.append(f"cmd={resp.command_id}")
        if resp.epoch is not None:
            extras.append(f"epoch={resp.epoch}")
        SwarmLogger.log_link_activity(
            module=f"DRONE_{self.port}",
            direction="RX",
            action=str(resp.action or "UNKNOWN"),
            detail=f"for={packet_action}" + (f" {' '.join(extras)}" if extras else ""),
            level="RESPONSE",
            interval_s=BRIDGE_LOG_INTERVAL_S,
            key=f"link:rx:{self.port}:{packet_action}:{resp.action}",
        )
