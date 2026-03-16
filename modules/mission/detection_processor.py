"""Detection Processor — Converts camera detections to GPS coordinates.

Isolated version of the legacy `check_human_detection` function (330+ lines).
Responsibilities:
  - Get frame → Run YOLO → Filter bbox
  - Calculate GPS via Ray-Ground Intersection
  - Apply Adaptive GPS Kalman filter
  - Enrich group detections with target registry
"""

import math
import time
import numpy as np
from collections import deque
from typing import List, Dict, Optional, Tuple

from modules.core.logger import SwarmLogger
from modules.core.geo_math import GeoMath
from modules.mission.bbox_smoother import BboxSmoother
# GPSFilter removed - filtering is done on the leader side
from config import (
    CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_RESOLUTION_WIDTH,
    GPS_FILTER_WINDOW_SIZE, BBOX_FILTER_WINDOW_SIZE,
    EDGE_MARGIN_RATIO, MIN_BOX_DIM,
    METERS_PER_DEGREE_LAT,
    CAMERA_GROUND_CONTACT_Y_RATIO,
)


# ======================================================================
# Detection Result — normalized form of a single detection
# ======================================================================

class DetectionResult:
    """Data object carrying all information for a single detection."""
    __slots__ = (
        "track_id", "lat", "lon", "conf", "covariance",
        "bbox_center", "bbox_size", "is_edge",
        "is_group", "group_member_count", "group_id",
        "member_track_ids",
    )

    def __init__(self):
        self.track_id: Optional[int] = None
        self.lat: Optional[float] = None
        self.lon: Optional[float] = None
        self.conf: float = 0.0
        self.covariance = None
        self.bbox_center: Optional[Tuple[float, float]] = None
        self.bbox_size: Optional[Tuple[float, float]] = None
        self.is_edge: bool = False
        self.is_group: bool = False
        self.group_member_count: int = 0
        self.group_id: Optional[int] = None
        self.member_track_ids: Tuple[int, ...] = ()

    def to_swarm_dict(self, drone_gps, heading, attitude, img_dims=None) -> dict:
        """Dict format ready to be sent to swarm manager."""
        return {
            "track_id": self.track_id,
            "bbox": (self.bbox_center[0], self.bbox_center[1],
                     self.bbox_size[0], self.bbox_size[1]) if self.bbox_center and self.bbox_size else None,
            "confidence": self.conf,
            "world_xyz": (self.lat, self.lon, 0.0),
            "covariance": self.covariance.tolist() if hasattr(self.covariance, "tolist") else self.covariance,
            "drone_gps": drone_gps,
            "heading": heading,
            "attitude": attitude,
            "img_dims": img_dims or (CAMERA_RESOLUTION_WIDTH, CAMERA_HEIGHT),
            "is_group": self.is_group,
            "group_member_count": self.group_member_count,
            "group_id": self.group_id,
            "member_ids": list(self.member_track_ids),
        }


# ======================================================================
# Edge Filter
# ======================================================================

def is_edge_detection(cx, cy, w, h, attack_approved=False,
                      locked_id=None, box_id=None) -> bool:
    """Is the detection at the edge of the frame?"""
    w_img, h_img = CAMERA_WIDTH, CAMERA_HEIGHT
    margin = EDGE_MARGIN_RATIO

    # Relax margin for locked target
    if attack_approved and locked_id is not None and box_id is not None:
        if int(box_id) == int(locked_id):
            margin = 0.02

    mx = w_img * margin
    my = h_img * margin

    return (cx < mx or cx > (w_img - mx) or
            cy < my or cy > (h_img - my) or
            w < MIN_BOX_DIM or h < MIN_BOX_DIM)




# ======================================================================
# Detection Processor
# ======================================================================

class DetectionProcessor:
    """Get camera frame → Process detections → Return GPS results."""

    def __init__(self, port: int, camera_handler, human_tracker, drone_manager):
        self.port = port
        self.camera_handler = camera_handler
        self.human_tracker = human_tracker
        self.drone_manager = drone_manager
        self.bbox_width_history = deque(maxlen=BBOX_FILTER_WINDOW_SIZE)
        
        # Bbox Temporal Smoothing (Phase 1)
        self.bbox_smoother = BboxSmoother()

        # Rate-limited logging
        self._log_ts = 0.0
        self._log_interval = 2.0
        # Change-based detection tracking
        self._prev_det_count = 0
        self._prev_det_ids: set = set()

    def reset_runtime_state(self) -> None:
        """Clear per-mission detection caches and bbox smoothing history."""
        self.bbox_width_history.clear()
        self.bbox_smoother.reset_all()
        self._log_ts = 0.0
        self._prev_det_count = 0
        self._prev_det_ids.clear()

    # ------------------------------------------------------------------
    # Main processing
    # ------------------------------------------------------------------

    def process_frame(self, attack_approved: bool = False,
                      locked_id: Optional[int] = None,
                      debug: bool = False,
                      attack_mode: bool = False) -> Tuple[Optional[dict], List[DetectionResult]]:
        """Process a single frame.

        Returns:
            (tracker_result, detections) — tracker_result: raw tracker output,
            detections: normalized DetectionResult list
        """
        frame = self.camera_handler.get_latest_frame_as_array(self.port)
        if frame is None:
            return None, []

        tracker_result = self.human_tracker.detect_and_track(
            frame, debug=debug, attack_mode=attack_mode)
        if tracker_result is None:
            return None, []

        if not tracker_result.get("detected", False):
            return tracker_result, []

        detections = self._extract_detections(
            tracker_result, attack_approved, locked_id)

        return tracker_result, detections

    def process_scan_frame(self, attack_approved: bool = False,
                           locked_id: Optional[int] = None) -> List[DetectionResult]:
        """Scan mode frame processing (detection only, no annotated frame)."""
        frame = self.camera_handler.get_latest_frame_as_array(self.port)
        if frame is None:
            return []

        tracker_result = self.human_tracker.detect_and_track(frame, debug=False)
        if tracker_result is None or not tracker_result.get("detected", False):
            return []

        return self._extract_detections(tracker_result, attack_approved, locked_id)

    # ------------------------------------------------------------------
    # Extraction
    # ------------------------------------------------------------------

    def _extract_detections(self, tracker_result: dict,
                            attack_approved: bool,
                            locked_id: Optional[int]) -> List[DetectionResult]:
        """Extract DetectionResult list from tracker output."""
        results = []

        # Vehicle telemetry
        vehicle = self.drone_manager.drones.get(self.port)
        if vehicle is None or vehicle == "connecting":
            return results
        loc = vehicle.location.global_relative_frame
        if not loc or loc.lat == 0:
            return results

        d_lat, d_lon, d_alt = loc.lat, loc.lon, loc.alt
        roll = vehicle.attitude.roll
        pitch = vehicle.attitude.pitch
        yaw = vehicle.attitude.yaw

        # Forward drone pose
        self.human_tracker.set_drone_pose(d_lat, d_lon, d_alt, roll, pitch, yaw)

        # Candidates from tracking mode
        candidates = tracker_result.get("detected_candidates", [])
        if candidates:
            results = self._process_candidates(candidates, vehicle, locked_id)
        else:
            # Fallback: all_detections boxes
            all_det = tracker_result.get("all_detections", [])
            if all_det:
                results = self._process_boxes(
                    all_det[0].boxes, vehicle, attack_approved, locked_id)

        # Group abstraction: people inside a cluster stop being individual
        # operational targets. We keep one synthetic target per group.
        group_info = tracker_result.get("group_info")
        if group_info and hasattr(group_info, "groups") and group_info.groups:
            results = self._apply_group_abstraction(group_info, results)

        # Rate-limited log after group abstraction so camera/radar counts match.
        self._rate_limited_log(tracker_result, results)

        return results

    def _process_candidates(self, candidates: list, vehicle,
                            locked_id: Optional[int]) -> List[DetectionResult]:
        """detected_candidates listesinden DetectionResult'lar çıkar."""
        results = []
        loc = vehicle.location.global_relative_frame
        d_lat, d_lon, d_alt = loc.lat, loc.lon, loc.alt

        for c in candidates:
            c_cx, c_cy = c["center"]
            c_w, c_h = c["dims"]
            c_id = c.get("id")
            c_conf = c.get("conf", 0.0)
            
            # Bbox temporal smoothing (Phase 1)
            smoothed_cx, smoothed_cy = self.bbox_smoother.smooth(
                c_id, c_cx, c_cy, c_conf, c_w, c_h
            )
            ground_cx, ground_cy = GeoMath.bbox_ground_contact_point(
                smoothed_cx, smoothed_cy, c_w, c_h, CAMERA_GROUND_CONTACT_Y_RATIO
            )

            rgi = GeoMath.ray_ground_intersection(
                d_lat, d_lon, d_alt,
                vehicle.attitude.roll, vehicle.attitude.pitch, vehicle.attitude.yaw,
                ground_cx, ground_cy)

            if rgi is None or len(rgi) < 2 or rgi[0] is None:
                continue

            det = DetectionResult()
            det.lat, det.lon = rgi[0], rgi[1]
            det.covariance = rgi[2] if len(rgi) > 2 else None
            det.track_id = c_id
            det.conf = c_conf
            det.bbox_center = (smoothed_cx, smoothed_cy)  # Smoothed center
            det.bbox_size = (c_w, c_h)
            det.is_edge = is_edge_detection(c_cx, c_cy, c_w, c_h)  # Edge check with raw center
            results.append(det)

        return results

    def _process_boxes(self, boxes, vehicle, attack_approved: bool,
                       locked_id: Optional[int]) -> List[DetectionResult]:
        """Convert YOLO boxes to DetectionResult list (scan mode)."""
        results = []
        loc = vehicle.location.global_relative_frame
        d_lat, d_lon, d_alt = loc.lat, loc.lon, loc.alt

        for box in boxes:
            try:
                if hasattr(box.xywh[0], "tolist"):
                    bx = box.xywh[0].tolist()
                else:
                    bx = box.xywh[0]
                cx, cy, w, h = bx[0], bx[1], bx[2], bx[3]
                box_id = box.id[0].item() if box.id is not None else None
                conf = box.conf[0].item() if hasattr(box.conf[0], "item") else float(box.conf[0])

                edge = is_edge_detection(cx, cy, w, h, attack_approved, locked_id, box_id)
                
                # Bbox temporal smoothing (Phase 1)
                track_id_int = int(box_id) if box_id is not None else None
                smoothed_cx, smoothed_cy = self.bbox_smoother.smooth(
                    track_id_int, cx, cy, conf, w, h
                )
                ground_cx, ground_cy = GeoMath.bbox_ground_contact_point(
                    smoothed_cx, smoothed_cy, w, h, CAMERA_GROUND_CONTACT_Y_RATIO
                )

                rgi = GeoMath.ray_ground_intersection(
                    d_lat, d_lon, d_alt,
                    vehicle.attitude.roll, vehicle.attitude.pitch, vehicle.attitude.yaw,
                    ground_cx, ground_cy)

                if rgi is None or len(rgi) < 2 or rgi[0] is None:
                    continue

                t_lat, t_lon = rgi[0], rgi[1]
                t_cov = rgi[2] if len(rgi) > 2 else None


                det = DetectionResult()
                det.lat, det.lon = t_lat, t_lon
                det.covariance = t_cov
                det.track_id = track_id_int
                det.conf = conf
                det.bbox_center = (smoothed_cx, smoothed_cy)  # Smoothed center
                det.bbox_size = (w, h)
                det.is_edge = edge
                results.append(det)
            except Exception:
                continue

        return results

    # ------------------------------------------------------------------
    # Group enrichment
    # ------------------------------------------------------------------

    def _apply_group_abstraction(self, group_info, detections: List[DetectionResult]) -> List[DetectionResult]:
        """Convert clustered people into a single operational group target.

        Individual detections that belong to a group are removed from the
        mission/swarm pipeline. Only true singles remain as person targets.
        """
        if not group_info.groups or not detections:
            return detections

        det_by_track = {
            int(det.track_id): det for det in detections
            if det.track_id is not None
        }
        grouped_member_ids = set()
        grouped_results: List[DetectionResult] = []

        for grp in group_info.groups:
            member_dets = [
                det_by_track[int(mid)] for mid in grp.member_ids
                if int(mid) in det_by_track
            ]
            if len(member_dets) < 2:
                continue

            grouped_member_ids.update(int(det.track_id) for det in member_dets if det.track_id is not None)

            rep_det = min(
                member_dets,
                key=lambda det: (
                    (det.bbox_center[0] - grp.center_px[0]) ** 2 +
                    (det.bbox_center[1] - grp.center_px[1]) ** 2
                ) if det.bbox_center else 1e9
            )

            group_det = DetectionResult()
            group_det.track_id = rep_det.track_id
            group_det.lat, group_det.lon = grp.center_gps
            group_det.conf = grp.avg_confidence
            group_det.is_group = True
            group_det.group_member_count = grp.member_count
            group_det.group_id = grp.group_id
            group_det.member_track_ids = tuple(sorted(int(mid) for mid in grp.member_ids))
            group_det.bbox_center = grp.center_px
            x1, y1, x2, y2 = grp.bounding_rect_px
            group_det.bbox_size = (max(1.0, x2 - x1), max(1.0, y2 - y1))
            group_det.is_edge = any(det.is_edge for det in member_dets)
            group_det.covariance = self._build_group_covariance(member_dets, grp.center_gps)
            grouped_results.append(group_det)

        singles = [
            det for det in detections
            if det.track_id is None or int(det.track_id) not in grouped_member_ids
        ]

        return grouped_results + singles

    def _build_group_covariance(self, member_dets: List[DetectionResult], center_gps: Tuple[float, float]):
        """Approximate group covariance from member covariances + group spread."""
        covariances = [
            np.array(det.covariance, dtype=np.float64)
            for det in member_dets
            if det.covariance is not None
        ]

        if covariances:
            base_cov = sum(covariances) / len(covariances)
        else:
            base_cov = np.eye(3, dtype=np.float64) * 25.0

        spread_n = []
        spread_e = []
        c_lat, c_lon = center_gps
        for det in member_dets:
            if det.lat is None or det.lon is None:
                continue
            d_n = (det.lat - c_lat) * METERS_PER_DEGREE_LAT
            d_e = (det.lon - c_lon) * METERS_PER_DEGREE_LAT * math.cos(math.radians(c_lat))
            spread_n.append(d_n)
            spread_e.append(d_e)

        if spread_n and spread_e:
            base_cov = base_cov.copy()
            base_cov[0, 0] += float(np.var(spread_n))
            base_cov[1, 1] += float(np.var(spread_e))

        return base_cov

    # ------------------------------------------------------------------
    # Rate-limited logging
    # ------------------------------------------------------------------

    def _rate_limited_log(self, tracker_result: dict, detections: List[DetectionResult]):
        """Change-based detection logging — instant on count/ID change, every 10s when stable.
        
        Enhanced: GPS, covariance, group statistics, camera vs radar comparison.
        """
        now = time.time()

        if not detections:
            # Detection lost: log if we had detections before
            if self._prev_det_ids:
                SwarmLogger.log("CAMERA_STATS", f"DRONE_{self.port}",
                                "CAMERA: 0 bbox | 0 grup | 0 ID (all lost)", "VISION")
                self._prev_det_ids = set()
                self._prev_det_count = 0
                self._log_ts = now
            return

        current_ids = {d.track_id for d in detections if d.track_id is not None}
        current_count = len(detections)

        # Determine if something changed
        count_changed = current_count != self._prev_det_count
        ids_changed = current_ids != self._prev_det_ids

        # New detections or lost detections
        new_ids = current_ids - self._prev_det_ids if self._prev_det_ids else set()
        lost_ids = self._prev_det_ids - current_ids if self._prev_det_ids else set()

        # Change-based: log immediately on change
        # Periodic: log every 10s when stable
        is_change = count_changed or ids_changed
        is_periodic = (now - self._log_ts >= 10.0)

        if not is_change and not is_periodic:
            return

        self._log_ts = now
        self._prev_det_count = current_count
        self._prev_det_ids = current_ids

        # --- GROUP STATISTICS ---
        group_dets = [d for d in detections if d.is_group]
        group_count = len(group_dets)
        total_group_members = sum(d.group_member_count for d in group_dets) if group_dets else 0
        unique_group_ids = set(d.group_id for d in group_dets if d.group_id)
        
        # --- GPS/COVARIANCE STATISTICS ---
        gps_summaries = []
        sigma_values = []
        for d in detections:
            if d.covariance is not None:
                try:
                    import numpy as np
                    cov = d.covariance
                    if hasattr(cov, 'shape') and cov.shape[0] >= 2:
                        sigma = (cov[0, 0] + cov[1, 1]) ** 0.5
                        sigma_values.append(sigma)
                        gps_summaries.append(
                            f"ID:{d.track_id} GPS:({d.lat:.6f},{d.lon:.6f}) σ={sigma:.1f}m conf={d.conf:.2f}"
                        )
                    else:
                        gps_summaries.append(f"ID:{d.track_id} GPS:({d.lat:.6f},{d.lon:.6f}) conf={d.conf:.2f}")
                except Exception:
                    gps_summaries.append(f"ID:{d.track_id} GPS:({d.lat:.6f},{d.lon:.6f})")
            else:
                gps_summaries.append(f"ID:{d.track_id} GPS:({d.lat:.6f},{d.lon:.6f})")
        
        # Average sigma
        avg_sigma = sum(sigma_values) / len(sigma_values) if sigma_values else 0.0
        max_sigma = max(sigma_values) if sigma_values else 0.0

        # --- CAMERA SUMMARY (for radar comparison) ---
        SwarmLogger.log(
            "CAMERA_STATS", f"DRONE_{self.port}",
            f"CAMERA: {len(detections)} bbox | {group_count} grup ({total_group_members} üye) | "
            f"IDs: {sorted(current_ids)} | avg_σ={avg_sigma:.1f}m max_σ={max_sigma:.1f}m",
            "VISION"
        )

        # --- GPS DETAIL (on change) ---
        if is_change and gps_summaries:
            SwarmLogger.log(
                "CAMERA_STATS", f"DRONE_{self.port}",
                f"GPS_DETAY: {' | '.join(gps_summaries[:5])}" + (" ..." if len(gps_summaries) > 5 else ""),
                "VISION"
            )

        # Change annotation
        change_note = ""
        if is_change and self._prev_det_ids:
            parts = []
            if new_ids:
                parts.append(f"+{','.join(str(i) for i in sorted(new_ids))}")
            if lost_ids:
                parts.append(f"-{','.join(str(i) for i in sorted(lost_ids))}")
            if parts:
                change_note = f" | CHG: {' '.join(parts)}"

        # Group detail
        if group_dets:
            group_summary = "GROUPS: " + " ".join(
                f"[G{d.group_id} x{d.group_member_count}]" for d in group_dets
            )
            SwarmLogger.log(
                "CAMERA_STATS", f"DRONE_{self.port}",
                group_summary + change_note, "VISION"
            )
