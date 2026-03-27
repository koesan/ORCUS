"""Detection normalization and world projection."""

import math
import re
import time
import numpy as np
from typing import List, Dict, Optional, Tuple

from modules.core.logger import SwarmLogger
from modules.core.geo_math import GeoMath
from modules.vision.group_tracker import GroupBboxSmoother
from config import (
    CAMERA_WIDTH, CAMERA_HEIGHT,
    EDGE_MARGIN_RATIO, MIN_BOX_DIM,
    CAMERA_GROUND_CONTACT_Y_RATIO,
)

_WORLD_TRACK_STALE_S = 2.0
_WORLD_MAX_STEP_BASE_M = 1.5
_WORLD_MAX_SPEED_MPS = 8.0
_WORLD_FILTER_ALPHA_BOOST = 0.18
_ATTACK_WORLD_FILTER_ALPHA_BOOST = 0.18
_ATTACK_WORLD_MAX_STEP_BASE_M = 2.5
_ATTACK_WORLD_MAX_SPEED_MPS = 14.0
_WORLD_FILTER_ALPHA = 0.60
_WORLD_FILTER_BETA = 0.12
_ATTACK_WORLD_FILTER_ALPHA = 0.78
_ATTACK_WORLD_FILTER_BETA = 0.20


class DetectionResult:
    """Normalized detection payload."""
    __slots__ = (
        "track_id", "lat", "lon", "conf", "covariance",
        "bbox_center", "bbox_size", "bbox_xyxy", "is_edge",
        "is_group", "group_member_count", "group_id", "member_ids",
    )

    def __init__(self):
        self.track_id: Optional[int] = None
        self.lat: Optional[float] = None
        self.lon: Optional[float] = None
        self.conf: float = 0.0
        self.covariance = None
        self.bbox_center: Optional[Tuple[float, float]] = None
        self.bbox_size: Optional[Tuple[float, float]] = None
        self.bbox_xyxy: Optional[Tuple[float, float, float, float]] = None
        self.is_edge: bool = False
        self.is_group: bool = False
        self.group_member_count: int = 0
        self.group_id: Optional[object] = None
        self.member_ids: Tuple[int, ...] = ()

    def to_swarm_dict(self, drone_gps, heading, attitude, img_dims=None) -> dict:
        """Build the sanitized payload sent to the leader."""
        return {
            "track_id": self.track_id,
            "confidence": self.conf,
            "world_xyz": (self.lat, self.lon, 0.0),
            "covariance": self.covariance.tolist() if hasattr(self.covariance, "tolist") else self.covariance,
            "observation_quality": self.leader_quality_score(),
            "is_edge": self.is_edge,
            "is_group": self.is_group,
            "group_member_count": self.group_member_count,
            "group_id": self.group_id,
            "member_ids": list(self.member_ids or ()),
        }

    def leader_quality_score(self) -> float:
        """Score a detection before leader ingest."""
        score = max(0.0, float(self.conf or 0.0)) * 8.0
        cov = self.covariance
        if cov is not None:
            try:
                cov = np.array(cov, dtype=np.float64)
                if cov.ndim == 2 and cov.shape[0] >= 2:
                    trace = float(cov[0, 0] + cov[1, 1])
                    if trace > 0.0:
                        score += min(6.0, 60.0 / trace)
            except Exception:
                pass
        else:
            score += 2.0

        if self.is_edge:
            score *= 0.7
        if self.is_group:
            score += min(1.5, float(self.group_member_count or 0) * 0.25)
        return max(0.0, score)

    def log_summary(self) -> str:
        center = "-"
        if self.bbox_center is not None:
            try:
                center = f"({float(self.bbox_center[0]):.0f},{float(self.bbox_center[1]):.0f})"
            except Exception:
                center = str(self.bbox_center)
        size = "-"
        if self.bbox_size is not None:
            try:
                size = f"{float(self.bbox_size[0]):.0f}x{float(self.bbox_size[1]):.0f}"
            except Exception:
                size = str(self.bbox_size)
        gps = "-"
        if self.lat is not None and self.lon is not None:
            gps = f"({self.lat:.6f},{self.lon:.6f})"
        group = ""
        if self.is_group:
            group = f" G{self.group_id}x{self.group_member_count}"
            if self.member_ids:
                group += f" m={list(self.member_ids[:4])}"
        return (
            f"id={self.track_id} box={center}/{size} gps={gps} "
            f"q={self.leader_quality_score():.1f} edge={int(self.is_edge)}{group}"
        )


def is_edge_detection(cx, cy, w, h, attack_approved=False,
                      locked_id=None, box_id=None) -> bool:
    """Check whether the bbox is near the image edge."""
    w_img, h_img = CAMERA_WIDTH, CAMERA_HEIGHT
    margin = EDGE_MARGIN_RATIO

    if attack_approved and locked_id is not None and box_id is not None:
        if int(box_id) == int(locked_id):
            margin = 0.02

    mx = w_img * margin
    my = h_img * margin

    return (cx < mx or cx > (w_img - mx) or
            cy < my or cy > (h_img - my) or
            w < MIN_BOX_DIM or h < MIN_BOX_DIM)


class DetectionProcessor:
    """Process tracker output into world observations."""

    @staticmethod
    def _canonical_group_id(group_id) -> Optional[str]:
        """Normalize local group ids to one wire format."""
        if group_id is None:
            return None
        token = str(group_id).strip()
        if not token:
            return None
        match = re.fullmatch(r"[Gg]?(\d+)", token)
        if match:
            return f"G{int(match.group(1))}"
        return token.upper()

    def __init__(self, port: int, camera_handler, human_tracker, drone_manager):
        self.port = port
        self.camera_handler = camera_handler
        self.human_tracker = human_tracker
        self.drone_manager = drone_manager
        self.group_bbox_smoother = GroupBboxSmoother()
        self._log_ts = 0.0
        self._prev_det_count = 0
        self._prev_det_ids: set = set()
        self._stable_id_counter = 0
        self._stable_tracks: Dict[int, Dict[str, object]] = {}
        self._raw_to_stable: Dict[int, int] = {}
        self._stable_to_raw_current: Dict[int, int] = {}
        self._group_representatives: Dict[str, int] = {}
        self._group_rep_centers: Dict[str, Tuple[float, float]] = {}
        self._group_aliases: Dict[str, str] = {}
        self._frame_claimed_stable_ids: set = set()
        self._world_track_state: Dict[str, Dict[str, float]] = {}

    def reset_runtime_state(self) -> None:
        """Clear per-mission caches."""
        self.group_bbox_smoother.reset_all()
        self._log_ts = 0.0
        self._prev_det_count = 0
        self._prev_det_ids.clear()
        self._stable_id_counter = 0
        self._stable_tracks.clear()
        self._raw_to_stable.clear()
        self._stable_to_raw_current.clear()
        self._group_representatives.clear()
        self._group_rep_centers.clear()
        self._group_aliases.clear()
        self._frame_claimed_stable_ids.clear()
        self._world_track_state.clear()

    def process_frame(self, attack_approved: bool = False,
                      locked_id: Optional[int] = None,
                      debug: bool = False,
                      attack_mode: bool = False) -> Tuple[Optional[dict], List[DetectionResult]]:
        """Process one frame."""
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
            tracker_result, attack_approved, locked_id, attack_mode=attack_mode)
        self._normalize_tracker_result_ids(tracker_result)

        return tracker_result, detections

    def process_scan_frame(self, attack_approved: bool = False,
                           locked_id: Optional[int] = None,
                           attack_mode: bool = False):
        """Process one UI frame and keep annotations."""
        frame = self.camera_handler.get_latest_frame_as_array(self.port)
        if frame is None:
            return None, []

        tracker_result = self.human_tracker.detect_and_track(frame, debug=True, attack_mode=attack_mode)
        if tracker_result is None or not tracker_result.get("detected", False):
            return tracker_result, []

        detections = self._extract_detections(
            tracker_result, attack_approved, locked_id, attack_mode=attack_mode)
        return tracker_result, detections

    def _extract_detections(self, tracker_result: dict,
                            attack_approved: bool,
                            locked_id: Optional[int],
                            attack_mode: bool = False) -> List[DetectionResult]:
        """Extract normalized detections from tracker output."""
        results = []
        self._frame_claimed_stable_ids.clear()

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

        self.human_tracker.set_drone_pose(d_lat, d_lon, d_alt, roll, pitch, yaw)

        candidates = tracker_result.get("detected_candidates", [])
        if candidates:
            results = self._process_candidates(candidates, vehicle, locked_id, attack_mode=attack_mode)
        else:
            raw_results = tracker_result.get("results", [])
            if raw_results:
                results = self._process_boxes(
                    raw_results[0].boxes, vehicle, attack_approved, locked_id, attack_mode=attack_mode)

        group_info = tracker_result.get("group_info")
        if group_info:
            all_targets = getattr(group_info, 'all_targets', None)
            groups = getattr(group_info, 'groups', None)
            targets = all_targets if all_targets else groups
            if targets:
                results = self._apply_group_abstraction_v2(targets, results, vehicle, attack_mode=attack_mode)

        results = self._promote_group_only(results)
        results = self._normalize_group_aliases(results)
        results = self._collapse_same_group_detections(results)
        results = self._project_group_world_positions(results, vehicle, attack_mode=attack_mode)
        results = self._collapse_same_physical_groups(results)

        self._rate_limited_log(results)

        return results

    def _normalize_tracker_result_ids(self, tracker_result: dict) -> None:
        if not isinstance(tracker_result, dict):
            return

        raw_track_id = tracker_result.get("track_id")
        if raw_track_id is not None:
            stable_id = self._raw_to_stable.get(int(raw_track_id))
            if stable_id is not None:
                tracker_result["raw_track_id"] = int(raw_track_id)
                tracker_result["track_id"] = int(stable_id)

        normalized_candidates = []
        for cand in tracker_result.get("detected_candidates", []) or []:
            if not isinstance(cand, dict):
                normalized_candidates.append(cand)
                continue
            normalized = dict(cand)
            raw_id = normalized.get("id")
            if raw_id is not None:
                stable_id = self._raw_to_stable.get(int(raw_id))
                if stable_id is not None:
                    normalized["raw_id"] = int(raw_id)
                    normalized["id"] = int(stable_id)
            normalized_candidates.append(normalized)
        tracker_result["detected_candidates"] = normalized_candidates

    @staticmethod
    def _bbox_iou_xyxy(box_a, box_b) -> float:
        if box_a is None or box_b is None:
            return 0.0
        ax1, ay1, ax2, ay2 = [float(v) for v in box_a]
        bx1, by1, bx2, by2 = [float(v) for v in box_b]
        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)
        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter = inter_w * inter_h
        if inter <= 0.0:
            return 0.0
        area_a = max(1.0, (ax2 - ax1) * (ay2 - ay1))
        area_b = max(1.0, (bx2 - bx1) * (by2 - by1))
        return inter / max(1.0, area_a + area_b - inter)

    @staticmethod
    def _bbox_center_distance(box_a, box_b) -> float:
        if box_a is None or box_b is None:
            return float("inf")
        ax1, ay1, ax2, ay2 = [float(v) for v in box_a]
        bx1, by1, bx2, by2 = [float(v) for v in box_b]
        acx, acy = (ax1 + ax2) / 2.0, (ay1 + ay2) / 2.0
        bcx, bcy = (bx1 + bx2) / 2.0, (by1 + by2) / 2.0
        return math.hypot(acx - bcx, acy - bcy)

    def _next_stable_id(self) -> int:
        self._stable_id_counter += 1
        return self._stable_id_counter

    def _assign_stable_id(self, raw_id: Optional[int], box_xyxy, conf: float) -> Optional[int]:
        if raw_id is None:
            return None
        try:
            raw_id = int(raw_id)
        except (TypeError, ValueError):
            return None

        now = time.time()
        existing = self._raw_to_stable.get(raw_id)
        if existing is not None and existing in self._stable_tracks:
            self._stable_tracks[existing] = {
                "bbox": tuple(float(v) for v in box_xyxy),
                "last_seen": now,
                "raw_id": raw_id,
                "conf": float(conf or 0.0),
            }
            self._stable_to_raw_current[existing] = raw_id
            self._frame_claimed_stable_ids.add(existing)
            return existing

        best_stable = None
        best_score = -1.0
        for stable_id, state in list(self._stable_tracks.items()):
            if stable_id in self._frame_claimed_stable_ids:
                continue
            age = now - float(state.get("last_seen", 0.0) or 0.0)
            if age > 1.5:
                continue
            iou = self._bbox_iou_xyxy(box_xyxy, state.get("bbox"))
            center_dist = self._bbox_center_distance(box_xyxy, state.get("bbox"))
            if iou < 0.35 and center_dist > 80.0:
                continue
            score = iou - (center_dist / 1000.0)
            if score > best_score:
                best_score = score
                best_stable = stable_id

        stable_id = int(best_stable) if best_stable is not None else self._next_stable_id()
        self._raw_to_stable[raw_id] = stable_id
        self._stable_tracks[stable_id] = {
            "bbox": tuple(float(v) for v in box_xyxy),
            "last_seen": now,
            "raw_id": raw_id,
            "conf": float(conf or 0.0),
        }
        self._stable_to_raw_current[stable_id] = raw_id
        self._frame_claimed_stable_ids.add(stable_id)
        return stable_id

    def raw_track_id_for_stable(self, stable_id: Optional[int]) -> Optional[int]:
        if stable_id is None:
            return None
        try:
            stable_id = int(stable_id)
        except (TypeError, ValueError):
            return None
        raw_id = self._stable_to_raw_current.get(stable_id)
        if raw_id is not None:
            return int(raw_id)
        state = self._stable_tracks.get(stable_id)
        if state is None:
            return None
        raw_id = state.get("raw_id")
        return int(raw_id) if raw_id is not None else None

    def stable_track_id_for_raw(self, raw_id: Optional[int]) -> Optional[int]:
        if raw_id is None:
            return None
        try:
            raw_id = int(raw_id)
        except (TypeError, ValueError):
            return None
        stable_id = self._raw_to_stable.get(raw_id)
        return int(stable_id) if stable_id is not None else None

    def _process_candidates(self, candidates: list, vehicle,
                            locked_id: Optional[int],
                            attack_mode: bool = False) -> List[DetectionResult]:
        """Build detections from tracker candidate dicts."""
        results = []
        for c in candidates:
            c_cx, c_cy = c["center"]
            c_w, c_h = c["dims"]
            c_id = c.get("id")
            c_conf = c.get("conf", 0.0)
            box_xyxy = c.get("box_xyxy")
            stable_id = self._assign_stable_id(c_id, box_xyxy, c_conf) if box_xyxy is not None else c_id

            det = DetectionResult()
            det.track_id = stable_id
            det.conf = c_conf
            det.bbox_center = (c_cx, c_cy)
            det.bbox_size = (c_w, c_h)
            det.bbox_xyxy = (c_cx - c_w / 2.0, c_cy - c_h / 2.0,
                            c_cx + c_w / 2.0, c_cy + c_h / 2.0)
            det.is_edge = is_edge_detection(c_cx, c_cy, c_w, c_h)
            results.append(det)

        return results

    def _process_boxes(self, boxes, vehicle, attack_approved: bool,
                       locked_id: Optional[int], attack_mode: bool = False) -> List[DetectionResult]:
        """Build detections from raw YOLO boxes."""
        results = []
        for box in boxes:
            try:
                if hasattr(box.xywh[0], "tolist"):
                    bx = box.xywh[0].tolist()
                else:
                    bx = box.xywh[0]
                cx, cy, w, h = bx[0], bx[1], bx[2], bx[3]
                box_id = box.id[0].item() if box.id is not None else None
                conf = box.conf[0].item() if hasattr(box.conf[0], "item") else float(box.conf[0])
                xyxy = box.xyxy[0].tolist() if hasattr(box.xyxy[0], "tolist") else box.xyxy[0]
                stable_id = self._assign_stable_id(box_id, xyxy, conf)

                edge = is_edge_detection(cx, cy, w, h, attack_approved, locked_id, box_id)
                
                det = DetectionResult()
                det.track_id = stable_id
                det.conf = conf
                det.bbox_center = (cx, cy)
                det.bbox_size = (w, h)
                det.is_edge = edge
                results.append(det)
            except Exception:
                continue

        return results

    def _apply_group_abstraction_v2(
        self,
        target_groups: list,
        detections: List[DetectionResult],
        vehicle,
        attack_mode: bool = False,
    ) -> List[DetectionResult]:
        """Convert grouped tracker output into group detections."""
        if not target_groups or not detections:
            return detections

        det_by_track = {
            int(det.track_id): det for det in detections
            if det.track_id is not None
        }
        grouped_member_ids = set()
        grouped_results: List[DetectionResult] = []

        for grp in target_groups:
            stable_member_ids = []
            for mid in grp.member_ids:
                stable_mid = self.stable_track_id_for_raw(mid)
                if stable_mid is None:
                    try:
                        stable_mid = int(mid)
                    except (TypeError, ValueError):
                        continue
                stable_member_ids.append(int(stable_mid))

            unique_member_ids = tuple(sorted(set(stable_member_ids)))
            member_dets = [
                det_by_track[int(mid)] for mid in unique_member_ids
                if int(mid) in det_by_track
            ]
            if not member_dets:
                continue

            grouped_member_ids.update(
                int(det.track_id) for det in member_dets if det.track_id is not None
            )

            rep_det = self._select_group_representative(grp, member_dets)
            x1, y1, x2, y2 = grp.bounding_rect_px
            group_w = max(1.0, float(x2) - float(x1))
            group_h = max(1.0, float(y2) - float(y1))
            smoothed_group_cx, smoothed_group_cy = self.group_bbox_smoother.smooth(
                f"group:{grp.group_id}",
                float(grp.center_px[0]),
                float(grp.center_px[1]),
                float(grp.avg_confidence or 0.0),
                group_w,
                group_h,
            )

            group_det = DetectionResult()
            group_det.track_id = rep_det.track_id
            group_det.conf = grp.avg_confidence
            group_det.is_group = True
            group_det.group_member_count = grp.member_count
            group_det.group_id = self._canonical_group_id(grp.group_id)
            group_det.member_ids = unique_member_ids
            group_det.bbox_center = (smoothed_group_cx, smoothed_group_cy)
            group_det.bbox_size = (group_w, group_h)
            group_det.bbox_xyxy = (
                smoothed_group_cx - group_w / 2.0,
                smoothed_group_cy - group_h / 2.0,
                smoothed_group_cx + group_w / 2.0,
                smoothed_group_cy + group_h / 2.0,
            )
            group_det.is_edge = any(det.is_edge for det in member_dets)
            grouped_results.append(group_det)

        singles = [
            det for det in detections
            if det.track_id is None or int(det.track_id) not in grouped_member_ids
        ]
        return grouped_results + singles

    def _promote_group_only(self, detections: List[DetectionResult]) -> List[DetectionResult]:
        """Normalize all outgoing detections to group semantics."""
        normalized: List[DetectionResult] = []
        for det in detections or []:
            if det is None:
                continue
            if det.is_group:
                if det.group_member_count <= 0:
                    det.group_member_count = 1
                if det.group_id is None:
                    det.group_id = self._canonical_group_id(det.track_id if det.track_id is not None else len(normalized) + 1)
                else:
                    det.group_id = self._canonical_group_id(det.group_id)
                normalized.append(det)
                continue

            det.is_group = True
            det.group_member_count = 1
            det.group_id = self._canonical_group_id(det.track_id if det.track_id is not None else len(normalized) + 1)
            det.member_ids = tuple(int(mid) for mid in (det.member_ids or ()))
            normalized.append(det)
        return normalized

    def _collapse_same_group_detections(self, detections: List[DetectionResult]) -> List[DetectionResult]:
        """Emit one canonical detection per group id."""
        canonical: Dict[str, DetectionResult] = {}
        ordered_keys: List[str] = []
        for det in detections or []:
            if det is None:
                continue
            key = f"group:{det.group_id}" if det.group_id is not None else f"track:{det.track_id}"
            existing = canonical.get(key)
            if existing is None:
                canonical[key] = det
                ordered_keys.append(key)
                continue
            canonical[key] = self._prefer_group_detection(key, existing, det)
        return [canonical[key] for key in ordered_keys]

    def _normalize_group_aliases(self, detections: List[DetectionResult]) -> List[DetectionResult]:
        """Rewrite transient group ids to their canonical family id."""
        for det in detections or []:
            if det is None or det.group_id is None:
                continue
            canonical = self._canonical_group_id(det.group_id)
            det.group_id = self._group_aliases.get(str(canonical), canonical)
        return detections

    def _prefer_group_detection(self, key: str, left: DetectionResult, right: DetectionResult) -> DetectionResult:
        """Choose the best canonical detection for one group."""
        rep_track = self._group_representatives.get(str(getattr(left, "group_id", None)))
        rep_center = self._group_rep_centers.get(str(getattr(left, "group_id", None)))

        def score(det: DetectionResult) -> float:
            value = float(det.conf or 0.0) * 10.0
            if rep_track is not None and det.track_id is not None and int(det.track_id) == int(rep_track):
                value += 40.0
            if rep_center is not None and det.bbox_center is not None:
                value -= math.hypot(
                    float(det.bbox_center[0]) - float(rep_center[0]),
                    float(det.bbox_center[1]) - float(rep_center[1]),
                ) * 0.2
            if det.bbox_size is not None:
                value += max(1.0, float(det.bbox_size[0]) * float(det.bbox_size[1])) / 1000.0
            return value

        chosen = left if score(left) >= score(right) else right
        group_id = getattr(chosen, "group_id", None)
        if group_id is not None:
            if chosen.track_id is not None:
                self._group_representatives[str(group_id)] = int(chosen.track_id)
            if chosen.bbox_center is not None:
                self._group_rep_centers[str(group_id)] = (
                    float(chosen.bbox_center[0]),
                    float(chosen.bbox_center[1]),
                )
        return chosen

    def _collapse_same_physical_groups(self, detections: List[DetectionResult]) -> List[DetectionResult]:
        """Collapse different group ids that still describe the same physical group."""
        collapsed: List[DetectionResult] = []
        for det in detections or []:
            if det is None or not det.is_group:
                if det is not None:
                    collapsed.append(det)
                continue

            merged = False
            for idx, existing in enumerate(collapsed):
                if existing is None or not getattr(existing, "is_group", False):
                    continue
                if not self._same_physical_group(existing, det):
                    continue
                collapsed[idx] = self._merge_group_family_detection(existing, det)
                merged = True
                break

            if not merged:
                collapsed.append(det)
        return collapsed

    def _same_physical_group(self, left: DetectionResult, right: DetectionResult) -> bool:
        """Return True when two group detections are the same physical family."""
        if not (left.is_group and right.is_group):
            return False

        if left.group_id is not None and right.group_id is not None:
            if str(left.group_id) == str(right.group_id):
                return True
            if self._group_aliases.get(str(left.group_id)) == str(right.group_id):
                return True
            if self._group_aliases.get(str(right.group_id)) == str(left.group_id):
                return True

        left_members = {int(mid) for mid in (left.member_ids or ())}
        right_members = {int(mid) for mid in (right.member_ids or ())}
        if left_members and right_members and (left_members & right_members):
            return True

        left_track = left.track_id
        right_track = right.track_id
        try:
            if left_track is not None and right_members and int(left_track) in right_members:
                return True
            if right_track is not None and left_members and int(right_track) in left_members:
                return True
        except (TypeError, ValueError):
            pass

        if left.lat is None or left.lon is None or right.lat is None or right.lon is None:
            return False
        if left.bbox_center is None or right.bbox_center is None:
            return False
        if left.bbox_size is None or right.bbox_size is None:
            return False

        count_diff = abs(int(left.group_member_count or 0) - int(right.group_member_count or 0))
        if count_diff > 2:
            return False

        world_dist = GeoMath.haversine_distance(left.lat, left.lon, right.lat, right.lon)
        if world_dist > 7.0:
            return False

        center_dist = math.hypot(
            float(left.bbox_center[0]) - float(right.bbox_center[0]),
            float(left.bbox_center[1]) - float(right.bbox_center[1]),
        )
        if center_dist > 120.0:
            return False

        left_area = max(1.0, float(left.bbox_size[0]) * float(left.bbox_size[1]))
        right_area = max(1.0, float(right.bbox_size[0]) * float(right.bbox_size[1]))
        size_ratio = min(left_area, right_area) / max(left_area, right_area)
        return size_ratio >= 0.40

    def _merge_group_family_detection(self, left: DetectionResult, right: DetectionResult) -> DetectionResult:
        """Keep one canonical output for the same physical group family."""
        chosen = self._prefer_group_detection(f"group:{left.group_id}", left, right)
        other = right if chosen is left else left
        canonical_gid = self._canonical_group_id(getattr(chosen, "group_id", getattr(other, "group_id", "")))
        other_gid = getattr(other, "group_id", None)
        if other_gid is not None and canonical_gid:
            self._group_aliases[str(self._canonical_group_id(other_gid))] = canonical_gid
            other.group_id = canonical_gid
        chosen.group_id = canonical_gid
        member_union = sorted({
            int(mid) for mid in (getattr(left, "member_ids", ()) or ())
        }.union({
            int(mid) for mid in (getattr(right, "member_ids", ()) or ())
        }))
        chosen.member_ids = tuple(member_union)
        chosen.group_member_count = max(
            int(getattr(left, "group_member_count", 0) or 0),
            int(getattr(right, "group_member_count", 0) or 0),
        )
        return chosen

    def _project_group_world_positions(
        self,
        detections: List[DetectionResult],
        vehicle,
        attack_mode: bool = False,
    ) -> List[DetectionResult]:
        """Project final group observations into world coordinates."""
        projected: List[DetectionResult] = []
        if vehicle is None:
            return projected

        frame = getattr(getattr(vehicle, "location", None), "global_relative_frame", None)
        if frame is None:
            return projected
        d_lat = getattr(frame, "lat", None)
        d_lon = getattr(frame, "lon", None)
        d_alt = getattr(frame, "alt", None)
        if d_lat in (None, 0) or d_lon is None or d_alt is None:
            return projected

        for det in detections or []:
            if det is None or det.bbox_center is None or det.bbox_size is None:
                continue
            cx, cy = det.bbox_center
            w, h = det.bbox_size
            ground_cx, ground_cy = GeoMath.bbox_ground_contact_point(
                cx, cy, w, h, CAMERA_GROUND_CONTACT_Y_RATIO
            )
            try:
                rgi = GeoMath.ray_ground_intersection(
                    d_lat,
                    d_lon,
                    d_alt,
                    vehicle.attitude.roll,
                    vehicle.attitude.pitch,
                    vehicle.attitude.yaw,
                    ground_cx,
                    ground_cy,
                )
            except Exception as exc:
                SwarmLogger.log_throttled(
                    "WARNING",
                    f"DRONE_{self.port or '?'}",
                    f"Group ground projection failed: {exc}",
                    "TRACKING",
                    interval_s=2.0,
                )
                continue
            if rgi is None or len(rgi) < 2 or rgi[0] is None:
                continue

            track_key = f"group:{det.group_id}" if det.group_id is not None else f"stable:{det.track_id}"
            det.lat, det.lon, det.covariance = self._stabilize_world_observation(
                track_key,
                float(rgi[0]),
                float(rgi[1]),
                rgi[2] if len(rgi) > 2 else None,
                bbox_center=det.bbox_center,
                bbox_size=det.bbox_size,
                attack_mode=attack_mode,
            )
            projected.append(det)
        return projected

    def _stabilize_world_observation(
        self,
        track_key: str,
        lat: float,
        lon: float,
        covariance,
        bbox_center: Optional[Tuple[float, float]] = None,
        bbox_size: Optional[Tuple[float, float]] = None,
        attack_mode: bool = False,
    ) -> Tuple[float, float, np.ndarray]:
        """Track world position with a low-lag alpha-beta filter."""
        now = time.time()
        cov = np.array(covariance, dtype=np.float64) if covariance is not None else np.eye(3, dtype=np.float64) * 25.0
        if cov.ndim != 2 or cov.shape[0] < 3 or cov.shape[1] < 3:
            padded = np.eye(3, dtype=np.float64) * 25.0
            s0 = min(3, cov.shape[0]) if hasattr(cov, "shape") else 0
            s1 = min(3, cov.shape[1]) if hasattr(cov, "shape") else 0
            if s0 and s1:
                padded[:s0, :s1] = cov[:s0, :s1]
            cov = padded
        cov = 0.5 * (cov + cov.T)
        for i in range(3):
            if not np.isfinite(cov[i, i]) or cov[i, i] <= 0.0:
                cov[i, i] = 25.0

        state = self._world_track_state.get(track_key)
        if attack_mode and bbox_center is not None and bbox_size is not None:
            bbox_area = max(1.0, float(bbox_size[0]) * float(bbox_size[1]))
            best_key = track_key
            best_state = state
            best_score = float("inf") if state is None else -1.0
            for candidate_key, candidate_state in self._world_track_state.items():
                age = now - float(candidate_state.get("ts", 0.0) or 0.0)
                if age > _WORLD_TRACK_STALE_S:
                    continue
                cand_center = candidate_state.get("bbox_center")
                cand_area = float(candidate_state.get("bbox_area", 0.0) or 0.0)
                if cand_center is None or cand_area <= 0.0:
                    continue
                dist_px = math.hypot(
                    float(bbox_center[0]) - float(cand_center[0]),
                    float(bbox_center[1]) - float(cand_center[1]),
                )
                if dist_px > 220.0:
                    continue
                size_ratio = min(bbox_area, cand_area) / max(bbox_area, cand_area)
                if size_ratio < 0.22:
                    continue
                score = dist_px + (1.0 - size_ratio) * 140.0
                if state is None or score < best_score:
                    best_score = score
                    best_key = candidate_key
                    best_state = candidate_state
            if best_key != track_key and best_state is not None:
                SwarmLogger.log_throttled(
                    "INFO",
                    f"DRONE_{self.port}",
                    f"ATTACK WORLD ANCHOR: {track_key} -> {best_key}",
                    "VISION",
                    interval_s=1.0,
                )
                track_key = best_key
                state = best_state

        if state is None or (now - float(state.get("ts", 0.0))) > _WORLD_TRACK_STALE_S:
            self._world_track_state[track_key] = {
                "lat": float(lat),
                "lon": float(lon),
                "vn": 0.0,
                "ve": 0.0,
                "ts": now,
                "bbox_center": tuple(float(v) for v in bbox_center) if bbox_center is not None else None,
                "bbox_area": max(1.0, float(bbox_size[0]) * float(bbox_size[1])) if bbox_size is not None else 0.0,
            }
            return float(lat), float(lon), cov

        prev_lat = float(state["lat"])
        prev_lon = float(state["lon"])
        prev_vn = float(state.get("vn", 0.0) or 0.0)
        prev_ve = float(state.get("ve", 0.0) or 0.0)
        dt = max(1e-3, now - float(state.get("ts", now)))

        if attack_mode:
            max_step_m = _ATTACK_WORLD_MAX_STEP_BASE_M + _ATTACK_WORLD_MAX_SPEED_MPS * dt
            alpha = _ATTACK_WORLD_FILTER_ALPHA
            beta = _ATTACK_WORLD_FILTER_BETA
        else:
            max_step_m = _WORLD_MAX_STEP_BASE_M + _WORLD_MAX_SPEED_MPS * dt
            alpha = _WORLD_FILTER_ALPHA
            beta = _WORLD_FILTER_BETA

        pred_lat = prev_lat + (prev_vn * dt) / 111320.0
        pred_lon = prev_lon + (prev_ve * dt) / max(
            111320.0 * math.cos(math.radians(max(min(prev_lat, 89.9), -89.9))),
            1e-6,
        )

        d_north = (float(lat) - pred_lat) * 111320.0
        d_east = (float(lon) - pred_lon) * max(
            111320.0 * math.cos(math.radians(max(min(pred_lat, 89.9), -89.9))),
            1e-6,
        )
        innovation_m = math.hypot(d_north, d_east)

        if innovation_m > max_step_m and innovation_m > 1e-3:
            ratio = max_step_m / innovation_m
            d_north *= ratio
            d_east *= ratio

        motion_ratio = min(1.0, innovation_m / max(max_step_m, 1e-3))
        alpha = min(0.94, alpha + motion_ratio * (_ATTACK_WORLD_FILTER_ALPHA_BOOST if attack_mode else _WORLD_FILTER_ALPHA_BOOST))

        corr_north = alpha * d_north
        corr_east = alpha * d_east
        stable_lat = pred_lat + corr_north / 111320.0
        stable_lon = pred_lon + corr_east / max(
            111320.0 * math.cos(math.radians(max(min(pred_lat, 89.9), -89.9))),
            1e-6,
        )
        stable_vn = prev_vn + (beta / dt) * d_north
        stable_ve = prev_ve + (beta / dt) * d_east
        self._world_track_state[track_key] = {
            "lat": stable_lat,
            "lon": stable_lon,
            "vn": stable_vn,
            "ve": stable_ve,
            "ts": now,
            "bbox_center": tuple(float(v) for v in bbox_center) if bbox_center is not None else state.get("bbox_center"),
            "bbox_area": max(1.0, float(bbox_size[0]) * float(bbox_size[1])) if bbox_size is not None else float(state.get("bbox_area", 0.0) or 0.0),
        }
        return stable_lat, stable_lon, cov

    def _select_group_representative(self, grp, member_dets: List[DetectionResult]) -> DetectionResult:
        rep_id = self._group_representatives.get(str(grp.group_id))
        if rep_id is not None:
            for det in member_dets:
                if det.track_id is not None and int(det.track_id) == int(rep_id):
                    if det.bbox_center is not None:
                        self._group_rep_centers[str(grp.group_id)] = (
                            float(det.bbox_center[0]),
                            float(det.bbox_center[1]),
                        )
                    return det
        prev_center = self._group_rep_centers.get(str(grp.group_id))
        if prev_center is not None:
            rep_det = min(
                member_dets,
                key=lambda det: (
                    (det.bbox_center[0] - prev_center[0]) ** 2 +
                    (det.bbox_center[1] - prev_center[1]) ** 2
                ) if det.bbox_center else 1e9
            )
        else:
            rep_det = min(
                member_dets,
                key=lambda det: (
                    (det.bbox_center[0] - grp.center_px[0]) ** 2 +
                    (det.bbox_center[1] - grp.center_px[1]) ** 2
                ) if det.bbox_center else 1e9
            )
        if rep_det.track_id is not None:
            self._group_representatives[str(grp.group_id)] = int(rep_det.track_id)
        if rep_det.bbox_center is not None:
            self._group_rep_centers[str(grp.group_id)] = (
                float(rep_det.bbox_center[0]),
                float(rep_det.bbox_center[1]),
            )
        return rep_det

    # ------------------------------------------------------------------
    def _rate_limited_log(self, detections: List[DetectionResult]):
        """Log detection changes without flooding the log."""
        now = time.time()

        if not detections:
            if self._prev_det_ids:
                SwarmLogger.log("CAMERA_STATS", f"DRONE_{self.port}",
                                "CAMERA: 0 bbox | 0 group | 0 ID (all lost)", "VISION")
                self._prev_det_ids = set()
                self._prev_det_count = 0
                self._log_ts = now
            return

        current_ids = {d.track_id for d in detections if d.track_id is not None}
        current_count = len(detections)

        count_changed = current_count != self._prev_det_count
        ids_changed = current_ids != self._prev_det_ids

        new_ids = current_ids - self._prev_det_ids if self._prev_det_ids else set()
        lost_ids = self._prev_det_ids - current_ids if self._prev_det_ids else set()

        is_change = count_changed or ids_changed
        is_periodic = (now - self._log_ts >= 10.0)

        if not is_change and not is_periodic:
            return

        self._log_ts = now
        self._prev_det_count = current_count
        self._prev_det_ids = current_ids

        group_dets = [d for d in detections if d.is_group]
        group_count = len(group_dets)
        total_group_members = sum(d.group_member_count for d in group_dets) if group_dets else 0

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
        
        avg_sigma = sum(sigma_values) / len(sigma_values) if sigma_values else 0.0
        max_sigma = max(sigma_values) if sigma_values else 0.0

        SwarmLogger.log(
            "CAMERA_STATS", f"DRONE_{self.port}",
            f"CAMERA: {len(detections)} bbox | {group_count} group ({total_group_members} members) | "
            f"IDs: {sorted(current_ids)} | avg_σ={avg_sigma:.1f}m max_σ={max_sigma:.1f}m",
            "VISION"
        )

        det_preview = " | ".join(d.log_summary() for d in detections[:4])
        if len(detections) > 4:
            det_preview += " | ..."
        SwarmLogger.log_throttled(
            "CAMERA_FLOW", f"DRONE_{self.port}",
            f"DETECTION_FLOW count={len(detections)} groups={group_count} "
            f"stable_ids={sorted(current_ids)} :: {det_preview}",
            "VISION",
            interval_s=5.0,
        )

        if is_change and gps_summaries:
            SwarmLogger.log(
                "CAMERA_STATS", f"DRONE_{self.port}",
                f"GPS_DETAIL: {' | '.join(gps_summaries[:5])}" + (" ..." if len(gps_summaries) > 5 else ""),
                "VISION"
            )

        change_note = ""
        if is_change and self._prev_det_ids:
            parts = []
            if new_ids:
                parts.append(f"+{','.join(str(i) for i in sorted(new_ids))}")
            if lost_ids:
                parts.append(f"-{','.join(str(i) for i in sorted(lost_ids))}")
            if parts:
                change_note = f" | CHG: {' '.join(parts)}"

        if group_dets:
            group_summary = "GROUPS: " + " ".join(
                f"[G{d.group_id} x{d.group_member_count}]" for d in group_dets
            )
            SwarmLogger.log(
                "CAMERA_STATS", f"DRONE_{self.port}",
                group_summary + change_note, "VISION"
            )
