"""Group clustering and group bbox smoothing."""

import time
import math
import numpy as np
from typing import List, Dict, Tuple
from dataclasses import dataclass, field

from config import (
    BBOX_SMOOTH_ALPHA,
    BBOX_SIZE_STABILITY_THRESHOLD,
    GROUP_CLUSTER_EPS_M,
    GROUP_CLUSTER_MIN_SAMPLES,
    GROUP_PERSISTENCE_SEC,
    METERS_PER_DEGREE_LAT,
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
)
from modules.core.geo_math import GeoMath
from modules.core.runtime import monotonic_now


class GroupBboxSmoother:
    """Adaptive EMA for group bbox centers."""

    def __init__(self, alpha: float = BBOX_SMOOTH_ALPHA):
        self.alpha = alpha
        self._history: Dict[object, dict] = {}
        self._last_cleanup = monotonic_now()
        self._cleanup_interval = 30.0

    def smooth(self, track_id, cx: float, cy: float, _conf: float, w: float, h: float) -> Tuple[float, float]:
        if track_id is None:
            return cx, cy
        self._cleanup_if_needed()
        if track_id not in self._history:
            self._history[track_id] = {
                "smoothed_cx": cx,
                "smoothed_cy": cy,
                "last_w": w,
                "last_h": h,
                "last_update": monotonic_now(),
            }
            return cx, cy

        entry = self._history[track_id]
        size_ratio_change = self._calc_size_ratio_change(entry["last_w"], entry["last_h"], w, h)
        if size_ratio_change > BBOX_SIZE_STABILITY_THRESHOLD:
            entry["smoothed_cx"] = cx
            entry["smoothed_cy"] = cy
            entry["last_w"] = w
            entry["last_h"] = h
            entry["last_update"] = monotonic_now()
            return cx, cy

        dx = float(cx) - float(entry["smoothed_cx"])
        dy = float(cy) - float(entry["smoothed_cy"])
        motion_px = math.hypot(dx, dy)
        scale_px = max(20.0, float(max(w, h)))
        motion_ratio = min(1.0, motion_px / scale_px)
        effective_alpha = min(0.88, self.alpha + motion_ratio * 0.42)

        smoothed_cx = entry["smoothed_cx"] + effective_alpha * dx
        smoothed_cy = entry["smoothed_cy"] + effective_alpha * dy
        entry["smoothed_cx"] = smoothed_cx
        entry["smoothed_cy"] = smoothed_cy
        entry["last_w"] = w
        entry["last_h"] = h
        entry["last_update"] = monotonic_now()
        return smoothed_cx, smoothed_cy

    def reset_all(self) -> None:
        self._history.clear()

    @staticmethod
    def _calc_size_ratio_change(old_w: float, old_h: float, new_w: float, new_h: float) -> float:
        if old_w <= 0 or old_h <= 0:
            return 0.0
        w_ratio = max(new_w / old_w, old_w / new_w) - 1.0
        h_ratio = max(new_h / old_h, old_h / new_h) - 1.0
        return max(w_ratio, h_ratio)

    def _cleanup_if_needed(self) -> None:
        now = monotonic_now()
        if now - self._last_cleanup < self._cleanup_interval:
            return
        self._last_cleanup = now
        expired = [track_id for track_id, entry in self._history.items() if now - entry["last_update"] > 10.0]
        for track_id in expired:
            del self._history[track_id]


@dataclass
class TargetGroup:
    """Unified group output."""
    group_id: str
    center_gps: Tuple[float, float]
    center_px: Tuple[float, float]
    member_ids: List[int]
    member_count: int
    bounding_rect_px: Tuple[int, int, int, int]
    avg_confidence: float
    avg_bbox_area: float = 0.0
    member_candidates: List[dict] = field(default_factory=list)

@dataclass
class GroupResult:
    """Output of GroupClusterEngine.update()."""
    groups: List[TargetGroup]
    singles: List[dict] = field(default_factory=list)
    all_targets: List[TargetGroup] = field(default_factory=list)

@dataclass
class _PersistentGroup:
    """Persistent group state."""
    group_id: str
    center_enu: np.ndarray
    center_gps: Tuple[float, float]
    center_px: Tuple[float, float]
    member_ids: List[int]
    member_count: int
    bounding_rect_px: Tuple[int, int, int, int]
    last_seen: float
    avg_bbox_area: float = 0.0
    miss_frames: int = 0

class GroupClusterEngine:
    """DBSCAN-based group clustering."""

    _GRACE_FRAMES = 8
    _MIN_BBOX_AREA = 100.0
    _REF_BBOX_AREA = 3000.0

    def __init__(self):
        self._group_counter = 0
        self._prev_groups: Dict[str, _PersistentGroup] = {}

    def update(
        self,
        candidates: List[dict],
        drone_lat: float,
        drone_lon: float,
        drone_alt: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> GroupResult:
        """Update group state for one frame."""
        now = monotonic_now()

        if not candidates or drone_alt < 0.5:
            self._tick_grace(now)
            grace_groups = self._get_grace_groups(now)
            return GroupResult(
                groups=[g for g in grace_groups if g.member_count >= 2],
                singles=[],
                all_targets=grace_groups,
            )

        valid_candidates = self._filter_candidates(candidates)
        if not valid_candidates:
            self._tick_grace(now)
            grace_groups = self._get_grace_groups(now)
            return GroupResult(
                groups=[g for g in grace_groups if g.member_count >= 2],
                singles=[],
                all_targets=grace_groups,
            )

        enu_points, gps_points, valid_indices = self._project_to_enu(
            valid_candidates, drone_lat, drone_lon, drone_alt, roll, pitch, yaw
        )

        if not enu_points:
            self._tick_grace(now)
            grace_groups = self._get_grace_groups(now)
            return GroupResult(
                groups=[g for g in grace_groups if g.member_count >= 2],
                singles=[],
                all_targets=grace_groups,
            )

        avg_area = np.mean([self._candidate_area(valid_candidates[i]) for i in valid_indices])
        merge_eps = self._dynamic_merge_eps(avg_area)

        enu_array = np.array(enu_points)
        all_groups: List[TargetGroup] = []

        if len(enu_points) >= GROUP_CLUSTER_MIN_SAMPLES:
            from sklearn.cluster import DBSCAN
            labels = DBSCAN(
                eps=merge_eps,
                min_samples=GROUP_CLUSTER_MIN_SAMPLES,
                metric='euclidean',
            ).fit_predict(enu_array)

            cluster_map: Dict[int, List[int]] = {}
            noise_indices: List[int] = []
            for i, label in enumerate(labels):
                if label == -1:
                    noise_indices.append(i)
                else:
                    cluster_map.setdefault(label, []).append(i)

            for label, indices in cluster_map.items():
                tg = self._build_target_group(
                    indices, valid_indices, valid_candidates, gps_points, enu_points
                )
                all_groups.append(tg)

            for i in noise_indices:
                tg = self._build_target_group(
                    [i], valid_indices, valid_candidates, gps_points, enu_points
                )
                all_groups.append(tg)
        else:
            for i in range(len(enu_points)):
                tg = self._build_target_group(
                    [i], valid_indices, valid_candidates, gps_points, enu_points
                )
                all_groups.append(tg)

        matched_groups = self._match_and_assign_ids(all_groups, enu_points, valid_indices, now)

        grace_adds = self._get_grace_groups(now)
        current_gids = {g.group_id for g in matched_groups}
        for gg in grace_adds:
            if gg.group_id not in current_gids:
                matched_groups.append(gg)

        self._purge_stale(now)
        multi_groups = [g for g in matched_groups if g.member_count >= 2]

        return GroupResult(
            groups=multi_groups,
            singles=[],
            all_targets=matched_groups,
        )

    def _dynamic_merge_eps(self, avg_bbox_area: float) -> float:
        """Scale clustering distance by bbox area."""
        if avg_bbox_area <= 0:
            return GROUP_CLUSTER_EPS_M

        ratio = self._REF_BBOX_AREA / max(avg_bbox_area, self._MIN_BBOX_AREA)
        scale = math.sqrt(ratio)
        eps = max(2.0, min(15.0, GROUP_CLUSTER_EPS_M * scale))
        return eps

    def _filter_candidates(self, candidates: List[dict]) -> List[dict]:
        """Keep valid bbox candidates only."""
        result = []
        for c in candidates:
            w, h = c.get('dims', (0, 0))
            if w * h < self._MIN_BBOX_AREA:
                continue
            if c.get('center') is None:
                continue
            result.append(c)
        return result

    @staticmethod
    def _candidate_area(c: dict) -> float:
        w, h = c.get('dims', (1, 1))
        return max(1.0, float(w) * float(h))

    def _project_to_enu(self, candidates, drone_lat, drone_lon, drone_alt, roll, pitch, yaw):
        """Project candidates into GPS and ENU."""
        enu_points = []
        gps_points = []
        valid_indices = []

        for i, c in enumerate(candidates):
            cx, cy = c.get('ground_point', c['center'])
            try:
                result = GeoMath.ray_ground_intersection(
                    drone_lat, drone_lon, drone_alt,
                    roll, pitch, yaw,
                    cx, cy,
                )
            except Exception:
                continue

            if result is None or len(result) < 2 or result[0] is None:
                continue

            t_lat, t_lon = result[0], result[1]

            d_north = (t_lat - drone_lat) * METERS_PER_DEGREE_LAT
            d_east = (t_lon - drone_lon) * METERS_PER_DEGREE_LAT * math.cos(math.radians(drone_lat))

            enu_points.append([d_east, d_north])
            gps_points.append((t_lat, t_lon))
            valid_indices.append(i)

        return enu_points, gps_points, valid_indices

    def _build_target_group(
        self,
        member_indices: List[int],
        valid_indices: List[int],
        candidates: List[dict],
        gps_points: List[Tuple[float, float]],
        enu_points: List[list],
    ) -> TargetGroup:
        """Build one target group from candidate indices."""
        member_cands = [candidates[valid_indices[i]] for i in member_indices]
        member_ids = [c['id'] for c in member_cands]

        total_conf = sum(c.get('conf', 1.0) for c in member_cands) or 1.0
        lat_c = sum(
            gps_points[i][0] * candidates[valid_indices[i]].get('conf', 1.0)
            for i in member_indices
        ) / total_conf
        lon_c = sum(
            gps_points[i][1] * candidates[valid_indices[i]].get('conf', 1.0)
            for i in member_indices
        ) / total_conf

        all_xyxy = [candidates[valid_indices[i]].get('box_xyxy') for i in member_indices]
        all_xyxy = [b for b in all_xyxy if b is not None and len(b) == 4]

        if all_xyxy:
            x1 = int(min(float(b[0]) for b in all_xyxy))
            y1 = int(min(float(b[1]) for b in all_xyxy))
            x2 = int(max(float(b[2]) for b in all_xyxy))
            y2 = int(max(float(b[3]) for b in all_xyxy))
            px_cx = (x1 + x2) / 2.0
            px_cy = (y1 + y2) / 2.0
        else:
            centers = [c['center'] for c in member_cands]
            px_cx = sum(c[0] for c in centers) / len(centers)
            px_cy = sum(c[1] for c in centers) / len(centers)
            x1 = y1 = 0
            x2 = int(CAMERA_WIDTH)
            y2 = int(CAMERA_HEIGHT)

        avg_conf = total_conf / max(1, len(member_cands))
        avg_area = np.mean([self._candidate_area(c) for c in member_cands])

        self._group_counter += 1
        gid = f"G{self._group_counter}"

        return TargetGroup(
            group_id=gid,
            center_gps=(lat_c, lon_c),
            center_px=(px_cx, px_cy),
            member_ids=member_ids,
            member_count=len(member_ids),
            bounding_rect_px=(x1, y1, x2, y2),
            avg_confidence=avg_conf,
            avg_bbox_area=avg_area,
            member_candidates=member_cands,
        )

    def _match_and_assign_ids(
        self,
        new_groups: List[TargetGroup],
        enu_points: List[list],
        valid_indices: List[int],
        now: float,
    ) -> List[TargetGroup]:
        """Match groups against previous frame state."""
        if not new_groups:
            return []

        used_prev = set()
        result: List[TargetGroup] = []

        sorted_groups = sorted(new_groups, key=lambda g: g.member_count, reverse=True)

        for tg in sorted_groups:
            best_id = None
            best_cost = float('inf')

            for gid, pg in self._prev_groups.items():
                if gid in used_prev:
                    continue

                tg_enu = self._gps_to_enu_approx(tg.center_gps, enu_points, valid_indices)
                dist = np.linalg.norm(tg_enu - pg.center_enu)
                overlap = len(set(tg.member_ids) & set(pg.member_ids))
                overlap_bonus = overlap * 0.5
                count_penalty = abs(int(tg.member_count or 0) - int(pg.member_count or 0)) * 1.8
                grace_penalty = min(6.0, max(0, int(pg.miss_frames or 0)) * 0.8)
                persistence_bonus = max(0.0, 2.5 - min(2.5, now - float(pg.last_seen or now)))

                cost = dist + count_penalty + grace_penalty - overlap_bonus - persistence_bonus

                if cost < best_cost:
                    best_cost = cost
                    best_id = gid

            max_match = self._dynamic_merge_eps(tg.avg_bbox_area) * 3
            if best_id is not None and best_cost < max_match:
                tg.group_id = best_id
                used_prev.add(best_id)
            tg_enu = self._gps_to_enu_approx(tg.center_gps, enu_points, valid_indices)
            self._prev_groups[tg.group_id] = _PersistentGroup(
                group_id=tg.group_id,
                center_enu=tg_enu,
                center_gps=tg.center_gps,
                center_px=tg.center_px,
                member_ids=tg.member_ids,
                member_count=tg.member_count,
                bounding_rect_px=tg.bounding_rect_px,
                last_seen=now,
                avg_bbox_area=tg.avg_bbox_area,
                miss_frames=0,
            )
            result.append(tg)

        return result

    @staticmethod
    def _gps_to_enu_approx(
        gps: Tuple[float, float],
        enu_points: List[list],
        valid_indices: List[int],
    ) -> np.ndarray:
        """Approximate GPS in a stable local metric frame."""
        if gps is None or len(gps) < 2:
            return np.array([0.0, 0.0])
        lat, lon = float(gps[0]), float(gps[1])
        north = lat * METERS_PER_DEGREE_LAT
        east = lon * METERS_PER_DEGREE_LAT * math.cos(math.radians(lat))
        return np.array([north, east], dtype=np.float64)

    def _tick_grace(self, now: float):
        """Advance miss counters for unseen groups."""
        for pg in self._prev_groups.values():
            if (now - pg.last_seen) > 0.1:
                pg.miss_frames += 1

    def _get_grace_groups(self, now: float) -> List[TargetGroup]:
        """Return groups still covered by grace time."""
        result = []
        for pg in self._prev_groups.values():
            if pg.miss_frames > 0 and pg.miss_frames <= self._GRACE_FRAMES:
                tg = TargetGroup(
                    group_id=pg.group_id,
                    center_gps=pg.center_gps,
                    center_px=pg.center_px,
                    member_ids=pg.member_ids,
                    member_count=pg.member_count,
                    bounding_rect_px=pg.bounding_rect_px,
                    avg_confidence=0.3,
                    avg_bbox_area=pg.avg_bbox_area,
                )
                result.append(tg)
        return result

    def _purge_stale(self, now: float):
        """Drop expired groups."""
        stale = [
            gid for gid, pg in self._prev_groups.items()
            if pg.miss_frames > self._GRACE_FRAMES
            or (now - pg.last_seen) > GROUP_PERSISTENCE_SEC
        ]
        for gid in stale:
            del self._prev_groups[gid]
