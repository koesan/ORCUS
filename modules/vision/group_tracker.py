"""Group Clustering Engine - DBSCAN on GPS/ENU for grouping detections."""

import time
import math
import numpy as np
from typing import List, Dict, Tuple, Optional, NamedTuple
from dataclasses import dataclass, field
from sklearn.cluster import DBSCAN

from config import (
    GROUP_CLUSTER_EPS_M,
    GROUP_CLUSTER_MIN_SAMPLES,
    GROUP_PERSISTENCE_SEC,
    GROUP_MERGE_HYSTERESIS,
    GROUP_SPLIT_HYSTERESIS,
    GROUP_BBOX_SCALE_RATIO_MAX,
    GROUP_VISUAL_COLOR_BGR,
    METERS_PER_DEGREE_LAT,
)
from modules.core.geo_math import GeoMath
from modules.core.logger import SwarmLogger


@dataclass
class GroupInfo:
    """Detected group of people."""
    group_id: str
    center_gps: Tuple[float, float]
    center_px: Tuple[float, float]
    member_ids: List[int]
    member_count: int
    bounding_rect_px: Tuple[int, int, int, int]
    avg_confidence: float
    member_candidates: List[dict] = field(default_factory=list)


@dataclass
class GroupResult:
    """Output of GroupClusterEngine.update()."""
    groups: List[GroupInfo]
    singles: List[dict]


@dataclass
class _PersistentGroup:
    """Internal bookkeeping for persistent group."""
    group_id: str
    center_enu: np.ndarray
    center_gps: Tuple[float, float]
    member_ids: List[int]
    last_seen: float
    merge_counter: int = 0
    split_counter: int = 0


class GroupClusterEngine:
    """Clusters YOLO detections into groups using DBSCAN on ENU metres."""

    def __init__(self):
        self._group_counter = 0
        self._prev_groups: Dict[str, _PersistentGroup] = {}
        self._pending_merges: Dict[str, int] = {}
        self._pending_splits: Dict[str, int] = {}

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
        """Main entry point. Runs every frame after BoT-SORT."""
        now = time.time()

        if not candidates or drone_alt < 0.5:
            self._purge_stale(now)
            return GroupResult(groups=[], singles=list(candidates))

        # Project candidates to GPS & ENU
        enu_points, gps_points, valid_indices = self._project_to_enu(
            candidates, drone_lat, drone_lon, drone_alt, roll, pitch, yaw
        )

        if len(enu_points) < GROUP_CLUSTER_MIN_SAMPLES:
            self._purge_stale(now)
            return GroupResult(groups=[], singles=list(candidates))

        # DBSCAN clustering
        enu_array = np.array(enu_points)
        labels = DBSCAN(
            eps=GROUP_CLUSTER_EPS_M,
            min_samples=GROUP_CLUSTER_MIN_SAMPLES,
            metric='euclidean',
        ).fit_predict(enu_array)

        # Build raw clusters
        cluster_map: Dict[int, List[int]] = {}
        for i, label in enumerate(labels):
            if label == -1:
                continue
            cluster_map.setdefault(label, []).append(i)

        # IoU + bbox-scale gating
        refined_groups: List[List[int]] = []
        for label, indices in cluster_map.items():
            refined = self._apply_iou_gating(indices, valid_indices, candidates)
            for sub_group in refined:
                if len(sub_group) >= GROUP_CLUSTER_MIN_SAMPLES:
                    refined_groups.append(sub_group)

        # Compute group info
        raw_group_infos: List[dict] = []
        for member_indices in refined_groups:
            info = self._compute_group_info(
                member_indices, valid_indices, candidates, gps_points, enu_points
            )
            raw_group_infos.append(info)

        # Match against previous frame
        matched = self._match_groups(raw_group_infos, now)

        # Merge/Split hysteresis
        final_groups = self._apply_hysteresis(matched, now)

        # Mark singles
        grouped_ids = set()
        for g in final_groups:
            grouped_ids.update(g.member_ids)
        singles = [c for c in candidates if c['id'] not in grouped_ids]

        self._purge_stale(now)

        return GroupResult(groups=final_groups, singles=singles)

    def _project_to_enu(
        self, candidates, drone_lat, drone_lon, drone_alt, roll, pitch, yaw
    ):
        """Project candidate pixel centres to GPS and local ENU."""
        enu_points = []
        gps_points = []
        valid_indices = []

        for i, c in enumerate(candidates):
            cx, cy = c.get('ground_point', c['center'])
            result = GeoMath.ray_ground_intersection(
                drone_lat, drone_lon, drone_alt,
                roll, pitch, yaw,
                cx, cy,
            )
            if result[0] is None:
                continue

            t_lat, t_lon = result[0], result[1]

            d_north = (t_lat - drone_lat) * METERS_PER_DEGREE_LAT
            d_east = (t_lon - drone_lon) * METERS_PER_DEGREE_LAT * math.cos(math.radians(drone_lat))

            enu_points.append([d_east, d_north])
            gps_points.append((t_lat, t_lon))
            valid_indices.append(i)

        return enu_points, gps_points, valid_indices

    def _apply_iou_gating(
        self,
        cluster_indices: List[int],
        valid_indices: List[int],
        candidates: List[dict],
    ) -> List[List[int]]:
        """Refine cluster by splitting members with different bbox scales."""
        keep = list(cluster_indices)
        removed = []

        for i in range(len(keep)):
            ci = candidates[valid_indices[keep[i]]]
            wi, hi = ci['dims']
            area_i = wi * hi
            for j in range(i + 1, len(keep)):
                cj = candidates[valid_indices[keep[j]]]
                wj, hj = cj['dims']
                area_j = wj * hj

                if area_i > 0 and area_j > 0:
                    ratio = max(area_i, area_j) / max(min(area_i, area_j), 1)
                    if ratio > GROUP_BBOX_SCALE_RATIO_MAX ** 2:
                        if area_i < area_j:
                            if keep[i] not in removed:
                                removed.append(keep[i])
                        else:
                            if keep[j] not in removed:
                                removed.append(keep[j])

        final = [idx for idx in keep if idx not in removed]
        return [final] if len(final) >= GROUP_CLUSTER_MIN_SAMPLES else []

    def _compute_group_info(
        self,
        member_indices: List[int],
        valid_indices: List[int],
        candidates: List[dict],
        gps_points,
        enu_points,
    ) -> dict:
        """Compute centre, bounding rect, etc. for a raw cluster."""
        member_cands = [candidates[valid_indices[i]] for i in member_indices]
        member_ids = [c['id'] for c in member_cands]

        total_conf = sum(c['conf'] for c in member_cands) or 1.0
        lat_c = sum(gps_points[i][0] * candidates[valid_indices[i]]['conf'] for i in member_indices) / total_conf
        lon_c = sum(gps_points[i][1] * candidates[valid_indices[i]]['conf'] for i in member_indices) / total_conf

        enu_c = np.mean([enu_points[i] for i in member_indices], axis=0)

        all_xyxy = [candidates[valid_indices[i]]['box_xyxy'] for i in member_indices]
        x1 = int(min(b[0] for b in all_xyxy))
        y1 = int(min(b[1] for b in all_xyxy))
        x2 = int(max(b[2] for b in all_xyxy))
        y2 = int(max(b[3] for b in all_xyxy))
        px_cx = (x1 + x2) / 2.0
        px_cy = (y1 + y2) / 2.0

        avg_conf = total_conf / len(member_cands)

        return {
            'center_gps': (lat_c, lon_c),
            'center_px': (px_cx, px_cy),
            'center_enu': enu_c,
            'member_ids': member_ids,
            'member_count': len(member_ids),
            'bounding_rect_px': (x1, y1, x2, y2),
            'avg_confidence': avg_conf,
            'member_candidates': member_cands,
        }

    def _match_groups(self, raw_infos: List[dict], now: float) -> List[GroupInfo]:
        """Match raw clusters to previous frame's groups using greedy matching."""
        used_prev = set()
        result: List[GroupInfo] = []

        raw_infos_sorted = sorted(raw_infos, key=lambda g: g['member_count'], reverse=True)

        for info in raw_infos_sorted:
            best_id = None
            best_cost = float('inf')

            for gid, pg in self._prev_groups.items():
                if gid in used_prev:
                    continue

                dist = np.linalg.norm(info['center_enu'] - pg.center_enu)
                overlap = len(set(info['member_ids']) & set(pg.member_ids))
                overlap_bonus = overlap * 0.5

                cost = dist - overlap_bonus

                if cost < best_cost:
                    best_cost = cost
                    best_id = gid

            MAX_MATCH_DIST = GROUP_CLUSTER_EPS_M * 3
            if best_id is not None and best_cost < MAX_MATCH_DIST:
                gid = best_id
                used_prev.add(gid)
            else:
                self._group_counter += 1
                gid = f"G{self._group_counter}"

            gi = GroupInfo(
                group_id=gid,
                center_gps=info['center_gps'],
                center_px=info['center_px'],
                member_ids=info['member_ids'],
                member_count=info['member_count'],
                bounding_rect_px=info['bounding_rect_px'],
                avg_confidence=info['avg_confidence'],
                member_candidates=info['member_candidates'],
            )
            result.append(gi)

            self._prev_groups[gid] = _PersistentGroup(
                group_id=gid,
                center_enu=info['center_enu'],
                center_gps=info['center_gps'],
                member_ids=info['member_ids'],
                last_seen=now,
            )

        return result

    def _apply_hysteresis(self, groups: List[GroupInfo], now: float) -> List[GroupInfo]:
        """Prevent rapid merge/split flickering."""
        current_gids = {g.group_id for g in groups}
        for gid, pg in self._prev_groups.items():
            if gid not in current_gids:
                pg.split_counter += 1
            else:
                pg.split_counter = 0

        return groups

    def _purge_stale(self, now: float):
        """Remove groups not seen for GROUP_PERSISTENCE_SEC."""
        stale = [
            gid for gid, pg in self._prev_groups.items()
            if (now - pg.last_seen) > GROUP_PERSISTENCE_SEC
        ]
        for gid in stale:
            SwarmLogger.log("DEBUG", "GROUP", f"Group {gid} expired", "CLUSTER")
            del self._prev_groups[gid]
