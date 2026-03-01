"""Swarm Manager - Battlespace Leader for swarm intelligence and target tracking."""

import math
import time
import threading
import numpy as np
from typing import Dict, List, Optional, Tuple

import config
from modules.core.logger import SwarmLogger
from modules.core.geo_math import GeoMath
from modules.swarm.target_fusion import TargetFusionManager, weighted_covariance_fusion, should_fuse_targets
from config import (
    SWARM_MERGE_DISTANCE_M, SWARM_ATTACK_MIN_CONFIDENCE, SWARM_BIAS_DISTANCE_M, 
    ASSIGNMENT_LOOP_INTERVAL_S, TARGET_COAST_TIME_S, TARGET_PRUNE_TIME_S,
    SWARM_GHOST_PRUNE_TIME_S, SWARM_ZOMBIE_SWITCH_TIME_S, SWARM_ALIGNMENT_TOLERANCE_M,
    TRACK_CONFIRMATION_TIME_SEC, TRACK_MIN_OBSERVATIONS, TRACK_MIN_OBSERVATIONS_SEARCH,
    TRACK_TENTATIVE_TIMEOUT_SEC,
    TRACK_LOST_TIMEOUT_SEC, TRACK_DELETE_TIMEOUT_SEC,
    ENABLE_SPATIAL_FALLBACK, TRACKER_ID_ALPHA,
    DUPLICATE_MERGE_DISTANCE_M, DUPLICATE_VELOCITY_SIMILARITY_MPS,
    LEADER_TIMEOUT_S,
    ATTACK_STALE_ENGAGED_TIMEOUT_S,
    ENABLE_TARGET_COLLAPSE, ENABLE_CAPACITY_GUARD, ENABLE_ATTACK_IMMUTABILITY,
    HANDOFF_CONSECUTIVE_FRAMES, HANDOFF_QUALITY_RATIO, HANDOFF_COOLDOWN_S,
    MERGE_DISTANCE_BASE_M, MERGE_DISTANCE_MAX_M, MERGE_DISTANCE_SIGMA_SCALE,
    SWARM_LOCAL_ID_INDEX_TTL_S, SWARM_LOCAL_ID_INDEX_MAX_JUMP_M, SWARM_FUSION_MAX_SIGMA_M
)

# Optional Dependency: Hungarian Algorithm
try:
    from scipy.optimize import linear_sum_assignment
    HUNGARIAN_AVAILABLE = True
except ImportError:
    HUNGARIAN_AVAILABLE = False
    SwarmLogger.log("WARNING", "SwarmCoord", "scipy not found, falling back to greedy assignment", "IMPORT")


class TrackedTarget:
    """Tracked Target with Track-Before-Detect lifecycle and ownership model."""
    # Ownership States
    OWNERSHIP_FREE = "FREE"              # No owner, any drone can claim
    OWNERSHIP_OWNED = "OWNED"            # Has active owner
    OWNERSHIP_RESERVED = "RESERVED"      # Owner lost temporarily, grace period
    OWNERSHIP_HANDOFF = "HANDOFF_PENDING"  # Handoff in progress
    
    def __init__(self, t_id, lat, lon, first_seen_time, tracker_id=None, covariance=None, source_drone_port=None):
        self.id = t_id
        self.lat = lat
        self.lon = lon
        self.first_seen_time = first_seen_time
        self.last_seen_time = first_seen_time
        self.assigned_drone_port = None
        self.status = "PENDING"  # PENDING, ENGAGED, LOST, LOCKED
        self.approved_for_attack = False
        self.tracker_id = tracker_id
        
        # Track-Before-Detect lifecycle
        self.track_state = "TENTATIVE"  # TENTATIVE → CONFIRMED → LOST → DELETED
        self.observation_count = 1
        self.last_observation_time = first_seen_time
        
        # Ownership Model
        self.ownership_state = self.OWNERSHIP_FREE
        self.owner_port = None
        self.owner_lost_time = None
        self.last_handoff_time = None
        self.handoff_candidate_port = None
        self.handoff_score_accumulator = {}  # port -> consecutive score
        
        # Secondary Observations
        self.secondary_observations: Dict[int, Dict] = {}  # port -> observation data
        
        # Quality Metrics
        self.observation_quality: Dict[int, float] = {}  # port -> quality score
        
        # Stability Metrics
        self.id_switch_count = 0
        self.drop_count = 0
        self.consecutive_obs = 1
        self._edge_accumulator = 0.0  # Fractional credits bucket
        
        # Local ID mapping per drone
        self.drone_local_ids = {} 
        if source_drone_port is not None and tracker_id is not None:
             self.drone_local_ids[source_drone_port] = tracker_id

        self.source_drone_port = source_drone_port
        self.covariance = covariance if covariance is not None else np.eye(3) * 25.0
        self.observation_sources: Dict[int, Tuple[float, float, float]] = {}
        self.local_id_hysteresis = {}  # port -> {"id": candidate_id, "count": int}
        
        # Group clustering
        self.is_group = False
        self.group_member_count = 0
        self.group_id_local = None

    def is_immutable(self) -> bool:
        """Check if target is in immutable state (LOCKED/ATTACK)."""
        return self.status in {"LOCKED", "ECHO_WAIT", "CONFIRMED_ATTACK", "CENTERING", "ATTACKING", "DIVING"}
    
    def can_handoff(self) -> bool:
        """Check if target can be handed off to another drone."""
        if self.is_immutable():
            return False
        if self.ownership_state == self.OWNERSHIP_FREE:
            return True
        # 5 second cooldown
        if self.last_handoff_time is not None:
            if time.time() - self.last_handoff_time < 5.0:
                return False
        return True
    
    def record_observation(self, drone_port: int, quality_score: float, is_owner: bool = False):
        """Record observation quality for handoff decisions."""
        self.observation_quality[drone_port] = quality_score
        self.last_observation_time = time.time()
        
        if not is_owner:
            self.secondary_observations[drone_port] = {'time': time.time(), 'quality': quality_score}
    
    def get_owner_quality(self) -> float:
        """Get quality score of current owner."""
        if self.owner_port is None:
            return 0.0
        return self.observation_quality.get(self.owner_port, 0.0)
    
    def get_best_secondary(self) -> Tuple[Optional[int], float]:
        """Get best secondary observer (port, quality)."""
        if not self.secondary_observations:
            return None, 0.0
        best_port = max(self.secondary_observations.keys(), 
                        key=lambda p: self.secondary_observations[p]['quality'])
        return best_port, self.secondary_observations[best_port]['quality']

    def update_position(self, lat, lon, covariance=None, is_edge=False):
        self.lat = lat
        self.lon = lon
        now = time.time()
        self.last_seen_time = now
        self.last_observation_time = now
        
        if is_edge and self.track_state == "TENTATIVE":
            # Fractional credit: 3 frames = 1 point
            self._edge_accumulator += config.SWARM_EDGE_FILTER_FRACTIONAL_CREDIT 
            if self._edge_accumulator >= 1.0:
                earned = int(self._edge_accumulator)
                self.observation_count += earned
                self.consecutive_obs += earned
                self._edge_accumulator -= earned
        else:
            self.observation_count += 1
            self.consecutive_obs += 1
            
        if covariance is not None:
            self.covariance = covariance
        
        # Re-acquire LOST tracks
        if self.track_state == "LOST":
            self.track_state = "CONFIRMED"
            self.drop_count += 1
            self.consecutive_obs = 1
            SwarmLogger.log("LIFECYCLE", "LEADER", f"{self.id}: LOST → CONFIRMED (re-acquired)", "TRACK")
        if self.status == "LOST":
            self.status = "ACTIVE"
    
    def check_confirmation(self, current_time, is_drone_searching: bool = True):
        """Check if TENTATIVE track should be promoted to CONFIRMED."""
        if self.track_state != "TENTATIVE":
            return False
            
        time_alive = current_time - self.first_seen_time
        required_obs = TRACK_MIN_OBSERVATIONS_SEARCH if is_drone_searching else TRACK_MIN_OBSERVATIONS
        
        if time_alive >= TRACK_CONFIRMATION_TIME_SEC and self.observation_count >= required_obs:
            self.track_state = "CONFIRMED"
            SwarmLogger.log("LIFECYCLE", "LEADER",
                f"{self.id}: TENTATIVE → CONFIRMED (t={time_alive:.2f}s, obs={self.observation_count})", "TRACK")
            return True
            
        return False


class SwarmManager:
    # Targets in these states are protected from merge/collapse/reassignment
    IMMUTABLE_STATES = frozenset({"LOCKED", "ECHO_WAIT", "CONFIRMED_ATTACK", "CENTERING", "ATTACKING", "DIVING"})

    def __init__(self, drone_manager):
        self.drone_manager = drone_manager
        self.targets: Dict[str, TrackedTarget] = {}  # target_id -> TrackedTarget
        self.drone_targets = {}
        self.target_aliases: Dict[str, str] = {}  # alias_id -> canonical_id
        
        # Local identity index: (drone_port, local_tracker_id) -> {"tid": str, "ts": float}
        self.local_identity_index: Dict[Tuple[int, int], Dict[str, object]] = {}
        
        # O(1) atomic tracking
        self.drone_active_target: Dict[int, Optional[str]] = {}  # port -> canonical_tid
        self.target_owner: Dict[str, Optional[int]] = {}  # canonical_tid -> port
        
        # Merge backoff and alias TTL
        self.merge_abort_backoff: Dict[Tuple[str, str], Dict[str, float]] = {}
        self.alias_ttl: Dict[str, Dict[str, object]] = {}
        
        self.attack_mode_active = False
        self.lock = threading.RLock()
        self.target_counter = 0
        
        # Immutable log rate-limit (prevent spam)
        self.immutable_log_ts: Dict[int, float] = {}  # port -> last_log_time
        self.immutable_log_target: Dict[int, str] = {}  # port -> last_target_id
        
        # Drone heartbeat tracking
        self.drone_heartbeat_ts: Dict[int, float] = {}
        
        # Geo-validation cooldowns
        self.drone_mismatch_cooldowns: Dict[int, Dict[str, float]] = {}  # port -> {target_id: timestamp}
        
        # Radar Log (rate-limited) - Liderin gördüğü tüm hedefler
        self._radar_log_last_ts = 0.0
        self._radar_log_interval = 3.0  # Log every 3 seconds
        
        # Target Fusion Manager
        self.fusion_manager = TargetFusionManager(fusion_threshold_m=SWARM_MERGE_DISTANCE_M)
        
        # Proactive Assignment Loop
        self._assignment_loop_running = True
        self._assignment_thread = threading.Thread(
            target=self._periodic_optimization,
            name="SwarmAssignmentLoop",
            daemon=True
        )
        
        SwarmLogger.init_log()
        SwarmLogger.log("INFO", "LEADER", "Battlespace Manager v3 (Proactive Assignment + Failsafe) Started.", "INIT")
        
        self._assignment_thread.start()

    def _log_radar_snapshot(self):
        """Log all targets visible to leader (radar view) - rate limited.
        Web UI ile AYNI veriyi loglar (LOST ve TTL alias exclude).
        """
        now = time.time()
        if now - self._radar_log_last_ts < self._radar_log_interval:
            return
        self._radar_log_last_ts = now
        
        if not self.targets:
            return
        
        target_summaries = []
        for t_id, target in self.targets.items():
            # WEB UI İLE TUTARLI EXCLUDE KURALLARI
            if target.track_state == "DELETED":
                continue
            if target.status == "LOST":
                continue  # Web UI'da görünmeyen hedefleri loglama
            if t_id in self.alias_ttl:
                continue  # TTL alias (birleştirilmiş hedef) - Web UI'da görünmez
            
            # Temel bilgiler
            local_ids = ",".join([f"{p}:{lid}" for p, lid in target.drone_local_ids.items()]) if target.drone_local_ids else "-"
            tracker_id = target.tracker_id if target.tracker_id else "-"
            
            # Konum
            lat_str = f"{target.lat:.6f}" if target.lat else "-"
            lon_str = f"{target.lon:.6f}" if target.lon else "-"
            
            # Durum bilgisi
            state = f"{target.track_state}/{target.status}/{target.ownership_state}"
            
            # Atanan drone
            assigned = f"D{target.assigned_drone_port}" if target.assigned_drone_port else "FREE"
            
            # Stabilite
            stability = f"obs={target.observation_count} consec={target.consecutive_obs} switch={target.id_switch_count}"
            
            # Grup bilgisi
            group_str = ""
            if target.is_group:
                group_str = f" GRP[{target.group_id_local}]x{target.group_member_count}"
            
            
            target_summaries.append(
                f"{t_id}[{state}] @{lat_str},{lon_str} | "
                f"TID:{tracker_id} Local:[{local_ids}] | {assigned} | {stability}{group_str}"
            )
        
        
        if target_summaries:
            SwarmLogger.log("RADAR", "LEADER", 
                f"TARGETS({len(target_summaries)}): " + " | ".join(target_summaries), "BATTLESPACE")
    
    def _cleanup_local_identity_index(self, now: Optional[float] = None):
        now = time.time() if now is None else float(now)
        ttl_s = SWARM_LOCAL_ID_INDEX_TTL_S
        if ttl_s <= 0:
            return
        to_del = []
        for k, v in self.local_identity_index.items():
            try:
                ts = float(v.get('ts', 0.0))
                tid = v.get('tid')
                if (now - ts) > ttl_s:
                    to_del.append(k)
                elif tid and tid not in self.targets:
                    to_del.append(k)
            except Exception:
                to_del.append(k)
        for k in to_del:
            self.local_identity_index.pop(k, None)

    def _update_local_identity_index(self, drone_port: int, tracker_id: int, tid: str, now: Optional[float] = None):
        if drone_port is None or tracker_id is None or not tid:
            return
        now = time.time() if now is None else float(now)
        try:
            key = (int(drone_port), int(tracker_id))
        except Exception:
            return
        self.local_identity_index[key] = {"tid": tid, "ts": now}

    def _generate_target_id(self):
        self.target_counter += 1
        return f"T{self.target_counter}"

    def is_target_in_attack_pipeline(self, target):
        """Single Source of Truth: Checks if target is in an active attack phase."""
        return target.status in {"ECHO_WAIT", "CONFIRMED_ATTACK", "CENTERING", "ATTACKING", "LOCKED", "ENGAGED"}

    def is_target_exclusive(self, target):
        """Single Source of Truth: Checks if target is protected from reassignment or fusion transfer."""
        return self.is_target_in_attack_pipeline(target)
    
    def _calculate_observation_quality(self, drone_port: int, observation_data: Dict) -> float:
        """Calculate quality score for observation. Higher = better."""
        score = 0.0
        
        # Covariance factor (lower = higher score)
        covariance = observation_data.get('covariance')
        if covariance is not None:
            try:
                if hasattr(covariance, 'shape') and covariance.shape[0] >= 2:
                    cov_trace = float(covariance[0, 0] + covariance[1, 1])
                    if cov_trace > 0:
                        score += min(10.0, 100.0 / cov_trace)
            except (IndexError, TypeError):
                pass
        else:
            score += 5.0  # Default if no covariance
        
        # Detection confidence
        confidence = observation_data.get('confidence', 0.5)
        score += confidence * 10.0
        
        # Target center proximity
        bbox = observation_data.get('bbox')
        img_dims = observation_data.get('img_dims', (640, 480))
        if bbox and len(bbox) >= 2:
            cx, cy = bbox[0], bbox[1]
            img_cx, img_cy = img_dims[0] / 2, img_dims[1] / 2
            dist_norm = math.sqrt(((cx - img_cx) / img_cx) ** 2 + ((cy - img_cy) / img_cy) ** 2)
            score += max(0, 5.0 - dist_norm * 5.0)
        
        # Ray angle factor
        drone_gps = observation_data.get('drone_gps')
        target_gps = observation_data.get('world_xyz')
        if drone_gps and target_gps:
            alt = drone_gps[2] if len(drone_gps) > 2 else 5.0
            if alt > 0:
                score += min(5.0, alt / 10.0)
        
        return max(0.0, score)
    
    def _check_handoff_conditions(self, target: TrackedTarget, candidate_port: int, candidate_quality: float) -> bool:
        """Check if handoff conditions are met."""
        if target.is_immutable():
            return False
        
        consecutive_count = target.handoff_score_accumulator.get(candidate_port, 0)
        if consecutive_count < HANDOFF_CONSECUTIVE_FRAMES:
            return False
        
        owner_quality = target.get_owner_quality()
        if candidate_quality <= owner_quality * HANDOFF_QUALITY_RATIO:
            return False
        
        if target.last_handoff_time is not None:
            if time.time() - target.last_handoff_time < HANDOFF_COOLDOWN_S:
                return False
        
        return True
    
    def _process_secondary_observation(self, target: TrackedTarget, drone_port: int, observation_data: Dict):
        """Process observation from non-owner drone for handoff decisions."""
        quality = self._calculate_observation_quality(drone_port, observation_data)
        target.record_observation(drone_port, quality, is_owner=False)
        
        # Update handoff accumulator
        if quality > target.get_owner_quality() * HANDOFF_QUALITY_RATIO:
            target.handoff_score_accumulator[drone_port] = target.handoff_score_accumulator.get(drone_port, 0) + 1
        else:
            target.handoff_score_accumulator[drone_port] = 0
        
        if self._check_handoff_conditions(target, drone_port, quality):
            self._initiate_handoff(target, drone_port)
    
    def _initiate_handoff(self, target: TrackedTarget, new_owner_port: int):
        """Initiate ownership handoff with ping-pong protection."""
        # BLOCK: Candidate drone already has a target
        if new_owner_port in self.drone_active_target and self.drone_active_target[new_owner_port] is not None:
            current_target = self.drone_active_target[new_owner_port]
            if current_target != target.id:
                SwarmLogger.log("HANDOFF_BLOCKED", "LEADER", f"Handoff blocked for {target.id}: Drone_{new_owner_port} already has target {current_target}", "OWNERSHIP")
                return
        
        if target.last_handoff_time is not None:
            time_since_last = time.time() - target.last_handoff_time
            if time_since_last < HANDOFF_COOLDOWN_S:
                SwarmLogger.log("HANDOFF_BLOCKED", "LEADER", f"Handoff blocked for {target.id}: cooldown ({time_since_last:.1f}s < {HANDOFF_COOLDOWN_S}s)", "OWNERSHIP")
                return
        
        old_owner = target.owner_port
        SwarmLogger.log("HANDOFF", "LEADER", f"Ownership handoff initiated: {target.id} from {old_owner} to {new_owner_port}", "OWNERSHIP")
        
        target.ownership_state = TrackedTarget.OWNERSHIP_HANDOFF
        target.handoff_candidate_port = new_owner_port
        target.last_handoff_time = time.time()
        
        target.owner_port = new_owner_port
        target.assigned_drone_port = new_owner_port
        target.ownership_state = TrackedTarget.OWNERSHIP_OWNED
        target.handoff_candidate_port = None
        target.handoff_score_accumulator = {}
        target.secondary_observations = {}
        
        self.target_owner[target.id] = new_owner_port
        if old_owner is not None:
            self.drone_active_target[old_owner] = None
        self.drone_active_target[new_owner_port] = target.id
        
        SwarmLogger.log("OWNERSHIP", "LEADER", f"Owner switched {target.id}: {old_owner} -> {new_owner_port}", "OWNERSHIP")
    
    def _process_owner_loss(self, target: TrackedTarget):
        """Handle owner losing sight: OWNED -> RESERVED -> FREE."""
        if target.ownership_state == TrackedTarget.OWNERSHIP_OWNED:
            target.ownership_state = TrackedTarget.OWNERSHIP_RESERVED
            target.owner_lost_time = time.time()
            SwarmLogger.log("OWNERSHIP", "LEADER", f"{target.id}: OWNED -> RESERVED (owner {target.owner_port} lost sight)", "OWNERSHIP")
        
        elif target.ownership_state == TrackedTarget.OWNERSHIP_RESERVED:
            if target.owner_lost_time is not None:
                grace_period = 5.0
                if time.time() - target.owner_lost_time > grace_period:
                    best_secondary, quality = target.get_best_secondary()
                    if best_secondary is not None and quality > 0:
                        self._initiate_handoff(target, best_secondary)
                    else:
                        target.ownership_state = TrackedTarget.OWNERSHIP_FREE
                        target.owner_port = None
                        target.assigned_drone_port = None
                        target.owner_lost_time = None
                        SwarmLogger.log("OWNERSHIP", "LEADER", f"{target.id}: RESERVED -> FREE (no secondary observer)", "OWNERSHIP")

    def _assign_drone(self, port: int, canonical_id: str):
        """Atomic assignment with one-drone-one-target enforcement."""
        with self.lock:
            for t_id, t in self.targets.items():
                if t.assigned_drone_port == port and t_id != canonical_id:
                    t.assigned_drone_port = None
                    t.ownership_state = TrackedTarget.OWNERSHIP_FREE
                    t.owner_port = None
                    self.target_owner.pop(t_id, None)
                    SwarmLogger.log("ASSIGN", "LEADER", f"Released {t_id} from Drone_{port} (one-drone-one-target)", "ASSIGN")
            
            self.drone_active_target[port] = canonical_id
            self.target_owner[canonical_id] = port
            t = self.targets.get(canonical_id)
            if t:
                t.assigned_drone_port = port
                t.assignment_time = time.time()
                t.status = "PENDING" if t.status not in self.IMMUTABLE_STATES else t.status
                
                if t.ownership_state == TrackedTarget.OWNERSHIP_FREE:
                    t.ownership_state = TrackedTarget.OWNERSHIP_OWNED
                    t.owner_port = port
                    SwarmLogger.log("OWNERSHIP", "LEADER", f"{canonical_id}: FREE -> OWNED (assigned to Drone_{port})", "OWNERSHIP")

    def _release_drone(self, port: int):
        """Atomic Release: Clears bindings."""
        with self.lock:
             tid = self.drone_active_target.get(port)
             if tid:
                 self.target_owner.pop(tid, None)
                 t = self.targets.get(tid)
                 if t and t.assigned_drone_port == port:
                     t.assigned_drone_port = None
                     if t.status in {"PENDING", "ACTIVE", "ECHO_WAIT"}:
                         t.status = "ACTIVE"
                         t.approved_for_attack = False
             self.drone_active_target.pop(port, None)

    def _transfer_owner(self, old_port: int, new_port: int, canonical_id: str):
        """Atomic Transfer (e.g., during Fusion)."""
        with self.lock:
             self._release_drone(old_port)
             self._assign_drone(new_port, canonical_id)

    def is_drone_busy(self, drone_port):
        """O(1) check if drone has an active mapping."""
        with self.lock:
            tid = self.drone_active_target.get(drone_port)
            if not tid:
                return False
            
            target = self.targets.get(tid)
            if target and target.track_state != "DELETED":
                # Consider busy if in attack pipeline or PENDING/ACTIVE
                if target.status in {"PENDING", "ACTIVE"} or self.is_target_in_attack_pipeline(target):
                    return True
            
            return False

    def check_mission_updates(self, drone_port, raw_data=None):
        """Process incoming telemetry/detections and return mission status."""
        if raw_data is not None:
            if isinstance(raw_data, dict) and "detected_targets" in raw_data:
                det_targets = raw_data.get("detected_targets", [])
                self.drone_targets[drone_port] = det_targets
                
                for item in det_targets:
                    lat, lon = item.get('world_xyz', (0,0))[:2]
                    conf = item.get('confidence', 0)
                    tid = item.get('track_id')
                    cov_list = item.get('covariance')
                    cov_arr = np.array(cov_list) if cov_list else None
                    self.report_target(drone_port, lat, lon, conf, tid, raw_data=item, covariance=cov_arr)

            elif isinstance(raw_data, dict) and "action" in raw_data:
                msg_action = raw_data.get("action")
                t_id = raw_data.get("target_id")
                
                if msg_action == "REQUEST_LOCK":
                    return self._handle_lock_request(drone_port, t_id, raw_data)
                elif msg_action == "LOCKED":
                    return self._handle_lock_confirmation(drone_port, t_id)
                elif msg_action == "ECHO_TARGET":
                    return self._handle_echo_target(drone_port, t_id, raw_data)
                elif msg_action == "LOCKED_DATA":
                    return self._handle_locked_data(drone_port, t_id, raw_data)
                elif msg_action == "ATTACKING":
                    return self._handle_attacking(drone_port, t_id)
                elif msg_action == "MISMATCH":
                    SwarmLogger.log("MISMATCH", "LEADER", f"Drone_{drone_port} rejected Target {t_id} due to Geo-Validation failure.", "PROTOCOL")
                    if t_id in self.targets:
                        target = self.targets[t_id]
                        if target.assigned_drone_port == drone_port:
                            self._release_drone(drone_port)
                    if drone_port not in self.drone_mismatch_cooldowns:
                        self.drone_mismatch_cooldowns[drone_port] = {}
                    self.drone_mismatch_cooldowns[drone_port][t_id] = time.time()

            elif isinstance(raw_data, str) and raw_data == "HEARTBEAT":
                self.report_target(drone_port, 0, 0, 0, None, raw_data="HEARTBEAT")

        with self.lock:
            candidates = []
            
            # Check if drone already has an immutable target
            immutable_target = None
            immutable_action = None
            for t_id, target in self.targets.items():
                if target.assigned_drone_port == drone_port:
                    if target.is_immutable():
                        local_t_id = target.drone_local_ids.get(drone_port)
                        if target.status in ("CENTERING", "ATTACKING"):
                            immutable_action = ("ATTACK", t_id, local_t_id, target.lat, target.lon)
                        elif target.status == "CONFIRMED_ATTACK":
                            immutable_action = ("CENTER", t_id, local_t_id, target.lat, target.lon)
                        elif target.status == "ECHO_WAIT":
                            immutable_action = ("ATTACK_ASSIGN", t_id, local_t_id, target.lat, target.lon)
                        elif target.status in ("LOCKED", "DIVING"):
                            immutable_action = ("ATTACK", t_id, local_t_id, target.lat, target.lon)
                        else:
                            immutable_action = ("ATTACK", t_id, local_t_id, target.lat, target.lon)
                        immutable_target = t_id
                        break
            
            if immutable_action:
                # Rate-limit IMMUTABLE log to prevent spam
                now = time.time()
                last_log_ts = self.immutable_log_ts.get(drone_port, 0)
                last_target = self.immutable_log_target.get(drone_port, "")
                # Log only if: 1s passed OR target changed
                if now - last_log_ts >= 1.0 or immutable_target != last_target:
                    SwarmLogger.log("STATE", "PUBLISH", f"Drone_{drone_port} IMMUTABLE: {immutable_action[0]} -> Target {immutable_target}", "ACTION")
                    self.immutable_log_ts[drone_port] = now
                    self.immutable_log_target[drone_port] = immutable_target
                return immutable_action
            
            # Use drone_active_target as single source of truth
            active_target_id = self.drone_active_target.get(drone_port)
            
            if active_target_id and active_target_id in self.targets:
                target = self.targets[active_target_id]
                local_t_id = target.drone_local_ids.get(drone_port)
                
                # Skip if drone doesn't have local ID for this target
                if local_t_id is None and target.status not in self.IMMUTABLE_STATES:
                    SwarmLogger.log("GUARD", "PUBLISH", f"Drone_{drone_port} has no local ID for {active_target_id} - skipping", "ACTION")
                    self.drone_active_target[drone_port] = None
                else:
                    priority = 0
                    action_to_return = None
                    
                    if target.status in ("CENTERING", "ATTACKING"):
                        priority = 60
                        action_to_return = ("ATTACK", active_target_id, local_t_id, target.lat, target.lon)
                    elif target.status == "CONFIRMED_ATTACK":
                        priority = 50
                        action_to_return = ("CENTER", active_target_id, local_t_id, target.lat, target.lon)
                    elif target.status == "ECHO_WAIT":
                        priority = 40
                        action_to_return = ("ATTACK_ASSIGN", active_target_id, local_t_id, target.lat, target.lon)
                    elif target.status == "ENGAGED":
                        priority = 30
                        action_to_return = ("ATTACK", active_target_id, local_t_id, target.lat, target.lon)
                    elif target.status == "PENDING":
                        if target.approved_for_attack:
                            if target.consecutive_obs >= 12 and target.id_switch_count <= 1:
                                target.status = "ECHO_WAIT"
                                priority = 40
                                action_to_return = ("ATTACK_ASSIGN", active_target_id, local_t_id, target.lat, target.lon)
                            else:
                                priority = 20
                                action_to_return = ("HOVER", active_target_id, local_t_id, target.lat, target.lon)
                                SwarmLogger.log("GUARD", "PUBLISH", f"Drone_{drone_port} delaying attack on {active_target_id} due to instability", "ACTION")
                        else:
                            priority = 10
                            action_to_return = ("HOVER", active_target_id, local_t_id, target.lat, target.lon)
                    
                    if action_to_return and action_to_return[0] != "HOVER":
                        stability_score = target.consecutive_obs / (1.0 + target.id_switch_count + target.drop_count)
                        d_lat, d_lon = 0.0, 0.0
                        vehicle = self.drone_manager.drones.get(drone_port)
                        if vehicle and vehicle != "connecting":
                            try:
                                loc = vehicle.location.global_relative_frame
                                d_lat, d_lon = loc.lat, loc.lon
                            except: pass
                        dist = GeoMath.haversine_distance(d_lat, d_lon, target.lat, target.lon) if d_lat != 0 else 999.0
                        immutability = 1 if target.status in self.IMMUTABLE_STATES else 0
                        # Rate-limit action logs (every 0.5s)
                        action_log_key = f"action_{drone_port}"
                        now = time.time()
                        last_action_ts = self.immutable_log_ts.get(action_log_key, 0) if hasattr(self, 'immutable_log_ts') else 0
                        if now - last_action_ts >= 0.5:
                            SwarmLogger.log("INFO", "PUBLISH", f"Drone_{drone_port} action: {action_to_return[0]} for {active_target_id} (Pri={priority}, Immut={immutability}, Stab={stability_score:.1f}, Dist={dist:.1f}m)", "ACTION")
                            if not hasattr(self, '_action_log_ts'):
                                self._action_log_ts = {}
                            self._action_log_ts[action_log_key] = now
                        return action_to_return
                    elif action_to_return:
                        return action_to_return
            
            # Fallback: Check for targets with assigned_drone_port (legacy compatibility)
            for t_id, target in self.targets.items():
                if target.assigned_drone_port == drone_port:
                    local_t_id = target.drone_local_ids.get(drone_port)
                    priority = 0
                    action_to_return = None
                    
                    if target.status in ("CENTERING", "ATTACKING"):
                        priority = 60
                        action_to_return = ("ATTACK", t_id, local_t_id, target.lat, target.lon)
                    elif target.status == "CONFIRMED_ATTACK":
                        priority = 50
                        action_to_return = ("CENTER", t_id, local_t_id, target.lat, target.lon)
                    elif target.status == "ECHO_WAIT":
                        priority = 40
                        action_to_return = ("ATTACK_ASSIGN", t_id, local_t_id, target.lat, target.lon)
                    elif target.status == "ENGAGED":
                        priority = 30
                        action_to_return = ("ATTACK", t_id, local_t_id, target.lat, target.lon)
                    elif target.status == "PENDING":
                        if target.approved_for_attack:
                            if target.consecutive_obs >= 12 and target.id_switch_count <= 1:
                                target.status = "ECHO_WAIT"
                                priority = 40
                                action_to_return = ("ATTACK_ASSIGN", t_id, local_t_id, target.lat, target.lon)
                            else:
                                priority = 20
                                action_to_return = ("HOVER", t_id, local_t_id, target.lat, target.lon)
                                SwarmLogger.log("GUARD", "PUBLISH", f"Drone_{drone_port} delaying attack on {t_id} due to instability", "ACTION")
                        else:
                            priority = 10
                            action_to_return = ("HOVER", t_id, local_t_id, target.lat, target.lon)
                    elif target.status == "SEARCH":
                        priority = 0
                        action_to_return = ("SEARCH", None, None)
                    
                    if action_to_return:
                        stability_score = target.consecutive_obs / (1.0 + target.id_switch_count + target.drop_count)
                        d_lat, d_lon = 0.0, 0.0
                        vehicle = self.drone_manager.drones.get(drone_port)
                        if vehicle and vehicle != "connecting":
                            try:
                                loc = vehicle.location.global_relative_frame
                                d_lat, d_lon = loc.lat, loc.lon
                            except: pass
                        dist = GeoMath.haversine_distance(d_lat, d_lon, target.lat, target.lon) if d_lat != 0 else 999.0
                        age = time.time() - getattr(target, 'assignment_time', time.time())
                        time_assigned = age
                        immutability = 1 if target.status in self.IMMUTABLE_STATES else 0
                        candidates.append((priority, immutability, time_assigned, stability_score, -dist, t_id, action_to_return))

            if candidates:
                candidates.sort(key=lambda x: (x[0], x[1], x[2], x[3], x[4], x[5]), reverse=True)
                top_tuple = candidates[0]
                top_action = top_tuple[-1]
                SwarmLogger.log("INFO", "PUBLISH", f"Drone_{drone_port} action: {top_action[0]} for {top_action[1]} (Pri={top_tuple[0]})", "ACTION")
                return top_action

            # Direct reassignment: look for new CONFIRMED target that drone already sees
            # BLOCK: Only if drone doesn't already have a target
            if drone_port in self.drone_active_target and self.drone_active_target[drone_port] is not None:
                return "SEARCH", None, None
            
            unassigned_target_id = None
            vehicle = self.drone_manager.drones.get(drone_port)
            if vehicle is not None and vehicle != "connecting":
                try:
                    loc = vehicle.location.global_relative_frame
                    d_lat, d_lon = loc.lat, loc.lon
                    best_dist = float('inf')
                    for t_id, target in self.targets.items():
                        if target.assigned_drone_port is not None: continue
                        if target.status == "LOST" or target.track_state != "CONFIRMED": continue
                        if drone_port not in target.drone_local_ids: continue
                        
                        if drone_port in self.drone_mismatch_cooldowns:
                            from config import SWARM_GEO_COOLDOWN_S
                            last_mismatch = self.drone_mismatch_cooldowns[drone_port].get(t_id, 0)
                            if time.time() - last_mismatch < SWARM_GEO_COOLDOWN_S:
                                continue
                        
                        dist = GeoMath.haversine_distance(d_lat, d_lon, target.lat, target.lon)
                        if dist < best_dist:
                            best_dist = dist
                            unassigned_target_id = t_id
                except:
                    pass

            if unassigned_target_id:
                target = self.targets[unassigned_target_id]
                self._assign_drone(drone_port, unassigned_target_id)
                local_t_id = target.drone_local_ids.get(drone_port)
                SwarmLogger.log("DIRECT_REASSIGN", "LEADER", f"Drone_{drone_port} reassigned to {unassigned_target_id} (LocalID={local_t_id})", "ASSIGN")
                return "REASSIGN", unassigned_target_id, local_t_id, target.lat, target.lon
        
        return "SEARCH", None, None

    def report_target(self, drone_port, lat, lon, confidence, tracker_id=None, raw_data=None, covariance=None):
        """Receive, fuse, and manage target detections. Returns action dict."""
        if raw_data != "HEARTBEAT" and confidence < SWARM_ATTACK_MIN_CONFIDENCE:
            return {"action": "SEARCH", "message": "Low Confidence"}

        with self.lock:
            self._cleanup_local_identity_index()

            # Handshake protocol check
            if isinstance(raw_data, dict) and "action" in raw_data:
                msg_action = raw_data.get("action")
                t_id = raw_data.get("target_id")
                if msg_action == "REQUEST_LOCK":
                    return self._handle_lock_request(drone_port, t_id, raw_data)
                elif msg_action == "LOCKED":
                    return self._handle_lock_confirmation(drone_port, t_id)

            # Heartbeat mode
            if raw_data == "HEARTBEAT":
                for t_id, target in self.targets.items():
                    is_protected = target.status == "ENGAGED" or (ENABLE_ATTACK_IMMUTABILITY and target.status in self.IMMUTABLE_STATES)
                    if target.assigned_drone_port == drone_port and is_protected:
                        target.last_seen_time = time.time()
                        self.drone_heartbeat_ts[drone_port] = time.time()
                        local_t_id = target.drone_local_ids.get(drone_port)
                        SwarmLogger.log("HEARTBEAT", f"DRONE_{drone_port}", f"Target {t_id} still {target.status}", "FAILSAFE")
                        return {"action": "ATTACK", "target_id": t_id, "tracker_id": local_t_id}
                SwarmLogger.log("HEARTBEAT", f"DRONE_{drone_port}", f"No active target for heartbeat", "FAILSAFE")
                return {"action": "SEARCH", "message": "No active target for heartbeat"}

            # Leader verification
            final_lat, final_lon = lat, lon
            is_edge_detection = False
            
            if raw_data and isinstance(raw_data, dict):
                try:
                    if 'drone_gps' not in raw_data or 'bbox' not in raw_data:
                        pass
                    else:
                        d_lat, d_lon, d_alt = raw_data['drone_gps']
                        cx, cy, w, h = raw_data['bbox']
                        
                        W, H = float(config.CAMERA_WIDTH), float(config.CAMERA_HEIGHT)
                        margin_x = W * config.SWARM_EDGE_FILTER_MARGIN_RATIO
                        margin_y = H * config.SWARM_EDGE_FILTER_MARGIN_RATIO
                        area_thresh = (W * config.SWARM_EDGE_FILTER_AREA_RATIO) * (H * config.SWARM_EDGE_FILTER_AREA_RATIO)
                        
                        edge_x = (cx < margin_x or cx > (W - margin_x))
                        edge_y = (cy < margin_y or cy > (H - margin_y))
                        small_area = (w * h) < area_thresh
                        is_edge_detection = (edge_x or edge_y) and small_area

                        if 'attitude' in raw_data:
                            roll, pitch, yaw = raw_data['attitude']
                        else:
                            heading = raw_data.get('heading', 0)
                            roll, pitch = 0.0, 0.0
                            yaw = math.radians(heading) if heading else 0.0
                        
                        v_result = GeoMath.ray_ground_intersection(d_lat, d_lon, d_alt, roll, pitch, yaw, cx, cy)
                        v_lat = v_result[0]
                        v_lon = v_result[1]
                        
                        if v_lat is not None:
                            diff = GeoMath.haversine_distance(lat, lon, v_lat, v_lon)
                            if diff > SWARM_ALIGNMENT_TOLERANCE_M:
                                final_lat, final_lon = v_lat, v_lon
                            else:
                                final_lat = (lat + v_lat) / 2
                                final_lon = (lon + v_lon) / 2
                except Exception as e:
                    SwarmLogger.log("ERROR", "SwarmCoordinator", f"Leader Verification error: {e}", "LEADER")

            current_time = time.time()
            self._prune_targets(current_time)

            best_match_id = None
            best_score = float('inf')
            match_reason = ""

            # LOCAL ID INDEX: Match existing target by (drone_port, tracker_id)
            if tracker_id is not None:
                try:
                    idx_key = (int(drone_port), int(tracker_id))
                except Exception:
                    idx_key = None
                if idx_key is not None and idx_key in self.local_identity_index:
                    idx_tid = self.local_identity_index[idx_key].get('tid')
                    if isinstance(idx_tid, str) and idx_tid in self.targets:
                        cand = self.targets[idx_tid]
                        if cand.track_state != "DELETED" and cand.status != "LOST":
                            try:
                                d = GeoMath.haversine_distance(final_lat, final_lon, cand.lat, cand.lon)
                            except Exception:
                                d = 0.0
                            hard_cap = SWARM_LOCAL_ID_INDEX_MAX_JUMP_M
                            if d <= hard_cap:
                                best_match_id = idx_tid
                                best_score = 0.0
                                match_reason = "LOCAL_ID_INDEX"
            
            # SPATIAL-FIRST ASSOCIATION: Score by Mahalanobis distance
            if ENABLE_SPATIAL_FALLBACK and best_match_id is None:
                for t_id, target in self.targets.items():
                    if target.track_state == "DELETED":
                        continue
                    
                    should_fuse, mahal_dist, raw_dist = should_fuse_targets(
                        (final_lat, final_lon), 
                        (target.lat, target.lon),
                        cov1=covariance, 
                        cov2=target.covariance,
                        threshold_m=SWARM_MERGE_DISTANCE_M
                    )
                    
                    # Pre-clustering bypass for very close targets
                    dist_m = GeoMath.haversine_distance(final_lat, final_lon, target.lat, target.lon)
                    if not should_fuse and dist_m < DUPLICATE_MERGE_DISTANCE_M:
                        should_fuse = True
                        raw_dist = dist_m / 10.0
                        # Rate-limited debug log (every 2s per target)
                        bypass_log_key = f"bypass_{t_id}"
                        now = time.time()
                        if not hasattr(self, '_bypass_log_ts'):
                            self._bypass_log_ts = {}
                        last_ts = self._bypass_log_ts.get(bypass_log_key, 0)
                        if now - last_ts >= 2.0:
                            SwarmLogger.log("DEBUG", "LEADER", f"Pre-clustering bypass: {t_id} (Dist: {dist_m:.1f}m)", "FUSION")
                            self._bypass_log_ts[bypass_log_key] = now
                    
                    if not should_fuse:
                        continue
                    
                    score = mahal_dist
                    id_matched = False
                    
                    # ID boost if tracker IDs match
                    if tracker_id is not None:
                        mapped_local = target.drone_local_ids.get(drone_port)
                        if mapped_local == tracker_id:
                            score *= (1.0 - TRACKER_ID_ALPHA)
                            id_matched = True
                    
                    if score < best_score:
                        best_score = score
                        best_match_id = t_id
                        match_reason = "SPATIAL_ID_BOOST" if id_matched else "SPATIAL_MAHALANOBIS"
            else:
                # Legacy: ID-first association
                if tracker_id is not None:
                    for t_id, target in self.targets.items():
                        if target.assigned_drone_port == drone_port and target.tracker_id == tracker_id:
                            best_match_id = t_id
                            match_reason = "ID_PERSIST"
                            break
            
            target_id = None

            if best_match_id:
                target = self.targets[best_match_id]
                
                # Ownership model integration
                is_owner = (target.owner_port == drone_port)
                has_owner = (target.owner_port is not None)
                quality_score = self._calculate_observation_quality(drone_port, raw_data or {})
                
                if has_owner and not is_owner:
                    # Secondary observation: non-owner sees this target
                    self._process_secondary_observation(target, drone_port, raw_data or {})
                    target.update_position(final_lat, final_lon, covariance, is_edge=is_edge_detection)
                    if tracker_id is not None:
                        target.drone_local_ids[drone_port] = tracker_id
                    owner_local_id = target.drone_local_ids.get(target.owner_port) if target.owner_port else None
                    return {"action": "TRACK", "target_id": best_match_id, "tracker_id": target.drone_local_ids.get(drone_port), "is_secondary": True, "owner_port": target.owner_port}
                
                # Owner observation: update position normally
                target.update_position(final_lat, final_lon, covariance, is_edge=is_edge_detection)
                target.record_observation(drone_port, quality_score, is_owner=True)
                
                if target.ownership_state == TrackedTarget.OWNERSHIP_RESERVED:
                    target.ownership_state = TrackedTarget.OWNERSHIP_OWNED
                    target.owner_lost_time = None
                    SwarmLogger.log("OWNERSHIP", "LEADER", f"{best_match_id}: RESERVED -> OWNED (owner {drone_port} re-acquired)", "OWNERSHIP")
                
                # Update group metadata
                if isinstance(raw_data, dict) and raw_data.get('is_group'):
                    target.is_group = True
                    target.group_member_count = raw_data.get('group_member_count', target.group_member_count)
                    target.group_id_local = raw_data.get('group_id', target.group_id_local)
                
                # Check if TENTATIVE → CONFIRMED
                just_confirmed = target.check_confirmation(current_time, is_drone_searching=not self.attack_mode_active)
                
                if just_confirmed and self.attack_mode_active:
                    target.approved_for_attack = True
                    target.status = "ENGAGED"
                    SwarmLogger.log("ASSIGN", "LEADER", f"{best_match_id} Auto-Approved (Attack Mode Active)", "AUTO")
                
                # Update Local ID with Hysteresis for attack pipeline targets
                if tracker_id is not None:
                    old_local_id = target.drone_local_ids.get(drone_port)
                    
                    if (old_local_id is not None and old_local_id != tracker_id and 
                        self.is_target_in_attack_pipeline(target) and target.assigned_drone_port == drone_port):
                        # Hysteresis: 10 consecutive frames to flip Local ID during attack
                        hyst = target.local_id_hysteresis.setdefault(drone_port, {"id": None, "count": 0})
                        if hyst["id"] == tracker_id:
                            hyst["count"] += 1
                        else:
                            hyst["id"] = tracker_id
                            hyst["count"] = 1
                        if hyst["count"] >= 10:
                            target.drone_local_ids[drone_port] = tracker_id
                            target.tracker_id = tracker_id
                            SwarmLogger.log("ID_UPDATE", "LEADER", f"{best_match_id}: Drone_{drone_port} LocalID {old_local_id}→{tracker_id}", "ID")
                            hyst["count"] = 0
                    else:
                        target.drone_local_ids[drone_port] = tracker_id
                        if target.assigned_drone_port == drone_port:
                            target.tracker_id = tracker_id
                        self._update_local_identity_index(drone_port, tracker_id, best_match_id, now=current_time)
                        
                        if old_local_id is not None and old_local_id != tracker_id:
                            # Spam suppression (1.0s limit for ID_UPDATE logs)
                            now = time.time()
                            suppression_key = f"ID_UPDATE_{best_match_id}_{drone_port}"
                            if not hasattr(self, "last_id_update_time"):
                                self.last_id_update_time = {}
                            last_log_time = self.last_id_update_time.get(suppression_key, 0.0)
                            if now - last_log_time > 1.0:
                                SwarmLogger.log("ID_UPDATE", "LEADER", f"{best_match_id}: Drone_{drone_port} LocalID {old_local_id}→{tracker_id}", "ID")
                                self.last_id_update_time[suppression_key] = now
                    
                target_id = best_match_id
            else:
                # New Target - check for duplicates first
                duplicate_target_id = None
                for existing_id, existing_t in self.targets.items():
                    if existing_t.track_state == "DELETED":
                        continue
                    if existing_t.lat is None or existing_t.lon is None:
                        continue
                    dist = GeoMath.haversine_distance(final_lat, final_lon, existing_t.lat, existing_t.lon)
                    if dist < DUPLICATE_MERGE_DISTANCE_M:
                        duplicate_target_id = existing_id
                        SwarmLogger.log("MERGE", "LEADER", f"NEW TARGET BLOCKED: too close to {existing_id} ({dist:.1f}m)", "FUSION")
                        break
                
                if duplicate_target_id:
                    target = self.targets[duplicate_target_id]
                    target.update_position(final_lat, final_lon, covariance, is_edge=is_edge_detection)
                    if tracker_id is not None:
                        target.drone_local_ids[drone_port] = tracker_id
                    target_id = duplicate_target_id
                    quality_score = self._calculate_observation_quality(drone_port, raw_data or {})
                    target.record_observation(drone_port, quality_score, is_owner=False)
                else:
                    # Create new target
                    self.target_counter += 1
                    target_id = f"T{self.target_counter}"
                    new_target = TrackedTarget(target_id, final_lat, final_lon, current_time, tracker_id=tracker_id, covariance=covariance, source_drone_port=drone_port)
                    
                    # Set initial ownership - first observer becomes owner (unless drone has existing target)
                    has_existing_target = any(
                        t.assigned_drone_port == drone_port and t.track_state != "DELETED"
                        for t in self.targets.values()
                    )
                    
                    if ENABLE_CAPACITY_GUARD and has_existing_target:
                        new_target.ownership_state = TrackedTarget.OWNERSHIP_FREE
                        new_target.owner_port = drone_port
                        new_target.assigned_drone_port = None
                        SwarmLogger.log("CAPACITY", "LEADER", f"Drone_{drone_port} already has target - new {target_id} stays FREE", "CAPACITY")
                    else:
                        new_target.ownership_state = TrackedTarget.OWNERSHIP_OWNED
                        new_target.owner_port = drone_port
                        new_target.assigned_drone_port = drone_port
                    
                    quality_score = self._calculate_observation_quality(drone_port, raw_data or {})
                    new_target.record_observation(drone_port, quality_score, is_owner=True)
                    
                    if is_edge_detection:
                        new_target.observation_count = 0
                        new_target.consecutive_obs = 0
                        new_target._edge_accumulator = config.SWARM_EDGE_FILTER_FRACTIONAL_CREDIT
                    self.targets[target_id] = new_target
                    
                    self.target_owner[target_id] = drone_port
                    if new_target.assigned_drone_port == drone_port:
                        self.drone_active_target[drone_port] = target_id

                    if tracker_id is not None:
                        self._update_local_identity_index(drone_port, tracker_id, target_id, now=current_time)

                    if isinstance(raw_data, dict):
                        if raw_data.get('is_group'):
                            new_target.is_group = True
                            new_target.group_member_count = raw_data.get('group_member_count', 0)
                            new_target.group_id_local = raw_data.get('group_id')

                    SwarmLogger.log("TARGET", "LEADER", f"NEW TARGET (TENTATIVE): {target_id} (TrackerID: {tracker_id}) @ ({final_lat:.6f}, {final_lon:.6f}) | Owner: Drone_{drone_port}", "TARGET")

            # Kalman fusion recording
            if self.fusion_manager and target_id:
                try:
                    import numpy as np
                    if covariance is not None and hasattr(covariance, 'shape'):
                        obs_cov = np.array(covariance, dtype=np.float64)
                    else:
                        sigma = 2.0
                        obs_cov = np.eye(3) * (sigma ** 2)
                    
                    self.fusion_manager.add_observation(
                        target_id, (final_lat, final_lon, 0.0), obs_cov, drone_port,
                        is_group=self.targets[target_id].is_group,
                        group_member_count=self.targets[target_id].group_member_count
                    )
                except Exception as e:
                    SwarmLogger.log("ERROR", "SwarmCoordinator", f"add_observation error: {e}", "LEADER")

            # Assignment state lookup (no assignment happens here - only in _periodic_optimization)
            target = self.targets[target_id]
            
            if target.track_state == "TENTATIVE":
                # Local ID redirect mapping for robust local recovery
                assigned_redirect_id = None
                for t_id_check, t_check in self.targets.items():
                    if (t_check.assigned_drone_port == drone_port and 
                        t_check.track_state == "CONFIRMED" and
                        t_id_check != target_id):
                        dist = GeoMath.haversine_distance(
                            self.targets[target_id].lat, self.targets[target_id].lon,
                            t_check.lat, t_check.lon
                        )
                        if dist < 20.0:
                            assigned_redirect_id = t_id_check
                            break

                if assigned_redirect_id:
                    redirect_target = self.targets[assigned_redirect_id]
                    redirect_target.update_position(final_lat, final_lon, covariance)
                    redirect_target.last_seen_time = time.time()
                    if tracker_id is not None:
                        redirect_target.drone_local_ids[drone_port] = tracker_id
                        if redirect_target.assigned_drone_port == drone_port:
                            redirect_target.tracker_id = tracker_id
                    del self.targets[target_id]
                    target_id = assigned_redirect_id
                    target = redirect_target
                else:
                    return {"action": "SEARCH", "target_id": target_id, "tracker_id": tracker_id, "message": "Awaiting confirmation"}

            # Current state lookup
            if target.assigned_drone_port == drone_port:
                if target.approved_for_attack:
                    target.status = "ENGAGED"
                    return {"action": "ATTACK", "target_id": target_id, "tracker_id": target.tracker_id}
                else:
                    target.status = "PENDING"
                    return {"action": "HOVER", "target_id": target_id, "tracker_id": target.tracker_id}
            elif target.assigned_drone_port is None:
                return {"action": "TRACK", "target_id": target_id, "tracker_id": tracker_id, "message": "Unassigned"}
            else:
                return {"action": "TRACK", "target_id": target_id, "owner": target.assigned_drone_port}

    def approve_attack(self, target_id):
        """User approves attack on a specific target."""
        with self.lock:
            if target_id in self.targets:
                self.targets[target_id].approved_for_attack = True
                self.targets[target_id].status = "ENGAGED"
                SwarmLogger.log("USER_CMD", "LEADER", f"ATTACK APPROVED: {target_id}", "USER")
                return True
            return False

    def approve_all_targets(self):
        """Approve attack on all pending targets with duplicate protection and stability sorting."""
        count = 0
        with self.lock:
            # Enable attack mode
            self.attack_mode_active = True
            SwarmLogger.log("USER_CMD", "LEADER", "ATTACK MODE ACTIVATED", "USER")
            
            # Collect eligible targets
            eligible_targets = []
            for t_id, target in self.targets.items():
                if target.track_state == "DELETED" or target.status == "LOST":
                    continue
                if target.status not in ("PENDING", "CONFIRMED") and target.approved_for_attack:
                    continue
                if target.lat is None or target.lon is None:
                    continue
                stability = target.consecutive_obs / (1.0 + target.id_switch_count + target.drop_count)
                eligible_targets.append((t_id, target, stability))
            
            # Sort by stability (highest first)
            eligible_targets.sort(key=lambda x: x[2], reverse=True)
            
            # Duplicate filtering
            approved_locations = []
            targets_to_approve = []
            
            for t_id, target, stability in eligible_targets:
                is_duplicate = False
                for approved_lat, approved_lon, approved_id, approved_stab in approved_locations:
                    dist = GeoMath.haversine_distance(target.lat, target.lon, approved_lat, approved_lon)
                    if dist < DUPLICATE_MERGE_DISTANCE_M:
                        SwarmLogger.log("MERGE", "LEADER", f"DUPLICATE PROTECTION: {t_id} skipped (too close to {approved_id})", "FUSION")
                        is_duplicate = True
                        break
                if not is_duplicate:
                    targets_to_approve.append((t_id, target))
                    approved_locations.append((target.lat, target.lon, t_id, stability))
            
            for t_id, target in targets_to_approve:
                if target.status == "PENDING" or (target.track_state == "CONFIRMED" and not target.approved_for_attack):
                    target.approved_for_attack = True
                    target.status = "ECHO_WAIT"
                    SwarmLogger.log("USER_CMD", "LEADER", f"MASS APPROVE: {t_id} → ECHO_WAIT", "USER")
                    count += 1
        return count

    def release_drone_assignment(self, drone_port):
        """Release any target assignment for this drone (R1/R32)."""
        with self.lock:
            released_targets = []
            for t_id, target in self.targets.items():
                if target.assigned_drone_port == drone_port:
                    released_targets.append(t_id)
            
            self._release_drone(drone_port)
            
            # Remove from heartbeat tracking
            self.drone_heartbeat_ts.pop(drone_port, None)
            
            if released_targets:
                SwarmLogger.log("LIFECYCLE", "LEADER", f"Drone_{drone_port} RELEASED: targets={released_targets}", "TRACK")

    def _prune_targets(self, current_time):
        """Track lifecycle: TENTATIVE→CONFIRMED→LOST→DELETE based on timeouts."""
        to_delete = []
        
        for t_id, target in self.targets.items():
            time_since_obs = current_time - target.last_observation_time
            
            # TENTATIVE lifecycle
            if target.track_state == "TENTATIVE":
                just_confirmed = target.check_confirmation(current_time, is_drone_searching=not self.attack_mode_active)
                if just_confirmed and self.attack_mode_active:
                    target.approved_for_attack = True
                    target.status = "ENGAGED"
                    SwarmLogger.log("ASSIGN", "LEADER", f"{t_id} Auto-Approved (Attack Mode Active)", "AUTO")
                if target.track_state == "TENTATIVE" and time_since_obs > TRACK_TENTATIVE_TIMEOUT_SEC:
                    # CHECK: Is ANY drone still seeing this target?
                    any_drone_seeing = len(target.drone_local_ids) > 0
                    if any_drone_seeing:
                        # Keep alive - at least one drone has this target in view
                        continue
                    SwarmLogger.log("LIFECYCLE", "LEADER", f"TENTATIVE DELETED: {t_id} | NoObs={time_since_obs:.2f}s", "LIFECYCLE")
                    to_delete.append(t_id)
                continue
            
            # CONFIRMED lifecycle
            if target.track_state == "CONFIRMED":
                is_protected = target.status == "ENGAGED" or (ENABLE_ATTACK_IMMUTABILITY and target.status in self.IMMUTABLE_STATES)
                if is_protected and target.assigned_drone_port is not None:
                    drone_still_connected = (
                        target.assigned_drone_port in self.drone_manager.drones
                        and self.drone_manager.drones[target.assigned_drone_port] != "connecting"
                    )
                    if not drone_still_connected:
                        SwarmLogger.log("PRUNE", "LEADER", f"GHOST RELEASE: {t_id} | Drone_{target.assigned_drone_port} disconnected", "PRUNE")
                        self._release_drone(target.assigned_drone_port)
                        continue
                    
                    if time_since_obs > TARGET_PRUNE_TIME_S:
                        SwarmLogger.log("PRUNE", "LEADER", f"TARGET DELETED (Stuck Ghost): {t_id} | NoObs={time_since_obs:.1f}s | Status={target.status} | Drone={target.assigned_drone_port}", "PRUNE")
                        to_delete.append(t_id)
                    elif time_since_obs > ATTACK_STALE_ENGAGED_TIMEOUT_S:
                        # P6 FIX: Force-release stale ENGAGED target
                        SwarmLogger.log("PRUNE", "LEADER", f"STALE {target.status}: {t_id} | NoObs={time_since_obs:.1f}s → LOST | Drone={target.assigned_drone_port}", "PRUNE")
                        target.track_state = "LOST"
                        target.status = "LOST"
                        self._release_drone(target.assigned_drone_port)
                    elif time_since_obs > TARGET_COAST_TIME_S:
                        SwarmLogger.log("GUARD", "LEADER", f"PRUNE GUARD: {t_id} is {target.status} — kept (NoObs={time_since_obs:.1f}s, Drone={target.assigned_drone_port})", "GUARD")
                    continue
                
                # Non-engaged CONFIRMED → LOST after timeout
                if time_since_obs > TRACK_LOST_TIMEOUT_SEC:
                    target.track_state = "LOST"
                    target.status = "LOST"
                    SwarmLogger.log("LIFECYCLE", "LEADER", f"{t_id}: CONFIRMED → LOST (NoObs={time_since_obs:.1f}s)", "TRACK")
                continue
            
            # LOST lifecycle
            if target.track_state == "LOST":
                if time_since_obs > TRACK_DELETE_TIMEOUT_SEC:
                    SwarmLogger.log("LIFECYCLE", "LEADER", f"{t_id}: LOST → DELETED (NoObs={time_since_obs:.1f}s)", "TRACK")
                    if target.assigned_drone_port:
                        SwarmLogger.log("LIFECYCLE", "LEADER", f"Drone_{target.assigned_drone_port} Released ({t_id} deleted)", "TRACK")
                        self._release_drone(target.assigned_drone_port)
                    to_delete.append(t_id)
                continue

        for t_id in to_delete:
            if self.fusion_manager:
                self.fusion_manager.remove_target(t_id)
            if t_id in self.targets:
                del self.targets[t_id]

        self._cleanup_local_identity_index(current_time)
        self._merge_duplicate_active_targets()
            
    def _get_dynamic_merge_threshold(self, t1, t2) -> float:
        """Calculate dynamic merge threshold: base + avg_sigma * scale."""
        def _sigma_from_cov(cov):
            try:
                if cov is None:
                    return None
                v0 = float(cov[0][0]) if len(cov) > 0 else 0
                v1 = float(cov[1][1]) if len(cov) > 1 and len(cov[0]) > 1 else 0
                return math.sqrt(max(0.0, max(v0, v1)))
            except Exception:
                return None
        
        sigma1 = _sigma_from_cov(t1.covariance)
        sigma2 = _sigma_from_cov(t2.covariance)
        
        if sigma1 is not None and sigma2 is not None:
            avg_sigma = (sigma1 + sigma2) / 2.0
        elif sigma1 is not None:
            avg_sigma = sigma1
        elif sigma2 is not None:
            avg_sigma = sigma2
        else:
            avg_sigma = 0.0
        
        threshold = MERGE_DISTANCE_BASE_M + avg_sigma * MERGE_DISTANCE_SIGMA_SCALE
        return min(threshold, MERGE_DISTANCE_MAX_M)
    
    def _merge_duplicate_active_targets(self):
        """Merge spatially close CONFIRMED targets using dynamic threshold."""
        all_ids = list(self.targets.keys())
        to_remove = []
        
        for i in range(len(all_ids)):
            id1 = all_ids[i]
            if id1 not in self.targets or id1 in to_remove: continue
            t1 = self.targets[id1]
            if t1.track_state != "CONFIRMED": continue

            for j in range(i + 1, len(all_ids)):
                id2 = all_ids[j]
                if id2 not in self.targets or id2 in to_remove: continue
                t2 = self.targets[id2]
                if t2.track_state != "CONFIRMED": continue
                
                dist = GeoMath.haversine_distance(t1.lat, t1.lon, t2.lat, t2.lon)
                
                # Dynamic merge threshold based on position uncertainty
                dynamic_threshold = self._get_dynamic_merge_threshold(t1, t2)
                
                # Holistic merge check using Mahalanobis
                should_fuse, effective_mahal, effective_dist = should_fuse_targets(
                    (t1.lat, t1.lon), (t2.lat, t2.lon),
                    cov1=t1.covariance, cov2=t2.covariance,
                    threshold_m=dynamic_threshold
                )
                
                if should_fuse:
                    # Group safety: never merge different group clusters
                    if getattr(t1, 'is_group', False) or getattr(t2, 'is_group', False):
                        if bool(getattr(t1, 'is_group', False)) != bool(getattr(t2, 'is_group', False)):
                            continue
                        g1 = getattr(t1, 'group_id_local', None)
                        g2 = getattr(t2, 'group_id_local', None)
                        if g1 is not None and g2 is not None and g1 != g2:
                            continue

                    # Attack/approval safety: block merge only if BOTH are in active attack
                    in_attack_1 = self.is_target_in_attack_pipeline(t1)
                    in_attack_2 = self.is_target_in_attack_pipeline(t2)
                    approved_1 = getattr(t1, 'approved_for_attack', False)
                    approved_2 = getattr(t2, 'approved_for_attack', False)
                    
                    if (in_attack_1 or approved_1) and (in_attack_2 or approved_2):
                        continue

                    # Geo quality gating: avoid merges under poor covariance
                    max_sigma_m = SWARM_FUSION_MAX_SIGMA_M
                    if max_sigma_m > 0.0:
                        def _sigma_from_cov(cov):
                            try:
                                if cov is None:
                                    return None
                                v0 = float(cov[0][0])
                                v1 = float(cov[1][1])
                                return math.sqrt(max(0.0, max(v0, v1)))
                            except Exception:
                                return None
                        s1 = _sigma_from_cov(t1.covariance)
                        s2 = _sigma_from_cov(t2.covariance)
                        if (s1 is not None and s1 > max_sigma_m) or (s2 is not None and s2 > max_sigma_m):
                            if dist > 3.0:
                                continue

                    # Velocity similarity check
                    velocity_ok = True
                    if self.fusion_manager:
                        pos1 = self.fusion_manager.get_target_position(id1)
                        pos2 = self.fusion_manager.get_target_position(id2)
                        if pos1 and pos2:
                            kf1 = self.fusion_manager.filters.get(id1)
                            kf2 = self.fusion_manager.filters.get(id2)
                            if kf1 and kf2:
                                vel1 = kf1.get_velocity()
                                vel2 = kf2.get_velocity()
                                vel_diff = np.linalg.norm(vel1 - vel2)
                                if vel_diff > DUPLICATE_VELOCITY_SIMILARITY_MPS:
                                    velocity_ok = False
                    
                    if not velocity_ok:
                        # Spatial bypass for velocity filter
                        bypass_threshold = DUPLICATE_MERGE_DISTANCE_M / 3.0
                        if dist < bypass_threshold:
                            SwarmLogger.log("MERGE", "LEADER", f"MERGE: {id1} & {id2} velocity check bypassed ({dist:.1f}m)", "FUSION")
                            velocity_ok = True
                    
                    if not velocity_ok:
                        continue
                    
                    # Attack immutability guard
                    if ENABLE_ATTACK_IMMUTABILITY and dist >= 5.0:
                        if (
                            t1.status in self.IMMUTABLE_STATES or t2.status in self.IMMUTABLE_STATES or
                            getattr(t1, 'approved_for_attack', False) or getattr(t2, 'approved_for_attack', False)
                        ):
                            # Throttle spam and apply backoff
                            pair = tuple(sorted([id1, id2]))
                            now = time.time()
                            backoff = self.merge_abort_backoff.get(pair, {"next_attempt": 0.0, "penalty": config.SWARM_MERGE_BACKOFF_INIT_S})
                            
                            if now < backoff["next_attempt"]:
                                # UI Visual Grouping (Alias TTL Update)
                                bypass_threshold = DUPLICATE_MERGE_DISTANCE_M / 3.0
                                if dist < bypass_threshold:
                                    main_id, alias_id = (id1, id2) if t1.observation_count >= t2.observation_count else (id2, id1)
                                    # Alias Hysteresis (Sticky Winner)
                                    if id1 in self.alias_ttl and self.alias_ttl[id1]["canonical"] == id2:
                                        if t1.observation_count < t2.observation_count + 5:
                                            main_id, alias_id = id2, id1
                                    elif id2 in self.alias_ttl and self.alias_ttl[id2]["canonical"] == id1:
                                        if t2.observation_count < t1.observation_count + 5:
                                            main_id, alias_id = id1, id2
                                    self.alias_ttl[alias_id] = {"canonical": main_id, "expires": now + config.SWARM_ALIAS_TTL_S}
                                continue
                            
                            SwarmLogger.log("IMMUTABLE", "LEADER", f"MERGE ABORTED: {id1} ({t1.status}) or {id2} ({t2.status}) protected", "MERGE")
                            new_penalty = min(backoff["penalty"] * 2.0, config.SWARM_MERGE_BACKOFF_MAX_S)
                            self.merge_abort_backoff[pair] = {"next_attempt": now + new_penalty, "penalty": new_penalty}
                            continue

                    protected_states = {"ENGAGED", "LOCKED", "CONFIRMED_ATTACK", "CENTERING", "ATTACKING", "ECHO_WAIT"}

                    # Keep protected preference, then most-observed
                    keep_id, drop_id = id1, id2
                    if t2.status in protected_states and t1.status not in protected_states:
                        keep_id, drop_id = id2, id1
                    elif t2.status in protected_states and t1.status in protected_states:
                        # Prevent merging two actively attacking targets
                        if t1.assigned_drone_port != t2.assigned_drone_port:
                            if dist < 5.0:
                                SwarmLogger.log("MERGE", "LEADER", f"DOUBLE ATTACK BYPASSED: Perfect overlap ({dist:.1f}m), {id1} & {id2}", "FUSION")
                                if t2.observation_count > t1.observation_count:
                                    keep_id, drop_id = id2, id1
                            else:
                                # Throttle spam and apply backoff
                                pair = tuple(sorted([id1, id2]))
                                now = time.time()
                                backoff = self.merge_abort_backoff.get(pair, {"next_attempt": 0.0, "penalty": config.SWARM_MERGE_BACKOFF_INIT_S})
                                if now < backoff["next_attempt"]:
                                    continue
                                SwarmLogger.log("WARNING", "LEADER", f"MERGE ABORTED: {id1} & {id2} both have active protocol lock", "MERGE")
                                new_penalty = min(backoff["penalty"] * 2.0, config.SWARM_MERGE_BACKOFF_MAX_S)
                                self.merge_abort_backoff[pair] = {"next_attempt": now + new_penalty, "penalty": new_penalty}
                                continue
                    elif t2.observation_count > t1.observation_count:
                        keep_id, drop_id = id2, id1
                    
                    drop_target = self.targets[drop_id]
                    keep_target = self.targets[keep_id]

                    SwarmLogger.log("MERGE", "LEADER", f"MERGE: {keep_id} ← {drop_id} | Dist={dist:.1f}m | KeepObs={keep_target.observation_count} DropObs={drop_target.observation_count}", "FUSION")
                    
                    # Transfer drone assignment if needed
                    if drop_target.assigned_drone_port:
                        port = drop_target.assigned_drone_port
                        is_busy_elsewhere = False
                        for _tid, targ in self.targets.items():
                            if _tid != drop_id and targ.assigned_drone_port == port and targ.track_state != "DELETED":
                                if targ.status in {"PENDING", "ACTIVE"} or self.is_target_in_attack_pipeline(targ):
                                    is_busy_elsewhere = True
                                    break
                        if keep_target.assigned_drone_port is None and not is_busy_elsewhere:
                            self._release_drone(port)
                            self._assign_drone(port, keep_id)
                            keep_target.tracker_id = drop_target.tracker_id
                        elif keep_target.assigned_drone_port is not None and not is_busy_elsewhere:
                            # Co-tracking: transfer ownership to canonical ID
                            self.drone_active_target[port] = keep_id
                            if drop_id in self.target_owner and self.target_owner[drop_id] == port:
                                del self.target_owner[drop_id]
                            drop_target.assigned_drone_port = None
                            SwarmLogger.log("MERGE", "LEADER", f"Drone_{port} Transferred ({drop_id} merged → {keep_id})", "FUSION")
                        else:
                            self._release_drone(port)
                            SwarmLogger.log("MERGE", "LEADER", f"Drone_{port} Released ({drop_id} merged → {keep_id} capacity full)", "FUSION")
                    
                    # Merge local ID maps
                    for port, lid in drop_target.drone_local_ids.items():
                        if port not in keep_target.drone_local_ids:
                            keep_target.drone_local_ids[port] = lid
                    
                    # Merge metadata (Group Info)
                    if drop_target.is_group:
                        keep_target.is_group = True
                        if keep_target.group_member_count is None:
                            keep_target.group_member_count = drop_target.group_member_count
                        else:
                            keep_target.group_member_count = max(
                                int(keep_target.group_member_count or 0),
                                int(drop_target.group_member_count or 0),
                            )
                        SwarmLogger.log("MERGE", "LEADER", f"Metadata transferred: {keep_id} is now Group x{keep_target.group_member_count}", "FUSION")
                    
                    to_remove.append(drop_id)
        
        for mid in to_remove:
            if self.fusion_manager:
                self.fusion_manager.remove_target(mid)
            if mid in self.targets:
                del self.targets[mid]
            
    def handle_drone_disconnect(self, port):
        """Releases targets if assigned drone disconnects."""
        with self.lock:
            for t_id, target in self.targets.items():
                if target.assigned_drone_port == port:
                    SwarmLogger.log("WARNING", "LEADER", f"DRONE_{port} LOST. Releasing {t_id}", "DRONE")
                    self._release_drone(port)

    def stop_assignment_loop(self):
        """Stops the periodic assignment thread."""
        self._assignment_loop_running = False
        if hasattr(self, '_assignment_thread') and self._assignment_thread:
            self._assignment_thread.join(timeout=2.0)
    # =========================================================================
    # TARGET COLLAPSE MANAGER
    # =========================================================================

    def _collapse_duplicate_targets(self):
        """Group targets by (Drone Port, Local Tracker ID) and keep one canonical."""
        if not ENABLE_TARGET_COLLAPSE:
            return

        with self.lock:
            # Identity grouping: (drone_port, local_id) -> list of target_ids
            identity_map = {}
            for t_id, t in self.targets.items():
                if t.track_state == "DELETED" or t.status == "LOST":
                    continue
                
                for port, lid in t.drone_local_ids.items():
                    key = (port, lid)
                    if key not in identity_map:
                        identity_map[key] = []
                    if t_id not in identity_map[key]:
                        identity_map[key].append(t_id)
                
                if t.assigned_drone_port is not None and t.tracker_id is not None:
                    key = (t.assigned_drone_port, t.tracker_id)
                    if key not in identity_map:
                        identity_map[key] = []
                    if t_id not in identity_map[key]:
                        identity_map[key].append(t_id)

            protected_states = {"ENGAGED", "LOCKED", "CONFIRMED_ATTACK", "CENTERING", "ATTACKING", "ECHO_WAIT"}
            to_remove = []

            for key, t_ids in identity_map.items():
                valid_t_ids = [tid for tid in t_ids if tid not in to_remove and tid in self.targets]
                if len(valid_t_ids) <= 1:
                    continue

                # Do not collapse if this would mix different group semantics
                try:
                    any_group = any(getattr(self.targets[tid], 'is_group', False) for tid in valid_t_ids)
                    any_nongroup = any(not getattr(self.targets[tid], 'is_group', False) for tid in valid_t_ids)
                    if any_group and any_nongroup:
                        continue
                    if any_group:
                        gids = set([getattr(self.targets[tid], 'group_id_local', None) for tid in valid_t_ids])
                        gids.discard(None)
                        if len(gids) > 1:
                            continue
                except Exception:
                    pass

                # Do not collapse attack/approved targets
                try:
                    if any(
                        (getattr(self.targets[tid], 'approved_for_attack', False) or self.targets[tid].status in self.IMMUTABLE_STATES)
                        for tid in valid_t_ids
                    ):
                        continue
                except Exception:
                    pass
                
                # Clean assimilation: sort by protected status then oldest
                def sorting_key(tid):
                    t = self.targets[tid]
                    state_score = 3 if t.status in protected_states else (2 if t.status == "PENDING" and t.track_state == "CONFIRMED" else 1)
                    return (-state_score, t.first_seen_time)
                
                valid_t_ids.sort(key=sorting_key)
                canonical_id = valid_t_ids[0]
                alias_ids = valid_t_ids[1:]

                if alias_ids:
                    SwarmLogger.log("COLLAPSE", "LEADER", f"COLLAPSE: {','.join(alias_ids)} -> {canonical_id} (Drone={key[0]}, TrackerID={key[1]})", "COLLAPSE")
                    
                    canonical_target = self.targets[canonical_id]
                    for aid in alias_ids:
                        alias_target = self.targets[aid]
                        SwarmLogger.log("ALIAS", "LEADER", f"ALIAS CREATED: {aid} -> {canonical_id}", "COLLAPSE")
                        self.target_aliases[aid] = canonical_id
                        
                        # Merge local IDs (transfer history)
                        for dp, lid in alias_target.drone_local_ids.items():
                            canonical_target.drone_local_ids[dp] = lid
                            
                        
                        # Absorb max observation count
                        canonical_target.observation_count = max(canonical_target.observation_count, alias_target.observation_count)
                        
                        to_remove.append(aid)

            # Cleanup removed aliases
            for mid in to_remove:
                if self.fusion_manager:
                    self.fusion_manager.remove_target(mid)
                if mid in self.targets:
                    del self.targets[mid]
            
            # Spatial duplicate collapse
            spatial_to_remove = []
            spatial_processed = set()
            
            for t_id_a, t_a in self.targets.items():
                if t_id_a in spatial_processed or t_id_a in to_remove:
                    continue
                if t_a.track_state == "DELETED":
                    continue
                if t_a.lat is None or t_a.lon is None:
                    continue
                
                for t_id_b, t_b in self.targets.items():
                    if t_id_a == t_id_b or t_id_b in spatial_processed or t_id_b in to_remove:
                        continue
                    if t_b.track_state == "DELETED":
                        continue
                    if t_b.lat is None or t_b.lon is None:
                        continue
                    
                    dist = GeoMath.haversine_distance(t_a.lat, t_a.lon, t_b.lat, t_b.lon)
                    dynamic_threshold = self._get_dynamic_merge_threshold(t_a, t_b)
                    
                    if dist < dynamic_threshold:
                        # Don't merge targets owned by different active drones unless very close
                        owner_a = t_a.assigned_drone_port or t_a.owner_port
                        owner_b = t_b.assigned_drone_port or t_b.owner_port
                        
                        both_have_different_owners = (
                            owner_a is not None and 
                            owner_b is not None and
                            owner_a != owner_b
                        )
                        
                        if both_have_different_owners:
                            in_attack_a = self.is_target_in_attack_pipeline(t_a)
                            in_attack_b = self.is_target_in_attack_pipeline(t_b)
                            if in_attack_a and in_attack_b:
                                continue
                            if dist >= 10.0 and not (in_attack_a or in_attack_b):
                                continue
                        
                        # Merge: keep the one with higher stability
                        stab_a = t_a.consecutive_obs / (1.0 + t_a.id_switch_count + t_a.drop_count)
                        stab_b = t_b.consecutive_obs / (1.0 + t_b.id_switch_count + t_b.drop_count)
                        
                        if stab_a >= stab_b:
                            canonical, alias = t_a, t_b
                            canonical_id, alias_id = t_id_a, t_id_b
                        else:
                            canonical, alias = t_b, t_a
                            canonical_id, alias_id = t_id_b, t_id_a
                        
                        if alias.approved_for_attack and canonical.status in self.IMMUTABLE_STATES:
                            continue
                        
                        SwarmLogger.log("COLLAPSE", "LEADER", f"SPATIAL MERGE: {alias_id} -> {canonical_id} (Dist={dist:.1f}m, Stab={stab_a:.1f} vs {stab_b:.1f})", "COLLAPSE")
                        
                        # Transfer local IDs
                        for dp, lid in alias.drone_local_ids.items():
                            canonical.drone_local_ids[dp] = lid
                        
                        # Transfer assignment if canonical doesn't have one
                        if canonical.assigned_drone_port is None and alias.assigned_drone_port is not None:
                            canonical.assigned_drone_port = alias.assigned_drone_port
                            canonical.owner_port = alias.owner_port
                            canonical.ownership_state = alias.ownership_state
                        
                        # Absorb observation count
                        canonical.observation_count = max(canonical.observation_count, alias.observation_count)
                        
                        # Create alias mapping
                        self.target_aliases[alias_id] = canonical_id
                        
                        spatial_to_remove.append(alias_id)
                        spatial_processed.add(alias_id)
            
            # Cleanup spatial merges
            for mid in spatial_to_remove:
                if self.fusion_manager:
                    self.fusion_manager.remove_target(mid)
                if mid in self.targets:
                    del self.targets[mid]

    # =========================================================================
    # PERIODIC OPTIMAL ASSIGNMENT LOOP
    # =========================================================================
    
    def _periodic_optimization(self):
        """Periodic loop: owner loss detection, Hungarian assignment, fusion."""
        SwarmLogger.log("INFO", "LEADER", "Proactive Assignment Loop Started.", "OPT_LOOP")
        
        while self._assignment_loop_running:
            try:
                time.sleep(ASSIGNMENT_LOOP_INTERVAL_S)
                
                # RADAR SNAPSHOT - Liderin gördüğü tüm hedefler
                self._log_radar_snapshot()
                
                # Target fusion batch processing
                if self.fusion_manager:
                    try:
                        results = self.fusion_manager.process_observations()
                        with self.lock:
                            for t_id, (fused_pos, fused_cov) in results.items():
                                if t_id in self.targets:
                                    gps_pos = self.fusion_manager.ned_to_gps(fused_pos)
                                    self.targets[t_id].lat = gps_pos[0]
                                    self.targets[t_id].lon = gps_pos[1]
                    except Exception as e:
                        SwarmLogger.log("ERROR", "OPT_LOOP", f"Fusion Process Error: {e}", "OPT_LOOP")
                
                # Owner loss detection
                with self.lock:
                    now = time.time()
                    for t_id, t in list(self.targets.items()):
                        if t.ownership_state == TrackedTarget.OWNERSHIP_OWNED and t.owner_port is not None:
                            time_since_obs = now - t.last_observation_time
                            owner_lost_threshold = 3.0
                            owner_quality = t.observation_quality.get(t.owner_port, 0)
                            if time_since_obs > owner_lost_threshold and owner_quality > 0:
                                self._process_owner_loss(t)
                
                # Target collapse
                self._collapse_duplicate_targets()
                
                with self.lock:
                    pending_targets = [
                        t_id for t_id, t in self.targets.items()
                        if t.status == "PENDING" and t.assigned_drone_port is None
                    ]
                    
                    active_drones = set()
                    for port, vehicle in self.drone_manager.drones.items():
                        if vehicle != "connecting":
                            active_drones.add(port)
                    
                    assigned_drones = set(
                        t.assigned_drone_port for t in self.targets.values()
                        if t.assigned_drone_port is not None 
                        and t.status in ("PENDING", "ENGAGED")
                    )
                    
                    idle_drones = active_drones - assigned_drones
                
                # Proactive handoff: DISABLED - was overriding Hungarian assignment
                # Handoff should only happen via _process_secondary_observation with proper quality checks
                # if idle_drones:
                #     with self.lock:
                #         for idle_port in list(idle_drones):
                #             best_handoff_target = None
                #             best_handoff_dist = float('inf')
                #             
                #             for t_id, t in self.targets.items():
                #                 if t.ownership_state == TrackedTarget.OWNERSHIP_OWNED and t.owner_port != idle_port:
                #                     if idle_port in t.secondary_observations:
                #                         vehicle = self.drone_manager.drones.get(idle_port)
                #                         if vehicle and vehicle != "connecting":
                #                             try:
                #                                 loc = vehicle.location.global_relative_frame
                #                                 d_lat, d_lon = loc.lat, loc.lon
                #                                 dist = GeoMath.haversine_distance(d_lat, d_lon, t.lat, t.lon)
                #                                 if dist < best_handoff_dist:
                #                                     best_handoff_dist = dist
                #                                     best_handoff_target = t_id
                #                             except:
                #                                 pass
                #             
                #             if best_handoff_target and best_handoff_dist < 100.0:
                #                 t = self.targets[best_handoff_target]
                #                 old_owner = t.owner_port
                #                 SwarmLogger.log("HANDOFF", "LEADER", f"Proactive handoff: {best_handoff_target} from Drone_{old_owner} to Drone_{idle_port}", "OWNERSHIP")
                #                 self._initiate_handoff(t, idle_port)
                
                # Hungarian assignment
                if pending_targets and idle_drones:
                    assignments = self.assign_attack_targets()
                    if assignments:
                        SwarmLogger.log("ASSIGN", "OPT_LOOP", f"Optimal Assignment: {assignments}", "ASSIGN")
                
                # Cross-drone fusion
                fused = self.fuse_cross_drone_targets()
                if fused > 0:
                    SwarmLogger.log("FUSION", "OPT_LOOP", f"{fused} targets merged via fusion.", "FUSION")
                        
            except Exception as e:
                SwarmLogger.log("ERROR", "OPT_LOOP", f"Optimization Error: {e}", "OPT_LOOP")
            
            # Prune assignments for drones with stale heartbeats
            try:
                self._prune_stale_drones()
            except Exception as e:
                SwarmLogger.log("ERROR", "OPT_LOOP", f"Heartbeat prune error: {e}", "OPT_LOOP")
    
    def _prune_stale_drones(self):
        """Release assignments for drones that haven't sent a heartbeat within LEADER_TIMEOUT_S."""
        now = time.time()
        with self.lock:
            for t_id, target in self.targets.items():
                port = target.assigned_drone_port
                if port is None:
                    continue
                # Only check drones that have ever sent a heartbeat
                last_hb = self.drone_heartbeat_ts.get(port)
                if last_hb is not None and (now - last_hb) > LEADER_TIMEOUT_S:
                    SwarmLogger.log("FAILSAFE", "LEADER", f"Drone_{port} heartbeat stale ({now - last_hb:.1f}s). Releasing {t_id}.", "FAILSAFE")
                    self._release_drone(port)
    
    def _find_nearest_unassigned_target(self, drone_port, exclude_id=None):
        """Finds nearest unassigned target for a drone."""
        vehicle = self.drone_manager.drones.get(drone_port)
        if vehicle is None or vehicle == "connecting":
            return None
        
        try:
            loc = vehicle.location.global_relative_frame
            if not loc or loc.lat == 0:
                return None
            d_lat, d_lon = loc.lat, loc.lon
        except:
            return None
        
        best_id = None
        best_dist = float('inf')
        
        for t_id, target in self.targets.items():
            if t_id == exclude_id: continue
            if target.assigned_drone_port is not None: continue
            if target.status == "LOST": continue
            if target.track_state != "CONFIRMED": continue
            
            dist = GeoMath.haversine_distance(d_lat, d_lon, target.lat, target.lon)
            if dist < best_dist:
                best_dist = dist
                best_id = t_id
        
        return best_id

    def get_battlespace_state(self):
        """Returns full state for Visualization UI."""
        # Clean expired TTL aliases
        now = time.time()
        expired_aliases = [aid for aid, data in self.alias_ttl.items() if now > data["expires"]]
        for aid in expired_aliases:
            del self.alias_ttl[aid]

        with self.lock:
            state = {
                "targets": {},
                "drones": {}
            }
            
            # Determine Leader (lowest port)
            active_ports = sorted([p for p in self.drone_manager.drones.keys() if self.drone_manager.drones[p] != "connecting"])
            leader_port = active_ports[0] if active_ports else None

            # Targets (exclude LOST)
            for t_id, t in self.targets.items():
                if t.status != "LOST":
                    # UI visual grouping (TTL alias)
                    if t_id in self.alias_ttl:
                        continue
                    
                    cov_list = t.covariance.tolist() if hasattr(t.covariance, 'tolist') else t.covariance
                    
                    dist_leader = -1
                    if leader_port and leader_port in self.drone_manager.drones:
                        l_veh = self.drone_manager.drones[leader_port]
                        l_loc = l_veh.location.global_relative_frame
                        if l_loc:
                            dist_leader = GeoMath.haversine_distance(l_loc.lat, l_loc.lon, t.lat, t.lon)

                    # Dynamic aliases
                    combined_aliases = [aid for aid, cid in getattr(self, 'target_aliases', {}).items() if cid == t_id]
                    combined_aliases.extend([aid for aid, data in self.alias_ttl.items() if data["canonical"] == t_id])

                    state["targets"][t_id] = {
                        "lat": t.lat,
                        "lon": t.lon,
                        "status": t.status,
                        "track_state": t.track_state,
                        "assigned_to": t.assigned_drone_port,
                        "age": time.time() - t.first_seen_time,
                        "observation_count": t.observation_count,
                        "seen_by_drones": list(t.drone_local_ids.keys()),
                        "local_ids": t.drone_local_ids,
                        "covariance": cov_list,
                        "distance_to_leader": dist_leader,
                        "aliases": combined_aliases,
                        "is_group": t.is_group,
                        "group_member_count": t.group_member_count,
                    }
                
            # Drones
            for port, vehicle in self.drone_manager.drones.items():
                if vehicle != "connecting":
                    loc = vehicle.location.global_relative_frame
                    if loc:
                        controller = self.drone_manager.drone_controllers.get(port)
                        current_action = "IDLE"
                        engaged_tid = None
                        
                        if controller:
                            if controller.tracking_mode: current_action = "TRACKING"
                            elif controller.is_mission_active: current_action = "MISSION"
                            
                            if hasattr(controller, 'attack_approved') and controller.attack_approved:
                                current_action = "ENGAGING"
                                raw_tid = controller.current_target_id
                                # Resolve aliases so UI doesn't break if target was merged
                                visited = set()
                                while hasattr(self, 'target_aliases') and raw_tid in self.target_aliases and raw_tid not in visited:
                                    visited.add(raw_tid)
                                    raw_tid = self.target_aliases[raw_tid]
                                if raw_tid in self.alias_ttl:
                                    raw_tid = self.alias_ttl[raw_tid]["canonical"]
                                engaged_tid = raw_tid
                        
                        role = "LEADER" if port == leader_port else "FOLLOWER"

                        state["drones"][port] = {
                            "lat": loc.lat,
                            "lon": loc.lon,
                            "alt": loc.alt,
                            "heading": vehicle.heading,
                            "action": current_action,
                            "role": role,
                            "engaged_target_id": engaged_tid,
                            "lock_status": getattr(controller, 'lock_status', None)
                        }
            return state

    # =========================================================================
    # OPTIMAL ASSIGNMENT LOGIC (HUNGARIAN)
    # =========================================================================
    
    def assign_attack_targets(self) -> Dict[int, str]:
        """Assign unique targets to drones via Hungarian algorithm with ownership rules."""
        with self.lock:
            available_targets = []
            locked_assignments = {}
            
            protected_states = {"ENGAGED", "LOCKED", "CONFIRMED_ATTACK", "CENTERING", "ATTACKING", "ECHO_WAIT"}
            
            # Ownership model: only FREE targets can be assigned
            for t_id, t in self.targets.items():
                if t.track_state == "DELETED" or t.status == "LOST":
                    continue
                
                if t.status in protected_states:
                    if t.assigned_drone_port is not None:
                        locked_assignments[t.assigned_drone_port] = t_id
                    continue
                
                if t.ownership_state == TrackedTarget.OWNERSHIP_OWNED:
                    if t.owner_port is not None:
                        locked_assignments[t.owner_port] = t_id
                    continue
                elif t.ownership_state == TrackedTarget.OWNERSHIP_RESERVED:
                    continue
                elif t.ownership_state == TrackedTarget.OWNERSHIP_HANDOFF:
                    continue
                
                if t.ownership_state == TrackedTarget.OWNERSHIP_FREE:
                    if t.lat is not None and t.lon is not None:
                        available_targets.append((t_id, t.lat, t.lon))

            # De-duplicate available_targets by target_id
            if available_targets:
                dedup = {}
                for tid, lat, lon in available_targets:
                    if tid not in dedup:
                        dedup[tid] = (tid, lat, lon)
                available_targets = list(dedup.values())
            
            # Available drones (not locked and capacity compliant)
            available_drones = []
            active_drone_ports = set(locked_assignments.keys())

            for port, vehicle in self.drone_manager.drones.items():
                if vehicle != "connecting":
                    if ENABLE_CAPACITY_GUARD and port in active_drone_ports:
                        SwarmLogger.log("GUARD", "LEADER", f"CAPACITY_GUARD: Drone_{port} already ACTIVE on {locked_assignments[port]}", "ASSIGN")
                        continue
                    if port in locked_assignments:
                        continue
                    loc = vehicle.location.global_relative_frame
                    if loc and loc.lat != 0:
                        available_drones.append((port, loc.lat, loc.lon))
            
            if not available_drones or not available_targets:
                return locked_assignments
            
            n_drones = len(available_drones)
            n_targets = len(available_targets)
            
            # Cost matrix - Enhanced Scoring System
            # RULE: LOWER cost = HIGHER priority
            cost_matrix = np.zeros((n_drones, n_targets))
            
            # Collect assigned target positions for distribution calculation
            assigned_targets_info = {}
            for port, tid in locked_assignments.items():
                if tid in self.targets:
                    t = self.targets[tid]
                    assigned_targets_info[port] = (tid, t.lat, t.lon)
            
            for i, (d_port, d_lat, d_lon) in enumerate(available_drones):
                vehicle = self.drone_manager.drones.get(d_port)
                
                for j, (t_id, t_lat, t_lon) in enumerate(available_targets):
                    t_obj = self.targets[t_id]
                    
                    # === BASE: Fixed starting point ===
                    cost = 500.0
                    
                    # === DISTANCE: Dynamic - Near = BONUS, Far = PENALTY ===
                    dist = GeoMath.haversine_distance(d_lat, d_lon, t_lat, t_lon)
                    # Linear: 0m = -300 bonus, 100m = 0, 200m = +300 penalty
                    cost += (dist - 100.0) * 3.0
                    
                    # === COVARIANCE: Low = bonus, High = penalty ===
                    if t_obj.covariance is not None:
                        try:
                            cov_trace = float(t_obj.covariance[0, 0] + t_obj.covariance[1, 1])
                            if cov_trace < 20:  # Low uncertainty = good
                                cost -= 80.0
                            elif cov_trace > 50:  # High uncertainty = bad
                                cost += min(100.0, cov_trace)
                        except:
                            pass
                    
                    # === VISIBILITY: Seeing target = BONUS ===
                    is_seeing_target = (d_port in t_obj.drone_local_ids)
                    if is_seeing_target:
                        cost -= 100.0
                    else:
                        cost += 150.0  # Not seeing = penalty
                    
                    # === ANGLE: Target in front = BONUS, behind = PENALTY ===
                    if vehicle and hasattr(vehicle, 'heading'):
                        drone_heading = vehicle.heading
                        bearing_to_target = GeoMath.get_bearing_to_target(d_lat, d_lon, t_lat, t_lon)
                        diff = (bearing_to_target - drone_heading + 180) % 360 - 180
                        if abs(diff) <= 45:  # Front 90° sector
                            cost -= 60.0
                        elif abs(diff) > 90:  # Behind
                            cost += 200.0
                    
                    # === DISTRIBUTION: Dynamic - Far from locked = bonus, Close = HEAVY penalty ===
                    if assigned_targets_info:
                        min_dist_to_locked = float('inf')
                        locked_target_status = None
                        for other_port, (other_tid, other_lat, other_lon) in assigned_targets_info.items():
                            if other_port != d_port:
                                dist_to_locked = GeoMath.haversine_distance(t_lat, t_lon, other_lat, other_lon)
                                if dist_to_locked < min_dist_to_locked:
                                    min_dist_to_locked = dist_to_locked
                                    other_t = self.targets.get(other_tid)
                                    if other_t:
                                        locked_target_status = other_t.status
                        
                        if min_dist_to_locked < float('inf'):
                            # Base: 0m = +500 penalty, 100m = 0, 200m = -300 bonus
                            cost += (100.0 - min_dist_to_locked) * 5.0
                            
                            # EXTRA: If target is LOCKED, add additional penalty for close targets
                            if locked_target_status == "LOCKED" and min_dist_to_locked < 50.0:
                                cost += 300.0  # Extra penalty for being close to a LOCKED target
                    
                    # === STICKINESS: Reduced bonus + proximity penalty to others' targets ===
                    if t_obj.assigned_drone_port == d_port:
                        cost = cost * 0.85  # 15% reduction (was 40%)
                    
                    # === SAFETY: Minimum cost ===
                    cost_matrix[i, j] = max(0.0, cost)
            
            assignments = locked_assignments.copy()
            
            # Clear dangling pending assignments to prevent unassigned target hoarding
            for t_id, _, _ in available_targets:
                port_to_release = self.targets[t_id].assigned_drone_port
                if port_to_release:
                    self._release_drone(port_to_release)
            
            if HUNGARIAN_AVAILABLE and n_drones > 0 and n_targets > 0:
                try:
                    row_ind, col_ind = linear_sum_assignment(cost_matrix)
                    
                    for i, j in zip(row_ind, col_ind):
                        if j < n_targets:
                            drone_port = available_drones[i][0]
                            target_id = available_targets[j][0]
                            
                            self._assign_drone(drone_port, target_id)
                            
                            assignments[drone_port] = target_id
                            
                            score = cost_matrix[i, j] # Isolate Dist vs Score
                            d_lat, d_lon = available_drones[i][1], available_drones[i][2]
                            t_lat, t_lon = available_targets[j][1], available_targets[j][2]
                            actual_dist = GeoMath.haversine_distance(d_lat, d_lon, t_lat, t_lon)
                            
                            SwarmLogger.log("ASSIGN", "LEADER", f"HUNGARIAN: {target_id} -> DRONE_{drone_port} (Dist: {actual_dist:.1f}m | Score: {score:.1f})", "ASSIGN")
                            
                except Exception as e:
                    SwarmLogger.log("ERROR", "LEADER", f"Hungarian Error: {e}", "ASSIGN")
            
            else:
                # Greedy Fallback
                assigned_targets_local = set()
                assigned_drones_local = set()
                
                pairs = []
                for i, (d_port, d_lat, d_lon) in enumerate(available_drones):
                    for j, (t_id, t_lat, t_lon) in enumerate(available_targets):
                        pairs.append((cost_matrix[i, j], d_port, t_id))
                
                pairs.sort(key=lambda x: x[0])
                
                for dist, d_port, t_id in pairs:
                    if d_port not in assigned_drones_local and t_id not in assigned_targets_local:
                        self._assign_drone(d_port, t_id)
                        
                        assignments[d_port] = t_id
                        assigned_drones_local.add(d_port)
                        assigned_targets_local.add(t_id)
                        
                        SwarmLogger.log("ASSIGN", "LEADER", f"GREEDY: {t_id} -> DRONE_{d_port} (Dist: {dist:.1f}m)", "ASSIGN")
            
            return assignments
    
    def fuse_cross_drone_targets(self) -> int:
        """Fuse duplicate targets observed by different drones. Returns count."""
        with self.lock:
            fused_count = 0
            target_ids = list(self.targets.keys())
            to_remove = []
            
            for i in range(len(target_ids)):
                id1 = target_ids[i]
                if id1 in to_remove or id1 not in self.targets:
                    continue
                t1 = self.targets[id1]
                
                for j in range(i + 1, len(target_ids)):
                    id2 = target_ids[j]
                    if id2 in to_remove or id2 not in self.targets:
                        continue
                    t2 = self.targets[id2]

                    # Never merge different group clusters
                    if getattr(t1, 'is_group', False) or getattr(t2, 'is_group', False):
                        if bool(getattr(t1, 'is_group', False)) != bool(getattr(t2, 'is_group', False)):
                            continue
                        g1 = getattr(t1, 'group_id_local', None)
                        g2 = getattr(t2, 'group_id_local', None)
                        if g1 is not None and g2 is not None and g1 != g2:
                            continue

                    # Attack/immutability guard
                    if (t1.status in self.IMMUTABLE_STATES or t2.status in self.IMMUTABLE_STATES or
                        getattr(t1, 'approved_for_attack', False) or getattr(t2, 'approved_for_attack', False)):
                        continue
                    
                    # Same drone source check
                    if t1.assigned_drone_port is not None and t1.assigned_drone_port == t2.assigned_drone_port:
                        if t1.tracker_id is not None and t2.tracker_id is not None:
                            if t1.tracker_id != t2.tracker_id:
                                continue
                    
                    # Fusion logic
                    should_merge, mahal_dist, effect_dist = should_fuse_targets(
                        (t1.lat, t1.lon), (t2.lat, t2.lon),
                        t1.covariance[:2, :2] if t1.covariance is not None else None,
                        t2.covariance[:2, :2] if t2.covariance is not None else None,
                        threshold_m=SWARM_MERGE_DISTANCE_M
                    )
                    
                    if should_merge:
                        keep_id, drop_id = id1, id2
                        keep_t, drop_t = t1, t2
                        
                        # Immutability guard
                        if (keep_t.status in self.IMMUTABLE_STATES or drop_t.status in self.IMMUTABLE_STATES or
                            getattr(keep_t, 'approved_for_attack', False) or getattr(drop_t, 'approved_for_attack', False)):
                            continue
                        
                        protected_states = {"ENGAGED", "LOCKED", "CONFIRMED_ATTACK", "CENTERING", "ATTACKING", "ECHO_WAIT"}
                        
                        # Canonical target selection
                        if t2.status in protected_states and t1.status not in protected_states:
                            keep_id, drop_id = id2, id1
                            keep_t, drop_t = t2, t1
                        elif t2.status in protected_states and t1.status in protected_states:
                            if t1.first_seen_time > t2.first_seen_time:
                                keep_id, drop_id = id2, id1
                                keep_t, drop_t = t2, t1
                        
                        # Covariance fusion
                        if drop_t.covariance is not None:
                            try:
                                measurements = [
                                    {'position': [keep_t.lat, keep_t.lon, 0], 'covariance': keep_t.covariance},
                                    {'position': [drop_t.lat, drop_t.lon, 0], 'covariance': drop_t.covariance}
                                ]
                                fused_pos, fused_cov = weighted_covariance_fusion(measurements)
                                keep_t.lat, keep_t.lon = fused_pos[0], fused_pos[1]
                                keep_t.covariance = fused_cov
                            except:
                                pass
                        
                        if drop_t.assigned_drone_port:
                            port = drop_t.assigned_drone_port
                            is_busy_elsewhere = False
                            for _tid, targ in self.targets.items():
                                if _tid != drop_id and targ.assigned_drone_port == port and targ.track_state != "DELETED":
                                    if targ.status in {"PENDING", "ACTIVE"} or self.is_target_in_attack_pipeline(targ):
                                        is_busy_elsewhere = True
                                        break
                            
                            if keep_t.assigned_drone_port is None and not is_busy_elsewhere:
                                self._release_drone(port)
                                self._assign_drone(port, keep_id)
                                keep_t.status = drop_t.status
                                SwarmLogger.log("INFO", "LEADER", f"FUSION: {keep_id} & {drop_id} Merged. Assignment transferred to Drone_{port}.", "FUSION")
                            elif keep_t.assigned_drone_port is not None and not is_busy_elsewhere:
                                self.drone_active_target[port] = keep_id
                                if drop_id in self.target_owner and self.target_owner[drop_id] == port:
                                    del self.target_owner[drop_id]
                                drop_t.assigned_drone_port = None
                                SwarmLogger.log("INFO", "LEADER", f"FUSION: {keep_id} & {drop_id} Merged. Drone_{port} detached.", "FUSION")
                            else:
                                self._release_drone(port)
                                SwarmLogger.log("WARNING", "LEADER", f"FUSION: {keep_id} & {drop_id} Merged. Drone_{port} Released.", "FUSION")
                        
                        # Merge local ID maps
                        for port, lid in drop_t.drone_local_ids.items():
                            keep_t.drone_local_ids[port] = lid
                        
                        # Create alias for dropped target
                        if not hasattr(self, 'target_aliases'):
                            self.target_aliases = {}
                        self.target_aliases[drop_id] = keep_id
                        
                        to_remove.append(drop_id)
                        fused_count += 1
            
            for tid in to_remove:
                if tid in self.targets:
                    del self.targets[tid]
            
            return fused_count
    
    def get_assigned_local_id(self, drone_port: int) -> Optional[int]:
        """Returns the Local Tracker ID of the target assigned to this drone."""
        with self.lock:
            for t_id, target in self.targets.items():
                if target.assigned_drone_port == drone_port and target.status == "ENGAGED":
                    return target.drone_local_ids.get(drone_port)
        return None

    # =========================================================================
    # 7-STEP ATTACK PROTOCOL HANDLERS
    # =========================================================================

    def _handle_echo_target(self, drone_port, target_id, raw_data):
        """Step 2-5: Echo validation, cross-check, ghost detection, approval."""
        with self.lock:
            if target_id not in self.targets:
                SwarmLogger.log("PROTOCOL", f"DRONE_{drone_port}", f"ECHO for {target_id} REJECTED — target no longer exists", "LEADER")
                return "SEARCH", None, None
            
            target = self.targets[target_id]
            
            if target.assigned_drone_port != drone_port:
                SwarmLogger.log("PROTOCOL", f"DRONE_{drone_port}", f"ECHO for {target_id} REJECTED — assigned to Drone_{target.assigned_drone_port}", "LEADER")
                return "SEARCH", None, None
            
            drone_gps = raw_data.get("drone_gps")
            local_tracker_id = raw_data.get("local_tracker_id")
            
            SwarmLogger.log("PROTOCOL", f"DRONE_{drone_port}", f"ECHO received: {target_id} | DroneGPS={drone_gps} | LocalID={local_tracker_id}", "LEADER")
            
            # Cross-validation: duplicate lock check
            for other_tid, other_target in self.targets.items():
                if other_tid == target_id:
                    continue
                if other_target.status in ("CENTERING", "ATTACKING", "CONFIRMED_ATTACK"):
                    dist = GeoMath.haversine_distance(target.lat, target.lon, other_target.lat, other_target.lon)
                    if dist < SWARM_MERGE_DISTANCE_M:
                        SwarmLogger.log("PROTOCOL", f"DRONE_{drone_port}", f"ECHO REJECTED: {target_id} too close ({dist:.1f}m) to {other_tid} — GHOST", "LEADER")
                        alt = self._find_nearest_unassigned_target(drone_port, exclude_id=target_id)
                        if alt:
                            alt_target = self.targets[alt]
                            self._release_drone(drone_port)
                            self._assign_drone(drone_port, alt)
                            alt_target.status = "ECHO_WAIT"
                            alt_target.approved_for_attack = True
                            SwarmLogger.log("PROTOCOL", f"DRONE_{drone_port}", f"REASSIGNED: {target_id} → {alt}", "LEADER")
                            alt_local_id = alt_target.drone_local_ids.get(drone_port)
                            return "REASSIGN", alt, alt_local_id
                        else:
                            self._release_drone(drone_port)
                            return "SEARCH", None, None
            
            # Cross-validation: duplicate drone check
            for other_tid, other_target in self.targets.items():
                if other_tid == target_id:
                    continue
                if other_target.assigned_drone_port == drone_port and other_target.status in ("CENTERING", "ATTACKING", "CONFIRMED_ATTACK"):
                    SwarmLogger.log("PROTOCOL", f"DRONE_{drone_port}", f"ECHO REJECTED: Drone already has active attack on {other_tid}", "LEADER")
                    self._release_drone(drone_port)
                    return "SEARCH", None, None
            
            # All checks passed: approve
            target.status = "CONFIRMED_ATTACK"
            SwarmLogger.log("PROTOCOL", f"DRONE_{drone_port}", f"ATTACK CONFIRMED: {target_id} → CENTER", "LEADER")
            local_t_id = target.drone_local_ids.get(drone_port)
            return "ATTACK_CONFIRMED", target_id, local_t_id

    def _handle_locked_data(self, drone_port, target_id, raw_data):
        """Handle LOCKED_DATA message from drone during attack."""
        with self.lock:
            if target_id not in self.targets:
                return "SEARCH", None, None
            
            target = self.targets[target_id]
            
            if target.assigned_drone_port != drone_port:
                return "SEARCH", None, None
            
            target_gps = raw_data.get("target_gps")
            if target_gps and target_gps[0] is not None:
                target.lat, target.lon = target_gps[0], target_gps[1]
                target.last_seen_time = time.time()
                target.last_observation_time = time.time()
        
        return "ATTACK", target_id, None

    def _handle_attacking(self, drone_port, target_id):
        """Step 7: Drone notifies diving - fire-and-forget."""
        with self.lock:
            if target_id in self.targets:
                target = self.targets[target_id]
                if target.assigned_drone_port == drone_port:
                    target.status = "ATTACKING"
                    SwarmLogger.log("ATTACK_FINAL", f"DRONE_{drone_port}", f"{target_id} | DIVING — autonomous kamikaze started", "LEADER")
        return "ATTACK", target_id, None

    def _handle_lock_request(self, drone_port, target_id, raw_data):
        """Step 2: Handshake - Drone requests lock on a target."""
        with self.lock:
            # Alias resolution
            if not hasattr(self, 'target_aliases'):
                self.target_aliases = {}
            if target_id in self.target_aliases and target_id not in self.targets:
                alias_id = target_id
                target_id = self.target_aliases[target_id]
                SwarmLogger.log("ALIAS", "HANDSHAKE", f"Resolved {alias_id} -> {target_id}", "ALIAS")

            if target_id not in self.targets:
                return {"action": "SEARCH", "target_id": None} 
            
            target = self.targets[target_id]
            
            # BLOCK: Drone already has a different target locked
            if drone_port in self.drone_active_target and self.drone_active_target[drone_port] is not None:
                current_target = self.drone_active_target[drone_port]
                if current_target != target_id:
                    SwarmLogger.log("LOCK_BLOCKED", "HANDSHAKE", f"Lock REJECTED for {target_id}: Drone_{drone_port} already has target {current_target}", "LEADER")
                    return {"action": "SEARCH", "target_id": None}
            
            # Already locked by another?
            if target.status == "LOCKED" and target.assigned_drone_port != drone_port:
                SwarmLogger.log("INFO", "HANDSHAKE", f"Lock Request for {target_id} by Drone_{drone_port} REJECTED (Already Locked by {target.assigned_drone_port})", "LEADER")
                return {"action": "SEARCH", "target_id": None} 
            
            # Ghost check
            for other_tid, other_target in self.targets.items():
                if other_tid != target_id and other_target.status == "LOCKED":
                    dist = GeoMath.haversine_distance(target.lat, target.lon, other_target.lat, other_target.lon)
                    if dist < SWARM_MERGE_DISTANCE_M:
                        SwarmLogger.log("WARN", "HANDSHAKE", f"Lock REJECTED for {target_id}. Too close ({dist:.1f}m) to LOCKED target {other_tid}", "LEADER")
                        return {"action": "SEARCH", "target_id": None}

            # Approve
            self._assign_drone(drone_port, target_id)
            local_t_id = target.drone_local_ids.get(drone_port)
            SwarmLogger.log("INFO", "HANDSHAKE", f"Lock Request for {target_id} by Drone_{drone_port} APPROVED", "LEADER")
            return {"action": "ATTACK_APPROVED", "target_id": target_id, "tracker_id": local_t_id}

    def _handle_lock_confirmation(self, drone_port, target_id):
        """Step 3b: Lock Confirmation."""
        with self.lock:
            # Alias resolution
            if not hasattr(self, 'target_aliases'):
                self.target_aliases = {}
            if target_id in self.target_aliases and target_id not in self.targets:
                target_id = self.target_aliases[target_id]

            if target_id in self.targets:
                target = self.targets[target_id]
                if target.assigned_drone_port == drone_port:
                    target.status = "LOCKED"
                    SwarmLogger.log("INFO", "HANDSHAKE", f"Target {target_id} LOCKED by Drone_{drone_port}", "LEADER")
                    return {"action": "ACK", "target_id": target_id}
        return {"action": "SEARCH", "target_id": None}

    def get_drone_targets_buffer(self):
        """Returns visual buffer of targets detected by each drone."""
        return self.drone_targets
