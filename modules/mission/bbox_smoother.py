"""Bbox Temporal Smoothing - Protects bbox center from frame-to-frame jitter.

This module smooths small bbox center fluctuations from YOLO/BoT-SORT.
This prevents unnecessary jumps in GPS calculations.
"""

import time
from typing import Dict, Optional, Tuple
from collections import deque

from config import (
    BBOX_SMOOTH_ALPHA,
    BBOX_SMOOTH_MIN_CONF,
    BBOX_SIZE_STABILITY_THRESHOLD,
    BBOX_FILTER_WINDOW_SIZE,
)
from modules.core.logger import SwarmLogger


class BboxSmoother:
    """Exponential Moving Average smoother for bbox centers.
    
    Features:
    - Confidence-weighted smoothing: Higher confidence = less smoothing
    - Size stability check: Reduce smoothing if bbox size changes significantly
    - Track ID based history: Separate state for each track
    
    NOTE: First version only does center smoothing.
    Bbox size smoothing can be added in Phase 2 if needed.
    """
    
    def __init__(self, alpha: float = BBOX_SMOOTH_ALPHA, 
                 window_size: int = BBOX_FILTER_WINDOW_SIZE):
        self.alpha = alpha
        self.window_size = window_size
        # track_id -> {smoothed_cx, smoothed_cy, last_w, last_h, last_update, history}
        self._history: Dict[int, dict] = {}
        # Last cleanup time for stale entries
        self._last_cleanup = time.time()
        # Cleanup interval (seconds)
        self._cleanup_interval = 30.0
    
    def smooth(self, track_id: Optional[int], cx: float, cy: float,
               conf: float, w: float, h: float) -> Tuple[float, float]:
        """Smooth bbox center.
        
        Args:
            track_id: Tracker ID (no smoothing if None)
            cx: Raw bbox center X
            cy: Raw bbox center Y
            conf: Detection confidence (0-1)
            w: Bbox width
            h: Bbox height
            
        Returns:
            (smoothed_cx, smoothed_cy): Smoothed center
        """
        # No smoothing if no track ID
        if track_id is None:
            return cx, cy
        
        # No smoothing if confidence is too low
        if conf < BBOX_SMOOTH_MIN_CONF:
            return cx, cy
        
        # Periodic cleanup
        self._cleanup_if_needed()
        
        # New track?
        if track_id not in self._history:
            self._history[track_id] = {
                'smoothed_cx': cx,
                'smoothed_cy': cy,
                'last_w': w,
                'last_h': h,
                'last_update': time.time(),
                'history': deque(maxlen=self.window_size),
            }
            self._history[track_id]['history'].append((cx, cy))
            SwarmLogger.log("BBOX_SMOOTH", "BboxSmoother",
                f"Track {track_id}: First detection, smoothing initialized", "VISION")
            return cx, cy
        
        entry = self._history[track_id]
        
        # Size stability check
        size_ratio_change = self._calc_size_ratio_change(entry['last_w'], entry['last_h'], w, h)
        
        # Has size changed significantly? (likely new target or ID switch)
        if size_ratio_change > BBOX_SIZE_STABILITY_THRESHOLD:
            SwarmLogger.log("BBOX_SMOOTH", "BboxSmoother",
                f"Track {track_id}: Size instability detected (ratio={size_ratio_change:.2f}), reset smoothing", "VISION")
            # Reset smoothing - likely new target with same ID
            entry['smoothed_cx'] = cx
            entry['smoothed_cy'] = cy
            entry['last_w'] = w
            entry['last_h'] = h
            entry['history'].clear()
            entry['history'].append((cx, cy))
            return cx, cy
        
        # Confidence-weighted alpha
        # Higher confidence = less smoothing (trust raw value more)
        # Lower confidence = more smoothing (trust previous value more)
        # Fix: Old formula was inverted (conf↑ → alpha↓)
        # New: conf=1.0 → alpha×1.0, conf=0.0 → alpha×0.5
        effective_alpha = self.alpha * (0.5 + conf * 0.5)
        
        # EMA update
        smoothed_cx = entry['smoothed_cx'] + effective_alpha * (cx - entry['smoothed_cx'])
        smoothed_cy = entry['smoothed_cy'] + effective_alpha * (cy - entry['smoothed_cy'])
        
        # Update entry
        entry['smoothed_cx'] = smoothed_cx
        entry['smoothed_cy'] = smoothed_cy
        entry['last_w'] = w
        entry['last_h'] = h
        entry['last_update'] = time.time()
        entry['history'].append((cx, cy))
        
        # Log (throttled)
        delta_x = cx - smoothed_cx
        delta_y = cy - smoothed_cy
        if abs(delta_x) > 2 or abs(delta_y) > 2:
            SwarmLogger.log_throttled("BBOX_SMOOTH", "BboxSmoother",
                f"Track {track_id}: Smoothed ({cx:.1f},{cy:.1f}) → ({smoothed_cx:.1f},{smoothed_cy:.1f}) | "
                f"delta=({delta_x:.1f},{delta_y:.1f}) | conf={conf:.2f}",
                "VISION", interval_s=5.0)
        
        return smoothed_cx, smoothed_cy
    
    def _calc_size_ratio_change(self, old_w: float, old_h: float, 
                                 new_w: float, new_h: float) -> float:
        """Calculate bbox size change ratio.
        
        Returns:
            0 = no change
            1 = 100% change (doubled or halved)
        """
        if old_w <= 0 or old_h <= 0:
            return 0.0
        
        w_ratio = max(new_w / old_w, old_w / new_w) - 1.0
        h_ratio = max(new_h / old_h, old_h / new_h) - 1.0
        
        return max(w_ratio, h_ratio)
    
    def _cleanup_if_needed(self):
        """Clean up old track entries."""
        now = time.time()
        if now - self._last_cleanup < self._cleanup_interval:
            return
        
        self._last_cleanup = now
        expired = []
        
        # Mark entries older than 10 seconds
        for track_id, entry in self._history.items():
            if now - entry['last_update'] > 10.0:
                expired.append(track_id)
        
        # Delete
        for track_id in expired:
            del self._history[track_id]
        
        if expired:
            SwarmLogger.log("BBOX_SMOOTH", "BboxSmoother",
                f"Cleanup: {len(expired)} old tracks deleted", "VISION")
    
    def reset(self, track_id: int):
        """Reset smoothing history for a specific track."""
        if track_id in self._history:
            del self._history[track_id]
    
    def reset_all(self):
        """Reset all smoothing history."""
        self._history.clear()
        SwarmLogger.log("BBOX_SMOOTH", "BboxSmoother",
            "All smoothing history reset", "VISION")
    
    def get_jitter_std(self, track_id: int) -> Optional[float]:
        """Return bbox center jitter standard deviation for a track.
        
        This value can be used for covariance enhancement (Phase 2).
        
        Returns:
            Jitter std (pixels) or None (if not enough data)
        """
        if track_id not in self._history:
            return None
        
        history = self._history[track_id]['history']
        if len(history) < 3:
            return None
        
        # Calculate separate std for X and Y, then average
        xs = [h[0] for h in history]
        ys = [h[1] for h in history]
        
        import numpy as np
        std_x = np.std(xs)
        std_y = np.std(ys)
        
        return (std_x + std_y) / 2.0
