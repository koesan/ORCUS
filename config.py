"""
ORCUS - Swarm Kamikaze Drone System Configuration

Physical drone settings, mission control parameters,
ROS camera integration, and status messages.
"""

import os
import math

# ==============================================================================
# DRONE SETTINGS
# ==============================================================================
# PROJECT ROOT DIRECTORY (Automatically detected)
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

TAKEOFF_ALTITUDE = 5                # Takeoff altitude (meters)
TAKEOFF_ARM_WAIT_S = 1.0            # Wait time for armable state (seconds)
TAKEOFF_ARMING_CYCLE_S = 0.5        # Wait time during arming cycle (seconds)
TAKEOFF_POST_DELAY_S = 1.0          # Delay after takeoff complete (seconds)
CRITICAL_BATTERY_LEVEL = 20         # RTH Trigger Battery Level (%)
MAX_TOUR_COUNT = 1                  # Number of times to repeat a mission
SQUARE_SIZE = 20                    # Size of the observation area (meters)
DRONE_SPEED = 90                    # Drone speed (cm/s)
CONNECTION_TIMEOUT = 20             # Max wait time for drone connection (seconds)
DRONE_CONNECTION_RETRY_COUNT = 3    # Connection retry count
DRONE_CONNECTION_RETRY_DELAY = 5    # Delay between connection retries (seconds)

# ==============================================================================
# MISSION CONTROL PARAMETERS
# ==============================================================================
MAX_SPEED_M_S = 4.0                 # Max speed (m/s)
CONTROL_INTERVAL_S = 0.15           # PID control loop interval (seconds)
CELL_REACHED_THRESHOLD_M = 1.2      # Threshold to consider cell center reached (meters)
FINE_APPROACH_THRESHOLD_M = 7.0     # Distance threshold for deceleration (meters)
FINE_APPROACH_SCALE = 0.35          # Approach speed scale
CENTER_CONFIRM_TOLERANCE_M = 1.5    # Deviation tolerance for center confirmation (meters)
CENTER_CONFIRM_HOLD_S = 0.2         # Hold time for center confirmation (seconds)
FINE_APPROACH_HOLD_S = 0.5          # Short hold time for center confirmation (seconds)
PER_CELL_TIMEOUT_S = 60.0           # Max time to reach a cell (seconds)
RETRY_LIMIT = 2                     # Retry limit before skipping cell
GPS_MEDIAN_WINDOW = 7               # GPS median filter window size
GPS_FILTER_WINDOW_SIZE = 15         # Moving average filter size (for GPS stabilization)
BBOX_FILTER_WINDOW_SIZE = 15        # Bbox width filter window size (to prevent distance jumps)
STUCK_MOVED_THRESHOLD_M = 0.12      # Movement threshold for stuck detection (meters)
STUCK_TIMEOUT_S = 8.0               # Timeout for stuck detection (seconds)
PID_KP = 0.9
PID_KI = 0.02
PID_KD = 0.12

# ==============================================================================
# SWARM SYSTEM & SWITCHER SETTINGS
# ==============================================================================
RTL_LANDING_TIMEOUT_S = 30.0        # RTL landing wait time after critical battery (seconds)
MONITOR_INTERVAL_S = 1.0            # Battery monitor check interval (seconds)

# ==============================================================================
# CAMERA AND ROS SETTINGS
# ==============================================================================
# Dictionary specifying ROS topic for each port
CAMERA_TOPICS = {
    5760: "/webcam/image_raw",
    5761: "/webcam/image_raw",
    5762: "/webcam/image_raw",
    5763: "/webcam/image_raw",
    5770: "/webcam_2/image_raw_2",
    5771: "/webcam_2/image_raw_2",
    5772: "/webcam_2/image_raw_2",
    5773: "/webcam_2/image_raw_2"
}

LOG_THROTTLE_SEC = 1.0              # Log write frequency (seconds) - Increased to reduce spam
JPEG_QUALITY = 80                   # JPEG compression quality (0-100)

# ==============================================================================
# IMAGE PROCESSING & PLACEHOLDER SETTINGS
# ==============================================================================
PLACEHOLDER_IMAGE_SIZE = (640, 480) # Size of placeholder image when no stream
FONT_SCALE = 0.6                    # Font scale for placeholder text
FONT_THICKNESS = 1                  # Font thickness for placeholder text

# ==============================================================================
# AI (YOLO) SETTINGS
# ==============================================================================
YOLO_MODEL_PATH = os.path.join(PROJECT_ROOT, "modules", "vision", "tracker", "weights", "yolov12n.pt")
YOLO_CONF_THRESHOLD = 0.25          # Confidence threshold
YOLO_IOU_THRESHOLD = 0.40           # IoU threshold

# Inference Device (cuda:0, cuda:1, or cpu)
INFERENCE_DEVICE = '0'              # GPU device ID ('0' = cuda:0, 'cpu' = CPU)

# ==============================================================================
# COLLISION MISSION SETTINGS
# ==============================================================================
COLLISION_SCREEN_THRESHOLD = 0.40     # Collision assumed if target covers >40% of screen
COLLISION_FORWARD_SPEED = 2.0         # Constant forward speed during tracking (m/s)
HUMAN_LOST_TIMEOUT = 5.0              # Keep moving for this duration after human lost (seconds)
APPROACH_MIN_SCREEN_COVERAGE = 0.001  # 0.1% - Below this, aggressive approach mode
APPROACH_BOOST_SPEED = 4.0            # Boost speed if target is very small (m/s)
RESUME_SCAN_AFTER_LOST = True         # Resume scanning if target lost (True) or RTL (False)

# 360° Scanning Settings
ROTATION_YAW_INCREMENT = 45           # Rotation angle per step (degrees)
ROTATION_YAW_SPEED = 10               # Rotation speed (deg/s) - reduced by 20% for target obs accumulation
ROTATION_DIRECTION = 1                # Rotation direction (1=CW, -1=CCW)

# ==============================================================================
# PID CONTROL PARAMETERS (TRACKING)
# ==============================================================================
# YAW (Horizontal) PID Parameters
TRACKING_YAW_PID_TAU = 0.1            # Filter time constant
TRACKING_YAW_PID_KP = 1.5             # Proportional gain
TRACKING_YAW_PID_KI = 0.01            # Integral gain
TRACKING_YAW_PID_KD = 0.08            # Derivative gain
TRACKING_YAW_PID_INT_MAX = 0.01       # Integrator max limit (Anti-windup)
TRACKING_YAW_PID_INT_MIN = -0.01      # Integrator min limit
TRACKING_YAW_PID_OUT_MAX = 3.14159    # Output max (rad/s)
TRACKING_YAW_PID_OUT_MIN = -3.14159   # Output min (rad/s)
TRACKING_YAW_DEADZONE_DEG = 5.0       # Deadzone to prevent oscillation (degrees)

# PITCH (Vertical) PID Parameters
TRACKING_PITCH_PID_TAU = 0.1
TRACKING_PITCH_PID_KP = 2.5
TRACKING_PITCH_PID_KI = 0.03
TRACKING_PITCH_PID_KD = 0.08
TRACKING_PITCH_PID_INT_MAX = 0.05
TRACKING_PITCH_PID_INT_MIN = -0.05
TRACKING_PITCH_PID_OUT_MAX = 10.0     # Output max (m/s)
TRACKING_PITCH_PID_OUT_MIN = -10.0    # Output min (m/s)

# ==============================================================================
# CAMERA OPTICAL PARAMETERS (FROM SDF)
# ==============================================================================
# Source: simulator/drone/drone1/model.sdf
# All drones share the same camera structure

# Raw SDF Values
CAMERA_WIDTH = 1280                   # Image width (px)
CAMERA_HEIGHT = 720                   # Image height (px)
CAMERA_HFOV = 1.2                     # Horizontal FOV (radians) ~69 degrees
CAMERA_PITCH_OFFSET = 0.1309          # Camera pitch offset (radians) ~7.5 degrees

# Calculated Intrinsic Matrix Values
# fx = width / (2 * tan(HFOV/2))
CAMERA_FX = CAMERA_WIDTH / (2 * math.tan(CAMERA_HFOV / 2))  # ~467.7 px
CAMERA_FY = CAMERA_FX                 # Assuming square pixels
CAMERA_CX = CAMERA_WIDTH / 2.0        # Optical center X
CAMERA_CY = CAMERA_HEIGHT / 2.0       # Optical center Y

# Compatibility Alias
CAMERA_RESOLUTION_WIDTH = CAMERA_WIDTH

# SDF-Synchronized Parameters (from simulator/drone/model.sdf & worlds/multi_drone.world)
CAMERA_UPDATE_RATE_FPS = 30           # Camera sensor update rate (Hz)
CAMERA_CLIP_NEAR = 0.1               # Camera near clip distance (m)
CAMERA_CLIP_FAR = 1000.0             # Camera far clip distance (m)
SIM_FDM_PORTS = {                    # ArduPilot FDM port pairs per drone
    "drone1": {"in": 9002, "out": 9003},
    "drone2": {"in": 9012, "out": 9013},
}
WORLD_HEADING_DEG = 0                 # World heading offset (degrees)
WORLD_SURFACE_MODEL = "EARTH_WGS84"  # World coordinate model

# ==============================================================================
# MONOCULAR DISTANCE ESTIMATION PARAMETERS
# ==============================================================================
TARGET_REAL_WIDTH_M = 0.45            # Estimated human shoulder width (m)
TARGET_REAL_HEIGHT_M = 1.70           # Estimated human height (m)
MONOCULAR_DISTANCE_MAX_M = 100.0      # Max distance limit (m)
MONOCULAR_DISTANCE_MIN_M = 1.0        # Min distance limit (m)

# ==============================================================================
# VISUAL TRACKING PARAMETERS
# ==============================================================================
TRACKING_TARGET_OFFSET_Y_RATIO = 0.75      # Target point Y offset ratio (0.75 = Chest/Shoulder)
TRACKING_SCREEN_OFFSET_Y_RATIO = 0.0625    # Screen center Y offset ratio (1/16)
TRACKING_LATERAL_CORRECTION_GAIN = 1.5     # Lateral correction gain
TRACKING_VERTICAL_FALLBACK_GAIN = 0.008    # Vertical gain fallback (m/s per pixel)

# ==============================================================================
# FRAME BUFFERING PARAMETERS
# ==============================================================================
FRAME_BUFFER_TIMEOUT_FRAMES = 20      # Frame buffer timeout (frames) ~1s @20fps

# ==============================================================================
# GEOGRAPHIC & NAV CONSTANTS
# ==============================================================================
HOME_LATITUDE = -35.3627754
HOME_LONGITUDE = 149.1658777
METERS_PER_DEGREE_LAT = 111320.0

# Ray-Ground Intersection Thresholds
RGI_RAY_Z_MIN = 0.05                    # Minimum ray Z component for valid intersection
RGI_MAX_HORIZ_DIST_M = 500.0            # Maximum horizontal distance for valid target (meters)
RGI_SIN_EPS_MIN = 0.01                  # Minimum sin(depression angle) to avoid singularity
RGI_COT_EPS_CLAMP = 100.0               # Maximum cot(eps) for radial error calculation
RGI_MIN_SIGMA_CROSS_M = 1.0             # Minimum cross-range uncertainty (meters)
RGI_MIN_SIGMA_RADIAL_M = 2.0            # Minimum radial uncertainty (meters)
RGI_SHALLOW_VARIANCE_MULT = 1000.0      # Variance multiplier for shallow rays
RGI_Z_STD_M = 0.5                       # Z-axis standard deviation (meters)

# Kalman Filter Thresholds
KALMAN_MAHALANOBIS_THRESHOLD = 9.21     # Chi-square 99% threshold for 2 DOF outlier rejection
KALMAN_MIN_DT = 0.05                    # Minimum time step for prediction (seconds)

# IBVS Guidance Limits
IBVS_YAW_RATE_MAX = 1.0                 # Maximum yaw rate (rad/s) ±1 rad/s

# Attack Timeout
ATTACK_TIMEOUT_S = 30.0                 # Attack phase timeout before abort (seconds)

# Target Fusion Thresholds
MAHALANOBIS_5SIGMA_SQ = 25.0            # 5σ squared for outlier rejection in fusion
MAHALANOBIS_5SIGMA = 5.0                # 5σ threshold for fusion acceptance

# IBVS Pitch Limits
IBVS_MAX_PITCH_TAN = 0.27               # ~15 degrees max pitch tangent (terminal dive)
IBVS_APPROACH_PITCH_TAN = 0.18          # ~10 degrees approach pitch tangent
IBVS_STARTUP_PITCH_TAN = 0.087          # ~5 degrees startup pitch (very gentle initial approach)

# IBVS Speed Ramp-up (Bee-like smooth attack)
IBVS_STARTUP_VX_MIN = 1.5               # Very slow initial speed (m/s)
IBVS_RAMP_UP_TIME_S = 3.0               # Time to reach full speed (seconds)
IBVS_RAMP_UP_COVERAGE = 20.0            # Coverage % to start ramping up speed

# Detection Thresholds
IOU_THRESHOLD = 0.60                    # IoU threshold for track matching

# ==============================================================================
# SENSOR NOISE MODEL (For Covariance Scaling)
# ==============================================================================
# These values scale observation uncertainty with distance.
# Critical for fusing distant vs near measurements correctly.
DRONE_YAW_STD_DEV_DEG = 5.0           # Compass/Yaw uncertainty (increased to cover 25m drift)
DRONE_PITCH_STD_DEV_DEG = 2.0         # Pitch/Attitude uncertainty
DRONE_PIXEL_NOISE_STD_DEV = 5.0       # Pixel detection noise (pixels)


# ==============================================================================
# ATTACK & MANEUVER SETTINGS
# ==============================================================================
STUCK_MANEUVER_SCALE_BACK = 0.4       # Backup speed scale for stuck maneuver
STUCK_MANEUVER_SCALE_FWD = 1.1        # Forward speed scale for stuck maneuver
STUCK_MANEUVER_DURATION = 0.4         # Duration of stuck maneuver back/fwd pulse (s)
TIMEOUT_PENALTY_SCALE = 0.5           # Timeout increase factor per retry

# ==============================================================================
# VISION FILTERING SETTINGS
# ==============================================================================
EDGE_MARGIN_RATIO = 0.10              # 10% margin from edges to ignore detections (Lens distortion)
MIN_BOX_DIM = 10                      # Minimum bbox width/height to consider valid (pixels)
TRACKER_FRAME_RATE = 30               # BoT-SORT assumed frame rate

# Debug UI Colors & Font
DEBUG_FONT_SCALE = 0.6
DEBUG_FONT_THICKNESS = 2
COLOR_PRIMARY = (0, 0, 255)           # Red (Locked - BGR)
COLOR_SECONDARY = (255, 0, 0)         # Blue (Candidate/Observer - BGR)

# ==============================================================================
# APP SERVER & STATUS MESSAGES
# ==============================================================================
APP_HOST = '0.0.0.0'
APP_PORT = 5000
CONNECTION_STATUS_CONNECTED = "connected"
CONNECTION_STATUS_NOT_CONNECTED = "not_connected"

MISSION_STATUS_MESSAGES = {
    "NOT_CONNECTED": "Waiting for connection...",
    "CONNECTING": "Connecting to drone...",
    "CONNECTION_ERROR": "Connection error. Please retry.",
    "RETRYING": "Connection failed. Retrying... (Attempt {current}/{total})",
    "ALL_RETRIES_FAILED": "All connection attempts failed.",
    "CONNECTED": "Drone connected. Ready for mission.",
    "AREA_SET": "Observation area set successfully. ({rows}x{cols} grid)",
    "STARTING": "Starting mission...",
    "INVALID_TYPE": "Invalid mission type: {mission_type}",
    "ALREADY_ACTIVE": "Mission already active.",
    "NO_AREA": "Please set an observation area first.",
    "STOPPED": "Mission stopped. Drone returning to launch (RTL).",
    "NO_ACTIVE": "No active mission.",
    "TAKING_OFF": "Taking off. Target Altitude: {altitude} m.",
    "ARMING": "Arming motors...",
    "TAKEOFF_SUCCESS": "Takeoff complete. Altitude: {altitude} m.",
    "BEGIN_MISSION_FLIGHT": "Altitude reached, proceeding to observation area.",
    "NAVIGATING_TO_CELL": "Navigating to Cell [{row},{col}].",
    "APPROACHING": "Approaching Cell ({to_row},{to_col}). Dist: {distance:.1f} m, Speed: {speed:.1f} m/s.",
    "VISITING": "Observing Cell [{row},{col}]. Stabilizing at center.",
    "CELL_CONFIRMED": "Cell observation complete: [{row},{col}].",
    "CELL_SKIPPED": "Cell observation skipped: [{row},{col}].",
    "STUCK": "Drone stuck. Executing escape maneuver.",
    "TIMEOUT": "Cell reach timeout. Skipping to next.",
    "CRITICAL_BATTERY": "Critical Battery! RTL initiated.",
    "MISSION_COMPLETE": "Mission complete. Returning to launch.",
    "MISSION_ERROR": "Unexpected mission error: {error}",
    "SWITCHING_DRONE": "Battery low. Switching to next drone...",
    "DRONE_IS_BACK": "Drone returned to home and is charging.",
    "RESUMING": "Resuming mission...",
    "HANDOVER_MISSION_TYPE": "Handing over mission type: {mission_type}",
    "SWITCH_TO_NEXT": "Switching to next drone...",
    "NEXT_DRONE_SELECTED": "New drone ({port}) selected. Continuing mission.",
    "MONITOR_STARTED": "[DroneSwitcher] Monitor started.",
    "MONITOR_STOPPED": "[DroneSwitcher] Monitor stopped.",
    "RTL_STARTED": "[DroneSwitcher] Active drone switched to RTL.",
    "MONITOR_ERROR": "[DroneSwitcher] Error during monitoring: {error}",
    "RESUMING_FROM_POINT": "Resuming from point: {point}",
    "NO_LAST_POINT": "No last point found, restarting.",
    "HUMAN_DETECTED": "Human detected! Engaged Tracking & Collision Mode.",
    "TRACKING_HUMAN": "Tracking Human. Screen Coverage: {coverage:.1f}%",
    "COLLISION_DETECTED": "COLLISION! Screen Coverage: {coverage:.1f}%",
    "HUMAN_LOST": "Target Lost. Continuing last known trajectory...",
    "ROTATING_360": "Single Cell Scan: Performing 360° rotation.",
    "SCANNING_AREA": "Scanning Area. Cell: [{row},{col}]"
}

# ==============================================================================
# SWARM INTELLIGENCE SETTINGS
# ==============================================================================
SWARM_LOG_PATH = os.path.join(PROJECT_ROOT, "logs", "swarm_log.txt")
SWARM_MERGE_DISTANCE_M = 12.0       # Merge distance for close targets (m) - reduced to prevent false merges
SWARM_FUSION_HARD_CAP_M = 15.0      # Absolute max distance for merge (prevents 28m+ wrong merges)
SWARM_FUSION_MAX_SIGMA_M = 0.0      # Max sigma for covariance quality gating (0=disabled)
SWARM_LOCAL_ID_INDEX_MAX_JUMP_M = 60.0  # Max GPS jump for local ID index matching (meters)
SWARM_LOCAL_ID_INDEX_TTL_S = 3.0     # Time-to-live for local ID index entries (seconds)
SWARM_COV_REG_EPS = 1e-6            # Covariance regularization epsilon for matrix inversion
SWARM_GEO_VALIDATION_THRESHOLD_M = 15.0 # Max allowed distance between Drone's RGI and Leader's assigned GPS
SWARM_GEO_COOLDOWN_S = 5.0          # Time to prevent re-assigning a mismatched drone to the same target
SWARM_BIAS_DISTANCE_M = 1000.0      # Sticky assignment bias penalty for protected targets

# Swarm Edge Filter & Visual Configurations
SWARM_EDGE_FILTER_MARGIN_RATIO = 0.15      # X/Y margin ratio for edge-of-FOV penalization
SWARM_EDGE_FILTER_AREA_RATIO = 0.05        # Max area ratio for "small" distant targets to be penalized
SWARM_EDGE_FILTER_FRACTIONAL_CREDIT = 0.34 # Credit earned per max edge frame (3 frames = 1 point)
SWARM_ALIAS_TTL_S = 1.0              # Time-To-Live for visual map aliasing of immutable close targets

# Merge Abort Spam Throttle Configurations
SWARM_MERGE_BACKOFF_INIT_S = 0.2     # Initial backoff timeout for merge spam throttle (reduced for faster merges)
SWARM_MERGE_BACKOFF_MAX_S = 2.0      # Maximum backoff timeout for merge spam throttle (reduced from 10s)

# --- ENHANCED TARGET STABILITY & LOAD BALANCING ---
ENABLE_TARGET_COLLAPSE = True       # Group alias targets bound to same drone + local ID
ENABLE_CAPACITY_GUARD = True        # Restrict drones to 1 ACTIVE target for fair distribution
ENABLE_ATTACK_IMMUTABILITY = True   # Protect targets from reassignment/merge during active attack

SWARM_ATTACK_MIN_CONFIDENCE = 0.25  # Min YOLO confidence for attack approval

# Handoff Policy - STABLE ownership
HANDOFF_CONSECUTIVE_FRAMES = 30     # 15 → 30: Daha fazla frame gerekli
HANDOFF_QUALITY_RATIO = 1.5         # 1.3 → 1.5: Candidate %50 daha kaliteli olmalı
HANDOFF_COOLDOWN_S = 10.0           # 3 → 10 saniye cooldown

# Proactive Assignment Loop
ASSIGNMENT_LOOP_INTERVAL_S = 2.0    # Assignment loop interval (seconds)

# Target Lifecycle
TARGET_COAST_TIME_S = 8.0     # Keep target alive for this duration if signal lost (Coasting)
TARGET_PRUNE_TIME_S = 20.0    # Force delete target after this duration if not updated
SWARM_GHOST_PRUNE_TIME_S = 5.0 # Prune PENDING ghosts faster
SWARM_ZOMBIE_SWITCH_TIME_S = 2.0 # Time to consider an assigned target "lost" and switch
SWARM_IMMUTABLE_TIMEOUT_S = 10.0  # Time to blindly follow invisible locked target before aborting
SWARM_ALIGNMENT_TOLERANCE_M = 5.0 # Tolerance for leader verification and cross-checks

# FUSION CONSTANTS
FUSION_MATCH_THRESHOLD_M = 8.0
KALMAN_PROCESS_NOISE_SCALE = 0.01

# ==============================================================================
# GROUP CLUSTERING (DBSCAN - GPS/ENU meter-space)
# ==============================================================================
GROUP_CLUSTER_EPS_M = 5.0             # Clustering radius (meters)
GROUP_CLUSTER_MIN_SAMPLES = 2          # Min persons per group (2+)
GROUP_PERSISTENCE_SEC = 5.0            # Group memory - lost member retention (s)
GROUP_MERGE_HYSTERESIS = 2             # Merge hysteresis (frame count)
GROUP_SPLIT_HYSTERESIS = 2             # Split hysteresis (frame count)
GROUP_BBOX_SCALE_RATIO_MAX = 4.0       # Max bbox size ratio diff (near/far filter)
GROUP_VISUAL_COLOR_BGR = (0, 200, 255) # Camera overlay group rect color (BGR, yellow-orange)

# ==============================================================================
# TRACK LIFECYCLE (TIME-BASED)
# ==============================================================================
# Derived from swarm_log.txt statistics:
#   - Detection cadence: ~17ms mean, 62ms P95
#   - Ghost burst lifespan: 0.1–0.6s
#   - Max real-target observation gap: 0.31s
#   - Full 360° scan cycle: ~8s
TRACK_CONFIRMATION_TIME_SEC = 0.3     # Time to promote TENTATIVE → CONFIRMED
TRACK_MIN_OBSERVATIONS = 3            # Minimum detections before confirmation (safety floor)
TRACK_MIN_OBSERVATIONS_SEARCH = 2     # Minimum detections in SEARCH mode (faster confirmation)
TRACK_TENTATIVE_TIMEOUT_SEC = 2.5     # TENTATIVE auto-delete if no re-observation (increased to survive full 360 scan)
TRACK_LOST_TIMEOUT_SEC = 2.0          # CONFIRMED → LOST timeout (tolerates occlusion)
TRACK_DELETE_TIMEOUT_SEC = 10.0       # LOST → DELETE timeout (covers full scan cycle)

# ==============================================================================
# ASSOCIATION PARAMETERS (SPATIAL-FIRST)
# ==============================================================================
ASSOCIATION_GATING_THRESHOLD = 3.0    # Mahalanobis sigma threshold (3-sigma rule)
ENABLE_SPATIAL_FALLBACK = True        # Spatial-first association (tracker ID is soft boost only)
TRACKER_ID_ALPHA = 0.15               # Multiplicative ID-match ranking boost (0=no boost, max 0.5)

# ==============================================================================
# DUPLICATE SUPPRESSION
# ==============================================================================
DUPLICATE_MERGE_DISTANCE_M = 12.0    # Base distance to auto-merge CONFIRMED targets (meters)
DUPLICATE_VELOCITY_SIMILARITY_MPS = 3.0  # Max velocity diff for merge eligibility (m/s) - relaxed for noisy tracking

# Dynamic Merge Distance Parameters
MERGE_DISTANCE_BASE_M = 8.0            # Minimum merge distance (meters)
MERGE_DISTANCE_MAX_M = 20.0            # Maximum merge distance (meters)
MERGE_DISTANCE_SIGMA_SCALE = 2.0       # Merge distance = base + avg_sigma * scale

# Leader Failsafe
LEADER_TIMEOUT_S = 10.0             # Timeout before switching to autonomous mode (seconds)

# ==============================================================================
# IBVS (Image-Based Visual Servoing) KAMIKAZE GUIDANCE PARAMETERS
# ==============================================================================
# Yaw Rate Control
IBVS_KP_YAW = 1.5                   # Yaw rate gain (rad/s per normalized error)

# Dive Control - Balanced for stable approach + aggressive terminal
IBVS_KP_DIVE = 1.2                  # Dive speed gain (m/s per normalized error)
IBVS_SHALLOW_DIVE_VZ_MAX = 1.5      # Max vz during shallow approach phase (m/s)
IBVS_STEEP_DIVE_COVERAGE = 45       # Screen coverage % to switch from shallow to steep dive

# Forward Speed Control - Fast approach
IBVS_VX_MAX = 8.0                   # Max forward speed (m/s)
IBVS_VX_MIN = 3.0                   # Min forward speed (m/s)

# Vertical Speed Limits - Controlled by vx/vz ratio in code
IBVS_VZ_MAX = 3.0                   # Max dive speed (m/s) - limited by vx*ratio anyway
IBVS_VZ_MIN = 0.0                   # Min vertical speed (No climbing allowed)

# Safety Floor
IBVS_ALTITUDE_FLOOR_M = 2.0         # Min altitude during dive (m) — prevents ground collision

# Terminal Phase - AGGRESSIVE for final strike
IBVS_TERMINAL_COVERAGE = 55         # Terminal phase screen coverage (%)
IBVS_STEEP_DIVE_VZ_MAX = 4.0        # Max vz in terminal/steep dive phase (m/s)
IBVS_TERMINAL_MAX_PITCH_TAN = 0.27   # Max pitch in terminal phase (~15 deg) - controlled dive

# Ghost Mode
IBVS_GHOST_TIMEOUT = 0.5            # Ghost mode timeout (seconds)

# IBVS Filter Coefficients (Low-pass filter alpha values)
IBVS_YAW_FILTER_ALPHA = 0.2        # Yaw filter smoothing (lower = smoother)
IBVS_VZ_FILTER_ALPHA = 0.15        # Vertical speed filter smoothing
IBVS_VX_FILTER_ALPHA = 0.25        # Forward speed filter smoothing
IBVS_PITCH_COMPENSATION_FACTOR = 0.4  # Drone pitch compensation factor
IBVS_VELOCITY_SMOOTHER_DT = 0.02   # Default dt for velocity smoother (seconds)

# ==============================================================================
# ATTACK PROTOCOL CONFIGURATION (7-Step Handshake)
# ==============================================================================
ATTACK_ECHO_TIMEOUT_S = 5.0              # Max wait for leader confirmation after echo
ATTACK_CENTER_TIMEOUT_S = 10.0            # Max time to center target on camera
ATTACK_CENTER_ERROR_THRESHOLD = 0.05      # Normalized pixel error to consider centered
ATTACK_CENTER_CONFIRM_FRAMES = 10         # Consecutive centered frames to confirm
ATTACK_STALE_ENGAGED_TIMEOUT_S = 15.0     # Force-release ENGAGED target with no observations

# ==============================================================================
# CENTERING STABILIZATION (Yaw control during centering phase)
# ==============================================================================
CENTERING_YAW_KP = 0.4                    # Yaw proportional gain (reduced for stability)
CENTERING_YAW_RATE_MAX = 0.15             # Max yaw rate during centering (rad/s) ~8.6°/s
CENTERING_YAW_DEADZONE = 0.02             # Normalized deadzone to prevent oscillation
CENTERING_SETTLE_FRAMES = 3                # Extra frames to settle after centered
CENTERING_YAW_FILTER_ALPHA = 0.15         # Low-pass filter for yaw smoothing

# ==============================================================================
# IMPACT DETECTION (Collision/Engaged confirmation)
# ==============================================================================
IMPACT_COVERAGE_THRESHOLD = 40.0          # Screen coverage % to confirm impact (reduced for distant targets)
IMPACT_ALTITUDE_MIN_M = 0.5               # Min altitude to confirm ground impact
IMPACT_CONFIRM_FRAMES = 3                 # Consecutive frames to confirm impact
IMPACT_VELOCITY_THRESHOLD = 0.3           # Below this velocity, assume impact
IMPACT_ALTITUDE_PROGRESSIVE = True        # Use progressive altitude thresholds
IMPACT_ALTITUDE_HIGH_M = 3.0              # High altitude - use coverage only
IMPACT_ALTITUDE_LOW_M = 1.5               # Low altitude - reduce coverage requirement

# ==============================================================================
# PHASE TRANSITION BUFFER (Stabilization between phases)
# ==============================================================================
PHASE_TRANSITION_HOLD_S = 0.3             # Hold time between phase transitions
PHASE_TRANSITION_VELOCITY_ZERO = True     # Send zero velocity during transition
CENTERING_TO_DIVING_HOLD_S = 0.5          # Extra hold before diving


# ==============================================================================
# BoT-SORT CONFIGURATION
# ==============================================================================
class BoTSORTConfig:
    """BoT-SORT tracker configuration. All model paths config-driven for field changes."""
    def __init__(self):
        self.project_root = PROJECT_ROOT
        self.tracker_root = os.path.join(self.project_root, "modules", "vision", "tracker")
        
        # Model Weights — change here for rapid model swap
        self.weights = os.path.join(self.tracker_root, "weights", "yolov12n.pt")
        
        # Inference Args
        self.device = INFERENCE_DEVICE
        self.img_size = 640
        self.conf_thres = 0.05
        self.iou_thres = 0.7
        self.classes = [0]           # Person class only
        self.agnostic_nms = False
        self.augment = False
        self.nosave = True
        self.trace = False
        self.mot20 = False
        
        # Tracking Args
        self.track_high_thresh = 0.2
        self.track_low_thresh = 0.05
        self.new_track_thresh = 0.25   # Reverted from 0.35
        self.track_buffer = 90         # Reverted from 120
        self.match_thresh = 0.7        # Reverted from 0.5
        self.min_box_area = 10
        
        # CMC (Camera Motion Compensation)
        self.cmc_method = "sparseOptFlow"
        
        # IoU Gating
        self.proximity_thresh = 0.5
        
        # Extra
        self.ablation = False
        self.name = 'BoT-SORT'