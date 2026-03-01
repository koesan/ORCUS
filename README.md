<div align="center">

# ORCUS v2.0
### Autonomous Swarm Kamikaze Drone System

<img src="image/logo.png" alt="ORCUS Logo" width="400"/>

[![Python](https://img.shields.io/badge/Python-3.8+-3776AB.svg?logo=python&logoColor=white)](https://www.python.org/)
[![ROS](https://img.shields.io/badge/ROS-Melodic/Noetic-22314E.svg?logo=ros&logoColor=white)](https://www.ros.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-orange.svg)](http://gazebosim.org/)
[![YOLO](https://img.shields.io/badge/YOLOv12-Detection-00FFFF.svg)](https://github.com/ultralytics/ultralytics)
[![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8.svg?logo=opencv&logoColor=white)](https://opencv.org/)
[![Flask](https://img.shields.io/badge/Flask-Web-000000.svg?logo=flask&logoColor=white)](https://flask.palletsprojects.com/)
[![DroneKit](https://img.shields.io/badge/DroneKit-Python-blue.svg)](https://dronekit.io/)
[![Docker](https://img.shields.io/badge/Docker-Support-2496ED.svg?logo=docker&logoColor=white)](https://www.docker.com/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

[![Demo Video](https://img.shields.io/badge/Demo%20Video-▶️-FF0000?style=for-the-badge&logo=youtube&logoColor=white)](https://github.com/koesan/ORCUS/blob/main/image/demo.mp4)

🎥 **Demo Video**  
[![Video Thumbnail](https://img.youtube.com/vi/vFxlOCKCGh8/maxresdefault.jpg)](https://youtu.be/vFxlOCKCGh8)

</div>

---

🇬🇧[English](#-english-documentation) | 🇹🇷[Türkçe](#-türkçe-dokümantasyon)

---

# 📚 English Documentation

## 📖 About the Project

**ORCUS v2.0** is an autonomous **Swarm Kamikaze Drone System** designed for multi-drone coordination in surveillance and target engagement scenarios. The system implements a **leader-follower architecture** where a coordinator drone manages target assignments and orchestrates attack missions across the swarm.

Built with a modular architecture, ORCUS provides real-time human detection and tracking, GPS-based position estimation from monocular camera feeds, and autonomous attack guidance. The web-based control hub enables operators to monitor all drones, define mission areas, and track target assignments through a unified interface.

**Key System Capabilities:**
- Multi-drone swarm coordination with centralized command
- Real-time target detection and persistent tracking
- GPS position estimation from visual data
- Autonomous attack trajectory guidance
- Web-based monitoring and control interface

### What's New in v2.0?

| Feature | v1.0 | v2.0 |
|---------|------|------|
| **Target Tracking** | Simple tracker, first-seen target locked | Multi-target tracking with BoT-SORT |
| **Attack Strategy** | Lock first detected target, immediate attack | Optimal assignment via Hungarian algorithm |
| **Position Estimation** | None | Ray-Ground Intersection algorithm (geo_math.py) |
| **Multi-Drone Fusion** | None (independent drones) | Kalman Filter observation fusion |
| **Swarm Coordination** | Grid partitioning, independent operation | Leader-follower with ownership states |
| **Target States** | DETECTED → ATTACKING | Full state machine (FREE, OWNED, LOCKED, ATTACKING) |
| **Assignment Logic** | First-come-first-served | Cost matrix scoring (visibility, heading, distance, covariance) |
| **Measurement Quality** | None | Covariance-based uncertainty tracking |

---

## 🎯 Key Features

- **Leader-Follower Architecture**: One drone coordinates the swarm, managing target assignments and orchestrating attack missions
- **YOLOv12 + BoT-SORT**: Real-time human detection with persistent multi-object tracking
- **Ray-Ground Intersection**: Monocular camera position estimation with covariance uncertainty quantification (geo_math.py)
- **Kalman Filter Fusion**: Multi-drone observation fusion for improved target position accuracy
- **Hungarian Assignment**: Optimal drone-target pairing with cost matrix scoring (visibility, heading, distance, covariance)
- **Target State Machine**: Full lifecycle management (FREE → OWNED → LOCKED → ATTACKING)
- **IBVS Guidance**: Image-Based Visual Servoing with PID control for precision attack trajectories
- **Web Control Hub**: Real-time monitoring and mission control interface

---

## 🛠️ Technology Stack

| Component | Technology |
|-----------|------------|
| Flight Controller | ArduPilot SITL |
| Simulation | Gazebo + ROS |
| Object Detection | YOLOv12 (Ultralytics) |
| Object Tracking | BoT-SORT |
| State Estimation | Kalman Filter |
| Assignment | Hungarian Algorithm |
| Communication | DroneKit, MAVLink |
| Backend | Flask |
| Frontend | MJPEG Streaming |

---

## � Module Dependencies

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           MODULE ARCHITECTURE                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────┐                                                            │
│  │   app.py    │  ◀── Entry Point (Flask Web Server)                       │
│  └──────┬──────┘                                                            │
│         │                                                                    │
│         ▼                                                                    │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                        CORE LAYER                                    │    │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │    │
│  │  │fleet_manager │──│   logger     │──│  geo_math    │              │    │
│  │  └──────┬───────┘  └──────────────┘  └──────┬───────┘              │    │
│  │         │                              ▲     │                       │    │
│  │         │                              │     │                       │    │
│  │  ┌──────┴───────┐  ┌──────────────┐  │  ┌──┴───────┐              │    │
│  │  │kalman_filter │  │pid_controller│  │  │  logger  │              │    │
│  │  └──────────────┘  └──────────────┘  │  └──────────┘              │    │
│  └───────────────────────────────────────┼─────────────────────────────┘    │
│                                         │                                    │
│  ┌───────────────────────────────────────┼─────────────────────────────┐    │
│  │                        VISION LAYER   │                              │    │
│  │  ┌──────────────┐  ┌──────────────┐  │  ┌──────────────┐           │    │
│  │  │  detector    │──│group_tracker │──┼──│camera_handler│           │    │
│  │  └──────┬───────┘  └──────────────┘  │  └──────────────┘           │    │
│  │         │                           │                               │    │
│  │  ┌──────┴───────┐                  │                               │    │
│  │  │   tracker/   │                  │                               │    │
│  │  │  (BoT-SORT)  │                  │                               │    │
│  │  └──────────────┘                  │                               │    │
│  └─────────────────────────────────────┼─────────────────────────────────┘    │
│                                         │                                    │
│  ┌───────────────────────────────────────┼─────────────────────────────┐    │
│  │                        SWARM LAYER    │                              │    │
│  │  ┌──────────────┐  ┌──────────────┐  │                               │    │
│  │  │   swarm_     │──│ target_fusion│──┘                              │    │
│  │  │ coordinator  │  └──────────────┘                                  │    │
│  │  └──────────────┘                                                    │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                         ▲                                    │
│                                         │                                    │
│  ┌───────────────────────────────────────┼─────────────────────────────┐    │
│  │                       MISSION LAYER   │                              │    │
│  │  ┌──────────────┐  ┌──────────────┐  │  ┌──────────────┐           │    │
│  │  │   mission_   │──│  tracking_   │──┼──│    ibvs_     │           │    │
│  │  │ controller   │  │ controller   │  │  │  guidance    │           │    │
│  │  └──────┬───────┘  └──────┬───────┘  │  └──────────────┘           │    │
│  │         │                 │          │                               │    │
│  │  ┌──────┴───────┐        │          │                               │    │
│  │  │   flight_    │────────┘          │                               │    │
│  │  │ controller   │                   │                               │    │
│  │  └──────────────┘                   │                               │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  LEGEND:                                                                     │
│  ──▶ Depends on / Imports from                                               │
│  ──▶ Data flow / Communication                                              │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Module Responsibilities

| Module | Layer | Responsibility |
|--------|-------|----------------|
| `fleet_manager` | Core | Drone connection management, video stream coordination |
| `geo_math` | Core | GPS coordinate estimation from camera data |
| `kalman_filter` | Core | State estimation and sensor fusion |
| `pid_controller` | Core | Control loops for guidance |
| `logger` | Core | Centralized logging system |
| `detector` | Vision | YOLOv12 detection and BoT-SORT tracking |
| `camera_handler` | Vision | Camera stream acquisition and processing |
| `group_tracker` | Vision | Group clustering for multiple targets |
| `swarm_coordinator` | Swarm | Leader-follower logic, target assignment |
| `target_fusion` | Swarm | Multi-drone observation fusion |
| `mission_controller` | Mission | Mission state machine, attack logic, area scanning |
| `tracking_controller` | Mission | Target tracking and engagement |
| `flight_controller` | Mission | MAVLink commands, drone movement |
| `ibvs_guidance` | Mission | Image-Based Visual Servoing for attacks |

---

## � Installation & Setup

### Prerequisites
- Ubuntu 20.04
- Python 3.8+
- ROS Noetic

### Step 1: Setup Simulation Environment

Follow the complete setup instructions in our Docker-based simulation repository:

🔗 **[ArduGazeboSim-Docker Repository](https://github.com/koesan/ArduGazeboSim-Docker)**

This includes:
- Docker installation
- ROS package setup
- ArduPilot SITL installation
- Gazebo simulation environment

### Step 2: Clone ORCUS Project

```bash
cd ArduGazeboSim
git clone https://github.com/koesan/ORCUS.git
```

### Step 3: Configure Drone Models & World

```bash
# Copy drone models with cameras
cp -r ORCUS/simulator/drone/drone1/* catkin_ws/src/iq_sim/models/drone1/
cp -r ORCUS/simulator/drone/drone2/* catkin_ws/src/iq_sim/models/drone2/

# Copy world file with human actors
cp ORCUS/simulator/worlds/multi_drone.world catkin_ws/src/iq_sim/worlds/
```

---

## 🎮 Running the System

### Terminal 1: Launch Simulation
```bash
roslaunch iq_sim multi_drone.launch
```

### Terminal 2-3: Connect Drones
```bash
# Terminal 2 - Drone 1
sim_vehicle.py -v ArduCopter -f gazebo-iris -I0

# Terminal 3 - Drone 2
sim_vehicle.py -v ArduCopter -f gazebo-iris -I1
```

### Terminal 4: Start ORCUS Control Hub
```bash
cd ArduGazeboSim/ORCUS
pip3 install -r requirements.txt
python3 app.py
```

### Access Web Interface
```
http://localhost:5000/
```

---

## 📋 System Workflow

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         ORCUS SYSTEM WORKFLOW                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐         │
│  │  START   │───▶│ CONNECT  │───▶│ DEFINE   │───▶│  TAKEOFF │         │
│  │          │    │  DRONES  │    │  AREA    │    │          │         │
│  └──────────┘    └──────────┘    └──────────┘    └──────────┘         │
│                                                        │               │
│                                                        ▼               │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐         │
│  │  ATTACK  │◀───│  LOCK    │◀───│  TRACK   │◀───│  SEARCH  │         │
│  │  MODE    │    │  TARGET  │    │  TARGET  │    │  AREA    │         │
│  └──────────┘    └──────────┘    └──────────┘    └──────────┘         │
│       │                                                                │
│       ▼                                                                │
│  ┌──────────┐                                                         │
│  │ COLLISION│                                                         │
│  │ MISSION  │                                                         │
│  └──────────┘                                                         │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 🎯 Detection & Localization Workflow

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    DETECTION & LOCALIZATION PIPELINE                    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────┐                                                       │
│  │  RGB CAMERA │ ◀── Drone onboard camera (30 FPS)                    │
│  └──────┬──────┘                                                       │
│         │                                                               │
│         ▼                                                               │
│  ┌─────────────┐     ┌─────────────┐                                   │
│  │   YOLOv12   │────▶│ BOUNDING    │  Detection: Class, Confidence    │
│  │  DETECTION  │     │    BOX      │  (x, y, w, h, conf, class)        │
│  └─────────────┘     └──────┬──────┘                                   │
│                             │                                          │
│                             ▼                                          │
│  ┌─────────────┐     ┌─────────────┐                                   │
│  │  BoT-SORT   │────▶│   TRACK     │  Persistent ID assignment         │
│  │  TRACKING   │     │    ID       │  Track state management           │
│  └─────────────┘     └──────┬──────┘                                   │
│                             │                                          │
│                             ▼                                          │
│  ┌─────────────────────────────────────────────────────┐              │
│  │         RAY-GROUND INTERSECTION ALGORITHM            │              │
│  ├─────────────────────────────────────────────────────┤              │
│  │  1. Camera Intrinsics (fx, fy, cx, cy)              │              │
│  │  2. Drone State (lat, lon, alt, heading, pitch)    │              │
│  │  3. Gimbal Angles (roll, pitch, yaw)               │              │
│  │  4. Bounding Box Center → Pixel Coordinates        │              │
│  │  5. Ray Casting → Ground Intersection              │              │
│  │  6. Coordinate Transform → GPS (lat, lon)          │              │
│  └──────────────────────────┬──────────────────────────┘              │
│                             │                                          │
│                             ▼                                          │
│  ┌─────────────┐     ┌─────────────┐                                   │
│  │   TARGET    │────▶│  COVARIANCE │  Position uncertainty            │
│  │  POSITION   │     │  MATRIX     │  (σ_lat, σ_lon)                   │
│  │ (lat, lon)  │     │             │                                   │
│  └─────────────┘     └─────────────┘                                   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## ⚔️ Attack Mission Workflow

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      ATTACK MISSION PIPELINE                            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  TARGET STATE MACHINE:                                                  │
│                                                                         │
│  ┌─────────┐   detection    ┌─────────┐   confirm     ┌─────────┐     │
│  │  FREE   │ ──────────────▶│  OWNED  │ ─────────────▶│ LOCKED  │     │
│  └─────────┘                └─────────┘               └─────────┘     │
│       ▲                          │                         │          │
│       │                          │                         │          │
│       │                     lost │                    attack │          │
│       │                          ▼                         │          │
│       │                    ┌─────────┐                      ▼          │
│       └────────────────────│  LOST   │             ┌──────────┐      │
│                            └─────────┘             │ ATTACKING │      │
│                                                    └──────────┘      │
│                                                         │              │
│                                                         ▼              │
│                                                  ┌──────────┐         │
│                                                  │ COLLISION│         │
│                                                  └──────────┘         │
│                                                                         │
│  ASSIGNMENT FLOW:                                                       │
│                                                                         │
│  ┌────────────┐    ┌────────────┐    ┌────────────┐    ┌────────────┐  │
│  │   DETECT   │───▶│    FUSE    │───▶│   ASSIGN   │───▶│   LOCK     │  │
│  │   TARGET   │    │ OBSERVATIONS│   │   DRONE    │    │   TARGET   │  │
│  └────────────┘    └────────────┘    └────────────┘    └────────────┘  │
│        │                 │                 │                 │          │
│        ▼                 ▼                 ▼                 ▼          │
│  YOLO+BoT-SORT    Kalman Filter    Hungarian Alg.    State Machine    │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 🧠 Algorithms

### 1. Ray-Ground Intersection Geolocation

Position estimation from monocular RGB camera using ray-ground intersection:

```
Input: Bounding box center (pixel_x, pixel_y), Drone state (lat, lon, alt, roll, pitch, yaw)
Output: GPS coordinates (lat, lon) with 3x3 covariance matrix

Algorithm (geo_math.py):
1. Pixel → Normalized Camera: (x_norm, y_norm) = ((px - cx)/fx, (py - cy)/fy)
2. Camera → Body Frame: Axis transformation for drone body alignment
3. Apply Camera Pitch Offset: Compensate for gimbal mounting angle
4. Body → NED Frame: Transform using RPY rotation matrix (ZYX sequence)
5. Ray-Ground Intersection: Solve for t where ray meets z=0 plane
   - intersection = p0 + t * ray_ned
   - Clamp shallow rays (ray_z < threshold) for numerical stability
6. NED → GPS: Convert (d_north, d_east) to (lat, lon) offsets

Covariance Estimation (Jacobian Propagation):
- σ_azimuth = √(σ_yaw² + σ_pixel²) → σ_cross = slant_range * σ_azimuth
- σ_elevation = √(σ_pitch² + σ_pixel²) → σ_radial = slant_range * cot(ε) * σ_elevation
- Build 3x3 covariance in NED frame, transform to GPS bearing
```

### 2. BoT-SORT Tracking

Multi-object tracking algorithm:

```
Features:
- Kalman Filter for motion prediction
- IoU-based track association
- Track management (new, confirmed, lost, deleted)
- Camera motion compensation (CMC)
```

### 3. Target Fusion (Kalman Filter)

Multi-drone observation fusion:

```
State Vector: [lat, lon, v_lat, v_lon]
Process Model: Constant velocity
Measurement Model: GPS + covariance

Fusion Steps:
1. Predict state using process model
2. Receive observations from multiple drones
3. Mahalanobis distance gating
4. Update state with valid observations
5. Output: Fused position with reduced covariance
```

### 4. Hungarian Assignment

Optimal drone-target assignment:

```
Cost Matrix Factors:
- Distance (base cost)
- Visibility (drone sees target: bonus)
- Heading (target in front: bonus, behind: penalty)
- Motion (moving drone: penalty)
- Distribution (close to assigned targets: penalty, far: bonus)
- Covariance (high uncertainty: penalty, low: bonus)
- Stickiness (current assignment: bonus)

Algorithm: linear_sum_assignment (scipy.optimize)
```

---

## 🏗️ Project Structure

```
ORCUS/
├── app.py                              # Main Flask application
├── config.py                           # System configuration
├── requirements.txt                    # Python dependencies
│
├── modules/
│   ├── core/
│   │   ├── logger.py                   # Logging system
│   │   ├── geo_math.py                 # Ray-ground intersection GPS estimation
│   │   ├── kalman_filter.py            # Kalman Filter implementation
│   │   ├── pid_controller.py           # PID control
│   │   └── fleet_manager.py            # Fleet state management
│   │
│   ├── mission/
│   │   ├── flight_controller.py       # MAVLink flight control
│   │   ├── mission_controller.py      # Mission orchestration
│   │   ├── tracking_controller.py     # Target tracking control
│   │   └── ibvs_guidance.py            # Image-Based Visual Servoing
│   │
│   ├── swarm/
│   │   ├── swarm_coordinator.py        # Swarm leader-follower logic
│   │   └── target_fusion.py            # Kalman Filter fusion
│   │
│   ├── vision/
│   │   ├── detector.py                 # YOLOv12 detection + tracking
│   │   ├── camera_handler.py           # Camera stream handling
│   │   ├── group_tracker.py            # Group tracking logic
│   │   └── tracker/
│   │       ├── bot_sort.py             # BoT-SORT implementation
│   │       ├── mc_bot_sort.py          # Motion-compensated BoT-SORT
│   │       ├── kalman_filter.py        # Tracker Kalman Filter
│   │       ├── matching.py             # Track association
│   │       ├── gmc.py                  # Camera motion compensation
│   │       ├── basetrack.py            # Base track class
│   │       └── weights/
│   │           └── yolov12n.pt         # YOLO model weights
│
├── templates/
│   └── index.html                      # Web interface
│
├── static/
│   ├── css/                            # Stylesheets
│   └── js/                             # JavaScript
│
├── simulator/
│   ├── drone/                          # Gazebo drone models
│   └── worlds/                         # Gazebo world files
│
├── logs/
│   └── swarm_log.txt                   # System logs
│
└── tests/                              # Unit tests
```

---

## 📊 Area Partitioning Algorithm (v1.0 Legacy)

> **Note:** This grid-based partitioning was used in v1.0. In v2.0, the leader-follower architecture provides more flexible coordination.

```
Total Area (16 cells):
┌──┬──┬──┬──┐
│ 1│ 2│ 3│ 4│  → Row 0
├──┼──┼──┼──┤
│ 5│ 6│ 7│ 8│  → Row 1  } Drone 1 (Port 5760)
├──┼──┼──┼──┤  ═══════════════════════
│ 9│10│11│12│  → Row 2
├──┼──┼──┼──┤
│13│14│15│16│  → Row 3  } Drone 2 (Port 5761)
└──┴──┴──┴──┘

Boustrophedon Pattern:
Drone 1: 1→2→3→4, 8←7←6←5
Drone 2: 9→10→11→12, 16←15←14←13
```

---

## ⚙️ Configuration

Key parameters in `config.py`:

```python
# Drone Settings
TAKEOFF_ALTITUDE = 5                    # meters
DRONE_SPEED = 90                        # cm/s

# Collision Mission
COLLISION_FORWARD_SPEED = 2.0           # m/s
COLLISION_SCREEN_THRESHOLD = 0.40       # 40% screen coverage triggers collision
HUMAN_LOST_TIMEOUT = 5.0                # seconds

# AI Detection
YOLO_CONF_THRESHOLD = 0.25              # Detection confidence threshold
YOLO_IOU_THRESHOLD = 0.40               # IoU threshold for NMS

# Tracker Parameters (v2.0) - Instance attributes in BotSortArgs
# new_track_thresh = 0.25               # New track creation threshold
# track_buffer = 90                      # Track retention (frames)
# match_thresh = 0.7                     # Association threshold

# Fusion Parameters
FUSION_MATCH_THRESHOLD_M = 8.0          # Target fusion distance (meters)
KALMAN_PROCESS_NOISE_SCALE = 0.01       # Process noise scale
```

---

## 🔮 Upcoming: v2.1 Roadmap

**Status: Planning Phase**

v2.1 focuses on system optimization, architecture consolidation, and algorithm refinement:

### Core Objectives

| Goal | Description |
|------|-------------|
| **System Optimization** | Reduce parameter complexity, streamline algorithms, improve performance |
| **Architecture Cleanup** | Remove legacy code, resolve algorithm conflicts, consolidate redundant logic |
| **Tracking Stability** | Eliminate ID flip-flop, improve track persistence, enhance assignment consistency |
| **Position Accuracy** | Refine Ray-Ground Intersection, improve covariance estimation |

### Key Improvements

**Algorithm Consolidation:**
- Remove deprecated/legacy algorithms accumulated from incremental development
- Resolve conflicts between overlapping algorithms
- Unify duplicate implementations into single, optimized modules

**Tracking & Assignment:**
- Reduce tracker ID flip-flop through improved association logic
- Stabilize drone-target assignments with refined cost matrix scoring
- Enhance multi-target scenario handling

**Position Estimation:**
- Improve Ray-Ground Intersection accuracy and edge case handling
- Better covariance estimation for uncertain measurements
- More robust fusion under challenging conditions

### Known Issues Being Addressed

- Excessive parameter count causing configuration complexity
- Algorithm conflicts from incremental development history
- Tracker ID instability in multi-target scenarios
- Drone-target assignment flip-flop during concurrent detections
- Performance bottlenecks for larger swarm sizes

---

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

---

## ⚠️ Disclaimer

This project is for **educational and research purposes only**. The developers are not responsible for any misuse of this system. Always comply with local laws and regulations regarding drone operations.

---

# 📚 Türkçe Dokümantasyon

## 📖 Proje Hakkında

**ORCUS v2.0**, gözetleme ve hedef angajman senaryolarında çoklu drone koordinasyonu için tasarlanmış otonom bir **Sürü Kamikaze İHA Sistemi**'dir. Sistem, koordinatör dronun hedef atamalarını yönettiği ve sürü genelinde saldırı görevlerini orkestre ettiği **lider-takipçi mimarisi** uygular.

Modüler bir mimari ile inşa edilen ORCUS, gerçek zamanlı insan tespiti ve takibi, monoküler kamera görüntülerinden GPS tabanlı konum tahmini ve otonom saldırı güdümlemesi sağlar. Web tabanlı kontrol merkezi, operatörlerin tüm droneleri izlemesine, görev alanlarını tanımlamasına ve hedef atamalarını takip etmesine olanak tanır.

**Temel Sistem Yetenekleri:**
- Merkezi komut ile çoklu drone sürü koordinasyonu
- Gerçek zamanlı hedef tespiti ve kalıcı takip
- Görsel verilerden GPS konum tahmini
- Otonom saldırı yörünge güdümlemesi
- Web tabanlı izleme ve kontrol arayüzü

### v2.0'daki Yenilikler

| Özellik | v1.0 | v2.0 |
|---------|------|------|
| **Hedef Takibi** | Basit tracker, ilk görülen hedef kilitlenir | BoT-SORT ile çoklu hedef takibi |
| **Saldırı Stratejisi** | İlk tespit edilen hedefe kilitle, anında saldır | Hungarian algoritması ile optimal atama |
| **Konum Tahmini** | Yok | Işın-Zemin Kesişimi algoritması (geo_math.py) |
| **Çoklu Drone Füzyonu** | Yok (bağımsız dronelar) | Kalman Filtre gözlem füzyonu |
| **Sürü Koordinasyonu** | Izgara bölme, bağımsız çalışma | Sahiplik durumlu lider-takipçi |
| **Hedef Durumları** | TESPİT → SALDIRI | Tam durum makinesi (FREE, OWNED, LOCKED, ATTACKING) |
| **Atama Mantığı** | İlk-gelen-ilk-saldırır | Cost matrix skorlaması (görünürlük, yönelim, mesafe, kovaryans) |
| **Ölçüm Kalitesi** | Yok | Kovaryans tabanlı belirsizlik takibi |

---

## 🎯 Temel Özellikler

- **Lider-Takipçi Mimarisi**: Bir drone sürüyü koordine eder, hedef atamalarını yönetir ve saldırı görevlerini orkestre eder
- **YOLOv12 + BoT-SORT**: Kalıcı çoklu nesne takibi ile gerçek zamanlı insan tespiti
- **Işın-Zemin Kesişimi**: Monoküler kamera konum tahmini ile kovaryans belirsizlik ölçümü (geo_math.py)
- **Kalman Filtre Füzyonu**: Hedef konum doğruluğunu artırmak için çoklu drone gözlem füzyonu
- **Hungarian Ataması**: Cost matrix skorlaması ile optimal drone-hedef eşleştirmesi (görünürlük, yönelim, mesafe, kovaryans)
- **Hedef Durum Makinesi**: Tam yaşam döngüsü yönetimi (FREE → OWNED → LOCKED → ATTACKING)
- **IBVS Güdümlemesi**: Hassas saldırı yörüngeleri için PID kontrollü Görüntü Tabanlı Görsel Servoing
- **Web Kontrol Merkezi**: Gerçek zamanlı izleme ve görev kontrol arayüzü

---

## 🛠️ Teknoloji Yığını

| Bileşen | Teknoloji |
|---------|-----------|
| Uçuş Kontrolcüsü | ArduPilot SITL |
| Simülasyon | Gazebo + ROS |
| Nesne Tespiti | YOLOv12 (Ultralytics) |
| Nesne Takibi | BoT-SORT |
| Durum Tahmini | Kalman Filtre |
| Atama | Hungarian Algoritması |
| İletişim | DroneKit, MAVLink |
| Backend | Flask |
| Frontend | MJPEG Streaming |

---

## 🔗 Modül Bağımlılıkları

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          MODÜL MİMARİSİ                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────┐                                                            │
│  │   app.py    │  ◀── Giriş Noktası (Flask Web Sunucusu)                   │
│  └──────┬──────┘                                                            │
│         │                                                                    │
│         ▼                                                                    │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                        ÇEKİRDEK KATMANI                              │    │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │    │
│  │  │fleet_manager │──│   logger     │──│  geo_math    │              │    │
│  │  └──────┬───────┘  └──────────────┘  └──────┬───────┘              │    │
│  │         │                              ▲     │                       │    │
│  │         │                              │     │                       │    │
│  │  ┌──────┴───────┐  ┌──────────────┐  │  ┌──┴───────┐              │    │
│  │  │kalman_filter │  │pid_controller│  │  │  logger  │              │    │
│  │  └──────────────┘  └──────────────┘  │  └──────────┘              │    │
│  └───────────────────────────────────────┼─────────────────────────────┘    │
│                                         │                                    │
│  ┌───────────────────────────────────────┼─────────────────────────────┐    │
│  │                        GÖRÜNTÜ KATMANI│                              │    │
│  │  ┌──────────────┐  ┌──────────────┐  │  ┌──────────────┐           │    │
│  │  │  detector    │──│group_tracker │──┼──│camera_handler│           │    │
│  │  └──────┬───────┘  └──────────────┘  │  └──────────────┘           │    │
│  │         │                           │                               │    │
│  │  ┌──────┴───────┐                  │                               │    │
│  │  │   tracker/   │                  │                               │    │
│  │  │  (BoT-SORT)  │                  │                               │    │
│  │  └──────────────┘                  │                               │    │
│  └─────────────────────────────────────┼─────────────────────────────────┘    │
│                                         │                                    │
│  ┌───────────────────────────────────────┼─────────────────────────────┐    │
│  │                         SÜRÜ KATMANI  │                              │    │
│  │  ┌──────────────┐  ┌──────────────┐  │                               │    │
│  │  │   swarm_     │──│ target_fusion│──┘                              │    │
│  │  │ coordinator  │  └──────────────┘                                  │    │
│  │  └──────────────┘                                                    │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                         ▲                                    │
│                                         │                                    │
│  ┌───────────────────────────────────────┼─────────────────────────────┐    │
│  │                        GÖREV KATMANI  │                              │    │
│  │  ┌──────────────┐  ┌──────────────┐  │  ┌──────────────┐           │    │
│  │  │   mission_   │──│  tracking_   │──┼──│    ibvs_     │           │    │
│  │  │ controller   │  │ controller   │  │  │  guidance    │           │    │
│  │  └──────┬───────┘  └──────┬───────┘  │  └──────────────┘           │    │
│  │         │                 │          │                               │    │
│  │  ┌──────┴───────┐        │          │                               │    │
│  │  │   flight_    │────────┘          │                               │    │
│  │  │ controller   │                   │                               │    │
│  │  └──────────────┘                   │                               │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
│  AÇIKLAMA:                                                                   │
│  ──▶ Bağımlı olduğu / İçe aktardığı                                         │
│  ──▶ Veri akışı / İletişim                                                  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Modül Sorumlulukları

| Modül | Katman | Sorumluluk |
|-------|--------|------------|
| `fleet_manager` | Çekirdek | Drone bağlantı yönetimi, video akış koordinasyonu |
| `geo_math` | Çekirdek | Kamera verilerinden GPS koordinat tahmini |
| `kalman_filter` | Çekirdek | Durum tahmini ve sensör füzyonu |
| `pid_controller` | Çekirdek | Güdümleme için kontrol döngüleri |
| `logger` | Çekirdek | Merkezi loglama sistemi |
| `detector` | Görüntü | YOLOv12 tespiti ve BoT-SORT takibi |
| `camera_handler` | Görüntü | Kamera akışı edinimi ve işleme |
| `group_tracker` | Görüntü | Çoklu hedefler için grup kümeleme |
| `swarm_coordinator` | Sürü | Lider-takipçi mantığı, hedef ataması |
| `target_fusion` | Sürü | Çoklu drone gözlem füzyonu |
| `mission_controller` | Görev | Görev durum makinesi, saldırı mantığı, alan tarama |
| `tracking_controller` | Görev | Hedef takibi ve angajman |
| `flight_controller` | Görev | MAVLink komutları, drone hareketi |
| `ibvs_guidance` | Görev | Saldırılar için Görüntü Tabanlı Görsel Servoing |

---

## 🚀 Kurulum ve Yapılandırma

### Gereksinimler
- Ubuntu 20.04
- Python 3.8+
- ROS Noetic

### Adım 1: Simülasyon Ortamı Kurulumu

Docker tabanlı simülasyon deposundaki kurulum talimatlarını takip edin:

🔗 **[ArduGazeboSim-Docker Deposu](https://github.com/koesan/ArduGazeboSim-Docker)**

Bu şunları içerir:
- Docker kurulumu
- ROS paket kurulumu
- ArduPilot SITL kurulumu
- Gazebo simülasyon ortamı

### Adım 2: ORCUS Projesini Klonlama

```bash
cd ArduGazeboSim
git clone https://github.com/koesan/ORCUS.git
```

### Adım 3: Drone Modelleri ve Dünya Yapılandırması

```bash
# Kamereli drone modellerini kopyala
cp -r ORCUS/simulator/drone/drone1/* catkin_ws/src/iq_sim/models/drone1/
cp -r ORCUS/simulator/drone/drone2/* catkin_ws/src/iq_sim/models/drone2/

# İnsan aktörlü dünya dosyasını kopyala
cp ORCUS/simulator/worlds/multi_drone.world catkin_ws/src/iq_sim/worlds/
```

---

## 🎮 Sistemi Çalıştırma

### Terminal 1: Simülasyonu Başlat
```bash
roslaunch iq_sim multi_drone.launch
```

### Terminal 2-3: Drone'ları Bağla
```bash
# Terminal 2 - Drone 1
sim_vehicle.py -v ArduCopter -f gazebo-iris -I0

# Terminal 3 - Drone 2
sim_vehicle.py -v ArduCopter -f gazebo-iris -I1
```

### Terminal 4: ORCUS Kontrol Merkezini Başlat
```bash
cd ArduGazeboSim/ORCUS
pip3 install -r requirements.txt
python3 app.py
```

### Web Arayüzüne Eriş
```
http://localhost:5000/
```

---

## 📋 Sistem İş Akışı

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      ORCUS SİSTEM İŞ AKIŞI                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐         │
│  │  BAŞLAT  │───▶│ DRONE    │───▶│  ALAN    │───▶│ KALKIŞ   │         │
│  │          │    │ BAĞLA    │    │ TANIMLA  │    │          │         │
│  └──────────┘    └──────────┘    └──────────┘    └──────────┘         │
│                                                        │               │
│                                                        ▼               │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐         │
│  │ SALDIRI  │◀───│  KİLİT   │◀───│  TAKİP   │◀───│  TARAMA  │         │
│  │  MODU    │    │  HEDEF   │    │  HEDEF   │    │  ALANI   │         │
│  └──────────┘    └──────────┘    └──────────┘    └──────────┘         │
│       │                                                                │
│       ▼                                                                │
│  ┌──────────┐                                                         │
│  │ ÇARPIŞMA │                                                         │
│  │ GÖREVİ   │                                                         │
│  └──────────┘                                                         │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 🎯 Tespit ve Konumlandırma İş Akışı

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    TESPİT VE KONUMLANDIRMA SÜRECİ                      │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────┐                                                       │
│  │  RGB KAMERA │ ◀── Drone onboard kamera (30 FPS)                    │
│  └──────┬──────┘                                                       │
│         │                                                               │
│         ▼                                                               │
│  ┌─────────────┐     ┌─────────────┐                                   │
│  │   YOLOv12   │────▶│ SINIR       │  Tespit: Sınıf, Güven            │
│  │  TESPİT     │     │ KUTUSU      │  (x, y, w, h, conf, class)        │
│  └─────────────┘     └──────┬──────┘                                   │
│                             │                                          │
│                             ▼                                          │
│  ┌─────────────┐     ┌─────────────┐                                   │
│  │  BoT-SORT   │────▶│   TRACK     │  Kalıcı ID ataması                │
│  │  TAKİP      │     │    ID       │  Track durum yönetimi             │
│  └─────────────┘     └──────┬──────┘                                   │
│                             │                                          │
│                             ▼                                          │
│  ┌─────────────────────────────────────────────────────┐              │
│  │        IŞIN-ZEMİN KESİŞİMİ ALGORİTMASI             │              │
│  ├─────────────────────────────────────────────────────┤              │
│  │  1. Kamera İç Parametreler (fx, fy, cx, cy)        │              │
│  │  2. Drone Durumu (lat, lon, alt, heading, pitch)   │              │
│  │  3. Gimbal Açıları (roll, pitch, yaw)              │              │
│  │  4. Sinir Kutusu Merkezi → Piksel Koordinatları    │              │
│  │  5. Işın Atma → Zemin Kesişimi                     │              │
│  │  6. Koordinat Dönüşümü → GPS (lat, lon)            │              │
│  └──────────────────────────┬──────────────────────────┘              │
│                             │                                          │
│                             ▼                                          │
│  ┌─────────────┐     ┌─────────────┐                                   │
│  │   HEDEF     │────▶│  KOVARYANS  │  Konum belirsizliği              │
│  │  KONUM      │     │  MATRİSİ    │  (σ_lat, σ_lon)                   │
│  │ (lat, lon)  │     │             │                                   │
│  └─────────────┘     └─────────────┘                                   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## ⚔️ Saldırı Görevi İş Akışı

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      SALDIRI GÖREVİ SÜRECİ                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  HEDEF DURUM MAKİNESİ:                                                  │
│                                                                         │
│  ┌─────────┐   tespit       ┌─────────┐   onay        ┌─────────┐     │
│  │  FREE   │ ──────────────▶│  OWNED  │ ─────────────▶│ KİLİTLİ │     │
│  └─────────┘                └─────────┘               └─────────┘     │
│       ▲                          │                         │          │
│       │                          │                         │          │
│       │                     kayıp │                    saldırı│         │
│       │                          ▼                         │          │
│       │                    ┌─────────┐                      ▼          │
│       └────────────────────│  KAYIP  │             ┌──────────┐       │
│                            └─────────┘             │SALDIRIYOR│       │
│                                                    └──────────┘       │
│                                                         │              │
│                                                         ▼              │
│                                                  ┌──────────┐         │
│                                                  │ ÇARPIŞMA │         │
│                                                  └──────────┘         │
│                                                                         │
│  ATAMA AKIŞI:                                                           │
│                                                                         │
│  ┌────────────┐    ┌────────────┐    ┌────────────┐    ┌────────────┐  │
│  │   TESPİT   │───▶│   FÜZYON   │───▶│   ATAMA    │───▶│   KİLİT    │  │
│  │   HEDEF    │    │ GÖZLEMLER  │    │   DRONE    │    │   HEDEF    │  │
│  └────────────┘    └────────────┘    └────────────┘    └────────────┘  │
│        │                 │                 │                 │          │
│        ▼                 ▼                 ▼                 ▼          │
│  YOLO+BoT-SORT    Kalman Filtre    Hungarian Alg.   Durum Makinesi    │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 🧠 Algoritmalar

### 1. Işın-Zemin Kesişimi Jeolokasyonu

Monoküler RGB kamera ile ışın-zemin kesişimi kullanarak konum tahmini:

```
Girdi: Sınırlayıcı kutu merkezi (pixel_x, pixel_y), Drone durumu (lat, lon, alt, roll, pitch, yaw)
Çıktı: 3x3 kovaryans matrisi ile GPS koordinatları (lat, lon)

Algoritma (geo_math.py):
1. Piksel → Normalize Kamera: (x_norm, y_norm) = ((px - cx)/fx, (py - cy)/fy)
2. Kamera → Gövde Çerçevesi: Drone gövde hizalaması için eksen dönüşümü
3. Kamera Pitch Offseti Uygula: Gimbal montaj açısını telafi et
4. Gövde → NED Çerçevesi: RPY rotasyon matrisi ile dönüşüm (ZYX sırası)
5. Işın-Zemin Kesişimi: Işının z=0 düzlemiyle buluştuğu t'yi çöz
   - kesişim = p0 + t * ray_ned
   - Sığ ışınları (ray_z < eşik) sayısal kararlılık için sıkıştır
6. NED → GPS: (d_north, d_east) değerlerini (lat, lon) ofsetlerine dönüştür

Kovaryans Tahmini (Jakobi Yayılımı):
- σ_azimuth = √(σ_yaw² + σ_pixel²) → σ_cross = slant_range * σ_azimuth
- σ_elevation = √(σ_pitch² + σ_pixel²) → σ_radial = slant_range * cot(ε) * σ_elevation
- NED çerçevesinde 3x3 kovaryans oluştur, GPS bearing'e dönüştür
```

### 2. BoT-SORT Takibi

Çoklu nesne takibi algoritması:

```
Özellikler:
- Hareket tahmini için Kalman Filtre
- IoU tabanlı track ilişkilendirme
- Track yönetimi (yeni, onaylı, kayıp, silindi)
- Kamera hareket telafisi (CMC)
```

### 3. Hedef Füzyonu (Kalman Filtre)

Çoklu drone gözlem füzyonu:

```
Durum Vektörü: [lat, lon, v_lat, v_lon]
Süreç Modeli: Sabit hız
Ölçüm Modeli: GPS + kovaryans

Füzyon Adımları:
1. Süreç modeli ile durumu tahmin et
2. Birden fazla drondan gözlemleri al
3. Mahalanobis mesafesi kapısı
4. Geçerli gözlemlerle durumu güncelle
5. Çıktı: Azaltılmış kovaryanslı füzyonlanmış konum
```

### 4. Hungarian Ataması

Optimal drone-hedef ataması:

```
Cost Matrix Faktörleri:
- Mesafe (temel maliyet)
- Görünürlük (drone hedefi görüyor: bonus)
- Yönelim (hedef önde: bonus, arkada: ceza)
- Hareket (hareketli drone: ceza)
- Dağılım (atanmış hedeflere yakın: ceza, uzak: bonus)
- Kovaryans (yüksek belirsizlik: ceza, düşük: bonus)
- Yapışkanlık (mevcut atama: bonus)

Algoritma: linear_sum_assignment (scipy.optimize)
```

---

## 🏗️ Proje Yapısı

```
ORCUS/
├── app.py                              # Ana Flask uygulaması
├── config.py                           # Sistem yapılandırması
├── requirements.txt                    # Python bağımlılıkları
│
├── modules/
│   ├── core/
│   │   ├── logger.py                   # Loglama sistemi
│   │   ├── geo_math.py                 # Işın-zemin kesişimi GPS tahmini
│   │   ├── kalman_filter.py            # Kalman Filtre uygulaması
│   │   ├── pid_controller.py           # PID kontrol
│   │   └── fleet_manager.py            # Filo durum yönetimi
│   │
│   ├── mission/
│   │   ├── flight_controller.py       # MAVLink uçuş kontrolü
│   │   ├── mission_controller.py      # Görev orkestrasyonu
│   │   ├── tracking_controller.py     # Hedef takip kontrolü
│   │   └── ibvs_guidance.py            # Görüntü tabanlı görsel servoing
│   │
│   ├── swarm/
│   │   ├── swarm_coordinator.py        # Sürü lider-takipçi mantığı
│   │   └── target_fusion.py            # Kalman Filtre füzyonu
│   │
│   ├── vision/
│   │   ├── detector.py                 # YOLOv12 tespit + takip
│   │   ├── camera_handler.py           # Kamera akış yönetimi
│   │   ├── group_tracker.py            # Grup takip mantığı
│   │   └── tracker/
│   │       ├── bot_sort.py             # BoT-SORT uygulaması
│   │       ├── mc_bot_sort.py          # Hareket telafili BoT-SORT
│   │       ├── kalman_filter.py        # Tracker Kalman Filtre
│   │       ├── matching.py             # Track ilişkilendirme
│   │       ├── gmc.py                  # Kamera hareket telafisi
│   │       ├── basetrack.py            # Temel track sınıfı
│   │       └── weights/
│   │           └── yolov12n.pt         # YOLO model ağırlıkları
│
├── templates/
│   └── index.html                      # Web arayüzü
│
├── static/
│   ├── css/                            # Stil dosyaları
│   └── js/                             # JavaScript
│
├── simulator/
│   ├── drone/                          # Gazebo drone modelleri
│   └── worlds/                         # Gazebo dünya dosyaları
│
├── logs/
│   └── swarm_log.txt                   # Sistem logları
│
└── tests/                              # Birim testleri
```

---

## 🔮 Yaklaşan: v2.1 Yol Haritası

**Durum: Planlama Aşaması**

v2.1, sistem optimizasyonu, mimari konsolidasyonu ve algoritma iyileştirmelerine odaklanacak:

### Temel Hedefler

| Amaç | Açıklama |
|------|----------|
| **Sistem Optimizasyonu** | Parametre karmaşıklığını azalt, algoritmaları sadeleştir, performansı artır |
| **Mimari Temizliği** | Eski kodları kaldır, algoritma çakışmalarını çöz, yedek mantığı birleştir |
| **Takip Stabilitesi** | ID değişimini engelle, takip kalıcılığını artır, atama tutarlılığını güçlendir |
| **Konum Doğruluğu** | Işın-Zemin Kesişimi'ni iyileştir, kovaryans tahminini geliştir |

### Ana İyileştirmeler

**Algoritma Konsolidasyonu:**
- Artımlı geliştirmeden biriken eski/kullanılmayan algoritmaları kaldır
- Çakışan algoritmalar arasındaki uyumsuzlukları çöz
- Yinelenen uygulamaları tek, optimize edilmiş modüllerde birleştir

**Takip ve Atama:**
- İyileştirilmiş ilişkilendirme mantığı ile tracker ID değişimini azalt
- Gelişmiş cost matrix skorlaması ile drone-hedef atamalarını stabilize et
- Çoklu hedef senaryolarında performansı artır

**Konum Tahmini:**
- Işın-Zemin Kesişimi doğruluğunu ve kenar durumları iyileştir
- Belirsiz ölçümler için daha iyi kovaryans tahmini
- Zorlu koşullarda daha sağlam füzyon

### Ele Alınan Bilinen Sorunlar

- Aşırı parametre sayısının neden olduğu yapılandırma karmaşıklığı
- Artımlı geliştirme geçmişinden kaynaklanan algoritma çakışmaları
- Çoklu hedef senaryolarında tracker ID kararsızlığı
- Eşzamanlı tespitlerde drone-hedef atama değişimleri
- Daha büyük sürüler için performans darboğazları

---

## 📄 Lisans

Bu proje Apache Lisansı 2.0 altında lisanslanmıştır - detaylar için [LICENSE](LICENSE) dosyasına bakın.

---

## ⚠️ Yasal Uyarı

Bu proje **eğitim ve araştırma amaçlıdır**. Geliştiriciler bu sistemin kötüye kullanımından sorumlu değildir. Her zaman drone operasyonlarıyla ilgili yerel yasalara ve düzenlemelere uyun.
