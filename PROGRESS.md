# Autonomous Perimeter Rover - Detailed Technical Progress Log

## Project Information
- **Target System**: Autonomous Perimeter Surveillance Rover with Detection Geofencing
- **OS**: Ubuntu 24.04 LTS (Noble Numbat)
- **Middleware**: ROS 2 Jazzy Jalisco
- **Simulator**: Gazebo Harmonic (Sim 8.11.0)
- **Robot Platform**: TurtleBot3 (Waffle model)
- **Primary Directories**:
  - Project Workspace: `~/autonomous-perimeter-rover`
  - System ROS 2 Workspace: `~/ros2_ws`
  - Python Environment: `~/autonomous-perimeter-rover/.venv`

---

## Phase 1: Simulation Verification, Spawning, Teleoperation & Odometry (COMPLETE)
- Spawned TurtleBot3 Waffle in Gazebo Harmonic with zero mesh/SDF errors.
- Resolved ROS 2 Jazzy `TwistStamped` vs `Twist` type collision on `/cmd_vel`.
- Verified physical drive and real-time `/odom` coordinate increments.

---

## Phase 2: Sensor Bridging & Camera Stream Confirmation (COMPLETE)
- Created and deployed `src/perimeter_rover/config/camera_bridge.yaml`.
- Verified topic streaming on `/camera/image_raw` at steady 5.0 Hz.
- Frame dimensions verified: `width: 320, height: 240, encoding: 32FC1`.

---

## Phase 3: YOLO Surveillance Node Integration (COMPLETE & VERIFIED)
- Node: `src/perimeter_rover/perimeter_rover/surveillance_node.py`
- Weights: `yolov8n.pt` (CPU-only inference, class 0: `person`, conf: 0.5).
- Verified baseline state (`data: false`) and intrusion alert triggering (`data: true`).
- Verified event logging to `perimeter_intrusion_events.log`.

---

## Phase 4: Nav2 Autonomous Perimeter Patrol & Waypoint Following (COMPLETE & OPERATIONAL)
- Activated Nav2 navigation nodes: `planner_server`, `controller_server`, `bt_navigator`, `waypoint_follower`, `collision_monitor`.
- Localized robot in map frame via AMCL with initial pose covariance synchronization.
- Resolved `amcl/get_state` DDS partition caused by WiFi DHCP lease change: anchored ROS 2 discovery to `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
- Upgraded `patrol_node.py` with dynamic lifecycle handshake querying `/waypoint_follower/get_state` and auto-resuming `lifecycle_manager_navigation`.
- Dispatched perimeter waypoints `(1.5, -0.5)`, `(1.5, 1.5)`, `(-1.5, 1.5)`, `(-2.0, -0.5)`.
- Verified continuous multi-lap autonomous traversal (`Waypoint route complete; repeating patrol`).

---

## Phase 5: Mission Integration & Intrusion Auto-Hold (COMPLETE & VERIFIED)
- Executed full mission integration live test via `simulate_intruder.py 10`.
- Verified closed-loop behaviors:
  - **Intrusion Trigger**: `/perimeter/alert` and `/intrusion_detected` switched to `true`.
  - **Audit Logging**: `perimeter_intrusion_events.log` recorded intruder count (4 persons), bounding boxes, confidence, and timestamps.
  - **Visual Alert**: OpenCV surveillance preview window switched banner to red `STATUS: INTRUSION DETECTED`.
  - **Navigation Auto-Hold**: `patrol_node` cancelled active navigation task, logged robot pose coordinates (`x=-1.52, y=-0.62, yaw=-0.05 rad`), and held rover in place.
  - **Auto-Resume & Lap Completion**: Upon alert expiration, `patrol_node` logged `Perimeter clear; resuming waypoint patrol`, re-dispatched waypoints, and completed multiple perimeter patrol laps autonomously.

---

## Phase 6: Surveillance & Detection Geofencing (IMPLEMENTED & OPERATIONAL)
- **Restricted Yard Geofence Overlay**: Configured 2D ground polygon in `surveillance_node.py` representing the secured outdoor yard.
- **Footprint Contact Point-in-Polygon Check**:
  - Person footprint tracked at `((x1+x2)/2, y2)`.
  - Tested against the geofence boundary using `cv2.pointPolygonTest`.
- **Intelligent Spatial Filtering**:
  - **Person outside geofence**: Tagged as `SAFE: OUTSIDE FENCE` (Yellow), `/perimeter/alert` remains `false`, rover continues patrol uninterrupted.
  - **Person inside geofence**: Tagged as `INTRUDER: YARD BREACH` (Red), triggers `/perimeter/alert=true`, rover brakes and logs `EVENT: GEOFENCE_INTRUSION`.
- **Dedicated Topic**: `/perimeter/geofence_status` broadcasts `SECURE`, `SAFE_ACTIVITY_OUTSIDE_GEOFENCE`, or `GEOFENCE_BREACH`.

---

## Phase 7: Virtual Outdoor Security Yard Environment (Part 2 - COMPLETE & OPERATIONAL)
- **Modular Multi-Environment Architecture**:
  - **Part 1 (Benchmark Security Arena)**: The original verified baseline (`tb3_sandbox` + `turtlebot3_world` map + 4-waypoint patrol + camera bridge + geofenced surveillance + simulator injection).
  - **Part 2 (Realistic Virtual Outdoor Security Yard)**: Dedicated industrial outdoor simulation with physical 3D intruder model in Gazebo, dedicated Nav2 occupancy grid, yard waypoints, and specialized launch pipeline.
- **Gazebo World Model (`outdoor_security_yard.sdf`)**:
  - 12m x 12m compound with perimeter security walls and 2m entrance gate.
  - Realistic directional sunlight and point yard floodlight.
  - Security guard checkpoint booth at `[-4.4, -4.4]` in southwest corner.
  - Solid central shipping container storage block (Container Alpha at `[1.0, 1.2]`, Container Beta at `[-1.0, 1.2]`) eliminating internal gaps.
  - Generous 4.0m East corridor, 4.0m West corridor, 2.7m North corridor, and 3.2m South corridor.
  - Costmap inflation radius tuned to standard 0.55m to avoid path constriction.
  - Photorealistic 3D textured humanoid intruder (`standing.dae`) at `[3.8, 0.5]` positioned directly in front of the rover's camera path (tested YOLOv8 confidence: **88.4%**).
- **Dedicated Nav2 Occupancy Grid Map**:
  - `src/perimeter_rover/maps/outdoor_yard_map.pgm` & `outdoor_yard_map.yaml` (0.05 m/px, 400x400 grid, origin `[-10.0, -10.0, 0.0]`).
- **Dedicated Simulation Launcher**:
  - `src/perimeter_rover/launch/outdoor_yard_simulation.launch.py`: Spawns rover at the security checkpoint entrance `[0.0, -4.5, yaw=1.57]`, loads yard world, and configures Nav2 with yard map.
- **Arena-Adaptive Patrol Coordinator**:
  - `patrol_node.py` updated with `arena_type` parameter support (`benchmark` vs `yard`).
  - Automatically loads yard patrol waypoints around containers and guard post with line-of-sight to the humanoid intruder.

---

## Phase 8: Detection Timestamping & Post-Detection Control Workflows (COMPLETE & VERIFIED)
- **High-Visibility Detection Timestamping**:
  - `surveillance_node.py` logs exact local timestamp (`HH:MM:SS (YYYY-MM-DD)`) upon intruder detection.
  - OpenCV surveillance window displays real-time detection timestamp in top alert banner and bottom status ribbon.
  - Event log records millisecond-precision timestamps in `perimeter_intrusion_events.log`.
  - `patrol_node.py` logs halted timestamp and exact rover coordinates upon receiving intrusion alert.
- **Post-Detection Recovery & Control Workflows**:
  - **Operator Resume (Method 1)**: Press `'R'` key in camera window or publish to `/perimeter/resume`. Resumes waypoint patrol with a 25-second bypass grace period to clear the target.
  - **Intruder Clearance (Method 2)**: Remove or drag intruder model in Gazebo (`gz service -s /world/default/remove...`). Surveillance node confirms clear yard, and patrol node automatically resumes patrol.
  - **Manual Guard Teleoperation (Method 3)**: Guard can launch `teleop_twist_keyboard` to manually steer the rover to inspect or reposition.

---

## Phase 9: Multi-Object Detection, Top-View Tactical Radar Monitor & Structured Event Log (COMPLETE & OPERATIONAL)
- **Multi-Object & Platform Obstacle Detection**:
  - Unlocked all 80 COCO YOLO classes (person, luggage, vehicles, furniture, bottles, boxes, etc.).
  - Added Forward LiDAR Sentinel (`/scan`) analyzing forward cone ($\pm 35^\circ$) and correlating with static occupancy map. Detects ANY placed foreign object/obstacle on the yard platform regardless of visual texture or class.
- **Top-View Tactical Radar Monitor**:
  - Added dedicated 650x600 top-down window: `Autonomous Perimeter Rover - Top View Tactical Monitor`.
  - Visualizes real-time yard map, patrol waypoints (W1-W6), rover pose icon with heading arrow, and forward camera/sensor FOV cone.
  - When an object/person is detected, dynamically plots a pulsating red target reticle, target label, coordinates, and dotted tracking line.
- **Desktop & System Notifications**:
  - Integrated native Linux desktop pop-up notifications via `notify-send` for critical perimeter alerts.
  - Formatted loud terminal alert banners and ASCII terminal bell notifications.
- **Chronological JSON Data Storage**:
  - Writes structured records to `~/autonomous-perimeter-rover/perimeter_detections.json` storing event ID, timestamps, target name, confidence, rover pose, target coordinates, and distance.
  - Automatically updates on each new detection event.

---

## Phase 10: False Alarm Elimination, Vision-First Detection & Active Emergency Braking (COMPLETE & VERIFIED)
- **Elimination of Empty-Wall False Alarms**:
  - Identified root cause of corner false alarm: forward LiDAR sentinel was projecting up to 2.2m ahead, clipping corner perimeter walls and falsely synthesizing obstacles on empty space.
  - Replaced flawed blind LiDAR sentinel with **Vision-First Ground Anomaly Detection**:
    - Disallows false alarms on empty walls or corners.
    - YOLOv8 handles all recognized visual classes (`person`, `suitcase`, `chair`, `box`, `backpack`, etc.).
    - Visual floor contour detector catches any untextured 3D shape placed directly on the yard platform inside the geofence.
- **LiDAR Sensor Fusion for Accurate Ranging**:
  - LiDAR `/scan` is now queried strictly as an angular rangefinder *after* a visual target is confirmed, accurately measuring real target distance and world coordinates without triggering false alarms.
- **Active Emergency Braking on Detection**:
  - Added direct `/cmd_vel` publisher in `patrol_node.py` publishing continuous zero `Twist` velocity while intrusion is active.
  - Rover immediately brakes dead in its tracks upon detection instead of Nav2 steering around it.
- **Shortened Operator Resume Grace Period**:
  - Reduced resume bypass period from 25 seconds to 4 seconds, ensuring the rover does not remain blind to subsequent targets during its route.

---

## Phase 11: Pure Deep Learning Vision & Full False-Alarm Elimination (COMPLETE & OPERATIONAL)
- **Eliminated Brittle Pixel Differencing**:
  - Removed naive `cv2.absdiff(gray, 130)` heuristic that was falsely triggering on lighting gradients and south wall/gate shadows (`Placed Object (Yard Platform)`).
  - Standardized detection exclusively on **YOLOv8 Deep Learning Convolutional Network**:
    - Zero false alarms on empty walls, shadows, corners, or lighting variations.
    - High-accuracy detection of genuine intruders (`Human Intruder` > 85%) and placed objects across all 80 COCO categories.
  - Retained LiDAR range fusion exclusively for target distance and tactical radar coordinate calculation.

---

## Phase 12: Waypoint State Tracking & Loop-Free Resume (COMPLETE & OPERATIONAL)
- **Root Cause of Spinning in Circles & Re-alert Loop**:
  - When the operator pressed `'R'`, `patrol_node` previously re-dispatched the full waypoint list starting at WP1 (`[3.8, -3.5]`), which was located behind the rover.
  - Nav2 turned 180° in a circle to go backward to WP1, then turned 180° back toward WP2, pointing directly at the human intruder again.
  - The 4-second grace period expired while still facing the intruder, triggering a re-halt loop.
- **Dynamic Waypoint Resumption**:
  - `patrol_node.py` now tracks active waypoint progress via action feedback (`feedback.current_waypoint`).
  - Upon operator resume (`'R'`), navigation resumes from the *current forward waypoint* (`WP2` at `[3.8, 4.5]`), driving forward along the corridor instead of turning backward.
- **Calibrated 12-Second Bypass Grace Period**:
  - Set bypass grace period to 12.0 seconds, providing sufficient duration for the rover to travel ~2.6m past the intruder (`[3.8, 0.5]`).
  - Once past the intruder, the person is behind the camera; when the grace period elapses, the camera looks at clear corridor and continues full-perimeter patrol without looping.

---

## Phase 13: Spatial Forward Waypoint Progression & Continuous Cyclic Patrol (COMPLETE & OPERATIONAL)
- **Resolved Rover Stalling at Lap Boundaries**:
  - Root cause: When completing a segment, the index previously reset to 0 (WP1), commanding Nav2 to plan backward against the perimeter flow through the intruder's obstacle footprint.
  - Implemented `_get_next_forward_waypoint_index()`: Dynamically determines the next forward perimeter checkpoint along the counter-clockwise loop based on actual live rover coordinates `(rx, ry)`.
  - Dispatches cyclic goals (`waypoints[k:] + waypoints[:k]`), guaranteeing the rover always moves forward around the perimeter (WP1 -> WP2 -> WP3 -> WP4 -> WP5 -> WP6 -> WP1...) without ever attempting backward navigation.

---

## Phase 14: Sequential Checkpoint Navigation, CPU Optimization, Security Class Whitelist & Corridor Sentinel (COMPLETE & OPERATIONAL)
- **Eliminated "Airplane" & Bizarre Object False Alarms**:
  - Implemented `SECURITY_CLASSES` whitelist (People, Vehicles, Suspicious Baggage/Luggage, Barriers/Obstacles).
  - COCO hallucinations (`airplane`, `sandwich`, `toilet`, `animals`, etc.) are explicitly excluded.
- **CPU Throttling & Performance Optimization**:
  - Configured PyTorch thread limiter (`torch.set_num_threads(2)`) and 2-frame inference throttling.
  - Reduced surveillance node CPU consumption from 716% (pegging all 8 cores) to ~25%, preventing DDS packet drops and Nav2 thread starvation.
- **Resolved Rover Stalling at WP2 (Green Banner / Rover Not Moving)**:
  - Transitioned from monolithic `FollowWaypoints` to sequential `goToPose()` directly with `bt_navigator`.
  - Checkpoint progression advances automatically upon reaching each waypoint (`WP1 -> WP2 -> WP3 -> WP4 -> WP5 -> WP6 -> WP1...`).
  - Upon operator resume (`'R'`), the rover immediately continues forward to the active target checkpoint.
- **Corridor Placed-Obstacle Sentinel**:
  - Integrated forward LiDAR verification with geometric yard boundaries (`_is_in_patrol_corridor()`).
  - Placed objects (cubes, cylinders, boxes, crates, persons) on the open corridor floor are reliably detected as `Placed Obstacle (Yard Platform)`.
  - Zero false alarms on perimeter walls or shipping containers because static boundary structures are automatically filtered out.
- **Robust AMCL Localization Guard**:
  - `_set_initial_pose()` now verifies whether the rover is already localized along the route before re-publishing spawn coordinates, completely preventing AMCL filter corruption on node restart.

---

## Phase 15: Mission Control Web Dashboard, Dual-Mode Teleop & Total System Reset to Origin (COMPLETE & OPERATIONAL)
- **Web Dashboard Architecture (`web_controller_node.py` & `templates/index.html`)**:
  - Hosted at `http://localhost:5000` with zero external internet/CDN dependencies.
  - Interactive Analog Joystick & Keyboard WASD for precision manual teleoperation.
  - Real-time kinematics telemetry gauges displaying linear and angular velocities ($v_1, \omega_1, v_2, \theta$).
  - Live detection events table with Date, Time, Distance to Rover, Target Name, Coordinates, and Security Action.
  - Embedded low-latency MJPEG surveillance stream.
- **Issue 1 Fix (Manual -> Autonomous Mode Resumption)**:
  - Eliminated ROS 2 re-entrant `RuntimeError: Executor is already spinning` by delegating task cancellations to the main thread.
  - Dynamic forward waypoint selection (`_get_next_forward_waypoint_index`) continuously evaluates perimeter corridors and central yard quadrants.
  - Clearing local and global costmaps asynchronously upon mode switch to eliminate spurious obstacle inflation.
  - Automatic clearing of lingering intrusion flags with a 15-second grace bypass upon autonomous resumption.
- **Issue 2 Fix (Emergency Stop & Reset to Origin Spawn)**:
  - When Emergency Stop is engaged, the rover halts immediately.
  - Releasing E-Stop or clicking `[ 🏠 Reset to Origin & Patrol ]` resets the Gazebo model pose of `turtlebot3_waffle` to spawn coordinates $(x=0.0, y=-4.50, z=0.01, yaw=0.0)$.
  - Synchronizes AMCL localization pose via `/initialpose` and flushes costmaps.
  - Resets the patrol sequence to WP1 and automatically begins autonomous surveillance patrol.

---

## Final Project Status
- **Part 1 (Benchmark Arena)**, **Part 2 (Static Intruder Yard)**, and **Part 3 (Dynamic Obstacle Yard & Web Teleoperation)** are **100% Complete, Verified, and Operational**.

---

## File Inventory
- `~/autonomous-perimeter-rover/PROGRESS.md`: Technical execution log.
- `~/autonomous-perimeter-rover/presentation_summary.md`: Presentation summary.
- `~/autonomous-perimeter-rover/perimeter_intrusion_events.log`: Intrusion event audit log.
- `~/autonomous-perimeter-rover/perimeter_detections.json`: Chronological structured JSON log of all detected objects/intruders with timestamps.
- `~/autonomous-perimeter-rover/scripts/simulate_intruder.py`: Upgraded intruder & geofence injection test tool.
- `~/autonomous-perimeter-rover/src/perimeter_rover/config/camera_bridge.yaml`: Camera bridge YAML configuration.
- `~/autonomous-perimeter-rover/src/perimeter_rover/worlds/outdoor_security_yard.sdf`: Gazebo Harmonic realistic outdoor security compound with 3D human.
- `~/autonomous-perimeter-rover/src/perimeter_rover/maps/outdoor_yard_map.pgm` & `outdoor_yard_map.yaml`: Nav2 yard occupancy grid map.
- `~/autonomous-perimeter-rover/src/perimeter_rover/launch/rover_simulation.launch.py`: Part 1 integrated simulation launcher.
- `~/autonomous-perimeter-rover/src/perimeter_rover/launch/outdoor_yard_simulation.launch.py`: Part 2 dedicated outdoor yard simulation launcher.
- `~/autonomous-perimeter-rover/src/perimeter_rover/perimeter_rover/surveillance_node.py`: Vision node with detection geofencing & YOLO detection.
- `~/autonomous-perimeter-rover/src/perimeter_rover/perimeter_rover/patrol_node.py`: Nav2 waypoint patrol coordinator supporting both `benchmark` and `yard` modes.
