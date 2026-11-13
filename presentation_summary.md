# Autonomous Perimeter Surveillance Rover - Presentation & Executive Summary

## 1. Executive Summary & Final System Status
The Autonomous Perimeter Surveillance Rover project implements a complete autonomous mobile security robotic system using **ROS 2 Jazzy** and **Gazebo Harmonic (Sim 8)** on **Ubuntu 24.04 LTS**. The rover autonomously patrols a multi-waypoint perimeter route, performs real-time visual intrusion inspection via **Ultralytics YOLO** on CPU, halts and holds position upon perimeter breach, and resumes patrol once clear.

| Subsystem / Feature | Status | Verification Gate |
| :--- | :--- | :--- |
| **Gazebo Harmonic Simulation & Spawning** | **VERIFIED & OPERATIONAL** (Phase 1) | Model spawns cleanly; DiffDrive active |
| **Teleoperation & Odometry Feedback** | **VERIFIED & OPERATIONAL** (Phase 1) | Closed-loop `/cmd_vel` response & `/odom` tracking |
| **Sensor & Image Bridge (`ros_gz_image`)** | **VERIFIED & OPERATIONAL** (Phase 2) | Active publishers on `/camera/image_raw` at 5.0 Hz |
| **YOLO Surveillance Node & Alerts** | **VERIFIED & OPERATIONAL** (Phase 3) | Person detection triggers alert + log + GUI banner |
| **Nav2 Autonomous Perimeter Patrol** | **VERIFIED & OPERATIONAL** (Phase 4) | Multi-waypoint continuous patrol verified live in simulation |
| **Mission Integration & Auto-Hold** | **VERIFIED & OPERATIONAL** (Phase 5) | Halts on intrusion, logs pose, holds position, auto-resumes & repeats laps |

---

## 2. Core Architecture & Data Flow
```
[ Gazebo Sim 8 (TurtleBot3 Waffle Camera Sensor) ]
                        │
                        ▼  (ros_gz_bridge / camera_bridge.yaml)
           ROS 2 Topic: `/camera/image_raw`  (5 Hz, 320x240)
                        │
                        ▼
      [ ROS 2 Python Node (rclpy + cv_bridge) ]
                        │  Image conversion to OpenCV BGR
                        ▼
       Ultralytics YOLO (yolov8n.pt / yolo26n.pt)
                        │
              Is person detected?
                 ├── Yes ➔ Publish `/perimeter/alert` + write `perimeter_intrusion_events.log`
                 │         └── Halt Nav2 Patrol & Hold Pose (patrol_node)
                 └── Always ➔ Publish visual annotations to `/perimeter/annotated_image`
```

---

## 3. Key Milestones Achieved
- **Phases 1–5 Fully Verified & 100% Complete**:
  - Closed-loop physical simulation in Gazebo Harmonic proven.
  - Camera sensor bridge streaming at steady 5.0 Hz.
  - Real-time YOLOv8 CPU intrusion detection, `/perimeter/alert` topic, and audit logging proven.
  - Nav2 multi-waypoint perimeter traversal validated live in simulation across multiple laps.
  - Full closed-loop integration: rover detected simulated intruder, immediately braked and held position at `x=-1.52, y=-0.62, yaw=-0.05 rad`, changed GUI banner to RED, logged event to `perimeter_intrusion_events.log`, automatically resumed waypoint traversal when clear, and completed multiple perimeter patrol laps.

---

---

## 4. Demonstration Run Instructions

### PART 1: Benchmark Security Arena (Standard Arena + Geofencing Test)
Use these commands when demonstrating the baseline benchmark world with virtual geofencing injection:

```bash
# Terminal 1: Simulation + Nav2 + RViz2 (Benchmark World)
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False use_rviz:=True

# Terminal 2: Camera Bridge
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$(ros2 pkg prefix perimeter_rover)/share/perimeter_rover/config/camera_bridge.yaml

# Terminal 3: YOLO Surveillance & Detection Geofencing Node
ros2 run perimeter_rover surveillance_node

# Terminal 4: Autonomous Perimeter Patrol Coordinator (Benchmark Mode)
ros2 run perimeter_rover patrol_node

# Terminal 5: Dynamic Geofence Test (Safe mode vs Yard Breach mode)
# Option A: Test Safe Person outside geofence (Yellow box, patrol continues):
python3 ~/autonomous-perimeter-rover/scripts/simulate_intruder.py safe 5

# Option B: Test Yard Breach inside geofence (Red box, rover brakes & holds):
python3 ~/autonomous-perimeter-rover/scripts/simulate_intruder.py breach 5
```

---

### PART 2: Realistic Virtual Outdoor Security Yard (Full 3D Physical Compound)
Use these commands when demonstrating the realistic industrial outdoor yard with shipping containers, guard checkpoint booth, and the physical 3D humanoid intruder standing in Gazebo:

```bash
# Terminal 1: Outdoor Security Yard Sim + Nav2 + RViz2 + Color Camera Bridge
ros2 launch perimeter_rover outdoor_yard_simulation.launch.py

# Terminal 2: Universal Surveillance, LiDAR Sentinel & Tactical Top-View Monitor
ros2 run perimeter_rover surveillance_node

# Terminal 3: Autonomous Perimeter Patrol Coordinator (Outdoor Yard Mode)
ros2 run perimeter_rover patrol_node --ros-args -p arena_type:=yard
```

### Visual & Operational Features:
- **Dual Live Windows**:
  1. `Autonomous Perimeter Rover - Surveillance Stream`: Full-color RGB stream with YOLO bounding boxes, class tags, and status ribbons.
  2. `Autonomous Perimeter Rover - Top View Tactical Monitor`: Real-time 2D bird's-eye map with rover pose beacon, heading arrow, sensor FOV cone, waypoint path, and pulsating red target reticles on detected obstacles.
- **Instant Notifications**:
  - Native Linux Desktop Pop-up via `notify-send`.
  - Loud formatted terminal alert banners with exact timestamps and coordinates.
- **Data Persistence**:
  - Chronological structured log: [`perimeter_detections.json`](file:///home/purshotam/autonomous-perimeter-rover/perimeter_detections.json).
  - Human-readable audit trail: [`perimeter_intrusion_events.log`](file:///home/purshotam/autonomous-perimeter-rover/perimeter_intrusion_events.log).

### Post-Detection Operational Controls:
Once the rover detects the intruder and halts with timestamped notification:
- **Method 1 (Resume Patrol)**: Click the camera window and press `'R'`, or run:
  `ros2 topic pub --once /perimeter/resume std_msgs/msg/Bool "{data: true}"`
- **Method 2 (Clear Intruder in Gazebo)**: Delete the model in Gazebo, or run:
  `gz service -s /world/default/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "human_intruder", type: 2'`
- **Method 3 (Manual Guard Teleoperation)**:
  `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
