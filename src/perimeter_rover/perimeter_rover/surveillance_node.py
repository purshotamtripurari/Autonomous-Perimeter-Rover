import datetime
import json
import math
import os
import subprocess
import sys
import time

# Ensure Python virtual environment packages (ultralytics, cv2, torch) are accessible
venv_site_packages = os.path.expanduser("~/autonomous-perimeter-rover/.venv/lib/python3.12/site-packages")
if os.path.exists(venv_site_packages) and venv_site_packages not in sys.path:
    sys.path.insert(0, venv_site_packages)

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String, Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener
from ultralytics import YOLO

# Limit PyTorch CPU threads to prevent 100% CPU saturation and starvation of ROS/Nav2
torch.set_num_threads(2)

# Strict whitelist of legitimate security & site obstacle classes
SECURITY_CLASSES = {
    0: "Human Intruder",
    1: "Bicycle",
    2: "Vehicle (Car)",
    3: "Motorcycle",
    5: "Vehicle (Bus)",
    7: "Vehicle (Truck)",
    24: "Suspicious Backpack",
    26: "Suspicious Handbag",
    28: "Suspicious Suitcase / Package",
}


class SurveillanceNode(Node):
    def __init__(self):
        super().__init__("surveillance_rover")

        # Operational parameters
        self.declare_parameter("arena_type", "yard")
        self.declare_parameter(
            "model_path",
            os.path.expanduser("~/autonomous-perimeter-rover/yolov8n.pt"),
        )
        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter(
            "log_file",
            os.path.expanduser("~/autonomous-perimeter-rover/perimeter_intrusion_events.log"),
        )
        self.declare_parameter(
            "json_log_file",
            os.path.expanduser("~/autonomous-perimeter-rover/perimeter_detections.json"),
        )
        self.declare_parameter(
            "map_file",
            os.path.expanduser("~/autonomous-perimeter-rover/src/perimeter_rover/maps/outdoor_yard_map.pgm"),
        )
        self.declare_parameter("show_preview", True)
        self.declare_parameter("avoidance_mode", True)

        # Surveillance detection geofence parameters (normalized image coordinates: 0.0 to 1.0)
        self.declare_parameter("enable_geofence", True)
        self.declare_parameter(
            "geofence_coords",
            [0.10, 0.30, 0.90, 0.30, 0.98, 0.98, 0.02, 0.98],
        )

        self.arena_type = str(self.get_parameter("arena_type").value).strip().lower()
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.log_file_path = str(self.get_parameter("log_file").value)
        self.json_log_path = str(self.get_parameter("json_log_file").value)
        self.map_file_path = str(self.get_parameter("map_file").value)
        self.show_preview = bool(self.get_parameter("show_preview").value)
        self.avoidance_mode = bool(self.get_parameter("avoidance_mode").value)
        self.enable_geofence = bool(self.get_parameter("enable_geofence").value)
        self.geofence_coords = [float(v) for v in self.get_parameter("geofence_coords").value]

        # Resolve model weights path
        model_path = str(self.get_parameter("model_path").value)
        if not os.path.exists(model_path):
            fallback = os.path.expanduser("~/autonomous-perimeter-rover/yolov8n.pt")
            if os.path.exists(fallback):
                model_path = fallback

        self.get_logger().info(f"Loading YOLO model: {model_path} (Device: CPU, Conf: {self.conf_threshold})")
        self.model = YOLO(model_path)

        # TF2 listener for real-time map-to-rover positioning
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Load yard map for tactical top-view display
        self.static_map = None
        self.tactical_base_map = None
        self._load_tactical_map()

        # Yard patrol waypoints for top-view tactical visualization
        if self.arena_type == "yard":
            self.patrol_waypoints = [
                (3.5, -4.5),
                (3.5, 4.5),
                (-3.5, 4.5),
                (-3.5, -1.5),
                (-2.0, -4.5),
                (0.0, -4.5),
            ]
        else:
            self.patrol_waypoints = [
                (1.5, -0.5),
                (1.5, 1.5),
                (-1.5, 1.5),
                (-2.0, -0.5),
            ]

        # Subscribers & Publishers
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10,
        )

        # Alert topics
        self.alert_pub = self.create_publisher(Bool, "/perimeter/alert", 10)
        self.intrusion_pub = self.create_publisher(Bool, "/intrusion_detected", 10)
        self.geofence_status_pub = self.create_publisher(String, "/perimeter/geofence_status", 10)

        # Annotated image & resume command
        self.annotated_pub = self.create_publisher(Image, "/perimeter/annotated_image", 10)
        self.resume_pub = self.create_publisher(Bool, "/perimeter/resume", 10)
        self.resume_sub = self.create_subscription(Bool, "/perimeter/resume", self.resume_topic_callback, 10)
        self.create_subscription(Bool, "/perimeter/reset_to_origin", self.reset_to_origin_callback, 10)
        self.dynamic_obstacle_pub = self.create_publisher(Bool, "/perimeter/dynamic_obstacle", 10)
        self.dynamic_obstacle_data_pub = self.create_publisher(Float32MultiArray, "/perimeter/dynamic_obstacle_data", 10)

        # State tracking
        self.last_scan = None
        self.last_scan_time = 0.0
        self.frame_count = 0
        self.last_log_time = 0.0
        self.detection_counter = self._get_initial_event_count()
        self.last_detection_time_str = "None"
        self.last_detection_date_str = "None"
        self.last_target_name = "None"
        self.last_target_pos = (0.0, 0.0)
        self.last_target_dist = 0.0
        self.prev_obstacle_coords = None
        self.prev_obstacle_time = 0.0
        self.last_target_speed = 0.0
        self.last_target_heading = 0.0
        self.is_target_dynamic = False
        self.last_dynamic_detection_time = 0.0
        self.breach_active = False
        self.resume_override_until = 0.0
        self.cached_detections = []
        self.notification_id = 8888
        self.display_available = "DISPLAY" in os.environ and bool(os.environ["DISPLAY"])

        # Dismiss any leftover notifications from previous sessions
        self._dismiss_desktop_notification()

    def resume_topic_callback(self, msg: Bool):
        if msg.data:
            self.resume_override_until = time.time() + 35.0
            self.breach_active = False
            self._dismiss_desktop_notification()

    def reset_to_origin_callback(self, msg: Bool):
        if msg.data:
            self.resume_override_until = time.time() + 15.0
            self.breach_active = False
            self._dismiss_desktop_notification()
            self.get_logger().info("🔄 Reset to Origin received: Cleared surveillance breach & armed 15s grace period.")

    def _get_initial_event_count(self):
        try:
            if os.path.exists(self.json_log_path):
                with open(self.json_log_path, "r") as f:
                    data = json.load(f)
                    if isinstance(data, list):
                        return len(data)
        except Exception:
            pass
        return 0

    def _load_tactical_map(self):
        try:
            if os.path.exists(self.map_file_path):
                raw_map = cv2.imread(self.map_file_path, cv2.IMREAD_GRAYSCALE)
                if raw_map is not None:
                    self.static_map = raw_map
                    # Pre-render 520x520 stylized tactical top-view map
                    scaled = cv2.resize(raw_map, (520, 520), interpolation=cv2.INTER_NEAREST)
                    tactical = np.zeros((520, 520, 3), dtype=np.uint8)
                    tactical[scaled > 200] = (38, 44, 52)      # Open yard driving ground (dark slate)
                    tactical[scaled < 60] = (170, 150, 110)    # Walls & containers (cyan/steel outline)
                    tactical[(scaled >= 60) & (scaled <= 200)] = (25, 28, 33) # Unknown/background
                    self.tactical_base_map = tactical
                    self.get_logger().info(f"Tactical map loaded from {self.map_file_path}")
                    return
        except Exception as e:
            self.get_logger().warning(f"Could not load tactical map: {e}")

        # Fallback empty dark grid
        self.tactical_base_map = np.full((520, 520, 3), (35, 40, 48), dtype=np.uint8)

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_time = time.time()

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            rot = t.transform.rotation
            yaw = math.atan2(
                2.0 * (rot.w * rot.z + rot.x * rot.y),
                1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z),
            )
            return tx, ty, yaw
        except TransformException:
            return None

    def _is_in_patrol_corridor(self, x, y):
        """Returns True only if (x, y) is in the open patrol corridor (not a wall or container)."""
        if self.arena_type == "yard":
            # Outer perimeter walls: mask out within 0.75m of outer walls (|x|=6.0, |y|=6.0)
            if abs(x) >= 5.25 or abs(y) >= 5.25:
                return False

            # Central shipping containers: span x:[-1.9, 1.9], y:[-0.8, 3.2]
            if -2.15 <= x <= 2.15 and -1.05 <= y <= 3.35:
                return False

            # Southwest Guard Booth: spans x:[-5.2, -3.6], y:[-5.2, -3.6]
            if x <= -3.2 and y <= -3.2:
                return False

            # Central Cargo Crate / Pallet Stack: spans x:[-0.7, 0.7], y:[-2.7, -1.3]
            if -1.05 <= x <= 1.05 and -2.95 <= y <= -1.05:
                return False

            return True  # Open patrol corridor ground
        else:
            if abs(x) >= 2.2 or abs(y) >= 2.2:
                return False
            if -0.65 <= x <= 0.65 and -0.65 <= y <= 0.65:
                return False
            return True

    def _cv2_to_image_msg(self, cvim, header=None, encoding="bgr8"):
        img_msg = Image()
        if header is not None:
            img_msg.header = header
        img_msg.height = cvim.shape[0]
        img_msg.width = cvim.shape[1]
        img_msg.encoding = encoding
        img_msg.is_bigendian = 0
        channels = cvim.shape[2] if len(cvim.shape) > 2 else 1
        img_msg.step = cvim.shape[1] * channels * cvim.dtype.itemsize
        img_msg.data = cvim.tobytes()
        return img_msg

    def _world_to_canvas(self, x, y, canvas_ox=40, canvas_oy=55, map_dim=520, world_span=20.0):
        # Maps world coordinates [-10.0 to +10.0] into canvas pixels
        u = canvas_ox + int((x + 10.0) * (map_dim / world_span))
        v = canvas_oy + int((10.0 - y) * (map_dim / world_span))
        return u, v

    def _heading_to_cardinal(self, yaw_rad):
        deg = math.degrees(yaw_rad) % 360
        directions = [
            "E", "ENE", "NE", "NNE",
            "N", "NNW", "NW", "WNW",
            "W", "WSW", "SW", "SSW",
            "S", "SSE", "SE", "ESE"
        ]
        idx = int((deg + 11.25) / 22.5) % 16
        return directions[idx]

    def image_callback(self, msg: Image):
        if not rclpy.ok():
            return
        self.frame_count += 1

        try:
            # 1. Convert incoming ROS 2 Image to OpenCV BGR
            if msg.encoding == "32FC1":
                depth_image = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
                depth_array = np.nan_to_num(depth_image, nan=0.0, posinf=5.0, neginf=0.0)
                depth_norm = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX)
                depth_u8 = np.uint8(depth_norm)
                cv_image = cv2.cvtColor(depth_u8, cv2.COLOR_GRAY2BGR)
            elif msg.encoding in ("rgb8", "bgr8"):
                cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                if msg.encoding == "rgb8":
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            h, w = cv_image.shape[:2]

            # 2. Build pixel coordinates for the outdoor yard geofence polygon
            pts = np.array([
                [int(self.geofence_coords[i] * w), int(self.geofence_coords[i + 1] * h)]
                for i in range(0, len(self.geofence_coords), 2)
            ], np.int32)
            geofence_polygon = pts.reshape((-1, 1, 2))

            # 3. Retrieve live robot pose from TF
            robot_pose = self.get_robot_pose()
            rx, ry, ryaw = robot_pose if robot_pose is not None else (0.0, -4.5, 0.0)

            # 4. YOLO Inference (Every 2nd frame for low CPU usage; cached for intermediate frames)
            if self.frame_count % 2 == 0 or not self.cached_detections:
                results = self.model(
                    cv_image,
                    conf=self.conf_threshold,
                    device="cpu",
                    verbose=False,
                )
                current_detections = []
                for r in results:
                    for box in r.boxes:
                        cls_id = int(box.cls[0])
                        conf = float(box.conf[0])
                        # Ignore irrelevant COCO classes (airplane, animals, food, etc.)
                        if cls_id not in SECURITY_CLASSES:
                            continue

                        cls_name = SECURITY_CLASSES[cls_id]
                        x1, y1, x2, y2 = [float(c) for c in box.xyxy[0]]
                        foot_point = ((x1 + x2) / 2.0, y2)

                        if self.enable_geofence:
                            is_inside = cv2.pointPolygonTest(geofence_polygon, foot_point, False) >= 0
                        else:
                            is_inside = True

                        current_detections.append({
                            "class_name": cls_name,
                            "confidence": round(conf, 2),
                            "bbox": [round(c, 1) for c in (x1, y1, x2, y2)],
                            "foot_point": [round(c, 1) for c in foot_point],
                            "inside_geofence": is_inside,
                            "source": "YOLO Vision",
                        })
                self.cached_detections = current_detections

            detections = list(self.cached_detections)

            # 5. Placed Obstacle Sentinel (LiDAR + Corridor Map Geometry)
            corridor_obstacle_detected = False
            corridor_obstacle_dist = 0.0
            corridor_obstacle_coords = (0.0, 0.0)
            corridor_obstacle_rel_ang = 0.0

            scan_is_fresh = self.last_scan is not None and (time.time() - self.last_scan_time) < 1.5
            if robot_pose is not None and scan_is_fresh:
                scan = self.last_scan
                consecutive_hits = 0
                closest_r = float("inf")
                best_coords = None
                best_rel_ang = 0.0

                for i, r in enumerate(scan.ranges):
                    if not math.isfinite(r) or r < 0.50 or r > 3.0:
                        consecutive_hits = 0
                        continue

                    ang = scan.angle_min + i * scan.angle_increment
                    while ang > math.pi:
                        ang -= 2.0 * math.pi
                    while ang < -math.pi:
                        ang += 2.0 * math.pi

                    # Check within forward camera FOV sector (+/- 22 degrees)
                    if abs(ang) <= math.radians(22):
                        cx = rx + r * math.cos(ryaw + ang)
                        cy = ry + r * math.sin(ryaw + ang)
                        if self._is_in_patrol_corridor(cx, cy):
                            consecutive_hits += 1
                            if consecutive_hits >= 3:
                                if r < closest_r:
                                    closest_r = r
                                    best_coords = (cx, cy)
                                    best_rel_ang = ang
                        else:
                            consecutive_hits = 0
                    else:
                        consecutive_hits = 0

                if closest_r < float("inf") and best_coords is not None:
                    corridor_obstacle_detected = True
                    corridor_obstacle_dist = closest_r
                    corridor_obstacle_coords = best_coords
                    corridor_obstacle_rel_ang = best_rel_ang

            # 6. Target Fusion & Classification
            geofence_breach = False
            primary_target = "None"
            top_conf = 0.0
            target_coords = (0.0, 0.0)
            target_dist = 0.0

            # Priority 1: Visual security detections inside geofence
            breached_visual = [d for d in detections if d.get("inside_geofence", False)]
            if breached_visual:
                geofence_breach = True
                self.is_target_dynamic = False  # Visual target is human or stationary object
                primary_target = breached_visual[0]["class_name"]
                top_conf = max([d["confidence"] for d in breached_visual])
                target_bbox = breached_visual[0]["bbox"]
                target_cx = (target_bbox[0] + target_bbox[2]) / 2.0
                target_rel_ang = ((target_cx / float(w)) - 0.5) * math.radians(60)

                # Look up matching LiDAR distance
                target_dist = 1.4
                if scan_is_fresh:
                    scan = self.last_scan
                    min_r = float("inf")
                    for i, r in enumerate(scan.ranges):
                        if not math.isfinite(r) or r < 0.30 or r > 6.0:
                            continue
                        ang = scan.angle_min + i * scan.angle_increment
                        while ang > math.pi:
                            ang -= 2.0 * math.pi
                        while ang < -math.pi:
                            ang += 2.0 * math.pi
                        if abs(ang - target_rel_ang) <= math.radians(18):
                            if r < min_r:
                                min_r = r
                    if min_r < float("inf"):
                        target_dist = min_r

                if robot_pose is not None:
                    target_coords = (
                        rx + target_dist * math.cos(ryaw + target_rel_ang),
                        ry + target_dist * math.sin(ryaw + target_rel_ang),
                    )
                else:
                    target_coords = (3.8, 0.5)

                # Reject false positive visual detections on solid shipping container walls
                if self.arena_type == "yard" and (-2.15 <= target_coords[0] <= 2.15 and -1.05 <= target_coords[1] <= 3.35):
                    geofence_breach = False
                    primary_target = "None"

            # Priority 2: Placed obstacle on corridor floor (or dynamic rover or standing intruder)
            elif corridor_obstacle_detected:
                geofence_breach = True
                obs_x, obs_y = corridor_obstacle_coords
                now_t = time.time()
                is_dynamic = False
                target_speed = 0.0
                target_heading = 0.0

                if self.prev_obstacle_coords is not None and (now_t - self.prev_obstacle_time) < 1.0:
                    dt = max(now_t - self.prev_obstacle_time, 0.04)
                    dx = obs_x - self.prev_obstacle_coords[0]
                    dy = obs_y - self.prev_obstacle_coords[1]
                    dist_moved = math.hypot(dx, dy)
                    calc_speed = dist_moved / dt
                    # Speed between 0.06 m/s and 3.0 m/s indicates a dynamic moving entity
                    if 0.06 <= calc_speed <= 3.0:
                        is_dynamic = True
                        target_speed = calc_speed
                        target_heading = math.atan2(dy, dx)
                        self.last_dynamic_detection_time = now_t

                # Memory latch: if detected as dynamic within last 6 seconds, maintain dynamic classification
                if not is_dynamic and (now_t - self.last_dynamic_detection_time < 6.0):
                    is_dynamic = True
                    target_speed = max(self.last_target_speed, 0.20)
                    target_heading = self.last_target_heading

                self.prev_obstacle_coords = (obs_x, obs_y)
                self.prev_obstacle_time = now_t
                self.last_target_speed = target_speed
                self.last_target_heading = target_heading

                # Check if this corridor obstacle is at/near the permanent human intruder (3.8, 0.5)
                is_near_human_intruder = (
                    self.arena_type == "yard"
                    and math.hypot(corridor_obstacle_coords[0] - 3.8, corridor_obstacle_coords[1] - 0.5) < 0.95
                )
                if is_near_human_intruder:
                    self.is_target_dynamic = False
                    primary_target = "Human Intruder (Standing Person)"
                    top_conf = 0.95
                elif is_dynamic:
                    self.is_target_dynamic = True
                    primary_target = f"Secondary Rover (Dynamic: {target_speed:.2f} m/s)"
                    top_conf = 0.98
                else:
                    self.is_target_dynamic = False
                    primary_target = "Placed Obstacle (Yard Platform)" if self.arena_type == "yard" else "Placed Obstacle (Arena Floor)"
                    top_conf = 0.92

                target_coords = corridor_obstacle_coords
                target_dist = corridor_obstacle_dist

                # Generate synthetic visual bounding box in camera display
                box_cx = int((0.5 + corridor_obstacle_rel_ang / math.radians(60.0)) * w)
                box_w = max(int(90.0 / target_dist), 40)
                box_h = max(int(70.0 / target_dist), 35)
                box_y = int(h * 0.72)
                det_box = [
                    max(box_cx - box_w // 2, 5),
                    max(box_y - box_h, 10),
                    min(box_cx + box_w // 2, w - 5),
                    min(box_y + box_h // 2, h - 5),
                ]
                detections.append({
                    "class_name": primary_target,
                    "confidence": top_conf,
                    "bbox": det_box,
                    "foot_point": (box_cx, box_y),
                    "inside_geofence": True,
                    "source": "Corridor LiDAR Sentinel",
                })
            else:
                self.is_target_dynamic = False

            # Check operator bypass grace period
            bypass_active = (time.time() < self.resume_override_until)
            if bypass_active:
                geofence_breach = False

            # 7. Publish Alert & Geofence Status
            if not rclpy.ok():
                return
            # Dual-Condition Logic:
            # - Dynamic Condition (Moving Secondary Rover): Do NOT halt rover, let Nav2 MPPI dynamically steer around it!
            # - Static Condition (Human Intruder / Stationary Shapes): HALT rover immediately, sound alert, and hold position!
            is_dynamic_rover = ("Secondary Rover" in primary_target and self.is_target_dynamic)
            if self.avoidance_mode and is_dynamic_rover:
                halt_rover = False
            else:
                halt_rover = geofence_breach

            alert_msg = Bool(data=halt_rover)
            self.alert_pub.publish(alert_msg)
            self.intrusion_pub.publish(alert_msg)
            self.dynamic_obstacle_pub.publish(Bool(data=self.is_target_dynamic))

            # Publish dynamic obstacle spatial telemetry [x, y, speed, distance, is_dynamic]
            dyn_data_msg = Float32MultiArray()
            if is_dynamic_rover:
                dyn_data_msg.data = [
                    float(target_coords[0]),
                    float(target_coords[1]),
                    float(target_speed),
                    float(target_dist),
                    1.0
                ]
            else:
                dyn_data_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.dynamic_obstacle_data_pub.publish(dyn_data_msg)

            status_str = String()
            if geofence_breach:
                status_str.data = "GEOFENCE_BREACH"
            elif detections:
                status_str.data = "SAFE_ACTIVITY_OUTSIDE_GEOFENCE"
            else:
                status_str.data = "SECURE"
            self.geofence_status_pub.publish(status_str)

            # 8. Detection Notifications & Event Storage Updates
            now_dt = datetime.datetime.now()
            time_str = now_dt.strftime("%H:%M:%S")
            date_str = now_dt.strftime("%Y-%m-%d")
            iso_str = now_dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            if geofence_breach:
                self.last_detection_time_str = time_str
                self.last_detection_date_str = date_str
                self.last_target_name = primary_target
                self.last_target_pos = target_coords
                self.last_target_dist = target_dist

                if not self.breach_active:
                    self.breach_active = True
                    self.detection_counter += 1

                    # A. Loud Terminal Alert
                    status_text = "Rover actively avoiding obstacle (Move Aside & Yield protocol)." if (self.avoidance_mode and is_dynamic_rover) else "Rover halted. Holding security perimeter."
                    resume_text = "   Autonomous maneuver active; maintaining perimeter route.\n" if (self.avoidance_mode and is_dynamic_rover) else (
                        "   To Resume: Press 'R' in the window, or run:\n"
                        "              ros2 topic pub --once /perimeter/resume std_msgs/msg/Bool \"{data: true}\"\n"
                    )
                    self.get_logger().error(
                        f"\n{'='*75}\n"
                        f"🚨 [PERIMETER SECURITY ALERT] DETECTION #{self.detection_counter} AT {time_str} ({date_str})!\n"
                        f"   Target: {primary_target}\n"
                        f"   Confidence: {top_conf * 100:.1f}%\n"
                        f"   Rover Position: (x={rx:.2f}, y={ry:.2f}, yaw={ryaw:.2f} rad)\n"
                        f"   Target Coordinates: (x={target_coords[0]:.2f}, y={target_coords[1]:.2f}) | Distance: {target_dist:.2f}m\n"
                        f"   Status: {status_text}\n"
                        f"{resume_text}"
                        f"{'='*75}"
                    )

                    # B. Native Linux Desktop Notification Pop-up (Auto-dismissing, replace in-place)
                    self._send_desktop_notification(primary_target, target_coords, target_dist, time_str)

                    # C. Store Structured Event in perimeter_detections.json
                    self._record_json_event(
                        event_id=self.detection_counter,
                        iso_str=iso_str,
                        time_str=time_str,
                        date_str=date_str,
                        target=primary_target,
                        conf=top_conf,
                        rx=rx, ry=ry, ryaw=ryaw,
                        tx=target_coords[0], ty=target_coords[1],
                        dist=target_dist,
                    )

                    # D. Append human-readable log
                    self._append_event_log(detections, target_coords, target_dist)

            else:
                if self.breach_active:
                    self.breach_active = False
                    self.get_logger().info(f"[{time_str}] 🟢 Perimeter clear; yard ground secure.")
                    # Automatically dismiss desktop notifications as soon as perimeter is clear
                    self._dismiss_desktop_notification()

            # 9. Annotate Camera Video Stream
            display_frame = cv_image.copy()

            # Translucent Geofence Zone Overlay
            if self.enable_geofence:
                overlay = display_frame.copy()
                zone_color = (0, 0, 180) if geofence_breach else (0, 180, 240)
                cv2.fillPoly(overlay, [geofence_polygon], zone_color)
                cv2.addWeighted(overlay, 0.20, display_frame, 0.80, 0, display_frame)
                cv2.polylines(display_frame, [geofence_polygon], isClosed=True, color=(0, 0, 255) if geofence_breach else (255, 200, 0), thickness=2)

            # Draw Object Bounding Boxes (hide during bypass)
            if not bypass_active:
                for det in detections:
                    x1, y1, x2, y2 = [int(c) for c in det["bbox"]]
                    inside = det["inside_geofence"]
                    box_color = (0, 0, 255) if inside else (0, 255, 255)
                    tag = f"ALERT: {det['class_name']} ({det['confidence']})" if inside else f"SAFE: {det['class_name']} ({det['confidence']})"

                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), box_color, 2)
                    cv2.putText(
                        display_frame,
                        tag,
                        (x1, max(y1 - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.44,
                        box_color,
                        2,
                        cv2.LINE_AA,
                    )

            # Top Camera Banner
            cv2.rectangle(display_frame, (0, 0), (w, 32), (30, 30, 30), -1)
            if bypass_active:
                banner_text = "STATUS: PATROL RESUMED (OPERATOR BYPASS ACTIVE)"
                banner_color = (0, 200, 255)
            elif geofence_breach:
                if self.is_target_dynamic and self.avoidance_mode:
                    banner_text = f"ALERT: AVOIDING {self.last_target_name.upper()} (ACTIVE REPLANNING)"
                    banner_color = (0, 180, 255)
                else:
                    banner_text = f"ALERT: {self.last_target_name.upper()} DETECTED AT {self.last_detection_time_str}!"
                    banner_color = (0, 0, 255)
            elif detections:
                banner_text = "STATUS: SAFE ACTIVITY OUTSIDE GEOFENCE"
                banner_color = (0, 220, 255)
            else:
                banner_text = "STATUS: OUTDOOR YARD CLEAR (GEOFENCE SECURE)"
                banner_color = (0, 255, 0)

            cv2.putText(display_frame, banner_text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, banner_color, 2, cv2.LINE_AA)

            # 9B. Tactical Heading Indicator (Top-Right Widget showing robot movement direction arrow)
            card_w, card_h = 114, 110
            card_x1 = w - card_w - 10
            card_y1 = 40
            card_x2 = card_x1 + card_w
            card_y2 = card_y1 + card_h

            # Semi-transparent HUD card background
            hud_card = display_frame.copy()
            cv2.rectangle(hud_card, (card_x1, card_y1), (card_x2, card_y2), (18, 22, 26), -1)
            cv2.addWeighted(hud_card, 0.70, display_frame, 0.30, 0, display_frame)
            cv2.rectangle(display_frame, (card_x1, card_y1), (card_x2, card_y2), (55, 68, 78), 1)

            # Header
            cv2.putText(display_frame, "ROVER HEADING", (card_x1 + 10, card_y1 + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, (180, 220, 180), 1, cv2.LINE_AA)

            # Circular compass dial with cardinal markers
            dial_cx = (card_x1 + card_x2) // 2
            dial_cy = card_y1 + 54
            dial_r = 26
            cv2.circle(display_frame, (dial_cx, dial_cy), dial_r, (65, 75, 85), 1)
            cv2.line(display_frame, (dial_cx - dial_r, dial_cy), (dial_cx + dial_r, dial_cy), (45, 55, 65), 1)
            cv2.line(display_frame, (dial_cx, dial_cy - dial_r), (dial_cx, dial_cy + dial_r), (45, 55, 65), 1)

            # Cardinal letters (N=North, S=South, E=East, W=West)
            cv2.putText(display_frame, "N", (dial_cx - 4, dial_cy - dial_r + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (0, 220, 255), 1, cv2.LINE_AA)
            cv2.putText(display_frame, "S", (dial_cx - 3, dial_cy + dial_r - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.26, (140, 140, 140), 1, cv2.LINE_AA)
            cv2.putText(display_frame, "E", (dial_cx + dial_r - 8, dial_cy + 3), cv2.FONT_HERSHEY_SIMPLEX, 0.26, (140, 140, 140), 1, cv2.LINE_AA)
            cv2.putText(display_frame, "W", (dial_cx - dial_r + 2, dial_cy + 3), cv2.FONT_HERSHEY_SIMPLEX, 0.26, (140, 140, 140), 1, cv2.LINE_AA)

            # Rover beacon & Green Arrow mark pointing direction of movement (matching top view monitor!)
            cv2.circle(display_frame, (dial_cx, dial_cy), 8, (0, 220, 255), 2)
            cv2.circle(display_frame, (dial_cx, dial_cy), 3, (0, 255, 120), -1)
            arr_len = 20
            tip_x = dial_cx + int(arr_len * math.cos(ryaw))
            tip_y = dial_cy - int(arr_len * math.sin(ryaw))
            cv2.arrowedLine(display_frame, (dial_cx, dial_cy), (tip_x, tip_y), (0, 255, 120), 2, tipLength=0.35)

            # Degree & Cardinal readout
            cardinal = self._heading_to_cardinal(ryaw)
            deg_val = math.degrees(ryaw)
            deg_str = f"{deg_val:+.0f}° [{cardinal}]"
            t_sz = cv2.getTextSize(deg_str, cv2.FONT_HERSHEY_SIMPLEX, 0.36, 1)[0]
            cv2.putText(display_frame, deg_str, (dial_cx - t_sz[0] // 2, card_y2 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 255, 120), 1, cv2.LINE_AA)

            # 9C. Dynamic Movement Ribbon Badge (Bottom HUD above footer)
            badge_w = 290
            bx1 = (w - badge_w) // 2
            by1 = h - 54
            bx2 = bx1 + badge_w
            by2 = h - 28
            hud_sub = display_frame.copy()
            cv2.rectangle(hud_sub, (bx1, by1), (bx2, by2), (18, 22, 26), -1)
            cv2.addWeighted(hud_sub, 0.70, display_frame, 0.30, 0, display_frame)
            cv2.rectangle(display_frame, (bx1, by1), (bx2, by2), (60, 75, 85), 1)

            # Movement direction arrow mark
            is_halted = (geofence_breach and not bypass_active)
            move_status = "HOLDING POSITION" if is_halted else "PATROL ADVANCING"
            move_color = (0, 0, 255) if is_halted else (0, 255, 120)

            arr_start = (bx1 + 14, by1 + 13)
            arr_end = (bx1 + 38, by1 + 13)
            cv2.arrowedLine(display_frame, arr_start, arr_end, move_color, 2, tipLength=0.4)
            dir_text = f"DIR: {cardinal} ({deg_val:+.0f}°) | {move_status}"
            cv2.putText(display_frame, dir_text, (bx1 + 46, by1 + 17),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, move_color, 1, cv2.LINE_AA)

            # Bottom Camera Ribbon
            cv2.rectangle(display_frame, (0, h - 22), (w, h), (20, 20, 20), -1)
            if self.last_detection_time_str != "None":
                footer_text = f"Last Detection: {self.last_target_name} at {self.last_detection_time_str} | Press 'R' to Resume Patrol"
            else:
                footer_text = "Surveillance Active | Press 'R' to Resume Patrol"
            cv2.putText(display_frame, footer_text, (10, h - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200, 200, 200), 1, cv2.LINE_AA)

            # Publish annotated camera stream
            annotated_msg = self._cv2_to_image_msg(display_frame, header=msg.header, encoding="bgr8")
            if rclpy.ok():
                self.annotated_pub.publish(annotated_msg)

            # 10. Render Top View Tactical Radar Window
            top_view_canvas = self._render_tactical_top_view(
                rx=rx, ry=ry, ryaw=ryaw,
                geofence_breach=geofence_breach,
                bypass_active=bypass_active,
                target_name=self.last_target_name,
                target_coords=self.last_target_pos,
                target_dist=self.last_target_dist,
                time_str=self.last_detection_time_str,
                current_time=time_str,
                is_dynamic=self.is_target_dynamic,
                target_speed=self.last_target_speed,
                target_heading=self.last_target_heading,
            )

            # 11. GUI Display & Interactive Controls
            if self.show_preview and self.display_available:
                try:
                    cv2.imshow("Autonomous Perimeter Rover - Surveillance Stream", display_frame)
                    cv2.imshow("Autonomous Perimeter Rover - Top View Tactical Monitor", top_view_canvas)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord('r'), ord('R')):
                        self.resume_override_until = time.time() + 35.0
                        self.breach_active = False
                        self._dismiss_desktop_notification()
                        self.resume_pub.publish(Bool(data=True))
                        self.get_logger().info("Operator pressed 'R': Dispatched patrol resume command (35s bypass enabled).")
                except Exception:
                    self.display_available = False

        except Exception as e:
            if rclpy.ok():
                self.get_logger().error(f"Error processing frame: {e}")

    def _render_tactical_top_view(self, rx, ry, ryaw, geofence_breach, bypass_active, target_name, target_coords, target_dist, time_str, current_time, is_dynamic=False, target_speed=0.0, target_heading=0.0):
        # 650 high x 600 wide tactical radar canvas
        canvas = np.zeros((650, 600, 3), dtype=np.uint8)
        canvas[:] = (20, 24, 28)

        # Draw base tactical yard map (520x520) at offset (40, 55)
        if self.tactical_base_map is not None:
            canvas[55:575, 40:560] = self.tactical_base_map.copy()

        # Draw Waypoint Patrol Route
        wps = self.patrol_waypoints
        for i in range(len(wps)):
            p1 = self._world_to_canvas(*wps[i])
            p2 = self._world_to_canvas(*wps[(i + 1) % len(wps)])
            cv2.line(canvas, p1, p2, (80, 80, 80), 1, cv2.LINE_AA)
            cv2.circle(canvas, p1, 4, (200, 180, 0), -1)
            cv2.putText(canvas, f"W{i+1}", (p1[0] + 5, p1[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (200, 180, 0), 1)

        # Draw Rover Beacon & FOV Sensor Cone
        ru, rv = self._world_to_canvas(rx, ry)
        cone_len = 65
        half_ang = 0.52  # ~30 degrees
        p_c1 = (ru, rv)
        p_c2 = (ru + int(cone_len * math.cos(ryaw - half_ang)), rv - int(cone_len * math.sin(ryaw - half_ang)))
        p_c3 = (ru + int(cone_len * math.cos(ryaw + half_ang)), rv - int(cone_len * math.sin(ryaw + half_ang)))

        # Translucent sensor cone overlay
        cone_overlay = canvas.copy()
        cone_color = (0, 0, 220) if geofence_breach else ((0, 200, 240) if bypass_active else (0, 255, 120))
        cv2.fillPoly(cone_overlay, [np.array([p_c1, p_c2, p_c3], np.int32)], cone_color)
        cv2.addWeighted(cone_overlay, 0.18, canvas, 0.82, 0, canvas)
        cv2.polylines(canvas, [np.array([p_c1, p_c2, p_c3], np.int32)], isClosed=True, color=cone_color, thickness=1)

        # Rover body & heading arrow
        cv2.circle(canvas, (ru, rv), 10, (0, 220, 255), 2)
        cv2.circle(canvas, (ru, rv), 4, (0, 255, 120), -1)
        tip_x = ru + int(20 * math.cos(ryaw))
        tip_y = rv - int(20 * math.sin(ryaw))
        cv2.arrowedLine(canvas, (ru, rv), (tip_x, tip_y), (0, 255, 120), 2, tipLength=0.3)

        # Draw Detected Object Target Reticle
        if geofence_breach and target_name != "None":
            tu, tv = self._world_to_canvas(*target_coords)

            # Line of sight from rover to target
            cv2.line(canvas, (ru, rv), (tu, tv), (0, 0, 255), 1, cv2.LINE_AA)

            # Pulsating radar reticle
            pulse = int((self.frame_count % 15) * 0.8)
            cv2.line(canvas, (tu - 16, tv), (tu + 16, tv), (0, 0, 255), 2)
            cv2.line(canvas, (tu, tv - 16), (tu, tv + 16), (0, 0, 255), 2)
            cv2.circle(canvas, (tu, tv), 8 + pulse, (0, 0, 255), 1)
            cv2.circle(canvas, (tu, tv), 18 + pulse, (0, 0, 255), 1)

            # If moving dynamic obstacle, draw motion vector arrow
            if is_dynamic:
                vec_len = min(max(int(target_speed * 120), 25), 55)
                tip_tu = tu + int(vec_len * math.cos(target_heading))
                tip_tv = tv - int(vec_len * math.sin(target_heading))
                cv2.arrowedLine(canvas, (tu, tv), (tip_tu, tip_tv), (0, 255, 255), 2, tipLength=0.35)

            # Target annotation label box
            lbl_x = min(tu + 14, 400)
            lbl_y = max(tv, 85)
            box_h = 44 if is_dynamic else 36
            cv2.rectangle(canvas, (lbl_x, lbl_y - 20), (lbl_x + 185, lbl_y - 20 + box_h), (25, 25, 25), -1)
            cv2.rectangle(canvas, (lbl_x, lbl_y - 20), (lbl_x + 185, lbl_y - 20 + box_h), (0, 220, 255) if is_dynamic else (0, 0, 255), 1)
            cv2.putText(canvas, f"TARGET: {target_name[:20]}", (lbl_x + 5, lbl_y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 220, 255) if is_dynamic else (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(canvas, f"Pos: ({target_coords[0]:.1f}, {target_coords[1]:.1f}) | {time_str}", (lbl_x + 5, lbl_y + 9), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (200, 200, 200), 1, cv2.LINE_AA)
            if is_dynamic:
                cv2.putText(canvas, f"SPEED: {target_speed:.2f} m/s (MOVING)", (lbl_x + 5, lbl_y + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (0, 255, 255), 1, cv2.LINE_AA)

        # Tactical Header Ribbon
        if bypass_active:
            cv2.rectangle(canvas, (0, 0), (600, 50), (35, 45, 10), -1)
            cv2.putText(canvas, "TACTICAL RADAR: PATROL RESUMED (BYPASS ACTIVE)", (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 220, 255), 2, cv2.LINE_AA)
            cv2.putText(canvas, f"Rover: ({rx:.2f}, {ry:.2f}, {math.degrees(ryaw):.0f} deg) | Time: {current_time}", (12, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 220, 180), 1, cv2.LINE_AA)
        elif geofence_breach:
            if is_dynamic and self.avoidance_mode:
                cv2.rectangle(canvas, (0, 0), (600, 50), (45, 35, 10), -1)
                cv2.putText(canvas, f"⚡ [DYNAMIC AVOIDANCE] {target_name.upper()}", (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 220, 255), 2, cv2.LINE_AA)
                cv2.putText(canvas, f"AVOIDING ROVER | MPPI Active Path Replanning | Speed: {target_speed:.2f} m/s", (12, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200, 240, 220), 1, cv2.LINE_AA)
            else:
                cv2.rectangle(canvas, (0, 0), (600, 50), (0, 0, 160), -1)
                cv2.putText(canvas, f"🚨 [TOP VIEW ALERT] {target_name.upper()} DETECTED AT {time_str}!", (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(canvas, f"ROVER HALTED | Target: ({target_coords[0]:.2f}, {target_coords[1]:.2f}) | Dist: {target_dist:.2f}m", (12, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (230, 230, 230), 1, cv2.LINE_AA)
        else:
            cv2.rectangle(canvas, (0, 0), (600, 50), (25, 45, 25), -1)
            cv2.putText(canvas, "TACTICAL RADAR: PATROL ACTIVE | PERIMETER SECURE", (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 100), 2, cv2.LINE_AA)
            cv2.putText(canvas, f"Rover: ({rx:.2f}, {ry:.2f}, {math.degrees(ryaw):.0f} deg) | Time: {current_time}", (12, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 220, 180), 1, cv2.LINE_AA)

        # Tactical Footer Ribbon
        cv2.rectangle(canvas, (0, 605), (600, 650), (20, 20, 20), -1)
        footer = "Controls: Press 'R' in window to Resume Patrol | Detections auto-stored to JSON"
        cv2.putText(canvas, footer, (12, 632), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (180, 180, 180), 1, cv2.LINE_AA)

        return canvas

    def _record_json_event(self, event_id, iso_str, time_str, date_str, target, conf, rx, ry, ryaw, tx, ty, dist):
        record = {
            "event_id": event_id,
            "timestamp": iso_str,
            "time": time_str,
            "date": date_str,
            "target": target,
            "confidence": round(conf, 2),
            "rover_position": {
                "x": round(rx, 2),
                "y": round(ry, 2),
                "yaw_degrees": round(math.degrees(ryaw), 1),
            },
            "target_coordinates": {
                "x": round(tx, 2),
                "y": round(ty, 2),
            },
            "distance_meters": round(dist, 2),
            "status": "DETECTED_AND_HALTED",
        }

        records = []
        try:
            if os.path.exists(self.json_log_path):
                with open(self.json_log_path, "r") as f:
                    data = json.load(f)
                    if isinstance(data, list):
                        records = data
        except Exception:
            records = []

        records.append(record)

        try:
            with open(self.json_log_path, "w") as f:
                json.dump(records, f, indent=2)
            self.get_logger().info(f"Event #{event_id} appended to {self.json_log_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write JSON event log: {e}")

    def _append_event_log(self, detections, target_coords, target_dist):
        try:
            with open(self.log_file_path, "a") as f:
                ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                summary = ", ".join([f"{d['class_name']}({d['confidence']})" for d in detections])
                f.write(
                    f"[{ts}] PERIMETER_ALERT: {summary} | "
                    f"Target Pos: ({target_coords[0]:.2f}, {target_coords[1]:.2f}) | "
                    f"Dist: {target_dist:.2f}m\n"
                )
        except Exception:
            pass

    def _send_desktop_notification(self, primary_target, target_coords, target_dist, time_str):
        """Dispatches an auto-dismissing, in-place desktop notification that expires in 4 seconds."""
        try:
            subprocess.Popen([
                "notify-send",
                "-r", str(self.notification_id),  # Replaces previous alert in-place (no notification stacking)
                "-t", "4000",                     # 4-second auto-expiration timeout
                "-e",                             # Transient (does not persist in system notification tray)
                "-u", "normal",                   # Normal priority (allows desktop auto-dismissal)
                "-a", "Autonomous Perimeter Rover",
                "-i", "security-high",
                f"🚨 PERIMETER ALERT [{time_str}]",
                f"Target: {primary_target}\nLocation: ({target_coords[0]:.1f}, {target_coords[1]:.1f}) | Dist: {target_dist:.1f}m\nRover Halted.",
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

    def _dismiss_desktop_notification(self):
        """Immediately dismisses active desktop notifications via D-Bus."""
        try:
            subprocess.run([
                "gdbus", "call", "--session",
                "--dest", "org.freedesktop.Notifications",
                "--object-path", "/org/freedesktop/Notifications",
                "--method", "org.freedesktop.Notifications.CloseNotification",
                str(self.notification_id),
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=0.5)
        except Exception:
            pass

    def destroy_node(self):
        self._dismiss_desktop_notification()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SurveillanceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._dismiss_desktop_notification()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
