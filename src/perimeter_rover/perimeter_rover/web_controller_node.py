#!/usr/bin/env python3
"""
Autonomous Perimeter Rover - Mission Control & Teleoperation Server Node
Provides a responsive Web Dashboard with:
1. Dual Active Control: Autonomous Patrol Mode vs. Manual Teleop Mode
2. Virtual Analog Joystick & Keyboard WASD Steering
3. Emergency Stop (E-Stop) & Origin Reset (returns rover to origin & starts surveillance)
4. Real-time Telemetry: Linear & Angular Velocities of Primary & Secondary Rovers
5. Live Detection Event Log Table (auto-updating with Date, Time, Distance, Target)
6. Live Surveillance Video Stream (MJPEG)
"""

import json
import math
import os
import subprocess
import sys
import threading
import time
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from flask import Flask, Response, jsonify, render_template, request
from flask_cors import CORS
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray, String


class WebControllerNode(Node):
    def __init__(self):
        super().__init__("web_controller_node")

        # Telemetry & State
        self.control_mode = "autonomous"    # "autonomous" | "manual"
        self.emergency_stop = False
        self.primary_linear_vel = 0.0
        self.primary_angular_vel = 0.0
        self.primary_x = 0.0
        self.primary_y = -4.5
        self.primary_yaw = 0.0
        self.primary_yaw_deg = 0.0
        self.primary_cardinal = "E"

        self.secondary_speed = 0.25
        self.secondary_x = 2.2
        self.secondary_y = -4.15
        self.secondary_dist = 0.0
        self.secondary_is_dynamic = True
        self.active_waypoint = 1

        # Teleop Commands from Web Joystick
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_teleop_time = 0.0

        # Video Frame Buffer
        self.bridge = CvBridge()
        self.latest_jpeg = None
        self.frame_lock = threading.Lock()

        # Detections JSON path
        self.json_path = "/home/purshotam/autonomous-perimeter-rover/perimeter_detections.json"

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.control_mode_pub = self.create_publisher(String, "/perimeter/control_mode", 10)
        self.emergency_stop_pub = self.create_publisher(Bool, "/perimeter/emergency_stop", 10)
        self.reset_to_origin_pub = self.create_publisher(Bool, "/perimeter/reset_to_origin", 10)
        self.resume_pub = self.create_publisher(Bool, "/perimeter/resume", 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        # Costmap clearing clients
        self.clear_local_srv = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap"
        )
        self.clear_global_srv = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap"
        )

        # Subscribers
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.create_subscription(
            Float32MultiArray, "/perimeter/dynamic_obstacle_data", self._dynamic_obs_callback, 10
        )
        self.create_subscription(Image, "/perimeter/annotated_image", self._image_callback, 10)
        self.create_subscription(Image, "/camera/image_raw", self._raw_image_callback, 10)
        self.create_subscription(String, "/perimeter/control_mode", self._mode_callback, 10)
        self.create_subscription(Bool, "/perimeter/emergency_stop", self._estop_callback, 10)

        # 20 Hz Teleoperation Dispatch Timer
        self.create_timer(0.05, self._teleop_loop)

        self.get_logger().info("🌐 Web Controller Node initialized.")

    def _odom_callback(self, msg: Odometry):
        # Velocities
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.primary_linear_vel = math.hypot(vx, vy)
        self.primary_angular_vel = msg.twist.twist.angular.z

        # Position
        self.primary_x = msg.pose.pose.position.x
        self.primary_y = msg.pose.pose.position.y

        # Heading Yaw from Quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.primary_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.primary_yaw_deg = math.degrees(self.primary_yaw)
        self.primary_cardinal = self._heading_to_cardinal(self.primary_yaw)

        # Estimate closest forward waypoint
        self.active_waypoint = self._estimate_waypoint(self.primary_x, self.primary_y)

    def _dynamic_obs_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 5:
            self.secondary_x = float(msg.data[0])
            self.secondary_y = float(msg.data[1])
            self.secondary_speed = float(msg.data[2])
            self.secondary_dist = float(msg.data[3])
            self.secondary_is_dynamic = bool(msg.data[4] > 0.5)

    def _image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ret, jpeg = cv2.imencode(".jpg", cv_img, [cv2.IMWRITE_JPEG_QUALITY, 72])
            if ret:
                with self.frame_lock:
                    self.latest_jpeg = jpeg.tobytes()
        except Exception:
            pass

    def _raw_image_callback(self, msg: Image):
        # Fallback if annotated image not published yet
        with self.frame_lock:
            if self.latest_jpeg is not None:
                return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ret, jpeg = cv2.imencode(".jpg", cv_img, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if ret:
                with self.frame_lock:
                    self.latest_jpeg = jpeg.tobytes()
        except Exception:
            pass

    def _mode_callback(self, msg: String):
        self.control_mode = msg.data.strip().lower()

    def _estop_callback(self, msg: Bool):
        self.emergency_stop = msg.data

    def _teleop_loop(self):
        # If in manual mode and not emergency stopped, publish commanded velocity
        if self.control_mode == "manual" and not self.emergency_stop:
            if time.time() - self.last_teleop_time < 0.40:
                twist = Twist()
                twist.linear.x = float(self.target_linear)
                twist.angular.z = float(self.target_angular)
                self.cmd_vel_pub.publish(twist)
            else:
                # Deadman brake: stop robot if no commands received recently
                self.cmd_vel_pub.publish(Twist())
        elif self.emergency_stop:
            self.cmd_vel_pub.publish(Twist())

    def _heading_to_cardinal(self, yaw_rad):
        deg = math.degrees(yaw_rad) % 360
        dirs = ["E", "ENE", "NE", "NNE", "N", "NNW", "NW", "WNW", "W", "WSW", "SW", "SSW", "S", "SSE", "SE", "ESE"]
        idx = int((deg + 11.25) / 22.5) % 16
        return dirs[idx]

    def _estimate_waypoint(self, x, y):
        # Find closest forward waypoint ahead in yard perimeter
        if y <= -3.0 and x < 2.8:
            return 1
        elif x >= 2.0 and y < 3.8:
            return 2
        elif y >= 3.0 and x > -2.8:
            return 3
        elif x <= -2.0 and y > -1.0:
            return 4
        elif x <= -1.5 and -3.5 < y <= -1.0:
            return 5
        else:
            return 6

    def clear_costmaps_async(self):
        """Asynchronously call ClearEntireCostmap for local and global costmaps."""
        try:
            if self.clear_local_srv.service_is_ready():
                self.clear_local_srv.call_async(ClearEntireCostmap.Request())
            if self.clear_global_srv.service_is_ready():
                self.clear_global_srv.call_async(ClearEntireCostmap.Request())
        except Exception as e:
            self.get_logger().warn(f"Clear costmaps exception: {e}")

    def reset_gazebo_model_pose(self, x=0.0, y=-4.50, z=0.01, yaw=0.0):
        """
        Resets Gazebo model pose of turtlebot3_waffle back to origin coordinates.
        Uses gz.transport13 Python API with fallback to gz service CLI.
        """
        success = False
        # Method 1: gz.transport13 Python bindings
        try:
            import gz.transport13 as gz_transport
            import gz.msgs10.pose_pb2 as pose_pb2
            import gz.msgs10.boolean_pb2 as boolean_pb2

            gz_node = gz_transport.Node()
            req = pose_pb2.Pose()
            req.name = "turtlebot3_waffle"
            req.position.x = float(x)
            req.position.y = float(y)
            req.position.z = float(z)
            req.orientation.z = float(math.sin(yaw / 2.0))
            req.orientation.w = float(math.cos(yaw / 2.0))
            res, _ = gz_node.request("/world/default/set_pose", req, pose_pb2.Pose, boolean_pb2.Boolean, 1500)
            if res:
                self.get_logger().info("✅ Gazebo model pose reset via gz.transport13.")
                success = True
        except Exception as e:
            self.get_logger().warn(f"gz.transport13 set_pose exception: {e}")

        # Method 2: CLI fallback
        if not success:
            try:
                sin_z = math.sin(yaw / 2.0)
                cos_w = math.cos(yaw / 2.0)
                cmd = [
                    "gz", "service", "-s", "/world/default/set_pose",
                    "--reqtype", "gz.msgs.Pose",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "1500",
                    "--req", f'name: "turtlebot3_waffle", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{z: {sin_z}, w: {cos_w}}}'
                ]
                subprocess.run(cmd, timeout=3.0, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self.get_logger().info("✅ Gazebo model pose reset via gz service CLI.")
                success = True
            except Exception as e:
                self.get_logger().warn(f"gz service CLI set_pose failed: {e}")

        return success

    def set_amcl_initial_pose(self, x=0.0, y=-4.50, yaw=0.0):
        """Broadcast initialpose message to AMCL to immediately lock filter at origin."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.01
        msg.pose.pose.orientation.z = float(math.sin(yaw / 2.0))
        msg.pose.pose.orientation.w = float(math.cos(yaw / 2.0))
        cov = [0.0] * 36
        cov[0] = 0.05   # x
        cov[7] = 0.05   # y
        cov[35] = 0.05  # yaw
        msg.pose.covariance = cov
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f"📍 AMCL /initialpose published at ({x:.2f}, {y:.2f}, yaw={yaw:.2f}).")

    def execute_total_reset_to_origin(self):
        """
        Full routine to reset rover to origin spawn position (0.0, -4.50) in Gazebo & AMCL,
        clear costmaps, and commence autonomous surveillance patrol.
        """
        self.get_logger().info("🔄 Executing Total System Reset to Origin (0.0, -4.50)...")

        # 1. Stop rover velocities
        self.cmd_vel_pub.publish(Twist())
        self.emergency_stop = False
        self.control_mode = "autonomous"

        # 2. Reset Gazebo model pose
        self.reset_gazebo_model_pose(x=0.0, y=-4.50, z=0.01, yaw=0.0)

        # 3. Synchronize AMCL initial pose
        self.set_amcl_initial_pose(x=0.0, y=-4.50, yaw=0.0)

        # 4. Clear costmaps
        self.clear_costmaps_async()

        # 5. Broadcast reset signals to all nodes
        self.emergency_stop_pub.publish(Bool(data=False))
        self.control_mode_pub.publish(String(data="autonomous"))
        self.reset_to_origin_pub.publish(Bool(data=True))
        self.resume_pub.publish(Bool(data=True))

        self.get_logger().info("✅ Total System Reset complete: Rover at Origin (0.0, -4.50) & Autonomous Surveillance Active!")


def create_flask_app(node: WebControllerNode):
    template_dir = os.path.join(os.path.dirname(__file__), "templates")
    app = Flask(__name__, template_folder=template_dir)
    CORS(app)

    @app.route("/")
    def index():
        return render_template("index.html")

    @app.route("/api/telemetry")
    def get_telemetry():
        return jsonify({
            "mode": node.control_mode,
            "emergency_stop": node.emergency_stop,
            "primary_rover": {
                "linear_velocity": round(node.primary_linear_vel, 2),
                "angular_velocity": round(node.primary_angular_vel, 2),
                "x": round(node.primary_x, 2),
                "y": round(node.primary_y, 2),
                "yaw_deg": round(node.primary_yaw_deg, 1),
                "cardinal": node.primary_cardinal,
            },
            "secondary_rover": {
                "linear_velocity": round(node.secondary_speed, 2),
                "x": round(node.secondary_x, 2),
                "y": round(node.secondary_y, 2),
                "distance": round(node.secondary_dist, 2),
                "is_dynamic": node.secondary_is_dynamic,
            },
            "active_waypoint": node.active_waypoint,
            "timestamp": datetime.now().strftime("%H:%M:%S")
        })

    @app.route("/api/mode", methods=["POST"])
    def set_mode():
        data = request.get_json(silent=True) or {}
        mode = str(data.get("mode", "autonomous")).strip().lower()
        if mode in ["autonomous", "manual"]:
            node.control_mode = mode
            node.control_mode_pub.publish(String(data=mode))
            if mode == "manual":
                node.cmd_vel_pub.publish(Twist())
            elif mode == "autonomous":
                # Clear lingering intrusions and flush costmaps so patrol resumes immediately
                node.resume_pub.publish(Bool(data=True))
                node.clear_costmaps_async()
            return jsonify({"status": "ok", "mode": mode})
        return jsonify({"status": "error", "message": "Invalid mode"}), 400

    @app.route("/api/teleop", methods=["POST"])
    def set_teleop():
        if node.emergency_stop:
            return jsonify({"status": "error", "message": "E-Stop active"}), 403
        data = request.get_json(silent=True) or {}
        try:
            node.target_linear = max(min(float(data.get("linear", 0.0)), 0.28), -0.28)
            node.target_angular = max(min(float(data.get("angular", 0.0)), 1.2), -1.2)
            node.last_teleop_time = time.time()
            return jsonify({"status": "ok", "linear": node.target_linear, "angular": node.target_angular})
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 400

    @app.route("/api/estop", methods=["POST"])
    def trigger_estop():
        data = request.get_json(silent=True) or {}
        stop = bool(data.get("stop", True))
        if stop:
            # Immediate Emergency Stop & Brake
            node.emergency_stop = True
            node.emergency_stop_pub.publish(Bool(data=True))
            node.cmd_vel_pub.publish(Twist())
            node.clear_costmaps_async()
            return jsonify({"status": "ok", "emergency_stop": True, "action": "halted"})
        else:
            # Releasing E-Stop: Teleport rover to origin and start autonomous surveillance patrol!
            node.execute_total_reset_to_origin()
            return jsonify({
                "status": "ok",
                "emergency_stop": False,
                "action": "reset_to_origin_and_patrol",
                "message": "Rover reset to Origin (0.0, -4.50) & Autonomous Surveillance Started"
            })

    @app.route("/api/reset_to_origin", methods=["POST"])
    def reset_to_origin():
        node.execute_total_reset_to_origin()
        return jsonify({
            "status": "ok",
            "message": "Rover reset to Origin (0.0, -4.50) & Autonomous Surveillance Started"
        })

    @app.route("/api/resume", methods=["POST"])
    def trigger_resume():
        node.resume_pub.publish(Bool(data=True))
        node.clear_costmaps_async()
        return jsonify({"status": "ok", "message": "Resume command published with 35s grace bypass"})

    @app.route("/api/clear_costmaps", methods=["POST"])
    def clear_costmaps():
        node.clear_costmaps_async()
        return jsonify({"status": "ok", "message": "Costmaps cleared"})

    @app.route("/api/reset", methods=["POST"])
    def trigger_reset():
        node.execute_total_reset_to_origin()
        return jsonify({"status": "ok", "message": "Rover reset to Origin (0.0, -4.50) & Surveillance Started"})

    @app.route("/api/detections")
    def get_detections():
        if os.path.exists(node.json_path):
            try:
                with open(node.json_path, "r") as f:
                    data = json.load(f)
                    if isinstance(data, list):
                        # Reverse list so newest events are first
                        return jsonify(data[::-1])
            except Exception:
                pass
        return jsonify([])

    def generate_mjpeg():
        blank = cv2.imencode(".jpg", cv2.cvtColor(
            cv2.putText(
                cv2.rectangle(
                    cv2.UMat(480, 640, cv2.CV_8UC3).get(),
                    (0, 0), (640, 480), (15, 20, 26), -1
                ),
                "Connecting to Rover Video Stream...", (110, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (148, 163, 184), 2
            ),
            cv2.COLOR_RGB2BGR
        ))[1].tobytes()

        while True:
            frame_bytes = None
            with node.frame_lock:
                if node.latest_jpeg is not None:
                    frame_bytes = node.latest_jpeg
            if frame_bytes is None:
                frame_bytes = blank
                time.sleep(0.1)
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
            )
            time.sleep(0.04)

    @app.route("/video_feed")
    def video_feed():
        return Response(generate_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")

    return app


def main(args=None):
    rclpy.init(args=args)
    node = WebControllerNode()

    flask_app = create_flask_app(node)

    # Run Flask in background daemon thread
    flask_thread = threading.Thread(
        target=lambda: flask_app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False),
        daemon=True
    )
    flask_thread.start()

    node.get_logger().info(
        "\n" + "=" * 70 + "\n"
        "🚀 MISSION CONTROL WEB DASHBOARD ONLINE!\n"
        "   URL: http://localhost:5000\n"
        "   - Dual Mode: Autonomous vs Manual Teleoperation\n"
        "   - Interactive Virtual Joystick + Keyboard WASD\n"
        "   - Emergency Stop & Total System Reset to Origin\n"
        "   - Live Velocity Telemetry & Detection Event Table\n"
        + "=" * 70
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
