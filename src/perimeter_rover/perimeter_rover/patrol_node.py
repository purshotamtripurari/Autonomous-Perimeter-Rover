import math
import time
from datetime import datetime
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ClearEntireCostmap, ManageLifecycleNodes, SetInitialPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from std_msgs.msg import Bool, Float32MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener


class PatrolNode(BasicNavigator):
    def __init__(self):
        super().__init__(node_name="patrol_node")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)

        if not self.has_parameter("arena_type"):
            self.declare_parameter("arena_type", "benchmark")
        arena = str(self.get_parameter("arena_type").value).strip().lower()

        if arena == "yard":
            default_waypoints = [
                3.5, -4.5, 1.57,
                3.5, 4.5, 3.14,
                -3.5, 4.5, -1.57,
                -3.5, -1.5, -1.57,
                -2.0, -4.5, 0.0,
                0.0, -4.5, 0.0,
            ]
            default_x = 0.0
            default_y = -4.5
            default_yaw = 0.0
        else:
            default_waypoints = [
                1.5, -0.5, 1.57,
                1.5, 1.5, 3.14,
                -1.5, 1.5, -1.57,
                -2.0, -0.5, 0.0,
            ]
            default_x = -2.0
            default_y = -0.5
            default_yaw = 0.0

        if not self.has_parameter("waypoints"):
            self.declare_parameter("waypoints", default_waypoints)
        if not self.has_parameter("global_frame"):
            self.declare_parameter("global_frame", "map")
        if not self.has_parameter("robot_base_frame"):
            self.declare_parameter("robot_base_frame", "base_footprint")
        if not self.has_parameter("slam"):
            self.declare_parameter("slam", False)
        if not self.has_parameter("initial_x"):
            self.declare_parameter("initial_x", default_x)
        if not self.has_parameter("initial_y"):
            self.declare_parameter("initial_y", default_y)
        if not self.has_parameter("initial_yaw"):
            self.declare_parameter("initial_yaw", default_yaw)

        self.get_logger().info(
            f"Patrol initialized for arena mode: '{arena.upper()}' | Spawn: ({default_x}, {default_y}, yaw={default_yaw})"
        )

        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_base_frame = str(self.get_parameter("robot_base_frame").value)
        self.waypoints = self._build_waypoints(self.get_parameter("waypoints").value)
        self.current_wp_index = 0
        self.intrusion_detected = False
        self.override_until = 0.0
        self.task_active = False
        self.dynamic_obstacle_active = False
        self.dynamic_obs_x = 0.0
        self.dynamic_obs_y = 0.0
        self.dynamic_obs_speed = 0.0
        self.dynamic_obs_dist = 99.0
        self.last_dynamic_obs_time = 0.0
        self.last_avoidance_maneuver_time = 0.0

        # Mode and state flags for re-entrant safe callback execution
        self.control_mode = "autonomous"
        self.emergency_stop_active = False
        self.pending_mode = None
        self.pending_cancel = False
        self.reset_to_origin_requested = False

        if not self.has_parameter("avoidance_mode"):
            self.declare_parameter("avoidance_mode", True)
        self.avoidance_mode = bool(self.get_parameter("avoidance_mode").value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Dedicated publisher and clients for robust Nav2 activation
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.set_pose_client = self.create_client(SetInitialPose, "/set_initial_pose")
        self.nav_lifecycle_client = self.create_client(
            ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes"
        )
        self.loc_lifecycle_client = self.create_client(
            ManageLifecycleNodes, "/lifecycle_manager_localization/manage_nodes"
        )
        self.waypoint_state_client = self.create_client(
            GetState, "/waypoint_follower/get_state"
        )

        # Costmap clearing clients (created in BasicNavigator or fallback)
        self.clear_costmap_local_srv = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap"
        )
        self.clear_costmap_global_srv = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap"
        )

        # Alert, resume, dynamic obstacle, and mode subscriptions
        self.create_subscription(Bool, "/perimeter/alert", self.intrusion_callback, 10)
        self.create_subscription(Bool, "/perimeter/resume", self.resume_callback, 10)
        self.create_subscription(Bool, "/perimeter/dynamic_obstacle", self.dynamic_obstacle_callback, 10)
        self.create_subscription(Float32MultiArray, "/perimeter/dynamic_obstacle_data", self.dynamic_obstacle_data_callback, 10)
        self.create_subscription(String, "/perimeter/control_mode", self.control_mode_callback, 10)
        self.create_subscription(Bool, "/perimeter/emergency_stop", self.emergency_stop_callback, 10)
        self.create_subscription(Bool, "/perimeter/reset_to_origin", self.reset_to_origin_callback, 10)

    def _build_waypoints(self, values):
        if len(values) < 12 or len(values) % 3 != 0:
            raise ValueError("waypoints must contain at least four x, y, yaw triples")

        poses = []
        for index in range(0, len(values), 3):
            x, y, yaw = (float(value) for value in values[index:index + 3])
            if not all(math.isfinite(value) for value in (x, y, yaw)):
                raise ValueError("waypoint coordinates and yaws must be finite")

            pose = PoseStamped()
            pose.header.frame_id = self.global_frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(pose)
        return poses

    def _set_initial_pose(self):
        # Allow time for simulation clock to synchronize and check if already localized
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)

        pose = self._current_pose()
        spawn_x = float(self.get_parameter("initial_x").value)
        spawn_y = float(self.get_parameter("initial_y").value)
        spawn_yaw = float(self.get_parameter("initial_yaw").value)

        # If already localized away from spawn, retain existing AMCL filter (prevents filter corruption)
        if pose is not None:
            rx, ry, ryaw = pose
            dist_from_spawn = math.hypot(rx - spawn_x, ry - spawn_y)
            if dist_from_spawn > 1.2:
                self.get_logger().info(
                    f"Robot already localized at position (x={rx:.2f}, y={ry:.2f}, yaw={ryaw:.2f} rad); retaining AMCL pose."
                )
                return

        x = spawn_x
        y = spawn_y
        yaw = spawn_yaw
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("initial_pose coordinates and yaw must be finite")

        pose_cov = PoseWithCovarianceStamped()
        pose_cov.header.frame_id = self.global_frame
        pose_cov.header.stamp = self.get_clock().now().to_msg()
        pose_cov.pose.pose.position.x = x
        pose_cov.pose.pose.position.y = y
        pose_cov.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_cov.pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose_cov.pose.covariance[0] = 0.25
        pose_cov.pose.covariance[7] = 0.25
        pose_cov.pose.covariance[35] = 0.0685

        self.get_logger().info(f"Publishing Initial Pose to AMCL at x={x:.2f}, y={y:.2f}...")

        # Also publish via BasicNavigator interface
        p_init = PoseStamped()
        p_init.header = pose_cov.header
        p_init.pose = pose_cov.pose.pose
        self.setInitialPose(p_init)

        # Broadcast initial pose across several cycles to guarantee AMCL reception
        for _ in range(5):
            pose_cov.header.stamp = self.get_clock().now().to_msg()
            self.pose_cov_pub.publish(pose_cov)
            self._setInitialPose()
            rclpy.spin_once(self, timeout_sec=0.2)

        # Try direct service if AMCL provides it
        if self.set_pose_client.wait_for_service(timeout_sec=0.5):
            req = SetInitialPose.Request()
            req.pose = pose_cov
            self.set_pose_client.call_async(req)

    def _current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_base_frame, Time()
            ).transform
        except TransformException:
            return None

        rotation = transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return transform.translation.x, transform.translation.y, yaw

    def _clear_costmaps_async(self):
        """Asynchronously clear local and global costmaps without blocking."""
        try:
            if self.clear_costmap_local_srv.service_is_ready():
                self.clear_costmap_local_srv.call_async(ClearEntireCostmap.Request())
            if self.clear_costmap_global_srv.service_is_ready():
                self.clear_costmap_global_srv.call_async(ClearEntireCostmap.Request())
        except Exception:
            pass

    def _safe_cancel_task(self):
        """Safely cancel current task without throwing or blocking executor."""
        if self.task_active:
            try:
                if self.goal_handle is not None:
                    self.cancelTask()
            except Exception as e:
                try:
                    if self.goal_handle is not None:
                        self.goal_handle.cancel_goal_async()
                except Exception:
                    pass
            finally:
                self.task_active = False

    def resume_callback(self, message):
        if message.data:
            now_str = datetime.now().strftime("%H:%M:%S")
            self.override_until = time.time() + 35.0
            self.intrusion_detected = False
            self._clear_costmaps_async()

            pose = self._current_pose()
            if pose is not None:
                rx, ry, _ = pose
                cur_target = self.waypoints[self.current_wp_index]
                tx = cur_target.pose.position.x
                ty = cur_target.pose.position.y
                dist = math.hypot(tx - rx, ty - ry)
                if dist < 0.85:
                    self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)
                    self.get_logger().info(
                        f"Rover within {dist:.2f}m of target; advancing to Checkpoint WP{self.current_wp_index + 1}."
                    )
                else:
                    fwd_idx = self._get_next_forward_waypoint_index()
                    if fwd_idx != self.current_wp_index:
                        self.current_wp_index = fwd_idx
                        self.get_logger().info(
                            f"Forward progression updated target to Checkpoint WP{self.current_wp_index + 1}."
                        )

            target_wp = self.current_wp_index + 1
            self.get_logger().info(
                f"[{now_str}] 🔓 Operator RESUME received! Resuming patrol towards WP{target_wp} (35s bypass grace period enabled)."
            )

    def control_mode_callback(self, msg: String):
        """Asynchronously queue mode changes without blocking or throwing in callbacks."""
        mode = msg.data.strip().lower()
        if mode in ["autonomous", "manual"]:
            self.pending_mode = mode
            if mode == "manual":
                self.pending_cancel = True

    def emergency_stop_callback(self, msg: Bool):
        """Queue emergency stop state without blocking."""
        if msg.data != self.emergency_stop_active:
            self.emergency_stop_active = msg.data
            if msg.data:
                self.pending_cancel = True

    def reset_to_origin_callback(self, msg: Bool):
        """Handle reset to origin signal from mission control web dashboard."""
        if msg.data:
            self.reset_to_origin_requested = True
            self.pending_cancel = True

    def intrusion_callback(self, message):
        if message.data:
            if time.time() < self.override_until or self.control_mode == "manual":
                return
            if not self.intrusion_detected:
                self.intrusion_detected = True
                self.pending_cancel = True
                now_str = datetime.now().strftime("%H:%M:%S")
                date_str = datetime.now().strftime("%Y-%m-%d")
                pose = self._current_pose()
                if pose is None:
                    self.get_logger().error(
                        f"🚨 [ROVER HALTED] Intrusion detected at {now_str} ({date_str})! Holding position."
                    )
                else:
                    x, y, yaw = pose
                    self.get_logger().error(
                        f"🚨 [ROVER HALTED] Intrusion detected at {now_str} ({date_str})! "
                        f"Rover stopped at position (x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad)."
                    )
        elif not message.data and self.intrusion_detected:
            self.intrusion_detected = False
            now_str = datetime.now().strftime("%H:%M:%S")
            self.get_logger().info(f"[{now_str}] 🟢 Perimeter clear; resuming waypoint patrol.")

    def dynamic_obstacle_callback(self, msg: Bool):
        if msg.data and not self.dynamic_obstacle_active:
            self.dynamic_obstacle_active = True
            now_str = datetime.now().strftime("%H:%M:%S")
            self.get_logger().info(
                f"[{now_str}] 🛡️ [ACTIVE AVOIDANCE] Secondary Rover detected! Autonomous evasive yield & pass maneuver standing by."
            )
        elif not msg.data and self.dynamic_obstacle_active:
            self.dynamic_obstacle_active = False

    def dynamic_obstacle_data_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 5 and msg.data[4] > 0.5:
            self.dynamic_obstacle_active = True
            self.dynamic_obs_x = float(msg.data[0])
            self.dynamic_obs_y = float(msg.data[1])
            self.dynamic_obs_speed = float(msg.data[2])
            self.dynamic_obs_dist = float(msg.data[3])
            self.last_dynamic_obs_time = time.time()
        else:
            if time.time() - self.last_dynamic_obs_time > 2.0:
                self.dynamic_obstacle_active = False

    def _execute_yield_and_pass_maneuver(self, resume_target_pose):
        """
        Executes active dynamic avoidance maneuver in current corridor:
        - In South Corridor (y < 0): Moves aside to shoulder (y = -5.05m), yields, returns to y = -4.50m.
        - In North Corridor (y > 0): Moves aside to shoulder (y = 3.85m), yields, returns to y = 4.50m.
        """
        self.override_until = time.time() + 12.0
        pose = self._current_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        is_south_corridor = (ry < 0.0)
        y_shoulder = -5.05 if is_south_corridor else 3.85
        y_path = -4.50 if is_south_corridor else 4.50
        dx_fwd = 0.35 if is_south_corridor else -0.35
        yaw_align = 0.0 if is_south_corridor else math.pi

        now_str = datetime.now().strftime("%H:%M:%S")
        corridor_name = "South" if is_south_corridor else "North"
        self.get_logger().info(
            f"[{now_str}] 🛡️ [DYNAMIC AVOIDANCE] Secondary Rover approaching ({self.dynamic_obs_dist:.2f}m) in {corridor_name} corridor! Moving aside to shoulder (y={y_shoulder:.2f}m)..."
        )
        self._safe_cancel_task()

        # Phase 1: Move Aside to Shoulder
        t_start = time.time()
        while rclpy.ok() and (time.time() - t_start < 4.0):
            rclpy.spin_once(self, timeout_sec=0.04)
            pose = self._current_pose()
            if pose is None:
                continue
            rx, ry, ryaw = pose
            if is_south_corridor and ry <= -4.95:
                break
            if not is_south_corridor and ry <= 3.92:
                break

            target_angle = math.atan2(y_shoulder - ry, dx_fwd)
            angle_err = math.atan2(math.sin(target_angle - ryaw), math.cos(target_angle - ryaw))
            twist = Twist()
            twist.linear.x = 0.16
            twist.angular.z = max(min(1.8 * angle_err, 0.7), -0.7)
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.04)

        # Phase 2: Halt & Yield on Shoulder while Secondary Rover Passes
        self.cmd_vel_pub.publish(Twist())
        now_str = datetime.now().strftime("%H:%M:%S")
        self.get_logger().info(
            f"[{now_str}] 🛡️ [YIELDING] Rover moved aside to shoulder (y={y_shoulder:.2f}m). Yielding right-of-way while Secondary Rover passes..."
        )
        yield_start = time.time()
        while rclpy.ok() and (time.time() - yield_start < 5.0):
            rclpy.spin_once(self, timeout_sec=0.08)
            self.cmd_vel_pub.publish(Twist())
            pose = self._current_pose()
            rx = pose[0] if pose else 0.0

            if time.time() - yield_start > 1.2:
                passed = False
                if is_south_corridor:
                    passed = (self.dynamic_obs_x < rx - 0.2) or (self.dynamic_obs_dist > 2.2)
                else:
                    passed = (self.dynamic_obs_x > rx + 0.2) or (self.dynamic_obs_dist > 2.2)

                if passed or not self.dynamic_obstacle_active:
                    break
            time.sleep(0.08)

        # Phase 3: Return to Original Path
        now_str = datetime.now().strftime("%H:%M:%S")
        self.get_logger().info(
            f"[{now_str}] 🟢 [RETURNING TO PATH] Secondary rover passed! Steering back into original path (y={y_path:.2f}m)..."
        )
        t_return = time.time()
        while rclpy.ok() and (time.time() - t_return < 4.0):
            rclpy.spin_once(self, timeout_sec=0.04)
            pose = self._current_pose()
            if pose is None:
                continue
            rx, ry, ryaw = pose
            if is_south_corridor and ry >= -4.58:
                break
            if not is_south_corridor and ry >= 4.42:
                break

            target_angle = math.atan2(y_path - ry, dx_fwd)
            angle_err = math.atan2(math.sin(target_angle - ryaw), math.cos(target_angle - ryaw))
            twist = Twist()
            twist.linear.x = 0.18
            twist.angular.z = max(min(1.8 * angle_err, 0.7), -0.7)
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.04)

        # Align heading with corridor direction
        t_align = time.time()
        while rclpy.ok() and (time.time() - t_align < 2.0):
            rclpy.spin_once(self, timeout_sec=0.04)
            pose = self._current_pose()
            if pose is None:
                break
            rx, ry, ryaw = pose
            yaw_err = math.atan2(math.sin(yaw_align - ryaw), math.cos(yaw_align - ryaw))
            if abs(yaw_err) < 0.15:
                break
            twist = Twist()
            twist.linear.x = 0.08
            twist.angular.z = max(min(1.5 * yaw_err, 0.6), -0.6)
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.04)

        self.cmd_vel_pub.publish(Twist())
        now_str = datetime.now().strftime("%H:%M:%S")
        self.get_logger().info(f"[{now_str}] ✅ [CONTINUING PATH] Rover back in original path! Resuming Nav2 waypoint patrol.")
        self.goToPose(resume_target_pose)
        self.task_active = True

    def waitUntilNav2Active(self, navigator="bt_navigator", localizer="amcl"):
        """Verify navigation system readiness and ensure lifecycle nodes are in active state."""
        self.get_logger().info("Verifying Nav2 lifecycle activation...")

        if self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Nav2 NavigateToPose action server is ACTIVE and ready!")
            return

        for attempt in range(15):
            if self.nav_to_pose_client.wait_for_server(timeout_sec=0.5):
                self.get_logger().info("Nav2 NavigateToPose action server is ACTIVE and ready!")
                return

            if self.waypoint_state_client.wait_for_service(timeout_sec=0.3):
                future = self.waypoint_state_client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
                if future.result() and future.result().current_state.label == "active":
                    self.get_logger().info("Nav2 bt_navigator & lifecycle nodes are ACTIVE and ready!")
                    return

            if self.nav_lifecycle_client.wait_for_service(timeout_sec=0.3):
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request.STARTUP if attempt < 4 else ManageLifecycleNodes.Request.RESUME
                future_mgr = self.nav_lifecycle_client.call_async(req)
                rclpy.spin_until_future_complete(self, future_mgr, timeout_sec=0.5)

            if self.loc_lifecycle_client.wait_for_service(timeout_sec=0.3):
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request.STARTUP if attempt < 4 else ManageLifecycleNodes.Request.RESUME
                future_loc = self.loc_lifecycle_client.call_async(req)
                rclpy.spin_until_future_complete(self, future_loc, timeout_sec=0.5)

            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("Nav2 NavigateToPose action server is ACTIVE and ready!")
            return

        super().waitUntilNav2Active(navigator=navigator, localizer=localizer)

    def _get_next_forward_waypoint_index(self):
        """
        Dynamically calculates the best forward checkpoint from current rover pose.
        Handles all perimeter corridors as well as the interior yard.
        """
        pose = self._current_pose()
        if pose is None:
            return self.current_wp_index
        rx, ry, _ = pose
        arena = str(self.get_parameter("arena_type").value).strip().lower()

        if arena != "yard":
            if rx > 0.0 and ry < 0.5:
                return 0
            elif rx > 0.0 and ry >= 0.5:
                return 1
            elif rx <= 0.0 and ry >= 0.5:
                return 2
            else:
                return 3

        # Yard Arena Perimeter Progression (Counter-Clockwise):
        # WP1: (3.5, -4.5)  idx 0
        # WP2: (3.5,  4.5)  idx 1
        # WP3: (-3.5, 4.5)  idx 2
        # WP4: (-3.5, -1.5) idx 3
        # WP5: (-2.0, -4.5) idx 4
        # WP6: (0.0, -4.5)  idx 5
        idx = 0
        if ry <= -3.0 and rx < 2.8:
            idx = 0  # In South corridor, moving East towards WP1
        elif rx >= 2.0 and ry < 3.8:
            idx = 1  # In East corridor, moving North towards WP2
        elif ry >= 3.0 and rx > -2.8:
            idx = 2  # In North corridor, moving West towards WP3
        elif rx <= -2.0 and ry > -1.0:
            idx = 3  # In upper West corridor, moving South towards WP4
        elif rx <= -1.5 and -3.5 < ry <= -1.0:
            idx = 4  # In lower West corridor, moving South towards WP5
        elif ry <= -3.0 and rx < 0.0:
            idx = 5  # In South entrance, moving towards WP6
        else:
            # Fallback for Central Yard: Angle from origin determines forward perimeter target
            theta = math.atan2(ry, rx)
            if -math.pi / 2.0 <= theta < 0.0:
                idx = 0 if rx < 2.5 else 1
            elif 0.0 <= theta < math.pi / 2.0:
                idx = 1 if ry < 3.5 else 2
            elif math.pi / 2.0 <= theta <= math.pi:
                idx = 2 if rx > -2.5 else 3
            else:
                idx = 3 if ry > -2.0 else 4

        # If already within 0.70m of chosen waypoint, advance to next
        cur_target = self.waypoints[idx]
        if math.hypot(cur_target.pose.position.x - rx, cur_target.pose.position.y - ry) < 0.70:
            idx = (idx + 1) % len(self.waypoints)

        return idx

    def patrol(self):
        if bool(self.get_parameter("slam").value):
            self.waitUntilNav2Active(localizer="slam_toolbox")
        else:
            self._set_initial_pose()
            self.waitUntilNav2Active()

        self.get_logger().info(
            f"Patrolling {len(self.waypoints)} waypoints sequentially in {self.global_frame}"
        )

        self.current_wp_index = self._get_next_forward_waypoint_index()

        while rclpy.ok():
            # Process pending task cancellations cleanly from main thread
            if self.pending_cancel:
                self.pending_cancel = False
                self._safe_cancel_task()
                self.cmd_vel_pub.publish(Twist())

            # Process Reset to Origin request
            if self.reset_to_origin_requested:
                self.reset_to_origin_requested = False
                self._safe_cancel_task()
                self.emergency_stop_active = False
                self.control_mode = "autonomous"
                self.intrusion_detected = False
                self.override_until = time.time() + 15.0
                self.current_wp_index = 0
                self._clear_costmaps_async()
                self.get_logger().info("🔄 Reset to Origin processed! Starting autonomous surveillance from WP1...")

            # Process pending control mode transitions
            if self.pending_mode is not None:
                new_mode = self.pending_mode
                self.pending_mode = None
                if new_mode == "manual":
                    self.control_mode = "manual"
                    self._safe_cancel_task()
                    self.cmd_vel_pub.publish(Twist())
                    self.get_logger().info("🕹️ Control mode switched to MANUAL. Autonomous patrol paused; joystick active.")
                elif new_mode == "autonomous":
                    self.control_mode = "autonomous"
                    self.intrusion_detected = False
                    self.override_until = time.time() + 15.0  # 15s grace buffer to clear local view
                    self._clear_costmaps_async()
                    self.current_wp_index = self._get_next_forward_waypoint_index()
                    target_num = self.current_wp_index + 1
                    self.get_logger().info(
                        f"🤖 Control mode switched to AUTONOMOUS. Resuming patrol towards Checkpoint WP{target_num}!"
                    )

            # Hold halted if Emergency Stop is engaged
            if self.emergency_stop_active:
                stop_twist = Twist()
                while self.emergency_stop_active and not self.reset_to_origin_requested and rclpy.ok():
                    self.cmd_vel_pub.publish(stop_twist)
                    rclpy.spin_once(self, timeout_sec=0.1)
                continue

            # Pause patrol if Manual Mode is engaged
            if self.control_mode == "manual":
                while self.control_mode == "manual" and not self.emergency_stop_active and not self.reset_to_origin_requested and self.pending_mode is None and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.1)
                continue

            # Hold position if static intrusion is detected
            if self.intrusion_detected:
                stop_twist = Twist()
                while self.intrusion_detected and not self.emergency_stop_active and self.control_mode == "autonomous" and not self.reset_to_origin_requested and self.pending_mode is None and rclpy.ok():
                    self.cmd_vel_pub.publish(stop_twist)
                    rclpy.spin_once(self, timeout_sec=0.1)
                continue

            # Pre-navigation proximity check
            pose = self._current_pose()
            if pose is not None:
                rx, ry, _ = pose
                cur_target = self.waypoints[self.current_wp_index]
                cur_tx = cur_target.pose.position.x
                cur_ty = cur_target.pose.position.y
                if math.hypot(cur_tx - rx, cur_ty - ry) < 0.65:
                    self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)
                    self.get_logger().info(
                        f"Rover already near checkpoint (within 0.65m); advancing to WP{self.current_wp_index + 1}."
                    )

            target_pose = self.waypoints[self.current_wp_index]
            wp_num = self.current_wp_index + 1
            tx = target_pose.pose.position.x
            ty = target_pose.pose.position.y

            self.get_logger().info(
                f"Dispatching rover to Checkpoint WP{wp_num} (x={tx:.2f}, y={ty:.2f})..."
            )

            if not self.goToPose(target_pose):
                self.get_logger().warning(f"Goal to WP{wp_num} rejected; clearing costmaps & re-verifying Nav2...")
                self._clear_costmaps_async()
                self.waitUntilNav2Active()
                time.sleep(1.0)
                rclpy.spin_once(self, timeout_sec=0.2)
                continue

            self.task_active = True
            nav_start_time = time.time()
            while self.task_active and rclpy.ok():
                # Emergency Stop check: immediately abort and halt
                if self.emergency_stop_active or self.pending_cancel:
                    self._safe_cancel_task()
                    self.cmd_vel_pub.publish(Twist())
                    break

                # Manual Mode check: pause patrol and yield to operator teleop
                if self.control_mode == "manual" or self.pending_mode == "manual":
                    self._safe_cancel_task()
                    self.cmd_vel_pub.publish(Twist())
                    break

                # Reset to origin check
                if self.reset_to_origin_requested:
                    self._safe_cancel_task()
                    self.cmd_vel_pub.publish(Twist())
                    break

                # Intrusion alert: Halt rover immediately and cancel Nav2 task
                if self.intrusion_detected:
                    self._safe_cancel_task()
                    self.cmd_vel_pub.publish(Twist())
                    self.get_logger().warn(
                        f"Patrol halted near WP{wp_num} due to intrusion. Holding position."
                    )
                    break

                # Dynamic Avoidance maneuver: When secondary rover approaches in North or South corridor
                if (
                    self.avoidance_mode
                    and self.dynamic_obstacle_active
                    and (time.time() - self.last_avoidance_maneuver_time > 10.0)
                ):
                    pose = self._current_pose()
                    if pose is not None:
                        rx, ry, ryaw = pose
                        in_corridor = (ry >= 3.8 and -3.5 <= rx <= 3.5) or (ry <= -3.8 and -1.0 <= rx <= 3.8)
                        if in_corridor and self.dynamic_obs_dist < 3.2:
                            self.last_avoidance_maneuver_time = time.time()
                            self._execute_yield_and_pass_maneuver(target_pose)

                # In-flight proximity arrival: if within 0.55m of target waypoint, consider achieved
                pose = self._current_pose()
                if pose is not None:
                    rx, ry, _ = pose
                    if math.hypot(tx - rx, ty - ry) < 0.55:
                        self._safe_cancel_task()
                        self.get_logger().info(f"✅ Reached Checkpoint WP{wp_num} (proximity threshold)!")
                        self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)
                        next_num = self.current_wp_index + 1
                        self.get_logger().info(f"Advancing patrol route to WP{next_num}...")
                        time.sleep(0.3)
                        break

                # Waypoint achieved check
                if self.isTaskComplete():
                    self.task_active = False
                    result = self.getResult()
                    if result == TaskResult.SUCCEEDED:
                        self.get_logger().info(f"✅ Reached Checkpoint WP{wp_num} (Nav2 SUCCEEDED)!")
                        self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)
                        next_num = self.current_wp_index + 1
                        self.get_logger().info(f"Advancing patrol route to WP{next_num}...")
                        time.sleep(0.3)
                        break
                    elif result == TaskResult.CANCELED:
                        self.get_logger().info(f"Task to WP{wp_num} was canceled.")
                        break
                    else:
                        self.get_logger().warning(
                            f"⚠️ Task to WP{wp_num} reported status {result}. Clearing costmaps and retrying..."
                        )
                        self._clear_costmaps_async()
                        time.sleep(1.0)
                        break

                # Navigation timeout watchdog: If task running > 45s, advance
                if time.time() - nav_start_time > 45.0:
                    self.get_logger().warning(f"Goal to WP{wp_num} timed out after 45s; advancing to next checkpoint...")
                    self._safe_cancel_task()
                    self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)
                    break

                rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    navigator = PatrolNode()
    try:
        navigator.patrol()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        if rclpy.ok():
            navigator.get_logger().info(f"Patrol node exiting: {e}")
    finally:
        navigator._safe_cancel_task()
        try:
            navigator.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
