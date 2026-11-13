#!/usr/bin/env python3
"""
Autonomous Perimeter Rover - Dynamic Obstacle & Secondary Rover Node (Part 3)
Spawns and animates a secondary autonomous security rover or dynamic obstacle
in the outdoor security yard, testing real-time detection and autonomous avoidance.
"""

import math
import os
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_pb2 import Entity
except ImportError:
    from gz.transport import Node as GzNode
    from gz.msgs.entity_factory_pb2 import EntityFactory
    from gz.msgs.pose_pb2 import Pose
    from gz.msgs.boolean_pb2 import Boolean
    from gz.msgs.entity_pb2 import Entity


class DynamicObstacleNode(Node):
    def __init__(self):
        super().__init__("dynamic_obstacle_node")

        self.declare_parameter("model_name", "secondary_rover")
        self.declare_parameter("corridor", "south")
        self.declare_parameter("speed", 0.25)          # Snappy speed (0.25 m/s)
        self.declare_parameter("amplitude", 1.1)       # Range x in [1.1, 3.3] (directly ahead of spawn!)
        self.declare_parameter("center_x", 2.2)
        self.declare_parameter("center_y", -4.15)      # South corridor lane

        self.model_name = str(self.get_parameter("model_name").value)
        self.corridor = str(self.get_parameter("corridor").value).strip().lower()
        self.speed = float(self.get_parameter("speed").value)
        self.amplitude = float(self.get_parameter("amplitude").value)
        self.center_x = float(self.get_parameter("center_x").value)
        self.center_y = float(self.get_parameter("center_y").value)

        self.gz_node = GzNode()
        self.start_time = time.time()
        self.last_log_time = 0.0
        self.spawned = False

        # Spawn secondary rover in Gazebo
        self._spawn_secondary_rover()

        # 20 Hz pose update timer
        self.timer = self.create_timer(0.05, self._update_pose)
        self.get_logger().info(
            f"🚗 Dynamic Obstacle Node started: '{self.model_name}' active in North corridor "
            f"(Range: x in [{-self.amplitude:.1f}, {self.amplitude:.1f}], y={self.center_y:.1f}, Speed: {self.speed:.2f} m/s)"
        )

    def _spawn_secondary_rover(self):
        factory = EntityFactory()
        factory.sdf = f"""<sdf version="1.8">
<model name="{self.model_name}">
  <static>true</static>
  <pose>{self.center_x} {self.center_y} 0.18 0 0 0</pose>
  
  <!-- Main Chassis -->
  <link name="chassis">
    <collision name="collision">
      <geometry>
        <box><size>0.65 0.50 0.32</size></box>
      </geometry>
    </collision>
    <visual name="chassis_visual">
      <geometry>
        <box><size>0.65 0.50 0.25</size></box>
      </geometry>
      <material>
        <ambient>1.0 0.55 0.0 1</ambient>
        <diffuse>1.0 0.55 0.0 1</diffuse>
        <specular>0.3 0.3 0.3 1</specular>
      </material>
    </visual>
    
    <!-- Sensor Dome / Warning Beacon -->
    <visual name="beacon_visual">
      <pose>0.0 0.0 0.18 0 0 0</pose>
      <geometry>
        <cylinder><radius>0.10</radius><length>0.12</length></cylinder>
      </geometry>
      <material>
        <ambient>0.1 0.8 1.0 1</ambient>
        <diffuse>0.1 0.8 1.0 1</diffuse>
      </material>
    </visual>

    <!-- Front Bumper Accent -->
    <visual name="bumper_visual">
      <pose>0.33 0.0 -0.05 0 0 0</pose>
      <geometry>
        <box><size>0.06 0.45 0.12</size></box>
      </geometry>
      <material>
        <ambient>0.1 0.1 0.1 1</ambient>
        <diffuse>0.1 0.1 0.1 1</diffuse>
      </material>
    </visual>
  </link>
</model>
</sdf>"""
        try:
            res, rep = self.gz_node.request(
                "/world/default/create", factory, EntityFactory, Boolean, 2000
            )
            if res and rep.data:
                self.spawned = True
                self.get_logger().info(f"✅ Spawned '{self.model_name}' successfully into Gazebo.")
            else:
                self.get_logger().warning(f"Failed to spawn '{self.model_name}' (it may already exist).")
        except Exception as e:
            self.get_logger().error(f"Error spawning model via gz.transport: {e}")

    def _update_pose(self):
        if not self.spawned:
            self._spawn_secondary_rover()

        elapsed = time.time() - self.start_time
        # Constant-speed linear bouncing motion: speed = 0.20 m/s, amplitude = +/- 2.0m
        period = 4.0 * self.amplitude / max(self.speed, 0.05)
        phase = (elapsed % period) / period  # 0.0 to 1.0

        if phase < 0.5:
            # Moving East (+X): from -amplitude to +amplitude
            progress = phase / 0.5
            x = self.center_x - self.amplitude + (2.0 * self.amplitude * progress)
            vx = self.speed
            yaw = 0.0
        else:
            # Moving West (-X): from +amplitude to -amplitude
            progress = (phase - 0.5) / 0.5
            x = self.center_x + self.amplitude - (2.0 * self.amplitude * progress)
            vx = -self.speed
            yaw = math.pi

        pose_msg = Pose()
        pose_msg.name = self.model_name
        pose_msg.position.x = x
        pose_msg.position.y = self.center_y
        pose_msg.position.z = 0.18
        pose_msg.orientation.z = math.sin(yaw / 2.0)
        pose_msg.orientation.w = math.cos(yaw / 2.0)

        try:
            self.gz_node.request("/world/default/set_pose", pose_msg, Pose, Boolean, 200)
        except Exception:
            pass

        now = time.time()
        if now - self.last_log_time >= 3.0:
            self.last_log_time = now
            direction = "East (+X)" if vx >= 0 else "West (-X)"
            self.get_logger().info(
                f"🚗 Secondary Rover cruising at (x={x:.2f}, y={self.center_y:.2f}) | Speed: {abs(vx):.2f} m/s | Heading: {direction}"
            )

    def destroy_node(self):
        # Remove secondary rover from Gazebo simulation on exit
        if self.spawned:
            try:
                ent = Entity()
                ent.name = self.model_name
                ent.type = Entity.MODEL
                self.gz_node.request("/world/default/remove", ent, Entity, Boolean, 1500)
                self.get_logger().info(f"🧹 Removed '{self.model_name}' from Gazebo.")
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleNode()
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
