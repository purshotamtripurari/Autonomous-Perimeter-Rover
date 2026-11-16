import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_perimeter_rover = get_package_share_directory("perimeter_rover")

    tb3_model_env = SetEnvironmentVariable(
        name="TURTLEBOT3_MODEL",
        value="waffle"
    )

    # 1. Base Outdoor Security Yard Simulation & Nav2 Bringup
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perimeter_rover, "launch", "outdoor_yard_simulation.launch.py")
        )
    )

    # 2. Part 3 Dynamic Obstacle Node (Secondary Autonomous Rover positioned ahead of spawn)
    dynamic_obstacle_node = Node(
        package="perimeter_rover",
        executable="dynamic_obstacle_node",
        name="dynamic_obstacle_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "corridor": "south",
            "center_x": 2.2,
            "center_y": -4.15,
            "speed": 0.25,
            "amplitude": 1.1
        }]
    )

    # 3. Surveillance Node (with Dual-Condition Security & Dynamic Avoidance Mode Enabled)
    surveillance_node = Node(
        package="perimeter_rover",
        executable="surveillance_node",
        name="surveillance_rover",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "arena_type": "yard",
            "avoidance_mode": True
        }]
    )

    # 4. Patrol Navigator Node
    patrol_node = Node(
        package="perimeter_rover",
        executable="patrol_node",
        name="patrol_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "arena_type": "yard",
            "avoidance_mode": True
        }]
    )

    surveillance_delayed = TimerAction(
        period=4.0,
        actions=[surveillance_node]
    )

    dynamic_delayed = TimerAction(
        period=5.0,
        actions=[dynamic_obstacle_node]
    )

    patrol_delayed = TimerAction(
        period=8.0,
        actions=[patrol_node]
    )

    # 5. Mission Control & Teleoperation Web Dashboard (localhost:5000)
    web_controller_node = Node(
        package="perimeter_rover",
        executable="web_controller_node",
        name="web_controller_node",
        output="screen"
    )

    web_delayed = TimerAction(
        period=4.0,
        actions=[web_controller_node]
    )

    return LaunchDescription([
        tb3_model_env,
        sim_launch,
        surveillance_delayed,
        dynamic_delayed,
        patrol_delayed,
        web_delayed
    ])
