import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_perimeter_rover = get_package_share_directory("perimeter_rover")

    # 1. Base Outdoor Security Yard Simulation & Nav2 Bringup (Static Yard with human_intruder at 3.8, 0.5)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perimeter_rover, "launch", "outdoor_yard_simulation.launch.py")
        )
    )

    # 2. Surveillance Node (Static Protocol: avoidance_mode:=false -> Halt & Hold on all intrusions)
    surveillance_node = Node(
        package="perimeter_rover",
        executable="surveillance_node",
        name="surveillance_rover",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "arena_type": "yard",
            "avoidance_mode": False
        }]
    )

    # 3. Patrol Navigator Node (Delayed 5s to allow Nav2 lifecycle bringup)
    patrol_node = Node(
        package="perimeter_rover",
        executable="patrol_node",
        name="patrol_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "arena_type": "yard",
            "avoidance_mode": False
        }]
    )

    patrol_delayed = TimerAction(
        period=5.0,
        actions=[patrol_node]
    )

    return LaunchDescription([
        sim_launch,
        surveillance_node,
        patrol_delayed
    ])
