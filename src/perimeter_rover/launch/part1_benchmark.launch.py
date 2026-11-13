import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_perimeter_rover = get_package_share_directory("perimeter_rover")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    # 1. Benchmark arena simulation with SLAM
    world_path = os.path.join(pkg_perimeter_rover, "worlds", "perimeter_arena.sdf")
    config_file = os.path.join(pkg_perimeter_rover, "config", "camera_bridge.yaml")

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "tb3_simulation_launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "slam": "True",
            "x_pose": "-2.0",
            "y_pose": "-0.5",
            "yaw": "0.0",
            "headless": "False",
            "use_rviz": "True",
            "autostart": "True"
        }.items()
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        parameters=[{"config_file": config_file}],
        output="screen"
    )

    # 2. Surveillance Node for Benchmark Arena
    surveillance_node = Node(
        package="perimeter_rover",
        executable="surveillance_node",
        name="surveillance_rover",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "arena_type": "benchmark",
            "avoidance_mode": False
        }]
    )

    # 3. Patrol Navigator Node for Benchmark Arena
    patrol_node = Node(
        package="perimeter_rover",
        executable="patrol_node",
        name="patrol_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "arena_type": "benchmark",
            "avoidance_mode": False,
            "slam": True
        }]
    )

    patrol_delayed = TimerAction(
        period=5.0,
        actions=[patrol_node]
    )

    return LaunchDescription([
        sim_launch,
        camera_bridge,
        surveillance_node,
        patrol_delayed
    ])
