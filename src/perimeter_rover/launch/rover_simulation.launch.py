import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_perimeter_rover = get_package_share_directory("perimeter_rover")
    config_file = os.path.join(pkg_perimeter_rover, "config", "camera_bridge.yaml")

    nav2_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "tb3_simulation_launch.py")
        ),
        launch_arguments={"headless": "False", "use_rviz": "False", "autostart": "False"}.items()
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        parameters=[{"config_file": config_file}],
        output="screen"
    )

    return LaunchDescription([
        nav2_sim,
        camera_bridge
    ])
