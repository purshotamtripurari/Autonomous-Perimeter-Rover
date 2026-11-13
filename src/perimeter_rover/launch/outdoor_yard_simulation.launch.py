import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_perimeter_rover = get_package_share_directory("perimeter_rover")

    world_path = os.path.join(pkg_perimeter_rover, "worlds", "outdoor_security_yard.sdf")
    map_path = os.path.join(pkg_perimeter_rover, "maps", "outdoor_yard_map.yaml")
    config_file = os.path.join(pkg_perimeter_rover, "config", "camera_bridge.yaml")
    params_path = os.path.join(pkg_perimeter_rover, "config", "nav2_params_yard.yaml")
    robot_sdf_path = os.path.join(pkg_perimeter_rover, "urdf", "gz_waffle_cam.sdf.xacro")
    models_path = os.path.join(pkg_perimeter_rover, "models")
    fuel_path = os.path.expanduser("~/.gz/fuel/fuel.gazebosim.org/openrobotics/models")

    tb3_model_env = SetEnvironmentVariable(
        name="TURTLEBOT3_MODEL",
        value="waffle"
    )

    gz_resource_env = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=f"{models_path}:{fuel_path}:" + os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    )

    nav2_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "tb3_simulation_launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "map": map_path,
            "params_file": params_path,
            "robot_sdf": robot_sdf_path,
            "x_pose": "0.0",
            "y_pose": "-4.5",
            "yaw": "0.0",
            "headless": "False",
            "use_rviz": "True",
            "autostart": "True",
        }.items()
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        parameters=[{"config_file": config_file}],
        output="screen"
    )

    return LaunchDescription([
        tb3_model_env,
        gz_resource_env,
        nav2_sim,
        camera_bridge
    ])
