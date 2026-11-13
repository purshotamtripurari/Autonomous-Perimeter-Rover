import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    tb3_model = "waffle"
    tb3_desc_share = get_package_share_directory("turtlebot3_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    tb3_env = SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value=tb3_model)

    # 1. Start Gazebo Sim with an empty or default world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r -v 2 empty.sdf"}.items(),
    )

    # 2. Robot State Publisher using turtlebot3_description
    urdf_path = os.path.join(tb3_desc_share, "urdf", f"turtlebot3_{tb3_model}.urdf")
    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_desc,
            "use_sim_time": True
        }],
    )

    # 3. Explicitly spawn TurtleBot3 Waffle in Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "waffle",
            "-string", robot_desc,
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1",
        ],
        output="screen",
    )

    # 4. Sensor and Control Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        output="screen",
    )

    # 5. Surveillance node (YOLO)
    surveillance = Node(
        package="perimeter_rover",
        executable="surveillance_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # 6. Patrol node
    patrol = Node(
        package="perimeter_rover",
        executable="patrol_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        tb3_env,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        surveillance,
        patrol,
    ])
