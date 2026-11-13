import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    pkg_perimeter_rover = get_package_share_directory("perimeter_rover")

    part_arg = DeclareLaunchArgument(
        "part",
        default_value="3",
        description="Select Project Part: 1 (Benchmark Arena), 2 (Static Intruder Yard), 3 (Dynamic Avoidance Yard)"
    )

    part = LaunchConfiguration("part")

    part1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perimeter_rover, "launch", "part1_benchmark.launch.py")
        ),
        condition=IfCondition(PythonExpression(["'", part, "' == '1'"]))
    )

    part2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perimeter_rover, "launch", "part2_static_yard.launch.py")
        ),
        condition=IfCondition(PythonExpression(["'", part, "' == '2'"]))
    )

    part3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perimeter_rover, "launch", "part3_dynamic_yard.launch.py")
        ),
        condition=IfCondition(PythonExpression(["'", part, "' == '3'"]))
    )

    return LaunchDescription([
        part_arg,
        part1_launch,
        part2_launch,
        part3_launch
    ])
