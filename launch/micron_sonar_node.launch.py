import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

package_description = "drivers_sonar_tritech"


def generate_launch_description():
    micron_yaml = os.path.join(
        get_package_share_directory(package_description),
        "config",
        "micron_sonar_node_params.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "micron_sonar_config_file",
                default_value=micron_yaml,
                description="Path to config filefor Micron Sonar",
            ),
            Node(
                package="drivers_sonar_tritech",
                executable="micron_sonar_node",
                name="micron_sonar_node",
                output="screen",
                parameters=[LaunchConfiguration("micron_sonar_config_file")],
            ),
        ]
    )
