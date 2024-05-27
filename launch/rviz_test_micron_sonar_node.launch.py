import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    config_rviz2 = os.path.join(
        get_package_share_directory("drivers_sonar_tritech"), "rviz", "micron_test.rviz"
    )

    map_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_transform",
        output="screen",
        arguments="--x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id sonar_frame".split(
            " "
        ),
    )

    # Rviz2 node
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_tritech_conf",
        output="screen",
        arguments=[["-d"], [config_rviz2]],
    )

    ld = LaunchDescription()
    ld.add_action(map_transform_node)
    ld.add_action(rviz2_node)

    return ld
