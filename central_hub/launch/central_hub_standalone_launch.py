import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="central_hub",
                executable="emergency_detection",
                name="central_hub_node",
                emulate_tty=True,
                output="screen",
            ),
        ]
    )
