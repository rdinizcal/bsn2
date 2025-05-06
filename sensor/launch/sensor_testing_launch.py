import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("sensor"), "config", "thermometer.yaml"
    )
    patient_config = os.path.join(
        get_package_share_directory("patient"), "config", "patient_params.yaml"
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="patient",
                executable="patient",
                name="patient_node",
                emulate_tty=True,
                output="screen",
                parameters=[patient_config],
                # arguments=["--ros-args", "--log-level", "debug"],
            ),
            launch_ros.actions.Node(
                package="sensor",
                executable="sensor",
                name="thermometer_node",
                output="screen",
                emulate_tty=True,
                parameters=[config],
                arguments=["--ros-args", "--log-level", "debug"],
            ),
        ]
    )
