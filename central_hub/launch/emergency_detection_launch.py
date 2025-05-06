import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_patient = os.path.join(
        get_package_share_directory("patient"), "config", "patient_params.yaml"
    )

    # Get sensor configuration files
    config_thermometer = os.path.join(
        get_package_share_directory("sensor"), "config", "thermometer.yaml"
    )
    config_oximeter = os.path.join(
        get_package_share_directory("sensor"), "config", "oximeter.yaml"
    )
    config_ecg = os.path.join(
        get_package_share_directory("sensor"), "config", "ecg.yaml"
    )
    config_abps = os.path.join(
        get_package_share_directory("sensor"), "config", "abps.yaml"
    )
    config_abpd = os.path.join(
        get_package_share_directory("sensor"), "config", "abpd.yaml"
    )
    config_glucosemeter = os.path.join(
        get_package_share_directory("sensor"), "config", "glucosemeter.yaml"
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="patient",
                executable="patient",
                name="patient_node",
                emulate_tty=True,
                output="screen",
                parameters=[config_patient],
            ),
            launch_ros.actions.Node(
                package="sensor",
                executable="sensor",
                name="thermometer_node",
                output="screen",
                emulate_tty=True,
                parameters=[config_thermometer],
            ),
            launch_ros.actions.Node(
                package="sensor",
                executable="sensor",
                name="oximeter_node",
                output="screen",
                emulate_tty=True,
                parameters=[config_oximeter],
            ),
            launch_ros.actions.Node(
                package="sensor",
                executable="sensor",
                name="ecg_node",
                output="screen",
                emulate_tty=True,
                parameters=[config_ecg],
            ),
            launch_ros.actions.Node(
                package="sensor",
                executable="sensor",
                name="abps_node",
                output="screen",
                emulate_tty=True,
                parameters=[config_abps],
            ),
            launch_ros.actions.Node(
                package="sensor",
                executable="sensor",
                name="abpd_node",
                output="screen",
                emulate_tty=True,
                parameters=[config_abpd],
            ),
            launch_ros.actions.Node(
                package="sensor",
                executable="sensor",
                name="glucosemeter_node",
                output="screen",
                emulate_tty=True,
                parameters=[config_glucosemeter],
            ),
            launch_ros.actions.Node(
                package="central_hub",
                executable="emergency_detection",
                name="central_hub_node",
                emulate_tty=True,
                output="screen",
            ),
        ]
    )
