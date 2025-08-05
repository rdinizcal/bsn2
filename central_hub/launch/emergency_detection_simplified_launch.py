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
    config_monitor = os.path.join(
        get_package_share_directory('system_monitor'),
        'config',
        'monitor_config.yaml'
    )
    config_enactor = os.path.join(
        get_package_share_directory('adaptation'),
        'config',
        'enactor.yaml'
    )
    config_data_access = os.path.join(
        get_package_share_directory('adaptation'),
        'config',
        'data_access.yaml'
    )
    config_reli_engine = os.path.join(
        get_package_share_directory('adaptation'),
        'config',
        'reli_engine.yaml'
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
                package="central_hub",
                executable="emergency_detection",
                name="central_hub_node",
                emulate_tty=True,
                output="screen",
            ),
            launch_ros.actions.Node(
                package='system_monitor',
                executable='system_monitor',
                name='logger',
                output='screen',
                emulate_tty=True,
                parameters=[config_monitor]
            ),
            launch_ros.actions.Node(
                package='system_monitor',
                executable='system_monitor',
                name='node_monitor',
                output='screen',
                emulate_tty=True,
                parameters=[config_monitor]
            ),
            launch_ros.actions.Node(
                package='system_monitor',
                executable='param_adapter',
                name='param_adapter',
                output='screen',
                emulate_tty=True,
                parameters=[config_monitor]
            ),
            launch_ros.actions.Node(
                package='adaptation',
                executable='enactor',
                name='enactor',
                output='screen',
                emulate_tty=True,
                #parameters=[config_enactor]
            ),
            launch_ros.actions.Node(
                package='adaptation',
                executable='data_access',
                name='data_access',
                output='screen',
                emulate_tty=True,
                #parameters=[config_data_access] if os.path.exists(config_data_access) else [],
            ),
            launch_ros.actions.Node(
                package='adaptation',
                executable='reli_engine',
                name='reli_engine',
                output='screen',
                emulate_tty=True,
                arguments=[
                '--ros-args',
                '--log-level', 'DEBUG'
                ],
                parameters=[config_reli_engine],
            ),
        ]
    )
