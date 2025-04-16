import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('central_hub'),
        'config',
        'patient_params.yaml'
    )

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='patient',
            executable='patient',
            name='patient_node',
            emulate_tty=True,
            output='screen',
            parameters=[config]
        ),
        launch_ros.actions.Node(
            package='sensor',
            executable='sensor', 
            name='thermometer_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'sensor': 'thermometer', 'vital_sign': 'temperature', 'frequency': '1.0'}]
        ),
        launch_ros.actions.Node(
            package='sensor',
            executable='sensor',
            name='oximeter_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'sensor': 'oximeter', 'vital_sign': 'oxigenation', 'frequency': '1.0'}]
        ),
        launch_ros.actions.Node(
            package='sensor',
            executable='sensor',
            name='ecg_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'sensor': 'ecg', 'vital_sign': 'heart_rate', 'frequency': '1.0'}]
        ),
        launch_ros.actions.Node(
            package='sensor',
            executable='sensor',
            name='abps_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'sensor': 'abps', 'vital_sign': 'abps', 'frequency': '1.0'}]
        ),
        launch_ros.actions.Node(
            package='sensor',
            executable='sensor',
            name='abpd_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'sensor': 'abpd', 'vital_sign': 'abpd', 'frequency': '1.0'}]
        ),
        launch_ros.actions.Node(
            package='sensor',
            executable='sensor',
            name='glucosemeter_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'sensor': 'glucosemeter', 'vital_sign': 'glucose', 'frequency': '1.0'}]
        ),
        launch_ros.actions.Node(
            package='central_hub',
            executable='emergency_detection',
            name='central_hub_node',
            emulate_tty=True,
            output='screen'
        ),

    ])