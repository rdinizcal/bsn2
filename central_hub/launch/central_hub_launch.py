import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='patient',
            executable='patient',
            name='patient_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='sensor',
            executable='thermometer',
            name='thermometer_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='central_hub',
            executable='emergency_detection',
            name='central_hub_node',
            output='screen'
        ),
    ])

